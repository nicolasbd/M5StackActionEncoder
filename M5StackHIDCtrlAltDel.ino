#include <M5Stack.h>

/*
 * this code is based on https://github.com/nkolban/esp32-snippets/blob/9112aebed4ef86cfccccfdbf3aedf8fe44ec08e4/cpp_utils/tests/BLETests/SampleHIDKeyboard.cpp
 */

#include "BLEDevice.h"
#include "BLEServer.h"
#include "BLEUtils.h"
#include "BLE2902.h"
#include "BLEHIDDevice.h"
#include "HIDKeyboardTypes.h"
#include <esp_log.h>
#include <string>
#include <vector>
#include "Task.h"
#include "sdkconfig.h"

#ifdef ARDUINO_ARCH_ESP32
#include "esp32-hal-log.h"
#endif

// Encoder Vars
#define Faces_Encoder_I2C_ADDR     0X5E
int encoder_increment;//positive: clockwise nagtive: anti-clockwise
int encoder_value=0;
int last_encoder_value;
int direction;//-1: clockwise 1: anti-clockwise
uint8_t last_button, cur_button;

boolean isModeSelector = false;

typedef struct {
  char title[30];
  KEYMAP maps[2];
  char buttons[2][10];
  KEYMAP encoderMoves[2];
  KEYMAP encoderAction[1];
} CONFIG;


CONFIG configurations[] = {
  CONFIG {
    "IDEA : RUN DEBUG",
    { KEYMAP {0x3e42, KEY_SHIFT}, KEYMAP {0x41, KEY_CTRL} },
    { "RE-RUN", "NEXT" },
    { KEYMAP {0x51}, KEYMAP {0x52} },
    KEYMAP {0x41, KEY_CTRL}
  },
  CONFIG {
    "TWITTER",
    { KEYMAP {0x17}, KEYMAP {0x0f} },
    { "RT", "LIKE" },
    { KEYMAP {0x0d}, KEYMAP {0x0e} },
    KEYMAP {0x28}

  }
};

int configIndex = 0;
void GetEncoderValue(void) {
    int temp_encoder_increment;

    Wire.requestFrom(Faces_Encoder_I2C_ADDR, 3);
    if(Wire.available()){
       temp_encoder_increment = Wire.read();
       cur_button = Wire.read();
    }
    if(temp_encoder_increment > 127){//anti-clockwise
        direction = -1;
        encoder_increment = 256 - temp_encoder_increment;
    } else if (temp_encoder_increment != 0) {
        direction = 1;
        encoder_increment = temp_encoder_increment;
    } else {
      direction = 0;
    }
    
}

static char LOG_TAG[] = "SampleHIDDevice";
static BLEHIDDevice* hid;
BLECharacteristic* input;
BLECharacteristic* output;
bool isConnected = false;


extern void DisplayConnectionText();
extern void DisplayStatusText(const char *text);
extern void DisplayGuide();

/*
 * This callback is connect with output report. In keyboard output report report special keys changes, like CAPSLOCK, NUMLOCK
 * We can add digital pins with LED to show status
 * bit 1 - NUM LOCK
 * bit 2 - CAPS LOCK
 * bit 3 - SCROLL LOCK
 */
class MyOutputCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* me){
    uint8_t* value = (uint8_t*)(me->getValue().c_str());
    ESP_LOGI(LOG_TAG, "special keys: %d", *value);
  }
};

class MyTask : public Task {
  private:
  KEYMAP payload[256];
  int length = 0;
  
  public:
  MyTask(const char *text) {
    this->setString(text);
  }
  
  MyTask(const KEYMAP *payload, int length) {
    this->setKeys(payload, length);
  }

  MyTask() {
    this->length = 0;
  }

  void setString(const char *text) {
    this->length = 0;
    const char *pointer = text;
    while(*pointer){
        KEYMAP map = keymap[(uint8_t)*pointer];
        this->payload[this->length++] = map;
        pointer++;
    }
  }
  
  void setKeys(const KEYMAP *payload, int length) {
    int realLen = min(256, length);
    for (int i=0 ; i<realLen ; i++) {
      this->payload[i] = payload[i];
    }
    this->length = realLen;
  }

  void deleteMe() {
    vTaskDelete(NULL);
  }
  
  private:
  void run(void*){
      DisplayStatusText("sending keys.");
      for(int i=0 ; i<this->length ; i++) {
        KEYMAP map = this->payload[i];
        /*
         * simulate keydown, we can send up to 6 keys
         */
        uint8_t a[] = {map.modifier, 0x0, map.usage, 0x0,0x0,0x0,0x0,0x0};
        input->setValue(a,sizeof(a));
        input->notify();

        /*
         * simulate keyup
         */
        uint8_t v[] = {0x0, 0x0, 0x0, 0x0,0x0,0x0,0x0,0x0};
        input->setValue(v, sizeof(v));
        input->notify();

        vTaskDelay(100/portTICK_PERIOD_MS);
      }
      DisplayStatusText("sent keys.");
  }
};

std::vector<MyTask *> tasks;

void resetScreen() {
  M5.Lcd.clear(BLACK);
  DisplayConnectionText();
}

void DisplayConnectionText() {
  int yStart = 0;
  int height = 50;
  M5.Lcd.setCursor(0, yStart);
  const char *text = isConnected ? "Connected" : "Not connected.";
  M5.Lcd.fillRect(0, yStart, 320, height, BLACK);
  M5.Lcd.printf(text);
}

void DisplayStatusText(const char *text) {
  int yStart = 100;
  int height = 50;
  M5.Lcd.setCursor(0, yStart);
  M5.Lcd.fillRect(0, yStart, 320, height, BLACK);
  M5.Lcd.printf(text);
}

void DisplayGuide() {
  
  
  int yStart = 200;

  CONFIG c = configurations[configIndex];
  M5.Lcd.setCursor(0, yStart - 40);
  M5.Lcd.print(c.title);
  if (isModeSelector) {
    M5.Lcd.setCursor(120, yStart);
    M5.Lcd.printf("SEL");
  } 

  M5.Lcd.setCursor(40, yStart);
  M5.Lcd.printf(c.buttons[0]);
  M5.Lcd.setCursor(220, yStart);
  M5.Lcd.printf(c.buttons[1]);
  M5.Lcd.setCursor(120, yStart);

}

class MyCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer){
    BLE2902* desc;
    desc = (BLE2902*) input->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
    desc->setNotifications(true);
    isConnected = true;
    DisplayConnectionText();
  }

  void onDisconnect(BLEServer* pServer){
    isConnected = false;
    DisplayConnectionText();

    for(int i=0 ; i<tasks.size(); i++) {
      MyTask *task = tasks[i];
      task->stop();
      task->deleteMe();
      //delete task;
    }
    tasks.clear();
  }
};

class MainBLEServer: public Task {
   BLEServer *pServer;

public:
   BLEServer *getServer() {
    return pServer;
   }
  
  void run(void *data) {
    ESP_LOGD(LOG_TAG, "Starting BLE work!");

    BLEDevice::init("M5Stack-HID");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyCallbacks());

    /*
     * Instantiate hid device
     */
    hid = new BLEHIDDevice(pServer);


    input = hid->inputReport(1); // <-- input REPORTID from report map
    output = hid->outputReport(1); // <-- output REPORTID from report map

    output->setCallbacks(new MyOutputCallbacks());

    /*
     * Set manufacturer name (OPTIONAL)
     * https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.manufacturer_name_string.xml
     */
    std::string name = "esp-community";
    hid->manufacturer()->setValue(name);

    /*
     * Set pnp parameters (MANDATORY)
     * https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.pnp_id.xml
     */

    hid->pnp(0x02, 0xe502, 0xa111, 0x0210);

    /*
     * Set hid informations (MANDATORY)
     * https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.hid_information.xml
     */
    hid->hidInfo(0x00,0x01);


    /*
     * Keyboard
     */
    const uint8_t reportMap[] = {
      USAGE_PAGE(1),      0x01,       // Generic Desktop Ctrls
      USAGE(1),           0x06,       // Keyboard
      COLLECTION(1),      0x01,       // Application
      REPORT_ID(1),   0x01,   // REPORTID
      USAGE_PAGE(1),      0x07,       //   Kbrd/Keypad
      USAGE_MINIMUM(1),   0xE0,
      USAGE_MAXIMUM(1),   0xE7,
      LOGICAL_MINIMUM(1), 0x00,
      LOGICAL_MAXIMUM(1), 0x01,
      REPORT_SIZE(1),     0x01,       //   1 byte (Modifier)
      REPORT_COUNT(1),    0x08,
      HIDINPUT(1),           0x02,       //   Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position
      REPORT_COUNT(1),    0x01,       //   1 byte (Reserved)
      REPORT_SIZE(1),     0x08,
      HIDINPUT(1),           0x01,       //   Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position
      REPORT_COUNT(1),    0x05,       //   5 bits (Num lock, Caps lock, Scroll lock, Compose, Kana)
      REPORT_SIZE(1),     0x01,
      USAGE_PAGE(1),      0x08,       //   LEDs
      USAGE_MINIMUM(1),   0x01,       //   Num Lock
      USAGE_MAXIMUM(1),   0x05,       //   Kana
      HIDOUTPUT(1),          0x02,       //   Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile
      REPORT_COUNT(1),    0x01,       //   3 bits (Padding)
      REPORT_SIZE(1),     0x03,
      HIDOUTPUT(1),          0x01,       //   Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile
      REPORT_COUNT(1),    0x06,       //   6 bytes (Keys)
      REPORT_SIZE(1),     0x08,
      LOGICAL_MINIMUM(1), 0x00,
      LOGICAL_MAXIMUM(1), 0x65,       //   101 keys
      USAGE_PAGE(1),      0x07,       //   Kbrd/Keypad
      USAGE_MINIMUM(1),   0x00,
      USAGE_MAXIMUM(1),   0x65,
      HIDINPUT(1),           0x00,       //   Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position
      END_COLLECTION(0)
    };
    /*
     * Set report map (here is initialized device driver on client side) (MANDATORY)
     * https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.report_map.xml
     */
    hid->reportMap((uint8_t*)reportMap, sizeof(reportMap));

    /*
     * We are prepared to start hid device services. Before this point we can change all values and/or set parameters we need.
     * Also before we start, if we want to provide battery info, we need to prepare battery service.
     * We can setup characteristics authorization
     */
    hid->startServices();

    BLESecurity *pSecurity = new BLESecurity();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);

    ESP_LOGD(LOG_TAG, "Wait a bit ...");
    delay(5000);

    /*
     * Its good to setup advertising by providing appearance and advertised service. This will let clients find our device by type
     */
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->setAppearance(HID_KEYBOARD);
    pAdvertising->addServiceUUID(hid->hidService()->getUUID());
    pAdvertising->start();

    ESP_LOGD(LOG_TAG, "Advertising started!");
    delay(1000000);
  }
};

MainBLEServer* pMainBleServer = NULL;

void StartBLEServer(void)
{
  esp_log_level_set("*", ESP_LOG_DEBUG);
  pMainBleServer = new MainBLEServer();
  pMainBleServer->setStackSize(20000);
  pMainBleServer->start();

}

void setup() {
  M5.begin();                   // M5STACK INITIALIZE
  Wire.begin();
  esp_log_level_set("*", ESP_LOG_DEBUG);        // set all components to ERROR level
  //esp_log_level_set("wifi", ESP_LOG_WARN);      // enable WARN logs from WiFi stack
  //log_e("HELLO_ERR");

  Serial.begin(115200);         // SERIAL
  dacWrite (25,0);
  
  M5.Lcd.setBrightness(200);    // BRIGHTNESS = MAX 255
  M5.Lcd.fillScreen(BLACK);     // CLEAR SCREEN
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(2);
  //M5.Lcd.setRotation(0);        // SCREEN ROTATION = 0

  DisplayStatusText("Initializing...");
  DisplayConnectionText();
  DisplayGuide();
  
  // put your setup code here, to run once:
  StartBLEServer();
  DisplayStatusText("Initialized.");
}

int nConfigs = (sizeof (configurations) / sizeof (CONFIG));
int lastConfigIndex = configIndex;
void loop() {
    
  if( M5.BtnB.wasReleased()) {
    isModeSelector = !isModeSelector;
    M5.update();
  }

  GetEncoderValue();
  if(isModeSelector) {
    if (direction != 0) {
      if (direction == -1) {
          configIndex -= 1;
      } else if (direction == 1) {
          configIndex += 1;
      }

      if (configIndex < 0) {
        configIndex = nConfigs - 1;
      } else if (configIndex > nConfigs -1) {
        configIndex = 0;
      }

      resetScreen();
      M5.update();
    }


  } else {
    if (isConnected) {
      
      CONFIG conf = configurations[configIndex];
      KEYMAP payload[1];
      if( M5.BtnA.wasReleased()) {
          payload[0] = conf.maps[0];
          MyTask *task2 = new MyTask(payload, 1);
          task2->start();
          tasks.push_back(task2);
          delay(300);
      }

      if (M5.BtnC.wasReleased()) {
        payload[0] = conf.maps[1];
        MyTask *task3 = new MyTask(payload, 1);
        task3->start();
        tasks.push_back(task3);
        delay(300);
      }    
      
      if(last_button != cur_button) {
          
        if (cur_button) {
            payload[0] = conf.encoderAction[0];
            MyTask *taskBtn = new MyTask(payload, 1);
            taskBtn->start();
            tasks.push_back(taskBtn);
        }
        
        last_button = cur_button;
        delay(300);
      }


      if (direction == 1) {
          encoder_value += encoder_increment;
          payload[0] = conf.encoderMoves[0]; 
          MyTask *downTask = new MyTask(payload, 1);
          downTask->start();
          tasks.push_back(downTask);
          delay(250);
      } else if (direction == -1){
          encoder_value -= encoder_increment;
          payload[0] = conf.encoderMoves[1]; 
          MyTask *upTask = new MyTask(payload, 1);
          upTask->start();
          tasks.push_back(upTask);
          delay(250);
      }  
    
    }
      
    
  }

  DisplayGuide();
  M5.update();
  

}
