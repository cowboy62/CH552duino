#include <WS2812.h>
#include <Servo.h>
#include <SoftI2C.h>

#define NUM_LEDS 8
#define COLOR_PER_LEDS 3
#define NUM_BYTES (NUM_LEDS * COLOR_PER_LEDS)

#if NUM_BYTES > 255
#error "NUM_BYTES 不能大於 255."
#endif

__xdata uint8_t ledData[NUM_BYTES];

// 引腳定義
const int analogInPin   = 11;  // 麥克風輸入
const int analogInPinB  = 14;  // 光敏電阻輸入
const int SCLPin        = 30;  // I2C SCL 引腳
const int SDAPin        = 31;  // I2C SDA 引腳
const int WS2812Pin     = 15;  // WS2812 引腳
const int RCServoPin    = 34;  // 遙控伺服引腳
const int trigPin       = 16;  // 超聲波感測器 Trig 引腳
const int echoPin       = 17;  // 超聲波感測器 Echo 引腳

int sensorValue = 0;        // 從電位器讀取的值
int sensorValueB = 0;       // 從電位器 B 讀取的值
long duration, cm;          // 計算距離所需的變數
int cnt = 0 ;

void setup() {
    // 初始化各個組件
    initSerial();          // 初始化序列通訊
    initAnalogPins();      // 初始化類比引腳
    initWS2812();          // 初始化 WS2812 LED
    initServo();           // 初始化伺服馬達
    initI2C();             // 初始化 I2C
    initUltrasonic();      // 初始化超聲波感測器
}

void loop() {
    USBSerial_println("");
    USBSerial_print(cnt ++ );
    USBSerial_println("   Srart");
    // 讀取傳感器值
    readAnalogSensors();
    
    // 控制 WS2812 LED
    controlWS2812();
    
    // 移動伺服馬達
    moveServo();
    
    // 掃描 I2C 設備
    scanI2CDevices();
    
    // 讀取超聲波感測器距離
    readUltrasonicDistance();
}

void initSerial() {
    USBSerial_println("Welcome CH552!!!");  // 歡迎信息
}

void initAnalogPins() {
    pinMode(analogInPin, INPUT);   // 設定類比輸入引腳
    pinMode(analogInPinB, INPUT);  // 設定類比輸入 B 引腳
    USBSerial_println("analog Input OK !!!"); // 顯示狀態
}

void initWS2812() {
    pinMode(WS2812Pin, OUTPUT); // 設定 WS2812 引腳為輸出
    USBSerial_println("WS2812 OK !!!"); // 顯示狀態
}

void initServo() {
    Servo_init();                  // 初始化伺服
    pinMode(RCServoPin, OUTPUT);   // 設定伺服引腳為輸出
    Servo_attach(RCServoPin);      // 附加伺服到引腳
    USBSerial_println("RCServo OK !!!"); // 顯示狀態
}

void initI2C() {
    Wire_begin(SCLPin, SDAPin);    // 初始化 I2C
    USBSerial_println("I2C OK !!!"); // 顯示狀態
}

void initUltrasonic() {
    pinMode(trigPin, OUTPUT);       // 設定 Trig 引腳為輸出
    pinMode(echoPin, INPUT);        // 設定 Echo 引腳為輸入
    USBSerial_println("HC-SR04 OK !!!"); // 顯示狀態
}

void readAnalogSensors() {
    sensorValue = analogRead(analogInPin);  // 讀取電位器 A 的值
    sensorValueB = analogRead(analogInPinB); // 讀取電位器 B 的值
    
    // 打印傳感器值
    USBSerial_print("sensorA = ");
    USBSerial_print(sensorValue);
    USBSerial_print("    sensorB = ");
    USBSerial_println(sensorValueB);
    
    // 等待模擬到數位轉換器穩定
    delay(200);
}

void controlWS2812() {
    // 控制 LED 顏色
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
        set_pixel_for_GRB_LED(ledData, i, 128, 0, 0); // 設定紅色
        neopixel_show_P1_5(ledData, NUM_BYTES); // 顯示 LED
        delay(50);
    }
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
        set_pixel_for_GRB_LED(ledData, i, 0, 128, 0); // 設定綠色
        neopixel_show_P1_5(ledData, NUM_BYTES);
        delay(50);
    }
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
        set_pixel_for_GRB_LED(ledData, i, 0, 0, 128); // 設定藍色
        neopixel_show_P1_5(ledData, NUM_BYTES);
        delay(50);
    }
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
        set_pixel_for_GRB_LED(ledData, i, 0, 0, 0); // 設定藍色
        neopixel_show_P1_5(ledData, NUM_BYTES);
        delay(50);
    }
}

void moveServo() {
    // 控制伺服馬達的角度
    Servo_write(RCServoPin, 0);   // 轉到 0 度
    delay(1000);
    Servo_write(RCServoPin, 90);  // 轉到 90 度
    delay(1000);
    Servo_write(RCServoPin, 180); // 轉到 180 度
}

void scanI2CDevices() {
    USBSerial_println("I2C scanner:");
    for (uint8_t i = 0; i < 128; i++) {
        delay(1);
        if (Wire_scan(i)) {
            USBSerial_print("I2C address: 0x");
            USBSerial_println(i, HEX); // 打印找到的 I2C 設備地址
        }
    }
}

void readUltrasonicDistance() {
    digitalWrite(trigPin, LOW); // 將 Trig 引腳設為低電位
    delayMicroseconds(5);       // 等待 5 微秒
    digitalWrite(trigPin, HIGH); // 將 Trig 引腳設為高電位
    delayMicroseconds(10);      // 等待 10 微秒
    digitalWrite(trigPin, LOW); // 將 Trig 引腳設為低電位
    
    duration = pulseIn(echoPin, HIGH, 30000); // 讀取 Echo 引腳的脈衝時間
    cm = (duration / 2) / 29.1; // 計算距離
    USBSerial_print("Ultrasonic : ");   // 單位為公分
    USBSerial_print(cm);        // 在序列監控器顯示距離
    USBSerial_println(" cm");   // 單位為公分
}
