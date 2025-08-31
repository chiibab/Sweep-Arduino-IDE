#include <M5StickCPlus2.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <M5Unified.h>

// 若需串口调试输出，取消注释
#define DEBUG_SERIAL

// 若需屏显，取消注释
#define ENABLE_DISPLAY

//IPアドレスみる
// const char* SSID      = "ちひろ";
// const char* PASSWORD  = "chiibabmi";
// const char* UDP_ADDR  = "172.20.10.2";

// const char* SSID      = "Buffalo-G-3000-WPA3";
// const char* PASSWORD  = "iag7vyxgngkri";
// const char* UDP_ADDR  = "192.168.11.10";

// const char* SSID      = "free-wifi";
// const char* PASSWORD  = "free-wifi";
// const char* UDP_ADDR  = "10.124.58.24";

const uint16_t UDP_PORT = 8887;
// const char* SSID      = "HR01a-FEE321";
// const char* PASSWORD  = "a4dc520d52";
const char* UDP_ADDR = "192.168.1.7";

const char* SSID      = "_VelopSetup23D";
const char* PASSWORD  = "df91frf9bx";


// 背光控制管脚（由 GPIO27 驱动 TFT VLED） :contentReference[oaicite:0]{index=0} :contentReference[oaicite:1]{index=1}
const uint8_t BACKLIGHT_PIN = 27;
bool backlightOn = true;  // 默认启动背光

WiFiUDP udp;

void setup() {//電源→一回だけ動く→あとは下のコードでloop
  #ifdef DEBUG_SERIAL
  Serial.begin(115200);
  Serial.println("Setup start");
  #endif

  // 初始化 M5StickC Plus 2
  M5.begin();
  M5.Lcd.setRotation(3);
  M5.Lcd.setTextSize(2);

  // 上电立即打开背光，保持显示 :contentReference[oaicite:2]{index=2}
  pinMode(BACKLIGHT_PIN, OUTPUT);
  digitalWrite(BACKLIGHT_PIN, HIGH);//この辺は見なくていい
  #ifdef DEBUG_SERIAL
  Serial.println("Backlight ON (default)");
  #endif

  pinMode(G36, INPUT);
  //pinMode(G36,INPUT_PULLDOWN);

  // WiFi 连接
  WiFi.begin(SSID, PASSWORD);
  #ifdef DEBUG_SERIAL
  Serial.print("Connecting to WiFi: "); Serial.println(SSID);// ここら辺はwifi接続していて、...だと接続できていない
  #endif
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    #ifdef DEBUG_SERIAL
    Serial.print(".");
    #endif
  }
  #ifdef DEBUG_SERIAL
  Serial.println("\nWiFi connected");
  Serial.print("IP address: "); Serial.println(WiFi.localIP());
  #endif

  #ifdef ENABLE_DISPLAY
  // 首次屏显
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.println("IMU Ready");
  #endif
}

void loop() {//IMUデータ見込み→UDP発送
  M5.update();  // 更新按键状态 :contentReference[oaicite:4]{index=4}



  // 中键 (BtnA)：切换背光与屏幕刷新
  if (M5.BtnA.wasPressed()) {
    backlightOn = !backlightOn;
    digitalWrite(BACKLIGHT_PIN, backlightOn ? HIGH : LOW);
    #ifdef DEBUG_SERIAL
    Serial.printf("Backlight %s\n", backlightOn ? "ON" : "OFF");
    #endif
  }
  // 侧键 (BtnB)：重启 :contentReference[oaicite:5]{index=5}
  if (M5.BtnB.wasPressed()) {
    #ifdef DEBUG_SERIAL
    Serial.println("Button B pressed: restarting");
    #endif
    esp_restart();
  }

  // データの読み取り
  float ax, ay, az;                        // 单位：g
  M5.Imu.getAccel(&ax, &ay, &az);          // Plus2 内置 MPU6886:contentReference[oaicite:0]{index=0}
  int pressureRaw = analogRead(G36); //G36で取った電圧出す
  int pressure = 4095- pressureRaw;
  //int pressure = analogRead(G36);

#ifdef DEBUG_SERIAL
Serial.printf("Accel: x=%.2f y=%.2f z=%.2f\n", ax, ay, az);
Serial.printf("Pressure: %d\n", pressure);
Serial.printf("battery: %d\n", M5.Power.getBatteryLevel());
#endif

#ifdef ENABLE_DISPLAY
  if (backlightOn) {
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.printf("Ax: %.2fg\nAy: %.2fg\nAz: %.2fg\n", ax, ay, az);
    M5.Lcd.printf("pressure: %d\n", pressure);
    M5.Lcd.printf("battery : %d\n", M5.Power.getBatteryLevel());
  }
#endif

  // JSON（データ形式）→UDP（時間を重視した通信）→Unity発送される
  // String json = String("{\"ax\":") + String(ax, 2) +
  //             ",\"ay\":" + String(ay, 2) +
  //             ",\"az\":" + String(az, 2) + "konnnitiha}";
  //             // ",\"pressure\":" + String(pressure,1) + "}";

  //               char fullJson[128];
  //               snprintf(fullJson, sizeof(fullJson), "%s,\"pressure\":%d}", json.c_str(), pressure);
char jsonBuffer[128]; // JSON文字列を格納するためのバッファ
  snprintf(jsonBuffer, sizeof(jsonBuffer),
           "{\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"pressure\":%d,\"battery\":%d}",
           ax, ay, az, pressure, M5.Power.getBatteryLevel());

  //UDPで送信
  udp.beginPacket(UDP_ADDR, UDP_PORT);
  udp.print(jsonBuffer);
  udp.endPacket();//ここで終わってloopする

  delay(100);// Unityに0.1秒に一回送信
  //まとめこれがUnityに送られる
}