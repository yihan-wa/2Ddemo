#include <Wire.h>
#include <MPU6050.h>
#include <U8glib.h>

// 创建 MPU6050 对象
MPU6050 mpu;

U8GLIB_SSD1306_128X32 u8g(U8G_I2C_OPT_NONE); 
// 定义按钮的引脚
//const int buttonPin = 6;  // 按钮连接到 Arduino 的 D10 引脚
const int buttonPin1 = 3;  
const int buttonPin2 = 5;  
const int buttonPin3 = 7;
const int xAxisPin = A0; // X轴连接到模拟引脚A0
const int yAxisPin = A1; // Y轴连接到模拟引脚A1
int buttonState = HIGH;      // 当前按钮状态
int lastButtonState1 = HIGH;
int lastButtonState2 = HIGH;
int lastButtonState3 = HIGH;
int buttonCounter = 0;       // 按钮按下计数器
char buffer[10];

// 用于显示角度的字符串
//char angleBuffer[20];

unsigned long lastDebounceTime1 = 0;  
unsigned long lastDebounceTime2 = 0;  
unsigned long lastDebounceTime3 = 0;
const unsigned long debounceDelay = 50;  // 去抖动时间（毫秒）
const unsigned long fireInterval = 160;  // Fire信号的时间间隔（毫秒）
unsigned long lastFireTime = 0;  // 上次触发 Fire 信号的时间

// 角度计算参数
const float alpha = 0.98; // 加速度计和陀螺仪融合系数
float angleX = 0; // 初始角度
float angleY = 0;
//float angleZ = 0;

unsigned long previousJoystickMillis = 0;
unsigned long previousGyroMillis = 0;
unsigned long previousSerialMillis = 0;
const long gyroInterval = 60; // 陀螺仪更新间隔（毫秒）
const long serialInterval = 100; // 串口发送间隔（毫秒）
const long joystickInterval = 50; // 每 10 毫秒读取一次摇杆数据

bool xStopped = false;
bool yStopped = false;
bool jumpSent = false;
bool defendSent = false;
bool fire1Sent = false;
bool fire2Sent = false;
bool fire3Sent = false;

void draw(void) {
  // 设置字体
  u8g.setFont(u8g_font_6x10);

  // 清空显示
  u8g.firstPage();
  do {
    u8g.drawStr(0, 10, "Button Counter:");
    sprintf(buffer, "%d", buttonCounter);
    u8g.drawStr(80, 10, buffer);
  } while (u8g.nextPage());
}

void setup() {
  // 初始化串口通信
  Serial.begin(9600);
  pinMode(buttonPin1, INPUT_PULLUP);  
  pinMode(buttonPin2, INPUT_PULLUP);  
  pinMode(buttonPin3, INPUT);
  pinMode(xAxisPin, INPUT);
  pinMode(yAxisPin, INPUT);

  u8g.begin();
  // 初始化 MPU6050
  Wire.begin();
  mpu.initialize();
  
  // 检查 MPU6050 是否正常工作
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
  angleX = 0;
  angleY = 180;
}

void loop() {
   unsigned long currentMillis = millis();
   
    if (currentMillis - previousJoystickMillis >= joystickInterval) {
    previousJoystickMillis = currentMillis;

    int xValue = analogRead(xAxisPin); // 读取X轴值
    int yValue = analogRead(yAxisPin); // 读取Y轴值
    
    // 处理摇杆数据
    if (xValue > 800) 
    {
      Serial.println("Down");
      Serial.println("defend");
      xStopped = false; // 动作时清除静止标志
      jumpSent = false;
      defendSent = false;
    } 
    else if (xValue < 200) 
    {
      Serial.println("Up");
      if(!jumpSent)
      {
        Serial.println("Jump");
        jumpSent = true;
      }
      if(!defendSent) Serial.println("defendstop");
      defendSent = true;
      xStopped = false; // 动作时清除静止标志
    }
    else if (400 < xValue && xValue < 624) 
    {
      if (!xStopped) 
      {
        Serial.println("Ystop");
        xStopped = true; // 发送消息后设置静止标志
      }
      if(!defendSent) Serial.println("defendstop");
      defendSent = true;
      jumpSent = false;
    }
    
    if (yValue > 800) 
    {
      Serial.println("Right");
      yStopped = false; // 动作时清除静止标志
    }
    else if (yValue < 200) 
    {
      Serial.println("Left");
      yStopped = false; // 动作时清除静止标志
    }
    else if (400 < yValue && yValue < 624) 
    {
      if (!yStopped) 
      {
        Serial.println("Xstop");
        yStopped = true; // 发送消息后设置静止标志
      }
    }
  }

  // 计算时间间隔
  static unsigned long lastUpdateTime = 0;
  float dt = (currentMillis - lastUpdateTime) / 1000.0; // 转换为秒
  lastUpdateTime = currentMillis;

  if (currentMillis - previousGyroMillis >= gyroInterval) {
    previousGyroMillis = currentMillis;

    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    float accelAngleX = atan2(ay, az) * 180 / PI;
    float accelAngleY = atan2(ax, az) * 180 / PI;
    
    float gyroRateX = gx / 65.5;
    float gyroRateY = gy / 65.5;

    // 获取当前角度而不是逐渐变化
    float dt = gyroInterval / 1000.0; // 转换为秒
    angleX = accelAngleX; // 当前加速度计角度
    angleY = accelAngleY; // 当前加速度计角度

    // 发送角度数据到串口
    Serial.print("Angles:");
    Serial.print(angleX);
    Serial.print(",");
    Serial.println(angleY);
  }

  if (currentMillis - previousSerialMillis >= serialInterval) {
    previousSerialMillis = currentMillis;
    // 输出到串口
    /*Serial.print("Angle X: ");
    Serial.print(angleX);
    Serial.print(" Angle Y: ");
    Serial.print(angleY);
    Serial.print(" Angle Z: ");
    Serial.println(angleZ);*/
  }
  handleButton1(buttonPin3, "Fire1", fire3Sent, lastDebounceTime3, lastButtonState3);
  handleButton2(buttonPin1, "Fire2", fire1Sent, lastDebounceTime1, lastButtonState1);
  handleButton2(buttonPin2, "Fire3", fire2Sent, lastDebounceTime2, lastButtonState2);
}

void handleButton1(int buttonPin, const char* fireSignal, bool& fireSent, unsigned long& lastDebounceTime, int& lastButtonState)
{
  int reading = digitalRead(buttonPin);
  
  // 去抖动处理
  if (reading != lastButtonState) {
    lastDebounceTime = millis(); 
  }

  // 检查去抖动后的按钮状态
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading == HIGH) {
      // 如果按钮被按下且尚未发送信号
      if (!fireSent) { 
        // 发送信号并更新状态
        Serial.println(fireSignal);
        fireSent = true; 
        lastFireTime = millis(); // 记录上次发送时间
      } else {
        // 如果已经发送信号，检查是否达到发送间隔
        if (millis() - lastFireTime >= fireInterval) {
          Serial.println(fireSignal); // 持续发送信号
          lastFireTime = millis(); // 更新上次发送时间
        }
      }
    } else {
      // 按钮松开时重置状态
      fireSent = false; 
    }
  }

  lastButtonState = reading; // 更新最后按钮状态
}

void handleButton2(int buttonPin, const char* fireSignal, bool& fireSent, unsigned long& lastDebounceTime, int& lastButtonState)
{
  int reading = digitalRead(buttonPin);
  
  // 去抖动处理
  if (reading != lastButtonState) {
    lastDebounceTime = millis(); 
  }

  // 检查去抖动后的按钮状态
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading == LOW) {
      // 如果按钮被按下且尚未发送信号
      if (!fireSent) { 
        // 发送信号并更新状态
        Serial.println(fireSignal);
        fireSent = true; 
        lastFireTime = millis(); // 记录上次发送时间
      } else {
        // 如果已经发送信号，检查是否达到发送间隔
        if (millis() - lastFireTime >= fireInterval) {
          Serial.println(fireSignal); // 持续发送信号
          lastFireTime = millis(); // 更新上次发送时间
        }
      }
    } else {
      // 按钮松开时重置状态
      fireSent = false; 
    }
  }

  lastButtonState = reading; // 更新最后按钮状态
}
