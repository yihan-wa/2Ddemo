#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

const float alpha = 0.98; // 加速度计和陀螺仪融合系数
const long gyroInterval = 60; // 减少陀螺仪更新间隔（毫秒）

float angleX = 0; // 初始角度
float angleY = 0;

unsigned long previousJoystickMillis = 0;
const long joystickInterval = 50; // 每 50 毫秒读取一次摇杆数据

unsigned long previousGyroMillis = 0;

// 按钮的引脚
const int buttonPin1 = 3;  
const int buttonPin2 = 4;  
const int buttonPin3 = 5;    
const int xAxisPin = A0; // X轴连接到模拟引脚A0
const int yAxisPin = A1; // Y轴连接到模拟引脚A1
int buttonState = HIGH;      
int lastButtonState1 = HIGH;
int lastButtonState2 = HIGH;
int lastButtonState3 = HIGH;

unsigned long lastDebounceTime1 = 0;  
unsigned long lastDebounceTime2 = 0;  
unsigned long lastDebounceTime3 = 0;    
const unsigned long debounceDelay = 50;  
const unsigned long fireInterval = 160;  
unsigned long lastFireTime = 0;  

bool xstopSent = false; // 添加一个变量来跟踪是否已发送 Xstop 信号
bool jumpSent = false; // 添加一个变量来跟踪是否已发送 Jump 信号
bool defendSent = false; // 标记是否处于防御状态
bool fire1Sent = false;
bool fire2Sent = false;
bool fire3Sent = false;
void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  pinMode(buttonPin1, INPUT);  
  pinMode(buttonPin2, INPUT);  
  pinMode(buttonPin3, INPUT);  
  pinMode(xAxisPin, INPUT);
  pinMode(yAxisPin, INPUT);

  // 初始化角度为0
  angleX = 0;
  angleY = 0;
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousJoystickMillis >= joystickInterval) {
    previousJoystickMillis = currentMillis;
    
    int xValue = analogRead(xAxisPin); // 读取X轴值
    int yValue = analogRead(yAxisPin); // 读取Y轴值
    
    // 处理摇杆数据
    if (yValue < 200) {
      Serial.println("Left");
      xstopSent = false; // 重置状态变量
    } else if (yValue > 800) {
      Serial.println("Right");
      xstopSent = false; // 重置状态变量
    } else if (xValue >= 200 && xValue <= 800) {
      // 只在 xstopSent 为 false 时发送 Xstop 信号
      if (!xstopSent) {
        Serial.println("Xstop");
        xstopSent = true; // 更新状态变量，防止重复发送
      }
    }

    // 处理 Y 轴数据，触发 Jump 信号
    if (xValue > 800) 
    { // 阈值需要根据实际情况调整
      if (!jumpSent) 
      {
        Serial.println("Jump");
        jumpSent = true; // 更新状态变量，防止重复发送
      }
    } 
    else 
    {
      jumpSent = false; // 当 Y 轴值低于阈值时，重置状态变量
    }
    
    if (xValue < 200) 
    {
      Serial.println("defend");
      defendSent = false;
    } 
    else
    {
      if(!defendSent) Serial.println("defendstop");
      defendSent = true;
    }
    
  }

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
  handleButton(buttonPin1, "Fire1", fire1Sent, lastDebounceTime1, lastButtonState1);
  handleButton(buttonPin2, "Fire2", fire2Sent, lastDebounceTime2, lastButtonState2);
  handleButton(buttonPin3, "Fire3", fire3Sent, lastDebounceTime3, lastButtonState3);

}
void handleButton(int buttonPin, const char* fireSignal, bool& fireSent, unsigned long& lastDebounceTime, int& lastButtonState)
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
