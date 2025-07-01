#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//-----------------------------------------------------------------------------OLED
// 定义OLED屏幕的宽度和高度
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
// 定义D1和D2引脚对应的GPIO编号
#define SCL_PIN 4
#define SDA_PIN 5
// 定义OLED的I2C地址
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//-----------------------------------------------------------------------------INA226
#define READ_ADDR                  0x81	 //A0=GND，A1=GND // R_addr=1, W_addr=0
#define WRITE_ADDR                 0x80

#define Config_Reg                 0x00  //配置寄存器  读写
#define Shunt_V_Reg                0x01  //分流电压    读
#define Bus_V_Reg                  0x02  //总线电压    读
#define Power_Reg                  0x03  //电源功率    读
#define Current_Reg                0x04  //电流        读
#define Calib_Reg                  0x05  //校准，设定满量程范围以及电流和功率测数的 
#define Mask_En_Reg                0x06  //屏蔽 使能 警报配置和转换准备就绪
#define Alert_Reg                  0x07  //包含与所选警报功能相比较的限定值
#define Man_ID_Reg                 0xFE  //0x5449
#define ID_Reg                     0xFF  //0x2260
//------------------------------------------------------------------INA226数据读取及定义
// 分流电阻为0.01R
#define SHUNT_RESISTANCE 0.01
#define CURRENT_LSB 0.02
#define CALIBRATION_VALUE (0.00512 / (CURRENT_LSB * SHUNT_RESISTANCE))

// 读取2字节数据
uint16_t INA226_Read2Byte(uint8_t reg_addr) {
  Wire.beginTransmission(WRITE_ADDR >> 1);
  Wire.write(reg_addr);
  if (Wire.endTransmission() != 0) {
    return 0;
  }
  delay(1); // 添加延时
  Wire.requestFrom(READ_ADDR >> 1, 2);
  if (Wire.available() < 2) {
    return 0;
  }
  uint16_t reg_data = Wire.read() << 8;
  reg_data |= Wire.read();
  return reg_data;
}

// 写2字节数据
uint8_t INA226_Write2Byte(uint8_t reg_addr, uint16_t reg_data) {
  uint8_t data_high = (uint8_t)((reg_data & 0xFF00) >> 8);
  uint8_t data_low = (uint8_t)reg_data & 0x00FF;
  Wire.beginTransmission(WRITE_ADDR >> 1);
  Wire.write(reg_addr);
  Wire.write(data_high);
  Wire.write(data_low);
  if (Wire.endTransmission() != 0) {
    return 0;
  }
  delay(2);
  return 1;
}

// 初始化INA226
void INA226_Init(void) {
  // 写配置寄存器
  INA226_Write2Byte(Config_Reg, 0x4527); 
  // 写校准寄存器
  INA226_Write2Byte(Calib_Reg, (uint16_t)CALIBRATION_VALUE); 
}

// 获取总线电压mV
float INA226_GetVoltage(void) {
  uint16_t bus_voltage_data = INA226_Read2Byte(Bus_V_Reg);
  float Bus_V = bus_voltage_data * 1.25 * 0.001; //总线电压LSB固定1.25mV
  Serial.print("data=");
  Serial.print(bus_voltage_data);
  Serial.print(", Bus_V  =");
  Serial.print(Bus_V);
  Serial.println(" mV");
  return Bus_V;
}

// 分流电压mV
float INA226_GetShuntVoltage(void) {
  uint16_t shunt_voltage_data = INA226_Read2Byte(Shunt_V_Reg);
  float Shunt_V = shunt_voltage_data * 2.5 * 0.001; //分流电压LSB固定2.5uV
  Serial.print("data=");
  Serial.print(shunt_voltage_data);
  Serial.print(", Shunt_V=");
  Serial.print(Shunt_V);
  Serial.println(" mV");
  return Shunt_V;
}

// 获取分流电流mA
float INA226_GetShunt_Current(void) {
  uint16_t current_data = INA226_Read2Byte(Current_Reg);
  float Curent = current_data * 0.02; //分流电流LSB选择0.02mA
  Serial.print("data=");
  Serial.print(current_data);
  Serial.print(", Curent =");
  Serial.print(Curent);
  Serial.println(" mA");
  return Curent;
}

// 获取功率
float INA226_Get_Power(void) {
  uint16_t power_data = INA226_Read2Byte(Power_Reg);
  float Power = power_data * 0.02 * 25; //功率LSB固定分流电流LSB的25倍
  Serial.print("data=");
  Serial.print(power_data);
  Serial.print(", Power=");
  Serial.print(Power);
  Serial.println(" mW");
  return Power;
}

void setup() {
  // 初始化串口通信
  Serial.begin(115200);
  // 初始化I2C总线，指定SCL和SDA引脚
  Wire.begin(SCL_PIN, SDA_PIN);
  // 尝试初始化OLED屏幕
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // 进入无限循环，停止程序运行
  }
  // 初始化INA226
  INA226_Init();
}
//-------------------------------------------------------------------数据打印
void loop() {
  // 获取功率
  float power = INA226_Get_Power();
  // 获取电压
  float voltage = INA226_GetVoltage();
  // 获取电流
  float current = INA226_GetShunt_Current() / 10; // 转换为A

  // 打印调试信息
  Serial.print("Power: ");
  Serial.print(power);
  Serial.print(" mW, Voltage: ");
  Serial.print(voltage);
  Serial.print(" mV, Current: ");
  Serial.print(current);
  Serial.println(" A");

  // 清空屏幕缓冲区
  display.clearDisplay();
  // 设置字体大小
  display.setTextSize(1);
  // 设置字体颜色为白色
  display.setTextColor(SSD1306_WHITE);
  // 设置光标位置为屏幕左上角
  display.setCursor(0, 0);
  display.println("VOLTAGE (V)");
  display.println(voltage); // 转换为V
  display.println("CURRENT (A)");
  display.println(current);
  display.println("POWER (W)");
  display.println(power / 10.0); // 转换为W
  // 将缓冲区内容显示到屏幕上
  display.display();
  delay(1000);
}    
