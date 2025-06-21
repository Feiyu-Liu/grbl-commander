#ifndef CONFIG_H
#define CONFIG_H

/////////////////////////////////
/////////////////////Interact.h/
/////////////////////////////////
//电机运动参数
const float slide[2] = {80, 8532};  //长度（cm）、总步进数
const float teleL[2] = {55, 88737}; 
const float teleS[2] = {35, 56113}; 

//误差补偿值(CM)：不够减，过了加
const float compensate[4][2] = { 
  {62.6, -1}, //(1B,1A)
  {80, 55.5}, //(2A,2B)
  {63, 0}, //(3B, 3A)
  {80, 55.5}  //(4A,4B)
};

//扬声器引脚
#define SPEAKER_PIN 3  
//显示屏引脚
#define SCREEN_SCK 22
#define SCREEN_MOSI 23
#define SCREEN_CS 24
#define SCREEN_RS 25
#define SCREEN_RST 26
#define SCREEN_LED_PIN 27
#define SCREEN_LED_BRIGHTNESS 28

/////////////////////////////////
/////////////////////Arc.h/
/////////////////////////////////

#define CW true //顺时针 = 1
#define CCW false //逆时针 = 0
#define PI 3.1415926

#define DELAY_TIME 0

/////////////////////////////////
/////////////////////GCodeSender.h/
/////////////////////////////////

#define GRBL_BAUD_RATE 115200 //grbl比特率(必须为115200)

//软串口函数调用
/*
  Mega 和 Mega 2560 上并非所有引脚都支持变化中断，因此仅以下引脚可用于 RX：
  10、11、12、13、50、51、52、53、62、63、64、65、66、67、68、69
*/
//连接电脑的软串口引脚
#define SOFTWARE_RX_PIN 50  //RX
#define SOFTWARE_TX_PIN 51  //TX
#define BRIDGE_BAUD_RATE 9600 //比特率(软串口的比特率不宜过高)

//预留的软串口引脚，可用作trigger触发
#define SOFTWARE_RX_PIN_2 52  //RX
#define SOFTWARE_TX_PIN_2 53  //TX
#define TIGGER_BAUD_RATE 9600 //比特率

#define IS_ESTABLISH_CONTACT 1 //是否检查串口的连接情况（若有串口未连接，则会提示并等待连接）
#define SERIAL_TIMEOUT 100 //设置等待串行数据的最大毫秒
#define SENDING_DELAY_TIME 1
#define CONTACT_TIMEOUT 30000 //检查串口连接超时时间（ms）

/////////////////////////////////
/////////////////////GCodeCtrl.h/
/////////////////////////////////
#define ROCKER_SPEED 10000  //摇杆控制电机默认运动速度
#define STEPPERS_SPEED 10000  //电机运行默认速度
#define SENDING_DELAY 0 //指令发送等待时间（毫秒）
#define STRAIGHT_TO_ARC_DELAY 0 //直线转弧形运动等待时间（秒）


//动态实验设置
#define DYNAMIC_TRIGGER_PIN 22 //动态试验窗口音频trigger引脚
#define DYNAMIC_TRIGGER_ON 1 //是否开启窗口开始旋转的音频trigger,1=on,0=off
#define TRIAL_TIME 7 //每次录音时间（s）
#define MOTION_CAPTURE_TRIGGER 23 //动作捕捉trigger

//遥杆引脚配置
#define S_X_PIN A0
#define S_Y_PIN A1
#define BUTTON_0_PIN 45
#define BUTTON_1_PIN 46
#define BUTTON_2_PIN 47
#define BUTTON_3_PIN 48
#define BUTTON_4_PIN 49

//debug
//#define serialDebug
//#define voidDebug
//#define DEBUG1

#endif