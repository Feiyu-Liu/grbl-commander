
#ifndef GCODECTRL_H
#define GCODECTRL_H

#include "GCodeSender.h"
#include "Arc.h"
#include <Arduino.h>
#include <WString.h>
#include <EEPROM.h>

////CONFIG//////
#define ROCKER_SPEED 10000  //摇杆控制电机默认运动速度
#define STEPPERS_SPEED 10000  //电机运行默认速度
#define SENDING_DELAY 0 //指令发送等待时间（毫秒）
#define STRAIGHT_TO_ARC_DELAY 0 //直线转弧形运动等待时间（秒）

#define DYNAMIC_SENSOR_SENSITIVITY 500 //探测灵敏度
#define SHAKENESS_DELAY_TIME 300 //抖动消除等待时间（ms）
#define TRIAL_TIME 7 //每次录音时间（s）
#define DYNAMIC_TRIGGER_PIN 22 //动态试验窗口音频trigger引脚
#define DYNAMIC_TRIGGER_ON 1 //是否开启窗口开始旋转的音频trigger,1=on,0=off
#define MOTION_CAPTURE_TRIGGER 23


//引脚配置
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

class GCodeCtrl {
  public:
    GCodeCtrl(float Slide[2], float TeleL[2], float TeleS[2]); //构造函数
    ~GCodeCtrl();//析构函数
    
    void debug();

    //Serial初始化（需要在主程序的set up里调用）
    void serialBegin(long bridgeBaud, long grblBaud);

    //Jogging相关方法
    void rockerController();  //摇杆控制电机,block程序运行
    void changeRockerCtrlSpeed(int Speed);  //改变摇杆控制电机速度
    
    //核心方法
    void steppersCalibration(float compensate[4][2]); //电机校准
    void runStraightToTarget(float WinCenter[2], float WinSize[2]);//直线运动
    void runStraightToTarget(float WinCenter[2], float WinSize[2], bool Direction, float Angle);
    void runArcToTarget(float WinCenter[2], float WinSize[2], bool Direction, float Angle);//圆弧运动(中心点为圆心)
    void runArcToTarget(float WinCenter[2], float WinSize[2], int vertexNum, bool Direction, float Angle);//圆弧运动(中心点为自定义顶点)
    /* Vertex Number Define
      1----------2
      |          |
      4----------3
    */

    void automaticArrival(float WindowsParameter[3][2]);//*路径规划，全自动到达目标点

    //配置
    void changeSteppersSpeed(float speed);  //改变电机运行速度

    //工具
    void eepromRead();//从eeprom中读取断电前数据

    //模式
    void freeControl();  //电脑设置窗口参数并运行
    void dynamicMode();  //动态试验模式（窗口绕中心点旋转）
    void dynamicMode2();  //动态试验模式2（窗口平移）
    void dynamicMode3();  //动态试验模式3（窗口绕顶点旋转）
    void Debug();
    GCodeSender *MySender = new GCodeSender;

  private:
    Arc* _MyArc;
    long _rockerControllerSpeed;  //摇杆电机运动速度
    float _steppersSpeed;  //电机运行默认速度

    //(已初始化)储存步进电机参数：(0)Slide((0)长度、(1)步长、(2)1cm步长),TeleL,TeleS
    float _stepsParameter[3][3]; 
    //（需刷新）储存当前顶点坐标（世界坐标）
    float _currentCoord[4][2];
    //（需刷新）储存当前窗口参数。第一层：窗口中心，第二层：窗口长宽，第三层:窗口方向、角度
    float _currentWinData[3][2] = {0};  

    //Tools
    //厘米转化为毫米
    float _unitConversionToMm(float cmLong); 
    //计算所有源坐标：输入矩形中心点和长宽，输出所有源坐标的首地址
    float *_calculateEveryCoord(float WinCenter[2], float WinSize[2]);
    //坐标系重建：世界坐标系转换为I、II型输出坐标系
    float *_coordinateRebuild(float oriAllPos[4][2]);
    //冒泡排序
    void _bubbleSort(float numArr[], int n);
    //EEPROM处理
    void _eepromClean();
    void _eepromWrite();
    void _eepromRead();
    //用于数组返回值
    float _calculateEveryCoordReturn[8];
    float _coordinateRebuildReturn[8];
    
};


#endif
