
#ifndef GCODECTRL_H
#define GCODECTRL_H

#include <Arduino.h>
#include <WString.h>
#include <EEPROM.h>

#include "GCodeSender.h"
#include "Arc.h"


class GCodeCtrl {
  public:
    GCodeCtrl(float Slide[2], float TeleL[2], float TeleS[2]); //构造函数
    ~GCodeCtrl();//析构函数
    

    //Serial初始化（需要在主程序的set up里调用）
    void serialBegin();

    //Jogging相关方法
    void rockerController();  //摇杆控制电机,block程序运行
    void changeRockerCtrlSpeed(int Speed);  //改变摇杆控制电机速度

    
    //核心方法
    void steppersCalibration(float compensate[4][2]); //电机校准
    void steppersCalibration(); //读取eeprom中的坐标进行校准
    void runStraightToTarget(float WinCenter[2], float WinSize[2]);//直线运动
    void runStraightToTarget(float WinCenter[2], float WinSize[2], bool Direction, float Angle);
    void runArcToTarget(float WinCenter[2], float WinSize[2], bool Direction, float Angle);//圆弧运动(中心点为圆心)
    void runArcToTarget(float WinCenter[2], float WinSize[2], int vertexNum, bool Direction, float Angle);//圆弧运动(中心点为自定义顶点)
    /* Vertex Number Define
      1----------2
      |          |
      4----------3
    */

    void automaticArrival(float WindowsParameter[3][2]);//路径规划，全自动到达目标点

    //配置
    void changeSteppersSpeed();  //改变电机运行速度

    //工具
    void sleep(float sleepPos[4][2]); //关机，断电前运行

    //模式
    void freeControl();  //电脑设置窗口参数并运行
    void dynamicMode();  //动态试验模式（窗口绕中心点旋转）
    void dynamicMode2();  //动态试验模式2（窗口平移）
    void dynamicMode3();  //动态试验模式3（窗口绕顶点旋转）

    #ifdef voidDebug
    void Debug();
    #endif

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
    //设置电机加速度
    void _setAcceleration(float accel); 
    
};


#endif
