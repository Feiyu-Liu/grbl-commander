#include <SoftwareSerial.h>

/*
版本号：v1.0
作者：刘飞宇
日期：20230331
库简介：多功能串口发送器
更新日志：
V1.0:可以正常使用3个发送器函数

*/
//debug
//#define serialDebug
//#define voidDebug

#ifndef GCODESENDER_H
#define GCODESENDER_H

#include <Arduino.h>
#include <WString.h>
#include "config.h"
#include <SoftwareSerial.h>


class GCodeSender {
  public:
    GCodeSender(); 
    //带参：软串口引脚
    //GCodeSender (int RXpin, int TXpin); 
    ~GCodeSender();
    void serialBegin();

    ///////Tools
    //发送信息至全体Grbl;q/Q=quit
    void computerCtrl_Universal();
    //发送信息至某个Grbl
    void computerCtrl_Specific();
    //写入所有命令后，同时发送
    void computerCtrl_Batch();
    
    #ifdef voidDebug
    void Debug();
    #endif
    
    //serial实例化 SOFTWARE_RX_PIN_2
    SoftwareSerial *bridgeSerial = new SoftwareSerial(SOFTWARE_RX_PIN, SOFTWARE_TX_PIN);
    //SoftwareSerial *triggerSerial = new SoftwareSerial(SOFTWARE_RX_PIN_2, SOFTWARE_TX_PIN_2);

    HardwareSerial *serialPort[4] = {&Serial, &Serial1, &Serial2, &Serial3};
    //结构声明，储存Grbls的相关参数
    struct Grbls {
      String currentCmd = "";
      String lastCmd = "";
    };
    //储存所有Grbl的指针
    Grbls *grblsArray[4] = {new Grbls, new Grbls, new Grbls, new Grbls};  //1，2，3，4

    //立即发送储存在currentCmd中的数据,并判断是否有主机返回error
    bool sendNow();  //error（false）输出上一指令
    bool sendNowWithoutBack(); //仅发送指令，并返回真值
    bool sendNowAutoRetry(); //error（false）重试3次，每条指令发送1次

    //核心函数
    bool sendStraightCmd(float allPos[4][2], float straightSpeed);  //输入：输出坐标系
    bool sendArcCmd(float allArcPos[4][2], float winCenterPos[4][2], float winOriPos[4][2], bool direction, float arcSpeed);  //绕中心点旋转
    bool sendAnchorArcCmd(float allArcPos[4][2], float winCenterPos[4][2], float winOriPos[4][2], bool direction, float arcSpeed[4]);  //绕顶点旋转

    void stopFeeding();

    //电机暂停指令(时间单位：秒)
    void delayCmd(float delayTime);

    //睡眠模式
    void grblSleep();

    //grbl是否ok
    bool grblOKState(int grblIndex);  //检查grbl是否发送ok
    bool waitGrblOK(int grblIndex, String cmd, long timeOut);  //等待grbl发送ok,接收超时返回false
  
  private:
    
    //Tools：
    //检查各串口是否有效连接    
    void _establishContact();
    //清除缓存区数据
    void _serialClean();
    void _serialClean(int witchPort);
    

};
#endif
