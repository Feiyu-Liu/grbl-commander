/*
版本号：V2
作者：刘飞宇
日期：20230224
库简介：计算窗口旋转后的坐标
更新日志：
V1.0:可以正常输出窗口旋转后每个顶点的坐标，与搭配主程序使用可以使所有电机直线运动到目标位置。没有集成圆弧插补类似算法，且没有越界报错功能
V1.1:修正了使用static的严重错误
V2:可以计算绕顶点旋转的目的坐标
*/

#ifndef ARC_H
#define ARC_H
#include <Arduino.h>

//#define serialDEBUG
//#define DEBUG1

#define CW true //顺时针 = 1
#define CCW false //逆时针 = 0
#define PI 3.1415926

#define DELAY_TIME 0

class Arc {
  public:
    Arc(float edgesX, float edgesY, long stepsOfSlide, long stepsOfTeleStepper_L, long stepsOfTeleStepper_S);   //边界x,y; 步长（滑台，长伸缩杆,短伸缩杆）
    Arc(float edgesX, float edgesY);
    ~Arc();
    void DEBUG();
    //////////////////////////
    //返回旋转后的所有目的坐标（基于原始坐标系）
    float *everyTargetCoord(float winCenter[2], float winSize[2], bool direction_w, float angle_w);  //绕矩形的中心点旋转
    float *everyTargetCoord(float winCenter[2], float winSize[2], int vertexNum, bool direction_w, float angle_w);  //绕自定义顶点(1-4号)旋转
    /* Vertex Number Define
      1----------2
      |          |
      4----------3
    */
    float *singleTargetCoord(float startPoint[2], float endPoint[2], bool direction_w, float angle_w);//计算单个坐标点的旋转结果

  private:
    float _edgesX, _edgesY;
    long _stepsOfSlide, _stepsOfTeleStepper_L, _stepsOfTeleStepper_S;

    //TOOLs
    //计算所有源坐标：输入矩形中心点和长宽，输出所有源坐标的首地址
    float *_calculateEveryCoord(float winCenter[2], float winSize[2]);  

    //坐标系重建：输入新的原点（相对于（0，0））和需要改变原点的坐标，输出新坐标的首地址
    float *_coordRebuild(float newCenter[2], float inputCoord[4][2]);  

    //计算目标坐标：输入重建后的坐标、旋转角度、方向，输出计算后的首地址
    float *_calculateTargetCoord(float midCoord[4][2], float ANGLE, bool DIR);  

    ///////用于数组返回值
    float ALLPOS[8];
    float TargetPos[8];
    float MIDPOS[8];
    float SINGLEPOS[2];
    float FINALPOS[8];

};

#endif