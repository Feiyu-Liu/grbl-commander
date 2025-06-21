#include "HardwareSerial.h"
#include "Arc.h"

Arc::Arc(float EDGESX, float EDGESY, long SLIDE_STEPS, long TELESTEPPER_STEPS_L, long TELESTEPPER_STEPS_S){
  this->_edgesX = EDGESX;
  this->_edgesY = EDGESY;
  this->_stepsOfSlide = SLIDE_STEPS;
  this->_stepsOfTeleStepper_L = TELESTEPPER_STEPS_L;
  this->_stepsOfTeleStepper_S = TELESTEPPER_STEPS_S;  
}

Arc::Arc(float EDGESX, float EDGESY){
  this->_edgesX = EDGESX;
  this->_edgesY = EDGESY; 
}

Arc::~Arc(){}

void Arc::DEBUG(){
  float *ptrAllPos;
  float wincenter[2] = {40, 40};
  float winsize[2] = {50, 20};
  float tempAllPos[4][2];
  ptrAllPos = this->everyTargetCoord(wincenter, winsize,1,1,1.1);
  for(int i=0;i<4;i++){
    for(int k=0;k<2;k++){
      tempAllPos[i][k] = *ptrAllPos;
      ptrAllPos++;
    }
  }
  for(int i=0;i<4;i++){
    for(int k=0;k<2;k++){
      Serial.print(tempAllPos[i][k]);
      Serial.print(" ");
    }
    Serial.println("");
  }


}


float *Arc::everyTargetCoord(float winCenter[2], float winSize[2], bool direction_w, float angle_w) { //绕矩形的中心点旋转
  //公有成员，输入窗口中心点、窗口大小、方向、角度
  #ifdef serialDEBUG
  Serial.println(F("++++++CoordCalculate,begin!++++++"));
  Serial.print(F("win center = "));
  Serial.print(winCenter[0]);
  Serial.println(winCenter[1]);
  Serial.print(F("win size = "));
  Serial.print(winSize[0]);
  Serial.println(winSize[1]);
  Serial.print(F("dir = "));
  Serial.println(direction_w);  
  Serial.print(F("angle = "));
  Serial.println(angle_w);
  #endif
  /*根据窗口获取每个点的源坐标*/
  delay(DELAY_TIME);
  float allPos[4][2];  //所有源坐标（8个）
  float *ptrAllPos = this->_calculateEveryCoord(winCenter, winSize);//源坐标首地址指针
  for (int i=0;i<4;i++) {  //储存指针中的数组数据
    for (int k=0;k<2;k++) {
      allPos[i][k] = *ptrAllPos;
      ptrAllPos++;
    }
  }  
  delay(DELAY_TIME);
  #ifdef serialDEBUG
  for(int i=0;i<4;i++){
    for(int k=0;k<2;k++){
      Serial.print(allPos[i][k]);
      Serial.print(" ");
    }
    Serial.println("");
  }
  Serial.println("");
  #endif
  delay(DELAY_TIME);
  /*坐标重建*/
  float midAllPos[4][2];  //重建坐标系后的坐标
  float *ptrMidPos = this->_coordRebuild(winCenter, allPos);
  for (int i=0;i<4;i++) {  
    for (int k=0;k<2;k++) {
      midAllPos[i][k] = *ptrMidPos;
      ptrMidPos++;
    }
  }
  delay(DELAY_TIME);
  #ifdef serialDEBUG
  for(int i=0;i<4;i++){
    for(int k=0;k<2;k++){
      Serial.print(midAllPos[i][k]);
      Serial.print(" ");
    }
    Serial.println("");
  }
  Serial.println("");
  #endif
  delay(DELAY_TIME);
  /*目标点坐标计算*/
  float targetPos[4][2]; //旋转后的坐标点
  float *ptrTargetPos = this->_calculateTargetCoord(midAllPos, angle_w, direction_w);
  for (int i=0;i<4;i++) {  //储存指针中的数组数据
    for (int k=0;k<2;k++) {
      targetPos[i][k] = *ptrTargetPos;
      ptrTargetPos++;
    }
  }
  delay(DELAY_TIME);
  #ifdef serialDEBUG
  for(int i=0;i<4;i++){
    for(int k=0;k<2;k++){
      Serial.print(targetPos[i][k]);
      Serial.print(" ");
    }
    Serial.println("");
  }
  Serial.println("");
  #endif
  /*坐标反重建*/
  float tempWinCenter[2] = {-winCenter[0], winCenter[1]};
  float *ptrFianlPos = this->_coordRebuild(tempWinCenter, targetPos);
  

  return ptrFianlPos;
}

float *Arc::everyTargetCoord(float winCenter[2], float winSize[2], int vertexNum, bool direction_w, float angle_w) { //绕自定义顶点(1-4号)旋转
  /*根据窗口获取每个点的源坐标*/
  delay(DELAY_TIME);
  float allPos[4][2];  //所有源坐标（8个）
  float *ptrAllPos = this->_calculateEveryCoord(winCenter, winSize);//源坐标首地址指针
  for (int i=0;i<4;i++) {  //储存指针中的数组数据
    for (int k=0;k<2;k++) {
      allPos[i][k] = *ptrAllPos;
      ptrAllPos++;
    }
  } 
  #ifdef serialDEBUG
  for(int i=0;i<4;i++){
    for(int k=0;k<2;k++){
      Serial.print(allPos[i][k]);
      Serial.print(" ");
    }
    Serial.println("");
  }
  Serial.println("");
  #endif
  delay(DELAY_TIME);

  /*坐标重建(将自定义旋转顶点定义为原点)*/
  float rebuildCenter[2];
  switch (vertexNum){
    case 1:
      rebuildCenter[0] = allPos[0][0];
      rebuildCenter[1] = allPos[0][1];
      break;
    case 2:
      rebuildCenter[0] = allPos[1][0];
      rebuildCenter[1] = allPos[1][1];
      break;
    case 3:
      rebuildCenter[0] = allPos[2][0];
      rebuildCenter[1] = allPos[2][1];
      break;
    case 4:
      rebuildCenter[0] = allPos[3][0];
      rebuildCenter[1] = allPos[3][1];
      break;
    default:
      rebuildCenter[0] = winCenter[0];
      rebuildCenter[1] = winCenter[1];
      break;
  }
  
  float midAllPos[4][2];  //重建坐标系后的坐标
  float *ptrMidPos = this->_coordRebuild(rebuildCenter, allPos);
  for (int i=0;i<4;i++) {  
    for (int k=0;k<2;k++) {
      midAllPos[i][k] = *ptrMidPos;
      ptrMidPos++;
    }
  }
  #ifdef serialDEBUG
  for(int i=0;i<4;i++){
    for(int k=0;k<2;k++){
      Serial.print(midAllPos[i][k]);
      Serial.print(" ");
    }
    Serial.println("");
  }
  Serial.println("");
  #endif
  delay(DELAY_TIME);
  /*目标点坐标计算*/
  float targetPos[4][2]; //旋转后的坐标点
  float *ptrTargetPos = this->_calculateTargetCoord(midAllPos, angle_w, direction_w);
  for (int i=0;i<4;i++) {  //储存指针中的数组数据
    for (int k=0;k<2;k++) {
      targetPos[i][k] = *ptrTargetPos;
      ptrTargetPos++;
    }
  }
  #ifdef serialDEBUG
  for(int i=0;i<4;i++){
    for(int k=0;k<2;k++){
      Serial.print(targetPos[i][k]);
      Serial.print(" ");
    }
    Serial.println("");
  }
  Serial.println("");
  #endif
  delay(DELAY_TIME);
  
  /*坐标反重建*/
  float tempWinCenter[2] = {-rebuildCenter[0], rebuildCenter[1]};
  float *ptrFianlPos = this->_coordRebuild(tempWinCenter, targetPos);
  for (int m=0; m<8; m++) {
      FINALPOS[m] =  *ptrFianlPos;
      ptrFianlPos++;
  }
  #ifdef serialDEBUG
  for (int i=0;i<8;i++) {
    Serial.println(FINALPOS[i]);
  }
  #endif

  return FINALPOS;
}  

float *Arc::singleTargetCoord(float startPoint[2], float endPoint[2], bool DIR,float ANGLE) {
  //重建
  float midCoord[2];
  midCoord[0] = endPoint[0] - startPoint[0];
  midCoord[1] = startPoint[1] - endPoint[1];
  
  //计算
  float TARGETPOS[2];
  float A, r, x1, y1, temp;
  float realAngle;
  if (DIR == true) {realAngle = -ANGLE;} else {realAngle = ANGLE;};
  //分象限,使用极坐标求解
  const int k = 0;
  temp = sq(midCoord[0])+sq(midCoord[1]);
  r = sqrt(temp);
  if (midCoord[0]>0 && midCoord[1]>0) { //第一象限
    A = atan(abs(midCoord[k+1])/abs(midCoord[k]));  //计算一个坐标点和x轴构成的角度值
    x1 = r*cos(A+realAngle); //新坐标值
    y1 = r*sin(A+realAngle);
    TARGETPOS[k] = x1;
    TARGETPOS[k+1] = y1;
  } else if (midCoord[0]<0 && midCoord[1]>0) {  //第二象限
    A = PI - atan(abs(midCoord[k+1])/abs(midCoord[k])); 
    x1 = r*cos(A+realAngle); 
    y1 = r*sin(A+realAngle);
    TARGETPOS[k] = x1;
    TARGETPOS[k+1] = y1;
  } else if (midCoord[0]<0 && midCoord[1]<0) {  //第三象限
    A = PI + atan(abs(midCoord[k+1])/abs(midCoord[k])); 
    x1 = r*cos(A+realAngle);
    y1 = r*sin(A+realAngle);
    TARGETPOS[k] = x1;
    TARGETPOS[k+1] = y1;
  } else if (midCoord[0]>0 && midCoord[1]<0) {  //第四象限
    A = 2*PI - atan(abs(midCoord[k+1])/abs(midCoord[k])); 
    x1 = r*cos(A+realAngle);
    y1 = r*sin(A+realAngle);
    TARGETPOS[k] = x1;
    TARGETPOS[k+1] = y1;
  } else {
    if (midCoord[0]>0 && midCoord[1]==0) { //在正X轴上
      A = 0;
      x1 = r*cos(A+realAngle);
      y1 = r*sin(A+realAngle);
      TARGETPOS[k] = x1;
      TARGETPOS[k+1] = y1;
    } else if (midCoord[0]==0 && midCoord[1]>0) { //在正y轴上
      A = PI/2;
      x1 = r*cos(A+realAngle);
      y1 = r*sin(A+realAngle);
      TARGETPOS[k] = x1;
      TARGETPOS[k+1] = y1;
    } else if (midCoord[0]<0 && midCoord[1]==0) { //在负X轴上
      A = PI;
      x1 = r*cos(A+realAngle);
      y1 = r*sin(A+realAngle);
      TARGETPOS[k] = x1;
      TARGETPOS[k+1] = y1;
    } else if (midCoord[0]==0 && midCoord[1]<0) { //在负y轴上
      A = 1.5*PI;
      x1 = r*cos(A+realAngle);
      y1 = r*sin(A+realAngle);
      TARGETPOS[k] = x1;
      TARGETPOS[k+1] = y1;
    }
  }
  //反重建
  SINGLEPOS[0] = TARGETPOS[0] + startPoint[0];
  SINGLEPOS[1] = startPoint[1] - TARGETPOS[1];

  return SINGLEPOS;
}


//////////////////////////////////////////////

float *Arc::_calculateEveryCoord(float winCenter[2], float winSize[2]) {  
  //计算所有源坐标：输入矩形中心点和长宽，输出所有源坐标的首地址
  ALLPOS[0] = winCenter[0] - winSize[0]/2;
  ALLPOS[1] = winCenter[1] - winSize[1]/2;
  ALLPOS[2] = winCenter[0] + winSize[0]/2;
  ALLPOS[3] = winCenter[1] - winSize[1]/2; 
  ALLPOS[4] = winCenter[0] + winSize[0]/2;
  ALLPOS[5] = winCenter[1] + winSize[1]/2;
  ALLPOS[6] = winCenter[0] - winSize[0]/2;
  ALLPOS[7] = winCenter[1] + winSize[1]/2; 
  
  return ALLPOS;
}



float *Arc::_coordRebuild(float newCenter[2], float inputCoord[4][2]) {
  //坐标系重建：输入新的原点（相对于（0，0））和需要改变原点的源坐标，输出新坐标的首地址
  MIDPOS[0] = inputCoord[0][0] - newCenter[0];
  MIDPOS[1] = newCenter[1] - inputCoord[0][1];
  MIDPOS[2] = inputCoord[1][0] - newCenter[0];
  MIDPOS[3] = newCenter[1] - inputCoord[1][1];
  MIDPOS[4] = inputCoord[2][0] - newCenter[0];
  MIDPOS[5] = newCenter[1] - inputCoord[2][1];
  MIDPOS[6] = inputCoord[3][0] - newCenter[0];
  MIDPOS[7] = newCenter[1] - inputCoord[3][1];
  return MIDPOS;
}

float *Arc::_calculateTargetCoord(float midCoord[4][2], float ANGLE, bool DIR){
  //计算目标坐标：输入重建后的坐标、旋转角度、方向，输出计算后的首地址
  float TARGETPOS[4][2];
  float A, r, x1, y1, temp;
  /*temp = sq(midCoord[0][0]-midCoord[2][0])+sq(midCoord[0][1]-midCoord[2][1]);
  r = sqrt(temp)/2;  //圆的半径*/
  float realAngle;
  if (DIR == true) {realAngle = -ANGLE;} else {realAngle = ANGLE;};
  //分象限,使用极坐标求解
  const int k = 0;
  for (int i=0;i<4;i++) {
    temp = sq(midCoord[i][0])+sq(midCoord[i][1]);
    r = sqrt(temp);
    if (midCoord[i][0]>0 && midCoord[i][1]>0) { //第一象限
      A = atan(abs(midCoord[i][k+1])/abs(midCoord[i][k]));  //计算一个坐标点和x轴构成的角度值
      x1 = r*cos(A+realAngle); //新坐标值
      y1 = r*sin(A+realAngle);
      TARGETPOS[i][k] = x1;
      TARGETPOS[i][k+1] = y1;
    } else if (midCoord[i][0]<0 && midCoord[i][1]>0) {  //第二象限
      A = PI - atan(abs(midCoord[i][k+1])/abs(midCoord[i][k])); 
      x1 = r*cos(A+realAngle); 
      y1 = r*sin(A+realAngle);
      TARGETPOS[i][k] = x1;
      TARGETPOS[i][k+1] = y1;
    } else if (midCoord[i][0]<0 && midCoord[i][1]<0) {  //第三象限
      A = PI + atan(abs(midCoord[i][k+1])/abs(midCoord[i][k])); 
      x1 = r*cos(A+realAngle);
      y1 = r*sin(A+realAngle);
      TARGETPOS[i][k] = x1;
      TARGETPOS[i][k+1] = y1;
    } else if (midCoord[i][0]>0 && midCoord[i][1]<0) {  //第四象限
      A = 2*PI - atan(abs(midCoord[i][k+1])/abs(midCoord[i][k])); 
      x1 = r*cos(A+realAngle);
      y1 = r*sin(A+realAngle);
      TARGETPOS[i][k] = x1;
      TARGETPOS[i][k+1] = y1;
    } else {
      if (midCoord[i][0]>0 && midCoord[i][1]==0) { //在正X轴上
        A = 0;
        x1 = r*cos(A+realAngle);
        y1 = r*sin(A+realAngle);
        TARGETPOS[i][k] = x1;
        TARGETPOS[i][k+1] = y1;
      } else if (midCoord[i][0]==0 && midCoord[i][1]>0) { //在正y轴上
        A = PI/2;
        x1 = r*cos(A+realAngle);
        y1 = r*sin(A+realAngle);
        TARGETPOS[i][k] = x1;
        TARGETPOS[i][k+1] = y1;
      } else if (midCoord[i][0]<0 && midCoord[i][1]==0) { //在负X轴上
        A = PI;
        x1 = r*cos(A+realAngle);
        y1 = r*sin(A+realAngle);
        TARGETPOS[i][k] = x1;
        TARGETPOS[i][k+1] = y1;
      } else if (midCoord[i][0]==0 && midCoord[i][1]<0) { //在负y轴上
        A = 1.5*PI;
        x1 = r*cos(A+realAngle);
        y1 = r*sin(A+realAngle);
        TARGETPOS[i][k] = x1;
        TARGETPOS[i][k+1] = y1;
      } else {  //在零点
        TARGETPOS[i][k] = midCoord[i][0];
        TARGETPOS[i][k+1] = midCoord[i][1];
      }
    }
  }
  int m=0;
  for (int i=0;i<4;i++) {
    for (int k=0;k<2;k++) {
      TargetPos[m] = TARGETPOS[i][k];
      m++;      
    }
  }
  
  return TargetPos;
}
  
