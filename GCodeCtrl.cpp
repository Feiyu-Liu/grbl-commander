
#include "GCodeCtrl.h"

////////////////////////PUBLIC////////////////////////

//构造函数
GCodeCtrl::GCodeCtrl (float Slide[2], float TeleL[2], float TeleS[2]) {
  _rockerControllerSpeed = ROCKER_SPEED;
  _steppersSpeed = STEPPERS_SPEED;
  //电机参数初始化
  _stepsParameter[0][0] = Slide[0];
  _stepsParameter[0][1] = Slide[1];
  _stepsParameter[0][2] =Slide[1]/Slide[0];
  _stepsParameter[1][0] = TeleL[0];
  _stepsParameter[1][1] = TeleL[1];
  _stepsParameter[1][2] =TeleL[1]/TeleL[0];
  _stepsParameter[2][0] = TeleS[0];
  _stepsParameter[2][1] = TeleS[1];
  _stepsParameter[2][2] =TeleS[1]/TeleS[0];
  //Arc实例初始化
  _MyArc = new Arc(_stepsParameter[0][0], _stepsParameter[0][0]);
  //trigger引脚初始化
  pinMode(DYNAMIC_TRIGGER_PIN, OUTPUT);
  digitalWrite(DYNAMIC_TRIGGER_PIN, LOW);
  //摄像机运动检测初始化
  pinMode(MOTION_CAPTURE_TRIGGER,INPUT);
}

//析构函数
GCodeCtrl::~GCodeCtrl () {

}

//Serial初始化（需要在主程序的set up里调用）
void GCodeCtrl::serialBegin(){
  MySender->serialBegin();
}

#ifdef voidDebug
void GCodeCtrl::Debug() {

  
}
#endif


void GCodeCtrl::steppersCalibration(float compensate[4][2]){
  #ifdef serialDebug
  MySender->bridgeSerial->println(F("/CALIBRATING/"));
  MySender->bridgeSerial->println(F("Rocker Controller Begin"));
  #endif
  this->rockerController();
  #ifdef serialDebug
  MySender->bridgeSerial->println(F("Rocker Controller Finished"));
  #endif
  
  delay(20);
  //将补偿坐标厘米转化为毫米
  float compensateMM[4][2];
  for (int i=0;i<4;i++) {
    for (int k=0;k<2;k++) {
      compensateMM[i][k] = this->_unitConversionToMm(compensate[i][k]);
    }
  }
  
  //归零并补偿工作坐标
  String compensateCmd = "";
  for (int i=0;i<4;i++) {
    compensateCmd = "G92 X" + String(compensateMM[i][0]) + "Y" + String(compensateMM[i][1]) + "\n";
    MySender->grblsArray[i]->currentCmd = compensateCmd;
    delay(10);

    #ifdef serialDebug
    MySender->bridgeSerial->print(i+1);
    MySender->bridgeSerial->print(":");
    String temp = compensateCmd;
    temp.replace("\n", "");
    MySender->bridgeSerial->println(temp);
    #endif 
    
  }
  
  //发送并自动重试
  MySender->sendNowAutoRetry();

  //刷新current参数
  for (int i=0;i<2;i++) {
    for (int k=0;k<2;k++) {
      _currentCoord[i][k] = 0;
    }
  }
  for (int i=2;i<4;i++) {
    for (int k=0;k<2;k++) {
      _currentCoord[i][k] = 80;
    }
  }
  
  for (int i=0;i<2;i++) {
    _currentWinData[0][i] = 40;
  } 
  for (int i=0;i<2;i++) {
    _currentWinData[1][i] = 80;
  }
  _currentWinData[2][0] = 0;
  _currentWinData[2][1] = 0; 
  //EEPROM写入
  
  this->_eepromWrite();
}

void GCodeCtrl::steppersCalibration(){

  this->_eepromRead();
  delay(10);

  //读取当前坐标
  float currentPos[4][2];
  for (int i=0;i<4;i++) {
    for (int k=0;k<2;k++) {
      currentPos[i][k] = _currentCoord[i][k];
    }
  }

  //世界坐标系转化为输出坐标系
  float outPutPos[4][2];
  float *ptrOutPutPos = this->_coordinateRebuild(currentPos);
  for (int i=0;i<4;i++) {  
    for (int k=0;k<2;k++) {
      outPutPos[i][k] = *ptrOutPutPos;
      ptrOutPutPos++;
    }
  }
  //cm转mm
  for (int i=0;i<4;i++) {
    for (int k=0;k<2;k++) {
      outPutPos[i][k] = this->_unitConversionToMm(outPutPos[i][k]);
    }
  }
  
  //归零并补偿工作坐标
  String cmd = "";
  for (int i=0;i<4;i++) {
    cmd = "G92 X" + String(outPutPos[i][0]) + "Y" + String(outPutPos[i][1]) + "\n";
    MySender->grblsArray[i]->currentCmd = cmd;
    delay(10);
  }
  
  //发送并自动重试
  MySender->sendNowAutoRetry();

}

void GCodeCtrl::runStraightToTarget(float WinCenter[2], float WinSize[2]){
  #ifdef serialDebug
  MySender->bridgeSerial->println(F("/STRAIIGHT_TO_TARGET/"));
  #endif
  //计算所有目标坐标，世界坐标系
  float allOriPos[4][2];  
  float *ptrAllPos = this->_calculateEveryCoord(WinCenter, WinSize);
  for (int i=0;i<4;i++) {  //储存指针中的数组数据
    for (int k=0;k<2;k++) {
      allOriPos[i][k] = *ptrAllPos;
      ptrAllPos++;
    }
  }
  #ifdef serialDebug
  MySender->bridgeSerial->println(F("WPos:"));
  for (int i=0;i<4;i++) {  //储存指针中的数组数据
    for (int k=0;k<2;k++) {
      MySender->bridgeSerial->print(allOriPos[i][k]);
      MySender->bridgeSerial->print(F(","));
    }
  }
  #endif
  //世界坐标系转化为输出坐标系
  float outPutPos[4][2];
  float *ptrOutPutPos = this->_coordinateRebuild(allOriPos);
  for (int i=0;i<4;i++) {  //储存指针中的数组数据
    for (int k=0;k<2;k++) {
      outPutPos[i][k] = *ptrOutPutPos;
      ptrOutPutPos++;
    }
  }
  //cm转mm
  for (int i=0;i<4;i++) {
    for (int k=0;k<2;k++) {
      outPutPos[i][k] = this->_unitConversionToMm(outPutPos[i][k]);
    }
  }
  #ifdef serialDebug
  MySender->bridgeSerial->println(F("OPos:"));
  for (int i=0;i<4;i++) {  //储存指针中的数组数据
    for (int k=0;k<2;k++) {
      MySender->bridgeSerial->print(outPutPos[i][k]);
      MySender->bridgeSerial->print(F(","));
    }
  }
  #endif
  //发送
  delay(SENDING_DELAY);
  bool state = MySender->sendStraightCmd(outPutPos, _steppersSpeed);
  if (!state){
    #ifdef serialDebug
    MySender->bridgeSerial->println(F("Error, check your cmd."));
    #endif
    //刷新成员参数(返回值不准确，故仍然刷新目前窗口参数)
    for (int i=0;i<4;i++) {
      for (int k=0;k<2;k++) {
        _currentCoord[i][k] = allOriPos[i][k];
      }
    }
    for (int i=0;i<2;i++) {
      _currentWinData[0][i] = WinCenter[i];
    } 
    for (int i=0;i<2;i++) {
      _currentWinData[1][i] = WinSize[i];
    }
    _currentWinData[2][0] = 0;
    _currentWinData[2][1] = 0; 
  } else if (state) {
    //刷新成员参数
    for (int i=0;i<4;i++) {
      for (int k=0;k<2;k++) {
        _currentCoord[i][k] = allOriPos[i][k];
      }
    }
    for (int i=0;i<2;i++) {
      _currentWinData[0][i] = WinCenter[i];
    } 
    for (int i=0;i<2;i++) {
      _currentWinData[1][i] = WinSize[i];
    }
    _currentWinData[2][0] = 0;
    _currentWinData[2][1] = 0; 
  }
  this->_eepromWrite();
}

void GCodeCtrl::runStraightToTarget(float WinCenter[2], float WinSize[2], bool Direction, float Angle){
  #ifdef serialDebug
  MySender->bridgeSerial->println(F("/STRAIIGHT_TO_TARGET_ANGLE/"));
  #endif
  //计算所有目标坐标（世界坐标系）
  float allArcPos[4][2];  
  float *ptrArcAllPos = _MyArc->everyTargetCoord(WinCenter, WinSize, Direction, Angle);
  for (int i=0;i<4;i++) {  //储存指针中的数组数据
    for (int k=0;k<2;k++) {
      allArcPos[i][k] = *ptrArcAllPos;
      ptrArcAllPos++;
    }
  }
  #ifdef serialDebug
  MySender->bridgeSerial->println(F("WPos:"));
  for (int i=0;i<4;i++) {  //储存指针中的数组数据
    for (int k=0;k<2;k++) {
      MySender->bridgeSerial->print(allArcPos[i][k]);
      MySender->bridgeSerial->print(F(","));
    }
  }
  #endif
  //世界坐标系转化为输出坐标系
  float outPutPos[4][2];
  float *ptrOutPutPos = this->_coordinateRebuild(allArcPos);
  for (int i=0;i<4;i++) {  //储存指针中的数组数据
    for (int k=0;k<2;k++) {
      outPutPos[i][k] = *ptrOutPutPos;
      ptrOutPutPos++;
    }
  }
  //cm转mm
  for (int i=0;i<4;i++) {
    for (int k=0;k<2;k++) {
      outPutPos[i][k] = this->_unitConversionToMm(outPutPos[i][k]);
    }
  }
  #ifdef serialDebug
  MySender->bridgeSerial->println(F("OPos:"));
  for (int i=0;i<4;i++) {  //储存指针中的数组数据
    for (int k=0;k<2;k++) {
      MySender->bridgeSerial->print(outPutPos[i][k]);
      MySender->bridgeSerial->print(F(","));
    }
  }
  #endif
  //发送
  delay(SENDING_DELAY);
  bool state = MySender->sendStraightCmd(outPutPos, _steppersSpeed);
  if (!state){
    #ifdef serialDebug
    MySender->bridgeSerial->println(F("Error, check your cmd."));
    #endif
    //此函数为过渡函数，刷新成员参数(返回值不准确，故仍然刷新目前窗口参数)
    for (int i=0;i<4;i++) {
      for (int k=0;k<2;k++) {
        _currentCoord[i][k] = allArcPos[i][k];
      }
    }
    for (int i=0;i<2;i++) {
      _currentWinData[0][i] = WinCenter[i];
    } 
    for (int i=0;i<2;i++) {
      _currentWinData[1][i] = WinSize[i];
    }
    _currentWinData[2][0] = Direction;
    _currentWinData[2][1] = Angle; 
  } else if (state) {
    //刷新成员参数
    for (int i=0;i<4;i++) {
      for (int k=0;k<2;k++) {
        _currentCoord[i][k] = allArcPos[i][k];
      }
    }
    for (int i=0;i<2;i++) {
      _currentWinData[0][i] = WinCenter[i];
    } 
    for (int i=0;i<2;i++) {
      _currentWinData[1][i] = WinSize[i];
    }
    _currentWinData[2][0] = Direction;
    _currentWinData[2][1] = Angle; 
  }
  this->_eepromWrite();
}

void GCodeCtrl::runArcToTarget(float WinCenter[2], float WinSize[2], bool Direction, float Angle){
  #ifdef serialDebug
  MySender->bridgeSerial->println(F("/ARC_TO_TARGET/"));
  #endif
  //获取当前源坐标
  float allOriPos[4][2];  
  for (int i=0;i<4;i++) {  //储存指针中的数组数据
    for (int k=0;k<2;k++) {
      allOriPos[i][k] = _currentCoord[i][k];
    }
  }
  //世界坐标系转化为输出坐标系
  float outPutOriPos[4][2];
  float *ptrOutPutOriPos = this->_coordinateRebuild(allOriPos);
  for (int i=0;i<4;i++) {  //储存指针中的数组数据
    for (int k=0;k<2;k++) {
      outPutOriPos[i][k] = *ptrOutPutOriPos;
      ptrOutPutOriPos++;
    }
  }
  //cm转mm
  for (int i=0;i<4;i++) {
    for (int k=0;k<2;k++) {
      outPutOriPos[i][k] = this->_unitConversionToMm(outPutOriPos[i][k]);
    }
  }
  #ifdef serialDebug
  MySender->bridgeSerial->println(F("Current OPos:"));
  for (int i=0;i<4;i++) {  //储存指针中的数组数据
    for (int k=0;k<2;k++) {
      MySender->bridgeSerial->print(outPutOriPos[i][k]);
      MySender->bridgeSerial->print(F(","));
    }
  }
  #endif
  ///////////////////////////////////////////////////
  //计算所有目标坐标，世界坐标系
  float allArcPos[4][2];  
  float *ptrArcAllPos = _MyArc->everyTargetCoord(WinCenter, WinSize, Direction, Angle);
  for (int i=0;i<4;i++) {  //储存指针中的数组数据
    for (int k=0;k<2;k++) {
      allArcPos[i][k] = *ptrArcAllPos;
      ptrArcAllPos++;
    }
  }
  #ifdef serialDebug
  MySender->bridgeSerial->println(F("Target WPos:"));
  for (int i=0;i<4;i++) {  //储存指针中的数组数据
    for (int k=0;k<2;k++) {
      MySender->bridgeSerial->print(allArcPos[i][k]);
      MySender->bridgeSerial->print(F(","));
    }
  }
  delay(1000);
  #endif
  //世界坐标系转化为输出坐标系
  float outPutPos[4][2];
  float *ptrOutPutPos = this->_coordinateRebuild(allArcPos);
  for (int i=0;i<4;i++) {  //储存指针中的数组数据
    for (int k=0;k<2;k++) {
      outPutPos[i][k] = *ptrOutPutPos;
      ptrOutPutPos++;
    }
  }
  //cm转mm
  for (int i=0;i<4;i++) {
    for (int k=0;k<2;k++) {
      outPutPos[i][k] = this->_unitConversionToMm(outPutPos[i][k]);
    }
  }
  ////////////////////////////////////////////////////
  //wincenter转输出坐标系mm
  float tempWinCenterOutPut[4][2];
  for (int i=0;i<4;i++) {  
    for (int k=0;k<2;k++) {
      tempWinCenterOutPut[i][k] = WinCenter[k];
    }
  }
  float winCenterOutPut[4][2];
  float *ptrwinCenterOutPut = this->_coordinateRebuild(tempWinCenterOutPut);
  for (int i=0;i<4;i++) {  
    for (int k=0;k<2;k++) {
      winCenterOutPut[i][k] = *ptrwinCenterOutPut;
      ptrwinCenterOutPut++;
    }
  }
  for (int i=0;i<4;i++) {  
    for (int k=0;k<2;k++) {
      winCenterOutPut[i][k] = this->_unitConversionToMm(winCenterOutPut[i][k]);
    }
  }
  
  #ifdef serialDebug
  MySender->bridgeSerial->println(F("OPos:"));
  for (int i=0;i<4;i++) {  //储存指针中的数组数据
    for (int k=0;k<2;k++) {
      MySender->bridgeSerial->print(outPutPos[i][k]);
      MySender->bridgeSerial->print(F(","));
    }
  }
  #endif
  //发送
  delay(SENDING_DELAY);
  bool state = MySender->sendArcCmd(outPutPos, winCenterOutPut,outPutOriPos, Direction, _steppersSpeed);
  if (!state){
    #ifdef serialDebug
    MySender->bridgeSerial->println(F("Error, check your cmd."));
    #endif
    //刷新成员参数(返回值不准确，故仍然刷新目前窗口参数)
    for (int i=0;i<4;i++) {
      for (int k=0;k<2;k++) {
        _currentCoord[i][k] = allArcPos[i][k];
      }
    }
    for (int i=0;i<2;i++) {
      _currentWinData[0][i] = WinCenter[i];
    } 
    for (int i=0;i<2;i++) {
      _currentWinData[1][i] = WinSize[i];
    }
    _currentWinData[2][0] = Direction;
    _currentWinData[2][1] = Angle;  
  } else if (state) {
    //刷新成员参数
    for (int i=0;i<4;i++) {
      for (int k=0;k<2;k++) {
        _currentCoord[i][k] = allArcPos[i][k];
      }
    }
    for (int i=0;i<2;i++) {
      _currentWinData[0][i] = WinCenter[i];
    } 
    for (int i=0;i<2;i++) {
      _currentWinData[1][i] = WinSize[i];
    }
    _currentWinData[2][0] = Direction;
    _currentWinData[2][1] = Angle;  
  }
  this->_eepromWrite();
}

void GCodeCtrl::runArcToTarget(float WinCenter[2], float WinSize[2], int vertexNum, bool Direction, float Angle){
  #ifdef serialDebug
  Serial.println(F("/ARC_TO_TARGET_S/"));
  #endif
  //获取当前源坐标
  float allOriPos[4][2];  
  for (int i=0;i<4;i++) {  //储存指针中的数组数据
    for (int k=0;k<2;k++) {
      allOriPos[i][k] = _currentCoord[i][k];
    }
  }
  #ifdef serialDebug
  Serial.println(F("current pos:"));
  for (int k=0;k<4;k++) {
    for (int m=0;m<2;m++) {
      Serial.println(allOriPos[k][m]);
      Serial.println("");
    }
  }
  #endif
  //世界坐标系转化为输出坐标系
  float outPutOriPos[4][2];
  float *ptrOutPutOriPos = this->_coordinateRebuild(allOriPos);
  for (int i=0;i<4;i++) {  //储存指针中的数组数据
    for (int k=0;k<2;k++) {
      outPutOriPos[i][k] = *ptrOutPutOriPos;
      ptrOutPutOriPos++;
    }
  }
  //cm转mm
  for (int i=0;i<4;i++) {
    for (int k=0;k<2;k++) {
      outPutOriPos[i][k] = this->_unitConversionToMm(outPutOriPos[i][k]);
    }
  }
  #ifdef serialDebug
  Serial.println(F("Current OPos:"));
  for (int i=0;i<4;i++) {  //储存指针中的数组数据
    for (int k=0;k<2;k++) {
      Serial.print(outPutOriPos[i][k]);
      Serial.print("   ");
      delay(SENDING_DELAY_TIME);
    }
  }
  #endif
  ///////////////////////////////////////////////////
  //计算所有目标坐标，世界坐标系
  float allArcPos[4][2];  
  float *ptrArcAllPos = _MyArc->everyTargetCoord(WinCenter, WinSize, vertexNum,Direction, Angle);
  for (int i=0;i<4;i++) {  //储存指针中的数组数据
    for (int k=0;k<2;k++) {
      allArcPos[i][k] = *ptrArcAllPos;
      ptrArcAllPos++;
    }
  }

  #ifdef serialDebug
  Serial.println(F("Target WPos:"));
  for (int i=0;i<4;i++) {  //储存指针中的数组数据
    for (int k=0;k<2;k++) {
      Serial.print(allArcPos[i][k]);
      Serial.print(F("   "));
      delay(SENDING_DELAY_TIME);
    }
  }
  delay(1000);
  #endif
  //世界坐标系转化为输出坐标系
  float outPutPos[4][2];
  float *ptrOutPutPos = this->_coordinateRebuild(allArcPos);
  for (int i=0;i<4;i++) {  //储存指针中的数组数据
    for (int k=0;k<2;k++) {
      outPutPos[i][k] = *ptrOutPutPos;
      ptrOutPutPos++;
    }
  }
  //cm转mm
  for (int i=0;i<4;i++) {
    for (int k=0;k<2;k++) {
      outPutPos[i][k] = this->_unitConversionToMm(outPutPos[i][k]);
    }
  }
  ////////////////////////////////////////////////////
  //计算旋转锚点坐标——>mm
  if (vertexNum == 1 || vertexNum == 2 || vertexNum == 3 || vertexNum == 4) {
  } else {
    MySender->bridgeSerial->println(F("Warning:Invalid vertex number"));
    vertexNum = 1;
  }
  float winAnchor[2]; //原始中心点
  winAnchor[0] = allOriPos[vertexNum-1][0];
  winAnchor[1] = allOriPos[vertexNum-1][1];
  float tempAnchorCenterOutput[4][2];
  for (int i=0;i<4;i++) {  
    for (int k=0;k<2;k++) {
      tempAnchorCenterOutput[i][k] = winAnchor[k];
    }
  }
  float anchorCenterOutPut[4][2];
  float *ptranchorCenterOutPut = this->_coordinateRebuild(tempAnchorCenterOutput);
  for (int i=0;i<4;i++) {  
    for (int k=0;k<2;k++) {
      anchorCenterOutPut[i][k] = *ptranchorCenterOutPut;
      ptranchorCenterOutPut++;
    }
  }
  //cm转mm
  for (int i=0;i<4;i++) {
    for (int k=0;k<2;k++) {
      anchorCenterOutPut[i][k] = this->_unitConversionToMm(anchorCenterOutPut[i][k]);
    }
  }

  //计算新窗口中心点
  float newWinCenter[2];
  float *ptrnewWinCenter = _MyArc->singleTargetCoord(allOriPos[vertexNum-1],WinCenter, Direction, Angle);
  newWinCenter[0] = *ptrnewWinCenter;
  ptrnewWinCenter++;
  newWinCenter[1] = *ptrnewWinCenter;

  //计算每个电机的速度
  float s_speedSync[4], s_r[4], s_tempR[4],s_temp, s_v;
  for (int m=0;m<4;m++) {
    s_temp = sq(allOriPos[m][0]-winAnchor[0]) + sq(allOriPos[m][1]-winAnchor[1]);
    s_r[m] = sqrt(s_temp);
    s_tempR[m] = s_r[m];
  }
  _bubbleSort(s_tempR,4);
  float s_wMin = _steppersSpeed / s_tempR[3];
  for (int m=0;m<4;m++){
    s_v = s_wMin * s_r[m];
    s_speedSync[m] = s_v;
  }

  //发送
  delay(SENDING_DELAY);
  bool state = MySender->sendAnchorArcCmd(outPutPos, anchorCenterOutPut, outPutOriPos, Direction, s_speedSync);
  if (!state){
    #ifdef serialDebug
    MySender->bridgeSerial->println(F("Error, check your cmd."));
    #endif
    //刷新成员参数(返回值不准确，故仍然刷新目前窗口参数)
    for (int i=0;i<4;i++) {
      for (int k=0;k<2;k++) {
        _currentCoord[i][k] = allArcPos[i][k]; //正确
      }
    }
    
    for (int i=0;i<2;i++) {
      _currentWinData[0][i] = newWinCenter[i]; //需计算真正的中心点
    } 
    for (int i=0;i<2;i++) {
      _currentWinData[1][i] = WinSize[i]; //正确
    }
    _currentWinData[2][0] = Direction; //正确
    _currentWinData[2][1] = Angle;  //正确
  } else if (state) {
    //刷新成员参数
    for (int i=0;i<4;i++) {
      for (int k=0;k<2;k++) {
        _currentCoord[i][k] = allArcPos[i][k];
      }
    }
    for (int i=0;i<2;i++) {
      _currentWinData[0][i] = newWinCenter[i]; //需计算真正的中心点
    } 
    for (int i=0;i<2;i++) {
      _currentWinData[1][i] = WinSize[i];
    }
    _currentWinData[2][0] = Direction;
    _currentWinData[2][1] = Angle;  
  }
  this->_eepromWrite();
}

void GCodeCtrl::automaticArrival(float WindowsParameter[3][2]){
  //（需刷新）储存当前窗口参数。第一层：窗口中心，第二层：窗口长宽，第三层:窗口方向、角度
  //float _currentWinData[3][2] = {0};  
  bool dirAndAngleState = WindowsParameter[2][0] == _currentWinData[2][0] && WindowsParameter[2][1] == _currentWinData[2][1];
  bool sizeAndCenterState = WindowsParameter[0][0] == _currentWinData[0][0] && WindowsParameter[0][1] == _currentWinData[0][1] && WindowsParameter[1][0] == _currentWinData[1][0] && WindowsParameter[1][1] == _currentWinData[1][1];
  bool newDir;
  bool dirJudge = WindowsParameter[2][0] == _currentWinData[2][0];
  bool angleJudge = WindowsParameter[2][1] > _currentWinData[2][1];  //新>旧
  bool dirFalseJudge = WindowsParameter[2][1] == 0;

  //路径规划
  if (dirAndAngleState) {
    if (sizeAndCenterState) {
      #ifdef serialDebug
      MySender->bridgeSerial->println("state4->same as last window");
      #endif
    } else if (!sizeAndCenterState) {
      #ifdef serialDebug
      MySender->bridgeSerial->println("state3->straight to target");
      #endif
      this->runStraightToTarget(WindowsParameter[0], WindowsParameter[1], WindowsParameter[2][0], WindowsParameter[2][1]);
    }
  } else if (!dirAndAngleState) {
    if (sizeAndCenterState) {
      if (dirJudge) {
        if (angleJudge) {
          #ifdef serialDebug
          MySender->bridgeSerial->println("state1->arc to target, same dir, new angle larger");
          #endif
          this->runArcToTarget(WindowsParameter[0], WindowsParameter[1], WindowsParameter[2][0], WindowsParameter[2][1]);
        } else if (!angleJudge) {
          #ifdef serialDebug
          MySender->bridgeSerial->println("state1->arc to target, same dir, old angle larger");
          #endif
          bool temp = WindowsParameter[2][0];
          temp = !temp;
          this->runArcToTarget(WindowsParameter[0], WindowsParameter[1], temp, WindowsParameter[2][1]);
        }
      } else if (!dirJudge) {
        if (dirFalseJudge) {
          #ifdef serialDebug
          MySender->bridgeSerial->println("state1->arc to target, different dir, new angle=0");
          #endif
          bool temp = _currentWinData[2][0];
          temp = !temp;
          this->runArcToTarget(WindowsParameter[0], WindowsParameter[1], temp, WindowsParameter[2][1]);
        } else if (!dirFalseJudge) {
          #ifdef serialDebug
          MySender->bridgeSerial->println("state1->arc to target, different dir, new angle!=0");
          #endif
          this->runArcToTarget(WindowsParameter[0], WindowsParameter[1], WindowsParameter[2][0], WindowsParameter[2][1]);
        }
      }
    } else if (!sizeAndCenterState) {
      this->runStraightToTarget(WindowsParameter[0], WindowsParameter[1], _currentWinData[2][0], _currentWinData[2][1]);
      //MySender->delayCmd(STRAIGHT_TO_ARC_DELAY);
      if (dirJudge) {
        if (angleJudge) {
          #ifdef serialDebug
          MySender->bridgeSerial->println("state2->straight and arc, same dir, new angle larger");
          #endif
          this->runArcToTarget(WindowsParameter[0], WindowsParameter[1], WindowsParameter[2][0], WindowsParameter[2][1]);
        } else if (!angleJudge) {
          #ifdef serialDebug
          MySender->bridgeSerial->println("state2->straight and arc, same dir, old angle larger");
          #endif
          bool temp = WindowsParameter[2][0];
          temp = !temp;
          this->runArcToTarget(WindowsParameter[0], WindowsParameter[1], temp, WindowsParameter[2][1]);
        }
      } else if (!dirJudge) {
        if (dirFalseJudge) {
          #ifdef serialDebug
          MySender->bridgeSerial->println("state2->straight and arc, different dir, new angle=0");
          #endif
          bool temp = _currentWinData[2][0];
          temp = !temp;
          this->runArcToTarget(WindowsParameter[0], WindowsParameter[1], temp, WindowsParameter[2][1]);
        } else if (!dirFalseJudge) {
          #ifdef serialDebug
          MySender->bridgeSerial->println("state2->straight and arc, same dir, new angle!=0");
          #endif
          this->runArcToTarget(WindowsParameter[0], WindowsParameter[1], WindowsParameter[2][0], WindowsParameter[2][1]);
        }
      }
    }
  }

  //刷新currentWinData
  for (int i=0;i<2;i++) {
    _currentWinData[0][i] = WindowsParameter[0][i];
  } 
  for (int i=0;i<2;i++) {
    _currentWinData[1][i] = WindowsParameter[1][i];
  }
  _currentWinData[2][0] = WindowsParameter[2][0];
  _currentWinData[2][1] = WindowsParameter[2][1];  

}

void GCodeCtrl::rockerController(){
  /*
           2
           |
           |    
          750  
  1---- 680   358------3
          260
           |
           |
           4
  */

  int rocker_X, rocker_Y;
  unsigned long rockerSpeed;
  const int btnArray[4] = {BUTTON_2_PIN, BUTTON_1_PIN, BUTTON_4_PIN, BUTTON_3_PIN};
  int buttonNumber = 1; //没有任何按钮被按下默认=1
  float dS = 0.1;//厘米
  float dt;  //秒
  String joggingCmd[4][4] = {
    {"X"+String(dS), "X-"+String(dS), "Y-"+String(dS), "Y"+String(dS)},
    {"X"+String(dS), "X-"+String(dS), "Y-"+String(dS), "Y"+String(dS)},
    {"X-"+String(dS), "X"+String(dS), "Y"+String(dS), "Y-"+String(dS)},
    {"X-"+String(dS), "X"+String(dS), "Y"+String(dS), "Y-"+String(dS)}
  };
  String finalCmd = "";
  bool runStop = true;
  int quadrant = -1; 

  // 加速度设置
  this->_setAcceleration(1500);

  while (digitalRead(BUTTON_0_PIN)) {
    rocker_Y = analogRead(S_Y_PIN);
    rocker_X = analogRead(S_X_PIN);
    if (rocker_X < 358 && rocker_Y < 750 && rocker_Y > 260 ) { //+x轴
      quadrant = 0;
      rockerSpeed = map(rocker_X, 358, 0, 50, _rockerControllerSpeed);
    } else if (rocker_X > 680 && rocker_Y < 750 && rocker_Y > 260 ) { //-x轴
      quadrant = 1;
      rockerSpeed = map(rocker_X, 680, 1023, 50, _rockerControllerSpeed);
    } else if (rocker_Y > 750 && rocker_X > 358 && rocker_X < 680 ) { //+Y轴
      quadrant = 2;
      rockerSpeed = map(rocker_Y, 750, 1023, 50, _rockerControllerSpeed);
    } else if (rocker_Y < 260 && rocker_X > 358 && rocker_X < 680 ) { //-Y轴
      quadrant = 3;
      rockerSpeed = map(rocker_Y, 260, 0, 50, _rockerControllerSpeed);
    } else {
      quadrant = -1; //摇杆归位
    }

    if (quadrant != -1) {
      //dt = (dS*10)/(rockerSpeed/60);
      finalCmd = "$J=G91G20" + joggingCmd[buttonNumber-1][quadrant] + 'F' + String(rockerSpeed) + "\n";
      MySender->serialPort[buttonNumber-1]->print(finalCmd);
      runStop = true;
      delay(map(rockerSpeed,100,_rockerControllerSpeed,50,6));
      while (!MySender->serialPort[buttonNumber-1]->find("ok")) {} //等待接收ok
    } else {
      for (int i=0;i<4;i++) {
        if (!digitalRead(btnArray[i])) { //按钮按下
          buttonNumber = i+1;
          break;
        }
      }
      if (runStop == true) {
        MySender->serialPort[buttonNumber-1]->write(0x85);
        MySender->serialPort[buttonNumber-1]->write(0x85);
        delay(100);
        runStop = false;
      }
    }
  }

  this->_setAcceleration(STEPPERS_ACCEL);
}


void GCodeCtrl::changeRockerCtrlSpeed(int changeSpeed) {
  _rockerControllerSpeed = changeSpeed;
}

void GCodeCtrl::changeSteppersSpeed(){
  MySender->bridgeSerial->println("Now Speed:" + String(_steppersSpeed));
  MySender->bridgeSerial->println("Send New Speed:");
  String bridgeCmd;
  float newSpeed;
  while (1) {
    //接受bridge数据
    bridgeCmd = "";
    while (MySender->bridgeSerial->available()) { 
      char tempChar = MySender->bridgeSerial->read();
      bridgeCmd += (char)tempChar;
      delay(10);
    }
    if (bridgeCmd != "") {
      String temp = bridgeCmd;
      temp.trim();
      if (temp == "q" || temp == "Q") {
        MySender->bridgeSerial->println(F("----Finished----"));
        break;
      }
      newSpeed = temp.toInt();
      _steppersSpeed = newSpeed;
      MySender->bridgeSerial->println("New Speed:" + String(_steppersSpeed));
      delay(50);
    }
  }
}

void GCodeCtrl::freeControl() {
  MySender->bridgeSerial->println(F("++++++Free Control Begin++++++"));
  MySender->bridgeSerial->println(F("Send window parameter:"));
  MySender->bridgeSerial->println(F("follow this format->(center X,center Y)(length,width)(direction,angle)"));
  MySender->bridgeSerial->println(F("CW=1, CCW=0"));
  char tempChar2;  //读取第一位
  bool quitOrNot = true; //true不退出，false退出
  float nextWindow[3][2]; //第一层：窗口中心，第二层：窗口长宽，第三层:窗口方向、角度
  while (1) {
    tempChar2 = "";
    char tempChar2;
    if (MySender->bridgeSerial->available()) {
      //读取第一位信息,'('读取数据，'q'退出，其他unkown
      tempChar2 = (char)MySender->bridgeSerial->read();
      delay(10);
      switch (tempChar2) {
        case '(':
            //读取剩余小数信息
            for (int k=0;k<3;k++) {
              for (int m=0;m<2;m++) {
                nextWindow[k][m] = MySender->bridgeSerial->parseFloat();
              }
            }
            //清空缓冲区
            while (MySender->bridgeSerial->available()) {
                MySender->bridgeSerial->read();
                delay(2);
            }
            //打印输入的窗口数据
            MySender->bridgeSerial->println("POS:("+String(nextWindow[0][0])+","+String(nextWindow[0][1])+")");
            MySender->bridgeSerial->println("SIZE:("+String(nextWindow[1][0])+","+String(nextWindow[1][1])+")");
            MySender->bridgeSerial->println("DIR ANGLE:("+String(nextWindow[2][0])+","+String(nextWindow[2][1])+")");

            //运行
            this->automaticArrival(nextWindow); 

            MySender->bridgeSerial->println("Free Control Ready!");
          break;
        case 'q': case 'Q':
          quitOrNot = false;
          break;
        case '!':
          this->MySender->stopFeeding();
          delay(500);
          break;
        default:
          MySender->bridgeSerial->println(F("Unknown command"));
          while (MySender->bridgeSerial->available()) {
              MySender->bridgeSerial->read();
              delay(2);
          }
          break;
      }
     
    }
    if (!quitOrNot) {
      MySender->bridgeSerial->println(F("----Free Control Finished----"));
      while (MySender->bridgeSerial->available()) {
          MySender->bridgeSerial->read();
          delay(2);
      }
      break;
    }
    
  }
}

void GCodeCtrl::dynamicMode(){
  MySender->bridgeSerial->println(F("+++++Dynamic Mode++++++"));
  delay(500);
  char tempChar2;  //读取第一位
  bool secLoopQuit = true; //true不退出，false退出
  bool mainLoopQuit = true;
  float nextWindow[3][2]; //第一层：窗口中心，第二层：窗口长宽，第三层:窗口方向、角度
  float homeWindow[3][2];
  char firstCmdChar;
  char secondCmdChar;
  while (1){
    MySender->bridgeSerial->println(F("Send target window:"));
    MySender->bridgeSerial->println(F("(center X,center Y)(length,width)(direction,angle)"));
    MySender->bridgeSerial->println(F("CW=1, CCW=0"));
    firstCmdChar = "";
    while(!MySender->bridgeSerial->available()){}
    if (MySender->bridgeSerial->available()) {
      //读取第一位信息,'('读取数据，'q'退出，其他unkown
      firstCmdChar = (char)MySender->bridgeSerial->read();
      delay(10);
      switch (firstCmdChar) {
        case '(':
            //读取剩余小数信息
            for (int k=0;k<3;k++) {
              for (int m=0;m<2;m++) {
                nextWindow[k][m] = MySender->bridgeSerial->parseFloat();
              }
            }
            //清空缓冲区
            while (MySender->bridgeSerial->available()) {
                MySender->bridgeSerial->read();
                delay(2);
            }
            //打印输入的窗口数据
            MySender->bridgeSerial->println("POS:("+String(nextWindow[0][0])+","+String(nextWindow[0][1])+")");
            MySender->bridgeSerial->println("SIZE:("+String(nextWindow[1][0])+","+String(nextWindow[1][1])+")");
            MySender->bridgeSerial->println("DIR ANGLE:("+String(nextWindow[2][0])+","+String(nextWindow[2][1])+")");
            if (DYNAMIC_TRIGGER_ON) {digitalWrite(DYNAMIC_TRIGGER_PIN,LOW);}
            //归位写入
            homeWindow[2][0]=0;
            homeWindow[2][1]=0;
            for (int i=0;i<2;i++) {
              for (int k=0;k<2;k++) {
                homeWindow[i][k]=nextWindow[i][k];
              }
            }
            this->automaticArrival(homeWindow); //窗口运行
            delay(3000);
            while (1){
              secLoopQuit = true;
              MySender->bridgeSerial->println("Ready For Next Trial...");
              delay(100);
              //等待串口触发
              if (KEY_CONFIRM_ON) {
                while(!MySender->bridgeSerial->available()){}
              }
              if (DYNAMIC_TRIGGER_ON) {digitalWrite(DYNAMIC_TRIGGER_PIN,HIGH);} //给声卡trigger
              MySender->bridgeSerial->println("Ready For Trigger...");
              //等待串口接收动作检测信息（检测到为0）
              if (MOTION_CAPTURE_TRIGGER_ON) {
                while(digitalRead(MOTION_CAPTURE_TRIGGER)==LOW){}
              }
              //等待光栅trigger
              if (RASTER_TRIGGER_ON) {
                while(analogRead(RASTER_TRIGGER_IN) < RASTER_TRIGGER_THRESHOLD){}
              }
              //运行
              //if (DYNAMIC_TRIGGER_ON) {digitalWrite(DYNAMIC_TRIGGER_PIN,HIGH);} //给声卡trigger
              this->automaticArrival(nextWindow); //窗口运行
              MySender->bridgeSerial->println("GO!");
              
              //清空缓冲区并等待指令输入
              MySender->bridgeSerial->println("(Waiting trial time(cmd:)...)");
              while (MySender->bridgeSerial->available()) {
                MySender->bridgeSerial->read();
                delay(2);
              }
              delay(TRIAL_TIME*1000);
              //检查串口是否有等待数据
              secondCmdChar = "";
              if (MySender->bridgeSerial->available()) {
                //读取第一位信息,'('读取数据，'q'退出，其他unkown
                secondCmdChar = (char)MySender->bridgeSerial->read();
                switch(secondCmdChar){
                  case 'q':
                    secLoopQuit = false;
                    break;
                  default:
                    MySender->bridgeSerial->println(F("Unknown command"));
                    break;
                }
              }
              if (DYNAMIC_TRIGGER_ON) {digitalWrite(DYNAMIC_TRIGGER_PIN,LOW);}
              //归位

              this->automaticArrival(homeWindow);
              
              if (!secLoopQuit){break;}
            }
          break;
        case 'Q':
          mainLoopQuit = false;
          break;
        default:
          MySender->bridgeSerial->println(F("Unknown command"));
          while (MySender->bridgeSerial->available()) {
              MySender->bridgeSerial->read();
              delay(2);
          }
          break;
      }
    
    }
    if (!mainLoopQuit) {
      MySender->bridgeSerial->println(F("----Dynamic Mode Finished----"));
      while (MySender->bridgeSerial->available()) {
          MySender->bridgeSerial->read();
          delay(2);
      }
      break;
    }
  }
}

void GCodeCtrl::dynamicMode2(){
  MySender->bridgeSerial->println(F("+++++Dynamic Mode 2++++++"));
  delay(500);
  char tempChar2;  //读取第一位
  bool secLoopQuit = true; //true不退出，false退出
  bool mainLoopQuit = true;
  float nextWindow[3][2]; //第一层：窗口中心，第二层：窗口长宽，第三层:窗口方向、角度
  float homeWindow[3][2];
  char firstCmdChar;
  char secondCmdChar;
  while (1){
    MySender->bridgeSerial->println(F("Send target window:"));
    MySender->bridgeSerial->println(F("(center X,center Y)(length,width)(0,0)"));
    MySender->bridgeSerial->println(F("CW=1, CCW=0"));
    firstCmdChar = "";
    while(!MySender->bridgeSerial->available()){}
    if (MySender->bridgeSerial->available()) {
      //读取第一位信息,'('读取数据，'q'退出，其他unkown
      firstCmdChar = (char)MySender->bridgeSerial->read();
      delay(10);
      switch (firstCmdChar) {
        case '(':
            //读取剩余小数信息
            for (int k=0;k<3;k++) {
              for (int m=0;m<2;m++) {
                nextWindow[k][m] = MySender->bridgeSerial->parseFloat();
              }
            }
            //清空缓冲区
            while (MySender->bridgeSerial->available()) {
                MySender->bridgeSerial->read();
                delay(2);
            }
            //打印输入的窗口数据
            MySender->bridgeSerial->println("POS:("+String(nextWindow[0][0])+","+String(nextWindow[0][1])+")");
            MySender->bridgeSerial->println("SIZE:("+String(nextWindow[1][0])+","+String(nextWindow[1][1])+")");
            MySender->bridgeSerial->println("DIR ANGLE:("+String(nextWindow[2][0])+","+String(nextWindow[2][1])+")");

            //归位写入
            MySender->bridgeSerial->println(F("Send home window:"));
            while(!MySender->bridgeSerial->available()){}
            for (int k=0;k<3;k++) {
              for (int m=0;m<2;m++) {
                homeWindow[k][m] = MySender->bridgeSerial->parseFloat();
              }
            }
            
            while (MySender->bridgeSerial->available()) {
                MySender->bridgeSerial->read();
                delay(2);
            }
            MySender->bridgeSerial->println("POS:("+String(homeWindow[0][0])+","+String(homeWindow[0][1])+")");
            MySender->bridgeSerial->println("SIZE:("+String(homeWindow[1][0])+","+String(homeWindow[1][1])+")");
            MySender->bridgeSerial->println("DIR ANGLE:("+String(homeWindow[2][0])+","+String(homeWindow[2][1])+")");

            this->automaticArrival(homeWindow); //窗口运行

            delay(3000);

            while (1){
              secLoopQuit = true;
              MySender->bridgeSerial->println("Ready For Next Trial...");
              delay(100);
              //等待串口触发
              if (KEY_CONFIRM_ON) {
                while(!MySender->bridgeSerial->available()){}
              }
              MySender->bridgeSerial->println("Ready For Trigger...");
              //等待串口接收动作检测信息（检测到为0）
              if (MOTION_CAPTURE_TRIGGER_ON) {
                while(digitalRead(MOTION_CAPTURE_TRIGGER)==LOW){}
              }
              //等待光栅trigger
              if (RASTER_TRIGGER_ON) {
                while(analogRead(RASTER_TRIGGER_IN) < RASTER_TRIGGER_THRESHOLD){}
              }
              //运行
              if (DYNAMIC_TRIGGER_ON) {digitalWrite(DYNAMIC_TRIGGER_PIN,HIGH);} //给声卡trigger
              //窗口运行
              this->automaticArrival(nextWindow);
              MySender->bridgeSerial->println("GO!");

              //清空缓冲区并等待指令输入
              MySender->bridgeSerial->println("(Waiting trial time(cmd:)...)");
              while (MySender->bridgeSerial->available()) {
                MySender->bridgeSerial->read();
                delay(2);
              }
              delay(TRIAL_TIME*1000);              
              //检查串口是否有等待数据
              secondCmdChar = "";
              if (MySender->bridgeSerial->available()) {
                //读取第一位信息,'('读取数据，'q'退出，其他unkown
                secondCmdChar = (char)MySender->bridgeSerial->read();
                switch(secondCmdChar){
                  case 'q':
                    secLoopQuit = false;
                    break;
                  default:
                    MySender->bridgeSerial->println(F("Unknown command"));
                    break;
                }
              }
              //归位
              this->automaticArrival(homeWindow);
              
              if (!secLoopQuit){break;}
            }
          break;
        case 'Q':
          mainLoopQuit = false;
          break;
        default:
          MySender->bridgeSerial->println(F("Unknown command"));
          while (MySender->bridgeSerial->available()) {
              MySender->bridgeSerial->read();
              delay(2);
          }
          break;
      }
    
    }
    if (!mainLoopQuit) {
      MySender->bridgeSerial->println(F("----Dynamic Mode Finished----"));
      while (MySender->bridgeSerial->available()) {
          MySender->bridgeSerial->read();
          delay(2);
      }
      break;
    }
  }
}

void GCodeCtrl::dynamicMode3(){
  MySender->bridgeSerial->println(F("+++++Dynamic Mode 3++++++"));
  delay(500);
  char tempChar2;  //读取第一位
  bool secLoopQuit = true; //true不退出，false退出
  bool mainLoopQuit = true;
  float nextWindow[3][2]; //第一层：窗口中心，第二层：窗口长宽，第三层:窗口方向、角度
  float homeWindow[3][2];
  int vertexNum;
  char firstCmdChar;
  char secondCmdChar;
  while (1){
    MySender->bridgeSerial->println(F("Warning:Dir must be different!"));
    MySender->bridgeSerial->println(F("Send target window:"));
    MySender->bridgeSerial->println(F("(c X,c Y)(l,w)(0,0)(ver)"));
    MySender->bridgeSerial->println(F("CW=1, CCW=0"));
    firstCmdChar = "";
    while(!MySender->bridgeSerial->available()){}
    if (MySender->bridgeSerial->available()) {
      //读取第一位信息,'('读取数据，'q'退出，其他unkown
      firstCmdChar = (char)MySender->bridgeSerial->read();
      delay(10);
      switch (firstCmdChar) {
        case '(':
            //读取剩余小数信息
            for (int k=0;k<3;k++) {
              for (int m=0;m<2;m++) {
                nextWindow[k][m] = MySender->bridgeSerial->parseFloat();
              }
            }
            vertexNum = MySender->bridgeSerial->parseInt();
            //清空缓冲区
            while (MySender->bridgeSerial->available()) {
                MySender->bridgeSerial->read();
                delay(2);
            }
            //打印输入的窗口数据
            MySender->bridgeSerial->print(F("POS:("));
            MySender->bridgeSerial->print(nextWindow[0][0]);
            MySender->bridgeSerial->print(",");
            MySender->bridgeSerial->print(nextWindow[0][1]);
            MySender->bridgeSerial->println(")");
            MySender->bridgeSerial->print(F("SIZE:("));
            MySender->bridgeSerial->print(nextWindow[1][0]);
            MySender->bridgeSerial->print(",");
            MySender->bridgeSerial->print(nextWindow[1][1]);
            MySender->bridgeSerial->println(")");
            MySender->bridgeSerial->print(F("DIR ANGLE:("));
            MySender->bridgeSerial->print(nextWindow[2][0]);
            MySender->bridgeSerial->print(",");
            MySender->bridgeSerial->print(nextWindow[2][1]);
            MySender->bridgeSerial->println(")");
            MySender->bridgeSerial->print(F("VERTEX = "));
            MySender->bridgeSerial->print(vertexNum);
            
            //归位写入
            MySender->bridgeSerial->println(F("Send home window:"));
            while(!MySender->bridgeSerial->available()){}
            for (int k=0;k<3;k++) {
              for (int m=0;m<2;m++) {
                homeWindow[k][m] = MySender->bridgeSerial->parseFloat();
              }
            }
            
            while (MySender->bridgeSerial->available()) {
                MySender->bridgeSerial->read();
                delay(2);
            }
            
            MySender->bridgeSerial->print(F("POS:("));
            MySender->bridgeSerial->print(homeWindow[0][0]);
            MySender->bridgeSerial->print(",");
            MySender->bridgeSerial->print(homeWindow[0][1]);
            MySender->bridgeSerial->println(")");
            MySender->bridgeSerial->print(F("SIZE:("));
            MySender->bridgeSerial->print(homeWindow[1][0]);
            MySender->bridgeSerial->print(",");
            MySender->bridgeSerial->print(homeWindow[1][1]);
            MySender->bridgeSerial->println(")");
            MySender->bridgeSerial->print(F("DIR ANGLE:("));
            MySender->bridgeSerial->print(homeWindow[2][0]);
            MySender->bridgeSerial->print(",");
            MySender->bridgeSerial->print(homeWindow[2][1]);
            MySender->bridgeSerial->println(")");
            MySender->bridgeSerial->print(homeWindow[2][1]);
            
            //归位
            bool dirTemp = homeWindow[2][0];
            float angleTemp = homeWindow[2][1];
            homeWindow[2][0] = 0;
            homeWindow[2][1] = 0;

            this->automaticArrival(homeWindow); 

            homeWindow[2][0] = dirTemp;
            homeWindow[2][1] = angleTemp;
            this->runArcToTarget(homeWindow[0],homeWindow[1],vertexNum,homeWindow[2][0],homeWindow[2][1]);  //窗口归位

            delay(3000);


            while (1){
              secLoopQuit = true;
              MySender->bridgeSerial->println("Ready For Next Trial...");
              delay(100);
              //等待串口触发
              while(!MySender->bridgeSerial->available()){}
              //运行
              MySender->bridgeSerial->println("GO!");
              //if (DYNAMIC_TRIGGER_ON) {digitalWrite(DYNAMIC_TRIGGER_PIN,HIGH);} //给声卡trigger

              //窗口运行
              for (int i=0;i<2;i++) {
                this->runArcToTarget(nextWindow[0],nextWindow[1],vertexNum,nextWindow[2][0],nextWindow[2][1]); 
                //this->MySender->delayCmd(0.5);
                delay(1000);
                this->runArcToTarget(homeWindow[0],homeWindow[1],vertexNum,homeWindow[2][0],homeWindow[2][1]); 
                //this->MySender->delayCmd(0.5);
                delay(1000);
              }

              //清空缓冲区并等待指令输入
              MySender->bridgeSerial->println("(Waiting trial time(cmd:)...)");
              while (MySender->bridgeSerial->available()) {
                MySender->bridgeSerial->read();
                delay(2);
              }
              delay(TRIAL_TIME*1000);
              //检查串口是否有等待数据
              secondCmdChar = "";
              if (MySender->bridgeSerial->available()) {
                //读取第一位信息,'('读取数据，'q'退出，其他unkown
                secondCmdChar = (char)MySender->bridgeSerial->read();
                switch(secondCmdChar){
                  case 'q':
                    secLoopQuit = false;
                    break;
                  default:
                    MySender->bridgeSerial->println(F("Unknown command"));
                    break;
                }
              }
              //归位

              // this->automaticArrival(homeWindow);
              
              if (!secLoopQuit){break;}
            }
          break;
        case 'Q':
          mainLoopQuit = false;
          break;
        default:
          MySender->bridgeSerial->println(F("Unknown command"));
          while (MySender->bridgeSerial->available()) {
              MySender->bridgeSerial->read();
              delay(2);
          }
          break;
      }
    
    }
    if (!mainLoopQuit) {
      MySender->bridgeSerial->println(F("----Dynamic Mode Finished----"));
      while (MySender->bridgeSerial->available()) {
          MySender->bridgeSerial->read();
          delay(2);
      }
      break;
    }
  }
}

void GCodeCtrl::sleep(float sleepPos[4][2]){
  MySender->bridgeSerial->println(F("正在关机.."));

  //世界坐标系转化为输出坐标系
  float outPutPos[4][2];
  float *ptrOutPutPos = this->_coordinateRebuild(sleepPos);
  for (int i=0;i<4;i++) {  
    for (int k=0;k<2;k++) {
      outPutPos[i][k] = *ptrOutPutPos;
      ptrOutPutPos++;
    }
  }
  //cm转mm
  for (int i=0;i<4;i++) {
    for (int k=0;k<2;k++) {
      outPutPos[i][k] = this->_unitConversionToMm(outPutPos[i][k]);
    }
  }
  MySender->sendStraightCmd(outPutPos, _steppersSpeed*2/3);

  for (int i=0;i<4;i++) {
    for (int k=0;k<2;k++) {
      _currentCoord[i][k] = sleepPos[i][k];
    }
  }
  this->_eepromWrite();
  
  MySender->grblSleep();
  
  
}


////////////////////////PRIVATE////////////////////////

//计算所有源坐标：输入矩形中心点和长宽，输出所有源坐标的首地址
float *GCodeCtrl::_calculateEveryCoord(float WinCenter[2], float WinSize[2]) {  
  
  _calculateEveryCoordReturn[0] = WinCenter[0] - WinSize[0]/2;
  _calculateEveryCoordReturn[1] = WinCenter[1] - WinSize[1]/2;
  _calculateEveryCoordReturn[2] = WinCenter[0] + WinSize[0]/2;
  _calculateEveryCoordReturn[3] = WinCenter[1] - WinSize[1]/2; 
  _calculateEveryCoordReturn[4] = WinCenter[0] + WinSize[0]/2;
  _calculateEveryCoordReturn[5] = WinCenter[1] + WinSize[1]/2;
  _calculateEveryCoordReturn[6] = WinCenter[0] - WinSize[0]/2;
  _calculateEveryCoordReturn[7] = WinCenter[1] + WinSize[1]/2; 
  
  return _calculateEveryCoordReturn;
}

float GCodeCtrl::_unitConversionToMm(float cmLong){
  float mmLong = cmLong*10;
  return mmLong;  
}

float *GCodeCtrl::_coordinateRebuild(float oriAllPos[4][2]){
  //I型坐标系，无需重建
  _coordinateRebuildReturn[0] = oriAllPos[0][0];
  _coordinateRebuildReturn[1] = oriAllPos[0][1];
  _coordinateRebuildReturn[2] = oriAllPos[1][0];
  _coordinateRebuildReturn[3] = oriAllPos[1][1];
  //II型坐标系重建
  _coordinateRebuildReturn[4] = _stepsParameter[0][0] - oriAllPos[2][0];//x3
  _coordinateRebuildReturn[5] = _stepsParameter[0][0] - oriAllPos[2][1];//y3
  _coordinateRebuildReturn[6] = _stepsParameter[0][0] - oriAllPos[3][0];//x4
  _coordinateRebuildReturn[7] = _stepsParameter[0][0] - oriAllPos[3][1];//y4

  return _coordinateRebuildReturn;
}

void GCodeCtrl::_eepromClean(){
  #ifdef serialDebug
  MySender->bridgeSerial->println(F("eepromClean,begin"));
  #endif
  for (int i =0; i <EEPROM.length(); i++) {
    EEPROM.write(i,0);
  }
  #ifdef serialDebug
  MySender->bridgeSerial->println(F("eepromClean, finished"));
  #endif
}
void GCodeCtrl::_eepromWrite(){ 
  //从第9位开始写，向eeprom写入
  #ifdef serialDebug
  MySender->bridgeSerial->println(F("eepromWrite, begin"));
  #endif
  //写入eeprom所有电机的位置
  int eepromAddress = 9;
  //写入所有电机的坐标位置
  for (int i=0;i<4;i++) {
    for (int k=0;k<2;k++) {
      EEPROM.put(eepromAddress, _currentCoord[i][k]);
      eepromAddress += 4;
    }
  }
  //写入窗口状态
  for (int i=0;i<3;i++) {
    for (int k=0;k<2;k++) {
      EEPROM.put(eepromAddress, _currentWinData[i][k]);
      eepromAddress += 4;
    }
  }

  #ifdef serialDebug
  for(int i=0;i<4;i++){
    for(int k=0;k<2;k++){
      MySender->bridgeSerial->print(_currentCoord[i][k]);
      MySender->bridgeSerial->print(" ");
    }
    MySender->bridgeSerial->println("");    
  }

  for(int i=0;i<3;i++){
    for(int k=0;k<2;k++){
      MySender->bridgeSerial->print(_currentWinData[i][k]);
      MySender->bridgeSerial->print(" ");
    }
    MySender->bridgeSerial->println("");
  }
  MySender->bridgeSerial->println(F("eepromWrite,finished"));
  #endif
}
void GCodeCtrl::_eepromRead(){
  //从eeprom获取数据，并写入到current中
  #ifdef serialDebug
  MySender->bridgeSerial->println(F("eepromRead,begin"));
  #endif
  int eepromAddress = 9;
  float tempNum;
  for (int i=0;i<4;i++) {
    for (int k=0;k<2;k++) {
      EEPROM.get(eepromAddress, tempNum);
      _currentCoord[i][k] = tempNum;
      eepromAddress+= 4;
    }
  }
  for (int i=0;i<3;i++) {
    for (int k=0;k<2;k++) {
      EEPROM.get(eepromAddress, tempNum);
      _currentWinData[i][k] = tempNum;
      eepromAddress+= 4;
    }
  }
  #ifdef serialDebug
  for(int i=0;i<4;i++){
    for(int k=0;k<2;k++){
      MySender->bridgeSerial->print(_currentCoord[i][k]);
      MySender->bridgeSerial->print(" ");
    }
    MySender->bridgeSerial->println("");    
  }

  for(int i=0;i<3;i++){
    for(int k=0;k<2;k++){
      MySender->bridgeSerial->print(_currentWinData[i][k]);
      MySender->bridgeSerial->print(" ");
    }
    MySender->bridgeSerial->println("");
  }
  MySender->bridgeSerial->println(F("eepromWrite,finished"));
  #endif

}

void GCodeCtrl::_bubbleSort(float numArr[], int n){
  for (int i = 0; i < n-1; i++) {
    for (int j = 0; j < n-i-1; j++) {
      // 如果当前元素大于下一个元素，则交换它们
      if (numArr[j] > numArr[j+1]) {
        float temp = numArr[j];
        numArr[j] = numArr[j+1];
        numArr[j+1] = temp;
      }
    }
  }
}

void GCodeCtrl::_setAcceleration(float accel){
  for (int i=0;i<4;i++){
    MySender->grblsArray[i]->currentCmd = "$120="+String(accel)+"\n";
  }
  MySender->sendNowWithoutBack();
  delay(50);
  for (int i=0;i<4;i++){
    MySender->grblsArray[i]->currentCmd = "$121="+String(accel)+"\n";
  }
  MySender->sendNowWithoutBack();
  delay(50);
}