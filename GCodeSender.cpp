#include "WString.h"

#include "GCodeSender.h"


GCodeSender::GCodeSender () {
}

GCodeSender::~GCodeSender () {

}

void GCodeSender::serialBegin() {
  delay(10);
  for (int i=0;i<4;i++) {
    serialPort[i]->begin(GRBL_BAUD_RATE);
    serialPort[i]->setTimeout(SERIAL_TIMEOUT);
  }
  bridgeSerial->begin(BRIDGE_BAUD_RATE);
  bridgeSerial->setTimeout(SERIAL_TIMEOUT);

  //triggerSerial->begin(TIGGER_BAUD_RATE);
  //triggerSerial->begin(SERIAL_TIMEOUT);

  if (IS_ESTABLISH_CONTACT){
    this->_establishContact();
  }
}


#ifdef voidDebug
void GCodeSender::Debug () {

}
#endif

void GCodeSender::serialClean () {
  _serialClean();
}


void GCodeSender::computerCtrl_Universal () {
  bridgeSerial->println(F("++++++Universal Control Begin++++++"));
  bridgeSerial->println(F("write your command:"));
  String bridgeCmd = "";
  while (1) {
    //接受bridge数据
    if (bridgeSerial->available()) { 
      bridgeCmd = bridgeSerial->readString();
      delay(10);

      String temp = bridgeCmd;
      temp.trim();
      if (temp == "q" || temp == "Q") {
        this->_serialClean();
        bridgeSerial->println(F("----Universal Control Finished----"));
        break;
      }
      bridgeSerial->println("<-TX:\n" + bridgeCmd);
      for (int i=0;i<4;i++) {
        grblsArray[i]->currentCmd = bridgeCmd + "\n";
      }
      //发送指令
      serialPort[0]->print(grblsArray[0]->currentCmd);
      serialPort[1]->print(grblsArray[1]->currentCmd);
      serialPort[2]->print(grblsArray[2]->currentCmd);
      serialPort[3]->print(grblsArray[3]->currentCmd);
    }
    
    //接收grbl数据
    if (serialPort[0]->available() || serialPort[1]->available() || serialPort[2]->available() || serialPort[3]->available()) {
      String grblRec[4];
      for (int i = 0; i < 4; i++){
        grblRec[i] = serialPort[i]->readString();
      }
      for (int i = 0; i < 4; i++){
        bridgeSerial->println("->RX:"+String(i+1)+"\n" + grblRec[i]);
      }
      bridgeSerial->println(F("----RX End----"));
    }
    
  }
  
}

void GCodeSender::computerCtrl_Specific(){
  bridgeSerial->println(F("++++++Specific Control Begin++++++"));
  char firstChar;
  bool isQiut = false;
  while (1) {
    bridgeSerial->println(F("choose the grbl:(send num 1~4)"));
    int grblNum = 0;
    while (!bridgeSerial->available()) {}
    if (bridgeSerial->available()) { 
      firstChar = bridgeSerial->read();
      grblNum = firstChar - '0' - 1; //ASCII码相减得到整数值
      this->_serialClean(0);
      if (-1 < grblNum && grblNum < 4) {
        bridgeSerial->print(F("<-grbl_"));
        bridgeSerial->println(grblNum+1);
        while (1) {
          // read from computer, send to grbl
          if (bridgeSerial->available()) {
            String bridgeCmd = bridgeSerial->readString();
            if (bridgeCmd == "q") {
              bridgeSerial->println();
              break;
            } else if (bridgeCmd == "Q") {
              isQiut = true;
              break;
            }
            bridgeSerial->println("<-TX:\n" + bridgeCmd);
            serialPort[grblNum]->print(bridgeCmd+"\n");
          }
          
          // read from grbl, send to computer
          if (serialPort[grblNum]->available()) {
            String grblRec = serialPort[grblNum]->readString();
            bridgeSerial->println("->RX:\n" + grblRec + "----RX End----");
          }
        }
        this->_serialClean();
      } else if (firstChar == 'Q') {
        isQiut = true;
      } else {
        bridgeSerial->println(F("Unknown Command"));
        bridgeSerial->println(grblNum);
        this->_serialClean(0);
      }
    }
    if (isQiut) {
      this->_serialClean();
      bridgeSerial->println(F("----Specific Control Finished----"));
      break;
    }
  }
}

void GCodeSender::computerCtrl_Batch(){
  bridgeSerial->println(F("++++++Batch Control Begin++++++"));
  bridgeSerial->println(F("Write your cmd:"));
  delay(300);
  bridgeSerial->println(F("(follow this format->Grbl_Number + : + Your_Command)"));
  this->_serialClean();
  String bridgeCmd = "";
  String tempStr = "";  //判断是否quit
  bool quitOrNot = true; //true不退出，false退出
  while (1) {
    bridgeCmd = "";
    char tempChar2;
    if (bridgeSerial->available()) {
      //读取第一位数字信息
      tempChar2 = bridgeSerial->read();
      delay(10);
      //去除第二位冒号
      bridgeSerial->read();
      delay(10);
      //读取剩余信息
      while (bridgeSerial->available()) { 
        char tempChar3 = bridgeSerial->read();
        bridgeCmd += (char)tempChar3;
        delay(10);
      }
      
      switch (tempChar2) {
        case '1':
          grblsArray[0]->currentCmd = bridgeCmd +"\n";
          bridgeSerial->print(F("grbl_1: command saved"));
          break;
        case '2':
          grblsArray[1]->currentCmd = bridgeCmd+"\n";
          bridgeSerial->print(F("grbl_2: command saved"));
          break;
        case '3':
          grblsArray[2]->currentCmd = bridgeCmd+"\n";
          bridgeSerial->print(F("grbl_3: command saved"));
          break;
        case '4':
          grblsArray[3]->currentCmd = bridgeCmd+"\n";
          bridgeSerial->print(F("grbl_4: command saved"));
          break;
        case 's': case 'S':
          this->sendNow();
          break;
        case 'q': case 'Q':
          quitOrNot = false;
          break;
        default:
          break;
      }
    }
    if (!quitOrNot) {
      bridgeSerial->println(F("----Batch Control Finished----"));
      this->_serialClean();
      break;
    }
  }

}

bool GCodeSender::sendStraightCmd(float allPos[4][2], float straightSpeed){
  String straightCmd = "";
  #ifdef serialDebug
  bridgeSerial->println(F("Straight Cmd:"));
  #endif
  for (int i=0;i<4;i++) {
    straightCmd = "G01 X" + String(allPos[i][0]) + "Y" + String(allPos[i][1]) + "F" + String(straightSpeed) + "\n";
    this->grblsArray[i]->currentCmd = straightCmd;
    #ifdef serialDebug
    bridgeSerial->print(i+1);
    bridgeSerial->print(":");
    String temp = straightCmd;
    temp.replace("\n", "");
    bridgeSerial->println(temp);
    #endif
    delay(SENDING_DELAY_TIME);
  }

  bool temp = this->sendNowWithoutBack();

  delay(SENDING_DELAY_TIME);
  return temp;
}

bool GCodeSender::sendArcCmd(float allArcPos[4][2], float winCenterPos[4][2], float winOriPos[4][2], bool direction, float arcSpeed){
  //(输出坐标系的目的坐标，旋转锚点坐标，窗口起始坐标，方向，旋转速度)
  String arcCmd = "";
  String directionCmd[2] = {"G2", "G3"};  //false = 0 = 逆时针 = G2, true = 1 = 顺时针 = G3，在grbl的笛卡尔坐标系中要反向
  float IJ[4][2];  //x\y相对于圆心的偏移量
  for (int i=0;i<4;i++) {
    for (int k=0;k<2;k++) {
      IJ[i][k] = winCenterPos[i][k] - winOriPos[i][k];
    }
  }
  #ifdef serialDebug
  bridgeSerial->println(F("arc Cmd:"));
  #endif
  for (int i=0;i<4;i++) {
    //G2 X Y I J F
    arcCmd = directionCmd[direction]+"X"+String(allArcPos[i][0])+"Y"+String(allArcPos[i][1])+"I"+String(IJ[i][0])+"J"+String(IJ[i][1])+"F"+String(arcSpeed)+"\n";
    this->grblsArray[i]->currentCmd = arcCmd;
    #ifdef serialDebug
    bridgeSerial->print(i+1);
    bridgeSerial->print(":");
    String temp = arcCmd;
    temp.replace("\n", "");
    bridgeSerial->println(temp);
    #endif
    delay(SENDING_DELAY_TIME);
  }

  bool temp = this->sendNowWithoutBack();

  delay(SENDING_DELAY_TIME);
  return temp;

}

bool GCodeSender::sendAnchorArcCmd(float allArcPos[4][2], float winCenterPos[4][2], float winOriPos[4][2], bool direction, float arcSpeed[4]){
  //(输出坐标系的目的坐标，旋转锚点坐标，窗口起始坐标，方向，最大旋转速度)
  String arcCmd = "";
  String directionCmd[2] = {"G2", "G3"};  //false = 0 = 逆时针 = G2, true = 1 = 顺时针 = G3，在grbl的笛卡尔坐标系中要反向
  float IJ[4][2];  //x\y相对于圆心的偏移量
  for (int i=0;i<4;i++) {
    for (int k=0;k<2;k++) {
      IJ[i][k] = winCenterPos[i][k] - winOriPos[i][k];
    }
  }

  #ifdef serialDebug
  bridgeSerial->println(F("arc Cmd:"));
  #endif
  for (int i=0;i<4;i++) {
    //G2 X Y I J F
    arcCmd = directionCmd[direction]+"X"+String(allArcPos[i][0])+"Y"+String(allArcPos[i][1])+"I"+String(IJ[i][0])+"J"+String(IJ[i][1])+"F"+String(arcSpeed[i])+"\n";
    this->grblsArray[i]->currentCmd = arcCmd;
    #ifdef serialDebug
    bridgeSerial->print(i+1);
    bridgeSerial->print(":");
    String temp = arcCmd;
    temp.replace("\n", "");
    bridgeSerial->println(temp);
    #endif
    delay(SENDING_DELAY_TIME);
  }

  bool temp = this->sendNowWithoutBack();

  delay(SENDING_DELAY_TIME);
  return temp;

}

bool GCodeSender::sendNow(){
  if (_senderSequence) {
    delay(SENDING_DELAY_TIME);
    serialPort[0]->print(grblsArray[0]->currentCmd);
    serialPort[1]->print(grblsArray[1]->currentCmd);
    serialPort[2]->print(grblsArray[2]->currentCmd);
    serialPort[3]->print(grblsArray[3]->currentCmd);
    delay(SENDING_DELAY_TIME);
    _senderSequence = false;
  } else {
    delay(SENDING_DELAY_TIME);
    serialPort[3]->print(grblsArray[3]->currentCmd);
    serialPort[2]->print(grblsArray[2]->currentCmd);
    serialPort[1]->print(grblsArray[1]->currentCmd);
    serialPort[0]->print(grblsArray[0]->currentCmd);
    delay(SENDING_DELAY_TIME);
    _senderSequence = true;
  }
  
  bool returnState;
  String returnString[4];
  for (int i=0; i<4; i++) {
    while (serialPort[i]->available()) { 
      char tempChar = serialPort[i]->read();
      returnString[i] += (char)tempChar;
      delay(10);
    }
    returnString[i].trim();
  }

  bool judgeErrorOrNot = true; 
  for (int i=0; i<4; i++) {
    if (returnString[i] != "ok") {
      bridgeSerial->print(F("->error"));
      bridgeSerial->print(i+1);
      judgeErrorOrNot = false;
    }  
  }
  if (judgeErrorOrNot) {
    //全部ok，覆写lastCmd，刷新currentCmd
    bridgeSerial->println(F("->ok"));
    for (int i=0;i<4;i++) {
      grblsArray[i]->lastCmd = grblsArray[i]->currentCmd;
      grblsArray[i]->currentCmd = "";
    }
    this->_serialClean();
    returnState = true;
  } else if (!judgeErrorOrNot) {
    //存在error，输出上一指令，不刷新lastCmd
    bridgeSerial->println(F("Back to last command..."));
    delay(50);
    serialPort[0]->print(grblsArray[0]->lastCmd);
    serialPort[1]->print(grblsArray[1]->lastCmd);
    serialPort[2]->print(grblsArray[2]->lastCmd);
    serialPort[3]->print(grblsArray[3]->lastCmd);
    delay(50); 
    bridgeSerial->println(F("ok"));
    this->_serialClean();
    returnState = false;
  }
  return returnState;

}

bool GCodeSender::sendNowWithoutBack() {
  // this->_serialClean();
  if (_senderSequence) {
    delay(SENDING_DELAY_TIME);
    serialPort[0]->print(grblsArray[0]->currentCmd);
    serialPort[1]->print(grblsArray[1]->currentCmd);
    serialPort[2]->print(grblsArray[2]->currentCmd);
    serialPort[3]->print(grblsArray[3]->currentCmd);
    delay(SENDING_DELAY_TIME);
    _senderSequence = false;
  } else {
    delay(SENDING_DELAY_TIME);
    serialPort[3]->print(grblsArray[3]->currentCmd);
    serialPort[2]->print(grblsArray[2]->currentCmd);
    serialPort[1]->print(grblsArray[1]->currentCmd);
    serialPort[0]->print(grblsArray[0]->currentCmd);
    delay(SENDING_DELAY_TIME);
    _senderSequence = true;
  }
  // this->_serialClean(); 
  return true;
}

bool GCodeSender::sendNowAutoRetry(){
  bool error = true;
  bool returnState;
  for (int m=0;m<3;m++){
    this->_serialClean();
    delay(50);
    serialPort[0]->print(grblsArray[0]->currentCmd);
    serialPort[1]->print(grblsArray[1]->currentCmd);
    serialPort[2]->print(grblsArray[2]->currentCmd);
    serialPort[3]->print(grblsArray[3]->currentCmd);
    delay(500);
    
    String returnString[4];
    for (int i=0; i<4; i++) {
      while (serialPort[i]->available()) { 
        char tempChar = serialPort[i]->read();
        returnString[i] += (char)tempChar;
        delay(10);
      }
      returnString[i].trim();
    }
    bool judgeErrorOrNot = true; 
    for (int i=0; i<4; i++) {
      if (returnString[i] != "ok") {
        bridgeSerial->print(F("->error"));
        bridgeSerial->print(i+1);
        judgeErrorOrNot = false;
      }  
    }

    if (judgeErrorOrNot) {
      //全部ok，覆写lastCmd，刷新currentCmd
      bridgeSerial->println(F("->ok"));
      for (int i=0;i<4;i++) {
        grblsArray[i]->lastCmd = grblsArray[i]->currentCmd;
        grblsArray[i]->currentCmd = "";
      }
      error = true;
      this->_serialClean();
      returnState = true;
      break;

    } else if (!judgeErrorOrNot) {
      //存在error，重试
      bridgeSerial->println(F("Retry..."));
      error = false;
     
    }
    delay(2000);
  }
  //如果三次重试没成功，返回上个指令
  if (!error) {
    bridgeSerial->println(F("Back to last command..."));
    delay(50);
    serialPort[0]->print(grblsArray[0]->lastCmd);
    serialPort[1]->print(grblsArray[1]->lastCmd);
    serialPort[2]->print(grblsArray[2]->lastCmd);
    serialPort[3]->print(grblsArray[3]->lastCmd);
    delay(50); 
    for (int i=0;i<4;i++) {
      grblsArray[i]->currentCmd = "";
    }
    bridgeSerial->println(F("Check your cmd."));
    this->_serialClean();
    returnState = false;
  }
  return returnState;
}

void GCodeSender::stopFeeding(){
  bridgeSerial->println(F("STOP!"));
  delay(50);
  serialPort[0]->print("!\n");
  serialPort[1]->print("!\n");
  serialPort[2]->print("!\n");
  serialPort[3]->print("!\n");
  delay(100); 
  serialPort[0]->print("!\n");
  serialPort[1]->print("!\n");
  serialPort[2]->print("!\n");
  serialPort[3]->print("!\n");
  delay(50); 
  this->_serialClean();
}

void GCodeSender::_establishContact(){
  bridgeSerial->println(F("正在初始化..."));
  unsigned long startTime;
  unsigned long myTime;
  for (int i=0;i<4;i++) {
    startTime = millis();
    bool note1 = false;
    bool note2 = false;
    while (!serialPort[i]->find("ok")) {
      this->_serialClean(i);
      serialPort[i]->print("\n");
      myTime = millis() - startTime;
      if (CONTACT_TIMEOUT/4 < myTime && myTime < CONTACT_TIMEOUT && !note1) {
          bridgeSerial->println("请连接grbl " + String(i+1));
          note1 = true;
          note2 = true;
      } else if (CONTACT_TIMEOUT < myTime) {
        bridgeSerial->println("grbl " + String(i+1) + "连接超时");
        note2 = false;
        break;
      }
      delay(100);
    }
    if (note2) {
      bridgeSerial->println(F("已连接!"));
    }
  }
  this->_serialClean();
  bridgeSerial->println(F("完成"));
}

void GCodeSender::_serialClean(){
  for (int i=0;i<4;i++) {
    while (serialPort[i]->available()) {
      serialPort[i]->read();
      delay(5);
    }
  }
  while (bridgeSerial->available()) {
    bridgeSerial->read();
    delay(5);
  }
}

void GCodeSender::_serialClean(int witchPort){
  switch (witchPort) {
    case 0:
      while (bridgeSerial->available()) {
        bridgeSerial->read();
        delay(5);
      }
      break;
    default:
      witchPort -= 1;
      while (serialPort[witchPort]->available()) {
        serialPort[witchPort]->read();
        delay(5);
      }
      break;
  }
}

void GCodeSender::delayCmd(float delayTimeSec){
  String delayCmd = "";
  #ifdef serialDebug
  bridgeSerial->println(F("Delay Time Cmd:"));
  #endif
  for (int i=0;i<4;i++) {
    delayCmd = "G4 P"+String(delayTimeSec)+"\n";
    this->grblsArray[i]->currentCmd = delayCmd;

    #ifdef serialDebug
    bridgeSerial->print(i+1);
    bridgeSerial->print(":");
    String temp = delayCmd;
    temp.replace("\n", "");
    bridgeSerial->println(temp);
    #endif
    delay(10);
  }
  this->sendNowWithoutBack();
  delay(10);
}

bool GCodeSender::grblOKState(int grblIndex) {
  if (serialPort[grblIndex]->find("ok")) {
    this->_serialClean(grblIndex);
    return true;
  } else {
    this->_serialClean(grblIndex);
    return false;
  }
}


bool GCodeSender::isGrblStepping(int grblIndex) {

  serialPort[grblIndex]->print("?");

  while(!serialPort[grblIndex]->available() > 4){}
  
  String res = serialPort[grblIndex]->readString();

  if (res.indexOf("Run") != -1) {
    return true;
  } else {
    return false;
  }

}

bool GCodeSender::waitGrblOK(int grblIndex, String cmd, long timeOut){
  unsigned long startTime = millis();
  unsigned long myTime;
  while (!serialPort[grblIndex]->find("ok")) {
    this->_serialClean(grblIndex);
    serialPort[grblIndex]->print(cmd);
    myTime = millis() - startTime;
    if (timeOut < myTime) {
      return false;
    }
    delay(50);
  }
  return true;
}

void  GCodeSender::grblSleep(){
  this->_serialClean();
  bool isSleep = true;
  unsigned long startTime;
  unsigned long myTime;
  for (int i=0; i<4; i++) {
    startTime = millis();
    while (!serialPort[i]->find("ok")) {
      this->_serialClean(i);
      serialPort[i]->print("$SLP\n");
      myTime = millis() - startTime;
      if (60000 < myTime) {
        isSleep = false;
      }
      delay(1000);
    }
  }
  if (isSleep){
    bridgeSerial->println(F("已关机"));
    while(1){delay(20000);}
  } else{
    bridgeSerial->println(F("关机失败，电机可能未归位"));
  }
}
