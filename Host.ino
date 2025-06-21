
        /////////////////////////////////////
//////////////////// Commander 4.4 Pro ////////////////////
      /////////////////////////////////////

//4.1 motor without 'any' delay
//4.2 Pro 添加了窗口绕顶点旋转功能
//4.4 dynamicMode2 改为动捕触发
#include "config.h"

String cmd;  //储存指令

void setup() {
  myController.serialBegin(4800, 115200);
  //显示屏setup
  u8g.setFont(u8g_font_gdr30r);
  u8g.setScale2x2();		
  u8g.setFontPosTop();
  u8g.setRot180(); //调转180度
  analogWrite(SCREEN_LED_PIN, SCREEN_LED_BRIGHTNESS); //亮度设置
  delay(100);
  display_1();

}

void loop() { 
  if (myController.MySender->bridgeSerial->available()) {
    cmd = (char)myController.MySender->bridgeSerial->read();
    cmd += (char)myController.MySender->bridgeSerial->read();
    while (myController.MySender->bridgeSerial->available()) { //读取2个字节数据，其他数据清除
      myController.MySender->bridgeSerial->read();
      delay(5);
    }
    cmd.trim();
    char cmdChar = cmd.charAt(0);
    int cmdInt;
    myController.MySender->bridgeSerial->println(cmd);
    switch (cmdChar) {
      case '1': case '2': case '3': case '4': case '5': case '6': case '7': case '8': case '9': case '10': case '11': case '12':
        cmdInt=cmd.toInt();
        cmdInt -= 1;
        display_2(windows[cmdInt][0], windows[cmdInt][1]); 
        //移动到窗口
        myController.automaticArrival(windows[cmdInt]); 
        delay(1000);
        break;
      case 'c': case 'C':
        tone(SPEAKER_PIN,2000, 50);
        delay(100);
        tone(SPEAKER_PIN,2000, 50);
        myController.rockerController();
        tone(SPEAKER_PIN,2000, 50);
        delay(500);
        break;
      case 'u': case 'U':
        tone(SPEAKER_PIN,2000, 50);
        delay(100);
        tone(SPEAKER_PIN,2000, 50);
        myController.MySender->computerCtrl_Universal();
        tone(SPEAKER_PIN,2000, 50);
        delay(500);
        break;
      case 's': case 'S':
        tone(SPEAKER_PIN,2000, 50);
        delay(100);
        tone(SPEAKER_PIN,2000, 50);
        myController.MySender->computerCtrl_Specific();
        tone(SPEAKER_PIN,2000, 50);
        delay(500);
        break;
      case 'b': case 'B':
        tone(SPEAKER_PIN,2000, 50);
        delay(100);
        tone(SPEAKER_PIN,2000, 50);
        myController.MySender->computerCtrl_Batch();
        tone(SPEAKER_PIN,2000, 50);
        delay(500);
        break;
      case '$': 
        display_3();
        break;
      case '!': //紧急停止
        tone(SPEAKER_PIN,1000, 1000);
        myController.MySender->stopFeeding();
        delay(500);
        break;
      case 'n': case 'N': //数字显示
        tone(SPEAKER_PIN,2000, 50);
        drawTrialTimes();
        delay(500);
        break;
      case 'v': case 'V'://速度设置
        tone(SPEAKER_PIN,2000, 50);
        changeSpeed();
        delay(500);
        break;
      case 'f': case 'F': //自由控制
        tone(SPEAKER_PIN,2000, 50);
        myController.freeControl();
        tone(SPEAKER_PIN,2000, 50);
        delay(500);
        break;
      case 'd': case 'D': //动态实验模式（窗口绕中心点旋转）
        tone(SPEAKER_PIN,2000, 50);
        myController.dynamicMode();
        tone(SPEAKER_PIN,2000, 50);
        delay(500);
        break;
      case 'e': case 'E': //动态实验模式（窗口平移）
        tone(SPEAKER_PIN,2000, 50);
        myController.dynamicMode2();
        tone(SPEAKER_PIN,2000, 50);
        delay(500);
        break;
      case 'g': case 'G': //动态实验模式（窗口绕顶点旋转）
        tone(SPEAKER_PIN,2000, 50);
        myController.dynamicMode3();
        tone(SPEAKER_PIN,2000, 50);
        delay(500);
        break;
      default:  //未知指令
        myController.MySender->bridgeSerial->println(F("Unknown Command"));
        tone(SPEAKER_PIN,200, 300);
        delay(500);
        break;
    }
    myController.MySender->bridgeSerial->println("");
    myController.MySender->bridgeSerial->println(F("Ready!"));
    myController.MySender->bridgeSerial->println("");
  }
  delay(10);
}
