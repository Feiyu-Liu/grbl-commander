
        /////////////////////////////////////
//////////////////// Commander 5 Pro ////////////////////
      /////////////////////////////////////

//4.1 motor without 'any' delay
//4.2 Pro 添加了窗口绕顶点旋转功能
//4.4 dynamicMode2 改为动捕触发
//5.1 添加光栅trigger

#include "Interact.h"
String cmd;  //储存指令

void setup() {
  myController.serialBegin();
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
  cmd = "";
  if (myController.MySender->bridgeSerial->available()) {
    cmd = myController.MySender->bridgeSerial->readString();
    cmd.trim();
    myController.MySender->bridgeSerial->print("<-");
    char charCmd = cmd.charAt(0);  int cmdInt;
    switch (charCmd) {
      case '1': case '2': case '3': case '4': case '5': case '6': case '7': case '8': case '9':
        cmdInt = cmd.toInt() - 1;
        display_2(windows[cmdInt][0], windows[cmdInt][1], windows[cmdInt][2]); 
        //移动到窗口
        myController.automaticArrival(windows[cmdInt]); 
        delay(1000);
        break;
      case 'c': case 'C':
        myController.MySender->bridgeSerial->println(F("摇杆控制"));
        tone(SPEAKER_PIN,2000, 50);
        delay(100);
        tone(SPEAKER_PIN,2000, 50);
        myController.rockerController();
        tone(SPEAKER_PIN,2000, 50);
        delay(500);
        break;
      case 'u': case 'U':
        myController.MySender->bridgeSerial->println(F("(开发者工具)广播通信"));
        tone(SPEAKER_PIN,2000, 50);
        delay(100);
        tone(SPEAKER_PIN,2000, 50);
        myController.MySender->computerCtrl_Universal();
        tone(SPEAKER_PIN,2000, 50);
        delay(500);
        break;
      case 's': case 'S':
        myController.MySender->bridgeSerial->println(F("(开发者工具)私有通信"));
        tone(SPEAKER_PIN,2000, 50);
        delay(100);
        tone(SPEAKER_PIN,2000, 50);
        myController.MySender->computerCtrl_Specific();
        tone(SPEAKER_PIN,2000, 50);
        delay(500);
        break;
      case 'b': case 'B':
        myController.MySender->bridgeSerial->println(F("(开发者工具)总线通信"));
        tone(SPEAKER_PIN,2000, 50);
        delay(100);
        tone(SPEAKER_PIN,2000, 50);
        myController.MySender->computerCtrl_Batch();
        tone(SPEAKER_PIN,2000, 50);
        delay(500);
        break;
      case '$': 
        myController.MySender->bridgeSerial->println(F("打印预设窗口"));
        display_3();
        break;
      case '!': //紧急停止
        myController.MySender->bridgeSerial->println(F("紧急停止"));
        tone(SPEAKER_PIN,1000, 1000);
        myController.MySender->stopFeeding();
        delay(500);
        break;
      case 'n': case 'N': //数字显示
        myController.MySender->bridgeSerial->println(F("数字显示"));
        tone(SPEAKER_PIN,2000, 50);
        drawTrialTimes();
        delay(500);
        break;
      case 'v': case 'V'://速度设置
        myController.MySender->bridgeSerial->println(F("电机速度设置"));
        tone(SPEAKER_PIN,2000, 50);
        myController.changeSteppersSpeed();
        delay(500);
        break;
      case 'f': case 'F': //自由控制
        myController.MySender->bridgeSerial->println(F("自由控制"));
        tone(SPEAKER_PIN,2000, 50);
        myController.freeControl();
        tone(SPEAKER_PIN,2000, 50);
        delay(500);
        break;
      case 'd': case 'D': //动态实验模式（窗口绕中心点旋转）
        myController.MySender->bridgeSerial->println(F("动态实验模式(绕中心点旋转)"));
        tone(SPEAKER_PIN,2000, 50);
        myController.dynamicMode();
        tone(SPEAKER_PIN,2000, 50);
        delay(500);
        break;
      case 'e': case 'E': //动态实验模式（窗口平移）
        myController.MySender->bridgeSerial->println(F("动态实验模式(平移)"));
        tone(SPEAKER_PIN,2000, 50);
        myController.dynamicMode2();
        tone(SPEAKER_PIN,2000, 50);
        delay(500);
        break;
      case 'g': case 'G': //动态实验模式（窗口绕顶点旋转）
        myController.MySender->bridgeSerial->println(F("动态实验模式(绕顶点旋转)"));
        tone(SPEAKER_PIN,2000, 50);
        myController.dynamicMode3();
        tone(SPEAKER_PIN,2000, 50);
        delay(500);
        break;
      #ifdef voidDebug
      case 'x': case 'x': //Debug
        myController.sleep(SLEEP_POS);
        break;
      #endif
      case 'p': case 'P': //关机
        myController.sleep(SLEEP_POS);
        break;
      default:  //未知指令
        myController.MySender->bridgeSerial->println(charCmd);
        myController.MySender->bridgeSerial->println(F("未知命令"));
        tone(SPEAKER_PIN,200, 300);
        delay(500);
        break;
    }
    myController.MySender->bridgeSerial->print(F("\n就绪!\n"));
  }
  delay(10);
}
