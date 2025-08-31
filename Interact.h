#ifndef INTERACT_H
#define INTERACT_H

#include "GCodeCtrl.h"
#include <U8glib.h>


//实例化
GCodeCtrl myController(SLIDE, TELE_L, TELE_S);  

U8GLIB_NHD_C12864 u8g(SCREEN_SCK, SCREEN_MOSI, SCREEN_CS, SCREEN_RS, SCREEN_RST);

//参数
int nowTrialTime = 0;

//坐标预设1
float windows[12][3][2]= 
{
  { //预设1
    {40, 40}, //winCenter
    {50, 20}, //winSize
    {0, 0}  //dir&angle
  },{ //预设2
    {40, 40},
    {20, 15},
    {0, 0}
  },{ //预设3
    {40, 40},
    {50, 20},
    {CCW, PI/4}
  },{ //预设4
    {40, 40},
    {20, 15},
    {CCW, PI/4}
  },{ //预设5
    {40, 40},
    {50, 20},
    {CW, PI/4}
  },{ //预设6
    {40, 40},
    {20, 15},
    {CW, PI/4}
  },{ //预设7
    {40, 40},
    {15, 50},
    {0, 0}
  },{ //预设8
    {40, 40},
    {10, 20},
    {0, 0}
  },{ //预设9
    {40, 40},
    {15, 50},
    {0, 0}
  },{ //预设10
    {40, 40},
    {10, 20},
    {0, 0}
  },{ //预设11
    {40, 40},
    {15, 50},
    {0, 0}
  },{ //预设12
    {40, 40},
    {10, 20},
    {0, 0}
  },
};
/*
//坐标预设2
float windows[12][3][2]= 
{
  { //预设1  188 Va
    {40, 40}, //winCenter   
    {36.3, 60}, //winSize   
    {0, 0}  //dir&angle    
  },
  { //预设2  188 Vb
    {40, 40},  
    {24.2, 60},
    {0, 0}
  },
  { //预设3  188 Vc
    {40, 40},
    {16.13, 60},
    {0, 0}
  },
  { //预设4  189 va
    {40, 40},
    {35.7, 60},
    {0, 0}
  },
  { //预设5 189 Vb
    {40, 40},
    {23.8, 60},
    {0, 0}
  },
  { //预设6  189 Vc
    {40, 40},
    {15.87, 60},
    {0, 0}
  },
  { //预设7 198 Va
    {40, 40},
    {34.65, 60},
    {0, 0}
  },
  { //预设8 198 Vb
    {40, 40},
    {23.1, 60},
    {0, 0}
  },
  { //预设9 198 Vc
    {40, 40},
    {15.4, 60},
    {0, 0}
  },
  { //预设10 199 Va
    {40, 40},
    {39.15, 60},
    {0, 0}
  },
  { //预设11 199 Vb
    {40, 40},
    {26.1, 60},
    {0, 0}
  },
  { //预设12 199 Vc
    {40, 40},
    {17.4, 60},
    {0, 0}
  },
};
*/

//欢迎界面
void display_1(){
  String displayWords_1 = F("\nCOMMANDER 5 Pro\n\n是否校准?\n1=开始校准, 0=不校准\n");
  myController.MySender->bridgeSerial->print(displayWords_1);
  //等待串口回应
  while (!myController.MySender->bridgeSerial->available()) {
    delay(10);
  }  
  char data = myController.MySender->bridgeSerial->read();
  while (myController.MySender->bridgeSerial->available()) { //仅读取一个字节数据，其他数据清除
    myController.MySender->bridgeSerial->read();
    delay(5);
  }

  if (data == '1') {
    myController.MySender->bridgeSerial->println(F("正在校准..."));
    tone(SPEAKER_PIN,2000, 200);
    myController.steppersCalibration(COMPENSATE); 
    tone(SPEAKER_PIN,2000, 200);
    myController.MySender->bridgeSerial->println(F("已就绪!"));
  } else if (data == '0') {
    for (int i=0;i<3;i++){
      tone(SPEAKER_PIN,2000, 50);
      delay(100);
    }
    myController.steppersCalibration();
    delay(10);
    myController.MySender->bridgeSerial->println(F("已就绪!"));
  } else {
    myController.MySender->bridgeSerial->println(F("未知指令，开始校准..."));
    myController.steppersCalibration(COMPENSATE); 
    tone(SPEAKER_PIN,2000, 200);
    myController.MySender->bridgeSerial->println(F("已就绪!"));
  }
  String dispalyWord_2 = F(" \n发送预设窗口编号:(1-9) // \nC: 摇杆控制\nV: 速度设置\nF: 窗口自由控制模式\n$: 显示预设窗口\nN: 实验次数显示\n$: 打印预设窗口\n!: 紧急停止\nP:关机\n\nD: 动态实验模式1(窗口绕中心点旋转)\nE: 动态实验模式2(窗口平移)\nG: 动态实验模式3(窗口绕顶点旋转)\nH: 动态实验模式4(窗口往复移动)\n");
  myController.MySender->bridgeSerial->println(dispalyWord_2);

}

//窗口信息显示
void display_2 (float winCenter[2], float winSize[2], float dir[2]) {
  myController.MySender->bridgeSerial->println("中心点:("+String(winCenter[0])+","+String(winCenter[1])+")");
  myController.MySender->bridgeSerial->println("尺寸:("+String(winSize[0])+","+String(winSize[1])+")");
  myController.MySender->bridgeSerial->println("方向及角度:("+String(dir[0])+","+String(dir[1])+")");
}

//打印预设窗口
void display_3(){
  myController.MySender->bridgeSerial->println("DEFAULTS");
  for (int i=0;i<8;i++ ) {
    myController.MySender->bridgeSerial->println(i+1);
    for (int m=0;m<3;m++) {
      for (int k=0;k<2;k++) {
        myController.MySender->bridgeSerial->print(windows[i][m][k]);
        myController.MySender->bridgeSerial->print("  ");
      }
      myController.MySender->bridgeSerial->print("\n");
    }
    delay(500);
  }
}

//数字显示
void drawNumber(int theNum) {
  u8g.setPrintPos(10, -10);
  u8g.print(theNum); 
}

//实验次数显示
void drawTrialTimes(){
  myController.MySender->bridgeSerial->println("Send the trial number:");
  String bridgeCmd;
  analogWrite(SCREEN_LED_PIN, SCREEN_LED_BRIGHTNESS);
  while (1) {
    //接受bridge数据
    bridgeCmd = "";
    while (myController.MySender->bridgeSerial->available()) { 
      char tempChar = myController.MySender->bridgeSerial->read();
      bridgeCmd += (char)tempChar;
      delay(10);
    }
    if (bridgeCmd != "") {
      String temp = bridgeCmd;
      temp.trim();
      if (temp == "q" || temp == "Q") {
        myController.MySender->bridgeSerial->println(F("----Finished----"));
        break;
      } else if (temp == "current number" || temp == "cn" ) {
        myController.MySender->bridgeSerial->print(F("current num: "));
        myController.MySender->bridgeSerial->println(nowTrialTime);
        continue;
      } 
      nowTrialTime = temp.toInt();
      myController.MySender->bridgeSerial->print(F("Times: "));
      myController.MySender->bridgeSerial->println(nowTrialTime);
      
      u8g.firstPage();
      do {
        drawNumber(nowTrialTime);
      } while (u8g.nextPage());
      delay(50);
    }
  }
}



#endif