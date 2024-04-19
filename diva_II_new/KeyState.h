#ifndef _KEYSTATE_
#define _KEYSTATE_

class ScanKB {  //键盘类
private:
  //int hp[4] = { 17, 18, 47, 48 };
  //int wp[4] = { 13, 14, 15, 16 };//我的
/*
  int wp[4] = { 17, 18, 19, 20 };
  int hp[4] = { 13, 14, 15, 16 };//给大湿的*/
  
  //int wp[4] = { 20, 19, 18, 17 };
  //int hp[4] = { 16, 15, 14, 13 };//p总的
  
  int hp[4] = { 17, 18, 47, 48 };
  int wp[4] = { 13, 14, 15, 16 };//给飞飞鱼的
  int result[16];
public:
  ScanKB() {
    for (int i = 0; i < 4; ++i) {
      pinMode(wp[i], OUTPUT);
      pinMode(hp[i], INPUT);
      digitalWrite(wp[i], 0);
    }
  }
  void scan() {
    for (int i = 0; i < 4; ++i) {
      digitalWrite(wp[i], 1);
      for (int j = 0; j < 4; ++j) {
        if (result[i * 4 + j] != -1)
          result[i * 4 + j] = digitalRead(hp[j]);
        else if (digitalRead(hp[j]) == 0)
          result[i * 4 + j] = 0;
      }
      digitalWrite(wp[i], 0);
    }
  }
  int KeyState(int n) {
    return result[n] != 0;
  }
  int KeyStateOnce(int n) {
    int a = result[n];
    if (a == 1) result[n] = -1;
    return a;
  }
};

#endif