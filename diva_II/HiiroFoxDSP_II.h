#ifndef _FOX_DSP_II_
#define _FOX_DSP_II_

#include "HiiroFoxDSP.h"

class ADSR {
private:
  float A = 1.0, D = 0.0, S = 1.0, R = 1.0;
  float V = 0.0;
  char mode = 3;  //0:a  1:d  2:s  3:r
public:
  void SetADSR(float a, float d, float s, float r) {
    A = a, D = d, S = s, R = r;
  }
  void SetGate(int state) {
    if (state == 0)
      mode = 3;
    else if (state == 1 && mode == 3) {  //必须先松开了才有按下
      mode = 0;
    }
  }
  float proc() {
    if (mode == 3) {
      if (V > 0.0) {
        V -= R;
        if (V < 0.0) V = 0.0;
      }
    } else if (mode == 0) {
      V += A;
      if (V > 1.0) {
        V = 1.0;
        mode = 1;  // -> d
      }
    } else if (mode == 1) {
      V -= D;
      if (V < S) {
        V = S;
        mode = 2;  // -> s
      }
    } else if (mode == 2) {
      V = S;
    }
    return V * V;  //平滑
  }
};

#endif