#include "HiiroFoxDSP.h"
class DrumSampler {
private:
  int16_t *data;
  int datalen;
  int trig;  //触发信号
  int pos;   //播放到的位置
  int lastpos;
  float decay = 1.0;
public:
  void InitSample(int16_t *dataPtr, int data_len) {
    data = dataPtr, datalen = data_len;
  }
  void SetTrig(int state) {
    if (state == 1) {
      pos = 0, trig = 1;
    }
    if (state == -1) {
      pos = 0, trig = 0;
    }
  }
  StereoSignal GetBufferStereo() {/*
    if (lastpos >= datalen) {
      lastpos = 0;
      decay = 0;
    }*/
    if (trig == 1 && pos < datalen) {
      //return (StereoSignal){ decay * data[lastpos++] + data[pos++], decay * data[lastpos++] + data[pos++] };
      return (StereoSignal){data[pos++],data[pos++] };
      //decay *= 0.9995;
    } else {/*
      if (trig == 0) {
        lastpos = pos / 2 * 2;
        decay = 1.0;
      }*/
      pos = 0, trig = 0;
      return (StereoSignal){ 0, 0 };
    }
  }
};