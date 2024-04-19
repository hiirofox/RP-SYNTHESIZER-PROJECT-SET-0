#ifndef _RP_DX3_
#define _RP_DX3_


class FM_3OP {
private:
  uint16_t freq;
  uint16_t v1 = 0, v2 = 0, v3 = 0;
  uint16_t p1 = 0, p2 = 0, p3 = 0;
  HF_Sine SIN;
  float t_fdbk = 0;
public:
  float proc(float f2, float f3, float vp1, float vp2, float vp3, float fdbk) {
    p1 += freq, p2 += f2 * freq, p3 += f3 * freq;
    return SIN.sin32768_fast(SIN.sin32768_fast((t_fdbk = SIN.sin32768_fast(p3 + t_fdbk * fdbk)) * vp3 * v3 + p2) * vp2 * v2 + p1) * vp1 * v1;
  }
  void SetTrig(uint16_t Freq) {
    freq = Freq;
    v1 = v2 = v3 = 65535;
  }
  void UpdataVolume(float vd1, float vd2, float vd3) {
    v1 = vd1 * v1;
    v2 = vd2 * v2;
    v3 = vd3 * v3;
  }
};

#endif