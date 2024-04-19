#ifndef _FOX_DSP_  //写得太乱了谁帮我整理一下
#define _FOX_DSP_

#include "math.h"
#include "dsps_math.h"  //dsp库
#include "dsps_biquad_gen.h"

#ifdef __cplusplus  //c++ for Arduino(ESP32)



struct int24_t {  //自制的24bit整型 用于24位播放（没用到）
  uint8_t num[3];
};
int24_t I32TOI24(int32_t x) {
  int24_t result;

  result.num[0] = ((uint8_t *)&x)[0];
  result.num[1] = ((uint8_t *)&x)[1];
  result.num[2] = ((uint8_t *)&x)[2];

  return result;
}


/*
ctof = ctof > 1.0 ? 1.0 : ctof;
    reso = reso > 1.0 ? 1.0 : reso;
    ctof = ctof < 0.0 ? 0.0 : ctof;
    reso = reso < 0.0 ? 0.0 : reso;
*/
struct StereoSignal {  //双声道信号类型
  int32_t l, r;
};
struct StereoSignalFloat {  //双声道信号类型
  float l, r;
};


class Filter {  //滤波器
private:
  float tmp1 = 0, tmp2 = 0, out1 = 0, out2 = 0;
public:
  void reset() {
    tmp1 = 0, tmp2 = 0, out1 = 0, out2 = 0;
  }
  void AutoZero() {
    /*
    if (isnan(tmp1)) tmp1 = 0.0;
    if (isnan(tmp2)) tmp2 = 0.0;
    if (isnan(out1)) out1 = 0.0;
    if (isnan(out2)) out2 = 0.0;
    */
  }
  float LPF1(float vin, float ctof, float reso) {  //一阶低通带反馈
    float fb = reso + reso / (1.0 - ctof);
    tmp1 += ctof * (vin - tmp1 + fb * (tmp1 - out1));
    out1 += ctof * (tmp1 - out1);
    return out1;
  }
  float LPF1_limit(float vin, float ctof, float reso, float limVol = 52000.0, float limK = 0.125) {  //一阶低通带限制反馈
    float fb = reso + reso / (1.0 - ctof);
    tmp1 += ctof * (vin - tmp1 + fb * (tmp1 - out1));
    out1 += ctof * (tmp1 - out1);
    out1 = out1 > limVol ? ((out1 - limVol) * limK + limVol) : out1;
    out1 = out1 < -limVol ? ((out1 + limVol) * limK - limVol) : out1;
    tmp1 = tmp1 > limVol ? ((tmp1 - limVol) * limK + limVol) : tmp1;
    tmp1 = tmp1 < -limVol ? ((tmp1 + limVol) * limK - limVol) : tmp1;
    return out1;
  }
  float LPF1_Oversampling_limit(float vin, float ctof, float reso, float limVol = 52000.0, float limK = 0.5) {  //超采样一阶低通，可以和普通低通一样使用
    ctof *= 0.5;
    float fb = reso + reso / (1.0 - ctof);
    tmp1 += ctof * (vin - tmp1 + fb * (tmp1 - out1));
    out1 += ctof * (tmp1 - out1);
    tmp1 += ctof * (vin - tmp1 + fb * (tmp1 - out1));
    out1 += ctof * (tmp1 - out1);
    out1 = out1 > limVol ? ((out1 - limVol) * limK + limVol) : out1;
    out1 = out1 < -limVol ? ((out1 + limVol) * limK - limVol) : out1;
    tmp1 = tmp1 > limVol ? ((tmp1 - limVol) * limK + limVol) : tmp1;
    tmp1 = tmp1 < -limVol ? ((tmp1 + limVol) * limK - limVol) : tmp1;
    return out1;
  }
  float LPF2(float vin, float ctof, float reso) {  //二阶的，同上
    float fb = reso + reso / (1.0 - ctof);
    tmp1 += ctof * (vin - tmp1 + fb * (tmp1 - out1));
    out1 += ctof * (tmp1 - out1);
    tmp2 += ctof * (out1 - tmp2 + fb * (tmp2 - out2));
    out2 += ctof * (tmp2 - out2);
    return out2;
  }
  float LPF2_limit(float vin, float ctof, float reso, float limVol = 52000.0, float limK = 0.125) {
    float fb = reso + reso / (1.0 - ctof);
    tmp1 += ctof * (vin - tmp1 + fb * (tmp1 - out1));
    out1 += ctof * (tmp1 - out1);
    tmp2 += ctof * (out1 - tmp2 + fb * (tmp2 - out2));
    out2 += ctof * (tmp2 - out2);
    out1 = out1 > limVol ? ((out1 - limVol) * limK + limVol) : out1;
    out1 = out1 < -limVol ? ((out1 + limVol) * limK - limVol) : out1;
    tmp1 = tmp1 > limVol ? ((tmp1 - limVol) * limK + limVol) : tmp1;
    tmp1 = tmp1 < -limVol ? ((tmp1 + limVol) * limK - limVol) : tmp1;
    out2 = out2 > limVol ? ((out2 - limVol) * limK + limVol) : out2;
    out2 = out2 < -limVol ? ((out2 + limVol) * limK - limVol) : out2;
    tmp2 = tmp2 > limVol ? ((tmp2 - limVol) * limK + limVol) : tmp2;
    tmp2 = tmp2 < -limVol ? ((tmp2 + limVol) * limK - limVol) : tmp2;
    return out2;
  }
  float LPF2_Oversampling_ResoLimit_limit(float vin, float ctof, float reso, float limVol = 52000.0, float limK = 0.125) {
    ctof *= 0.5;
    float fb = reso + reso / (1.0 - ctof);
    tmp1 += ctof * (vin - tmp1);
    out1 += ctof * (tmp1 - out1);
    tmp2 += ctof * (out1 - tmp2 + fb * (tmp2 - out2));
    out2 += ctof * (tmp2 - out2);
    tmp1 += ctof * (vin - tmp1);
    out1 += ctof * (tmp1 - out1);
    tmp2 += ctof * (out1 - tmp2 + fb * (tmp2 - out2));
    out2 += ctof * (tmp2 - out2);
    out1 = out1 > limVol ? ((out1 - limVol) * limK + limVol) : out1;
    out1 = out1 < -limVol ? ((out1 + limVol) * limK - limVol) : out1;
    tmp1 = tmp1 > limVol ? ((tmp1 - limVol) * limK + limVol) : tmp1;
    tmp1 = tmp1 < -limVol ? ((tmp1 + limVol) * limK - limVol) : tmp1;
    out2 = out2 > limVol ? ((out2 - limVol) * limK + limVol) : out2;
    out2 = out2 < -limVol ? ((out2 + limVol) * limK - limVol) : out2;
    tmp2 = tmp2 > limVol ? ((tmp2 - limVol) * limK + limVol) : tmp2;
    tmp2 = tmp2 < -limVol ? ((tmp2 + limVol) * limK - limVol) : tmp2;
    return out2;
  }
  float LPF2_ResoLimit_limit(float vin, float ctof, float reso, float limVol = 52000.0, float limK = 0.125) {
    float fb = reso + reso / (1.0 - ctof);
    tmp1 += ctof * (vin - tmp1);
    out1 += ctof * (tmp1 - out1);
    tmp2 += ctof * (out1 - tmp2 + fb * (tmp2 - out2));
    out2 += ctof * (tmp2 - out2);
    out1 = out1 > limVol ? ((out1 - limVol) * limK + limVol) : out1;
    out1 = out1 < -limVol ? ((out1 + limVol) * limK - limVol) : out1;
    tmp1 = tmp1 > limVol ? ((tmp1 - limVol) * limK + limVol) : tmp1;
    tmp1 = tmp1 < -limVol ? ((tmp1 + limVol) * limK - limVol) : tmp1;
    out2 = out2 > limVol ? ((out2 - limVol) * limK + limVol) : out2;
    out2 = out2 < -limVol ? ((out2 + limVol) * limK - limVol) : out2;
    tmp2 = tmp2 > limVol ? ((tmp2 - limVol) * limK + limVol) : tmp2;
    tmp2 = tmp2 < -limVol ? ((tmp2 + limVol) * limK - limVol) : tmp2;
    return out2;
  }
  float HPF1(float vin, float ctof) {  //高通
    tmp1 += ctof * (vin - tmp1);
    return vin - tmp1;
  }
};


float sinmap[256];      //sin打表
bool IsSinMapInit = 0;  //打过标志
class HF_Sine {         //快速正弦类
private:
  float x = 1, y = 0;
public:
  HF_Sine() {
    if (!IsSinMapInit) {
      IsSinMapInit = true;
      for (int i = 0; i < 256; ++i)
        sinmap[i] = sin((double)i / 255.0 * 2.0 * M_PI);
    }
  }
  float sin256_d(double t256) {  //t:(0.0~255.0)->(0~2.0*M_PI rad) 这是打表sin
    return sinmap[(uint8_t)t256] - (t256 - (int)t256) * (sinmap[(uint8_t)(t256 + 1.0)] - sinmap[(uint8_t)t256]);
  }
  float sin256_fast_d(double t256) {  //快速打表sin（无线性平滑）
    return sinmap[(uint8_t)t256];
  }
  float sin256_f(float t256) {  //t:(0.0~255.0)->(0~2.0*M_PI rad) 同上，只是类型为float
    return sinmap[(uint8_t)t256] - (t256 - (int)t256) * (sinmap[(uint8_t)(t256 + 1.0)] - sinmap[(uint8_t)t256]);
  }
  float sin256_fast_f(float t256) {
    return sinmap[(uint8_t)t256];
  }
  float sin_addt(float freq) {  //sin(t+=freq)  freq:(0.0~1.0)->(0~SAMPLE_RATE hz) 通过计算的高精度sin，输入的是频率不是角度。
    y += x * freq;
    x -= y * freq;
    return y;
  }
  float cos_addt(float freq) {  //cos(t+=freq)
    y += x * freq;
    x -= y * freq;
    return x;
  }
  void Unitization() {  //标准化（不改变相位）只适用于sin_addt和cos_addt
    float r = sqrt(x * x + y * y);
    x /= r;
    y /= r;
  }
  void ResetPhase() {  //重置
    x = 1, y = 0;
  }
  float dampsine(float freq, float damp) {  //带衰减的sin，我用来搓鼓机了
    y += x * freq;
    x -= y * freq;
    x *= damp;
    y *= damp;
    return y;
  }
};


int VCO_Select_SAW_4096(int x) {  //周期为4096的saw
  return x % 4096 - 2048;
}
HF_Sine WaveTable_SIN;
int VCO_Select_Wavetable_SampleRate(int x, float wt) {  //没写完的wavetable
  if (wt < 1.0) {
    //sin to trig
  } else if (wt < 2.0) {
    //trig to saw
  } else if (wt < 3.0) {
    //saw to 1/2 pulze
  } else if (wt < 4.0) {
    //1/2pulze to 1/4 pulze
  }
}
/*
    int v1 = (WaveTable_SIN.sin256_fast_f(x * 256 / 48000) + 1.0) * 24000;
    int v2 = x < 24000 ? (x * 2) : (48000 - x * 2);
    return (1.0 - wt) * v1 + (wt)*v2;

    wt -= 1.0;
    return x < wt * 48000 ? ((1.0-wt) * x) : (48000 - wt * x);
*/
#define MaxUnisonN (48)  //最多振荡器数
class VCO {              //绝顶的vco，rp厂所有(以下的saw都带unison)
private:
  int32_t t1 = 0;
  uint32_t ts1[MaxUnisonN];
  uint32_t ts2[MaxUnisonN];

  float freq1, freq2;
public:
  void debug() {
    Serial.printf("%d\n", ts1[2]);
  }
  void AutoZero() {  //自动归零。有些振荡器不归零会出bug
    if (t1 >= (int32_t)2147283647) t1 = 0;
    for (int i = 0; i < MaxUnisonN; ++i) {
      if (ts1[i] >= (int32_t)2147283647) ts1[i] = 0;
      if (ts2[i] >= (int32_t)2147283647) ts2[i] = 0;
    }
  }
  StereoSignal Func_Unison_Stereo_Realtime(int (*func)(int), int Tlen, float TotalFreq, int UniN, float delta) {  //对任意函数进行unison
    int tmp2_l = 0, tmp2_r = 0, tmp_switch = 1;
    TotalFreq = TotalFreq * Tlen / SAMPLE_RATE;
    freq1 = TotalFreq + freq1 - (int)freq1, freq2 = TotalFreq + freq2 - (int)freq2;
    for (int i = 0; i < UniN; ++i) {
      tmp2_l += func(((ts1[i] += (uint32_t)(freq1 += delta))) % Tlen);
      tmp2_r += func(((ts2[i] += (uint32_t)(freq2 -= delta))) % Tlen);
    }
    return (StereoSignal){ tmp2_l + tmp2_r, tmp2_l - tmp2_r };
  }
  float Saw1(int SawFreq, int UniN, float delta) {  //普通saw，unison为指数频率
    //t1 += (SawFreq << 15) / SAMPLE_RATE;
    t1 += SawFreq;
    float tmp1 = 1;
    int tmp2 = 0;
    for (int i = 0; i < UniN; ++i)
      //tmp2 += (uint16_t)((tmp1 *= delta) * t1);
      tmp2 += (uint32_t)((tmp1 *= delta) * (int32_t)t1) % SAMPLE_RATE;
    return tmp2 / UniN - (SAMPLE_RATE >> 1);
  }
  StereoSignal Saw1_Stereo(int SawFreq, int UniN, float delta) {  //普通双声道saw，unison为指数频率
    //t1 += (SawFreq << 15) / SAMPLE_RATE;
    t1 += SawFreq;
    float tmp1 = 1;
    int tmp2_l = 0, tmp2_r = 0, tmp_switch = 1;
    for (int i = 0; i < UniN; ++i) {
      //tmp2 += (uint16_t)((tmp1 *= delta) * t1);
      if (tmp_switch = !tmp_switch)
        tmp2_l += (uint32_t)((tmp1 *= delta) * (int32_t)t1) % SAMPLE_RATE;
      else
        tmp2_r += (uint32_t)((tmp1 *= delta) * (int32_t)t1) % SAMPLE_RATE;
    }
    return (StereoSignal){ (tmp2_l + tmp2_r) / UniN - (SAMPLE_RATE >> 1), (tmp2_l - tmp2_r) / UniN - (SAMPLE_RATE >> 1) };
  }
  /*
  StereoSignal Saw1_Stereo_Realtime(float SawFreq, int UniN, float delta) {
    //t1 += (SawFreq << 15) / SAMPLE_RATE;
    int tmp2_l = 0, tmp2_r = 0, tmp_switch = 1;
    for (int i = 0; i < UniN; ++i) {
      //tmp2 += (uint16_t)((tmp1 *= delta) * t1);
      if (tmp_switch = !tmp_switch)
        tmp2_l += (ts1[i] += (int32_t)(SawFreq *= delta)) % SAMPLE_RATE;
      else
        tmp2_r += (ts2[i] += (int32_t)(SawFreq *= delta)) % SAMPLE_RATE;
    }
    return (StereoSignal){ (tmp2_l + tmp2_r) / UniN - (SAMPLE_RATE >> 1), (tmp2_l - tmp2_r) / UniN - (SAMPLE_RATE >> 1) };
  }
*/
  StereoSignal Saw1_Stereo_Realtime(float SawFreq, int UniN, float delta) {  //实时的双声道saw
    //t1 += (SawFreq << 15) / SAMPLE_RATE;
    int tmp2_l = 0, tmp2_r = 0, tmp_switch = 1;
    freq1 = SawFreq + freq1 - (int)freq1, freq2 = SawFreq + freq2 - (int)freq2;
    for (int i = 0; i < UniN; ++i) {
      tmp2_l += ((ts1[i] += (uint32_t)(freq1 *= delta)) - 1357) % SAMPLE_RATE;
      tmp2_r += ((ts2[i] += (uint32_t)(freq2 /= delta)) + 2048) % SAMPLE_RATE;
    }
    return (StereoSignal){ (tmp2_l + tmp2_r) / UniN - (SAMPLE_RATE >> 1), (tmp2_l - tmp2_r) / UniN - (SAMPLE_RATE >> 1) };
  }
  void Multi_Saw1_Stereo(float *outl, float *outr, int SawFreq, int len, int UniN, float delta) {
    float tmp1 = 1;
    int tmp2_l = 0, tmp2_r = 0, tmp_switch = 1, sign;
    for (int p = 0; p < len; ++p) {
      t1 += SawFreq;

      tmp1 = 1;
      tmp2_l = 0, tmp2_r = 0, tmp_switch = 1, sign;
      for (int i = 0; i < UniN; ++i) {
        //tmp2 += (uint16_t)((tmp1 *= delta) * t1);
        if (tmp_switch = !tmp_switch)
          tmp2_l += (uint32_t)((tmp1 *= delta) * (int32_t)t1) % SAMPLE_RATE;
        else
          tmp2_r += (uint32_t)((tmp1 *= delta) * (int32_t)t1) % SAMPLE_RATE;
      }
      outl[p] = (tmp2_l + tmp2_r) / UniN - (SAMPLE_RATE >> 1);
      outr[p] = (tmp2_l - tmp2_r) / UniN - (SAMPLE_RATE >> 1);
      //outl[p] = tmp2_l + tmp2_r;
      //outr[p] = tmp2_l - tmp2_r;
    }
    /*
    float tmp_val = 1.0/UniN;
    dsps_mul_f32_ansi(outl,&tmp_val,outl,len,1,0,1);
    dsps_mul_f32_ansi(outr,&tmp_val,outr,len,1,0,1);
    tmp_val = SAMPLE_RATE >> 1;
    dsps_sub_f32_ansi(outl,&tmp_val,outl,len,1,0,1);
    dsps_sub_f32_ansi(outr,&tmp_val,outr,len,1,0,1);
    */
  }
  float Saw2(int SawFreq, int UniN, float delta) {
    //t1 += (SawFreq << 15) / SAMPLE_RATE;
    t1 += SawFreq;
    delta -= 1.0;
    float tmp1 = 1;
    int tmp2 = 0;
    for (int i = 0; i < UniN; ++i)
      //tmp2 += (uint16_t)((tmp1 *= delta) * t1);
      tmp2 += (uint32_t)((tmp1 += delta) * (int32_t)t1) % SAMPLE_RATE;
    return tmp2 / UniN - (SAMPLE_RATE >> 1);
  }
  StereoSignal Saw2_Stereo(int SawFreq, int UniN, float delta) {
    //t1 += (SawFreq << 15) / SAMPLE_RATE;
    t1 += SawFreq;
    float tmp1 = 1;
    int tmp2_l = 0, tmp2_r = 0, tmp_switch = 1, sign;
    delta -= 1.0;
    for (int i = 0; i < UniN; ++i) {
      //tmp2 += (uint16_t)((tmp1 *= delta) * t1);
      if (tmp_switch = !tmp_switch)
        tmp2_l += (uint32_t)((tmp1 += delta) * (int32_t)t1) % SAMPLE_RATE;
      else
        tmp2_r += (uint32_t)((tmp1 += delta) * (int32_t)t1) % SAMPLE_RATE;
    }
    return (StereoSignal){ (tmp2_l + tmp2_r) / UniN - (SAMPLE_RATE >> 1), (tmp2_l - tmp2_r) / UniN - (SAMPLE_RATE >> 1) };
  }
  /*
  StereoSignal Saw2_Stereo_Realtime(float SawFreq, int UniN, float delta) {
    //t1 += (SawFreq << 15) / SAMPLE_RATE;
    int tmp2_l = 0, tmp2_r = 0, tmp_switch = 1;
    for (int i = 0; i < UniN; ++i) {
      //tmp2 += (uint16_t)((tmp1 *= delta) * t1);
      if (tmp_switch = !tmp_switch)
        tmp2_l += (ts[i] += (int32_t)(SawFreq += delta)) % SAMPLE_RATE;
      else
        tmp2_r += (ts[i] += (int32_t)(SawFreq += delta)) % SAMPLE_RATE;
    }
    return (StereoSignal){ (tmp2_l + tmp2_r) / UniN - (SAMPLE_RATE >> 1), (tmp2_l - tmp2_r) / UniN - (SAMPLE_RATE >> 1) };
  }*/

  StereoSignal Saw2_Stereo_Realtime(float SawFreq, int UniN, float delta) {
    //t1 += (SawFreq << 15) / SAMPLE_RATE;
    int tmp2_l = 0, tmp2_r = 0, tmp_switch = 1;
    freq1 = SawFreq + freq1 - (int)freq1, freq2 = SawFreq + freq2 - (int)freq2;
    for (int i = 0; i < UniN; ++i) {
      tmp2_l += ((ts1[i] += (uint32_t)(freq1 += delta)) - 1357) % SAMPLE_RATE;
      tmp2_r += ((ts2[i] += (uint32_t)(freq2 -= delta)) + 2048) % SAMPLE_RATE;
    }
    return (StereoSignal){ (tmp2_l + tmp2_r) / UniN - (SAMPLE_RATE >> 1), (tmp2_l - tmp2_r) / UniN - (SAMPLE_RATE >> 1) };
  }

  StereoSignal Saw2_SyncFreq_Realtime(float SawFreq, float SyncFreq, int UniN, float delta) {  //带sync freq的saw
    //t1 += (SawFreq << 15) / SAMPLE_RATE;
    int tmp2_l = 0, tmp2_r = 0, tmp_switch = 1;
    freq1 = SawFreq + freq1 - (int)freq1, freq2 = SawFreq + freq2 - (int)freq2;
    for (int i = 0; i < UniN; ++i) {
      tmp2_l += (uint32_t)(SyncFreq * (((ts1[i] += (uint32_t)(freq1 += delta)) - 1357) % SAMPLE_RATE)) % SAMPLE_RATE;
      tmp2_r += (uint32_t)(SyncFreq * (((ts2[i] += (uint32_t)(freq2 -= delta)) + 2048) % SAMPLE_RATE)) % SAMPLE_RATE;
    }
    return (StereoSignal){ (tmp2_l + tmp2_r) / UniN - (SAMPLE_RATE >> 1), (tmp2_l - tmp2_r) / UniN - (SAMPLE_RATE >> 1) };
  }
  StereoSignal Saw2_SyncFreq_2Pole_Realtime(float SawFreq, float SyncFreq1, float SyncFreq2, int UniN, float delta) {  //带sync freq的saw
    //t1 += (SawFreq << 15) / SAMPLE_RATE;
    int tmp2_l = 0, tmp2_r = 0, tmp_switch = 1;
    freq1 = SawFreq + freq1 - (int)freq1, freq2 = SawFreq + freq2 - (int)freq2;
    for (int i = 0; i < UniN; ++i) {
      tmp2_l += (uint32_t)(SyncFreq2 * ((uint32_t)(SyncFreq1 * (((ts1[i] += (uint32_t)(freq1 += delta)) - 1357) % SAMPLE_RATE)) % SAMPLE_RATE)) % SAMPLE_RATE;
      tmp2_r += (uint32_t)(SyncFreq2 * ((uint32_t)(SyncFreq1 * (((ts2[i] += (uint32_t)(freq2 -= delta)) + 2048) % SAMPLE_RATE)) % SAMPLE_RATE)) % SAMPLE_RATE;
    }
    return (StereoSignal){ (tmp2_l + tmp2_r) / UniN - (SAMPLE_RATE >> 1), (tmp2_l - tmp2_r) / UniN - (SAMPLE_RATE >> 1) };
  }
  StereoSignal VCO_SyncFreq_Wavetable_Realtime(float SawFreq, float SyncFreq, int table, int UniN, float delta) {  //带syncfreq和wavetable和unison的vco
    //t1 += (SawFreq << 15) / SAMPLE_RATE;
    int tmp2_l = 0, tmp2_r = 0, tmp_switch = 1, lv = 0, rv = 0;
    freq1 = SawFreq + freq1 - (int)freq1, freq2 = SawFreq + freq2 - (int)freq2;
    for (int i = 0; i < UniN; ++i) {
      lv = (uint32_t)(SyncFreq * (((ts1[i] += (uint32_t)(freq1 += delta)) - 1357) % SAMPLE_RATE)) % SAMPLE_RATE;
      rv = (uint32_t)(SyncFreq * (((ts2[i] += (uint32_t)(freq2 -= delta)) + 2048) % SAMPLE_RATE)) % SAMPLE_RATE;
      lv = lv <= table ? 0 : lv;
      rv = rv <= table ? 0 : rv;
      lv = lv >= SAMPLE_RATE - 1 - table ? SAMPLE_RATE - 1 : lv;
      rv = rv >= SAMPLE_RATE - 1 - table ? SAMPLE_RATE - 1 : rv;
      tmp2_l += lv;
      tmp2_r += rv;
    }
    return (StereoSignal){ (tmp2_l + tmp2_r) / UniN - (SAMPLE_RATE >> 1), (tmp2_l - tmp2_r) / UniN - (SAMPLE_RATE >> 1) };
  }

  float Sqr1(int SawFreq, int UniN, float delta, int PWM) {
    //t1 += (SawFreq << 15) / SAMPLE_RATE;
    t1 += SawFreq;
    float tmp1 = 1;
    int tmp2 = 0;
    for (int i = 0; i < UniN; ++i)
      //tmp2 += (uint16_t)((tmp1 *= delta) * t1);
      tmp2 += ((int)((tmp1 *= delta) * (int32_t)t1) % SAMPLE_RATE) > PWM ? -1 : 1;
    return tmp2 * (SAMPLE_RATE >> 1) / UniN;
  }
  float Sqr2(int SawFreq, int UniN, float delta, int PWM) {
    //t1 += (SawFreq << 15) / SAMPLE_RATE;
    t1 += SawFreq;
    float tmp1 = 1;
    delta -= 1.0;
    int tmp2 = 0;
    for (int i = 0; i < UniN; ++i)
      //tmp2 += (uint16_t)((tmp1 *= delta) * t1);
      tmp2 += ((int)((tmp1 += delta) * (int32_t)t1) % SAMPLE_RATE) > PWM ? -1 : 1;
    return tmp2 * (SAMPLE_RATE >> 1) / UniN;
  }
  StereoSignal Sqr2_SyncFreq_Realtime(float SawFreq, float SyncFreq, int UniN, float delta) {  //带sync freq的saw
    //t1 += (SawFreq << 15) / SAMPLE_RATE;
    int tmp2_l = 0, tmp2_r = 0, tmp_switch = 1;
    freq1 = SawFreq + freq1 - (int)freq1, freq2 = SawFreq + freq2 - (int)freq2;
    for (int i = 0; i < UniN; ++i) {
      tmp2_l += ((uint32_t)(SyncFreq * (((ts1[i] += (uint32_t)(freq1 += delta)) - 1357) % SAMPLE_RATE)) % SAMPLE_RATE) > 24000 ? 1 : -1;
      tmp2_r += ((uint32_t)(SyncFreq * (((ts2[i] += (uint32_t)(freq2 -= delta)) + 2048) % SAMPLE_RATE)) % SAMPLE_RATE) > 24000 ? 1 : -1;
    }
    return (StereoSignal){ (tmp2_l + tmp2_r) / UniN * 24000.0, (tmp2_l - tmp2_r) / UniN * 24000.0 };
  }
};


class RP808DRUM {  //鼓合成器
private:
  float KickFreq = 0;  //kick
  //Filter osc;//原本是用拉满reso的滤波器做的振荡器
  HF_Sine osc;
public:
  float kick(int trig) {  //kick
    KickFreq += 0.00015 * (10.0 - KickFreq);
    if (trig) {
      KickFreq = 29;
      //osc.reset();
      osc.ResetPhase();
    }
    //return osc.LPF2(trig * 32768, (KickFreq * KickFreq) / SAMPLE_RATE, 0.975);
    return osc.dampsine((KickFreq * KickFreq) / SAMPLE_RATE, 0.9999975) * 4096.0;
  }
};

#define Phaser_MaxApfN (32)
class phaser {  //phaser效果器
private:
  float apf_v0[Phaser_MaxApfN];
  float apf(float vin, float freq, int n) {  //全通信号 = 2*低通信号 - 原信号
    apf_v0[n] += freq * (vin - apf_v0[n]);
    return apf_v0[n] * 2.0 - vin;
  }
  float v_fdbk;
public:
  void reset() {
    for (int i = 0; i < Phaser_MaxApfN; ++i)
      apf_v0[i] = 0;
  }
  phaser() {
    reset();
  }
  float proc(float in, float freq, float reso, float stat) {
    float v = in + reso * v_fdbk;
    for (int i = 0; i < (int)stat - 1; ++i)
      v = apf(v, freq, i);
    float v2 = apf(v, freq, (int)stat - 1);
    float mix = stat - (int)stat;
    return v_fdbk = v * (1.0 - mix) + v2 * (mix);
  }
  float proc_fast(float in, float freq, float reso, int stat) {
    float v = in + reso * v_fdbk;
    for (int i = 0; i < (int)stat; ++i)
      v = apf(v, freq, i);
    return v_fdbk = v;
  }
};

const int maxDlyTime = SAMPLE_RATE;
class HF_Delay {  //delay效果器
private:
  float *buf = NULL;
  //int mode[maxDlyTime];
  int mods;
  int t;
public:
  void reset() {
    t = 0;
    memset(buf, 0, maxDlyTime * sizeof(float));
    //memset(mode, 0, sizeof(mode));
  }
  HF_Delay() {
    buf = (float *)malloc(maxDlyTime * sizeof(float));
    reset();
  }

  float proc(float in, int dly_len, float fdbk, float mix) {
    t++;
    buf[(t + dly_len) % maxDlyTime] = in + fdbk * buf[t % maxDlyTime];
    //return in * (1.0 - mix) + buf[t % maxDlyTime] * (mix);  //0:in   0.5:(in+dat)/2   1:dat
    return in + buf[t % maxDlyTime] * (mix);  //0:in   0.5:(in+dat)/2   1:dat
  }
  float delay_line(float in, int dly_len) {
    t++;
    buf[(t + (int)dly_len) % maxDlyTime] = in;
    buf[(t + (int)dly_len + 1) % maxDlyTime] = in;
    buf[(t + (int)dly_len + 2) % maxDlyTime] = in;
    return buf[t % maxDlyTime];
  }
};

const int StringSampleLen = SAMPLE_RATE / 15;
float *RanData = NULL;
float *RanData2 = NULL;  //双声道用的
char isRanDataInit = false;
class karplus_strong {
private:
  float *delay_line = NULL;
  float *delay_line2 = NULL;
  int SampleLen = 10;
  int pos = 0;
  int Freq;
public:
  karplus_strong() {
    delay_line = (float *)malloc(sizeof(float) * StringSampleLen);
    delay_line2 = (float *)malloc(sizeof(float) * StringSampleLen);
    memset(delay_line, 0, sizeof(float) * StringSampleLen);
    memset(delay_line2, 0, sizeof(float) * StringSampleLen);
    if (isRanDataInit == false) {
      isRanDataInit = true;
      RanData = (float *)malloc(sizeof(float) * StringSampleLen);
      RanData2 = (float *)malloc(sizeof(float) * StringSampleLen);
      for (int i = 0; i < StringSampleLen; ++i) {
        //RanData[i] = (i * i) % 5000 - 2500;  //试试这个感觉也很有意思
        //RanData[i] = sin((i * i) / 1000.0) * 10000.0;  //这个感觉有意思
        //RanData[i] = (float)(rand() % 10000) * (rand() % 2 ? 1.0 : -1.0);  //随机采样
        RanData[i] = rand() % 2 ? 10000.0 : -10000.0;   //随机采样(10000或-10000)
        RanData2[i] = rand() % 2 ? 10000.0 : -10000.0;  //随机采样(10000或-10000)
      }
    }
  }
  int GetFreq() {
    return Freq;
  }
  void SetTrig(int freq) {
    Freq = freq;
    SampleLen = SAMPLE_RATE / freq;                   //这个*2是超采样
    if (SampleLen >= StringSampleLen) SampleLen = 0;  //就别响了
    pos = 0;
    memcpy(delay_line, RanData, sizeof(float) * SampleLen);
    memcpy(delay_line2, RanData, sizeof(float) * SampleLen);
  }
  float proc(float decay, float lpf) {
    pos++;
    int NextPos = (pos + 1) % SampleLen;
    //return delay_line[NextPos] = (lpf * delay_line[NextPos] + (1.0 - lpf) * delay_line[(pos) % SampleLen]) * decay;
    return delay_line[NextPos] = (lpf * (delay_line[(pos) % SampleLen] - delay_line[NextPos]) + delay_line[NextPos]) * decay;
  }
  StereoSignalFloat proc_stereo(float decay, float lpf) {
    pos++;
    int NextPos = (pos + 1) % SampleLen;
    //return delay_line[NextPos] = (lpf * delay_line[NextPos] + (1.0 - lpf) * delay_line[(pos) % SampleLen]) * decay;
    return (StereoSignalFloat){ delay_line[NextPos] = (lpf * (delay_line[(pos) % SampleLen] - delay_line[NextPos]) + delay_line[NextPos]) * decay, delay_line2[NextPos] = (lpf * (delay_line2[(pos) % SampleLen] - delay_line2[NextPos]) + delay_line2[NextPos]) * decay };
  }
};

int RevCombLens[4] = { 0.0297 * SAMPLE_RATE, 0.0371 * SAMPLE_RATE, 0.0411 * SAMPLE_RATE, 0.0437 * SAMPLE_RATE };
float RevCombFdbk[4] = { 0.742, 0.733, 0.715, 0.697 };
int RevAPFLens[2] = { 0.09683 * SAMPLE_RATE, 0.03292 * SAMPLE_RATE };
float RevAPFFdbk[2] = { 0.7, 0.7 };
#define MaxReverbLen 7200
class reverb {
private:
  float dlylines[4][MaxReverbLen];   //4个梳状滤波器
  float apflines[2][MaxReverbLen];   //2个全通
  float apflines2[2][MaxReverbLen];  //stereo
  int dlypos = 0, apfpos = 0;
  Filter dlyhpfs[4];
  Filter apfhpfs[2];
public:
  float proc(float vin, float fdbk, float mix) {
    float out = 0, tmp = 0;
    for (int i = 0; i < 4; ++i) {
      tmp = dlylines[i][dlypos % MaxReverbLen];
      dlylines[i][(dlypos + RevCombLens[i]) % MaxReverbLen] = dlyhpfs[i].HPF1(vin + tmp * RevCombFdbk[i] * fdbk, 0.005);  //加高通防止反馈直流爆电平
      out += tmp;
    }
    dlypos++;
    for (int i = 0; i < 2; ++i) {
      tmp = apflines[i][apfpos % MaxReverbLen];
      apflines[i][(apfpos + RevAPFLens[i]) % MaxReverbLen] = apfhpfs[i].HPF1(out + tmp * RevAPFFdbk[i] * fdbk, 0.005);
      out = tmp * fdbk * (1.0 - RevAPFFdbk[i] * RevAPFFdbk[i]) - out * RevAPFFdbk[i];
    }
    apfpos++;
    return vin + mix * out;
  }
  StereoSignalFloat proc_ToStereo(float vin, float fdbk, float mix, float roomsize) {
    float outs[4];
    float outL = 0, outR = 0, tmp = 0;
    for (int i = 0; i < 2; ++i) {
      tmp = dlylines[i][dlypos % MaxReverbLen];
      dlylines[i][(int)(dlypos + roomsize * RevCombLens[i]) % MaxReverbLen] = dlyhpfs[i].HPF1(vin + tmp * RevCombFdbk[i] * fdbk, 0.005);  //加高通防止反馈直流爆电平
      outs[i] = tmp;
    }
    dlypos++;
    outL = outs[0] + outs[1] + outs[2] + outs[3];
    outR = outs[0] - outs[1] + outs[2] - outs[3];
    for (int i = 0; i < 2; ++i) {
      tmp = apflines[i][apfpos % MaxReverbLen];
      apflines[i][(int)(apfpos + roomsize * RevAPFLens[i]) % MaxReverbLen] = apfhpfs[i].HPF1(outL + tmp * RevAPFFdbk[i] * fdbk, 0.005);
      outL = tmp * fdbk * (1.0 - RevAPFFdbk[i] * RevAPFFdbk[i]) - outL * RevAPFFdbk[i];
    }
    for (int i = 0; i < 2; ++i) {
      tmp = apflines2[i][apfpos % MaxReverbLen];
      apflines2[i][(int)(apfpos + roomsize * RevAPFLens[i]) % MaxReverbLen] = apfhpfs[i].HPF1(outR + tmp * RevAPFFdbk[i] * fdbk, 0.005);
      outR = tmp * fdbk * (1.0 - RevAPFFdbk[i] * RevAPFFdbk[i]) - outR * RevAPFFdbk[i];
    }
    apfpos++;
    return (StereoSignalFloat){ vin + mix * outL, vin + mix * outR };
  }
};
#else  //c for cdk(W806)
//w806是垃圾
#endif

#endif  //hiirofox(gy314) dsp copyright 2023