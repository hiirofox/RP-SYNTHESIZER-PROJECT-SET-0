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
struct StereoSignalFloat {  //双声道信号类型(float)
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
  float HPF1_limit(float vin, float ctof, float limVol = 52000.0, float limK = 0.125) {  //高通
    vin = vin > limVol ? ((vin - limVol) * limK + limVol) : vin;
    vin = vin < -limVol ? ((vin + limVol) * limK - limVol) : vin;
    tmp1 += ctof * (vin - tmp1);
    return vin - tmp1;
  }
};


float sinmap[256];      //sin打表
bool IsSinMapInit = 0;  //打过标志
class HF_Sine {         //快速正弦类
private:
public:
  float x = 1, y = 0;
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
  void proc(float freq) {
    y += x * freq;
    x -= y * freq;
  }
  float get_sin() {
    return y;
  }
  float get_cos() {
    return x;
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
    return buf[t % maxDlyTime];
  }
};



class PolyPhaseAPF {  //多相全通
private:
  float z2 = 0, z1 = 0;
  float y1 = 0, y2 = 0;
public:
  float proc(float sign, float a) {  //faster
    sign += z2 * a;
    a = z2 - sign * a;
    z2 = z1, z1 = sign;
    return a;
  }
  /*
  float proc(float sign, float a) {
    a = (sign + y2) * a - z2;
    z2 = z1, z1 = sign;
    y2 = y1, y1 = a;
    return a;
  }*/
};
#define MaxApfN (4)
const float a_re[4] = { 0.4021921162426, 0.8561710882420, 0.9722909545651, 0.9952884791278 };  //hilbert iir apf常数
const float a_im[4] = { 0.6923878000000, 0.9360654322959, 0.9882295226860, 0.9987488452737 };
class phase_shift {  //hilbert变换
private:
  PolyPhaseAPF filt_re[MaxApfN];
  PolyPhaseAPF filt_im[MaxApfN];
  float im_delay = 0;
public:
  float get_re, get_im;
  void proc_fast(float sign_re, float sign_im) {
    for (int i = 0; i < MaxApfN; i++) sign_re = filt_re[i].proc(sign_re, a_re[i]);
    for (int i = 0; i < MaxApfN; i++) sign_im = filt_im[i].proc(sign_im, a_im[i]);
    get_re = sign_re;
    get_im = im_delay;
    im_delay = sign_im;
  }
  float proc_re(float sign_re) {
    for (int i = 0; i < MaxApfN; i++) sign_re = filt_re[i].proc(sign_re, a_re[i]);
    return sign_re;
  }
  float proc_im(float sign_im) {
    for (int i = 0; i < MaxApfN; i++) sign_im = filt_im[i].proc(sign_im, a_im[i]);
    float tmp = im_delay;
    im_delay = sign_im;
    return tmp;
  }
  StereoSignalFloat proc(float sign) {
    float sign_re = sign, sign_im = sign;
    for (int i = 0; i < MaxApfN; i++) sign_re = filt_re[i].proc(sign_re, a_re[i]);
    for (int i = 0; i < MaxApfN; i++) sign_im = filt_im[i].proc(sign_im, a_im[i]);
    float tmp = im_delay;
    im_delay = sign_im;
    return (StereoSignalFloat){ sign_re, tmp };
  }
};
class BBP_Flanger {  //无限上升/下降flanger
private:
  HF_Sine lfo;
  phase_shift shift;
  HF_Delay delay;
  Filter hpfilt, lpfilt;
  float f_fdbk;
  float LastTotal = 0;
public:
  void AutoZero() {
    lfo.Unitization();
  }
  float proc(float in1, float Vrate, float Vstep, float Vfdbk, float Vmix) {
    float in2 = in1 + f_fdbk * Vfdbk;
    float total = delay.delay_line(in2, 2 * (int)Vstep);
    float re = shift.proc_re(total) * lfo.get_sin();
    float im = shift.proc_im(total) * lfo.get_cos();
    lfo.proc(Vrate);
    f_fdbk = -lpfilt.LPF2_Oversampling_ResoLimit_limit(hpfilt.HPF1_limit(re + im, 0.15), 1.5, 0);
    return (1.0 - Vmix) * in1 + Vmix * (f_fdbk + in2);
  }
};
#define MaxPhaserAPFN (128)
class PolyPhasePhaser {
private:
  PolyPhaseAPF apfs[MaxPhaserAPFN];
  float z1[MaxPhaserAPFN];
  float z2[MaxPhaserAPFN];
public:
  float proc(float in, float a, int step) {
    for (int i = 0; i < step; ++i)
      in = apfs[i].proc(in, a);
    return in;
  }
  float proc_2(float in, float sign_a, int step) {
    float tmp;
    for (int i = 0; i < step; ++i) {
      in += z2[i] * sign_a;
      tmp = z2[i] - in * sign_a;
      z2[i] = z1[i], z1[i] = in;
      in = tmp;
    }
    return in;
  }
};
class BBP_Phaser {  //无限上升/下降phaser
private:
  HF_Sine lfo;
  phase_shift shift;
  Filter hpfilt, lpfilt;
  PolyPhasePhaser phs;
  float f_fdbk;
  float LastTotal = 0;
public:
  void AutoZero() {
    lfo.Unitization();
  }
  float proc(float in1, float Vrate, float Vstep, float sign_a, float Vfdbk, float Vmix) {
    float in2 = in1 + f_fdbk * Vfdbk;
    float total = phs.proc(in2, sign_a, Vstep);
    shift.proc_fast(total,total);
    lfo.proc(Vrate);
    f_fdbk = -lpfilt.LPF2_Oversampling_ResoLimit_limit(hpfilt.HPF1_limit(shift.get_re * lfo.y + shift.get_im * lfo.x, 0.075), 1.75, 0);
    return (1.0 - Vmix) * in1 + Vmix * (f_fdbk + in2);
  }
};
class BBP_Phaser_Stereo {  //无限上升/下降phaser
private:
  HF_Sine lfo;
  phase_shift shiftl;
  phase_shift shiftr;
  Filter hpfiltl, lpfiltl;
  Filter hpfiltr, lpfiltr;
  PolyPhasePhaser phsl;
  PolyPhasePhaser phsr;
  float f_fdbkl;
  float f_fdbkr;
  float LastTotal = 0;
public:
  void AutoZero() {
    lfo.Unitization();
  }
  StereoSignalFloat proc(StereoSignalFloat sign, float Vrate, float Vstep, float sign_a, float Vfdbk, float Vmix) {
    float in2l = sign.l + f_fdbkl * Vfdbk;
    float in2r = sign.r + f_fdbkr * Vfdbk;
    float totall = phsl.proc(in2l, sign_a, Vstep);
    float totalr = phsr.proc(in2r, sign_a, Vstep);
    f_fdbkl = -lpfiltl.LPF2_Oversampling_ResoLimit_limit(hpfiltl.HPF1_limit(shiftl.proc_re(totall) * lfo.y + shiftl.proc_im(totall) * lfo.x, 0.075), 1.75, 0);
    f_fdbkr = -lpfiltr.LPF2_Oversampling_ResoLimit_limit(hpfiltr.HPF1_limit(shiftr.proc_re(totalr) * lfo.y + shiftr.proc_im(totalr) * lfo.x, 0.075), 1.75, 0);
    lfo.proc(Vrate);
    return (StereoSignalFloat){ (1.0 - Vmix) * sign.l + Vmix * (f_fdbkl + in2l), (1.0 - Vmix) * sign.r + Vmix * (f_fdbkr + in2r) };
  }
  StereoSignalFloat proc_mixr(StereoSignalFloat sign, float Vrate, float Vstep, float sign_a, float Vfdbk, float Vmix) {
    float in2l = sign.l + f_fdbkl * Vfdbk;
    float in2r = sign.r + f_fdbkl * Vfdbk;
    float totall = phsl.proc(in2l, sign_a, Vstep);
    f_fdbkl = -lpfiltl.LPF2_Oversampling_ResoLimit_limit(hpfiltl.HPF1_limit(shiftl.proc_re(totall) * lfo.get_sin() + shiftl.proc_im(totall) * lfo.get_cos(), 0.075), 1.75, 0);
    lfo.proc(Vrate);
    return (StereoSignalFloat){ (1.0 - Vmix) * sign.l + Vmix * (f_fdbkl + in2l), (1.0 - Vmix) * sign.r + Vmix * (f_fdbkl + in2r) };
  }
};


#else  //c for cdk(W806)
//w806是垃圾
#endif

#endif  //hiirofox(gy314) dsp copyright 2023