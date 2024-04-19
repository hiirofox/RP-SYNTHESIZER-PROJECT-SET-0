#include <math.h>
#include "driver/i2s.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp32-hal-cpu.h"


#define SAMPLE_RATE 56000
const float TO_RAD = 2.0 * M_PI / SAMPLE_RATE;
const float TO_256 = 256.0 / SAMPLE_RATE;
#include "HiiroFoxDSP.h"

class WaveOut {
private:
  i2s_config_t i2s_cfg;
  i2s_pin_config_t i2s_pin_cfg;
  int SampRate;
public:
  WaveOut(int SampleRate) {
    SampRate = SampleRate;
    i2s_cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    i2s_cfg.sample_rate = SampRate;
    i2s_cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
    i2s_cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
    i2s_cfg.communication_format = I2S_COMM_FORMAT_I2S_MSB;
    i2s_cfg.dma_buf_count = 2;
    i2s_cfg.dma_buf_len = 512;
    i2s_cfg.use_apll = false;
    i2s_cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    i2s_cfg.tx_desc_auto_clear = true;
    i2s_cfg.fixed_mclk = 0;
    /*
    //d0wdq6
    i2s_pin_cfg.bck_io_num = 26;
    i2s_pin_cfg.ws_io_num = 25;
    i2s_pin_cfg.data_out_num = 22;//d0wdq6
    i2s_pin_cfg.data_in_num = I2S_PIN_NO_CHANGE;
    */
    //s3
    i2s_pin_cfg.bck_io_num = 42;    //BCK
    i2s_pin_cfg.data_out_num = 41;  //DIN
    i2s_pin_cfg.ws_io_num = 40;     //LCK
    i2s_pin_cfg.data_in_num = I2S_PIN_NO_CHANGE;
  }
  void Start() {
    i2s_driver_install(I2S_NUM_0, &i2s_cfg, 0, NULL);                                                  //开启i2s
    i2s_set_pin(I2S_NUM_0, &i2s_pin_cfg);                                                              //设置引脚
    i2s_set_clk(I2S_NUM_0, SampRate, i2s_cfg.bits_per_sample, (i2s_channel_t)i2s_cfg.channel_format);  //设置时钟
    //i2s_set_dac_mode((i2s_dac_mode_t)(I2S_DAC_CHANNEL_RIGHT_EN | I2S_DAC_CHANNEL_LEFT_EN)); //没用，而且会崩溃
    i2s_zero_dma_buffer(I2S_NUM_0);  //清空dma缓存
  }
  int PlayAudio(void *Buffer) {  //播放buffer
    size_t bytes_written;
    uint64_t sT = esp_timer_get_time();
    i2s_write(I2S_NUM_0, (const char *)Buffer, GetBufferLen() * sizeof(int32_t), &bytes_written, portMAX_DELAY);
    uint64_t eT = esp_timer_get_time();
    return eT - sT;
  }
  void Stop() {
    i2s_driver_uninstall(I2S_NUM_0);
  }
  int GetBufferLen() {
    return i2s_cfg.dma_buf_len * i2s_cfg.dma_buf_count;
  }
};
WaveOut wo(SAMPLE_RATE);
const int BufLen = wo.GetBufferLen();
int32_t *WavBuf = NULL;
/*
float *stmpL = NULL;
float *stmpR = NULL;
float *tmp_f16 = NULL;
*/
void setup() {
  setCpuFrequencyMhz(240);
  Serial.begin(115200);

  for (int i = 1; i <= 12; ++i)
    pinMode(i, INPUT);

  wo.Start();                                            //开启i2s
  WavBuf = (int32_t *)malloc(BufLen * sizeof(int32_t));  //初始化buffer
  /*
  stmpL = (float *)malloc(BufLen * sizeof(float) / 2);    //L buf
  stmpR = (float *)malloc(BufLen * sizeof(float) / 2);    //R buf
  tmp_f16 = (float *)malloc(BufLen * sizeof(float) / 2);  //tmp buf
  */
}

class ScanKB {  //键盘类
private:
  int wp[4] = { 17, 18, 19, 20 };
  int hp[4] = { 13, 14, 15, 16 };
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
class RP303 {
private:
  ScanKB skb;

  //float Vsyncfreq2 = 0;
  float Vdecay = 0;  //adc调参
  float Vbpm = 0;
  float Voct = 0;
  float Vdrive = 0;  //失真
  float Vdist = 0;
  float Vpost = 0;
  float Vctof = 0;  //主滤波器
  float Vreso = 0;
  float VLPCtof = 0;  //失真后的lp
  float VHPCtof = 0;  //失真前的hp
  float Vslide = 0;
  float Vkbcf = 0;

  Filter adcfilt1, adcfilt2, adcfilt3, adcfilt4;     //adc的滤波器
  Filter adcfilt5, adcfilt6, adcfilt7, adcfilt8;     //adc的滤波器
  Filter adcfilt9, adcfilt10, adcfilt11, adcfilt12;  //adc的滤波器


  float loopfreq[128] = { 0 };  //音序器信息
  char loopgate[128] = { 0 };
  float SawFreq = 55, CutOff = 0, KbcFreq = 1.0;
  int isPlay = 0;
  int looplen = 0, looppos = 0, lastup = 0;

  float t = 0;  //音符更新

  Filter filt1;
  Filter filt2;
  Filter filt3;
  Filter filt4;
  Filter filt5;
  Filter filt6;

  VCO vco1;
  RP808DRUM drum1;
  HF_Sine lop1, lop2;
  HF_Sine rop1, rop2;
  phaser eff_phs;
  HF_Delay ldly, rdly;

public:
  void UpdataAlgm() {
    vco1.AutoZero();
    filt1.AutoZero();
    filt2.AutoZero();
    filt3.AutoZero();
    filt4.AutoZero();
    filt5.AutoZero();
    filt6.AutoZero();
  }
  void UpdataADC() {
    float tmpv1 = (float)analogRead(1);
    float tmpv2 = (float)analogRead(2);
    float tmpv3 = (float)analogRead(3);
    float tmpv4 = (float)analogRead(4);
    float tmpv5 = (float)analogRead(5);
    float tmpv6 = (float)analogRead(6);
    float tmpv7 = (float)analogRead(7);
    float tmpv8 = (float)analogRead(8);
    float tmpv9 = (float)analogRead(9);
    float tmpv10 = (float)analogRead(10);
    float tmpv11 = (float)analogRead(11);
    float tmpv12 = (float)analogRead(12);
    //vco
    Vdrive = adcfilt1.LPF1(tmpv1, 0.3, 0) / 4095.0 * 32.0 + 1.0;
    Vdist = 1.0 - adcfilt2.LPF2(tmpv2, 0.3, 0) / 4095.0 * 1.0;
    Vpost = adcfilt3.LPF1(tmpv3, 0.3, 0) / 4095.0 * 4.0;

    //setting
    Vbpm = (adcfilt4.LPF1(tmpv4, 0.1, 0) / 4095.0 + 0.2) * 200.0 / SAMPLE_RATE / 12.0;
    Vdecay = 1.0 - pow(adcfilt5.LPF1(tmpv5, 0.3, 0) / 4095.0, 3.0) * 0.002;
    Voct = pow(2, (int)(adcfilt6.LPF1(tmpv6, 0.3, 0) / 4095.0 * 8.0));

    //base filter
    Vctof = adcfilt7.LPF1(tmpv7, 0.3, 0) / 4095.0;  //1.85
    Vctof = Vctof * Vctof * 1.95;
    Vreso = sqrt(sqrt(adcfilt8.LPF1(tmpv8, 0.3, 0) / 4095.0)) * 0.95;

    //Other filter
    VHPCtof = adcfilt9.LPF1(tmpv9, 0.3, 0) / 4095.0;
    VLPCtof = adcfilt10.LPF1(tmpv10, 0.3, 0) / 4095.0;
    VHPCtof = VHPCtof * VHPCtof;
    VLPCtof = VLPCtof * VLPCtof;

    Vslide = adcfilt11.LPF1(tmpv11, 0.3, 0) / 4095.0;
    Vslide = Vslide * Vslide * Vslide * Vslide * Vslide;
    Vkbcf = 1.0 - adcfilt12.LPF1(tmpv12, 0.3, 0) / 4095.0;


    //oct
  }
  void UpdataKB() {                                                            //不用看，这就是屎山。我没想好很好的方法来写，因为扫描键盘毕竟和普通键盘不一样。
    skb.scan();                                                                //更新键盘
    //if (skb.KeyState(0) && skb.KeyState(1) && skb.KeyState(2)) esp_restart();  //软复位
    if (skb.KeyState(13) && skb.KeyState(15)) {                                //清空音序器
      memset(loopfreq, 0, sizeof(loopfreq));
      memset(loopgate, 0, sizeof(loopgate));
      looplen = 0;  //清空音序器
      looppos = 0;
      lastup = 0;
      isPlay = 0;
    }
    if (skb.KeyStateOnce(14) == 1) {  //模式选择
      isPlay = !isPlay;
      if (isPlay == 0) {
        looppos = 0;  //重置位置
      }
    }
    if (isPlay == 0) {
      int thisFreq = -999999, isdown = 0;
      for (int i = 0; i < 13; ++i) {  //检测按下
        if (skb.KeyStateOnce(i) == 1) isdown = 1, thisFreq = i;
      }
      if (skb.KeyStateOnce(13) == 1) {
        SawFreq = 0;
        KbcFreq = 0;
        loopgate[looplen] = 0;
      } else if (isdown == 1) {  //存在按下
        SawFreq = 110.0 * pow(2.0, (float)(thisFreq - 36 + 3) / 12.0);
        KbcFreq = SawFreq / 110.0 * Voct;
        CutOff = 1;
        loopgate[looplen] = 1;
      }
      if (skb.KeyStateOnce(15) == 1) {           //记录
        loopfreq[looplen] = SawFreq * Voct / 4;  //音序 freq
        looplen++;
      }
    } else {  //升调模式
      int thisFreq = -999999, isdown = 0;
      for (int i = 0; i < 13; ++i) {  //检测按下
        if (skb.KeyStateOnce(i) == 1) isdown = 1, thisFreq = i;
      }
      if (isdown == 1) {
        for (int i = 0; i < looplen; ++i) {  //统一升调
          loopfreq[i] *= pow(2.0, (float)(thisFreq - lastup) / 12.0);
        }
        lastup = thisFreq;
      }
    }
  }
  void proc() {
    for (int i = 0; i < BufLen; i += 2) {
      float datL = 0.0, datR = 0.0;

      int sign = vco1.Saw_NoUnison(filt1.LPF1_NoReso(SawFreq * Voct, Vslide));
      float now_ctof = (CutOff *= Vdecay) * Vctof * ((1.0 - Vkbcf) * KbcFreq + Vkbcf);
      now_ctof = now_ctof > 1.95 ? 1.95 : now_ctof;
      sign = filt2.LPF2_Oversampling_ResoLimit_limit(sign, now_ctof, Vreso);
      sign = filt3.LPF2_Oversampling_ResoLimit_limit(sign, now_ctof, Vreso);
      sign = filt5.HPF1_NoReso(sign, VHPCtof);
      sign = Vpost * Dist_Limit_Float(sign * Vdrive, Vdist, SAMPLE_RATE >> 1);
      sign = filt6.LPF1_NoReso(sign, VLPCtof);

      datL = sign;
      datR = sign;

      int tmpL = datL * 10000.0;
      int tmpR = datR * 10000.0;
      WavBuf[i + 0] = tmpL;
      WavBuf[i + 1] = tmpR;

      t += Vbpm;
      if (t >= 1.0) {
        if (isPlay) {
          SawFreq = loopfreq[looppos];
          KbcFreq = SawFreq / 110.0 * Voct;
          if (loopgate[looppos] == 1) CutOff = 1.0;  //如果触发gate
          looppos++;
          if (looppos >= looplen) looppos = 0;
        }
        t -= 1.0;
      }
    }
  }
};
RP303 RP303Proc;
void loop() {
  Serial.println(esp_get_idf_version());
  delay(100);
  for (int i = 0; i <= 400; ++i) {
    RP303Proc.UpdataADC();
    delay(1);
  }
  for (;;) {
    uint64_t sT = esp_timer_get_time();
    RP303Proc.proc();
    uint64_t eT = esp_timer_get_time();
    uint64_t PlayTime = wo.PlayAudio(WavBuf);  //播放buffer

    Serial.printf("All:%6d  Algm:%6d (%.2f%%)\n", (int)(eT - sT + PlayTime), (int)(eT - sT), (float)(eT - sT) / (eT - sT + PlayTime) * 100.0);

    RP303Proc.UpdataADC();   //更新adc
    RP303Proc.UpdataKB();    //更新键盘
    RP303Proc.UpdataAlgm();  //更新算法
  }
  //...
  wo.Stop();
}