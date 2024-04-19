#include <math.h>
#include "driver/i2s.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp32-hal-cpu.h"

#define SAMPLE_RATE 44100

#include "DrumSample.h"
#include "Sampler.h"

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
  memset(WavBuf, 0, BufLen * sizeof(int32_t));
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
class RPdrum {
private:
  ScanKB skb;

  DrumSampler sampler[13];
  char loopdata[13][16][3];  //n,len,pattern
  float t = 0;


  Filter adcfilt1, adcfilt2, adcfilt3, adcfilt4;     //adc的滤波器
  Filter adcfilt5, adcfilt6, adcfilt7, adcfilt8;     //adc的滤波器
  Filter adcfilt9, adcfilt10, adcfilt11, adcfilt12;  //adc的滤波器

  float Vbpm = 0;
public:
  void Init() {
    sampler[0].InitSample((int16_t *)KickData, 175525);
    sampler[1].InitSample((int16_t *)SnareData, 25765);
    sampler[2].InitSample((int16_t *)ChData, 11941);
    sampler[3].InitSample((int16_t *)OhData, 58021);
    sampler[4].InitSample((int16_t *)TomData, 51109);
    sampler[5].InitSample((int16_t *)CowbellData, 62629);
    sampler[6].InitSample((int16_t *)CrashData, 87973);
    sampler[7].InitSample((int16_t *)ClavData, 7333);
    sampler[8].InitSample((int16_t *)CongaData, 18853);
    memset(loopdata, 0, sizeof(loopdata));
  }
  void UpdataAlgm() {
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

    Vbpm = (adcfilt4.LPF1(tmpv4, 0.1, 0) / 4095.0 + 0.2) * 200.0 / SAMPLE_RATE / 12.0;
  }
  int SelectMode = 1, isplay = 0;
  int SelectDrum = 0;  //选择
  int block = 0, pattern = 0;
  int pos = 0;
  void UpdataKB() {  //不用看，这就是屎山。我没想好很好的方法来写，因为扫描键盘毕竟和普通键盘不一样。
    skb.scan();      //更新键盘
    if (skb.KeyStateOnce(13) == 1) {
      SelectMode = !SelectMode;
      if (SelectMode == 1) printf("SelectMode\n");
      else printf("EditMode\n");
    }
    if (skb.KeyStateOnce(14) == 1) {
      isplay = !isplay;
      if (isplay == 1) {
        printf("Play!\n");
      } else {
        printf("Stop!\n");
        block = 0;
        pattern = 0;
        pos = 0;
      }
    }
    if (skb.KeyStateOnce(15) == 1) {
      memset(loopdata, 0, sizeof(loopdata));
    }
    if (SelectMode == 1 && isplay == 0) {
      for (int i = 0; i < 13; ++i) {
        if (skb.KeyStateOnce(i) == 1) {
          Serial.printf("trig %d\n", i);
          sampler[i].SetTrig(1);
          SelectDrum = i;
        }
      }
    } else {
      if (skb.KeyStateOnce(1) == 1) block = 0;  //上8个
      if (skb.KeyStateOnce(3) == 1) block = 1;  //下8个
      /*
      if (skb.KeyStateOnce(6) == 1) pattern = 0;  //第一个pattern
      if (skb.KeyStateOnce(8) == 1) pattern = 1;
      if (skb.KeyStateOnce(10 == 1)) pattern = 2;
      */
      if (isplay == 0) {
        if (skb.KeyStateOnce(0) == 1) loopdata[SelectDrum][0 + block * 8][pattern] = !loopdata[SelectDrum][0 + block * 8][pattern], sampler[SelectDrum].SetTrig(1);
        if (skb.KeyStateOnce(2) == 1) loopdata[SelectDrum][1 + block * 8][pattern] = !loopdata[SelectDrum][1 + block * 8][pattern], sampler[SelectDrum].SetTrig(1);
        if (skb.KeyStateOnce(4) == 1) loopdata[SelectDrum][2 + block * 8][pattern] = !loopdata[SelectDrum][2 + block * 8][pattern], sampler[SelectDrum].SetTrig(1);
        if (skb.KeyStateOnce(5) == 1) loopdata[SelectDrum][3 + block * 8][pattern] = !loopdata[SelectDrum][3 + block * 8][pattern], sampler[SelectDrum].SetTrig(1);
        if (skb.KeyStateOnce(7) == 1) loopdata[SelectDrum][4 + block * 8][pattern] = !loopdata[SelectDrum][4 + block * 8][pattern], sampler[SelectDrum].SetTrig(1);
        if (skb.KeyStateOnce(9) == 1) loopdata[SelectDrum][5 + block * 8][pattern] = !loopdata[SelectDrum][5 + block * 8][pattern], sampler[SelectDrum].SetTrig(1);
        if (skb.KeyStateOnce(11) == 1) loopdata[SelectDrum][6 + block * 8][pattern] = !loopdata[SelectDrum][6 + block * 8][pattern], sampler[SelectDrum].SetTrig(1);
        if (skb.KeyStateOnce(12) == 1) loopdata[SelectDrum][7 + block * 8][pattern] = !loopdata[SelectDrum][7 + block * 8][pattern], sampler[SelectDrum].SetTrig(1);
      }
    }
    /*
    if (skb.KeyState(0) && skb.KeyState(1) && skb.KeyState(2)) esp_restart();  //软复位
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
    }*/
  }
  void proc() {
    for (int i = 0; i < BufLen; i += 2) {
      StereoSignal sign = { 0, 0 };
      for (int n = 0; n < 12; ++n) {
        StereoSignal tmp;
        tmp = sampler[n].GetBufferStereo();
        sign.l += tmp.l;
        sign.r += tmp.r;
      }
      WavBuf[i + 0] = sign.l * 20000.0;
      WavBuf[i + 1] = sign.r * 20000.0;

      t += Vbpm;
      if (t >= 1) {
        t -= 1.0;
        if (isplay) {
          for (int n = 0; n < 12; ++n) {
            if (loopdata[n][pos][pattern]) sampler[n].SetTrig(1);
          }
          pos++;
          if (pos >= 16) pos = 0;
        }
      }
    }
  }
};
RPdrum RPDrumProc;
void loop() {
  Serial.println(esp_get_idf_version());
  RPDrumProc.Init();
  delay(100);
  for (int i = 0; i <= 400; ++i) {
    RPDrumProc.UpdataADC();
    delay(1);
  }
  for (;;) {
    uint64_t sT = esp_timer_get_time();
    RPDrumProc.proc();
    uint64_t eT = esp_timer_get_time();
    uint64_t PlayTime = wo.PlayAudio(WavBuf);  //播放buffer

    //Serial.printf("All:%6d  Algm:%6d (%.2f%%)\n", (int)(eT - sT + PlayTime), (int)(eT - sT), (float)(eT - sT) / (eT - sT + PlayTime) * 100.0);  //debug

    RPDrumProc.UpdataADC();   //更新adc
    RPDrumProc.UpdataKB();    //更新键盘
    RPDrumProc.UpdataAlgm();  //更新算法
  }
  //...
  wo.Stop();
}