#include <math.h>
#include "driver/i2s.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp32-hal-cpu.h"

#include "freertos.h"
#include "task.h"

#define SAMPLE_RATE 56000
const float TO_RAD = 2.0 * M_PI / SAMPLE_RATE;
const float TO_256 = 256.0 / SAMPLE_RATE;
#include "HiiroFoxDSP.h"

const float ADCUpdataRate = 0.1;  //adc低通速率

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

void EventTask(void *pvParameters);  //事件检测
TaskHandle_t hTask1 = NULL;
void AlgorTask(void *pvParameters);  //算法更新
TaskHandle_t hTask2 = NULL;
void Alg3Task(void *pvParameters);  //某些算法更新
TaskHandle_t hTask3 = NULL;

void setup() {
  setCpuFrequencyMhz(240);
  Serial.begin(115200);

  int UnuseStack = uxTaskGetStackHighWaterMark(NULL);
  xTaskCreatePinnedToCore(EventTask, "EventTask", UnuseStack * 1 / 6, NULL, 0, &hTask1, 1);        //拿来搞事件检测的
  xTaskCreatePinnedToCore(AlgorTask, "AlgorTask", UnuseStack * 5 / 6, NULL, 10000, &hTask2, 0);  //拿来搞算法的
  //xTaskCreate(EventTask, "EventTask", UnuseStack / 6, NULL, 100, &hTask1);        //拿来搞事件检测的
  //xTaskCreate(AlgorTask, "AlgorTask", UnuseStack * 5 / 6, NULL, 10000, &hTask2);  //拿来搞算法的
  //xTaskCreate(Alg3Task, "Alg3Task", UnuseStack * 0.5 / 6, NULL, 0, &hTask3);  //拿来搞某些算法的(比如滤波器的limit和参数的平滑)

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
  //int wp[4] = { 20, 19, 18, 17 };
  //int hp[4] = { 16, 15, 14, 13 };
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
class RPBass {
private:

  ScanKB skb;

  float Vsyncfreq = 0;  //adc调参
  float Vdecay = 0;
  float Vbpm = 0;
  float Vctof = 0;
  float Vreso = 0;
  float Vdunison = 0;
  float Vunin = 0;
  float Vldt = 0, Vrdt = 0, Vdfdbk = 0, Vdmix = 0;
  float Voct = 0;
  Filter adcfilt1, adcfilt2, adcfilt3, adcfilt4;     //adc的滤波器
  Filter adcfilt5, adcfilt6, adcfilt7, adcfilt8;     //adc的滤波器
  Filter adcfilt9, adcfilt10, adcfilt11, adcfilt12;  //adc的滤波器


  float loopfreq[128] = { 0 };
  char loopgate[128] = { 0 };
  float SawFreq = 55, CutOff = 0;
  int isPlay = 0;
  int looplen = 0, looppos = 0, lastup = 0;

  float t = 0;

  float now_ctof = 0.0;
  /*
  Filter filt1_l, filt1_r;
  Filter filt2_l, filt2_r;
  Filter filt3_l, filt3_r;
  Filter filt4_l, filt4_r;
*/
  IIRFilter filt1_l, filt1_r;
  IIRFilter filt2_l, filt2_r;

  //FilterPlus filt_l, filt_r;

  VCO vco1;
  RP808DRUM drum1;
  HF_Sine lop1, lop2;
  HF_Sine rop1, rop2;
  phaser eff_phs;
  HF_Delay ldly, rdly;
public:
  int Count_Event = 0;
  int Count_Algor = 0;
  int Count_Alg3 = 0;
  void debug() {
    Serial.printf("%2.5f reso\n", Vreso);
  }
  void UpdataAlgm() {
    //这里适合更新一下乐鑫的滤波器参数

    filt1_l.UpdataLPF(now_ctof * 0.995, 1.0);
    filt2_l.UpdataLPF(now_ctof * 0.995, Vreso * 8.0 + 0.05);

    vco1.AutoZero();
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
    Vdunison = adcfilt1.LPF1(tmpv1, ADCUpdataRate, 0) / 4095.0 * 0.5 + 0.0005;
    Vunin = adcfilt2.LPF1(tmpv2, ADCUpdataRate, 0) / 4095.0 * 6.0 + 1.0;
    Vsyncfreq = adcfilt3.LPF1(tmpv3, ADCUpdataRate, 0) / 4095.0 * 7.0 + 1.0;
    //Vsyncfreq2 = adcfilt4.LPF1(tmpv4, ADCUpdataRate, 0) / 4095.0 * 7.0 + 1.0;
    //Vwavet = adcfilt4.LPF1(tmpv4, ADCUpdataRate, 0) / 4095.0 * (SAMPLE_RATE - 1) / 2.0;
    //Vkbcf = adcfilt4.LPF1(tmpv4, ADCUpdataRate, 0) / 4095.0;
    Vbpm = (adcfilt4.LPF1(tmpv4, ADCUpdataRate, 0) / 4095.0 + 0.2) * 200.0 / SAMPLE_RATE / 12.0;
    Vdecay = 1.0 - pow(adcfilt5.LPF1(tmpv5, ADCUpdataRate, 0) / 4095.0, 3.0) * 0.002;
    Voct = pow(2, (int)(adcfilt6.LPF1(tmpv6, ADCUpdataRate, 0) / 4095.0 * 8.0));

    //filter
    //Vctof = adcfilt5.LPF1(tmpv5, 0.3, 0) / 4096.0 * sqrt(2.0);
    Vctof = adcfilt7.LPF1(tmpv7, ADCUpdataRate, 0) / 4095.0;  //1.85
    Vctof = Vctof * Vctof;                                    //指数音高
    Vreso = adcfilt8.LPF1(tmpv8, ADCUpdataRate, 0) / 4095.0;

    //delay
    Vldt = (int)(adcfilt9.LPF1(tmpv9, ADCUpdataRate, 0) / 4095.0 * 15.0 + 1.0) * 800;
    Vrdt = (int)(adcfilt10.LPF1(tmpv10, ADCUpdataRate, 0) / 4095.0 * 15.0 + 1.0) * 800;
    Vdfdbk = adcfilt11.LPF1(tmpv11, ADCUpdataRate, 0) / 4095.0 * 0.9;
    Vdmix = adcfilt12.LPF1(tmpv12, ADCUpdataRate, 0) / 4095.0;

    //oct
  }
  void UpdataKB() {                                                            //不用看，这就是屎山。我没想好很好的方法来写，因为扫描键盘毕竟和普通键盘不一样。
    skb.scan();                                                                //更新键盘
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
        loopgate[looplen] = 0;
      } else if (isdown == 1) {  //存在按下
        SawFreq = 110.0 * pow(2.0, (float)(thisFreq - 36 + 3) / 12.0);
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
  float lv = 0, rv = 0;
  void proc() {
    for (int i = 0; i < BufLen; i += 2) {
      float datL = 0.0, datR = 0.0;

      //dat1 = vco1.Saw1(SawFreq, 4, 1.0121) / 2;
      //dat1 = filt1.LPF1(dat1, 0.0 + (CutOff *= 0.99980), 0.15) / 2.0;
      //dat2 = vco2.Saw1(SawFreq, 4, 1.0123) / 2;
      //dat2 = filt2.LPF1(dat2, 0.0 + (CutOff *= 0.99980), 0.15) / 2.0;


      //StereoSignal sign1 = vco1.Saw2_Stereo_Realtime(SawFreq, 2, Vdunison);
      StereoSignal sign1 = { 0, 0 };
      sign1 = vco1.Saw2_SyncFreq_Realtime_SR65536(SawFreq * Voct, Vsyncfreq, (int)Vunin, Vdunison);
      //sign1 = vco1.VCO_SyncFreq_Wavetable_Realtime(SawFreq * Voct, Vsyncfreq, Vwavet, (int)Vunin, Vdunison);
      //sign1 = vco1.Saw2_SyncFreq_2Pole_Realtime(SawFreq * Voct, Vsyncfreq, Vsyncfreq2, (int)Vunin, Vdunison);
      datL = sign1.l;
      datR = sign1.r;

      //filter

      //float now_ctof = (CutOff *= Vdecay) * Vctof;//这是自己的滤波器
      //datL = filt1_l.LPF2_Oversampling_ResoLimit_limit(datL, now_ctof, Vreso);
      //datR = filt1_r.LPF2_Oversampling_ResoLimit_limit(datR, now_ctof, Vreso);

      now_ctof = (CutOff *= Vdecay) * Vctof;  //更新一下ctof
      datL = filt1_l.proc(datL, filt1_l.GetIIRParam());
      datR = filt1_r.proc(datR, filt1_l.GetIIRParam());
      datL = filt2_l.proc(datL, filt2_l.GetIIRParam());
      datR = filt2_r.proc(datR, filt2_l.GetIIRParam());

      //delay
      datL = ldly.proc(datL, Vldt, Vdfdbk, Vdmix);
      datR = rdly.proc(datR, Vrdt, Vdfdbk, Vdmix);


      /*
      sint += SawFreq * TO_256; //fm test
      dat1 += lop1.sin256(sint+32.0*lop2.sin256(sint*2.0)) * 4096;
      dat2 += rop1.sin256(sint+32.0*rop2.sin256(sint*2.0)) * 4096;*/

      /*
      float kickv = drum1.kick(kickTrig) * 1.0;
      kickTrig = 0;

      dat1 += kickv;
      dat2 += kickv;
*/

      //datL = filt1_l.HPF1(datL,0.001);//隔直流
      //datR = filt1_r.HPF1(datR,0.001);

      int tmpL = datL * 2000.0;
      int tmpR = datR * 2000.0;
      WavBuf[i + 0] = tmpL;
      WavBuf[i + 1] = tmpR;
      //WavBuf[i + 0] = I32TOI24(datL);  //24bit
      //WavBuf[i + 1] = I32TOI24(datR);

      t += Vbpm;
      if (t >= 1.0) {
        if (isPlay) {
          SawFreq = loopfreq[looppos];
          if (loopgate[looppos] == 1) CutOff = 1.0;  //如果触发gate
          looppos++;
          if (looppos >= looplen) looppos = 0;
        }
        t -= 1.0;
      }
    }
  }
};
RPBass RPBassProc;

void EventTask(void *pvParameters) {  //我发现，把函数放一个类里面，两个核可以直接共享内存！！！！！
  for (int i = 0; i <= 400; ++i) {
    RPBassProc.UpdataADC();
  }
  for (;;) {
    RPBassProc.UpdataADC();    //更新adc
    RPBassProc.UpdataKB();     //更新键盘
    RPBassProc.UpdataAlgm();   //更新算法(这个还是单独放出来吧)
    RPBassProc.Count_Event++;  //记录次数，用来性能分析
  }
}
void Alg3Task(void *pvParameters) {  //更新滤波器的
  for (;;) {
    //RPBassProc.UpdataAlgm();  //更新算法(这个还是单独放出来吧)
    RPBassProc.Count_Alg3++;  //记录次数，用来性能分析
  }
}
void AlgorTask(void *pvParameters) {
  Serial.println(esp_get_idf_version());
  vTaskDelay(500);
  for (;;) {
    uint64_t sT = esp_timer_get_time();
    RPBassProc.proc();
    uint64_t eT = esp_timer_get_time();
    uint64_t PlayTime = wo.PlayAudio(WavBuf);  //播放buffer
    Serial.printf("All:%6d  Algm:%6d (%.2f%%)\n", (int)(eT - sT + PlayTime), (int)(eT - sT), (float)(eT - sT) / (eT - sT + PlayTime) * 100.0);
    RPBassProc.Count_Algor++;  //记录次数，用来性能分析

    int Count_Total = RPBassProc.Count_Event + RPBassProc.Count_Algor + RPBassProc.Count_Alg3;
    Serial.printf("Alg1:%2.2f%%  Event:%2.2f%%  Alg3:%2.2f%%\n", (double)RPBassProc.Count_Algor / Count_Total * 100.0, (double)RPBassProc.Count_Event / Count_Total * 100.0, (double)RPBassProc.Count_Alg3 / Count_Total * 100.0);

    //RPBassProc.debug();
  }
  wo.Stop();
}

void loop() {  //默认核，优先级无穷大，不使用
  for (;;)
    vTaskDelay(1000);
}