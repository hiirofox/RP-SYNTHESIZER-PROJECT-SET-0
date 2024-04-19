#include <math.h>
#include "driver/i2s.h"
#include "esp_system.h"


#define SAMPLE_RATE 52000
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

  wo.Start();                                            //开启i2s
  WavBuf = (int32_t *)malloc(BufLen * sizeof(int32_t));  //初始化buffer
  /*
  stmpL = (float *)malloc(BufLen * sizeof(float) / 2);    //L buf
  stmpR = (float *)malloc(BufLen * sizeof(float) / 2);    //R buf
  tmp_f16 = (float *)malloc(BufLen * sizeof(float) / 2);  //tmp buf
  */
}

class CD4067_Input {
private:
  uint8_t pin[4];
  uint8_t pinSign;
  float dat[16];
  uint16_t ADCBuf[1024];
  uint8_t tmp[4];  //让s0s1s2s3同时更改
  Filter lpfs[16];
public:
  CD4067_Input(uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3, uint8_t Sign) {
    pin[0] = s0;
    pin[1] = s1;
    pin[2] = s2;
    pin[3] = s3;
    pinSign = Sign;
    for (int i = 0; i < 4; ++i)
      pinMode(pin[i], OUTPUT);
    pinMode(pinSign, INPUT);
  }
  void updata() {
    for (int i = 0; i < 16; ++i) {
      tmp[0] = (i >> 0) & 1;
      tmp[1] = (i >> 1) & 1;
      tmp[2] = (i >> 2) & 1;
      tmp[3] = (i >> 3) & 1;
      digitalWrite(pin[0], tmp[0]);
      digitalWrite(pin[1], tmp[1]);
      digitalWrite(pin[2], tmp[2]);
      digitalWrite(pin[3], tmp[3]);
      lpfs[i].LPF1(dat[i], 0.3, 0);
      dat[i] = analogRead(pinSign);
    }
  }
  float GetData(uint8_t n) {
    return lpfs[n].LPF1(dat[n], 0.3, 0);
  }
  float GetValueFloat(char id) {
    tmp[0] = (id >> 0) & 1;
    tmp[1] = (id >> 1) & 1;
    tmp[2] = (id >> 2) & 1;
    tmp[3] = (id >> 3) & 1;
    digitalWrite(pin[0], tmp[0]);
    digitalWrite(pin[1], tmp[1]);
    digitalWrite(pin[2], tmp[2]);
    digitalWrite(pin[3], tmp[3]);
    float val = 1.0 / 4095.0;
    val = val * analogRead(pinSign);
    return val;
  }
};
class HC595 {
private:
  uint8_t pinData, pinSCK, pinRCK;
public:
  HC595(uint8_t data, uint8_t sck, uint8_t rck) {
    pinData = data;
    pinSCK = sck;
    pinRCK = rck;
    pinMode(data, OUTPUT);
    pinMode(sck, OUTPUT);
    pinMode(rck, OUTPUT);
  }
  void SendByte(unsigned char SendVal) {
    for (int i = 0; i < 8; i++) {
      if ((SendVal << i) & 0x80) digitalWrite(pinData, 1);
      else digitalWrite(pinData, 0);
      digitalWrite(pinSCK, 0);
      digitalWrite(pinSCK, 1);
    }
    digitalWrite(pinRCK, 0);
    digitalWrite(pinRCK, 1);
  }
  void Light(unsigned char n) {
    n = 1 << n;
    for (int i = 0; i < 8; i++) {
      if ((n << i) & 0x80) digitalWrite(pinData, 1);
      else digitalWrite(pinData, 0);
      digitalWrite(pinSCK, 0);
      digitalWrite(pinSCK, 1);
    }
    digitalWrite(pinRCK, 0);
    digitalWrite(pinRCK, 1);
  }
};
HC595 led(6, 8, 7);                 //led
CD4067_Input knobs(1, 2, 3, 4, 5);  //旋钮
class RPDFAM {
private:
  Filter adcfilt1, adcfilt2, adcfilt3, adcfilt4;     //adc的滤波器
  Filter adcfilt5, adcfilt6, adcfilt7, adcfilt8;     //adc的滤波器
  Filter adcfilt9, adcfilt10, adcfilt11, adcfilt12;  //adc的滤波器


  float t = 0;  //音符更新
  float nowPitch, nowVol;
  float nowCtof = 0, t_ctof;
  float nowFreq = 0;
  int nowPos = 0;

  float Vbpm;
  float Vdtune;
  float Vctof;
  float Vctofdecay;
  float Vreso;

  float Vmxfreq;
  float Vfrdecay;

  VCO vco1;
  Filter filt1_L, filt1_R;
public:
  void debug() {
    Serial.printf("%d:%.5f,%.5f\n", nowPos, nowPitch, nowVol);
  }
  void UpdataAlgm() {
  }
  void UpdataADC() {
    float tmpv1 = (float)analogRead(9);
    float tmpv2 = (float)analogRead(10);
    float tmpv3 = (float)analogRead(11);
    float tmpv4 = (float)analogRead(12);
    float tmpv5 = (float)analogRead(13);
    float tmpv6 = (float)analogRead(14);
    float tmpv7 = (float)analogRead(15);

    Vbpm = (adcfilt1.LPF1(tmpv1, 0.1, 0) / 4095.0) * 200.0 / SAMPLE_RATE / 12.0;
    Vdtune = adcfilt2.LPF1(tmpv2, 0.3, 0) / 4095.0 * 1.0 + 0.0005;
    Vctof = adcfilt3.LPF1(tmpv3, 0.3, 0) / 4095.0 * 4.0;
    Vctof = Vctof * Vctof;
    Vctofdecay = 1.0 - pow(adcfilt4.LPF1(tmpv4, 0.3, 0) / 4095.0, 3.0) * 0.002;
    Vreso = adcfilt5.LPF1(tmpv5, 0.3, 0) / 4095.0;
    Vreso = sqrt(Vreso);
    Vmxfreq = adcfilt6.LPF1(tmpv6, 0.3, 0) / 4095.0 * 2000;
    Vfrdecay = 1.0 - pow(adcfilt7.LPF1(tmpv7, 0.3, 0) / 4095.0, 3.0) * 0.002;
  }
  void UpdataKB() {  //RPDFAM没有键盘！！！！！
    knobs.updata();
    nowPitch = (float)knobs.GetData(nowPos) / 4095.0;
    nowPitch = nowPitch * nowPitch * nowPitch * nowPitch * 700;
    nowVol = (float)knobs.GetData(nowPos + 8) / 4095.0;
    nowVol = nowVol * nowVol;
    t_ctof = nowVol * Vctof;
    t_ctof = t_ctof > 1.75 ? 1.75 : t_ctof;
  }
  void proc() {
    for (int i = 0; i < BufLen; i += 2) {
      float datL = 0.0, datR = 0.0;

      StereoSignal sign = vco1.Saw2_Stereo_Realtime(nowPitch + nowFreq, 3, Vdtune);
      datL = filt1_L.LPF2_Oversampling_ResoLimit_limit(sign.l, t_ctof * nowCtof, Vreso);
      datR = filt1_R.LPF2_Oversampling_ResoLimit_limit(sign.r, t_ctof * nowCtof, Vreso);
      nowCtof *= Vctofdecay;
      nowFreq *= Vfrdecay;
      int tmpL = datL * 10000.0;
      int tmpR = datR * 10000.0;
      WavBuf[i + 0] = tmpL;
      WavBuf[i + 1] = tmpR;


      t += Vbpm;
      if (t >= 1.0) {
        //干脆就在这里更新那组旋钮
        nowCtof = 1.0;
        nowFreq = Vmxfreq;
        nowPos = (nowPos + 1) % 8;
        led.Light(nowPos);
        t -= 1.0;
      }
    }
  }
};
RPDFAM RPDFAMProc;
void loop() {
  Serial.println(esp_get_idf_version());
  delay(100);
  for (int i = 0; i <= 400; ++i) {
    RPDFAMProc.UpdataADC();
    delay(1);
  }
  for (;;) {
    uint64_t sT = esp_timer_get_time();
    RPDFAMProc.proc();
    uint64_t eT = esp_timer_get_time();
    uint64_t PlayTime = wo.PlayAudio(WavBuf);  //播放buffer

    Serial.printf("All:%6d  Algm:%6d (%.2f%%)\n", (int)(eT - sT + PlayTime), (int)(eT - sT), (float)(eT - sT) / (eT - sT + PlayTime) * 100.0);

    RPDFAMProc.debug();
    RPDFAMProc.UpdataADC();   //更新adc
    RPDFAMProc.UpdataKB();    //更新键盘
    RPDFAMProc.UpdataAlgm();  //更新算法
  }
  //...
  wo.Stop();
}