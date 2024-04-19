#include <math.h>
#include "driver/i2s.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp32-hal-cpu.h"


#define SAMPLE_RATE 48000
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
void setup() {
  setCpuFrequencyMhz(240);
  Serial.begin(115200);

  for (int i = 1; i <= 21; ++i)
    pinMode(i, INPUT);

  wo.Start();                                            //开启i2s
  WavBuf = (int32_t *)malloc(BufLen * sizeof(int32_t));  //初始化buffer
}

const int MaxDatN = 200;
char dats[MaxDatN];
int posIn = 0;
char pinstate[22];
float sampt = 0,sampv = 1.0;
void loop() {c:\Users\Administrator\Documents\Arduino\DrumSerial\teensyDrum\teensyDrum.ino
  Serial.println(esp_get_idf_version());
  char a = 0, b = 0;
  for (;;) {
    uint64_t sT = esp_timer_get_time();
    for (int i = 0; i < BufLen; i += 2) {
      sampt += sampv;
      if (sampt >= 1.0) {
        sampt -= 1.0;
        int result = 0;
        for (int j = 0; j < 22; ++j) {
          result += dats[(posIn + j) % MaxDatN] && pinstate[j];
          a <<= 1, a += result % 2;
          b <<= 1, b += !(result % 2);
        }
        dats[(posIn + 22) % MaxDatN] = !(a%2);
        posIn++;
      }
      WavBuf[i + 0] = 1000000 * a;
      WavBuf[i + 1] = 1000000 * b;
    }
    uint64_t eT = esp_timer_get_time();
    uint64_t PlayTime = wo.PlayAudio(WavBuf);  //播放buffer
    Serial.printf("All:%6d  Algm:%6d (%.2f%%)\n", (int)(eT - sT + PlayTime), (int)(eT - sT), (float)(eT - sT) / (eT - sT + PlayTime) * 100.0);

    for (int i = 1; i < 22; ++i)
      pinstate[i - 1] = digitalRead(i);
  }
  wo.Stop();
}