#include <math.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <esp32-hal-cpu.h>

#include <FreeRTOS.h>
#include <Task.h>

#define SAMPLE_RATE 48000
#define MaxPolyN (8)  //复音数

#include "ESP32WaveOut.h"
#include "HiiroFoxDSP.h"
#include "HiiroFoxDSP_II.h"
#include "KeyState.h"
#include "MidiState.h"

uint32_t TotalStackSize;  //任务栈总大小

//播放任务
void PlayAudioTask(void *);
TaskHandle_t hPlayAudio;
//Debug任务
void DebugTask(void *);
TaskHandle_t hDebug;
//控制任务
void CtrlTask(void *);
TaskHandle_t hCtrl;

WaveOut hWOut(SAMPLE_RATE);  //音频输出

const int BufLen = hWOut.GetBufferLen();                        //buf长度
int32_t *WavBuf = (int32_t *)malloc(BufLen * sizeof(int32_t));  //buf的指针，后面再分配

struct debug_t  //用来debug的数据类型
{
  bool flagPlay = 0;   //debug标记为播放任务
  uint64_t AlgorTime;  //算法所需时间
  uint64_t PlayTime;   //播放所需时间 (算法效率=算法时间/(播放时间+算法时间))

  bool flagCtrl = 0;       //debug标记为控制任务
  uint64_t CtrlCycleTime;  //一个周期的时间
};
struct param_t {        //用来控制向算法传递参数的类型(确保各任务数据访问不会冲突)
  float Vsyncfreq = 0;  //adc调参
  float Vdecay = 0;
  float Vbpm = 0;
  float Vctof = 0;
  float Vreso = 0;
  float Vdunison = 0;
  float Vunin = 0;
  //float Vldt = 0, Vrdt = 0, Vdfdbk = 0, Vdmix = 0;
  float Va, Vd, Vs, Vr;
  float Voct = 0;

  //以下是提前计算好的，保存参数时没有也无所谓
  IIRFilter FiltParam1[MaxPolyN];  //不带反馈的滤波器参数
  IIRFilter FiltParam2[MaxPolyN];  //带反馈的
  float VoctP2;                    //乘这个频率翻倍
};

QueueHandle_t hqDebug = xQueueCreate(64, sizeof(debug_t));  //数据队列
QueueHandle_t hqParam = xQueueCreate(64, sizeof(param_t));

void setup() {
  Serial.begin(115200);

  memset(WavBuf, 0, sizeof(int32_t) * BufLen);

  MidiStart();    //启动midi
  hWOut.Start();  //启动waveout

  delay(200);  //等待一下准备
  TotalStackSize = uxTaskGetStackHighWaterMark(NULL);

  xTaskCreatePinnedToCore(PlayAudioTask, "play_audio", TotalStackSize, NULL, 100, &hPlayAudio, 0);  //0核跑播放和算法
  //xTaskCreatePinnedToCore(DebugTask, "debug", TotalStackSize, NULL, 10, &hDebug, 1);                //1核跑debug任务
  xTaskCreatePinnedToCore(CtrlTask, "control", TotalStackSize, NULL, 0, &hCtrl, 1);                 //1核跑控制任务
  //关于为什么control的优先级比debug低，因为debug不需要很高的频率，基本上一直delay相当于空闲。
  xTaskCreatePinnedToCore(MidiUpdata, "midi_updata", TotalStackSize, NULL, 100, &hMidiUpdata, 1);  //1核跑midi更新(这个函数在MidiState.h)
}

//数据区 算法
VCO vco1[MaxPolyN];
Filter hp1_l[MaxPolyN], hp1_r[MaxPolyN];  //振荡器输出隔直流的高通
IIRFilter filt1_l[MaxPolyN], filt1_r[MaxPolyN];
IIRFilter filt2_l[MaxPolyN], filt2_r[MaxPolyN];
IIRFilter filt3_l[MaxPolyN], filt3_r[MaxPolyN];

//HF_Delay ldly, rdly;

ADSR adsr[MaxPolyN];  //adsr

int PolyPitch[MaxPolyN];   //存Pitch编号,方便音符release的时候找振荡器
float PolyHz[MaxPolyN];    //音调(hz)
float PolyCtof[MaxPolyN];  //截止频率(打算和velocity链接上)
int CycleIn = 0;           //循环到哪了
////////////

void PlayAudioTask(void *param) {  //播放任务
  debug_t dbg;
  param_t prm;
  midistate_t mid;
  for (;;) {
    while (uxQueueMessagesWaiting(hqParam)) {  //队列里获取参数
      xQueueReceive(hqParam, &prm, 0);         //一次读完就结束了
    }
    while (uxQueueMessagesWaiting(hqMidi)) {  //队列里获取midi事件
      xQueueReceive(hqMidi, &mid, 0);         //但是这个还要处理
      if (mid.command == 0x90) {
        PolyPitch[CycleIn] = mid.pitch;  //获取pitch编号
        PolyHz[CycleIn] = mid.pitchHz * prm.VoctP2;
        PolyCtof[CycleIn] = 1.0;
        adsr[CycleIn].SetGate(0);  //先松开先
        adsr[CycleIn].SetGate(1);  //再设置按下
        CycleIn = (CycleIn + 1) % MaxPolyN;
      } else if (mid.command == 0x80) {  //释放
        for (int i = 0; i < MaxPolyN; ++i) {
          if (PolyPitch[i] == mid.pitch) {  //找原本那个poly来释放
            adsr[i].SetGate(0);
          }
        }
      }
    }
    uint64_t tStart = esp_timer_get_time();
    for (int i = 0; i < BufLen; i += 2) {
      float datL = 0, datR = 0;
      StereoSignal sign;
      float tmpL, tmpR;
      for (int j = 0; j < MaxPolyN; ++j) {
        sign = vco1[j].Saw2_SyncFreq_Realtime_SR65536(PolyHz[j], prm.Vsyncfreq, (int)prm.Vunin, prm.Vdunison);
        //float adsrv = adsr[j].proc();
        tmpL = sign.l;
        tmpR = sign.r;

        //tmpL = hp1_l[j].HPF1(tmpL, 0.001);//其实也没必要
        //tmpR = hp1_r[j].HPF1(tmpR, 0.001);

        tmpL = filt1_l[j].proc(tmpL, &prm.FiltParam2[j]);
        tmpR = filt1_r[j].proc(tmpR, &prm.FiltParam2[j]);
        //tmpL = filt2_l[j].proc(tmpL, &prm.FiltParam2[j]);
        //tmpR = filt2_r[j].proc(tmpR, &prm.FiltParam2[j]);
        //tmpL = filt3_l[j].proc(tmpL, &prm.FiltParam1[j]);
        //tmpR = filt3_r[j].proc(tmpR, &prm.FiltParam1[j]);

        datL += tmpL;
        datR += tmpR;
      }

      //datL = ldly.proc(datL, prm.Vldt, prm.Vdfdbk, prm.Vdmix);
      //datR = rdly.proc(datR, prm.Vrdt, prm.Vdfdbk, prm.Vdmix);

      WavBuf[i + 0] = datL * 2500.0;  //填入缓存
      WavBuf[i + 1] = datR * 2500.0;
    }
    //for (int j = 0; j < MaxPolyN; ++j) {  //ctof的decay
    //  PolyCtof[j] *= prm.Vdecay;
    //}

    uint64_t tEnd = esp_timer_get_time();
    uint64_t PlayTime = hWOut.PlayAudio(WavBuf);  //播放buffer

    dbg.flagPlay = true;
    dbg.AlgorTime = tEnd - tStart;  //这里是debug信息处理
    dbg.PlayTime = PlayTime;
    xQueueSend(hqDebug, &dbg, 0);  //发送debug消息到队列
  }
}

void DebugTask(void *param) {
  debug_t dbg;
  uint64_t AlgorTime = 0, TotalPlayTime = 0;
  uint64_t CtrlTime = 0;
  int CountFlagPlay = 0;
  for (;;) {  //之后日志就在这里打印好了
    AlgorTime = 0;
    TotalPlayTime = 0;
    CtrlTime = 0;
    int CountFlagPlay = 0;
    while (uxQueueMessagesWaiting(hqDebug)) {
      xQueueReceive(hqDebug, &dbg, 0);
      if (dbg.flagPlay) {
        AlgorTime += dbg.AlgorTime;
        TotalPlayTime += dbg.AlgorTime + dbg.PlayTime;
        CountFlagPlay++;
      }
      if (dbg.flagCtrl) {
        CtrlTime = dbg.CtrlCycleTime;
      }
    }
    if (CountFlagPlay != 0) {
      AlgorTime /= CountFlagPlay;
      TotalPlayTime /= CountFlagPlay;
    }
    Serial.printf("\n\n");
    Serial.printf("algorithm performance:%2.2f%%\n", (float)AlgorTime / TotalPlayTime * 100.0);
    Serial.printf("control   performance:%02.2fX\n", (float)CtrlTime / TotalPlayTime);
    int TotalUsed = 3 * TotalStackSize - uxTaskGetStackHighWaterMark(hPlayAudio) - uxTaskGetStackHighWaterMark(hDebug) - uxTaskGetStackHighWaterMark(hCtrl);
    Serial.printf("All Task   : %d/%d (%2.2f%%)\n", TotalUsed, TotalStackSize, (float)TotalUsed / TotalStackSize * 100.0);
    Serial.printf("-play_audio: %d\n", TotalStackSize - uxTaskGetStackHighWaterMark(hPlayAudio));
    Serial.printf("-debug     : %d\n", TotalStackSize - uxTaskGetStackHighWaterMark(hDebug));
    Serial.printf("-control   : %d\n", TotalStackSize - uxTaskGetStackHighWaterMark(hCtrl));
    vTaskDelay(250);
  }
}

ScanKB skb;  //外设键盘控制的类

const float ADCUpdataRate = 0.1;                   //adc过低通的平滑度
Filter adcfilt1, adcfilt2, adcfilt3, adcfilt4;     //adc的滤波器
Filter adcfilt5, adcfilt6, adcfilt7, adcfilt8;     //adc的滤波器
Filter adcfilt9, adcfilt10, adcfilt11, adcfilt12;  //adc的滤波器
float Vsyncfreq = 0;                               //adc调参用的临时变量(算法任务不要用)
float Vdecay = 0;
float Vbpm = 0;
float Vctof = 0;
float Vreso = 0;
float Vdunison = 0;
float Vunin = 0;
//float Vldt = 0, Vrdt = 0, Vdfdbk = 0, Vdmix = 0;
float Va, Vd, Vs, Vr;  //adsr
float Voct = 0, VoctP2 = 0;
bool KBInfo[16];  //记录键盘按下的情况

midistate_t loopdat[256];
int looppos = 0;
int playpos = 0;
int isplay = 0;
midistate_t lastnote;
midistate_t nownote;
float loopv = 0;
float pitchmul = 1;
void CtrlTask(void *param) {
  debug_t dbg;
  param_t prm;
  midistate_t mid;
  uint64_t lastTime = 0;
  for (;;) {
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
    Vdunison = adcfilt1.LPF1(tmpv1, ADCUpdataRate, 0) / 4095.0 * 0.5 + 0.005;
    Vunin = adcfilt2.LPF1(tmpv2, ADCUpdataRate, 0) / 4095.0 * 5.0 + 1.0;
    Vsyncfreq = adcfilt3.LPF1(tmpv3, ADCUpdataRate, 0) / 4095.0 * 7.0 + 1.0;
    //Vsyncfreq2 = adcfilt4.LPF1(tmpv4, ADCUpdataRate, 0) / 4095.0 * 7.0 + 1.0;
    //Vwavet = adcfilt4.LPF1(tmpv4, ADCUpdataRate, 0) / 4095.0 * (SAMPLE_RATE - 1) / 2.0;
    //Vkbcf = adcfilt4.LPF1(tmpv4, ADCUpdataRate, 0) / 4095.0;
    //Vbpm = (adcfilt4.LPF1(tmpv4, ADCUpdataRate, 0) / 4095.0 + 0.2) * 200.0 / SAMPLE_RATE / 12.0;
    Vbpm = adcfilt4.LPF1(tmpv4, ADCUpdataRate, 0) / 4095.0;
    Vbpm *= Vbpm * 0.125;
    Vdecay = 1.0 - pow(adcfilt5.LPF1(tmpv5, ADCUpdataRate, 0) / 4095.0, 3.0) * 0.5;
    Voct = (int)(adcfilt6.LPF1(tmpv6, ADCUpdataRate, 0) / 4095.0 * 8.0);
    VoctP2 = pow(2, Voct - 3);

    //filter
    //Vctof = adcfilt5.LPF1(tmpv5, 0.3, 0) / 4096.0 * sqrt(2.0);
    Vctof = adcfilt7.LPF1(tmpv7, ADCUpdataRate, 0) / 4095.0;  //1.85
    Vctof = Vctof * Vctof;                                    //指数音高
    Vreso = adcfilt8.LPF1(tmpv8, ADCUpdataRate, 0) / 4095.0;

    //delay
    /*
    Vldt = (int)(adcfilt9.LPF1(tmpv9, ADCUpdataRate, 0) / 4095.0 * 15.0 + 1.0) * 800;
    Vrdt = (int)(adcfilt10.LPF1(tmpv10, ADCUpdataRate, 0) / 4095.0 * 15.0 + 1.0) * 800;
    Vdfdbk = adcfilt11.LPF1(tmpv11, ADCUpdataRate, 0) / 4095.0 * 0.9;
    Vdmix = adcfilt12.LPF1(tmpv12, ADCUpdataRate, 0) / 4095.0;
*/
    Va = adcfilt9.LPF1(tmpv9, ADCUpdataRate, 0) / 4095.0;  //adsr
    Va = Va * Va * Va * Va * 0.5;
    Vd = adcfilt10.LPF1(tmpv10, ADCUpdataRate, 0) / 4095.0;
    Vd = Vd * Vd * Vd * Vd * 0.5;
    Vs = adcfilt11.LPF1(tmpv11, ADCUpdataRate, 0) / 4095.0;
    Vr = adcfilt12.LPF1(tmpv12, ADCUpdataRate, 0) / 4095.0;
    Vr = Vr * Vr * Vr * Vr * 0.5;  //都是为了平滑过渡

    //键盘部分
    skb.scan();  //更新键盘
    //if (skb.KeyState(0) && skb.KeyState(1) && skb.KeyState(2)) esp_restart();  //软复位
    for (int i = 0; i < 13; ++i) {
      bool info = skb.KeyState(i);
      if (KBInfo[i] != info) {  //此时 键盘有更新
        KBInfo[i] = info;
        mid.pitch = i;                                                 //pitch
        mid.pitchHz = 110.0 * pow(2.0, 1.0 / 12.0 * (mid.pitch + 3));  //转成hz
        mid.velocity = 1.0;                                            //力度设为1.0
        if (info == true) {                                            //如果是按下
          mid.command = 0x90;
        } else {  //松开
          mid.command = 0x80;
        }
        if (isplay == 0) {
          lastnote = mid;
          lastnote.command = 0x90;
          xQueueSend(hqMidi, &mid, 0);  //发送midi事件到队列
        } else {
          pitchmul = pow(2.0, (float)i / 12.0);
        }
      }
    }
    //音序器
    bool info = skb.KeyState(13);
    if (KBInfo[13] != info) {  //none
      KBInfo[13] = info;
      if (info) {
        lastnote.command = 0x80;
      }
    }
    info = skb.KeyState(14);
    if (KBInfo[14] != info) {  //play
      KBInfo[14] = info;
      if (info) {
        playpos = 0;
        isplay = !isplay;
        if (!isplay) {
          nownote.command = 0x80;
          xQueueSend(hqMidi, &nownote, 0);  //最后一个别响了
        }
      }
    }
    info = skb.KeyState(15);
    if (KBInfo[15] != info) {  //next
      KBInfo[15] = info;
      if (info) {
        loopdat[looppos] = lastnote;
        looppos++;
      }
    }
    if (skb.KeyState(13) && skb.KeyState(15)) looppos = 0;
    if (isplay) {
      loopv += Vbpm;
      if (loopv >= 1.0) {
        loopv -= (int)loopv;
        nownote.command = 0x80;
        xQueueSend(hqMidi, &nownote, 0);  //发送midi事件到队列
        nownote = loopdat[playpos];
        nownote.pitchHz *= pitchmul;
        xQueueSend(hqMidi, &nownote, 0);  //发送midi事件到队列
        playpos++;
        if (playpos >= looppos) {
          playpos = 0;
        }
      }
    }

    //param
    prm.Vsyncfreq = Vsyncfreq;  //adc调参
    prm.Vdecay = Vdecay;
    prm.Vbpm = Vbpm;
    prm.Vctof = Vctof;
    prm.Vreso = Vreso;
    prm.Vdunison = Vdunison;
    prm.Vunin = Vunin;
    /*
    prm.Vldt = Vldt;
    prm.Vrdt = Vrdt;
    prm.Vdfdbk = Vdfdbk;
    prm.Vdmix = Vdmix;*/
    prm.Va = Va, prm.Vd = Vd, prm.Vs = Vs, prm.Vr = Vr;  //adsr
    prm.Voct = Voct;
    prm.VoctP2 = VoctP2;


    for (int j = 0; j < MaxPolyN; ++j) {                                                      //在这里更新滤波器的参数，真麻烦(而且数据可能冲突，幸好是读取)
      adsr[j].SetADSR(Va, Vd, Vs, Vr);                                                        //我居然在这里更新ADSR
      prm.FiltParam1[j].UpdataLPF(Vctof * adsr[j].proc() + 50.0 / SAMPLE_RATE, 0);            //更新一下滤波器参数
      prm.FiltParam2[j].UpdataLPF(Vctof * adsr[j].proc() + 50.0 / SAMPLE_RATE, Vreso * 8.0);  //因为滤波器参数在这里更了，ADSR又是绑一起的，直接在这里一起更了
    }                                                                                         //我真不懂放哪合适
    xQueueSend(hqParam, &prm, 0);                                                             //发送debug消息到队列

    //debug
    uint64_t nowTime = esp_timer_get_time();
    dbg.flagCtrl = true;
    dbg.CtrlCycleTime = nowTime - lastTime;
    lastTime = nowTime;
    xQueueSend(hqDebug, &dbg, 0);  //发送debug消息到队列
  }
}
void loop() {  //无用的东西，连打印日志都不配
  for (;;) vTaskDelay(100000);
}

//midi指令对照表
// 0x80     Note Off
// 0x90     Note On
// 0xA0     Aftertouch
// 0xB0     Continuous controller
// 0xC0     Patch change
// 0xD0     Channel Pressure
// 0xE0     Pitch bend
// 0xF0     (non-musical commands)
