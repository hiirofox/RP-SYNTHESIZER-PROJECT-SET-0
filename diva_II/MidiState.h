#ifndef _MIDISTATE_
#define _MIDISTATE_

#include <Adafruit_TinyUSB.h>
#include <MIDI.h>


struct midistate_t {
  int pitch;
  float pitchHz;
  float velocity;
  int command;
};
QueueHandle_t hqMidi = xQueueCreate(64, sizeof(midistate_t));  //(这个的类型在MidiState.h)

Adafruit_USBD_MIDI usb_midi;
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

void _callbackMidiNoteOn(byte channel, byte pitch, byte velocity) {
  midistate_t mid;
  mid.pitch = pitch;
  mid.velocity = (float)velocity / 127.0;
  mid.command = 0x90;
  mid.pitchHz = 110.0 * pow(2.0, 1.0 / 12.0 * (mid.pitch + 3 - 12 * 4));  //转成hz
  xQueueSend(hqMidi, &mid, 0);                                            //发送midi事件到队列
}

void _callbackMidiNoteOff(byte channel, byte pitch, byte velocity) {
  midistate_t mid;
  mid.pitch = pitch;
  mid.velocity = (float)velocity / 127.0;
  mid.command = 0x80;
  mid.pitchHz = 110.0 * pow(2.0, 1.0 / 12.0 * (mid.pitch + 3 - 12 * 4));  //转成hz
  xQueueSend(hqMidi, &mid, 0);                                            //发送midi事件到队列
}

void MidiStart() {  //midi 启动！
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.setHandleNoteOn(_callbackMidiNoteOn);
  MIDI.setHandleNoteOff(_callbackMidiNoteOff);
}

void MidiUpdata(void *param) {  //更新midi数据
  for (;;) {
    MIDI.read();
    vTaskDelay(2);  //至少给其他线程刷新一下吧
  }
}
TaskHandle_t hMidiUpdata;//这个的句柄

#endif