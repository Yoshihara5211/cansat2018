#include "mic_module_fft.h"

MIC::MIC(int _MIC_PIN){
  mic_pin=_MIC_PIN;
  pinMode(mic_pin,INPUT);
  analogReference(DEFAULT);
}
void MIC::dispData(char *inMsg, char *inData, int inN) {
  Serial.print(inMsg);
  for (int i = inN/2 ; i < inN; i++) {
    sprintf(buf, "%5d", inData[i]);
    Serial.print(buf);
  }
}
void MIC::FFT(){
  int  i;
  for (i = 0; i < N_SAMPLES; i++) {
    // 音声を読み込み
    data[i] = (analogRead(mic_pin) >> 2) - 128;

    // 虚数部をクリア
    im[i]   = 0;
  }
  // FFT
  fix_fft(data, im, 7, 0);  // full scale 2^7=128, FFT mode
  for (int i = 0; i < N_SAMPLES / 2; i++) {
    data[i] = sqrt(data[i] * data[i] + im[i] * im[i]);
  }
}

