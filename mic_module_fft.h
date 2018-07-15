
#ifndef _MIC_MODULE_FFT_
#define _MIC_MODULR_FFT_
#define N_SAMPLES 128
#include <fix_fft.h>
#include <TimeLib.h>//Time.h→TimeLib.h
#include "Arduino.h"

class MIC {
  public:
    MIC(int MIC_PIN);

    
    void FFT();//録音→fftまでやってくれるマン
    void dispData(char *inMsg, char *inData, int inN);//音の高さごとの音の大きさを表示
    void soundRead();//最大音量とかを拾う

    
    char buf[20];            // 文字出力バッファ
    char im[N_SAMPLES];      // 虚数部
    char data[N_SAMPLES];    // 入力/実数部
    int maxvol;              // 最大音量
    int maxfreq;             // 最大音の高さ
  private:
    int mic_pin;

};

#endif
