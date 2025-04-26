
#ifndef AUDIOTWOMICS_AUDIO_MAIN_H
#define AUDIOTWOMICS_AUDIO_MAIN_H
#define EPS 1.0e-7
#define FAC 3.051757812500000e-05f
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "Transform.h"

#include "mvdr2.h"
typedef struct MVDRVariables{
    // Timer* timer;
    double *topMicBuffer,*botMicBuffer;
    double *mixedBuffer;
    int stepSize,samplingRate,frameSize;
    int counter;
    bool isOn;
    
    int ake_counter;
    int count;
    int nFFT;
    //for dynamic input:
    short *mvdrmic_inbuffer,*inprocess_buffer;
    double *seprocessed_data,*finalout_buffer;
    int mvun_process_num,mvdrbuf_ready_num;
    mvdr_parameters *mvdr;
}MVDRVariables;

MVDRVariables* MainparamInitilization(int frequency, int stepSize,int frameSize,int fftsize,int buffer_size,bool isOn,float mic_daimeter,int look_direction,float max_gain);
void MVDR_mainProcessing(MVDRVariables *mainParam,short* originbuffer1,short* originbuffer2,int buf_len);
void MVDR_realtimeProcessing(MVDRVariables *mainParam);
void Destroymainparm(MVDRVariables *mainParam);
void MVDR_GETOUTPUT(MVDRVariables *mainParam,short* outbuffer,int buf_size);

#endif //AUDIOTWOMICS_AUDIO_MAIN_H
