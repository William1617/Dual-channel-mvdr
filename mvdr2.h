

#ifndef AUDIOTWOMICS_2_MVDR2_H
#define AUDIOTWOMICS_2_MVDR2_H

#include <stdio.h>

#include "Transform.h"

typedef struct mvdr_parameters {
    int num_channel,frame_len,stft_len,frame_shift, framecounter;
    double *sub_data1,*sub_data2,*mvproutput,*mvprwin,*mvproutput_final,*mvproutput_old,*mvdrsys_win;
    double  *mvZ_real1,*mvZ_img1,*mvZ_real2,*mvZ_img2;
  
    double energy,sum;

    int frame_count,num_stat;
    float sum1,determinant;
    int has_calweight;

    double  *rec_signal1_r,*rec_signal2_r,*rec_signal1_i,*rec_signal2_i;

    double  *global_covar_r1,*global_covar_r2,*global_covar_r3,*global_covar_r4,*corrvar_r1,*corrvar_r2,*corrvar_r3,*corrvar_r4,*w_r1,*w_r2,*global_covar_inv_r1,*global_covar_inv_r2,*global_covar_inv_r3,*global_covar_inv_r4,*global_covar_i1,*global_covar_i2,*global_covar_i3,*global_covar_i4,*corrvar_i1,*corrvar_i2,*corrvar_i3,*corrvar_i4,*w_i1,*w_i2,*global_covar_inv_i1,*global_covar_inv_i2,*global_covar_inv_i3,*global_covar_inv_i4;

    double temp,temp1,temp2,temp3;
    double *steering_vector_r1,*steering_vector_i1,*steering_vector_r2,*steering_vector_i2;

   // void(*enhance_signal)(struct mvdr_parameters *mvdr, double *input1, double *input2, int framecounter);

} mvdr_parameters;

void enhance_signal(mvdr_parameters *mvdr, double *input1, double *intput2, int framecounter);
mvdr_parameters* newparameters(int stft, int numstat, int framelen, int num_channel, int frame_shift);
void cal_steering_vector(mvdr_parameters *mvdr,int *mic_angle_vector,float mic_diameter,int look_direction,int sound_speed ,float max_gain,int audio_frequency);

void Destroy_mvpr(mvdr_parameters *mvdr);

#endif //AUDIOTWOMICS_2_MVDR2_H
