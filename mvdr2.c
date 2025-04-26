

#include "mvdr2.h"
//#include "mvdr.h"
mvdr_parameters* newparameters(int stft, int numstat, int framelen, int num_channel,  int frame_shift)
{
    mvdr_parameters* newparameters = (mvdr_parameters*)malloc(sizeof(mvdr_parameters));
    newparameters->framecounter=0;
    newparameters->frame_shift = frame_shift;
    newparameters->num_channel = num_channel;
    newparameters->stft_len = stft;
    newparameters->num_stat = numstat;
    newparameters->frame_len = framelen;

    newparameters->sub_data1 = (double*)calloc(framelen,sizeof(double));
    newparameters->sub_data2 = (double*)calloc(framelen, sizeof(double));
    newparameters->mvproutput = (double*)calloc(framelen,sizeof(double));
    newparameters->mvZ_real1 = (double*)calloc(stft,sizeof(double));
    newparameters->mvZ_real2 = (double*)calloc(stft,sizeof(double));
    newparameters->mvZ_img1 = (double*)calloc(stft,sizeof(double));
    newparameters->mvZ_img2 = (double*)calloc(stft,sizeof(double));
    newparameters->corrvar_r1 = (double*)calloc(stft/2+1,sizeof(double));
    newparameters->corrvar_r2 = (double*)calloc(stft/2+1,sizeof(double));
    newparameters->corrvar_r3 = (double*)calloc(stft/2+1,sizeof(double));
    newparameters->corrvar_r4 = (double*)calloc(stft/2+1,sizeof(double));

    newparameters->corrvar_i1 = (double*)calloc(stft/2+1,sizeof(double));
    newparameters->corrvar_i2 = (double*)calloc(stft/2+1,sizeof(double));
    newparameters->corrvar_i3 = (double*)calloc(stft/2+1,sizeof(double));
    newparameters->corrvar_i4 = (double*)calloc(stft/2+1,sizeof(double));;

    newparameters->global_covar_r1 = (double*)calloc(stft/2+1,sizeof(double));
    newparameters->global_covar_r2 = (double*)calloc(stft/2+1,sizeof(double));
    newparameters->global_covar_r3 = (double*)calloc(stft/2+1,sizeof(double));
    newparameters->global_covar_r4 = (double*)calloc(stft/2+1,sizeof(double));

    newparameters->global_covar_i1= (double*)calloc(stft/2+1,sizeof(double));
    newparameters->global_covar_i2= (double*)calloc(stft/2+1,sizeof(double));
    newparameters->global_covar_i3= (double*)calloc(stft/2+1,sizeof(double));
    newparameters->global_covar_i4= (double*)calloc(stft/2+1,sizeof(double));

    newparameters->w_r1 = (double*)calloc(stft/2+1,sizeof(double));
    newparameters->w_r2 = (double*)calloc(stft/2+1,sizeof(double));
    memset(newparameters->w_r1,1,(stft/2+1)*sizeof(double));
    memset(newparameters->w_r2,1,(stft/2+1)*sizeof(double));

    newparameters->global_covar_inv_r1 = (double*)calloc(stft/2+1,sizeof(double));
    newparameters->global_covar_inv_r2 = (double*)calloc(stft/2+1,sizeof(double));
    newparameters->global_covar_inv_r3 = (double*)calloc(stft/2+1,sizeof(double));
    newparameters->global_covar_inv_r4 = (double*)calloc(stft/2+1,sizeof(double));


    newparameters->w_i1= (double*)calloc(stft/2+1,sizeof(double));
    newparameters->w_i2= (double*)calloc(stft/2+1,sizeof(double));
    memset(newparameters->w_i1,0,(stft/2+1)*sizeof(double));
    memset(newparameters->w_i2,0,(stft/2+1)*sizeof(double));

    newparameters-> global_covar_inv_i1= (double*)calloc(stft/2+1,sizeof(double));
    newparameters-> global_covar_inv_i2= (double*)calloc(stft/2+1,sizeof(double));
    newparameters-> global_covar_inv_i3= (double*)calloc(stft/2+1,sizeof(double));
    newparameters-> global_covar_inv_i4= (double*)calloc(stft/2+1,sizeof(double));

    newparameters->mvprwin = (double*)calloc(stft, sizeof(double));

    newparameters->mvproutput_final = (double*)calloc(framelen,sizeof(double));
    

    newparameters->determinant = 0;
    //double complex z3[stft_len];
    //double complex z4[stft_len];
    newparameters->rec_signal1_r = (double*)calloc(stft, sizeof(double));
    newparameters->rec_signal2_r= (double*)calloc(stft, sizeof(double));

    newparameters->rec_signal1_i = (double*)calloc(stft, sizeof(double));
    newparameters->rec_signal2_i = (double*)calloc(stft, sizeof(double));

    newparameters->steering_vector_r1 = (double*)calloc(stft ,sizeof(double));
    newparameters->steering_vector_i1 = (double*)calloc(stft ,sizeof(double));
    newparameters->steering_vector_r2 = (double*)calloc(stft ,sizeof(double));
    newparameters->steering_vector_i2 = (double*)calloc(stft ,sizeof(double));
    memset(newparameters->steering_vector_r1,0,stft*sizeof(double));
    memset(newparameters->steering_vector_i1,0,stft*sizeof(double));
    memset(newparameters->steering_vector_r2,0,stft*sizeof(double));
    memset(newparameters->steering_vector_i2,0,stft*sizeof(double));
    
    int i=0;

  //  newparameters->enhance_signal = enhance_signal;
    for (i = 0; i < stft; i++)
    {
        newparameters->mvprwin[i] = 0.5 * (1 - cosf(2 * 3.1415926535*i  / (stft - 1)));
    }
   // for (i = 0; i < stft; i++)
   // {
       // newparameters->mvdrsys_win[i] = 1 / (mvdr->mvprwin[i] + mvdr->mvprwin[i + mvdr->frame_len]);
   // }

    return newparameters;

}
void cal_steering_vector(mvdr_parameters *mvdr,int *mic_angle_vector,float mic_diameter,int look_direction,int sound_speed,float max_gain,int audio_frequency){
    int j=0;
    double betad1=cosf((look_direction-mic_angle_vector[0])*3.1415926536/180);
    double betad2 =cosf((look_direction-mic_angle_vector[1])*3.1415926536/180);
   // double betad2 =sinf(look_direction*3.1415926536/180);
    double diameter=mic_diameter;
    for (j=0;j<mvdr->stft_len;j++){
        double frequency=audio_frequency*j/mvdr->stft_len;
        double wk=2*3.1415926536*frequency/sound_speed;
        mvdr->steering_vector_r1[j]=cosf(wk*betad1*diameter)/max_gain;
        mvdr->steering_vector_r2[j]=cosf(wk*betad2*diameter)/max_gain;
        mvdr->steering_vector_i1[j]=-sinf(wk*betad1*diameter)/max_gain;
        mvdr->steering_vector_i2[j]=-sinf(wk*betad2*diameter)/max_gain;
        
        if(j<mvdr->stft_len/2+1){
            mvdr->w_r1[j]=cosf(wk*betad1*diameter)*max_gain/2;
            mvdr->w_r2[j]=cosf(wk*betad2*diameter)*max_gain/2;
            mvdr->w_i1[j]=-sinf(wk*betad1*diameter)*max_gain/2;
            mvdr->w_i2[j]=-sinf(wk*betad2*diameter)*max_gain/2;
        }
    }

}
void enhance_signal(mvdr_parameters *mvdr, double *input1, double *input2, int framecounter)
{
    Transform *X1;
    Transform *X2;
    Transform *Y;
    
    int i=0;
    int m=0;
    int k1=0;
    
    X1=newTransform(mvdr->stft_len);
    X2=newTransform(mvdr->stft_len);
    Y = newTransform(mvdr->stft_len);
    double instantsum1=0;
    double instantsum2=0;
    double direction_check=0;
    double alpha=0.01;
    for (i = 0; i < mvdr->frame_len; i++){
        mvdr->sub_data1[i] = input1[i]*mvdr->mvprwin[i];
        mvdr->sub_data2[i] = input2[i] *mvdr->mvprwin[i];

    }
        
    mvdrFFT(X1,mvdr->sub_data1);
    mvdrFFT(X2,mvdr->sub_data2);
    for (i = 0; i < mvdr->stft_len; i++){
            
        mvdr->mvZ_real1[i]=X1->real[i];
        mvdr->mvZ_img1[i]=X1->imaginary[i];
        mvdr->mvZ_real2[i] = X2->real[i];
        mvdr->mvZ_img2[i] = X2->imaginary[i];

    }

    for(m = 0; m< (mvdr->stft_len/2)+1;m++){
        mvdr->corrvar_r1[m]=(mvdr->mvZ_real1[m]*mvdr->mvZ_real1[m])+(mvdr->mvZ_img1[m]*mvdr->mvZ_img1[m]);
        mvdr->corrvar_r2[m]=(mvdr->mvZ_real1[m]*mvdr->mvZ_real2[m])+(mvdr->mvZ_img1[m]*mvdr->mvZ_img2[m]);
        mvdr->corrvar_r3[m]=(mvdr->mvZ_real2[m]*mvdr->mvZ_real1[m])+(mvdr->mvZ_img2[m]*mvdr->mvZ_img1[m]);
        mvdr->corrvar_r4[m]=(mvdr->mvZ_real2[m]*mvdr->mvZ_real2[m])+(mvdr->mvZ_img2[m]*mvdr->mvZ_img2[m]);

        mvdr->corrvar_i1[m]=-(mvdr->mvZ_real1[m]*mvdr->mvZ_img1[m])+(mvdr->mvZ_img1[m]*mvdr->mvZ_real1[m]);
        mvdr->corrvar_i2[m]=-(mvdr->mvZ_real1[m]*mvdr->mvZ_img2[m])+(mvdr->mvZ_img1[m]*mvdr->mvZ_real2[m]);
        mvdr->corrvar_i3[m]=-(mvdr->mvZ_real2[m]*mvdr->mvZ_img1[m])+(mvdr->mvZ_img2[m]*mvdr->mvZ_real1[m]);
        mvdr->corrvar_i4[m]=-(mvdr->mvZ_real2[m]*mvdr->mvZ_img2[m])+(mvdr->mvZ_img2[m]*mvdr->mvZ_real2[m]);  
    }
    for(m = 0; m< (mvdr->stft_len/2)+1;m++) {
        mvdr->global_covar_r1[m] = alpha*mvdr->corrvar_r1[m] +(1-alpha)*mvdr->global_covar_r1[m];
        mvdr->global_covar_r2[m] = alpha*mvdr->corrvar_r2[m] +(1-alpha)*mvdr->global_covar_r2[m];
        mvdr->global_covar_r3[m] = alpha*mvdr->corrvar_r3[m] +(1-alpha)*mvdr->global_covar_r3[m];
        mvdr->global_covar_r4[m] = alpha*mvdr->corrvar_r4[m] +(1-alpha)*mvdr->global_covar_r4[m];
        mvdr->global_covar_i1[m] = alpha*mvdr->corrvar_i1[m] +(1-alpha)*mvdr->global_covar_i1[m];
        mvdr->global_covar_i2[m] = alpha*mvdr->corrvar_i2[m] +(1-alpha)*mvdr->global_covar_i2[m];
        mvdr->global_covar_i3[m] = alpha*mvdr->corrvar_i3[m] +(1-alpha)*mvdr->global_covar_i3[m];
        mvdr->global_covar_i4[m] = alpha*mvdr->corrvar_i4[m] +(1-alpha)*mvdr->global_covar_i4[m];
    }
        
    if (framecounter>=mvdr->num_stat && framecounter%50==10 )
    {
        double mvdr_ar1 =0;
        double mvdr_ai1 =0;
        double mvdr_ar2 =0;
        double mvdr_ai2 =0;

        double mvdr_b =0;
        double global_covsum1=0;
        double global_covsum2=0;
        double lambda=1e-5;
    
        for (m = 0; m < (mvdr->stft_len / 2) + 1; m++) {

            global_covsum1 +=mvdr->global_covar_r1[m];
            global_covsum2 +=mvdr->global_covar_r4[m];
        }
       
        if(global_covsum1>1 && global_covsum2>1 ){
            for(k1 =0;k1<mvdr->stft_len/2+1;k1++)
            {
                mvdr->determinant = ((mvdr->global_covar_r1[k1]+lambda)*(mvdr->global_covar_r4[k1]+lambda))-((mvdr->global_covar_r2[k1]*mvdr->global_covar_r3[k1])-(mvdr->global_covar_i2[k1]*mvdr->global_covar_i3[k1]));
                
                mvdr->global_covar_inv_r1[k1]=(mvdr->global_covar_r4[k1]+lambda)/mvdr->determinant;
                mvdr->global_covar_inv_r2[k1]=-mvdr->global_covar_r2[k1]/mvdr->determinant;
                mvdr->global_covar_inv_r3[k1]=-mvdr->global_covar_r3[k1]/mvdr->determinant;
                mvdr->global_covar_inv_r4[k1]=(mvdr->global_covar_r1[k1]+lambda)/mvdr->determinant;
                
                mvdr->global_covar_inv_i1[k1]=mvdr->global_covar_i4[k1]/mvdr->determinant;
                mvdr->global_covar_inv_i2[k1]=-mvdr->global_covar_i2[k1]/mvdr->determinant;
                mvdr->global_covar_inv_i3[k1]=-mvdr->global_covar_i3[k1]/mvdr->determinant;
                mvdr->global_covar_inv_i4[k1]=mvdr->global_covar_i1[k1]/mvdr->determinant;

                mvdr_ar1=mvdr->global_covar_inv_r1[k1]*mvdr->steering_vector_r1[k1] + mvdr->steering_vector_i1[k1]*mvdr->global_covar_inv_i1[k1]+mvdr->global_covar_inv_r3[k1]*mvdr->steering_vector_r2[k1] +mvdr->steering_vector_i2[k1]*mvdr->global_covar_inv_i3[k1];
                mvdr_ai1=-mvdr->global_covar_inv_r1[k1]*mvdr->steering_vector_i1[k1] +mvdr->global_covar_inv_i1[k1]*mvdr->steering_vector_r1[k1]-mvdr->global_covar_inv_r3[k1]*mvdr->steering_vector_i2[k1] +mvdr->global_covar_inv_i3[k1]*mvdr->steering_vector_r2[k1];
                mvdr_ar2=mvdr->global_covar_inv_r2[k1]*mvdr->steering_vector_r1[k1] + mvdr->steering_vector_i1[k1]*mvdr->global_covar_inv_i2[k1]+mvdr->global_covar_inv_r4[k1]*mvdr->steering_vector_r2[k1] +mvdr->steering_vector_i2[k1]*mvdr->global_covar_inv_i4[k1];
                mvdr_ai2=-mvdr->global_covar_inv_r2[k1]*mvdr->steering_vector_i1[k1] +mvdr->global_covar_inv_i2[k1]*mvdr->steering_vector_r1[k1]-mvdr->global_covar_inv_r4[k1]*mvdr->steering_vector_i2[k1] +mvdr->global_covar_inv_i4[k1]*mvdr->steering_vector_r2[k1];
                mvdr_b=mvdr_ar1*mvdr->steering_vector_r1[k1]-mvdr_ai1*mvdr->steering_vector_i1[k1] +mvdr_ar2*mvdr->steering_vector_r2[k1]-mvdr_ai2*mvdr->steering_vector_i2[k1];
                mvdr->w_r1[k1] = (mvdr->global_covar_inv_r1[k1]*mvdr->steering_vector_r1[k1] - mvdr->steering_vector_i1[k1]*mvdr->global_covar_inv_i1[k1]+mvdr->global_covar_inv_r2[k1]*mvdr->steering_vector_r2[k1] -mvdr->steering_vector_i2[k1]*mvdr->global_covar_inv_i2[k1])/mvdr_b;
                mvdr->w_r2[k1] = (mvdr->global_covar_inv_r3[k1]*mvdr->steering_vector_r1[k1] - mvdr->steering_vector_i1[k1]*mvdr->global_covar_inv_i3[k1]+mvdr->global_covar_inv_r4[k1]*mvdr->steering_vector_r2[k1] -mvdr->steering_vector_i2[k1]*mvdr->global_covar_inv_i4[k1])/mvdr_b;
                mvdr->w_i1[k1] = (mvdr->global_covar_inv_r1[k1]*mvdr->steering_vector_i1[k1] + mvdr->global_covar_inv_i1[k1]*mvdr->steering_vector_r1[k1]+mvdr->global_covar_inv_r2[k1]*mvdr->steering_vector_i2[k1] +mvdr->global_covar_inv_i2[k1]*mvdr->steering_vector_r2[k1])/mvdr_b;
                mvdr->w_i2[k1] = (mvdr->global_covar_inv_r3[k1]*mvdr->steering_vector_i1[k1] + mvdr->global_covar_inv_i3[k1]*mvdr->steering_vector_r1[k1]+mvdr->global_covar_inv_r4[k1]*mvdr->steering_vector_i2[k1] +mvdr->global_covar_inv_i4[k1]*mvdr->steering_vector_r2[k1])/mvdr_b;
            
            }
            
        }
        
    }
    for (i = 0; i < mvdr->frame_len; i++){
        mvdr->sub_data1[i] =input1[i];
        mvdr->sub_data2[i] =input2[i];
    }
        
      
    mvdrFFT(X1,mvdr->sub_data1);
    mvdrFFT(X2,mvdr->sub_data2);
    
    for(i = 0; i < mvdr->stft_len/2 +1; i++){
        mvdr->rec_signal1_r[i] = (mvdr->w_r1[i]*X1->real[i])+(mvdr->w_i1[i]*X1->imaginary[i])+ (mvdr->w_r2[i]*X2->real[i])+(mvdr->w_i2[i]*X2->imaginary[i]);
        mvdr->rec_signal1_i[i] = (mvdr->w_r1[i]*X1->imaginary[i])-(mvdr->w_i1[i]*X1->real[i]) +(mvdr->w_r2[i]*X2->imaginary[i])-(mvdr->w_i2[i]*X2->real[i]);
       
    }
        
    for(i = 1; i < mvdr->stft_len/2; i++)
    {
        mvdr->rec_signal1_r[mvdr->stft_len-i] = mvdr->rec_signal1_r[i];
        mvdr->rec_signal1_i[mvdr->stft_len -i]  =-mvdr->rec_signal1_i[i];

    }
        
    for(i =0; i<mvdr->stft_len;i++)
    {
           // mvdr->rec_signal1_r[i] = mvdr->rec_signal1_r[i] + mvdr->rec_signal2_r[i];
           // mvdr->rec_signal1_i[i] = mvdr->rec_signal1_i[i] + mvdr->rec_signal2_i[i];
            //printf("%d %f\n",i, (rec_signal1_i[i]));
            X1->real[i] = mvdr->rec_signal1_r[i];
            X1->imaginary[i] = mvdr->rec_signal1_i[i];

    }
    mvdrIFFT(Y,X1->real,X1->imaginary);
    memmove(mvdr->mvproutput_final,mvdr->mvproutput_final+mvdr->frame_shift ,(mvdr->frame_len -mvdr->frame_shift)*sizeof(double));
    memset(mvdr->mvproutput_final +mvdr->frame_len -mvdr->frame_shift,0,mvdr->frame_shift*sizeof(double));

    for(i =0; i<mvdr->frame_len;i++)
    {
        mvdr->mvproutput_final[i] += Y->real[i]*mvdr->mvprwin[i];

    }
        
    memmove(mvdr->mvproutput,mvdr->mvproutput+mvdr->frame_shift ,(mvdr->frame_len -mvdr->frame_shift)*sizeof(double));
    for(i =0; i<mvdr->frame_shift;i++) {
            
        mvdr->mvproutput[i+mvdr->frame_len -mvdr->frame_shift] =mvdr->mvproutput_final[i];
            
    }
    destroyTransform(Y);
    destroyTransform(X1);
    destroyTransform(X2);

}

void Destroy_mvpr(mvdr_parameters *mvdr){
 
    if (mvdr == NULL) {
        return;
    }

    // Freeing and nullifying each pointer
    if (mvdr->sub_data1 != NULL) {
        free(mvdr->sub_data1);
        mvdr->sub_data1 = NULL;
    }
  
    if (mvdr->sub_data2 != NULL) {
        free(mvdr->sub_data2);
        mvdr->sub_data2 = NULL;
    }
    
    if (mvdr->mvproutput != NULL) {
        free(mvdr->mvproutput);
        mvdr->mvproutput = NULL;
    }
    if (mvdr->mvprwin != NULL) {
        free(mvdr->mvprwin);
        mvdr->mvprwin = NULL;
    }
    if (mvdr->mvproutput_final != NULL) {
        free(mvdr->mvproutput_final);
        mvdr->mvproutput_final = NULL;
    }
   
    if (mvdr->rec_signal1_r != NULL) {
        free(mvdr->rec_signal1_r);
        mvdr->rec_signal1_r = NULL;
    }
    if (mvdr->rec_signal2_r != NULL) {
        free(mvdr->rec_signal2_r);
        mvdr->rec_signal2_r = NULL;
    }
    if (mvdr->rec_signal1_i != NULL) {
        free(mvdr->rec_signal1_i);
        mvdr->rec_signal1_i = NULL;
    }
    if (mvdr->rec_signal2_i != NULL) {
        free(mvdr->rec_signal2_i);
        mvdr->rec_signal2_i = NULL;
    }
    
    if (mvdr->global_covar_r1 != NULL) {
        free(mvdr->global_covar_r1);
        mvdr->global_covar_r1 = NULL;
    }

    if (mvdr->global_covar_r2 != NULL) {
        free(mvdr->global_covar_r2);
        mvdr->global_covar_r2 = NULL;
    }
    if (mvdr->global_covar_r3 != NULL) {
        free(mvdr->global_covar_r3);
        mvdr->global_covar_r3 = NULL;
    }
    if (mvdr->global_covar_r4 != NULL) {
        free(mvdr->global_covar_r4);
        mvdr->global_covar_r4 = NULL;
    }
    if (mvdr->corrvar_r1 != NULL) {
        free(mvdr->corrvar_r1);
        mvdr->corrvar_r1 = NULL;
    }
    
    if (mvdr->corrvar_r2 != NULL) {
        free(mvdr->corrvar_r2);
        mvdr->corrvar_r2 = NULL;
    }
    if (mvdr->corrvar_r3 != NULL) {
        free(mvdr->corrvar_r3);
        mvdr->corrvar_r3 = NULL;
    }
    
    if (mvdr->corrvar_r4 != NULL) {
        free(mvdr->corrvar_r4);
        mvdr->corrvar_r4 = NULL;
    }

    if (mvdr->w_r1 != NULL) {
        free(mvdr->w_r1);
        mvdr->w_r1 = NULL;
    }
    if (mvdr->w_r2 != NULL) {
        free(mvdr->w_r2);
        mvdr->w_r2 = NULL;
    }

    if (mvdr->global_covar_inv_r1 != NULL) {
        free(mvdr->global_covar_inv_r1);
        mvdr->global_covar_inv_r1 = NULL;
    }
    if (mvdr->global_covar_inv_r2 != NULL) {
        free(mvdr->global_covar_inv_r2);
        mvdr->global_covar_inv_r2 = NULL;
    }
    if (mvdr->global_covar_inv_r3 != NULL) {
        free(mvdr->global_covar_inv_r3);
        mvdr->global_covar_inv_r3 = NULL;
    }
    if (mvdr->global_covar_inv_r4 != NULL) {
        free(mvdr->global_covar_inv_r4);
        mvdr->global_covar_inv_r4 = NULL;
    }

    if (mvdr->global_covar_i1 != NULL) {
        free(mvdr->global_covar_i1);
        mvdr->global_covar_i1 = NULL;
    }
    if (mvdr->global_covar_i2 != NULL) {
        free(mvdr->global_covar_i2);
        mvdr->global_covar_i2 = NULL;
    }
    if (mvdr->global_covar_i3 != NULL) {
        free(mvdr->global_covar_i3);
        mvdr->global_covar_i3 = NULL;
    }
    if (mvdr->global_covar_i4 != NULL) {
        free(mvdr->global_covar_i4);
        mvdr->global_covar_i4 = NULL;
    }

    if (mvdr->corrvar_i1 != NULL) {
        free(mvdr->corrvar_i1);
        mvdr->corrvar_i1 = NULL;
    }
    if (mvdr->corrvar_i2 != NULL) {
        free(mvdr->corrvar_i2);
        mvdr->corrvar_i2 = NULL;
    }
    if (mvdr->corrvar_i3 != NULL) {
        free(mvdr->corrvar_i3);
        mvdr->corrvar_i3 = NULL;
    }
    if (mvdr->corrvar_i4 != NULL) {
        free(mvdr->corrvar_i4);
        mvdr->corrvar_i4 = NULL;
    }

    if (mvdr->w_i1 != NULL) {
        free(mvdr->w_i1);
        mvdr->w_i1 = NULL;
    }
    if (mvdr->w_i2 != NULL) {
        free(mvdr->w_i2);
        mvdr->w_i2 = NULL;
    }

    if (mvdr->global_covar_inv_i1 != NULL) {
        free(mvdr->global_covar_inv_i1);
        mvdr->global_covar_inv_i1 = NULL;
    }
    if (mvdr->global_covar_inv_i2 != NULL) {
        free(mvdr->global_covar_inv_i2);
        mvdr->global_covar_inv_i2 = NULL;
    }
    if (mvdr->global_covar_inv_i3 != NULL) {
        free(mvdr->global_covar_inv_i3);
        mvdr->global_covar_inv_i3 = NULL;
    }
    if (mvdr->global_covar_inv_i4 != NULL) {
        free(mvdr->global_covar_inv_i4);
        mvdr->global_covar_inv_i4 = NULL;
    }
    if (mvdr->steering_vector_r1 != NULL) {
        free(mvdr->steering_vector_r1);
        mvdr->steering_vector_r1 = NULL;
    }
    if (mvdr->steering_vector_i1 != NULL) {
        free(mvdr->steering_vector_i1);
        mvdr->steering_vector_i1 = NULL;
    }
    if (mvdr->steering_vector_r2 != NULL) {
        free(mvdr->steering_vector_r2);
        mvdr->steering_vector_r2 = NULL;
    }
    if (mvdr->steering_vector_i2 != NULL) {
        free(mvdr->steering_vector_i2);
        mvdr->steering_vector_i2 = NULL;
    }
    if (mvdr->mvZ_real1 != NULL) {
        free(mvdr->mvZ_real1);
        mvdr->mvZ_real1 = NULL;
    }
    if (mvdr->mvZ_img1 != NULL) {
        free(mvdr->mvZ_img1);
        mvdr->mvZ_img1 = NULL;
    }
    
    if (mvdr->mvZ_real2 != NULL) {
        free(mvdr->mvZ_real2);
        mvdr->mvZ_real2 = NULL;
    }
    if (mvdr->mvZ_img2 != NULL) {
        free(mvdr->mvZ_img2);
        mvdr->mvZ_img2 = NULL;
    }
    
   
    free(mvdr);
    mvdr = NULL;
}



