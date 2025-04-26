

#include "mvpraudio_main.h"
#include "mvdr2.h"


MVDRVariables* MainparamInitilization(int frequency, int stepSize,int frameSize, int fftsize,int buffer_size,bool isOn,float mic_daimeter,int look_direction,float max_gain){
    MVDRVariables *mainParam = (MVDRVariables*)malloc(sizeof(MVDRVariables));

    int num_channel = 2;
    int num_stat = 59;

    int i;
    mainParam->nFFT=fftsize;
    mainParam->counter = 0;
    mainParam->stepSize = stepSize;
    mainParam->samplingRate = frequency;
    mainParam->frameSize = frameSize;  ////Threadshold Time
    mainParam->topMicBuffer = (double*)calloc(frameSize,sizeof(double));
    mainParam->botMicBuffer = (double*)calloc(frameSize,sizeof(double));
    mainParam->mixedBuffer = (double*)calloc(stepSize,sizeof(double));
    
    mainParam->isOn=isOn;
    
    mainParam->mvdr = newparameters(fftsize,num_stat,frameSize,num_channel,stepSize);
    mainParam->ake_counter=0;

    int mic_angle_vector[2] ={90,-90};

    cal_steering_vector(mainParam->mvdr,mic_angle_vector,mic_daimeter,look_direction,343,max_gain,frequency);
    int mic_biffersize=buffer_size>frameSize ? buffer_size:frameSize;
    mainParam->mvdrmic_inbuffer=(short*)calloc(2*mic_biffersize,sizeof(short));
    mainParam->mvun_process_num=0;
    mainParam->inprocess_buffer=(short*)calloc(2*stepSize,sizeof(short));
    mainParam->mvdrbuf_ready_num=stepSize;
    int processed_num=buffer_size/stepSize +1;
    mainParam->seprocessed_data=(double*)calloc(processed_num*stepSize,sizeof(double));
    mainParam->finalout_buffer=(double*)calloc(stepSize+processed_num*stepSize,sizeof(double));

    return mainParam;
}
void MVDR_mainProcessing(MVDRVariables *mainParam,short* originbuffer1,short* originbuffer2,int buf_len){
    int idx=0;
    int process_num=(mainParam->mvun_process_num+buf_len)/(mainParam->stepSize);
    int subidx=0;
    for (idx=0;idx<process_num;idx++){
        for (subidx=0;subidx<mainParam->stepSize;subidx++){
            if(idx*mainParam->stepSize+subidx<mainParam->mvun_process_num){
                mainParam->inprocess_buffer[2*subidx] = mainParam->mvdrmic_inbuffer[2*idx*mainParam->stepSize+2*subidx];
                mainParam->inprocess_buffer[2*subidx+1] = mainParam->mvdrmic_inbuffer[2*idx*mainParam->stepSize+2*subidx+1];
            }else{
                mainParam->inprocess_buffer[2*subidx] = originbuffer1[idx*mainParam->stepSize+subidx-mainParam->mvun_process_num];
                mainParam->inprocess_buffer[2*subidx+1] = originbuffer2[idx*mainParam->stepSize+subidx-mainParam->mvun_process_num];
            }
        }
        MVDR_realtimeProcessing(mainParam);
        for (subidx=0;subidx<mainParam->stepSize;subidx++){
            mainParam->seprocessed_data[idx*mainParam->stepSize+subidx]=mainParam->mixedBuffer[subidx];
            //mainParam->seprocessed_data[idx*mainParam->stepSize+subidx] = (double)(mainParam->inprocess_buffer[2*subidx]+mainParam->inprocess_buffer[2*subidx+1])/(2*32768);
        }

    }
    int begin_indx1=process_num*mainParam->stepSize -mainParam->mvun_process_num;
    int res_len =mainParam->mvun_process_num+buf_len-process_num*mainParam->stepSize;
    
    
    if(begin_indx1>=0){
        for (idx=0;idx<res_len;idx++){
            mainParam->mvdrmic_inbuffer[2*idx] = originbuffer1[begin_indx1+idx];
            mainParam->mvdrmic_inbuffer[2*idx+1] = originbuffer2[begin_indx1+idx];
        }
    }else{
        memmove(mainParam->mvdrmic_inbuffer,mainParam->mvdrmic_inbuffer+process_num*mainParam->stepSize*2,-begin_indx1*2*sizeof(short));
        for (idx=-begin_indx1;idx<res_len;idx++){
            mainParam->mvdrmic_inbuffer[2*idx] = originbuffer1[begin_indx1+idx];
            mainParam->mvdrmic_inbuffer[2*idx+1] = originbuffer2[begin_indx1+idx];
        }

    }
    mainParam->mvun_process_num=res_len;
    for (idx=0;idx<process_num*mainParam->stepSize;idx++){
        mainParam->finalout_buffer[idx+mainParam->mvdrbuf_ready_num] = mainParam->seprocessed_data[idx];

    }
    mainParam->mvdrbuf_ready_num +=process_num*mainParam->stepSize;
    
}
void MVDR_GETOUTPUT(MVDRVariables *mainParam,short* outbuffer,int buf_size){
    if(mainParam->mvdrbuf_ready_num<buf_size){
        return;
    }
    int idx =0;
    for (idx=0;idx<buf_size;idx++){
        outbuffer[idx]=(short)(mainParam->finalout_buffer[idx]*32768);
    }
   
    mainParam->mvdrbuf_ready_num -=buf_size;
    memmove(mainParam->finalout_buffer,mainParam->finalout_buffer+buf_size,mainParam->mvdrbuf_ready_num*sizeof(double));

}


void MVDR_realtimeProcessing(MVDRVariables *mainParam){
    
    
    int i,j,stepSize,frameSize;
    stepSize = mainParam->stepSize;
    frameSize = mainParam->frameSize;

    double * x_frame = (double*)malloc(mainParam->frameSize * sizeof(double));
    double * h_frame = (double*)malloc(mainParam->frameSize * sizeof(double));
    int k=0;

    for (k = 0; k < mainParam->frameSize; k++) {

        x_frame[k] = mainParam->botMicBuffer[k];
        h_frame[k] = mainParam->topMicBuffer[k];

    }

    mainParam->ake_counter= mainParam->ake_counter+1;
    
    
    if(mainParam->isOn) {
        enhance_signal(mainParam->mvdr, x_frame, h_frame, mainParam->ake_counter);
    }


    for(i = 0,j=0;i<stepSize;i++,j+=2){
        
        mainParam->topMicBuffer[i]= mainParam->topMicBuffer[i+stepSize];
        mainParam->topMicBuffer[i+stepSize]= (double)(mainParam->inprocess_buffer[j+1])*FAC;
        
        mainParam->botMicBuffer[i]= mainParam->botMicBuffer[i+stepSize];
        mainParam->botMicBuffer[i+stepSize]= (double)(mainParam->inprocess_buffer[j])*FAC;
    }
    

    if(mainParam->isOn){
        for(i=0;i<stepSize;i++){

           mainParam->mixedBuffer[i] = (mainParam->mvdr->mvproutput[i]);
        }
    }else{
        for(i=0;i<stepSize;i++){
            mainParam->mixedBuffer[i] = (x_frame[i]+h_frame[i])/2;
        }

    }
   

    free(x_frame);
    free(h_frame);
    x_frame=NULL;
    h_frame=NULL;
    

}

void Destroymainparm(MVDRVariables *mainParam){
    if(NULL==mainParam){
        return;
    }
    if(mainParam->topMicBuffer !=NULL){
        free(mainParam->topMicBuffer);
        mainParam->topMicBuffer = NULL;
    }
    if(mainParam->botMicBuffer !=NULL){
        free(mainParam->botMicBuffer);
        mainParam->botMicBuffer = NULL;
    }
    if(mainParam->mixedBuffer !=NULL){
        free(mainParam->mixedBuffer);
        mainParam->mixedBuffer = NULL;
    }


    if (mainParam->mvdrmic_inbuffer != NULL) {
        free(mainParam->mvdrmic_inbuffer);
        mainParam->mvdrmic_inbuffer = NULL;
    }
    if (mainParam->inprocess_buffer != NULL) {
        free(mainParam->inprocess_buffer);
        mainParam->inprocess_buffer = NULL;
    }
    if (mainParam->finalout_buffer != NULL) {
       free(mainParam->finalout_buffer);
        mainParam->finalout_buffer = NULL;
   }
    if(mainParam->seprocessed_data !=NULL){
        free(mainParam->seprocessed_data);
        mainParam->seprocessed_data=NULL;
   }
    Destroy_mvpr(mainParam->mvdr);
    free(mainParam);
    mainParam=NULL;
    
}





