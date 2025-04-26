

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
//#include <string.h>
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>

#include "mvpraudio_main.h"

#define BUFFER_SIZE 160


void appendtofile(const char *path, uint8_t *buf, int len){
  static int first=1;
  if (first) {
    first = 0;
    unlink(path);
  }

  int fd = open(path, O_APPEND|O_CREAT|O_RDWR, 0777);
  if(fd<0){return;}
      
  write(fd, buf, len);
  close(fd);
}



int main() {

    int sampleRate = 16000;
    //路径问题？
   
    const char* WAvNames[2] = {"/tmp/mvdrtest1.wav",
                        "/tmp/mvdrtest2.wav",};
    
    
    MVDRVariables* mvpr_inst=MainparamInitilization(16000,256,512,512,160,true,0.03,0,4);
   
    int retlpb=0;
    int ret=0;
    int ret2=0;
    short buffer[BUFFER_SIZE];
    short buffer2[BUFFER_SIZE];
    short out_biffer[BUFFER_SIZE];
    int fd,fd2;
    for (int i=0;i<1;i++){
        fd = open(WAvNames[0], O_RDONLY);
        fd2= open(WAvNames[1], O_RDONLY);
        if (fd < 0) {
            printf("open wakeup file failed \n");
            return -1;}
        if (fd2 < 0) {
            printf("open wakeup file failed \n");
            return -1;}
      
        lseek(fd, 44, SEEK_SET);
        lseek(fd2, 44, SEEK_SET);
        
        while (1) {
            memset(buffer, 0x00, BUFFER_SIZE*2);
            memset(buffer2, 0x00, BUFFER_SIZE*2);
    
            ret = read(fd, buffer, BUFFER_SIZE*2);
            ret2 = read(fd2, buffer2, BUFFER_SIZE*2);
            
            if (ret < BUFFER_SIZE*2 || ret2<BUFFER_SIZE*2) {
                break;}
        
            MVDR_mainProcessing(mvpr_inst,buffer,buffer2,BUFFER_SIZE);
            MVDR_GETOUTPUT(mvpr_inst,out_biffer,BUFFER_SIZE);
            appendtofile("/tmp/mvprout.wav",(uint8_t *)out_biffer,BUFFER_SIZE*2);
        }   
        Destroymainparm(mvpr_inst); 
    }


}
