#include<stdio.h>
#include<sys/stat.h>
#include<unistd.h>
#include<fcntl.h>
#include<sys/types.h>
#include<string.h>
#define _PATH_NAME "/home/hemingshan/exo_ws/src/exoskeleton_control/Pipe_File/file.tmp"
#define _SIZE_ 100
int main()
{
    int fd=open(_PATH_NAME,O_RDONLY);
    if(fd<0){
        printf("open file error");
        return 1;
    }
   double buf[_SIZE_];
   memset(buf,'\0',sizeof(buf));
   while(1)
   {
       int ret=read(fd,buf,sizeof(buf));
       if(ret<0){
           printf("read end or error\n");
           break;
       }
   printf("%s",buf);
   }
   close(fd);
   return 0;
 }