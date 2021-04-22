#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <string.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

using namespace std;

struct MyData
{
    char name[20];
    int age;
};

void writeMemory()
{
    // specify shared file path
    char *shared_file_name = "/home/hemingshan/exo_ws/src/exoskeleton_control/Pipe_File/my_shared_memory";

    // define shared data
    //    unsigned long buff_size = 4096;
    //    char share_buffer[] = "greetings, hello world";
    //    MyData share_buffer("Tom", 18);
    MyData share_buffer = { "Tom", 20 };

    // create mmap file
    int fd = open(shared_file_name, O_CREAT | O_RDWR | O_TRUNC, 00777);
    if (fd < 0)
        cout << "create file error" << endl;

    size_t write_size = sizeof(share_buffer);

    ftruncate(fd, write_size); // extend file size

    // map memory to file
    void *p = mmap(NULL, write_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

    // copy data to shared memory
    memcpy(p, &share_buffer, write_size);

    cout << "already write to shared memory, wait ..." << endl;
    //cout << share_buffer << endl;
    this_thread::sleep_for(chrono::seconds(10));

    // unmap and close
    munmap(p, write_size);
    close(fd);

}

int main()
{
    writeMemory();

    return 0;
}