#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

// test trying to open the device file multiple times

int main(int argc, char **argv)
{
    int fd1, fd2, buffer_length, num_to_write, num_written;
    char *buffer;

    // error checking for wrong number of arguments
    if(argc != 3)
    {
        printf("USAGE: $write_test <buffer length>  <device file name>\n");
        return -1;
    }

    buffer_length = atoi(argv[1]);
    // allocate a user space buffer to read the data into from stdin
    // and write data from to the device
    buffer = (char *)malloc((size_t)buffer_length);

    // open the device file twice
    fd1 = open(argv[2], O_RDWR);
    fd2 = open(argv[2], O_RDWR);

    if((fd1 < 0) || (fd2 < 0))
    {
        printf("ERROR: opening the device file twice failed with fd1: %d, fd2: %d\n", fd1, fd2);
        return -1;
    }

    // read input from stdin to the user-space buffer
    num_to_write = read(STDIN_FILENO, buffer, buffer_length);

    // write the data from the user-space buffer to the device using fd1
    num_written = write(fd1, buffer, num_to_write);
    if(num_written < 0) 
    {
        printf("ERROR: first write failed with error code %d\n", num_written);
        return -1;
    }
    
    // write the data from the user-space buffer to the device using fd2
    num_written = write(fd2, buffer, num_to_write);
    if(num_written < 0) 
    {
        printf("ERROR: second write failed with error code %d\n", num_written);
        return -1;
    }

    close(fd1);
    close(fd2);
    return 0;
}
