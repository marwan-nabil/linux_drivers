#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

int main(int argc, char **argv)
{
    int fd, num_to_write, buffer_length, num_written;
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

    // open the device file
    fd = open(argv[2], O_RDWR);

    // read input from stdin to the user-space buffer
    num_to_write = read(STDIN_FILENO, buffer, buffer_length);

    // write the data from the user-space buffer to the device
    num_written = write(fd, buffer, num_to_write);
    if(num_written < 0) 
    {
        printf("ERROR: write failed with error code %d\n", num_written);
        return -1;
    }
    
    close(fd);
    return 0;
}
