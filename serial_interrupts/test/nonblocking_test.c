#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

// test trying to read from the driver when we open it in non-blocking mode
// this is impossible because the driver is reading from an external device
// and will always block on trying to read

int main(int argc, char **argv)
{
    int fd, result, buffer_length;
    char *buffer;

    // error checking for wrong number of arguments
    if(argc != 3)
    {
        printf("USAGE: $block_test <buffer length>  <device file name>\n");
        return -1;
    }

    buffer_length = atoi(argv[1]);
    
    // open the device file in non-blocking mode
    fd = open(argv[2], O_RDWR | O_NONBLOCK);

    // allocate a user space buffer to read the data into
    buffer = (char *)malloc((size_t)buffer_length);
   
    // try reading in non-blocking mode
    result = read(fd, buffer, buffer_length);
    if(result < 0) 
    {
        printf("ERROR: read failed with error code %d\n", result);
        return -1;
    }

    // try writing into the file with non-blocking mode
    result = write(fd, buffer, buffer_length);
    if(result < 0)
    {
        printf("ERROR: write failed with error code %d\n", result);
        return -1;
    }
  
    close(fd);
    free(buffer);
    return 0;
}
