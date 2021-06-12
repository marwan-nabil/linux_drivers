#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

// test reading from the device some data
// then writing this data to the terminal

int main(int argc, char **argv)
{
    int fd, num_read, buffer_length;

    // error checking for wrong number of arguments
    if(argc != 3)
    {
        printf("USAGE: $read_test <buffer length>  <device file name>\n");
        return -1;
    }

    // create a local char buffer
    int buffer_length = atoi(argv[1]);
    char buffer[buffer_length];
    
    // open the device file
    fd = open(argv[2], O_RDWR);

    // try reading
    num_read = read(fd, buffer, buffer_length);
    if(num_read < 0) 
    {
        printf("ERROR: read failed with error code %d\n", num_read);
        return -1;
    }

    // write the data that was read from the device to stdout
    write(STDOUT_FILENO, buffer, num_read);
    write(STDOUT_FILENO, "\n", 1);
    
    close(fd);
    return 0;
}
