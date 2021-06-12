#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

int main(int argc, char *argv[])
{
    // $write_test  buffer_size  device_file_name

    // open the device file
    int fd = open(argv[2], O_RDWR);

    // create a local char buffer
    int buffer_size = atoi(argv[1]);
    char buffer[buffer_size];

    // read from the device to the local buffer
    int num_read = read(fd, buffer, buffer_size);

    write(STDOUT_FILENO, buffer, num_read); // write from local buffer to stdout
    write(STDOUT_FILENO, "\n", 1); // write a newline
    
    // close the device file
    close(fd);
    return 0;
}
