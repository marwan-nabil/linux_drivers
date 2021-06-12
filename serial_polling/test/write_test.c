#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

int main(int argc, char *argv[])
{
    // $write_test  buffer_size  device_file_name

    // open the device file
    int fd = open(argv[2], O_RDWR);

    // create a local char buffer
    int buffer_size = atoi(argv[1]);
    char buffer[buffer_size];

    // read from standard input into the buffer
    int num_read = read(STDIN_FILENO, buffer, buffer_size);

    // write the data to the device
    write(fd, buffer, num_read);
    
    // close the device file
    close(fd);

    return 0;
}
