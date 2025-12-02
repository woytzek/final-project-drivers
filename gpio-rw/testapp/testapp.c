
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "../gpio_rw.h"

#define GPIO_RW_DEVICE "/dev/gpio_rw"

int pintab[MAX_PINS] = {17, 22, 27};
int pin_num = 3;

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    printf("GPIO RW Test Application\n");

    // open device to read GPIO values
    int fd = open(GPIO_RW_DEVICE, O_RDONLY);
    if (fd < 0)
    {
        perror("Failed to open GPIO RW device");
        return -1;
    }
    unsigned char value;
    while (1)
    {
        // reset offset
        if( lseek(fd, 0, SEEK_SET) < 0 )
        {
            perror("Failed to seek to start of device");
            close(fd);
            return -1;
        }
        // read GPIO values
        int n = read(fd, &value, 1);
        if (n < 0)
        {
            perror("Failed to read GPIO values");
            close(fd);
            return -1;
        }
        if( n == 0 )
        {
            printf("End of file reached\n");
            break;
        }
        printf("GPIO values: ");
        for (int i = 0; i < pin_num; i++)
        {
            int v = (value >> i) & 0x1;
            printf("GPIO-%02d=%d ", pintab[i], v);
        }
        printf("\n");
        sleep(5);
    }
    close(fd);

    return 0;
}