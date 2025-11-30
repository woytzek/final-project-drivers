
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
    
    #if (0)
    // check if there're cli parameters
    if (argc > 1 && argc < MAX_PINS)
    {
        printf("GPIO numbers to watch:\n");
        for (int i = 1; i < argc; i++)
        {
            int id = atoi(argv[i]);
            if( id == 0 && argv[i][0] != '0' )
            {
                printf("  Ignored invalid GPIO number: %s\n", argv[i]);
                continue;
            }
            printf("  GPIO-%02d\n", id);
            pintab[pin_num++] = id;
        }
    }
    else
    {
        printf("No GPIO numbers provided or too many arguments (max %d).\n", MAX_PINS);
        return -1;
    }
    #endif

    #if (0)
    // open device to set GPIO with ioctl
    int fd = open(GPIO_RW_DEVICE, O_RDWR);
    if (fd < 0)
    {
        perror("Failed to open GPIO RW device");
        return -1;
    }
    struct gpio_rw_pins pins_cfg;
    for (int i = 0; i < pin_num; i++)
    {
        pins_cfg.pins[i].pin = pintab[i];
        pins_cfg.pins[i].direction = 0; // input
    }
    pins_cfg.num_pins = pin_num;
    if (ioctl(fd, GPIO_RW_IOC_SET_PINS, &pins_cfg) < 0)
    {
        perror("Failed to set GPIO pins");
        close(fd);
        return -1;
    }
    #endif

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
        sleep(1);
    }
    close(fd);

    return 0;
}