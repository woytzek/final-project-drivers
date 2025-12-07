#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>

#include "../dht22.h"


int main(void)
{
    /* open device for read/write */
    int fd = open("/dev/dht22_dev", O_RDWR);
    if( fd < 0 )
    {
        perror("Failed to open /dev/dht22_dev");
        return -1;
    }
    printf("DHT22 device opened successfully\n");

    /* power ON the sensor */
    struct dht22_ioctl ioctl_data;
    ioctl_data.power = 1;
    if( ioctl(fd, DHT22_IOCTL_CMD_SET_POWER, &ioctl_data) < 0 )
    {
        perror("Failed to power ON the DHT22 sensor");
        close(fd);
        return -1;
    }
    printf("DHT22 sensor powered ON\n");

    /* delay 3 seconds */
    sleep(5);
    while(1)
    {
        /* trigger a reading */
        if( ioctl(fd, DHT22_IOCTL_CMD_TRIGGER, NULL) < 0 )
        {
            perror("Failed to trigger DHT22 reading");
            close(fd);
            return -1;
        }
        printf("DHT22 reading triggered\n");

        /* wait 10ms */
        usleep(10000);

        /* read data */
        uint8_t data[40];
        ssize_t bytes_read = read(fd, data, sizeof(data));
        if( bytes_read < 0 )
        {
            perror("Failed to read DHT22 data");
            close(fd);
            return -1;
        }

        /* print data */
        printf("DHT22 Data: ");
        for(int i = 0; i < bytes_read; i++)
        {
            printf("%c", data[i]);
        }
        printf("\n");

        /* wait before next reading */
        sleep(5);
    }
    return 0;
}
