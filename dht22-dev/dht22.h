#ifndef DHT22_H
#define DHT22_H

#include <linux/types.h>

#define DHT22_CMD_MAX 3
#define DHT22_IOCTL_MAGIC 'D'
#define DHT22_IOCTL_CMD_SET_POWER _IOW(DHT22_IOCTL_MAGIC, 0, struct dht22_ioctl)
#define DHT22_IOCTL_CMD_GET_POWER _IOR(DHT22_IOCTL_MAGIC, 1, struct dht22_ioctl)
#define DHT22_IOCTL_CMD_TRIGGER   _IOW(DHT22_IOCTL_MAGIC, 2, struct dht22_ioctl)

struct dht22_ioctl
{
    uint32_t power;
};

#endif /* DHT22_H */