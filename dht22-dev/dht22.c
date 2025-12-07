#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/ioctl.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include "dht22.h"

//#define DHT22_DEVICE_DEBUG

#undef PDEBUG
#ifdef DHT22_DEVICE_DEBUG
#  ifdef __KERNEL__
     /* This one is for kernel space */
#    define PDEBUG(fmt, args...) printk( KERN_DEBUG "dht22_dev: " fmt, ## args)
#  else
     /* This one is for user space */
#    define PDEBUG(fmt, args...) fprintf(stderr, "dht22_dev: " fmt, ## args)
#  endif
#else
#  define PDEBUG(fmt, args...) /* not debugging: nothing */
#endif

#define DEVICE_NAME "dht22_dev"
#define CLASS_NAME  "dht22_dev_class"

int dht22_dev_major =   0; // use dynamic major
int dht22_dev_minor =   0;

static dev_t dev_num;
static struct cdev dht22_cdev;
static struct class *dht22_class;
static struct device *dht22_device;

struct dht22_data
{
    struct gpio_desc *sup_gpio; // GPIO for power supply control
    struct gpio_desc *data_gpio; // GPIO for data line
    int power_status;
    ktime_t last_read_time;
    ktime_t data_timestamps[44];
    int edge_count;
    int data_available;
    loff_t last_read_offset;
};

struct dht22_decoded_data
{
    uint32_t timestamp;
    uint16_t humidity;
    uint16_t temperature;
    uint8_t temperature_sign;
};

static DEFINE_SPINLOCK(dht22_lock);
static struct dht22_data dht22_sensor;

static irqreturn_t dht22_data_irq_handler(int irq, void *dev_id)
{
    ktime_t current_time = ktime_get();

    unsigned long flags;
    spin_lock_irqsave(&dht22_lock, flags);
    if( dht22_sensor.edge_count < 44 )
    {
        dht22_sensor.data_timestamps[dht22_sensor.edge_count] = current_time;
        dht22_sensor.edge_count++;
        if( dht22_sensor.edge_count == 44 )
        {
            dht22_sensor.data_available = 1;
            PDEBUG("DHT22 data IRQ: all edges received, data available\n");
        }
    }
    else
    {
        PDEBUG("DHT22 data IRQ: edge count overflow\n");
    }
    spin_unlock_irqrestore(&dht22_lock, flags);

    return IRQ_HANDLED;
}

static int dht22_trigger(void)
{
    if( dht22_sensor.power_status == 0 )
    {
        PDEBUG("DHT22 device ioctl: sensor is powered OFF, cannot trigger reading\n");
        return -EIO;
    }
    if( time_before(ktime_to_ms(ktime_sub(ktime_get(), dht22_sensor.last_read_time)), 2500) )
    {
        PDEBUG("DHT22 device ioctl: reading triggered too soon after last read or power-on\n");
        return -EIO;
    }

    unsigned long flags;
    spin_lock_irqsave(&dht22_lock, flags);
    dht22_sensor.data_available = 0;
    ktime_t now = ktime_get();
    dht22_sensor.last_read_time = now;
    dht22_sensor.data_timestamps[0] = now;
    dht22_sensor.edge_count = 1;
    spin_unlock_irqrestore(&dht22_lock, flags);

    if( gpiod_direction_output(dht22_sensor.data_gpio, 0) < 0 )
    {
        PDEBUG("DHT22 device ioctl: failed to set data GPIO to output low\n");
        return -EIO;
    }
    mdelay(1); // hold low for at least 1 ms
    
    if( gpiod_direction_input(dht22_sensor.data_gpio) < 0 )
    {
        PDEBUG("DHT22 device ioctl: failed to set data GPIO to input\n");
        return -EIO;
    }

    return 0;
}

int dht22_decode_data( struct dht22_decoded_data *decoded )
{
    if( dht22_sensor.data_available == 0 )
    {
        PDEBUG("DHT22 decode data: no data available to decode\n");
        return -EIO;
    }

    uint8_t raw_data[5] = {0};
    int i, j;
    unsigned long flags;
    spin_lock_irqsave(&dht22_lock, flags);
    for( i = 4, j = 0; i < 44; i++, j++ )
    {
        int width = ktime_to_us( ktime_sub( dht22_sensor.data_timestamps[i], dht22_sensor.data_timestamps[i - 1] ) );
        if( width > 100 )
        {
            raw_data[j / 8] |= (1 << (7 - (j % 8)));
        }
    }
    spin_unlock_irqrestore(&dht22_lock, flags);

    uint8_t checksum = raw_data[0] + raw_data[1] + raw_data[2] + raw_data[3];
    if( checksum != raw_data[4] )
    {
        pr_info("DHT22 decode data: checksum mismatch\n");
        return -EIO;
    }

    decoded->timestamp = ktime_to_ms(dht22_sensor.data_timestamps[0]);
    decoded->humidity = 256 * raw_data[0] + raw_data[1];
    decoded->temperature = 256 * (raw_data[2] & 0x7F) + raw_data[3];
    decoded->temperature_sign = (raw_data[2] & 0x80) ? 1 : 0;

    return 0;
}

static int dht22_open(struct inode *inode, struct file *file)
{
    /* allocate buffer */
    char *buf = kmalloc(40, GFP_KERNEL);
    if( !buf )
    {
        pr_err("DHT22 device open: failed to allocate memory\n");
        return -ENOMEM;
    }
    buf[0] = 0;
    file->private_data = buf;
    dht22_sensor.last_read_offset = 0;

    PDEBUG("DHT22 device opened\n");
    return 0;
}

static int dht22_release(struct inode *inode, struct file *file)
{
    /* free buffer */
    char *buf = (char *)file->private_data;
    kfree(buf);

    PDEBUG("DHT22 device closed\n");
    return 0;
}

static ssize_t dht22_read(struct file *file, char __user *buf, size_t len, loff_t *offset)
{
    PDEBUG("DHT22 device read\n");
    if( dht22_sensor.data_available == 0 )
    {
        PDEBUG("DHT22 device read: no data available\n");
        return 0;
    }
    
    char *data = file->private_data;
    if( *offset == dht22_sensor.last_read_offset )
    {
        PDEBUG("DHT22 device read: first read of new data\n");

        struct dht22_decoded_data decoded;
        int ret;
        if( dht22_decode_data( &decoded ) < 0 )
        {
            ret = snprintf(data, 40, "Error: invalid data");
        }
        else
        {
            ret = snprintf(data, 40, "%ld: T=%c%u.%u H=%u.%u",
                        decoded.timestamp,
                        (decoded.temperature_sign ? '-' : '+'),
                        decoded.temperature / 10,
                        decoded.temperature % 10,
                        decoded.humidity / 10,
                        decoded.humidity % 10);
        }
        data[39] = 0; // ensure null-termination
        if( ret < 0 )
        {
            PDEBUG("DHT22 device read: snprintf failed\n");
            return -EIO;
        }
    }

    int off = *offset - dht22_sensor.last_read_offset;
    int copied = 0;
    while( data[off] && (copied < len) )
    {
        if( put_user(data[off], buf + off) )
        {
            PDEBUG("DHT22 device read: copy_byte_to_user failed\n");
            return -EFAULT;
        }
        off++;
        copied++;
    }

    if( copied == 0 )
    {
        /* EOF */
        dht22_sensor.last_read_offset = *offset;
        PDEBUG("DHT22 device read: EOF reached\n");
    }

    return copied;
}

static long dht22_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    PDEBUG("DHT22 device ioctl cmd=%u\n", _IOC_NR(cmd));
    
    /* handle ioctl */
    struct dht22_ioctl ioctl_data;

    if( _IOC_TYPE(cmd) != DHT22_IOCTL_MAGIC )
    {
        PDEBUG("DHT22 device ioctl: invalid magic\n");
        return -ENOTTY;
    }
    if( _IOC_NR(cmd) >= DHT22_CMD_MAX )
    {
        PDEBUG("DHT22 device ioctl: invalid command number\n");
        return -ENOTTY;
    }

    switch( _IOC_NR(cmd) )
    {
        case 0: // Set power ON/OFF the sensor
            if( copy_from_user(&ioctl_data, (struct dht22_ioctl __user *)arg, sizeof(struct dht22_ioctl)) )
            {
                PDEBUG("DHT22 device ioctl: failed to copy from user\n");
                return -EFAULT;
            }
            if( ioctl_data.power )
            {
                gpiod_set_value(dht22_sensor.sup_gpio, 1);
                dht22_sensor.power_status = 1;
                dht22_sensor.last_read_time = jiffies;
                PDEBUG("DHT22 sensor powered ON\n");
            }
            else
            {
                gpiod_set_value(dht22_sensor.sup_gpio, 0);
                dht22_sensor.power_status = 0;
                PDEBUG("DHT22 sensor powered OFF\n");
            }
            break;
        case 1: // Get power status
            ioctl_data.power = gpiod_get_value(dht22_sensor.sup_gpio);
            if( copy_to_user((struct dht22_ioctl __user *)arg, &ioctl_data, sizeof(struct dht22_ioctl)) )
            {                
                PDEBUG("DHT22 device ioctl: failed to copy to user\n");
                return -EFAULT;
            }
            PDEBUG("DHT22 sensor power status retrieved: %u\n", ioctl_data.power);
            break;
        case 2: // Trigger a reading
            if( dht22_trigger() < 0 )
            {
                PDEBUG("DHT22 device ioctl: failed to trigger reading\n");
                return -EIO;
            }
            PDEBUG("DHT22 sensor reading triggered\n");
            break;
        default:
            PDEBUG("DHT22 device ioctl: unknown command %u\n", cmd);
            return -EINVAL;
    }
    return 0;
}

static const struct file_operations dht22_fops = 
{
    .owner = THIS_MODULE,
    .open  = dht22_open,
    .release = dht22_release,
    .read  = dht22_read,
    .unlocked_ioctl = dht22_ioctl,
};

static int dht22_device_init(struct device *dev)
{
    struct device_node *np = dev->of_node;
    int ret;

    dht22_sensor.sup_gpio = NULL;
    dht22_sensor.data_gpio = NULL;

    /* get GPIOs from Device Tree */
    dht22_sensor.sup_gpio = gpiod_get(dev, "dht22-supply", GPIOD_OUT_LOW);
    dht22_sensor.data_gpio = gpiod_get(dev, "dht22-data", GPIOD_IN);
    if( IS_ERR(dht22_sensor.sup_gpio) || IS_ERR(dht22_sensor.data_gpio) )
    {
        pr_err("dht22_dev: failed to get GPIOs from device tree\n");
        ret = -ENODEV;
        goto err_gpiod;
    }
    /* get irq for data line */
    int irq = gpiod_to_irq(dht22_sensor.data_gpio);
    if (irq < 0)
    {
        pr_err("dht22_dev: failed to get IRQ for data GPIO\n");
        ret = irq;
        goto err_gpiod;
    }
    if( request_irq(irq, dht22_data_irq_handler, 
                    IRQF_TRIGGER_FALLING, DEVICE_NAME, NULL) )
    {
        pr_err("dht22_dev: failed to request IRQ for data GPIO\n");
        ret = -EBUSY;
        goto err_gpiod;
    }
    dht22_sensor.last_read_time = 0;
    dht22_sensor.power_status = 0;
    dht22_sensor.data_available = 0;

    PDEBUG("DHT22 device initialized\n");
    return 0;

err_gpiod:
    if( !IS_ERR(dht22_sensor.sup_gpio) )
        gpiod_put(dht22_sensor.sup_gpio);
    if( !IS_ERR(dht22_sensor.data_gpio) ) 
        gpiod_put(dht22_sensor.data_gpio);
    return ret;
}

static int __init dht22_init(void)
{
    int ret;

    ret = alloc_chrdev_region(&dev_num, dht22_dev_minor, 1, DEVICE_NAME);
    if (ret < 0)
    {
        pr_err("dht22_dev: failed to allocate char device region\n");
        return ret;
    }
    dht22_dev_major = MAJOR(dev_num);

    cdev_init(&dht22_cdev, &dht22_fops);
    dht22_cdev.owner = THIS_MODULE;

    ret = cdev_add(&dht22_cdev, dev_num, 1);
    if (ret < 0)
    {
        pr_err("dht22_dev: failed to add cdev\n");
        goto err_cdev;
    }

    dht22_class = class_create(CLASS_NAME);
    if (IS_ERR(dht22_class)) 
    {
        pr_err("dht22_dev: failed to create class\n");
        ret = PTR_ERR(dht22_class);
        goto err_class;
    }

    dht22_device = device_create(dht22_class, NULL, dev_num, NULL, DEVICE_NAME);
    if (IS_ERR(dht22_device)) 
    {
        pr_err("dht_dev: failed to create device\n");
        ret = PTR_ERR(dht22_device);
        goto err_device;
    }

    dht22_device->of_node = of_find_node_by_name(NULL, "dht22-dev");
    if (!dht22_device->of_node) 
    {
        pr_err("dht22_dev: failed to find device tree node\n");
        ret = -ENODEV;
        goto err_dt_node;
    }

    ret = dht22_device_init(dht22_device);
    if (ret < 0)
    {
        pr_err("dht22_dev: failed to initialize DHT22 sensor\n");
        goto err_sensor;
    }

    pr_info("DHT22 device driver initialized\n");
    return 0;

err_sensor:
err_dt_node:
    device_destroy(dht22_class, dev_num);
err_device:
    class_destroy(dht22_class);
err_class:
    cdev_del(&dht22_cdev);
err_cdev:
    unregister_chrdev_region(dev_num, 1);
    return ret;
}

static void __exit dht22_exit(void)
{
    free_irq(gpiod_to_irq(dht22_sensor.data_gpio), NULL);
    gpiod_set_value(dht22_sensor.sup_gpio, 0); // power off sensor
    gpiod_put(dht22_sensor.sup_gpio);
    gpiod_put(dht22_sensor.data_gpio);
    device_destroy(dht22_class, dev_num);
    class_destroy(dht22_class);
    cdev_del(&dht22_cdev);
    unregister_chrdev_region(dev_num, 1);
    pr_info("DHT22 device driver exited\n");
}

module_init(dht22_init);
module_exit(dht22_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("woytzek");
MODULE_DESCRIPTION("DHT22 sensor device driver");