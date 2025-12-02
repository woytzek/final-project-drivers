#ifdef __KERNEL__
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
#include <linux/jiffies.h>
#else
/* stubs to avoid includePath errors when not building with kernel headers */
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

typedef int ssize_t;
typedef long loff_t;
typedef unsigned int dev_t;

#define pr_info(...) ((void)0)
#define pr_err(...) ((void)0)
#define THIS_MODULE NULL
#define MODULE_LICENSE(x) ((void)0)
#define MODULE_AUTHOR(x) ((void)0)
#define MODULE_DESCRIPTION(x) ((void)0)
#define __init
#define __exit

/* Minimal helpers to satisfy simple checks in this file */
#define IS_ERR(x) (0)
#define PTR_ERR(x) (0)

#endif

//#define GPIO_RW_DEBUG

#undef PDEBUG
#ifdef GPIO_RW_DEBUG
#  ifdef __KERNEL__
     /* This one is for kernel space */
#    define PDEBUG(fmt, args...) printk( KERN_DEBUG "gpio_rw: " fmt, ## args)
#  else
     /* This one is for user space */
#    define PDEBUG(fmt, args...) fprintf(stderr, "gpio_rw: " fmt, ## args)
#  endif
#else
#  define PDEBUG(fmt, args...) /* not debugging: nothing */
#endif

#include "gpio_rw.h"

#define DEVICE_NAME "gpio_rw"
#define CLASS_NAME  "gpio_rw_class"

int gpio_rw_major =   0; // use dynamic major
int gpio_rw_minor =   0;

DEFINE_SPINLOCK(gpio_rw_lock);

struct gpio_rw_pin_state
{
    unsigned int value;     // debounced value, 0: low, 1: high
    unsigned int ud_value;  // actual, nto debounced value from gpio
    unsigned long timestamp;
};

struct gpio_rw_pin
{
        struct gpio_desc *desc;
        unsigned int num;
        struct gpio_rw_pin_state state;
};

struct gpio_rw_data
{
    struct gpio_rw_pin pins[MAX_PINS];
    unsigned int num_pins;
};

static struct gpio_rw_data gpio_data;

static dev_t dev_num;
static struct cdev gpio_cdev;
static struct class *gpio_class;
static struct device *gpio_device;

#define DEBOUNCE_TIME_MS 40

/* interrupt handler 
    - debounces input changes in both directions
*/
static irqreturn_t gpio_rw_irq_handler(int irq, void *dev_id)
{
    struct gpio_rw_pin *pin = (struct gpio_rw_pin *)dev_id;
    unsigned int new_ud_value = gpiod_get_value(pin->desc);

    unsigned long flags;
    spin_lock_irqsave(&gpio_rw_lock, flags);

    if( new_ud_value != pin->state.ud_value )
    {
        // value changed, reset timestamp
        pin->state.ud_value = new_ud_value;
        pin->state.timestamp = jiffies;
        PDEBUG("gpio_rw: GPIO %u unstable change to %d\n", pin->num, new_ud_value);
    }

    spin_unlock_irqrestore(&gpio_rw_lock, flags);

    return IRQ_HANDLED;
}

static int gpio_rw_init_pins(struct device *dev)
{
    struct device_node *np = dev->of_node;
    int i, ret;

    /* get number of pins */
    gpio_data.num_pins = of_count_phandle_with_args(np, "gpiorw-gpios", "#gpio-cells");
    if (gpio_data.num_pins <= 0 || gpio_data.num_pins > MAX_PINS) 
    {
        pr_err("gpio_rw: invalid number of gpios in DT\n");
        return -EINVAL;
    }
    /* register gpios */
    for (i = 0; i < gpio_data.num_pins; i++) 
    {
        gpio_data.pins[i].desc = gpiod_get_index(dev, "gpiorw", i, GPIOD_IN);
        if (IS_ERR(gpio_data.pins[i].desc)) 
        {
            pr_err("gpio_rw: failed to get gpio index %d\n", i);
            ret = PTR_ERR(gpio_data.pins[i].desc);
            goto err_get;
        }
        gpio_data.pins[i].num = desc_to_gpio(gpio_data.pins[i].desc);
        PDEBUG("gpio_rw: got gpio %u at index %d\n", gpio_data.pins[i].num, i);
    }
    /* map interrupts */
    unsigned long flags;
    spin_lock_irqsave(&gpio_rw_lock, flags);
    for( i = 0; i < gpio_data.num_pins; i++ ) 
    {
        int irq = gpiod_to_irq(gpio_data.pins[i].desc);
        if( irq < 0 )
        {
            pr_err("gpio_rw: failed to map gpio %u to irq\n", gpio_data.pins[i].num);
            ret = irq;
            goto err_irq;
        }
        PDEBUG("gpio_rw: gpio %u mapped to irq %d\n", gpio_data.pins[i].num, irq);
        if( request_irq(irq, gpio_rw_irq_handler, 
                        IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                        DEVICE_NAME, (void*)&gpio_data.pins[i]) < 0 )
        {
            pr_err("gpio_rw: failed to request irq %d for gpio %u\n", irq, gpio_data.pins[i].num);
            ret = -EBUSY;
            goto err_irq;
        }
    }
    spin_unlock_irqrestore(&gpio_rw_lock, flags);
    return 0;

err_irq:
    while(--i >= 0)
    {
        free_irq(gpiod_to_irq(gpio_data.pins[i].desc), (void*)&gpio_data.pins[i]);
    }
    i = gpio_data.num_pins;
err_get:
    while (--i >= 0)
    {
        gpiod_put(gpio_data.pins[i].desc);
    }
    gpio_data.num_pins = 0;
    spin_unlock_irqrestore(&gpio_rw_lock, flags);
    return ret;
}

static void gpio_rw_free_pins(void)
{
    int i;
    for (i = 0; i < gpio_data.num_pins; i++)
    {
        free_irq(gpiod_to_irq(gpio_data.pins[i].desc), (void*)&gpio_data.pins[i]);
        gpiod_put(gpio_data.pins[i].desc);
    }
}

/* File operations */

static int gpio_rw_open(struct inode *inode, struct file *file)
{
    return 0;
}

static loff_t gpio_rw_llseek(struct file *file, loff_t offset, int whence)
{
    loff_t newpos;

    switch(whence) 
    {
        case SEEK_SET:
            if( offset != 0 )
                return -EINVAL;
            newpos = offset;
            break;
        case SEEK_CUR:
        case SEEK_END:
        default:
            return -EINVAL;
    }

    if (newpos < 0 || newpos > 1)
        return -EINVAL;

    file->f_pos = newpos;
    return newpos;
}

static ssize_t gpio_rw_read(struct file *file, char __user *buf,
                                   size_t len, loff_t *offset)
{
    unsigned char value = 0;
    int i;

    if (len < 1)
        return -EINVAL;

    if( *offset != 0 )
        return 0; // EOF

    unsigned long flags;
    spin_lock_irqsave(&gpio_rw_lock, flags);
    for (i = 0; i < gpio_data.num_pins; i++) 
    {
        if( gpio_data.pins[i].state.ud_value != gpio_data.pins[i].state.value )
        {
            // value changed, check debounce time
            if( time_after(jiffies, gpio_data.pins[i].state.timestamp + msecs_to_jiffies(DEBOUNCE_TIME_MS)) )
            {
                // debounce time passed, update debounced value
                gpio_data.pins[i].state.value = gpio_data.pins[i].state.ud_value;
                PDEBUG("gpio_rw: GPIO %u debounced to %d\n", gpio_data.pins[i].num, gpio_data.pins[i].state.value);
            }
        }
        if (gpio_data.pins[i].state.value)
        {
            value |= (1 << i);
        }
    }
    spin_unlock_irqrestore(&gpio_rw_lock, flags);
    *offset += 1;

    PDEBUG("gpio_rw: read value 0x%02x\n", value);

    if (copy_to_user(buf, &value, 1))
        return -EFAULT;

    return 1;
}

static const struct file_operations gpio_fops = 
{
    .owner = THIS_MODULE,
    .open  = gpio_rw_open,
    .read  = gpio_rw_read,
    .llseek = gpio_rw_llseek,
};

static int __init gpio_rw_init(void)
{
    int ret;

    ret = alloc_chrdev_region(&dev_num, gpio_rw_minor, 1, DEVICE_NAME);
    if (ret < 0)
    {
        pr_err("gpio_rw: failed to allocate char device region\n");
        return ret;
    }
    gpio_rw_major = MAJOR(dev_num);

    cdev_init(&gpio_cdev, &gpio_fops);
    gpio_cdev.owner = THIS_MODULE;

    ret = cdev_add(&gpio_cdev, dev_num, 1);
    if (ret < 0)
    {
        pr_err("gpio_rw: failed to add cdev\n");
        goto err_cdev;
    }

    gpio_class = class_create(CLASS_NAME);
    if (IS_ERR(gpio_class)) 
    {
        pr_err("gpio_rw: failed to create class\n");
        ret = PTR_ERR(gpio_class);
        goto err_class;
    }

    gpio_device = device_create(gpio_class, NULL, dev_num, NULL, DEVICE_NAME);
    if (IS_ERR(gpio_device)) 
    {
        pr_err("gpio_rw: failed to create device\n");
        ret = PTR_ERR(gpio_device);
        goto err_device;
    }

    /* initialize 'gpio_device->of_node' pointer to have access to GPIO from device tree */
    gpio_device->of_node = of_find_node_by_name(NULL, "gpio-rw");
    if (!gpio_device->of_node) 
    {
        pr_err("gpio_rw: failed to find device tree node\n");
        ret = -ENODEV;
        goto err_dt_node;
    }

    ret = gpio_rw_init_pins(gpio_device);
    if (ret < 0)
    {
        pr_err("gpio_rw: failed to initialize GPIO pins\n");
        goto err_gpio;
    }

    pr_info("gpio_rw: loaded, device /dev/%s\n", DEVICE_NAME);
    return 0;

err_gpio:
err_dt_node:
    device_destroy(gpio_class, dev_num);
err_device:
    class_destroy(gpio_class);
err_class:
    cdev_del(&gpio_cdev);
err_cdev:
    unregister_chrdev_region(dev_num, 1);
    return ret;
}

static void __exit gpio_rw_exit(void)
{
    gpio_rw_free_pins();
    device_destroy(gpio_class, dev_num);
    class_destroy(gpio_class);
    cdev_del(&gpio_cdev);
    unregister_chrdev_region(dev_num, 1);
    pr_info("gpio_rw: unloaded\n");
}

module_init(gpio_rw_init);
module_exit(gpio_rw_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("woytzek");
MODULE_DESCRIPTION("Simple GPIO R/W char device using gpiod and DT with IOCTL support");