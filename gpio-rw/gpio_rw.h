/*
 * gpio_rw.h
*/

#ifndef GPIO_RW_H
#define GPIO_RW_H

#define MAX_PINS 8

#define GPIO_RW_IOC_MAGIC  'g'
#define GPIO_RW_IOC_SET_PINS      _IOW(GPIO_RW_IOC_MAGIC, 1, struct gpio_rw_pins)
#define GPIO_RW_IOC_SET_DIRECTION _IOW(GPIO_RW_IOC_MAGIC, 2, struct gpio_rw_direction)

#if 0
struct gpio_rw_pin
{
    unsigned int pin;
    unsigned int direction; // 0: input, 1: output
};

struct gpio_rw_pins 
{
    struct gpio_rw_pin pins[MAX_PINS];
    unsigned int num_pins;
};

struct gpio_rw_direction 
{
    unsigned int pin_index;
    unsigned int direction; // 0: input, 1: output
};
#endif

#endif /* GPIO_RW_H */