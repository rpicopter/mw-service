#include <unistd.h>
#include <stdint.h>

#define GPIO_LOW 0
#define GPIO_HIGH 1

int8_t gpio_export(uint8_t gpio);

int8_t gpio_unexport(uint8_t gpio);

int8_t gpio_set_value(uint8_t pin, uint8_t value);
