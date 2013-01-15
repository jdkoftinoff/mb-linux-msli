#include <linux/autoconf.h>

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)

#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/platform_device.h>
#include <asm-generic/gpio.h>

#define GPIO_WCLOCK_VALID (230)

static struct gpio_keys_button microblaze_gpio_keys_table[] = {
        { BTN_0, GPIO_WCLOCK_VALID, 0, "gpio-wclock", EV_KEY, 0, 0 },
};

static struct gpio_keys_platform_data microblaze_gpio_keys_data __initdata = {
        .buttons        = microblaze_gpio_keys_table,
        .nbuttons       = ARRAY_SIZE(microblaze_gpio_keys_table),
        .rep            = 0,
};

static struct platform_device microblaze_device_gpiokeys __initdata = {
        .name   = "gpio-keys",
        .dev = {
                .platform_data = &microblaze_gpio_keys_data,
        },
};

static int __init gpiobuttons_platform_init(void)
{
        platform_device_register(&microblaze_device_gpiokeys);
        return 0;
}

device_initcall(gpiobuttons_platform_init);

#else
#error "No GPIO events available for Mosaic platform!"
#endif
