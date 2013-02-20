#include <linux/autoconf.h>

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)

#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/platform_device.h>
#include <asm-generic/gpio.h>

#define GPIO_WCLOCK_VALID (230)
#define GPIO_FN_BTN1      (231)
#define GPIO_FN_BTN2      (232)
#define GPIO_WCLK_SW      (233)
#define GPIO_SYNC_STATUS  (234)

#define DEBOUNCE_INTERVAL_MS 100 // Set to 100 until CO 3 boards arrive, which will have debounced buttons and all

static struct gpio_keys_button microblaze_gpio_keys_table[] = {
        { BTN_0, GPIO_WCLOCK_VALID, 0, "gpio-wclock", EV_KEY, 0, DEBOUNCE_INTERVAL_MS },
        { BTN_1, GPIO_FN_BTN1     , 0, "gpio-fnbtn1", EV_KEY, 0, DEBOUNCE_INTERVAL_MS },
        { BTN_2, GPIO_FN_BTN2     , 0, "gpio-fnbtn2", EV_KEY, 0, DEBOUNCE_INTERVAL_MS },
        { BTN_3, GPIO_WCLK_SW     , 0, "gpio-wclksw", EV_KEY, 0, DEBOUNCE_INTERVAL_MS },
        { BTN_4, GPIO_SYNC_STATUS , 0, "gpio-syncst", EV_KEY, 0, DEBOUNCE_INTERVAL_MS },
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
#error "No GPIO events available for Mosaic platform! Make sure GPIO and gpio keyboard driver are enabled."
#endif
