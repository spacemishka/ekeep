cmake_minimum_required(VERSION 3.20.0)

zephyr_library()

# Only build the local compatibility driver when explicitly enabled
# and upstream ZMK is not already providing CONFIG_GPIO_595.
if(CONFIG_EKEEP_GPIO_595_COMPAT AND NOT CONFIG_GPIO_595)
  zephyr_library_sources(drivers/gpio/gpio_595_compat.c)
endif()
