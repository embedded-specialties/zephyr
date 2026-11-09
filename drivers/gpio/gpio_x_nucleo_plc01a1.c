/* drivers/gpio/gpio_x_nucleo_plc01a1.c */
#define DT_DRV_COMPAT st_x_nucleo_plc01a1

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio/gpio_utils.h>

struct plc_config {
    struct gpio_driver_config common;
    struct spi_dt_spec bus;
    struct gpio_dt_spec cs1_clt;
    struct gpio_dt_spec cs2_vni;
};

struct plc_data {
    struct gpio_driver_data common;
    struct k_mutex lock;
    uint8_t output_state; 
};

static int plc_port_update_hardware(const struct device *dev, uint8_t *rx_buf_out)
{
    const struct plc_config *config = dev->config;
    struct plc_data *data = dev->data;

    /* Single 16-bit values for independent communication */
    uint16_t tx_vni = 0;
    uint16_t rx_vni = 0;
    uint16_t tx_clt = 0;
    uint16_t rx_clt = 0;

    /* 
     * 1. Align and format the VNI8200XP Output State:
     * Shift left by 1 bit to clear room for the parity bit at Bit 0.
     */
    uint16_t vni_word = ((uint16_t)data->output_state) << 1;

    /* 
     * 2. Compute Even Parity:
     * __builtin_parity returns 1 if the number of set bits is odd.
     * Force this into Bit 0 to guarantee an even bit count.
     */
    uint16_t parity_bit = __builtin_parity(vni_word);
    vni_word |= (parity_bit & 0x01);

    /* 
     * 3. Fix Little-to-Big Endian alignment for the STM32H7 SPI master block:
     */
    tx_vni = __builtin_bswap16(vni_word);
    tx_clt = 0x0000; // CLT01 operation dummy command

    /* 4. Map into standard Zephyr 16-bit (2 byte) sub-buffers */
    struct spi_buf tx_vni_spi = { .buf = &tx_vni, .len = 2 };
    const struct spi_buf_set tx_vni_set = { .buffers = &tx_vni_spi, .count = 1 };
    struct spi_buf rx_vni_spi = { .buf = &rx_vni, .len = 2 };
    const struct spi_buf_set rx_vni_set = { .buffers = &rx_vni_spi, .count = 1 };

    struct spi_buf tx_clt_spi = { .buf = &tx_clt, .len = 2 };
    const struct spi_buf_set tx_clt_set = { .buffers = &tx_clt_spi, .count = 1 };
    struct spi_buf rx_clt_spi = { .buf = &rx_clt, .len = 2 };
    const struct spi_buf_set rx_clt_set = { .buffers = &rx_clt_spi, .count = 1 };

    /* 
     * 5. Execute TRANSACTION 1: Update VNI8200XP (Kicks the watchdog!)
     * Manually assert CS2 (VNI) low, execute transceive, then raise high.
     */
    gpio_pin_set_dt(&config->cs2_vni, 1); 
    int ret = spi_transceive_dt(&config->bus, &tx_vni_set, &rx_vni_set);
    gpio_pin_set_dt(&config->cs2_vni, 0); 
    if (ret < 0) {
        return ret;
    }

    /* 
     * 6. Execute TRANSACTION 2: Read CLT01 Inputs
     * Manually assert CS1 (CLT) low, execute transceive, then raise high.
     */
    gpio_pin_set_dt(&config->cs1_clt, 1);
    ret = spi_transceive_dt(&config->bus, &tx_clt_set, &rx_clt_set);
    gpio_pin_set_dt(&config->cs1_clt, 0);
    if (ret < 0) {
        return ret;
    }

    /* 
     * 7. De-serialize and unpack data back to the GPIO API layer:
     */
    uint16_t vni_status = __builtin_bswap16(rx_vni);
    uint16_t clt_inputs = __builtin_bswap16(rx_clt);

    rx_buf_out[0] = (uint8_t)(vni_status >> 8); // Diagnostic status out
    rx_buf_out[1] = (uint8_t)(clt_inputs >> 8); // Physical input matrix pins

    return 0;
}

static int plc_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
    if (pin < 8 && (flags & GPIO_OUTPUT)) {
        return -ENOTSUP; /* Pins 0-7 are read-only CLT01 inputs */
    }
    if (pin >= 8 && pin < 16 && (flags & GPIO_INPUT)) {
        return -ENOTSUP; /* Pins 8-15 are write-only VNI8200 outputs */
    }
    return 0;
}

static int plc_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
    struct plc_data *data = dev->data;
    uint8_t rx_buf[2] = {0};
    int ret;

    k_mutex_lock(&data->lock, K_FOREVER);
    ret = plc_port_update_hardware(dev, rx_buf);
    k_mutex_unlock(&data->lock);

    if (ret < 0) {
        return ret;
    }

    /* 
     * Shift combined state: 
     * Lower byte (Pins 0-7) = CLT01 Inputs (swapped into rx_buf[1] due to daisy-chain positioning)
     * Upper byte (Pins 8-15) = Current VNI8200 local driver output cache
     */
    *value = (gpio_port_value_t)(rx_buf[1] | ((uint32_t)data->output_state << 8));
    return 0;
}

static int plc_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask, gpio_port_value_t value)
{
    struct plc_data *data = dev->data;
    uint8_t rx_buf[2] = {0};
    int ret;
    
    k_mutex_lock(&data->lock, K_FOREVER);
    
    /* Shift virtual pins 8-15 down to map to physical byte 0-7 */
    uint8_t target_mask = (uint8_t)(mask >> 8);
    uint8_t target_val = (uint8_t)(value >> 8);
    
    data->output_state = (data->output_state & ~target_mask) | (target_val & target_mask);
    
    ret = plc_port_update_hardware(dev, rx_buf);
    
    k_mutex_unlock(&data->lock);
    return ret;
}

static int plc_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
    return plc_port_set_masked_raw(dev, pins, pins);
}

static int plc_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
    return plc_port_set_masked_raw(dev, pins, 0);
}

static int plc_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
    struct plc_data *data = dev->data;
    uint32_t current_outputs = ((uint32_t)data->output_state) << 8;
    return plc_port_set_masked_raw(dev, pins, ~current_outputs);
}

static const struct gpio_driver_api plc_api = {
    .pin_configure = plc_pin_configure,
    .port_get_raw = plc_port_get_raw,
    .port_set_masked_raw = plc_port_set_masked_raw,
    .port_set_bits_raw = plc_port_set_bits_raw,
    .port_clear_bits_raw = plc_port_clear_bits_raw,
    .port_toggle_bits = plc_port_toggle_bits,
};

static int plc_init(const struct device *dev)
{
    const struct plc_config *config = dev->config;
    struct plc_data *data = dev->data;
    uint8_t dummy_rx[2] = {0};

    if (!spi_is_ready_dt(&config->bus)) {
        return -ENODEV;
    }

    k_mutex_init(&data->lock);

    /* 
     * Initial output data cache set to all outputs off (0x00).
     */
    data->output_state = 0x00;

    /* 
     * Kick the VNI8200XP Watchdog immediately during initialization.
     * This pushes a clean 0x00 command frame with accurate Even Parity out to the shield,
     * resetting the VNI8200XP watchdog state machine and turning off the startup FAULT LED.
     */
    int ret = plc_port_update_hardware(dev, dummy_rx);
    if (ret < 0) {
        /* SPI transaction failed during initialization */
        return ret; 
    }

    return 0;
}


#define PLC_DEVICE_INIT(inst)                                                                                       \
    static const struct plc_config plc_config_##inst = {                                                            \
        .common = { .port_pin_mask = (gpio_port_pins_t)GPIO_PORT_PIN_MASK_FROM_NGPIOS(16) },                        \
        .bus = SPI_DT_SPEC_INST_GET(inst, SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(16)),    \
    };                                                                                                              \
    static struct plc_data plc_data_##inst;                                                                         \
    DEVICE_DT_INST_DEFINE(inst, plc_init, NULL, &plc_data_##inst,                                                   \
                          &plc_config_##inst, POST_KERNEL, 80, &plc_api);

DT_INST_FOREACH_STATUS_OKAY(PLC_DEVICE_INIT)