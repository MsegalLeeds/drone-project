#include "tusb.h"
#include <string.h>

// TinyUSB's example VID/PID for a CDC-only local bench device.
static const tusb_desc_device_t device = {
    .bLength = sizeof(device), .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200, .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON, .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = 64, .idVendor = 0xcafe, .idProduct = 0x4001,
    .bcdDevice = 0x0100, .iManufacturer = 1, .iProduct = 2,
    .iSerialNumber = 3, .bNumConfigurations = 1
};
static const uint8_t config[] = {
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN, 0, 100),
    TUD_CDC_DESCRIPTOR(0, 0, 0x81, 8, 0x02, 0x82, 64)
};
uint8_t const *tud_descriptor_device_cb(void) { return (const uint8_t *)&device; }
uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    return index == 0 ? config : NULL;
}
const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t lang) {
    (void)lang;
    static const char *const strings[] = {"", "Drone Project", "STM32 Tilt Bench", "DroneTilt01"};
    static uint16_t descriptor[32];
    size_t len;
    if (index == 0) {
        descriptor[1] = 0x0409;
        len = 1;
    } else {
        if (index >= sizeof strings / sizeof strings[0]) return NULL;
        len = strlen(strings[index]);
        for (size_t i = 0; i < len; ++i) descriptor[i + 1] = strings[index][i];
    }
    descriptor[0] = (TUSB_DESC_STRING << 8) | (2 + 2 * len);
    return descriptor;
}
