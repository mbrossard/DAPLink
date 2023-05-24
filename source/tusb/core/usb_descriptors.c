/**
 * @file    usb_descriptors.c
 * @brief   USB Descriptors
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2023 Arm Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "tusb.h"
#include "dap_strings.h"

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,

#if (USBD_BOS_ENABLE)
    .bcdUSB             = 0x0210, // USB Specification version 2.10 for BOS
#elif ((USBD_HS_ENABLE) || (USBD_MULTI_IF))
    .bcdUSB             = 0x0200, // USB Specification version 2.00
#else
    .bcdUSB             = 0x0110, // USB Specification version 1.10
#endif

#if (USBD_MULTI_IF)
    .bDeviceClass       = 0xEF, // USB Device Class Miscellaneous
    .bDeviceSubClass    = 0x02,
    .bDeviceProtocol    = 0x01,
#else
    .bDeviceClass       = 0x00, // Each interface specifies its own
    .bDeviceSubClass    = 0x00, // Each interface specifies its own
    .bDeviceProtocol    = 0x00,
#endif
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = 0x0D28, // Arm
    .idProduct          = 0x0204, // DAPLink
    .bcdDevice          = 0x1000, // Version 10.00

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const * tud_descriptor_device_cb(void)
{
  return (uint8_t const *) &desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+

enum {
#if HID_ENDPOINT
    ITF_NUM_HID,
#endif
#if MSC_ENDPOINT
    ITF_NUM_MSC,
#endif
#if CDC_ENDPOINT
    ITF_NUM_CDC_COM,
    ITF_NUM_CDC_DATA,
#endif
#if BULK_ENDPOINT
    ITF_NUM_BULK,
#endif
    ITF_NUM_TOTAL
};

#define USBD_HID_EP_INTIN           0x81
#define USBD_HID_EP_INTOUT          0x01
#define USBD_MSC_EP_BULKIN          0x82
#define USBD_MSC_EP_BULKOUT         0x02
#define USBD_CDC_ACM_EP_INTIN       0x83
#define USBD_CDC_ACM_EP_BULKIN      0x84
#define USBD_CDC_ACM_EP_BULKOUT     0x04
#define USBD_BULK_EP_BULKIN         0x85
#define USBD_BULK_EP_BULKOUT        0x05

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + \
  (HID_ENDPOINT * TUD_HID_INOUT_DESC_LEN) + \
  (MSC_ENDPOINT * TUD_MSC_DESC_LEN) + \
  (CDC_ENDPOINT * TUD_CDC_DESC_LEN) + \
  (BULK_ENDPOINT * TUD_VENDOR_DESC_LEN))

#if HID_ENDPOINT
static uint8_t const desc_hid_report[] =
{
  TUD_HID_REPORT_DESC_GENERIC_INOUT(CFG_TUD_HID_EP_BUFSIZE)
};

uint8_t const * tud_hid_descriptor_report_cb(uint8_t itf)
{
  (void) itf;
  return desc_hid_report;
}
#endif

uint8_t const desc_fs_configuration[] =
{
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
#if HID_ENDPOINT
  TUD_HID_INOUT_DESCRIPTOR(ITF_NUM_HID, 4, HID_ITF_PROTOCOL_NONE, sizeof(desc_hid_report),
                           USBD_HID_EP_INTOUT, USBD_HID_EP_INTIN, CFG_TUD_HID_EP_BUFSIZE, 1),
#endif
#if MSC_ENDPOINT
  // Interface number, string index, EP Out & EP In address, EP size
  TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 7, USBD_MSC_EP_BULKOUT, USBD_MSC_EP_BULKIN, 64),
#endif
#if CDC_ENDPOINT
  // Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_COM, 6, USBD_CDC_ACM_EP_INTIN, 64, USBD_CDC_ACM_EP_BULKOUT, USBD_CDC_ACM_EP_BULKIN, 64),
#endif
#if BULK_ENDPOINT
  TUD_VENDOR_DESCRIPTOR(ITF_NUM_BULK, 5, USBD_BULK_EP_BULKOUT, USBD_BULK_EP_BULKIN, 64),
#endif
};

#if TUD_OPT_HIGH_SPEED

uint8_t const desc_hs_configuration[] =
{
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
#if HID_ENDPOINT
  // HID (named interface)
  TUD_HID_INOUT_DESCRIPTOR(ITF_NUM_HID, 4, HID_ITF_PROTOCOL_NONE, sizeof(desc_hid_report),
                           USBD_HID_EP_INTOUT, USBD_HID_EP_INTIN, CFG_TUD_HID_EP_BUFSIZE, 1),
#endif
#if MSC_ENDPOINT
  // Interface number, string index, EP Out & EP In address, EP size
  TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 7, USBD_MSC_EP_BULKOUT, USBD_MSC_EP_BULKIN, 512),
#endif
#if CDC_ENDPOINT
  // Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_COM, 6, USBD_CDC_ACM_EP_INTIN, 64, USBD_CDC_ACM_EP_BULKOUT, USBD_CDC_ACM_EP_BULKIN, 512),
#endif
#if BULK_ENDPOINT
  TUD_VENDOR_DESCRIPTOR(ITF_NUM_BULK, 5, USBD_BULK_EP_BULKOUT, USBD_BULK_EP_BULKIN, 512),
#endif
};

// other speed configuration
uint8_t desc_other_speed_config[CONFIG_TOTAL_LEN];

// device qualifier is mostly similar to device descriptor since we don't change configuration based on speed
tusb_desc_device_qualifier_t const desc_device_qualifier =
{
  .bLength            = sizeof(tusb_desc_device_qualifier_t),
  .bDescriptorType    = TUSB_DESC_DEVICE_QUALIFIER,
#if (USBD_BOS_ENABLE)
    .bcdUSB             = 0x0210, // USB Specification version 2.10 for BOS
#elif ((USBD_HS_ENABLE) || (USBD_MULTI_IF))
    .bcdUSB             = 0x0200, // USB Specification version 2.00
#else
    .bcdUSB             = 0x0110, // USB Specification version 1.10
#endif

  .bDeviceClass       = TUSB_CLASS_MISC,
  .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
  .bDeviceProtocol    = MISC_PROTOCOL_IAD,

  .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
  .bNumConfigurations = 0x01,
  .bReserved          = 0x00
};

// Invoked when received GET DEVICE QUALIFIER DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete.
// device_qualifier descriptor describes information about a high-speed capable device that would
// change if the device were operating at the other speed. If not highspeed capable stall this request.
uint8_t const* tud_descriptor_device_qualifier_cb(void)
{
  return (uint8_t const*) &desc_device_qualifier;
}

// Invoked when received GET OTHER SEED CONFIGURATION DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
// Configuration descriptor in the other speed e.g if high speed then this is for full speed and vice versa
uint8_t const* tud_descriptor_other_speed_configuration_cb(uint8_t index)
{
  (void) index; // for multiple configurations

  // if link speed is high return fullspeed config, and vice versa
  // Note: the descriptor type is OHER_SPEED_CONFIG instead of CONFIG
  memcpy(desc_other_speed_config,
         (tud_speed_get() == TUSB_SPEED_HIGH) ? desc_fs_configuration : desc_hs_configuration,
         CONFIG_TOTAL_LEN);

  desc_other_speed_config[1] = TUSB_DESC_OTHER_SPEED_CONFIG;

  return desc_other_speed_config;
}

#endif // highspeed

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
  (void) index; // for multiple configurations

#if TUD_OPT_HIGH_SPEED
  // Although we are highspeed, host may be fullspeed.
  return (tud_speed_get() == TUSB_SPEED_HIGH) ?  desc_hs_configuration : desc_fs_configuration;
#else
  return desc_fs_configuration;
#endif
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// array of pointer to string descriptors
char const * string_desc_arr [] =
{
  (const char[]) { 0x09, 0x04 }, // 0: is supported language is English (0x0409)
  "Arm",                    // 1: Manufacturer
  CMSIS_DAP_PRODUCT_NAME,   // 2: Product
  NULL,                 // 3: Serial, uses flash unique ID
  "CMSIS-DAP v1",           // 4: Interface descriptor for HID transport
  "CMSIS-DAP v2",           // 5: Interface descriptor for Bulk transport
  "mbed Serial Port",       // 6: Interface descriptor for CDC
  "USB_MSC",                // 7: Interface descriptor for MSC
};

static uint16_t _desc_str[32];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void) langid;

  uint8_t chr_count;

  if (index == 0) {
    memcpy(&_desc_str[1], string_desc_arr[0], 2);
    chr_count = 1;
  } else {
    // Convert ASCII string into UTF-16
    if (!(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))) {
      return NULL;
    }

    const char* str = index != 3 ? string_desc_arr[index] : info_get_unique_id();

    // Cap at max char
    chr_count = strlen(str);
    if ( chr_count > 31 ) {
      chr_count = 31;
    }

    for(uint8_t i = 0; i < chr_count; i++) {
        _desc_str[i + 1] = str[i];
    }
  }

  // first byte is length (including header), second byte is string type
  _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);

  return _desc_str;
}

/* [incoherent gibbering to make Windows happy] */

//--------------------------------------------------------------------+
// BOS Descriptor
//--------------------------------------------------------------------+

/* Microsoft OS 2.0 registry property descriptor
Per MS requirements https://msdn.microsoft.com/en-us/library/windows/hardware/hh450799(v=vs.85).aspx
device should create DeviceInterfaceGUIDs. It can be done by driver and
in case of real PnP solution device should expose MS "Microsoft OS 2.0
registry property descriptor". Such descriptor can insert any record
into Windows registry per device/configuration/interface. In our case it
will insert "DeviceInterfaceGUIDs" multistring property.


https://developers.google.com/web/fundamentals/native-hardware/build-for-webusb/
(Section Microsoft OS compatibility descriptors)
*/
#define MS_OS_20_DESC_LEN  0xB2

#define BOS_TOTAL_LEN      (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)

uint8_t const desc_bos[] =
{
  // total length, number of device caps
  TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 1),

  // Microsoft OS 2.0 descriptor
  TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, 1)
};

uint8_t const desc_ms_os_20[] =
{
  // Set header: length, type, windows version, total length
  U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR), U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),

  // Configuration subset header: length, type, configuration index, reserved, configuration total length
  U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION), 0, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN-0x0A),

  // Function Subset header: length, type, first interface, reserved, subset length
  U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION), 0, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN-0x0A-0x08),

  // MS OS 2.0 Compatible ID descriptor: length, type, compatible ID, sub compatible ID
  U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID), 'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // sub-compatible

  // MS OS 2.0 Registry property descriptor: length, type
  U16_TO_U8S_LE(MS_OS_20_DESC_LEN-0x0A-0x08-0x08-0x14), U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),
  U16_TO_U8S_LE(0x0007), U16_TO_U8S_LE(0x002A), // wPropertyDataType, wPropertyNameLength and PropertyName "DeviceInterfaceGUIDs\0" in UTF-16
  'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00, 'I', 0x00, 'n', 0x00, 't', 0x00, 'e', 0x00,
  'r', 0x00, 'f', 0x00, 'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00, 'D', 0x00, 's', 0x00, 0x00, 0x00,
  U16_TO_U8S_LE(0x0050), // wPropertyDataLength
  // bPropertyData "{CDB3B5AD-293B-4663-AA36-1AAE46463776}" as a UTF-16 string (b doesn't mean bytes)
  '{', 0x00, 'C', 0x00, 'D', 0x00, 'B', 0x00, '3', 0x00, 'B', 0x00, '5', 0x00, 'A', 0x00, 'D', 0x00, '-', 0x00,
  '2', 0x00, '9', 0x00, '3', 0x00, 'B', 0x00, '-', 0x00, '4', 0x00, '6', 0x00, '6', 0x00, '3', 0x00, '-', 0x00,
  'A', 0x00, 'A', 0x00, '3', 0x00, '6', 0x00, '-', 0x00, '1', 0x00, 'A', 0x00, 'A', 0x00, 'E', 0x00, '4', 0x00,
  '6', 0x00, '4', 0x00, '6', 0x00, '3', 0x00, '7', 0x00, '7', 0x00, '6', 0x00, '}', 0x00, 0x00, 0x00, 0x00, 0x00
};

TU_VERIFY_STATIC(sizeof(desc_ms_os_20) == MS_OS_20_DESC_LEN, "Incorrect size");

uint8_t const * tud_descriptor_bos_cb(void)
{
  return desc_bos;
}
