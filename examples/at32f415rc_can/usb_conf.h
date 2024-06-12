#ifndef __USB_CONF_H
#define __USB_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "at32f415_usb.h"
#include "at32f415.h"
#include "stdio.h"

#define USE_OTG_DEVICE_MODE
/* #define USE_OTG_HOST_MODE */

#define USB_VIRTUAL_COMPORT

#define USB_ID                           0
#define OTG_CLOCK                        CRM_OTGFS1_PERIPH_CLOCK
#define OTG_IRQ                          OTGFS1_IRQn
#define OTG_IRQ_HANDLER                  OTGFS1_IRQHandler
#define OTG_WKUP_IRQ                     OTGFS1_WKUP_IRQn

#define OTG_WKUP_HANDLER                 OTGFS1_WKUP_IRQHandler
#define OTG_WKUP_EXINT_LINE              EXINT_LINE_18

#define OTG_PIN_GPIO                     GPIOA
#define OTG_PIN_GPIO_CLOCK               CRM_GPIOA_PERIPH_CLOCK
#define OTG_PIN_VBUS                     GPIO_PINS_9
#define OTG_PIN_ID                       GPIO_PINS_10

#define OTG_PIN_SOF_GPIO                 GPIOA
#define OTG_PIN_SOF_GPIO_CLOCK           CRM_GPIOB_PERIPH_CLOCK
#define OTG_PIN_SOF                      GPIO_PINS_8

#ifdef USE_OTG_DEVICE_MODE
/* otg1 device fifo */
#define USBD_RX_SIZE                     128
#define USBD_EP0_TX_SIZE                 24
#define USBD_EP1_TX_SIZE                 20
#define USBD_EP2_TX_SIZE                 20
#define USBD_EP3_TX_SIZE                 20

#ifndef USB_EPT_MAX_NUM
#define USB_EPT_MAX_NUM                   4
#endif
#endif


#ifdef USE_OTG_HOST_MODE
#ifndef USB_HOST_CHANNEL_NUM
#define USB_HOST_CHANNEL_NUM             8
#endif

#define USBH_RX_FIFO_SIZE                128
#define USBH_NP_TX_FIFO_SIZE             96
#define USBH_P_TX_FIFO_SIZE              96
#endif

/* #define USB_SOF_OUTPUT_ENABLE */

/**
  * @brief ignore vbus detection, only available in at32f415xx revision C.
  *        at32f415xx revision B: (not support)
  *        the vbus detection pin (pa9) can not be used for other functionality.
  *        vbus pin must kept at VBUS or VDD.
  *
  *        at32f415xx revision C: (support)
  *        ignore vbus detection, the internal vbus is always valid.
  *        the vbus pin (pa9) can be used for other functionality.
  */
#define USB_VBUS_IGNORE

/* #define USB_LOW_POWER_WAKUP */

void usb_delay_ms(uint32_t ms);
void usb_delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif
