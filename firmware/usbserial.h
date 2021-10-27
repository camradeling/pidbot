#ifndef USBSERIAL_H
#define USBSERIAL_H
//------------------------------------------------------------------------------
#include "includes.h"
#include "tusb_fifo.h"
//------------------------------------------------------------------------------
#define USBD_STACK_SIZE     (3*configMINIMAL_STACK_SIZE/2)
#define CDC_STACK_SIZE      configMINIMAL_STACK_SIZE
//------------------------------------------------------------------------------
void usb_device_task(void* param);
void cdc_task(void* params);
//------------------------------------------------------------------------------
#endif/*USBSERIAL_H*/