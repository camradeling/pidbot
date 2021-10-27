#include "usbserial.h"
#include "tusb.h"
//------------------------------------------------------------------------------
StackType_t  usb_device_stack[USBD_STACK_SIZE];
StaticTask_t usb_device_taskdef;
//------------------------------------------------------------------------------
StackType_t  cdc_stack[CDC_STACK_SIZE];
StaticTask_t cdc_taskdef;
//------------------------------------------------------------------------------
ComMessage UsbRxMSG = {.data={0}, .length=0};
ComMessage UsbTxMSG = {.data={0}, .length=0};
//------------------------------------------------------------------------------
void usb_device_task(void* param)
{
  (void) param;
  // This should be called after scheduler/kernel is started.
  // Otherwise it could cause kernel issue since USB IRQ handler does use RTOS queue API.
  tusb_init();
  // RTOS forever loop
  while (1)
  {
    // tinyusb device task
    tud_task();
  }
}
//------------------------------------------------------------------------------
void cdc_task(void* params)
{
  (void) params;
  while (1)
  {
    if(tud_cdc_available())
    {
      UsbRxMSG.length = tud_cdc_read(UsbRxMSG.data, sizeof(UsbRxMSG.data));
      MODBUS_HR[MBHR_REG_IN_UART_PACKS]++;
      int res = process_net_packet(&UsbRxMSG, &UsbTxMSG);
      if(res != MODBUS_PACKET_VALID_AND_PROCESSED)
        MODBUS_HR[MBHR_REG_IN_UART_PACKS_ERR]++;
      if(UsbTxMSG.length)
      {
        tud_cdc_write(UsbTxMSG.data, UsbTxMSG.length);
        tud_cdc_write_flush();
      }
      memset(UsbTxMSG.data,0,TXRX_BUFFER_SIZE);
      UsbTxMSG.length=0;
    }
  }
}
//------------------------------------------------------------------------------
