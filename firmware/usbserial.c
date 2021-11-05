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
int RxInd=0;
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
void drop_rx()
{
  memset(UsbRxMSG.data,0,TXRX_BUFFER_SIZE);
  UsbRxMSG.length=0;
  RxInd=0;
}
//------------------------------------------------------------------------------
void cdc_task(void* params)
{
  (void) params;
  while (1)
  {
    int num=0;
    if(num=tud_cdc_available())
    {
      //UsbRxMSG.length = tud_cdc_read(UsbRxMSG.data, sizeof(UsbRxMSG.data));
      if(num + UsbRxMSG.length <= TXRX_BUFFER_SIZE)
      {
        UsbRxMSG.length += tud_cdc_read(&UsbRxMSG.data[RxInd], sizeof(UsbRxMSG.data)-RxInd);
        RxInd=UsbRxMSG.length;
      }
      else
      {
        UsbRxMSG.length = tud_cdc_read(UsbRxMSG.data, sizeof(UsbRxMSG.data));
        drop_rx();
        continue;
      }
      //checking valid modbus packet conditions
      if(UsbRxMSG.data[0] != MB_BROADCAST_ADDR)
      {
        if(MyMBAddr)
        {
          if(UsbRxMSG.data[0] != *MyMBAddr)
          {
            drop_rx();
            continue;
          }
        }
        else
        {
          drop_rx();
          continue;
        }
      }
      //packet can be not fully received yet
      if(UsbRxMSG.data[1] == 3 || UsbRxMSG.data[1] == 4 || UsbRxMSG.data[1] == 6)
      {
        if(UsbRxMSG.length < 8)
          continue;
      }
      else if(UsbRxMSG.data[1] == 16)
      {
        if(UsbRxMSG.length < 6)
          continue;
        uint16_t nlen = UsbRxMSG.data[4];
        if(nlen > TXRX_BUFFER_SIZE)
        {
          drop_rx();
          continue;
        }
        if(UsbRxMSG.length < 6+2*nlen)
          continue;
      }
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
      drop_rx();
    }
  }
}
//------------------------------------------------------------------------------
