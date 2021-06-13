#ifndef INCLUDES_H
#define INCLUDES_H
//------------------------------------------------------------------------------
/* Standard includes. */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
//------------------------------------------------------------------------------
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
//------------------------------------------------------------------------------
/* Library includes. */
#include "stm32f10x_conf.h"
#include "stm32f10x_it.h"
#include "stm32f10x.h"
//------------------------------------------------------------------------------
/* app includes. */
#include "app_config.h"
#include "serial.h"
#include "modbus.h"
#include "externs.h"
//------------------------------------------------------------------------------
#endif/*INCLUDES_H*/