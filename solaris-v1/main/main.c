#include <stdint.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "osal/queue.h"
#include "core/types.h"
#include "core/returntypes.h"

#include "macros_esp.h" // Hacen falta?
#include "spi.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "macros.h"



void app_main()
{
   void* p_QueueHandle = SPP_OSAL_QueueCreate(10, 20); // Crea cola y guarda su handle en p_QueueHandle

   retval_t ret;
   if (p_QueueHandle == NULL) ret = SPP_ERROR;
}








