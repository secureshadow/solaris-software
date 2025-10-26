#include "macros_esp.h"
#include "spi.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "osal/queue.h"


void app_main()
{
   void* p_queue_handle = NULL;
   retval_t ret = SPP_OK;
   ret = SPP_OSAL_QueueCreate(p_queue_handle, 100, 10);

}