#include <stdio.h>
#include "macros_esp.h"
#include "spi.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "macros.h"
#include "osal/task.h"
#include "core/returntypes.h"
#include "core/types.h"
//simplemente lle pedina chat gpt un main para comprobar se funcionaba porque se chamaba ao bmp390 da problemas por esa parte(por o menos a ultima vez que probaramos segun recordo)
static bool my_idle(void){ return false; }

static void test_task(void* arg){
    for(int i=0;i<3;i++){
        SPP_OSAL_TaskDelayUntil(50);
        spp_uint32_t p = SPP_OSAL_TaskPriorityGet(NULL);
        SppTaskState  s = SPP_OSAL_TaskGetState(NULL);
        printf("[TEST_Task] iter=%d prio=%d state=%d\n", i, (int)p, (int)s);
    }
    SPP_OSAL_TaskYield();
    SPP_OSAL_TaskDelay(20);
}

void app_main(void){
    void* h = NULL;
    SppRetVal_t r;

    r = SPP_OSAL_IdleHookRegister(my_idle);
    printf("[MAIN] IdleHookRegister ret=%d\n", (int)r);

    r = SPP_OSAL_TaskCreate(test_task, "TEST_Task", SPP_OSAL_STACK_BYTES, NULL,
                            SPP_OSAL_PRIORITY_NORMAL, &h);
    printf("[MAIN] Create ret=%d handle=%p\n", (int)r, (void*)h);
    if(r!=SPP_OK || !h) return;

    spp_uint32_t pr = SPP_OSAL_TaskPriorityGet(h);
    SppTaskState  st = SPP_OSAL_TaskGetState(h);
    printf("[MAIN] Get prio=%d state=%d\n", (int)pr, (int)st);

    r = SPP_OSAL_TaskPrioritySet(h, SPP_OSAL_PRIORITY_HIGH);
    printf("[MAIN] PrioritySet ret=%d\n", (int)r);
    pr = SPP_OSAL_TaskPriorityGet(h);
    printf("[MAIN] New prio=%d\n", (int)pr);

    r = SPP_OSAL_TaskYield();
    printf("[MAIN] Yield ret=%d\n", (int)r);

    r = SPP_OSAL_SuspendAll();
    printf("[MAIN] SuspendAll ret=%d\n", (int)r);
    r = SPP_OSAL_ResumeAll();
    printf("[MAIN] ResumeAll ret=%d\n", (int)r);

    r = SPP_OSAL_TaskSuspend(h);
    printf("[MAIN] TaskSuspend ret=%d\n", (int)r);
    SPP_OSAL_TaskDelay(100);
    r = SPP_OSAL_TaskResume(h);
    printf("[MAIN] TaskResume ret=%d\n", (int)r);

    SPP_OSAL_TaskDelay(50);

    pr = SPP_OSAL_TaskPriorityGet(h);
    st = SPP_OSAL_TaskGetState(h);
    printf("[MAIN] Before delete prio=%d state=%d\n", (int)pr, (int)st);

    r = SPP_OSAL_TaskDelete(h);
    printf("[MAIN] TaskDelete ret=%d\n", (int)r);
    h = NULL;
}

/*
FLUJO ESPERADO DE INICIALIZACIÓN SPI (BMP → ICM) CON CASOS DE FALLO

1) Bus Init
   main → SPP_HAL_SPI_BusInit()
   - Interno: llama a spi_bus_initialize().
   - Si ESP_OK → SPP_OK. Si falla → SPP_ERROR y main retorna (no seguimos sin bus).

2) Bucle de alta de dispositivos (máx. MAX_DEVICES iteraciones)

   Iteración 1:
   main → p_dev = SPP_HAL_SPI_GetHandler()
   - Interno: recorre device_state[0..n_devices-1] y devuelve &device_ids[i] del PRIMER SLOT con SLOT_EMPTY.
   - Estado inicial típico: [EMPTY, EMPTY] → devuelve &device_ids[0] (BMP).
   - Si no hay slots vacíos → devuelve NULL (main romperá el bucle).

   main → ret = SPP_HAL_DeviceInit(p_dev)
   - Interno: valida p_dev != NULL y que apunte a UNO DE NUESTROS SLOTS (&device_ids[j]) → obtiene flag=j.
   - Comprueba estado paralelo:
       * Si device_state[flag] == SLOT_READY → retorna SPP_OK (idempotencia).
       * Si device_state[flag] == SLOT_EMPTY → compone perfil interno (según flag: BMP o ICM).
   - Llama a spi_bus_add_device(USED_HOST, &devcfg, (spi_device_handle_t*)p_dev)
       * Si ESP_OK → el IDF rellena *p_dev con el handle opaco; MARCA device_state[flag] = SLOT_READY; retorna SPP_OK.
       * Si error → NO marca READY; retorna SPP_ERROR y main rompe el bucle (fallo duro).

   Iteración 2 (camino normal):
   main → p_dev = SPP_HAL_SPI_GetHandler()
   - Interno: ahora device_state típico es [READY, EMPTY] → devuelve &device_ids[1] (ICM).
   main → SPP_HAL_DeviceInit(p_dev)
   - Interno: como SLOT_EMPTY en flag=1 → añade device ICM; si éxito → estado [READY, READY].

   Iteración 3:
   main → p_dev = SPP_HAL_SPI_GetHandler()
   - Interno: ya no hay EMPTY → devuelve NULL; main rompe el bucle y continúa.

CASOS ESPECIALES CONTROLADOS (sin romper la ejecución):

A) Repetición de BMP (llega dos veces seguidas BMP)
   - Si ya se inicializó BMP antes:
       * DeviceInit(p_bmp) detecta device_state[0] == SLOT_READY → SPP_OK.
       * En la siguiente GetHandler(), se ofrecerá ICM (primer EMPTY), por lo que no bloquea al ICM.

B) Intento de tercer dispositivo
   - GetHandler() recorre y ve todos los slots READY → devuelve NULL.
   - main rompe el bucle de forma limpia (no hay hueco, no se intenta añadir un 3.º).

C) Fallo al añadir BMP (por hardware/IDF)
   - Iteración 1: DeviceInit(p_bmp) → spi_bus_add_device() falla → SPP_ERROR → main rompe (no seguimos).

D) Puntero ajeno a nuestros slots
   - DeviceInit(p) valida que p == &device_ids[j] para algún j∈[0..n_devices).
   - Si no coincide con ninguno → SPP_ERROR (estado interno intacto).

E) Idempotencia general
   - Si por cualquier razón DeviceInit() se llama sobre un slot ya READY → SPP_OK.
   - Esto garantiza que “llegadas repetidas” (BMP o ICM) no rompen ni alteran el estado.
*/
