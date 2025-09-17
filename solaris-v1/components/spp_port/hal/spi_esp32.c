#include "spi.h" 
#include "types.h"
#include "esp_log.h" 
#include "driver/spi_common.h"
#include "driver/spi_master.h"


static const char *TAG = "SPP_HAL_SPI";

retval_t SPP_SPI_Transmit(void* handler, void* data_to_send, void* data_to_recieve, spp_uint8_t length){
    spi_device_handle_t p_handler = (spi_device_handle_t)handler;
    spi_transaction_t trans_desc = {0};
    spp_uint8_t *p_tx_data = (spp_uint8_t*)data_to_send;
    spp_uint8_t *p_rx_data = (spp_uint8_t*)data_to_recieve;
    trans_desc.length = 8 * length;
    trans_desc.tx_buffer = p_tx_data;
    trans_desc.rx_buffer = p_rx_data;
    esp_err_t trans_result = spi_device_transmit(p_handler, &trans_desc);
    if (trans_result != ESP_OK){
        return SPP_ERROR;
    }
    return SPP_OK;
}

