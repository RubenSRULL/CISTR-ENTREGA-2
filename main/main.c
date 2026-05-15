//.........................................................................................................//
// Autor: Rubén Sahuquillo Redondo
// Descripción: Este proyecto implementa un sistema de comunicación CAN utilizando el microcontrolador ESP32
//.........................................................................................................//


// ================= LIBRERÍAS ================= //
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"


// ================= CONFIGURACIÓN ================= //
#define TX_GPIO     GPIO_NUM_4
#define RX_GPIO     GPIO_NUM_5
#define BITRATE     1000000
#define MSG_ID      0x100
#define POT         ADC_CHANNEL_6 
#define PULSADOR    GPIO_NUM_16


// ================= TAG LOG ================= //
static const char *TAG = "CAN";


// ========== PARAMETROS TAREAS ========== //
static uint32_t usStackDepth = 2048;
TaskHandle_t pvCreatedTask = NULL;
TaskHandle_t pvParameters = NULL;


// ================= GLOBAL VARIABLES ================= //
static adc_oneshot_unit_handle_t adc_handle;
static twai_node_handle_t node_hdl = NULL;
static SemaphoreHandle_t can_mutex;
static SemaphoreHandle_t btn_sem;
static QueueHandle_t rx_queue;


// ======== ISR PULSADOR ========== //
// Cuando se detecta una interrupción por el botón, se libera el semáforo btn_sem para que la tarea btn_task pueda procesar la acción del botón.
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    xSemaphoreGiveFromISR(btn_sem, NULL);
}


// ================= CALLBACKS CAN ================= //

// Callback de transmisión: Se llama cuando se completa una transmisión.
static IRAM_ATTR bool twai_tx_cb(twai_node_handle_t handle,const twai_tx_done_event_data_t *edata,void *user_ctx)
{
    if (edata->is_tx_success)
    {
        ;
        //ESP_EARLY_LOGI(TAG, "TX OK ID: 0x%X", edata->done_tx_frame->header.id);
    }
    else
    {
        ;
        //ESP_EARLY_LOGW(TAG, "TX NO OK");
    }
    return false;
}

// Callback de error: Se llama cuando ocurre un error en el bus CAN. Se registra el error en el log para su diagnóstico.
static IRAM_ATTR bool twai_error_cb(twai_node_handle_t handle,const twai_error_event_data_t *edata,void *user_ctx)
{
    ESP_EARLY_LOGW(TAG, "Error TWAI: 0x%x", edata->err_flags.val);
    return false;
}

// Callback de recepción: Se llama cuando se recibe un mensaje CAN.
static bool twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    uint8_t recv_buff[8];
    twai_frame_t rx_frame = {
        .buffer = recv_buff,
        .buffer_len = sizeof(recv_buff),
    };
    if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame)) {
        xQueueSendFromISR(rx_queue, &rx_frame, NULL);
        ESP_LOGE(TAG, "Recibido respuesta");
    }
    return false;
}



// =========== TAREA PARA ENVIAR POTENCIOMETRO ===========
// Esta tarea lee el valor del potenciómetro, calcula la media de 10 lecturas y envía el resultado a través del bus CAN cada 100 ms.
// Para evitar conflictos en el acceso al bus CAN, se utiliza un mutex (can_mutex) para asegurar que solo una tarea pueda transmitir en un momento dado.
void read_pot(void *pvParameters) {
    // Variables para almacenar el valor del potenciómetro, la suma de las lecturas y la media calculada.
    int valor_pot = 0;
    int suma_pot = 0;
    int media_pot = 0;

    // Bucle infinito para leer el potenciómetro, calcular la media y enviar el resultado a través del bus CAN.
    while (1) {
        suma_pot = 0;
        
        // Bucle para calcular la media de 10 lecturas del potencómetro
        for (int i = 0; i < 10; i++) {
            int temp_val = 0;
            adc_oneshot_read(adc_handle, POT, &temp_val);
            suma_pot += temp_val;
            vTaskDelay(pdMS_TO_TICKS(10)); 
        }
        media_pot = suma_pot / 10;

        // Crear el mensaje CAN con el valor de la media del potenciómetro
        twai_frame_t tx_msg = {
            .header.id = MSG_ID,
            .header.ide = false,
            .buffer = (uint8_t *)&media_pot,
            .buffer_len = sizeof(media_pot),
        };

        // Si se obtiene el mutex para acceder al bus CAN, se transmite el mensaje y se espera a que se complete la transmisión antes de liberar el mutex.
        if (xSemaphoreTake(can_mutex, portMAX_DELAY) == pdTRUE) {
            twai_node_transmit(node_hdl, &tx_msg, 0);
            twai_node_transmit_wait_all_done(node_hdl, pdMS_TO_TICKS(100));
            xSemaphoreGive(can_mutex);
            ESP_LOGI(TAG, "Enviado POT: %d", media_pot);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
}


// =========== TAREA PARA EL BOTÓN (POR INTERRUPCIÓN) ===========
// Esta tarea se activa cuando se presiona el botón. Al detectar la interrupción, se libera el semáforo btn_sem, lo que permite que esta tarea procese
// la acción del botón.
void btn_task(void *pvParameters) {
    // Valor fijo a enviar cuando se presiona el botón. En este caso, se envía el carácter '?'
    char msg = '?';

    // Bucle infinito para esperar a que se presione el botón.
    while (1) {
        // Cuando se detecta la interrupción y se libera el semáforo btn_sem, se envía un mensaje a través del bus CAN.
        if (xSemaphoreTake(btn_sem, portMAX_DELAY) == pdTRUE) {
            ESP_LOGW(TAG, "¡Botón Presionado!");
            // Crear el mensaje CAN con el valor fijo a enviar
            twai_frame_t tx_msg = {
                .header.id = MSG_ID,
                .header.ide = false,
                .buffer = (uint8_t *)&msg,
                .buffer_len = sizeof(msg),
            };
            // Si se obtiene el mutex para acceder al bus CAN, se transmite el mensaje y se espera a que se complete la transmisión antes de liberar el mutex.
            if (xSemaphoreTake(can_mutex, portMAX_DELAY) == pdTRUE) {
                twai_node_transmit(node_hdl, &tx_msg, 0);
                twai_node_transmit_wait_all_done(node_hdl, pdMS_TO_TICKS(100));
                xSemaphoreGive(can_mutex);
            }
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
}


// ======== TAREA PARA MOSTRAR ESTADO RECIBIDO ======== //
// Esta tarea se encarga de recibir los mensajes del bus CAN a través de la cola rx_queue y mostrar su contenido. Cada vez que se recibe un mensaje,
// se imprime su ID, DLC y los datos en formato hexadecimal.
void can_receive_task(void *pvParameters) {
    twai_frame_t rx_frame;

    while (1) {
        if (xQueueReceive(rx_queue, &rx_frame, portMAX_DELAY)) {

            printf("ID: 0x%lX DLC: %d Data: ", rx_frame.header.id, rx_frame.header.dlc);

            for (int i = 0; i < rx_frame.header.dlc; i++) {
                ESP_LOGI(TAG, "%02X ", rx_frame.buffer[i]);
            }

            printf("\n");
        }
    }
}


// ============== CONFIGURAR PERIFÉRICOS ============= //
// Esta función configura los periféricos necesarios para el proyecto, incluyendo el botón (PULSADOR) y el ADC para el potenciómetro (POT).
static void configure_peripherals(void)
{
    // Configuración del botón (PULSADOR)
    gpio_reset_pin(PULSADOR);
    gpio_set_direction(PULSADOR, GPIO_MODE_INPUT);
    gpio_pullup_en(PULSADOR);
    gpio_pulldown_dis(PULSADOR);
    // Configuración de la interrupción para el botón (PULSADOR)
    gpio_set_intr_type(PULSADOR, GPIO_INTR_NEGEDGE);
    // Instalar el servicio de manejo de interrupciones para los GPIO
    gpio_install_isr_service(0);
    // Registrar la función de manejo de interrupciones para el botón (PULSADOR)
    gpio_isr_handler_add(PULSADOR, gpio_isr_handler, (void*) PULSADOR);

    // Configuración del ADC para el potenciómetro (POT)
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    // Crear el controlador ADC para el potenciómetro (POT)
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc_handle));
    // Configuración del canal ADC para el potenciómetro (POT)
    adc_oneshot_chan_cfg_t config1 = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    // Configurar el canal ADC para el potenciómetro (POT)
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, POT, &config1));

}

// ============== CONFIGURAR CAN ============= //
// Esta función configura el bus CAN utilizando el controlador TWAI on-chip. Se establece la configuración de los pines de transmisión y recepción,
// la velocidad de transmisión, y se registran los callbacks para eventos de transmisión, recepción y errores. Finalmente, se habilita el nodo CAN
// para comenzar a operar.
static void init_can(void)
{
    // Configuración del nodo CAN
    twai_onchip_node_config_t node_config = {
        .io_cfg.tx = TX_GPIO,
        .io_cfg.rx = RX_GPIO,
        .bit_timing.bitrate = BITRATE,
        .tx_queue_depth = 5,
    };

    // Configuración de callbacks para eventos de TWAI
    twai_event_callbacks_t user_cbs = {
        .on_rx_done = twai_rx_cb,
        .on_tx_done = twai_tx_cb,
        .on_error = twai_error_cb
    };
    // Crear nodo CAN on-chip
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &node_hdl));
    // Registrar callbacks para eventos de TWAI
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(node_hdl, &user_cbs, NULL));
    // Habilitar el nodo CAN
    ESP_ERROR_CHECK(twai_node_enable(node_hdl));
}


// ================= FUNCIÓN PRINCIPAL ================= //
// En la función principal se crean los semáforos y la cola necesarios para la comunicación entre tareas, se configuran los periféricos y el bus CAN
// y se crean las tareas para leer el potenciómetro, procesar el botón y recibir mensajes del bus CAN.
void app_main(void)
{
    // Crear semáforos y cola
    can_mutex = xSemaphoreCreateMutex();
    btn_sem   = xSemaphoreCreateBinary();
    rx_queue  = xQueueCreate(10, sizeof(twai_frame_t));

    // Configurar periféricos y CAN
    configure_peripherals();
    init_can();

    // Crear tareas
    xTaskCreatePinnedToCore(
        read_pot,
        "Lectura Potenciometro",
        usStackDepth,
        &pvParameters,
        1,
        &pvCreatedTask,
        0
    );

    xTaskCreatePinnedToCore(
        btn_task,
        "Lectura Pulsador",
        usStackDepth,
        &pvParameters,
        2,
        &pvCreatedTask,
        0
    );

    xTaskCreatePinnedToCore(
        can_receive_task,
        "Recepcion CAN",
        usStackDepth,
        &pvParameters,
        1,
        &pvCreatedTask,
        0
    );
}