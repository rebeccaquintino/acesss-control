/**
 * @file main.c
 * @brief Sistema de leitor de SmartCard com comunicação Wi-Fi e LCD
 * @author Rebecca Quintino Do Ó
 */
 
#include <stdio.h> ///< Inclui funções padrão de entrada e saída
#include <string.h> ///< Inclui funções para manipulação de strings
#include <stdlib.h> ///< Inclui funções de uso geral, como alocação de memória e conversão de tipos
#include <time.h> ///< Para manipulação de tempo (timestamp)
#include <sys/time.h> ///< Para a função gettimeofday, usada para obter o tempo atual com alta precisão

#include "driver/i2c_master.h" ///< Driver para comunicação I2C mestre
#include "driver/gpio.h" ///< Driver para controle de pinos GPIO
#include "esp_wifi.h" ///< API do ESP-IDF para controle de Wi-Fi
#include "esp_event.h" ///< API do ESP-IDF para gerenciamento de eventos
#include "esp_log.h" ///< API do ESP-IDF para logging (mensagens de depuração)
#include "esp_sntp.h" ///< API do ESP-IDF para protocolo SNTP (sincronização de tempo de rede)
#include "nvs_flash.h" ///< API do ESP-IDF para NVS (Non-Volatile Storage), usada para armazenar dados de forma persistente
#include "esp_http_client.h" ///< API do ESP-IDF para cliente HTTP
#include "freertos/FreeRTOS.h" ///< Sistema operacional em tempo real FreeRTOS
#include "freertos/task.h" ///< Funções para gerenciamento de tarefas no FreeRTOS
#include "freertos/queue.h" ///< Funções para gerenciamento de filas no FreeRTOS (comunicação entre tarefas)
#include "freertos/event_groups.h" ///< Funções para gerenciamento de grupos de eventos no FreeRTOS (sinalização entre tarefas)

// Inclui o cabeçalho do LCD
#include "display_lcd.h" ///< Cabeçalho para funções de controle do display LCD

// Definições do hardware
#define BFC_PIN 2 ///< Pino do botão de fim de curso
#define I2C_SCL_PIN 3 ///< Pino SCL (Serial Clock) do barramento I2C
#define I2C_SDA_PIN 4 ///< Pino SDA (Serial Data) do barramento I2C
#define EEPROM_ADDRESS 0x50 ///< Endereço I2C do dispositivo EEPROM (SmartCard)
#define DEBOUNCE_DELAY_MS 100 ///< Tempo de debounce do botão em milissegundos, para evitar múltiplas leituras por um único clique

#define LED_VERMELHO_PIN 9 ///< Pino do LED vermelho
#define LED_VERDE_PIN 0 ///< Pino do LED verde

// Definições do servidor e credenciais para o Projeto 2
#define SERVER_URL "http://eel-7030-server.azurewebsites.net/" ///< URL do servidor para comunicação HTTP
#define AUTH_TOKEN "Q1w2E3r4T5y6U7i8" ///< Token de autenticação para as requisições ao servidor
#define ROOM_NUMBER "101" ///< Número do quarto ao qual o dispositivo ESP32-C3 está associado

// Tags de Log
#define TAG_HTTP "HTTP_CLIENT" ///< Tag para logs relacionados ao cliente HTTP
#define TAG_SNTP "SNTP" ///< Tag para logs relacionados ao SNTP
#define TAG_WIFI "Wi-Fi STA" ///< Tag para logs relacionados ao Wi-Fi (modo estação)
#define TAG_MAIN "SmartCard Reader" ///< Tag para logs principais do sistema leitor de SmartCard
#define TAG_LCD "LCD" ///< Tag para logs do display LCD
#define TAG_LED_VERDE "LED_VERDE" ///< Tag para logs do LED verde

// Credenciais Wi-Fi (ajuste conforme sua rede)
#define WIFI_SSID "Wokwi-GUEST" ///< SSID (nome) da rede Wi-Fi a ser conectada
#define WIFI_PASS "" ///< Senha da rede Wi-Fi (vazia para redes abertas)

// Grupo de Eventos para status Wi-Fi
#define WIFI_CONNECTED_BIT BIT0 ///< Bit que será setado no grupo de eventos quando o Wi-Fi estiver conectado
static EventGroupHandle_t wifi_event_group; ///< Handle para o grupo de eventos Wi-Fi

// Variáveis globais para I2C
i2c_master_bus_handle_t bus_handle; ///< Handle para o barramento I2C mestre
i2c_master_dev_handle_t dev_handle; ///< Handle para o dispositivo I2C (EEPROM do SmartCard)

// Variáveis para resposta HTTP
static char *http_response_buffer = NULL; ///< Buffer dinâmico para armazenar a resposta HTTP
static int http_response_len = 0; ///< Tamanho atual dos dados na resposta HTTP

// Fila para eventos do GPIO (BFC)
static QueueHandle_t gpio_evt_queue = NULL; ///< Fila usada para receber eventos de interrupção do GPIO (botão de fim de curso)

// Fila para comunicar estados do sistema para a tarefa de controle do LED Verde e LCD
typedef enum {
    STATE_CONNECTING_WIFI, ///< Estado: Conectando ao Wi-Fi
    STATE_WAITING_CARD, ///< Estado: Aguardando inserção do cartão
    STATE_VERIFYING_CARD, ///< Estado: Verificando o cartão
    STATE_ACCESS_GRANTED, ///< Estado: Acesso liberado
    STATE_ACCESS_DENIED, ///< Estado: Acesso negado
} system_state_t;

static QueueHandle_t system_state_queue = NULL; ///< Fila para enviar e receber estados do sistema entre tarefas

// Variável para sinalizar que o tempo foi sincronizado
static volatile bool time_synced = false; ///< Flag booleana que indica se o tempo foi sincronizado via SNTP

// --- Protótipos de Função ---
/**
 * @brief Configura o barramento I2C mestre
 * @param ptr_bus_handle Ponteiro para o handle do barramento I2C
 */
void configura_barramento_mestre(i2c_master_bus_handle_t* ptr_bus_handle);

/**
 * @brief Configura o dispositivo SmartCard no barramento I2C
 * @param bus_handle Handle do barramento I2C
 * @param ptr_dev_handle Ponteiro para o handle do dispositivo
 */
void configura_dispositivo_smartcard(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t* ptr_dev_handle);

/**
 * @brief Lê dados do cartão SmartCard
 * @param dev_handle Handle do dispositivo I2C
 * @param endereco_inicial Endereço inicial de leitura na EEPROM do cartão
 * @param buffer Buffer para armazenar os dados lidos
 * @param tamanho Tamanho dos dados a serem lidos
 */
void ler_dados_cartao(i2c_master_dev_handle_t dev_handle, uint8_t endereco_inicial, uint8_t* buffer, size_t tamanho);

/**
 * @brief Escreve timestamp no cartão SmartCard
 * @param dev_handle Handle do dispositivo I2C
 * @param timestamp Timestamp a ser escrito
 */
void escrever_timestamp_cartao(i2c_master_dev_handle_t dev_handle, uint64_t timestamp);

/**
 * @brief Handler de interrupção GPIO
 * @param arg Argumento passado para o handler (geralmente o número do pino que gerou a interrupção)
 */
void gpio_isr_handler(void* arg);

/**
 * @brief Inicializa os GPIOs
 */
void init_gpio();

/**
 * @brief Tarefa para processamento do cartão
 * @param arg Argumento da tarefa (não utilizado)
 */
void cartao_task(void* arg);

/**
 * @brief Tarefa para sincronização SNTP
 * @param pvParameter Argumento da tarefa (não utilizado)
 */
void sntp_sync_task(void *pvParameter);

/**
 * @brief Tarefa para gerenciamento dos LEDs e display LCD
 * @param pvParameter Argumento da tarefa (não utilizado)
 */
void leds_status_task(void *pvParameter);

/**
 * @brief Inicializa o NVS (Non-Volatile Storage)
 */
void nvs_init();

/**
 * @brief Inicializa o loop de eventos
 */
void event_loop_init();

/**
 * @brief Handler de eventos Wi-Fi
 * @param arg Argumento do handler
 * @param event_base Base do evento (e.g., WIFI_EVENT, IP_EVENT)
 * @param event_id ID do evento específico (e.g., WIFI_EVENT_STA_START, IP_EVENT_STA_GOT_IP)
 * @param event_data Dados do evento
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

/**
 * @brief Inicializa a conexão Wi-Fi
 */
void wifi_init();

/**
 * @brief Handler de eventos HTTP
 * @param evt Estrutura com dados do evento HTTP
 * @return Código de erro ESP
 */
esp_err_t http_post_request_event_handler(esp_http_client_event_t *evt);

/**
 * @brief Callback de sincronização de tempo SNTP
 * @param tv Estrutura com dados de tempo (struct timeval)
 */
void sntp_callback_sync_time(struct timeval *tv);

/**
 * @brief Atualiza o display LCD conforme o estado do sistema
 * @param state Estado do sistema a ser exibido
 */
void update_lcd_display(system_state_t state);

// --- Função Principal ---
/**
 * @brief Função principal do sistema, ponto de entrada do programa
 */
void app_main(void) {
    wifi_event_group = xEventGroupCreate(); ///< Criação do Grupo de Eventos Wi-Fi para gerenciar o estado da conexão
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t)); ///< Cria fila para eventos do GPIO (BFC), com capacidade para 10 itens
    system_state_queue = xQueueCreate(5, sizeof(system_state_t)); ///< Cria fila para estados do sistema, com capacidade para 5 estados

    system_state_t initial_state = STATE_CONNECTING_WIFI; ///< Define o estado inicial como "Conectando ao Wi-Fi"
    xQueueSend(system_state_queue, &initial_state, portMAX_DELAY); ///< Envia o estado inicial para a fila de estados do sistema (bloqueia indefinidamente se a fila estiver cheia)

    nvs_init(); ///< Inicializa o NVS (Non-Volatile Storage) para armazenamento persistente
    event_loop_init(); ///< Inicializa o loop de eventos do ESP-IDF para eventos de rede
    wifi_init(); ///< Inicializa e tenta conectar ao Wi-Fi

    configura_barramento_mestre(&bus_handle); ///< Inicializa o barramento I2C para comunicação com SmartCard e LCD
    configura_dispositivo_smartcard(bus_handle, &dev_handle); ///< Configura o dispositivo SmartCard no barramento I2C

    ///< Configura o display LCD via I2C
    esp_err_t lcd_err = lcd_config(bus_handle); ///< Tenta configurar o display LCD
    if (lcd_err == ESP_OK) {
        //ESP_LOGE(TAG_LCD, "Display LCD configurado com sucesso."); ///< Log de sucesso na configuração do LCD (comentado)
    } else {
        //ESP_LOGE(TAG_LCD, "Falha ao configurar Display LCD: %s", esp_err_to_name(lcd_err)); ///< Log de falha na configuração do LCD (comentado)
    }

    init_gpio(); ///< Inicializa os pinos GPIO e configura a interrupção para o botão de fim de curso (BFC)

    ///< Cria tarefas FreeRTOS
    xTaskCreate(cartao_task, "cartao_task", 4096, NULL, 10, NULL); ///< Cria a tarefa para processamento do cartão (stack de 4KB, prioridade 10)
    xTaskCreate(sntp_sync_task, "sntp_sync_task", 4096, NULL, 5, NULL); ///< Cria a tarefa para sincronização SNTP (stack de 4KB, prioridade 5)
    xTaskCreate(leds_status_task, "leds_status_task", 2048, NULL, 5, NULL); ///< Cria a tarefa para gerenciamento dos LEDs e LCD (stack de 2KB, prioridade 5)
    //ESP_LOGE(TAG_MAIN, "Sistema pronto. Aguardando inserção de cartão..."); ///< Log inicial do sistema (comentado)
}

/**
 * @brief Configura o barramento I2C mestre
 * @param ptr_bus_handle Ponteiro para o handle do barramento I2C
 */
void configura_barramento_mestre(i2c_master_bus_handle_t* ptr_bus_handle) {
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT, ///< Fonte do clock do I2C (padrão)
        .i2c_port = I2C_NUM_0, ///< Porta I2C a ser usada (I2C_NUM_0)
        .scl_io_num = I2C_SCL_PIN, ///< Pino GPIO para SCL
        .sda_io_num = I2C_SDA_PIN, ///< Pino GPIO para SDA
        .glitch_ignore_cnt = 7, ///< Número de pulsos de glitch para ignorar
        .flags.enable_internal_pullup = true ///< Habilita resistores de pull-up internos
    };
    i2c_new_master_bus(&i2c_mst_config, ptr_bus_handle); ///< Cria um novo barramento I2C mestre
}

/**
 * @brief Configura o dispositivo SmartCard no barramento I2C
 * @param bus_handle Handle do barramento I2C
 * @param ptr_dev_handle Ponteiro para o handle do dispositivo
 */
void configura_dispositivo_smartcard(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t* ptr_dev_handle) {
    i2c_device_config_t cfg_dispositivo = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, ///< Endereço do dispositivo tem 7 bits
        .device_address = EEPROM_ADDRESS, ///< Endereço I2C da EEPROM do SmartCard
        .scl_speed_hz = 100000, ///< Velocidade do clock SCL em Hz (100 KHz)
    };
    i2c_master_bus_add_device(bus_handle, &cfg_dispositivo, ptr_dev_handle); ///< Adiciona o dispositivo SmartCard ao barramento I2C
}

/**
 * @brief Lê dados do cartão SmartCard
 * @param dev_handle Handle do dispositivo I2C
 * @param endereco_inicial Endereço inicial de leitura
 * @param buffer Buffer para armazenar os dados lidos
 * @param tamanho Tamanho dos dados a serem lidos
 */
void ler_dados_cartao(i2c_master_dev_handle_t dev_handle, uint8_t endereco_inicial, uint8_t* buffer, size_t tamanho) {
    esp_err_t err = i2c_master_transmit_receive(dev_handle, &endereco_inicial, 1, buffer, tamanho, pdMS_TO_TICKS(100)); ///< Realiza a operação de transmissão (endereço) e recepção (dados) via I2C
}

/**
 * @brief Escreve timestamp no cartão SmartCard
 * @param dev_handle Handle do dispositivo I2C
 * @param timestamp Timestamp a ser escrito
 */
void escrever_timestamp_cartao(i2c_master_dev_handle_t dev_handle, uint64_t timestamp) {
    uint8_t write_buffer[9]; ///< Buffer para escrita, 1 byte para endereço + 8 bytes para timestamp
    write_buffer[0] = 0x18; ///< Endereço inicial na EEPROM para escrita do timestamp
    uint8_t read_buffer_eeprom[8]; ///< Buffer para leitura de verificação após a escrita

    memcpy(&write_buffer[1], &timestamp, sizeof(uint64_t)); ///< Copia o timestamp para o buffer de escrita, após o byte de endereço

    esp_err_t err = i2c_master_transmit(dev_handle, write_buffer, sizeof(write_buffer), pdMS_TO_TICKS(100)); ///< Realiza a transmissão dos dados (endereço + timestamp) via I2C

    if (err != ESP_OK) {
        //ESP_LOGE(TAG_MAIN, "Erro ao escrever timestamp no cartão: %s", esp_err_to_name(err)); ///< Log de erro se a escrita falhar (comentado)
    } else {
        ler_dados_cartao(dev_handle, 0x18, read_buffer_eeprom, sizeof(read_buffer_eeprom)); ///< Lê os dados recém-escritos para verificação

        uint64_t timestamp_lido_eeprom;
        memcpy(&timestamp_lido_eeprom, read_buffer_eeprom, sizeof(uint64_t)); ///< Copia os dados lidos de volta para uma variável uint64_t

        time_t time_lido = (time_t)timestamp_lido_eeprom; ///< Converte o timestamp para time_t
        struct tm timeinfo_lida;
        localtime_r(&time_lido, &timeinfo_lida); ///< Converte o timestamp para uma estrutura de tempo legível
        char time_str_lida[64];
        strftime(time_str_lida, sizeof(time_str_lida), "%Y-%m-%d %H:%M:%S", &timeinfo_lida); ///< Formata o tempo para uma string legível
    }
    vTaskDelay(pdMS_TO_TICKS(5)); ///< Pequeno atraso para estabilização da EEPROM
}

/**
 * @brief Inicializa os GPIOs
 */
void init_gpio() {
    // Configura pino do BFC
    gpio_config_t io_conf_bfc = {
        .pin_bit_mask = (1ULL << BFC_PIN), ///< Máscara de bits para o pino do BFC
        .mode = GPIO_MODE_INPUT, ///< Configura o pino como entrada
        .pull_up_en = GPIO_PULLUP_DISABLE, ///< Desabilita resistor de pull-up
        .pull_down_en = GPIO_PULLDOWN_ENABLE, ///< Habilita resistor de pull-down (garante estado baixo quando não ativado)
        .intr_type = GPIO_INTR_POSEDGE ///< Configura interrupção para borda de subida (quando o botão é pressionado e o pino vai de baixo para alto)
    };
    gpio_config(&io_conf_bfc); ///< Aplica a configuração ao pino do BFC

    // Configura pino do LED Vermelho
    gpio_config_t io_conf_led_vermelho = {
        .pin_bit_mask = (1ULL << LED_VERMELHO_PIN), ///< Máscara de bits para o pino do LED vermelho
        .mode = GPIO_MODE_OUTPUT, ///< Configura o pino como saída
        .pull_up_en = GPIO_PULLUP_DISABLE, ///< Desabilita pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE, ///< Desabilita pull-down
        .intr_type = GPIO_INTR_DISABLE ///< Desabilita interrupções
    };
    gpio_config(&io_conf_led_vermelho); ///< Aplica a configuração ao pino do LED vermelho
    gpio_set_level(LED_VERMELHO_PIN, 0); ///< Desliga o LED vermelho inicialmente

    // Configura pino do LED Verde
    gpio_config_t io_conf_led_verde = {
        .pin_bit_mask = (1ULL << LED_VERDE_PIN), ///< Máscara de bits para o pino do LED verde
        .mode = GPIO_MODE_OUTPUT, ///< Configura o pino como saída
        .pull_up_en = GPIO_PULLUP_DISABLE, ///< Desabilita pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE, ///< Desabilita pull-down
        .intr_type = GPIO_INTR_DISABLE ///< Desabilita interrupções
    };
    gpio_config(&io_conf_led_verde); ///< Aplica a configuração ao pino do LED verde
    gpio_set_level(LED_VERDE_PIN, 0); ///< Desliga o LED verde inicialmente

    gpio_install_isr_service(0); ///< Instala o serviço de interrupção GPIO
    gpio_isr_handler_add(BFC_PIN, gpio_isr_handler, (void*) BFC_PIN); ///< Adiciona o handler de interrupção para o pino do BFC
}

/**
 * @brief Handler de interrupção GPIO
 * @param arg Argumento passado para o handler
 */
void gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg; ///< Obtém o número do pino GPIO que gerou a interrupção
    gpio_intr_disable(gpio_num); ///< Desabilita temporariamente a interrupção para o pino para evitar múltiplos eventos
    BaseType_t xHigherPriorityTaskWoken = pdFALSE; ///< Variável para verificar se uma tarefa de maior prioridade foi "acordada"
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, &xHigherPriorityTaskWoken); ///< Envia o número do pino para a fila de eventos GPIO a partir da ISR
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR(); ///< Força uma troca de contexto se uma tarefa de maior prioridade foi acordada
    }
}

/**
 * @brief Tarefa para processamento do cartão
 * @param arg Argumento da tarefa (não utilizado)
 */
void cartao_task(void* arg) {
    uint32_t io_num; ///< Variável para armazenar o número do pino GPIO recebido da fila
    uint8_t id_parte1[8]; ///< Buffer para a primeira parte do ID do cartão
    uint8_t id_parte2[8]; ///< Buffer para a segunda parte do ID do cartão
    char card_id_str[17]; ///< String para armazenar o ID completo do cartão (16 caracteres + null terminator)

    while(1) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) { ///< Aguarda indefinidamente por um evento na fila do GPIO
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS)); ///< Atraso para debounce do botão

            if(gpio_get_level(io_num) == 1) { ///< Verifica se o botão ainda está ativado após o debounce
                system_state_t state_verifying = STATE_VERIFYING_CARD; ///< Define o estado como "Verificando Cartão"
                xQueueSend(system_state_queue, &state_verifying, 0); ///< Envia o estado para a fila do sistema (não bloqueante)

                ler_dados_cartao(dev_handle, 0x00, id_parte1, sizeof(id_parte1)); ///< Lê a primeira parte do ID do cartão (endereço 0x00)
                ler_dados_cartao(dev_handle, 0x08, id_parte2, sizeof(id_parte2)); ///< Lê a segunda parte do ID do cartão (endereço 0x08)

                memcpy(card_id_str, id_parte1, 8); ///< Copia a primeira parte do ID para a string
                memcpy(card_id_str + 8, id_parte2, 8); ///< Copia a segunda parte do ID para a string
                card_id_str[16] = '\0'; ///< Adiciona o terminador nulo à string

                if (time_synced) { ///< Verifica se o tempo foi sincronizado via SNTP
                    struct timeval tv_now;
                    gettimeofday(&tv_now, NULL); ///< Obtém o tempo atual do sistema
                    uint64_t timestamp_64bit = (uint64_t)tv_now.tv_sec; ///< Converte o tempo para um timestamp de 64 bits (segundos)
                    escrever_timestamp_cartao(dev_handle, timestamp_64bit); ///< Escreve o timestamp no cartão SmartCard

                    time_t current_time = tv_now.tv_sec;
                    struct tm timeinfo;
                    localtime_r(&current_time, &timeinfo); ///< Converte o timestamp para uma estrutura de tempo legível
                    char time_str[64];
                    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &timeinfo); ///< Formata o tempo para uma string legível
                }

                xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE,
                                    pdFALSE, portMAX_DELAY); ///< Aguarda indefinidamente até que o Wi-Fi esteja conectado

                char post_data[128];
                snprintf(post_data, sizeof(post_data),
                         "{\"token\": \"%s\", \"quarto\": \"%s\", \"id\": \"%s\"}",
                         AUTH_TOKEN, ROOM_NUMBER, card_id_str); ///< Formata os dados JSON para a requisição POST

                esp_http_client_config_t config = {
                    .url = SERVER_URL, ///< URL do servidor
                    .method = HTTP_METHOD_POST, ///< Método HTTP (POST)
                    .event_handler = http_post_request_event_handler, ///< Handler de eventos HTTP para processar a resposta
                    .timeout_ms = 5000, ///< Tempo limite da requisição em milissegundos
                    .user_agent = "esp-idf/5.x", ///< User-Agent para a requisição
                };

                esp_http_client_handle_t client = esp_http_client_init(&config); ///< Inicializa o cliente HTTP
                esp_http_client_set_header(client, "Content-Type", "application/json"); ///< Define o cabeçalho Content-Type como JSON
                esp_http_client_set_post_field(client, post_data, strlen(post_data)); ///< Define os dados a serem enviados no corpo da requisição POST

                esp_err_t err = esp_http_client_perform(client); ///< Realiza a requisição HTTP
                if (err == ESP_OK) {
                    int status_code = esp_http_client_get_status_code(client); ///< Obtém o código de status HTTP da resposta
                    //ESP_LOGE(TAG_HTTP, "HTTP POST Status = %d", status_code); ///< Log do status HTTP (comentado)

                    if (status_code == 200) { // Status 200 para acesso liberado
                        //ESP_LOGE(TAG_MAIN, "Acesso Liberado!"); ///< Log de acesso liberado (comentado)
                        system_state_t state_granted = STATE_ACCESS_GRANTED; ///< Define o estado como "Acesso Liberado"
                        xQueueSend(system_state_queue, &state_granted, 0); ///< Envia o estado para a fila
                    } else { // Resposta negativa
                        //ESP_LOGE(TAG_MAIN, "Acesso Negado. Status: %d", status_code); ///< Log de acesso negado (comentado)
                        system_state_t state_denied = STATE_ACCESS_DENIED; ///< Define o estado como "Acesso Negado"
                        xQueueSend(system_state_queue, &state_denied, 0); ///< Envia o estado para a fila
                    }
                } else {
                    //ESP_LOGE(TAG_HTTP, "Erro na requisição POST: %s", esp_err_to_name(err)); ///< Log de erro na requisição (comentado)
                    //ESP_LOGE(TAG_MAIN, "Acesso Negado (erro de comunicação)."); ///< Log de acesso negado por erro de comunicação (comentado)
                    system_state_t state_denied = STATE_ACCESS_DENIED; ///< Define o estado como "Acesso Negado"
                    xQueueSend(system_state_queue, &state_denied, 0); ///< Envia o estado para a fila
                }
                esp_http_client_cleanup(client); ///< Limpa os recursos do cliente HTTP
            } else {
                //ESP_LOGE(TAG_MAIN, "Botão de fim de curso desativado antes da leitura. Ignorando."); ///< Log de botão desativado (comentado)
            }

            gpio_intr_enable(io_num); ///< Reabilita a interrupção para o pino do BFC
            system_state_t state_waiting = STATE_WAITING_CARD; ///< Define o estado como "Aguardando Cartão"
            xQueueSend(system_state_queue, &state_waiting, 0); ///< Envia o estado para a fila
            //ESP_LOGE(TAG_MAIN, "Aguardando Cartao..."); ///< Log de aguardando cartão (comentado)
        }
    }
}


/**
 * @brief Realiza requisição HTTP POST para verificar o cartão
 * @param card_id_str ID do cartão a ser verificado
 * @note Esta função parece ser uma duplicação do trecho de código dentro de `cartao_task` e pode ser refatorada.
 */
void verify_card_http_post(char* card_id_str) {
    char post_data[128];
    snprintf(post_data, sizeof(post_data),
             "{\"token\": \"%s\", \"quarto\": \"%s\", \"id\": \"%s\"}",
             AUTH_TOKEN, ROOM_NUMBER, card_id_str); ///< Formata os dados JSON para a requisição POST

    esp_http_client_config_t config = {
        .url = SERVER_URL, ///< URL do servidor
        .method = HTTP_METHOD_POST, ///< Método HTTP (POST)
        .event_handler = http_post_request_event_handler, ///< Handler de eventos HTTP para processar a resposta
        .timeout_ms = 5000, ///< Tempo limite da requisição em milissegundos
        .user_agent = "esp-idf/5.x", ///< User-Agent para a requisição
    };

    esp_http_client_handle_t client = esp_http_client_init(&config); ///< Inicializa o cliente HTTP
    esp_http_client_set_header(client, "Content-Type", "application/json"); ///< Define o cabeçalho Content-Type como JSON
    esp_http_client_set_post_field(client, post_data, strlen(post_data)); ///< Define os dados a serem enviados no corpo da requisição POST

    esp_err_t err = esp_http_client_perform(client); ///< Realiza a requisição HTTP
    if (err == ESP_OK) {
        int status_code = esp_http_client_get_status_code(client); ///< Obtém o código de status HTTP da resposta

        if (status_code == 200) {
            system_state_t state_granted = STATE_ACCESS_GRANTED; ///< Define o estado como "Acesso Liberado"
            xQueueSend(system_state_queue, &state_granted, 0); ///< Envia o estado para a fila
        } else {
            system_state_t state_denied = STATE_ACCESS_DENIED; ///< Define o estado como "Acesso Negado"
            xQueueSend(system_state_queue, &state_denied, 0); ///< Envia o estado para a fila
        }
    } else {
        system_state_t state_denied = STATE_ACCESS_DENIED; ///< Define o estado como "Acesso Negado" (por erro de comunicação)
        xQueueSend(system_state_queue, &state_denied, 0); ///< Envia o estado para a fila
    }
    esp_http_client_cleanup(client); ///< Limpa os recursos do cliente HTTP
}

/**
 * @brief Inicializa o NVS (Non-Volatile Storage)
 */
void nvs_init() {
    esp_err_t ret = nvs_flash_init(); ///< Tenta inicializar o NVS
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) { ///< Se não há páginas livres ou uma nova versão é encontrada
        nvs_flash_erase(); ///< Apaga a flash NVS
        ret = nvs_flash_init(); ///< Tenta inicializar novamente
    }
}

/**
 * @brief Inicializa o loop de eventos
 */
void event_loop_init() {
    esp_netif_init(); ///< Inicializa a camada de interface de rede (netif)
    esp_event_loop_create_default(); ///< Cria o loop de eventos padrão do ESP-IDF
    esp_netif_create_default_wifi_sta(); ///< Cria a interface de rede padrão para o modo Wi-Fi estação (STA)

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL); ///< Registra o handler para todos os eventos Wi-Fi
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL); ///< Registra o handler para o evento de IP obtido no modo STA
}

/**
 * @brief Inicializa a conexão Wi-Fi
 */
void wifi_init() {
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT(); ///< Obtém a configuração padrão de inicialização do Wi-Fi
    esp_wifi_init(&wifi_cfg); ///< Inicializa o subsistema Wi-Fi com as configurações
    esp_wifi_set_mode(WIFI_MODE_STA); ///< Define o modo de operação do Wi-Fi como estação (STA)

    wifi_config_t wifi_config = {
        .sta = {
        .ssid = WIFI_SSID, ///< Define o SSID da rede Wi-Fi
        .password = WIFI_PASS}, ///< Define a senha da rede Wi-Fi
    };

    esp_wifi_set_config(WIFI_IF_STA, &wifi_config); ///< Aplica as configurações da rede Wi-Fi
    esp_wifi_start(); ///< Inicia o Wi-Fi
}

/**
 * @brief Handler de eventos Wi-Fi
 * @param arg Argumento do handler
 * @param event_base Base do evento
 * @param event_id ID do evento
 * @param event_data Dados do evento
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT) { ///< Se o evento é relacionado ao Wi-Fi
        switch (event_id) {
            case WIFI_EVENT_STA_START: { ///< Quando o modo STA é iniciado
                esp_wifi_connect(); ///< Tenta conectar à rede Wi-Fi
                break;
            };

            case WIFI_EVENT_STA_DISCONNECTED: { ///< Quando a estação Wi-Fi é desconectada
                xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT); ///< Limpa o bit de conexão Wi-Fi no grupo de eventos
                esp_wifi_connect(); ///< Tenta reconectar
                system_state_t state = STATE_CONNECTING_WIFI; ///< Define o estado como "Conectando ao Wi-Fi"
                xQueueSend(system_state_queue, &state, 0); ///< Envia o estado para a fila
                break;
            };

            default: {
                break;
            }
        }
    } else if (event_base == IP_EVENT) { ///< Se o evento é relacionado ao IP
        switch (event_id) {
            case IP_EVENT_STA_GOT_IP: { ///< Quando a estação Wi-Fi obtém um endereço IP
                xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT); ///< Seta o bit de conexão Wi-Fi no grupo de eventos
                system_state_t state = STATE_WAITING_CARD; ///< Define o estado como "Aguardando Cartão"
                xQueueSend(system_state_queue, &state, 0); ///< Envia o estado para a fila
                break;
            };

            default:
                break;
        }
    }
}

/**
 * @brief Handler de eventos HTTP
 * @param evt Estrutura com dados do evento HTTP
 * @return Código de erro ESP
 */
esp_err_t http_post_request_event_handler(esp_http_client_event_t *evt) {
    switch (evt->event_id) {
        case HTTP_EVENT_ON_DATA: ///< Evento de recebimento de dados da resposta HTTP
            if (evt->data && evt->data_len > 0) {
                char *new_buf = realloc(http_response_buffer,
                                        http_response_len + evt->data_len + 1); ///< Realoca o buffer para acomodar os novos dados
                if (!new_buf) {
                    free(http_response_buffer); ///< Libera o buffer antigo em caso de falha na realocação
                    http_response_buffer = NULL;
                    http_response_len = 0;
                    return ESP_FAIL; ///< Retorna falha
                }

                http_response_buffer = new_buf;
                memcpy(http_response_buffer + http_response_len, evt->data, evt->data_len); ///< Copia os novos dados para o buffer
                http_response_len += evt->data_len; ///< Atualiza o tamanho da resposta
                http_response_buffer[http_response_len] = '\0'; ///< Adiciona o terminador nulo
            };
            break;

        case HTTP_EVENT_ON_FINISH: ///< Evento de finalização da requisição HTTP
            if (http_response_buffer) {
                free(http_response_buffer); ///< Libera o buffer de resposta
                http_response_buffer = NULL;
                http_response_len = 0;
            };
            break;

        case HTTP_EVENT_DISCONNECTED: ///< Evento de desconexão HTTP
        case HTTP_EVENT_ERROR: ///< Evento de erro HTTP
            if (http_response_buffer) {
                free(http_response_buffer); ///< Libera o buffer em caso de desconexão ou erro
                http_response_buffer = NULL;
                http_response_len = 0;
            }
            break;

        default:
            break;
    }

    return ESP_OK; ///< Retorna sucesso
}

/**
 * @brief Tarefa para sincronização SNTP
 * @param pvParameter Argumento da tarefa (não utilizado)
 */
void sntp_sync_task(void *pvParameter) {
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE,
                                 pdFALSE, portMAX_DELAY); ///< Aguarda indefinidamente até que o Wi-Fi esteja conectado para iniciar a sincronização SNTP

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL); ///< Define o modo de operação SNTP como "polling"
    esp_sntp_setservername(0, "time.google.com"); ///< Define o servidor SNTP (servidor de tempo do Google)
    sntp_set_time_sync_notification_cb(sntp_callback_sync_time); ///< Registra a função de callback para notificação de sincronização de tempo

    sntp_set_sync_interval(1000*60*60*6); ///< Define o intervalo de sincronização para 6 horas (em milissegundos)

    esp_sntp_init(); ///< Inicializa o cliente SNTP
    vTaskDelete(NULL); ///< Deleta a própria tarefa após a inicialização do SNTP, pois a sincronização ocorrerá em segundo plano e através do callback
}

/**
 * @brief Callback de sincronização de tempo SNTP
 * @param tv Estrutura com dados de tempo
 */
void sntp_callback_sync_time(struct timeval *tv) {
    time_t now = 0;
    struct tm timeinfo = { 0 };

    setenv("TZ", "<-03>3", 1); ///< Define o fuso horário para GMT-3 (Brasília)
    tzset(); ///< Atualiza as informações de fuso horário

    time(&now); ///< Obtém o tempo atual (após a sincronização)
    localtime_r(&now, &timeinfo); ///< Converte o tempo para a estrutura tm (tempo local)
    time_synced = true; ///< Seta a flag para indicar que o tempo foi sincronizado
}

/**
 * @brief Atualiza o display LCD conforme o estado do sistema
 * @param state Estado do sistema a ser exibido
 */
void update_lcd_display(system_state_t state) {
    lcd_limpa(); ///< Limpa o conteúdo do display LCD
    lcd_posiciona_cursor(0, 0); ///< Posiciona o cursor na primeira coluna, primeira linha

    switch (state) {
        case STATE_CONNECTING_WIFI: ///< Caso o estado seja "Conectando ao Wi-Fi"
            lcd_escreve_string("Conectando"); ///< Escreve "Conectando"
            lcd_posiciona_cursor(0, 1); ///< Posiciona o cursor na primeira coluna, segunda linha
            lcd_escreve_string("ao Wi-Fi..."); ///< Escreve "ao Wi-Fi..."
            break;
        case STATE_WAITING_CARD: ///< Caso o estado seja "Aguardando Cartão"
            lcd_escreve_string("Aguardando"); ///< Escreve "Aguardando"
            lcd_posiciona_cursor(0, 1); ///< Posiciona o cursor
            lcd_escreve_string("Cartao..."); ///< Escreve "Cartao..."
            break;
        case STATE_VERIFYING_CARD: ///< Caso o estado seja "Verificando Cartão"
            lcd_escreve_string("Verificando"); ///< Escreve "Verificando"
            lcd_posiciona_cursor(0, 1); ///< Posiciona o cursor
            lcd_escreve_string("Cartao..."); ///< Escreve "Cartao..."
            break;
        case STATE_ACCESS_GRANTED: ///< Caso o estado seja "Acesso Liberado"
            lcd_escreve_string("Acesso"); ///< Escreve "Acesso"
            lcd_posiciona_cursor(0, 1); ///< Posiciona o cursor
            lcd_escreve_string("Liberado!"); ///< Escreve "Liberado!"
            break;
        case STATE_ACCESS_DENIED: ///< Caso o estado seja "Acesso Negado"
            lcd_escreve_string("Acesso"); ///< Escreve "Acesso"
            lcd_posiciona_cursor(0, 1); ///< Posiciona o cursor
            lcd_escreve_string("Negado!"); ///< Escreve "Negado!"
            break;
        default: ///< Para qualquer outro estado não previsto
            lcd_escreve_string("Erro de Estado!"); ///< Exibe uma mensagem de erro
            break;
    }
}

/**
 * @brief Tarefa para gerenciamento dos LEDs e display LCD
 * @param pvParameter Argumento da tarefa (não utilizado)
 */
void leds_status_task(void *pvParameter) {
    system_state_t current_state = STATE_CONNECTING_WIFI; ///< Define o estado inicial da tarefa
    system_state_t received_state; ///< Variável para armazenar o estado recebido da fila

    update_lcd_display(current_state); ///< Atualiza o LCD com o estado inicial

    while (1) {
        if (xQueueReceive(system_state_queue, &received_state, pdMS_TO_TICKS(100))) { ///< Tenta receber um novo estado da fila com um timeout
            current_state = received_state; ///< Atualiza o estado atual da tarefa
            update_lcd_display(current_state); ///< Atualiza o display LCD com o novo estado
        }

        switch (current_state) {
            case STATE_CONNECTING_WIFI: ///< Se está conectando ao Wi-Fi
                gpio_set_level(LED_VERDE_PIN, 1); ///< Liga o LED verde
                vTaskDelay(pdMS_TO_TICKS(1000)); ///< Atraso de 1 segundo
                gpio_set_level(LED_VERDE_PIN, 0); ///< Desliga o LED verde
                vTaskDelay(pdMS_TO_TICKS(1000)); ///< Atraso de 1 segundo (efeito de pisca-pisca)
                break;
            case STATE_WAITING_CARD: ///< Se está aguardando o cartão
                gpio_set_level(LED_VERDE_PIN, 1); ///< Mantém o LED verde ligado
                break;
            case STATE_VERIFYING_CARD: ///< Se está verificando o cartão
                gpio_set_level(LED_VERDE_PIN, 0); ///< Desliga o LED verde
                vTaskDelay(pdMS_TO_TICKS(200)); ///< Atraso curto
                gpio_set_level(LED_VERDE_PIN, 1); ///< Liga o LED verde brevemente
                current_state = STATE_WAITING_CARD; ///< Retorna ao estado de aguardando cartão
                break;
            case STATE_ACCESS_GRANTED: ///< Se o acesso foi liberado
                for (int i = 0; i < 2; i++) { ///< Pisca o LED verde duas vezes
                    gpio_set_level(LED_VERDE_PIN, 0);
                    vTaskDelay(pdMS_TO_TICKS(200));
                    gpio_set_level(LED_VERDE_PIN, 1);
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
                gpio_set_level(LED_VERMELHO_PIN, 1); ///< Liga o LED vermelho (indicando acesso liberado)
                vTaskDelay(pdMS_TO_TICKS(2000)); ///< Atraso de 2 segundos
                gpio_set_level(LED_VERMELHO_PIN, 0); ///< Desliga o LED vermelho
                current_state = STATE_WAITING_CARD; ///< Retorna ao estado de aguardando cartão
                xQueueSend(system_state_queue, &current_state, 0); ///< Envia o novo estado para a fila (para atualizar o LCD)
                break;
            case STATE_ACCESS_DENIED: ///< Se o acesso foi negado
                for (int i = 0; i < 6; i++) { ///< Pisca o LED verde seis vezes
                    gpio_set_level(LED_VERDE_PIN, 0);
                    vTaskDelay(pdMS_TO_TICKS(200));
                    gpio_set_level(LED_VERDE_PIN, 1);
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
                gpio_set_level(LED_VERMELHO_PIN, 0); ///< Garante que o LED vermelho esteja desligado (ou o LED vermelho poderia ser usado para indicar negação)
                current_state = STATE_WAITING_CARD; ///< Retorna ao estado de aguardando cartão
                xQueueSend(system_state_queue, &current_state, 0); ///< Envia o novo estado para a fila (para atualizar o LCD)
                break;
            default: ///< Para qualquer outro estado
                gpio_set_level(LED_VERDE_PIN, 0); ///< Desliga o LED verde
                gpio_set_level(LED_VERMELHO_PIN, 0); ///< Desliga o LED vermelho
                break;
        }
    }
}