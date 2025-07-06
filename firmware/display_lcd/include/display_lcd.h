/**
 * @file display_lcd.h
 * @brief Funções para controle de um display LCD
 *
 * Essa biblioteca fornece as funções fundamentais para o controle de um display LCD
 * conectado ao barramento I2C de um ESP32-C3. O display deve possuir o endereço padrao
 * I2C 0x27 para o correto funcionamento da comunicação.
 *
 * @author Raul Guedert
 */

#pragma once
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"

/**
 * @brief Configura o display LCD
 *
 * A função adiciona o display LCD no barramento fornecido e realiza as etapas básicas de configuração,
 * necessárias para iniciar o display no modo adequado e estar pronto para receber os comandos.
 *
 * @param bus_handle Identificador do barramento mestre I2C (i2c_master_bus_handle_t), deve ser configurado anteriormente
 * @return ESP_OK => Se a configuração foi realizada corretamente
 */
esp_err_t lcd_config(i2c_master_bus_handle_t bus_handle);

/**
 * @brief Limpa o display LCD
 *
 * Apaga qualquer valor escrito no display LCD.
 */
void lcd_limpa(void);

/**
 * @brief Posiciona o cursor no início
 */
void lcd_inicio(void);

/**
 * @brief Define a posição do cursor
 *
 * Antes de um processo de escrita, é necessário posicionar o cursor no ponto inicial da escrita,
 * definindo a linha e a coluna onde o cursor será posicionado
 *
 * @param col Coluna na qual o cursor será posicionado
 * @param row Linha na qual o cursor será posicionado (0 = primeira linha, 1 = segunda linha)
 */
void lcd_posiciona_cursor(uint8_t col, uint8_t row);

/**
 * @brief Escreve um caractere
 *
 * Escreve um único caractere na posição do cursor
 *
 * @param c Caractere que será escrito
 */
void lcd_escreve_caractere(char c);

/**
 * @brief Escreve uma string
 *
 * Escreve uma cadeira de caracteres a partir da posição do cursor
 *
 * @param str String que será escrita 
 */
void lcd_escreve_string(const char* str);

/**
 * @brief Liga a luz de fundo (backlight)
 */
void lcd_backlight_ligado(void);

/**
 * @brief Desliga a luz de fundo (backlight)
 */
void lcd_backlight_desligado(void);