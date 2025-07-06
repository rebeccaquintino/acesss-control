#include "display_lcd.h"

#define LCD_ADDR         0x27 // Endereço I2C do PCF8574
#define LCD_NUM_ROWS     2
#define LCD_NUM_COLS     16

#define LCD_BACKLIGHT  (1 << 3)
#define ENABLE_BIT     (1 << 2)
#define RS_BIT         (1 << 0)

static uint8_t backlight_state = (uint8_t) LCD_BACKLIGHT;
static i2c_master_dev_handle_t lcd_i2c_handle;

static void lcd_escreve_nibble(uint8_t nibble, uint8_t rs) {
  uint8_t base_data = (nibble << 4) | backlight_state | (rs ? RS_BIT : 0);
  uint8_t data_out[2] = {base_data | ENABLE_BIT, base_data & ~ENABLE_BIT};
  i2c_master_transmit(lcd_i2c_handle, data_out, sizeof(data_out),-1);
}

static void lcd_envia_cmd(uint8_t cmd) {
    lcd_escreve_nibble(cmd >> 4, 0);
    lcd_escreve_nibble(cmd & 0x0F, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
}

esp_err_t lcd_config(i2c_master_bus_handle_t bus_handle) {
  i2c_device_config_t lcd_i2c_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = LCD_ADDR,
    .scl_speed_hz = 100000,
  };

  i2c_master_bus_add_device(bus_handle, &lcd_i2c_cfg, &lcd_i2c_handle);

    // Sequência de inicialização
    lcd_escreve_nibble(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_escreve_nibble(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_escreve_nibble(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_escreve_nibble(0x02, 0);

    lcd_envia_cmd(0x28); // 4 bits, 2 linhas, 5x8
    lcd_envia_cmd(0x0C); // Display on, cursor off
    lcd_envia_cmd(0x06); // Incrementa cursor
    lcd_limpa();

    return ESP_OK;
}

static void lcd_envia_dado(uint8_t data) {
    lcd_escreve_nibble(data >> 4, 1);
    lcd_escreve_nibble(data & 0x0F, 1);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void lcd_limpa(void) {
    lcd_envia_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void lcd_inicio(void) {
    lcd_envia_cmd(0x02);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void lcd_posiciona_cursor(uint8_t col, uint8_t row) {
    const uint8_t row_offsets[] = {0x00, 0x40}; 
    if (row >= LCD_NUM_ROWS || col >= LCD_NUM_COLS) return;
    lcd_envia_cmd(0x80 | (col + row_offsets[row]));
}

void lcd_escreve_caractere(char c) {
    lcd_envia_dado((uint8_t)c);
}

void lcd_escreve_string(const char* str) {
    while (*str) {
        lcd_escreve_caractere(*str);
        str++;
    }
}

void lcd_backlight_ligado(void) {
    backlight_state = (uint8_t) LCD_BACKLIGHT;
    lcd_envia_cmd(0x00); // Força atualização
}

void lcd_backlight_desligado(void) {
    backlight_state = 0x00;
    lcd_envia_cmd(0x00);
}
