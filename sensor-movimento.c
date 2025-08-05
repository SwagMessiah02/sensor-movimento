#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "st7789/st7789.h"

#define MPU6050_ADDR 0x68
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define I2C_PORT i2c0
#define PIN_SDA 0
#define PIN_SCL 1
#define COLOR_BLACK 0x0000
#define COLOR_GREEN 0x07E0
#define COLOR_YELLOW 0xFF00
#define SERVO_GPIO 2

// Dimensões do display
const int lcd_width = 240;
const int lcd_height = 320;

uint slice_num;
uint chan_num; 

// Configuração do display 
const struct st7789_config lcd_config = {
    .spi = spi0,
    .gpio_din = PICO_DEFAULT_SPI_TX_PIN,
    .gpio_clk = PICO_DEFAULT_SPI_SCK_PIN,
    .gpio_cs = -1,
    .gpio_dc  = 4,
    .gpio_rst = 20
};

// Cabeçalho das funções 
float calcular_inclinacao(int16_t acel_x, int16_t acel_y, int16_t acel_z);
void ajustar_servo(float inclination);
int16_t read_raw_data(uint8_t reg);
void exibir_dados(float inclination);
void mpu6050_init();
void setup();

int main() {
    setup();

    printf("MPU6050 inicializado!\n");

    while (true) {
        int16_t ax = read_raw_data(MPU6050_REG_ACCEL_XOUT_H);
        int16_t ay = read_raw_data(MPU6050_REG_ACCEL_XOUT_H + 2);
        int16_t az = read_raw_data(MPU6050_REG_ACCEL_XOUT_H + 4);

        float inclination = calcular_inclinacao(ax, ay, az);

        printf("Inclinacao: %.2f\n", inclination);
        exibir_dados(inclination);
        ajustar_servo(inclination);
        sleep_ms(500);
    }
}

// Configura os periféricos do microcontrolador
void setup() {
    stdio_init_all();

    st7789_init(&lcd_config, lcd_width, lcd_height);
    st7789_fill(COLOR_BLACK);

    sleep_ms(4000);

    i2c_init(I2C_PORT, 400 * 1000); 
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);

    gpio_init(SERVO_GPIO);
    gpio_set_dir(SERVO_GPIO, GPIO_OUT);

    gpio_set_function(SERVO_GPIO, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(SERVO_GPIO);
    chan_num = pwm_gpio_to_channel(SERVO_GPIO);

    pwm_set_wrap(slice_num, 65535); 
    pwm_set_clkdiv(slice_num, 125); 

    pwm_set_wrap(slice_num, 25000 - 1); 
    pwm_set_clkdiv_int_frac(slice_num, 100, 0); 

    mpu6050_init();

    st7789_clear();
}


// Inicializa o sensor de moviemnto MPU6050
void mpu6050_init() {
    uint8_t buf[] = {MPU6050_REG_PWR_MGMT_1, 0x00};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
}

// Ler dados referentes ao registrador informado
int16_t read_raw_data(uint8_t reg) {
    uint8_t buf[2];
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
    return (int16_t)(buf[0] << 8 | buf[1]);
}

// Calcula a inclinação 
float calcular_inclinacao(int16_t acel_x, int16_t acel_y, int16_t acel_z) {
    float acel_x_g = acel_x / 16384.0;
    float acel_y_g = acel_y / 16384.0;
    float acel_z_g = acel_z / 16384.0;
    return atan2(acel_x_g, 
      sqrt(acel_y_g * acel_y_g + acel_z_g * acel_z_g)) * (180.0 / M_PI);
}

// Ajusta a posição do servo dependendo da luminosidade
void ajustar_servo(float inclination) {
    int angle;

    pwm_set_enabled(slice_num, true);

    angle = (500 + ((inclination > 0 ? inclination : -inclination) / 90) * 110);

    for (int pos = 500; pos <= angle; pos += 1) { 
            pwm_set_chan_level(slice_num, chan_num, pos);
            sleep_ms(2);
    }

    pwm_set_gpio_level(SERVO_GPIO, 0);
    pwm_set_enabled(slice_num, false);
}

// Imprime os dados no display LCD
void exibir_dados(float inclination) {
    char buffer_temp[13];

    snprintf(buffer_temp, sizeof(buffer_temp), 
        "%.2lf graus", inclination);
 
    st7789_clear();

    st7789_draw_text(20, 120, "INCLINATION", COLOR_GREEN, COLOR_BLACK, 3);
    st7789_draw_text(15, 175, buffer_temp, COLOR_YELLOW, COLOR_BLACK, 3);
} 
