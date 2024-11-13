#pragma once
#include <stdio.h>
#include <stdbool.h>

#define I2C_8_BIT_REGISTER false
#define I2C_16_BIT_REGISTER true

#define I2C_STAY 0
#define I2C_DONE 1
#define I2C_ACK 2
#define I2C_NACK 3
#define I2C_TIMEOUT 4
#define I2C_ERROR 5

#define I2C_SM_IDLE 0
#define I2C_SM_START 1
#define I2C_SM_STOP_AND_ERROR 0x80
#define I2C_SM_ERROR_CATCH 0x90
#define I2C_SM_DONE 0xFE
#define I2C_SM_ERROR 0xFF

#define I2C_RATE 10
#define I2C_MAX_TIMEOUT 5000 / I2C_RATE

typedef struct {
    const uint8_t pinSDA;
    const uint8_t pinSCL;
    uint64_t lastmicros;
    uint8_t addr;
    uint8_t reg;
    uint16_t len;
    bool reg16b;
    uint8_t machine_state;
    uint8_t* data;
    uint8_t data_out;
    uint8_t bit_count;
    uint16_t byte_count;
    uint16_t data_index;
    uint16_t timeout_count;
    uint8_t hw_seq;
} i2c_h;

void init_i2c_pin( uint8_t i2c_pin);

void begin_i2c_condition( i2c_h* hi2c);
uint8_t i2c_start( i2c_h* hi2c);
uint8_t i2c_restart( i2c_h* hi2c);
uint8_t i2c_stop( i2c_h* hi2c);
uint8_t i2c_untangle( i2c_h* hi2c);
void begin_write_i2c( i2c_h* hi2c);
uint8_t set_current_data_out( i2c_h* hi2c);
void i2c_init( i2c_h* hi2c);
uint8_t do_write_i2c( i2c_h* hi2c);
void begin_read_i2c( i2c_h* hi2c);
uint8_t do_read_i2c( i2c_h* hi2c);
bool i2c_send_start_blocking( i2c_h* hi2c);
bool i2c_send_stop_blocking( i2c_h* hi2c);
bool i2c_write_blocking( i2c_h* hi2c);
bool i2c_read_blocking( i2c_h* hi2c);

void i2c_transfer_setup( i2c_h* hi2c, uint8_t addr, uint16_t reg, uint16_t len, bool reg16b);
void i2c_write_sm(i2c_h* hi2c);
void i2c_read_sm(i2c_h* hi2c);

