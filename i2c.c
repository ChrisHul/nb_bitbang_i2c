// Raspberry Pi Pico port for:
// 2024-11-13 by C. Hultqvist <https://github.com/ChrisHul>
//
// Changelog:
//		2024-11-13 - Initial port release.

/* ============================================
nb_bitbang_i2c code is placed under the MIT license
Copyright (c) 2024 C. Hultqvist

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/structs/iobank0.h"

#include "pico/stdlib.h"
#include "i2c.h"

//#define DEBUG
//#define SHOW_DELAY
#ifdef SHOW_DELAY
#define DEBUG_DELAY printf("Delay\r\n");
#else
#define DEBUG_DELAY
#endif

int gpio_get_pad(uint gpio); // rp pico sdk prototype

void init_i2c_pin( uint8_t i2c_pin) {
    gpio_init( i2c_pin);
    gpio_pull_up( i2c_pin);
    gpio_set_dir( i2c_pin, GPIO_IN);
    gpio_put( i2c_pin, 0);
}

static inline void SDA_HIGH( i2c_h* hi2c) {
    gpio_set_dir( hi2c->pinSDA, GPIO_IN);
#ifdef DEBUG
    sleep_us(1);
    printf( "SDA high: %d\r\n", gpio_get( hi2c->pinSDA));
#endif
}

static inline void SDA_LOW( i2c_h* hi2c) {
    gpio_set_dir( hi2c->pinSDA, GPIO_OUT);
#ifdef DEBUG
    sleep_us(1);
    printf( "SDA low: %d\r\n", gpio_get( hi2c->pinSDA));
#endif
}

static inline void SCL_HIGH( i2c_h* hi2c) {
    gpio_set_dir( hi2c->pinSCL, GPIO_IN);
#ifdef DEBUG
    sleep_us(1);
    printf( "SCL high: %d\r\n", gpio_get( hi2c->pinSCL));
#endif
}

static inline void SCL_LOW( i2c_h* hi2c) {
    gpio_set_dir( hi2c->pinSCL, GPIO_OUT);
#ifdef DEBUG
    sleep_us(1);
    printf( "SCL low: %d\r\n", gpio_get( hi2c->pinSCL));
#endif
}

static inline uint8_t READ_SDA( i2c_h* hi2c) {
#ifdef DEBUG
    printf( "Read SDA: %d\r\n", gpio_get( hi2c->pinSDA));
#endif
    return gpio_get( hi2c->pinSDA);   
}

static inline uint8_t READ_SCL( i2c_h* hi2c) {
#ifdef DEBUG
    printf( "Read SCL: %d\r\n", gpio_get( hi2c->pinSCL));
#endif
    return gpio_get( hi2c->pinSCL);   
}

/*------------------------------------------------------
/ 'i2c_init' initializes the i2c channel pins
/ 'hi2c' is the i2c handle
/------------------------------------------------------*/
void i2c_init( i2c_h* hi2c) {
    init_i2c_pin( hi2c->pinSDA);
    init_i2c_pin( hi2c->pinSCL);
}

/*------------------------------------------------------
/ 'begin_i2c_condition' initializes the non-blocking
/ start/stop/restart/untangle operation
/ 'hi2c' is the i2c handle
/------------------------------------------------------*/
void begin_i2c_condition( i2c_h* hi2c) {
    hi2c->hw_seq = 0;
}

/*------------------------------------------------------
/ 'i2c_start' is the actual bit-banging of the i2c pins
/ for the i2c restart condition operation and is
/ non-blocking.
/ 'hi2c' is the i2c handle
/ this function should be called with a interval of
/ I2C_RATE until I2C_DONE or I2C_ERROR is returned.
/ I2C_STAY means continued operation
/------------------------------------------------------*/
uint8_t i2c_start( i2c_h* hi2c) {
#ifdef DEBUG
    printf("Start case %d\r\n", hi2c->hw_seq);
#endif
    switch( hi2c->hw_seq) {
        case 0:
        if( !READ_SCL( hi2c) || !READ_SDA( hi2c)) {
            return I2C_ERROR;
        } else {
            SDA_LOW( hi2c);
            hi2c->hw_seq++;
            return I2C_STAY;
        }

        case 1:
        SCL_LOW(hi2c);
        hi2c->hw_seq++;
        return I2C_STAY;

        case 2:
        SDA_HIGH( hi2c);
        return I2C_DONE;
    }
}

/*------------------------------------------------------
/ 'i2c_restart' is the actual bit-banging of the i2c pins
/ for the i2c restart condition operation and is
/ non-blocking.
/ 'hi2c' is the i2c handle
/ this function should be called with a interval of
/ I2C_RATE until I2C_DONE or I2C_ERROR is returned.
/ I2C_STAY means continued operation
/------------------------------------------------------*/
uint8_t i2c_restart( i2c_h* hi2c) {
#ifdef DEBUG
    printf("Restart case %d\r\n", hi2c->hw_seq);
#endif
    switch( hi2c->hw_seq) {
        case 0:
        // SDA should be high and SCL low
        if( !READ_SDA( hi2c) || READ_SCL( hi2c)) {
            return I2C_ERROR;
        } else {
            SCL_HIGH( hi2c);
            hi2c->hw_seq++;
        }
        break;

        case 1:
        SDA_LOW( hi2c);
        hi2c->hw_seq++;
        break;
        
        case 2:
        SCL_LOW( hi2c);
        hi2c->hw_seq++;
        break;
        
        case 3:
        SDA_HIGH( hi2c);
        return I2C_DONE;

        default:
        return I2C_ERROR;
    }
    return I2C_STAY;
}


/*------------------------------------------------------
/ 'i2c_stop' is the actual bit-banging of the i2c pins
/ for the i2c stop condition operation and is
/ non-blocking.
/ 'hi2c' is the i2c handle
/ this function should be called with a interval of
/ I2C_RATE until I2C_DONE or I2C_ERROR is returned.
/ I2C_STAY means continued operation
/------------------------------------------------------*/
uint8_t i2c_stop( i2c_h* hi2c) {
#ifdef DEBUG
    printf("Stop case %d\r\n", hi2c->hw_seq);
#endif
    switch( hi2c->hw_seq) {
        case 0:
        // SDA should be high and SCL low
        if( !READ_SDA( hi2c) || READ_SCL( hi2c)) {
            return I2C_ERROR;
        } else {
            SDA_LOW( hi2c);
            hi2c->hw_seq++;
        }
        break;

        case 1:
        SCL_HIGH( hi2c);
        hi2c->hw_seq++;
        break;

        case 2:
        SDA_HIGH( hi2c);
        return I2C_DONE;

        default:
        return I2C_ERROR;
    }
    return I2C_STAY;

}

uint8_t i2c_untangle( i2c_h* hi2c) {
    // clock out possible pending slave write/read
#ifdef DEBUG
    printf("Untangle case %d\r\n", hi2c->hw_seq);
#endif
    if( hi2c->hw_seq < 17) {
        if( hi2c->hw_seq++ & 1 > 0) SCL_HIGH( hi2c);
        else SCL_LOW( hi2c);
        return I2C_STAY;
    } else if ( hi2c->hw_seq == 17)  {
        // initiate stop condition (SCL is LOW)
        SDA_LOW( hi2c);
        hi2c->hw_seq++;
        return I2C_STAY;
    } else if ( hi2c->hw_seq == 18)  {
        SCL_HIGH( hi2c);
        hi2c->hw_seq++;
        return I2C_STAY;
    } else if ( hi2c->hw_seq == 19) {
        SDA_HIGH(hi2c);
        hi2c->hw_seq++;
    } else if ( hi2c->hw_seq == 20) {
        return I2C_DONE;
    }
    return I2C_STAY;
}

/*------------------------------------------------------
/ 'set_current_data_out' sets hi2c->data_out byte to be
/ shifted to the bus (write operation)
/ 'hi2c' is the i2c handle
/------------------------------------------------------*/
uint8_t set_current_data_out( i2c_h* hi2c) {
    switch( hi2c->data_index) {
        case 0:
        hi2c->data_out = hi2c->addr;
        break;

        case 1:
        hi2c->data_out = ( hi2c->reg16b) ? hi2c->reg >> 8 : hi2c->reg & 0xFF;
        break; 

        case 2:
        hi2c->data_out = hi2c->reg & 0xFF;
        if( hi2c->reg16b) break;

        default:
        hi2c->data_out = ( hi2c->reg16b)? *( hi2c->data + hi2c->data_index - 3):
                                         *( hi2c->data + hi2c->data_index - 2);
        break;
    }
#ifdef DEBUG
    printf("Data out : %d, %d --------- data out --------- data out --------- data out\r\n", hi2c->data_out, *(hi2c->data));
#endif
}


/*------------------------------------------------------
/ 'begin_write_i2c' initializes a write operation
/ 'hi2c' is the i2c handle
/------------------------------------------------------*/
void begin_write_i2c( i2c_h* hi2c) {
    hi2c->hw_seq = 0;
    hi2c->bit_count = 8;
    hi2c->data_index = 0;
    set_current_data_out( hi2c);
    if( ( hi2c->data_out & 0x80) == 0) SDA_LOW(hi2c); // send first bit                                        
    else SDA_HIGH( hi2c);
}

/*------------------------------------------------------
/ 'do_write_i2c' is the actual bit-banging of the i2c pins
/ for the write and is non-blocking.
/ This function is to be called with an interval of
/ I2C_RATE until I2C_DONE, NACK or I2C_ERROR is returned.
/ I2C_STAY means continued operation
/------------------------------------------------------*/
uint8_t do_write_i2c( i2c_h* hi2c) {
    static uint8_t current_data_byte;
#ifdef DEBUG
    printf("Do_write case %d, bytecount: %d, bitcount: %d\r\n", hi2c->hw_seq, hi2c->byte_count, hi2c->bit_count);
#endif
    switch( hi2c->hw_seq) {

        case 0:
        hi2c->hw_seq++; // delay after first bit setting
        break;

        case 1:
        SCL_HIGH( hi2c);
        hi2c->hw_seq++;
        break;

        case 2:
        if( !READ_SCL( hi2c)) {
            // Clock stretching                
            if( hi2c->timeout_count++ > I2C_MAX_TIMEOUT) return I2C_ERROR;
        } else {
            SCL_LOW( hi2c);
            hi2c->hw_seq++;
        }
        break;

        case 3:
        hi2c->data_out <<= 1;
        if( ( --hi2c->bit_count > 0) && ( ( hi2c->data_out & 0x80) == 0)) SDA_LOW( hi2c);
        else SDA_HIGH(hi2c);
        if( hi2c->bit_count > 0) hi2c->hw_seq = 1;
        else hi2c->hw_seq++;
        break;

        case 4:
        // start reading ACK/NACK
        hi2c->timeout_count = 0;
        SCL_HIGH( hi2c);
        hi2c->hw_seq++;
        break;

        case 5:
        if( !READ_SCL( hi2c)) {
                // clock stretching
            if( hi2c->timeout_count++ > I2C_MAX_TIMEOUT) {
                printf( "Timeout error\r\n");
                return I2C_ERROR;
            }
        } else {
            if( READ_SDA( hi2c)) hi2c->hw_seq = 6; // NACK
            else hi2c->hw_seq = 7;
            SCL_LOW( hi2c);
        }
        break;

        case 6:
        return I2C_NACK;

        case 7:
        if( --hi2c->byte_count == 0) return I2C_DONE;
        hi2c->bit_count = 8;
        hi2c->data_index++;
        set_current_data_out( hi2c);
        if( ( hi2c->data_out & 0x80) == 0) SDA_LOW(hi2c); // output MSB
        hi2c->hw_seq = 1;
        break;

        default:
        hi2c->hw_seq = 0;
        break;
    }
    return I2C_STAY;
}

/*------------------------------------------------------
/ 'begin_read_i2c' initializes the read operation
/ 'hi2c' is the i2c handle
/------------------------------------------------------*/
void begin_read_i2c( i2c_h* hi2c) {
    hi2c->hw_seq = 0;
    hi2c->byte_count = hi2c->len;
    hi2c->bit_count = 8;
    hi2c->data_index = 0;
}

/*------------------------------------------------------
/ 'do_read_i2c' is the actual bit-banging of the i2c pins
/ for the read and is non-blocking.
/ 'hi2c' is the i2c handle
/ this function should be called with an interval of
/ I2C_RATE until I2C_DONE or I2C_ERROR is returned.
/ I2C_STAY means continued operation
/------------------------------------------------------*/
uint8_t do_read_i2c( i2c_h* hi2c) {
#ifdef DEBUG
    printf("Do_write case %d, bytecount: %d, bitcount: %d\r\n", hi2c->hw_seq, hi2c->byte_count, hi2c->bit_count);
#endif

    switch( hi2c->hw_seq) {
        case 0:
        SCL_HIGH( hi2c);
        hi2c->hw_seq++;
        break;

        case 1:
        if( READ_SDA( hi2c)) *( hi2c->data + hi2c->data_index) |= 1;
        else *( hi2c->data + hi2c->data_index) &= 0xFE;
        SCL_LOW( hi2c);
        if( --hi2c->bit_count > 0) {
            *( hi2c->data + hi2c->data_index) <<= 1; // shift unless last bit
            hi2c->hw_seq = 0;
        } else hi2c->hw_seq++; // all bits read, proceed with ACK/NACK
        break;

        case 2:
        if( --hi2c->byte_count > 0) SDA_LOW( hi2c); // sending ACK: SCL will be pulsed next call
        else SDA_HIGH( hi2c);
        hi2c->hw_seq++;
        hi2c->timeout_count = 0;
        break;

        case 3:
        SCL_HIGH( hi2c);
        hi2c->hw_seq++;
        break;

        case 4:
        if( !READ_SCL( hi2c)) {
            // clock stretching
            if( hi2c->timeout_count++ > I2C_MAX_TIMEOUT) return I2C_ERROR;
            hi2c->hw_seq = 1;
        } else {
            SCL_LOW( hi2c);
            hi2c->hw_seq++;
        }
        break;

        case 5:
        SDA_HIGH( hi2c); // release SDA after ACK
        hi2c->hw_seq++;
        break;

        case 6:
        if( hi2c->byte_count == 0) return I2C_DONE;
        else {
            hi2c->bit_count = 8;
            hi2c->data_index++;
        }
        // fall through ( do next byte)

        default:
        hi2c->hw_seq = 0;
        break;
    }
    return I2C_STAY;
}

/*------------------------------------------------------
/ 'i2c_send_start_blocking' outputs a start condition on
/ the i2c bus. 'hi2c' is the bus handle.
/ This function is blocking and will only return after
/ completed operation or failure. Will return true on
/ error and false on successful completion
/------------------------------------------------------*/
bool i2c_send_start_blocking( i2c_h* hi2c) {
    uint8_t res;
    begin_i2c_condition( hi2c);
    while( res = i2c_start( hi2c) == I2C_STAY) { sleep_us( I2C_RATE); DEBUG_DELAY};
    if( res == I2C_ERROR) return true; // error while trying to send start condition
    else return false;
}

/*------------------------------------------------------
/ 'i2c_send_stop_blocking' outputs a stop condition on
/ the i2c bus. 'hi2c' is the bus handle.
/ This function is blocking and will only return after
/ completed operation or failure. Will return true on
/ error and false on successful completion
/------------------------------------------------------*/
bool i2c_send_stop_blocking( i2c_h* hi2c) {
    uint8_t res;
    begin_i2c_condition( hi2c);
    while( res = i2c_stop( hi2c) == I2C_STAY) { sleep_us( I2C_RATE); DEBUG_DELAY};
    if( res == I2C_ERROR) return true; // error while trying to send stop condition
    else return false;
}

/*------------------------------------------------------
/ 'i2c_transfer_setup' initiates a read/write operation
/ on the i2c bus.
/ 'hi2c' is the bus handle, 'addr' is the device address
/ to be read from, 'reg' is the internal register start
/ address, 'len' the number of bytes to be read and
/ 'reg16b' is boolean true for a 16-bit register value,
/ otherwise 8-bit.
/------------------------------------------------------*/
void i2c_transfer_setup( i2c_h* hi2c, uint8_t addr, uint16_t reg, uint16_t len, bool reg16b) {
    hi2c->addr = addr << 1;
    hi2c->len = len; // used for reading
    hi2c->byte_count = (reg16b)? len + 3: len + 2; // used for write operation
    hi2c->reg = reg;
    hi2c->reg16b = reg16b;
    hi2c->machine_state = I2C_SM_START;
}

/*------------------------------------------------------
/ 'i2c_read_reg_blocking' reads any number of bytes
/ from the i2c bus.
/ 'hi2c->data' points to the data buffer
/ 'i2c_send_start_blocking' needs to be called prior
/ to invoking this function.
/ This function is blocking and will only return after
/ completed operation or failure. Will return true on
/ error, otherwise false.
/------------------------------------------------------*/
bool i2c_read_blocking( i2c_h* hi2c) {
    uint8_t res;
#ifdef DEBUG
    printf( "read_data: %X, %X -----------------------\r\n", hi2c->addr, hi2c->reg);
#endif
    hi2c->byte_count = (hi2c->reg16b)? 3: 2;
    begin_write_i2c( hi2c);
    while( ( res = do_write_i2c( hi2c)) == I2C_STAY) { sleep_us ( I2C_RATE); DEBUG_DELAY}
#ifdef DEBUG
    printf( "res: %d\r\n", res);
#endif
    if( res == I2C_NACK || res == I2C_ERROR) return true;

    // i2c restart condition:
    begin_i2c_condition( hi2c);
    while ( ( res = i2c_restart( hi2c)) == I2C_STAY) { sleep_us ( I2C_RATE); DEBUG_DELAY}
    if( res == I2C_ERROR) return true;

    // read the device:
    hi2c->addr |= 1; hi2c->byte_count = 1; // send device address with read command
    begin_write_i2c( hi2c);

    while( ( res = do_write_i2c( hi2c)) == I2C_STAY) { sleep_us ( I2C_RATE); DEBUG_DELAY}
    if( res == I2C_NACK) return true;

    // start the actual reading
    begin_read_i2c( hi2c); sleep_us ( I2C_RATE); DEBUG_DELAY
    while( ( res = do_read_i2c( hi2c)) == I2C_STAY) { sleep_us ( I2C_RATE); DEBUG_DELAY}
    if( res == I2C_NACK) return true; // not responding

    // restart the i2c so that it's ready for another operation
    begin_i2c_condition(hi2c);
    while( ( res = i2c_restart( hi2c)) == I2C_STAY) { sleep_us ( I2C_RATE); DEBUG_DELAY}
    if( res == I2C_ERROR) return true;
    else return false; // all good
}

/*------------------------------------------------------
/ 'i2c_write_blocking' writes any number of bytes < 2^16
/ to the i2c bus.
/ Parameters are given in the hi2c structure.
/ 'hi2c->data' points to the data buffer
/ 'i2c_send_start_blocking' needs to be called prior
/ to invoking this function.
/ This function is blocking and will only return after
/ completed operation or failure. Will return true on
/ error
/------------------------------------------------------*/
bool i2c_write_blocking( i2c_h* hi2c) {
    uint8_t res;
#ifdef DEBUG
    printf( "write blocking data: %X, %X, %X ---------- write_data -------------\r\n",
            hi2c->addr, hi2c->reg, *( hi2c->data + 0));
#endif
    begin_write_i2c( hi2c);
    while( ( res = do_write_i2c( hi2c)) == I2C_STAY) { sleep_us ( I2C_RATE); DEBUG_DELAY}
#ifdef DEBUG
    printf( "Write res: %d\r\n", res);
#endif
    if( res == I2C_NACK || res == I2C_ERROR) return true; // not responding
    begin_i2c_condition( hi2c);
    while( (res = i2c_restart( hi2c)) == I2C_STAY) { sleep_us ( I2C_RATE); DEBUG_DELAY}
    if( res == I2C_ERROR) return true;
    else return false;
}

/*------------------------------------------------------
/ 'i2c_write_sm' is a state machine that performes a non-
/ blocking 8-bit register write operation on the i2c bus.
/ The funcion needs to be serviced in an interval of half
/ a clock cycle of the i2c clock rate.
/ Hence, a clock rate of 100 kHz will yield an interval of
/ 5 us. Since the bus is synchronized this timing is not
/ critical.
/ 'hi2c->data' points to the data buffer
/ 'hi2c->machine_state' needs to be monitored for the
/ following conditions:
/ 1. I2C_SM_DONE - meaning that the operation has been
/ completed and result is available in *(hi2c->data)
/ 2. I2C_SM_ERROR_CATCH - meaning that the the read failed.
/------------------------------------------------------*/
void i2c_write_sm(i2c_h* hi2c) {
uint8_t res;
#ifdef DEBUG
    printf("i2c_write_sm case %d\r\n", hi2c->machine_state);
#endif
switch( hi2c->machine_state) {
    case I2C_SM_IDLE:
    break;

    case I2C_SM_START:
    begin_i2c_condition( hi2c);
    hi2c->machine_state++;
    // fall through
    case I2C_SM_START + 1:
    if( ( res = i2c_start( hi2c)) == I2C_STAY) return;
    if( res == I2C_ERROR) hi2c->machine_state = I2C_SM_STOP_AND_ERROR;
    else hi2c->machine_state++;
    break;

    case I2C_SM_START + 2: // set read adress
    *( hi2c->data) = hi2c->addr;
    *( hi2c->data + 1) = hi2c->reg;
    hi2c->byte_count = hi2c->len + 2;
    begin_write_i2c( hi2c);
    hi2c->machine_state++;
    break;

    case I2C_SM_START + 3:
    res = do_write_i2c( hi2c);
    if( res == I2C_STAY) break;
    if( res == I2C_DONE) {
        begin_i2c_condition( hi2c);
        hi2c->machine_state++;
    }
    else hi2c->machine_state = I2C_SM_STOP_AND_ERROR;
    break;

    case I2C_SM_START + 4:
    if( ( res = i2c_stop( hi2c)) == I2C_DONE) {
        hi2c->machine_state = I2C_SM_DONE;
    }
    if( res == I2C_ERROR) hi2c->machine_state = I2C_SM_STOP_AND_ERROR;
    break;

    case I2C_SM_DONE:
        // Locked until calling layer initiates new operation
    break;

    case I2C_SM_STOP_AND_ERROR:
    // unwind any operation by clocking it out
    begin_i2c_condition( hi2c);
    hi2c->machine_state++;
    SDA_HIGH( hi2c); // ensure NACK
    // fall through

    case I2C_SM_STOP_AND_ERROR + 1:
    if( i2c_untangle( hi2c) == I2C_DONE) hi2c->machine_state = I2C_SM_ERROR_CATCH;
    break;

    case I2C_SM_ERROR_CATCH:
        // Locked until calling layer starts another operation
    break;

    default:
    hi2c->machine_state = I2C_SM_STOP_AND_ERROR;
    break;
}
}
/*------------------------------------------------------
/ 'i2c_read_sm' is a state machine that performes a non-
/ blocking 8-bit register read operation on the i2c bus.
/ The funcion needs to be serviced in an interval of half
/ a clock cycle of the i2c clock rate.
/ Hence, a clock rate of 100 kHz will yield an interval of
/ 5 us. Since the bus is synchronized this timing is not
/ critical.
/ 'hi2c->data' points to the data buffer
/ 'hi2c->machine_state' needs to be monitored for the
/ following conditions:
/ 1. I2C_SM_DONE - meaning the operation has been
/ completed and result is available in *(hi2c->data)
/ 2. I2C_SM_ERROR_CATCH - meaning the the read failed.
/------------------------------------------------------*/
void i2c_read_sm(i2c_h* hi2c) {
uint8_t res;
#ifdef DEBUG
    printf("i2c_read_sm case %d\r\n", hi2c->machine_state);
#endif
switch( hi2c->machine_state) {
    case I2C_SM_IDLE:
    break;

    case I2C_SM_START:
    begin_i2c_condition( hi2c);
    hi2c->machine_state++;
    // fall through

    case I2C_SM_START + 1:
    if( ( res = i2c_start( hi2c)) == I2C_STAY) return;
    if( res == I2C_ERROR) hi2c->machine_state = I2C_SM_STOP_AND_ERROR;
    else hi2c->machine_state++;
    break;

    case I2C_SM_START + 2: // read output: first set read adress
    hi2c->byte_count = (hi2c->reg16b) ? 3: 2;
#ifdef DEBUG
    printf( "Reading non-blocking sending reg address: %X, %X -----------------------\r\n", hi2c->addr, hi2c->reg);
#endif
    begin_write_i2c( hi2c);
    hi2c->machine_state++;
    break;

    case I2C_SM_START + 3:
    res = do_write_i2c( hi2c);
    if( res == I2C_STAY) break;
    if( res == I2C_DONE) {
        begin_i2c_condition( hi2c);
        hi2c->machine_state++;
    }
    else hi2c->machine_state = I2C_SM_STOP_AND_ERROR;
    break;

    case I2C_SM_START + 4:
    if( ( res = i2c_restart( hi2c)) == I2C_DONE) hi2c->machine_state++;
    if( res == I2C_ERROR) hi2c->machine_state = I2C_SM_STOP_AND_ERROR;
    break;

    case I2C_SM_START + 5:
    // start read sequence
    hi2c->addr |= 1;
    hi2c->byte_count = 1;
#ifdef DEBUG
    printf( "Sending device address: %d\r\n", hi2c->addr);
#endif
    begin_write_i2c( hi2c);
    hi2c->machine_state++;
    break;

    case I2C_SM_START + 6:
    if( ( res = do_write_i2c( hi2c)) == I2C_ERROR || res == I2C_NACK){
        hi2c->machine_state = I2C_SM_STOP_AND_ERROR; break;
        }
    if( res != I2C_DONE) break;
    begin_read_i2c( hi2c);
    hi2c->machine_state++;
#ifdef DEBUG
    printf( "Reading data ---- reading data ---- reading data ---- reading data ---- reading data\r\n", hi2c->addr);
#endif
    // fall through

    case I2C_SM_START + 7:
    if( ( res = do_read_i2c( hi2c)) == I2C_DONE) {
        begin_i2c_condition( hi2c);
        hi2c->machine_state++;
    }
    else if( res == I2C_ERROR) hi2c->machine_state = I2C_SM_STOP_AND_ERROR;
    break;

    case I2C_SM_START + 8:
    if( ( res = i2c_stop( hi2c)) == I2C_DONE) {
        hi2c->machine_state = I2C_SM_DONE;
    }
    else if( res == I2C_ERROR) hi2c->machine_state = I2C_SM_STOP_AND_ERROR;
    break;

    case I2C_SM_DONE:
        // Locked until calling layer initiates new operation
    break;

    case I2C_SM_STOP_AND_ERROR:
    // unwind any operation by clocking it out
    begin_i2c_condition( hi2c);
    hi2c->machine_state++;
    SDA_HIGH( hi2c); // ensure NACK
    // fall through

    case I2C_SM_STOP_AND_ERROR + 1:
    if( i2c_untangle( hi2c) == I2C_DONE) hi2c->machine_state = I2C_SM_ERROR_CATCH;
    break;

    case I2C_SM_ERROR_CATCH:
        // Locked until calling layer starts another operation
    break;

    default:
    hi2c->machine_state = I2C_SM_STOP_AND_ERROR;
    break;
}
}

