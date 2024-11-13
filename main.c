#include <stdio.h>
#include "pico/stdlib.h"
#include "i2c.h"

i2c_h hi2c0 = {
    .pinSDA = 2,
    .pinSCL = 3
};

int main() {

    // Buffer to store raw reads
    uint8_t data[6];
    hi2c0.data = ( uint8_t*) &data;
    bool error = false;
    uint8_t error_count;
    uint8_t machine_state;
    uint64_t last_reading;

    // init i2c channel 0
    i2c_init( &hi2c0);

    // Initialize chosen serial port
    stdio_init_all();

    sleep_ms(1000);

    printf( "Starting qmc5883_setup. Blocking functions are used for this\r\n");

    // issue start condition
    if( i2c_send_start_blocking( &hi2c0)) {
        printf("I2c channel 0 blocked on initialization -- operation stopped\r\n");
        while( true) tight_loop_contents();
    }
    
    // test REG_ID for QMC5883 chip validation (0XFF)
    i2c_transfer_setup( &hi2c0, 0xD, 0xD, 1, false);
    if( i2c_read_blocking( &hi2c0)) error = true;    
    if( *( hi2c0.data + 0) != 0xFF) error = true;
    if(error) {
        printf("ERROR: Could not communicate with qmc5883\r\n");
        while (true) tight_loop_contents();
    }
    // initialize set/reset register
    i2c_transfer_setup( &hi2c0, 0xD, 0xB, 1, false);
    *( hi2c0.data) = 1;
    if( i2c_write_blocking( &hi2c0)) error =  true;

    // initialize mode register
    i2c_transfer_setup( &hi2c0, 0xD, 9, 1, false);
    *( hi2c0.data) = 1 | 0xC | 0x10 | 0;
    if( i2c_write_blocking( &hi2c0)) error = true;

    // send stop
    i2c_send_stop_blocking( &hi2c0);

    machine_state = 0;
    error_count = 0;

    // Loop forever
    printf( "Starting loop...\r\n");
    while (true) {
        if( to_us_since_boot( get_absolute_time()) - hi2c0.lastmicros > I2C_RATE) {
           // ensure i2c timing
            hi2c0.lastmicros = to_us_since_boot( get_absolute_time());
            switch( machine_state) {
                case 0:
                // test if 1 sec since last readout
                if( to_us_since_boot( get_absolute_time()) - last_reading > (uint64_t)1000000) {
                    // if so, start reading
                    machine_state++;
                } else break;

                case 1:
                last_reading = to_us_since_boot( get_absolute_time());
                i2c_transfer_setup( &hi2c0, 0XD, 0, 6, false); // read bytes 0-5
                machine_state++;

                case 2:
                i2c_read_sm( &hi2c0);
                if( hi2c0.machine_state == I2C_SM_ERROR_CATCH) {
                    if( ++error_count > 5) {
                        printf( "No communication with qmc5883. System stopped!");
                        while( true) tight_loop_contents();
                    } else {
                        // retry
                        hi2c0.machine_state = 1;
                    }
                    break;
                } else if( hi2c0.machine_state != I2C_SM_DONE) break;
                // done with qmc5883
                printf( "Magnitude x:%d, y:%d, z:%d\r\n", (int16_t)(*(hi2c0.data + 0) | *(hi2c0.data + 1) << 8),
                                                          (int16_t)(*(hi2c0.data + 2) | *(hi2c0.data + 3) << 8),
                                                          (int16_t)(*(hi2c0.data + 4) | *(hi2c0.data + 5) << 8));
                machine_state = 0;
                error_count = 0;
                break;
            }
        }
        tight_loop_contents();
    }
}