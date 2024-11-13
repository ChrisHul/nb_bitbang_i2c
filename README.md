# nb_bitbang_i2c
A non-blocking bitbang state machine i2c module

nb_bitbang_i2c provides a non-blocking functionality for master reading from and writing to i2c devices. Any pair of pins can be used as a i2c port and the number of channels is only limited by the number of i/o pins available.

It is in the first place intended to operate with multiple sensor modules with i2c interface. This way, several identical modules with one unique device address can run on different i2c channels.

The module is 100% non-blocking down to every single bit operation.

The non-blocking nature of the functions makes it possible to run parallel tasks on different channels at the same time.

For all this to work, the non-blocking function calls must be placed in a tight loop. Other non-blocking functions may also be included in this loop, provided that no delays are permitted. I2c operations are synchronized by the clock, so that short delays will not affect performance.

Minimum test code is provided in main.c together with other files required to run the example in rp204 Pico SDK environment. It should be possible to run the code in other c language environments with some minor changes.
