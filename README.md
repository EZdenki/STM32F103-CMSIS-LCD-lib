# STM32F103-CMSIS-I2C-lib
Basic I2C routines for STM32F103 (Blue Pill) using only CMSIS. Simple projects using this library are:
+ https://github.com/sandynomike/STM32F103-CMSIS-I2C-LED-Scanner
+ https://github.com/sandynomike/STM32F103-CMSIS-I2C-LCD-lib
+ https://github.com/sandynomike/STM32F103-CMSIS-I2C-LCD-AHT10
+ https://github.com/sandynomike/STM32F103-CMSIS-I2C-EEPROM-lib


Requires the following files in the working directory:
+ stm32f103x8.h
+ STM32F103-Delay-lib.c

Another header file may be used instead of stm32f103x8.h if it offers similar functionality. In such cases, the name of the header file should be updated in STM32F103-I2C-lib.c<br>
<br>
Note that the desired target I2C interface, I2C1 or I2C2, will be passed to *thisI2C. The interface name is passed as-is, like<br> `I2C_init( I2C1 );`<br>
## The following routines are implemented:<br>

### Initialize the specified I2C interface at specified speed:
```
void
I2C_init( I2C_TypeDef *thisI2C, uint32_t i2cSpeed )
```
### Set the start bit and wait for acknowledge that it was set:
```
void
I2C_start( I2C_TypeDef *thisI2C )
``` 

### Send the I2C stop bit:
Sends the I2C stop bit and waits for confirmation that the stop bit was actually set.
```
void
I2C_stop( I2C_TypeDef *thisI2C )
```

### Send out the address of the desired I2C target:
Will wait for the target to acknowledge that the address was received.
Note that the routine will hang if no target device acknowledges the address.<br>
readBit = 0 indicates a write request, readBit = 1 indicates a read request.
```
void
I2C_address( I2C_TypeDef *thisI2C, uint8_t address, uint8_t readBit )
```

### Write a byte of data to the specified I2C interface:
Waits for the DR register to be empty and then writes the byte. Waits for acknowledgement that the
byte was transferred before resuming. Note that this routine does not
set the start/stop bits nor poll the address. Use the I2C_writeByte routine in the application
to send a byte of data to a specific I2C target device.
```
void
I2C_write( I2C_TypeDef *thisI2C, uint8_t data )
```

### Read a byte of data from the specified I2C interface:
Command for host to read a single byte. Waits for data to appear (by polling the RXNE bit) and then reads the data.
Will send an ACK bit if the "ack" parameter is non-zero. Does not include start or address or stop commands.<br>
Returns the byte that was read. Note that for reading in single bytes of data, `the I2C_readByte` routine is more
convenient to use.
```
uint8_t
I2C_read( I2C_TypeDef *thisI2C, uint8_t ack )
```
### Write a byte of data to the specified I2C interface at the specified I2C address:
This routine will send the start bit, poll the desired I2C address, send the byte to the targeted device,
and then send the stop bit when completed. This routine should generally be used to send single bytes of
data via I2C.
```
void
I2C_writeByte( I2C_TypeDef *thisI2C, uint8_t data, uint8_t Address )
```

### Read a byte of data from the specified I2C interface from the target of the specified I2C address:
This command will send the start bit, poll the desired I2C address, read a byte from the targeted device,
send the NAK bit, and then send the stop bit pulse. The byte received from the target device will be returned
to the calling routine.
```
uint_8
I2C_readByte( I2C_TypeDef *thisI2C, uint8_t address )
```
