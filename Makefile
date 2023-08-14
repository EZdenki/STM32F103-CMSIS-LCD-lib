################################################################################
# Makefile for STM32 CMSIS directory structure extracated from STM32CubeIDE
# project.
#
# Mike Shegedin, 2023
################################################################################

TARGET    = CMSIS-PWM-Input
SOURCE    = main
MCPU      = cortex-m3
STARTUP   = startup_stm32f103cbtx
LOADER    = STM32F103CBTX_FLASH.ld

FLASHER   = $(STMCUBE_PROG)
FLASHPORT = SWD

CC = arm-none-eabi-gcc

CFLAGS = -mcpu=cortex-m3 -g3 --specs=nano.specs -Os -mthumb -mfloat-abi=soft -Wall

INCLUDE1 =STM32CubeF1/Drivers/CMSIS/Device/ST/STM32F1xx/Include
INCLUDE2 =STM32CubeF1/Drivers/CMSIS/Include
#ST_INCL  =STM32CubeF1/Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc
ST_INCL  =STM32CubeF1/Core/Startup

$(TARGET).elf: $(SOURCE).o $(STARTUP).o $(LOADER) Makefile
	$(CC) -o $@ $(SOURCE).o $(STARTUP).o -mcpu=$(MCPU) --specs=nosys.specs -T"$(LOADER)" \
	-Wl,-Map=$(TARGET).map -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb \
	-Wl,--start-group -lc -lm -Wl,--end-group
	arm-none-eabi-size $(TARGET).elf
	$(FLASHER) -c port=$(FLASHPORT) -w $(TARGET).elf --start

$(STARTUP).o: $(ST_INCL)/$(STARTUP).s Makefile
	$(CC) $(CFLAGS) -DDEBUG -c -x assembler-with-cpp -o $@ $<

$(SOURCE).o: $(SOURCE).c Makefile
	$(CC) $< $(CFLAGS) -I$(INCLUDE1) -I$(INCLUDE2) -std=gnu11 -DDEBUG  \
	-c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -o $@

clean:
	del *.o *.elf *.map *.su
