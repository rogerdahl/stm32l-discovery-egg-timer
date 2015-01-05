# Put your stlink folder here so make flash will work.
STLINK=/home/dahl/bin/stlink

# put your *.o targets here, make should handle the rest!
#SRCS = main.c system_stm32l1xx.c stm32l_discovery_lcd.c
SRCS = main.c system_stm32l1xx.c stm32l_discovery_lcd.c

# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)
PROJ_NAME=timer

# Location of the Libraries folder from the STM32F0xx Standard Peripheral Library
STD_PERIPH_LIB=Libraries

# Location of the linker scripts
LDSCRIPT_INC=Device/ldscripts

# that's it, no need to change anything below this line!

###################################################

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump
SIZE=arm-none-eabi-size

CFLAGS  = -Wall -g -std=c99 -Os  
#CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m0 -march=armv6s-m
CFLAGS += -mlittle-endian -mcpu=cortex-m0  -march=armv6-m -mthumb
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -Wl,--gc-sections -Wl,-Map=$(PROJ_NAME).map

###################################################

vpath %.c src
vpath %.a $(STD_PERIPH_LIB)

ROOT=$(shell pwd)

# Search path for include headers.
CFLAGS += -I $(STD_PERIPH_LIB)
CFLAGS += -I $(STD_PERIPH_LIB)/CMSIS/Device/ST/STM32L1xx/Include
CFLAGS += -I $(STD_PERIPH_LIB)/CMSIS/Include
CFLAGS += -I $(STD_PERIPH_LIB)/STM32L1xx_StdPeriph_Driver/inc
CFLAGS += -I $(STD_PERIPH_LIB)/STMTouch_Driver/inc/

#CFLAGS += -include $(STD_PERIPH_LIB)/stm32l1xx_conf.h

SRCS += Device/startup_stm32l1xx_md.s

# need if you want to build with -DUSE_CMSIS 
#SRCS += stm32f0_discovery.c
#SRCS += stm32f0_discovery.c stm32f0xx_it.c

OBJS = $(SRCS:.c=.o)

###################################################

.PHONY: lib proj

all: lib proj

lib:
	$(MAKE) -C $(STD_PERIPH_LIB)

proj: 	$(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ -L$(STD_PERIPH_LIB) -lstm32l1 -L$(LDSCRIPT_INC) -Tstm32l1xx.ld
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin
	$(OBJDUMP) -St $(PROJ_NAME).elf >$(PROJ_NAME).lst
	$(SIZE) $(PROJ_NAME).elf
	
# Flash
flash: proj
	$(STLINK)/st-flash write $(PROJ_NAME).bin 0x08000000

clean:
	find ./ -name '*~' | xargs rm -f	
	rm -f *.o
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin
	rm -f $(PROJ_NAME).map
	rm -f $(PROJ_NAME).lst

reallyclean: clean
	$(MAKE) -C $(STD_PERIPH_LIB) clean
