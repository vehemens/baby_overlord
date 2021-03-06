TARGET=main
EXECUTABLE=main.elf

CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump

BIN=$(CP) -O ihex 

# Select the appropriate option for your device, the available options are listed below
# with a description copied from stm32f10x.h
# Make sure to set the startup code file to the right device family, too!
#
# STM32F10X_LD 		STM32F10X_LD: STM32 Low density devices
# STM32F10X_LD_VL	STM32F10X_LD_VL: STM32 Low density Value Line devices
# STM32F10X_MD		STM32F10X_MD: STM32 Medium density devices
# STM32F10X_MD_VL	STM32F10X_MD_VL: STM32 Medium density Value Line devices 
# STM32F10X_HD		STM32F10X_HD: STM32 High density devices
# STM32F10X_HD_VL	STM32F10X_HD_VL: STM32 High density value line devices
# STM32F10X_XL		STM32F10X_XL: STM32 XL-density devices
# STM32F10X_CL		STM32F10X_CL: STM32 Connectivity line devices 
#
# - Low-density devices are STM32F101xx, STM32F102xx and STM32F103xx microcontrollers
#   where the Flash memory density ranges between 16 and 32 Kbytes.
# 
# - Low-density value line devices are STM32F100xx microcontrollers where the Flash
#   memory density ranges between 16 and 32 Kbytes.
# 
# - Medium-density devices are STM32F101xx, STM32F102xx and STM32F103xx microcontrollers
#   where the Flash memory density ranges between 64 and 128 Kbytes.
# 
# - Medium-density value line devices are STM32F100xx microcontrollers where the 
#   Flash memory density ranges between 64 and 128 Kbytes.   
# 
# - High-density devices are STM32F101xx and STM32F103xx microcontrollers where
#   the Flash memory density ranges between 256 and 512 Kbytes.
# 
# - High-density value line devices are STM32F100xx microcontrollers where the 
#   Flash memory density ranges between 256 and 512 Kbytes.   
# 
# - XL-density devices are STM32F101xx and STM32F103xx microcontrollers where
#   the Flash memory density ranges between 512 and 1024 Kbytes.
# 
# - Connectivity line devices are STM32F105xx and STM32F107xx microcontrollers.
#
# HSE_VALUE sets the value of the HSE clock, 8MHz in this case 

DEFS = -DUSE_STDPERIPH_DRIVER -DSTM32F10X_MD -DHSE_VALUE=8000000 -DUSE_DXL_USART3

STARTUP = ../../Libraries/CMSIS/Device/ST/STM32F10x/Source/Templates/gcc_ride7/startup_stm32f10x_md.s

MCU = cortex-m3
MCFLAGS = -mcpu=$(MCU) -mthumb -mlittle-endian -mthumb-interwork

STM32_INCLUDES = -Iinc/ -I../../Libraries/CMSIS/Device/ST/STM32F10x/Include/ \
	-I../../Libraries/CMSIS/Include/ \
	-I../../Libraries/STM32F10x_StdPeriph_Driver/inc/ \
	-I../../Libraries/STM32_USB-FS-Device_Driver/inc/ \
	-I../../Libraries/STM32_USB-FS-Device_Driver/inc/

OPTIMIZE       = -Os

CFLAGS	= $(MCFLAGS)  $(OPTIMIZE)  $(DEFS) -I. -I./ $(STM32_INCLUDES)  -Wl,-T,stm32_flash.ld
AFLAGS	= $(MCFLAGS) 

SRC = src/button.c \
	src/CM_DXL_COM.c \
	src/gyro_acc.c \
	src/hw_config.c \
	src/led.c \
	src/main.c \
	src/stm32_it.c \
	src/usb_desc.c \
	src/usb_endp.c \
	src/usb_istr.c \
	src/usb_prop.c \
	src/usb_pwr.c \
	../../Libraries/CMSIS/Device/ST/STM32F10x/Source/Templates/system_stm32f10x.c \
	../../Libraries/STM32F10x_StdPeriph_Driver/src/misc.c \
	../../Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_adc.c \
	../../Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_bkp.c \
	../../Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_can.c \
	../../Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_cec.c \
	../../Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_crc.c \
	../../Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_dac.c \
	../../Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_dbgmcu.c \
	../../Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_dma.c \
	../../Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_exti.c \
	../../Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_flash.c \
	../../Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_fsmc.c \
	../../Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c \
	../../Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_i2c.c \
	../../Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_iwdg.c \
	../../Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_pwr.c \
	../../Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c \
	../../Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_rtc.c \
	../../Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_sdio.c \
	../../Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_spi.c \
	../../Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_tim.c \
	../../Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_usart.c \
	../../Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_wwdg.c \
	../../Libraries/STM32_USB-FS-Device_Driver/src/usb_core.c \
	../../Libraries/STM32_USB-FS-Device_Driver/src/usb_init.c \
	../../Libraries/STM32_USB-FS-Device_Driver/src/usb_int.c \
	../../Libraries/STM32_USB-FS-Device_Driver/src/usb_mem.c \
	../../Libraries/STM32_USB-FS-Device_Driver/src/usb_regs.c \
	../../Libraries/STM32_USB-FS-Device_Driver/src/usb_sil.c

OBJDIR = .
OBJ = $(SRC:%.c=$(OBJDIR)/%.o) 
OBJ += Startup.o

all: $(TARGET).hex

$(TARGET).hex: $(EXECUTABLE)
	$(CP) -O ihex $^ $@

$(EXECUTABLE): $(SRC) $(STARTUP)
	$(CC) $(CFLAGS) $^ -lm -lc -lnosys  -o $@

clean:
	rm -f Startup.lst $(TARGET) $(TARGET).lst $(OBJ) $(AUTOGEN) $(TARGET).out $(TARGET).hex  $(TARGET).map \
	 $(TARGET).dmp  $(EXECUTABLE)
