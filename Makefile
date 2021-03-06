####################################################################################################
# Makefile generated by updateMakefile.py
# Version 1.7 
# Last run: 2021-03-19 23:20:18.649836 
####################################################################################################

######################################
# target
######################################
TARGET = term_vent_code_git


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -O2


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Drivers/board/button.c \
Drivers/board/menu.c \
Drivers/board/peripherals.c \
Drivers/board/pinmap.c \
Drivers/board/ventilator.c \
Drivers/board/mode-auto.c \
Drivers/module/Src/BME280.c \
Drivers/module/Src/display_buffer.c \
Drivers/module/Src/lcd_12864.c \
Drivers/module/Src/one_wire.c \
Drivers/module/Src/rtc_api.c \
Drivers/module/Src/spi_com.c \
Drivers/module/Src/usb_api.c \
Drivers/module/Src/debounce.c \
Drivers/module/Src/ds18b20.c \
Drivers/module/Src/log.c \
Drivers/module/Src/pwm.c \
Drivers/module/Src/sleep.c \
Drivers/module/Src/Time.c \
Drivers/module/Src/waiter.c \
Src/main.c \
Src/gpio.c \
Src/adc.c \
Src/dma.c \
Src/rtc.c \
Src/spi.c \
Src/tim.c \
Src/usart.c \
Src/usb_device.c \
Src/usbd_conf.c \
Src/usbd_desc.c \
Src/usbd_cdc_if.c \
Src/stm32l0xx_it.c \
Src/stm32l0xx_hal_msp.c \
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_pcd.c \
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_pcd_ex.c \
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_usb.c \
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal.c \
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_i2c.c \
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_i2c_ex.c \
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rcc.c \
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rcc_ex.c \
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_flash_ramfunc.c \
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_flash.c \
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_flash_ex.c \
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_gpio.c \
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_dma.c \
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_pwr.c \
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_pwr_ex.c \
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_cortex.c \
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_exti.c \
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_adc.c \
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_adc_ex.c \
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rtc.c \
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rtc_ex.c \
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_spi.c \
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_tim.c \
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_tim_ex.c \
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_uart.c \
Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_uart_ex.c \
Src/system_stm32l0xx.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c  

# ASM sources
ASM_SOURCES =  \
startup_stm32l072xx.s


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m0plus

# fpu
# NONE for Cortex-M0/M0+/M3

# float-abi


# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32L072xx


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-IDrivers/board \
-IDrivers/module/Inc \
-IInc \
-IDrivers/STM32L0xx_HAL_Driver/Inc \
-IDrivers/STM32L0xx_HAL_Driver/Inc/Legacy \
-IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc \
-IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc \
-IDrivers/CMSIS/Device/ST/STM32L0xx/Include \
-IDrivers/CMSIS/Include \
-IDrivers/CMSIS/Include


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32L072RBTx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***

#######################################
# Print makefile variables
#######################################
print-%:
	@echo VARIABLE=$($*)
