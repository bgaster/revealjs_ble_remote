PROJECT_NAME := hrs

export OUTPUT_FILENAME
#MAKEFILE_NAME := $(CURDIR)/$(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST))
MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) )

TEMPLATE_PATH = $(NORDIC_SDK_PATH)/components/toolchain/gcc
ifeq ($(OS),Windows_NT)
include $(TEMPLATE_PATH)/Makefile.windows
else
include $(TEMPLATE_PATH)/Makefile.posix
endif

MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO :=
else
NO_ECHO := @
endif

# Toolchain commands
CC              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc'
AS              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as'
AR              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar' -r
LD              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld'
NM              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm'
OBJDUMP         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump'
OBJCOPY         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy'
SIZE            := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size'

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

# PATHs for mergehex and softdevice
MERGEHEX = $(NORDIC_SDK_PATH)/../mergehex/mergehex
S130_SOFTDEVICE = $(NORDIC_SDK_PATH)/components/softdevice/s130/hex/s130_nrf51_2.0.0_softdevice.hex

# Path for MK20 mount for MBED install, Red Bear Nano setup
MK20_PATH = /Volumes/MBED

#source common to all targets
C_SOURCE_FILES += \
$(NORDIC_SDK_PATH)/components/libraries/button/app_button.c \
$(NORDIC_SDK_PATH)components/libraries/util/app_error.c \
$(NORDIC_SDK_PATH)/components/libraries/util/app_error_weak.c \
$(NORDIC_SDK_PATH)/components/libraries/timer/app_timer_freertos.c \
$(NORDIC_SDK_PATH)/components/libraries/trace/app_trace.c \
$(NORDIC_SDK_PATH)/components/libraries/util/app_util_platform.c \
$(NORDIC_SDK_PATH)/components/libraries/fstorage/fstorage.c \
$(NORDIC_SDK_PATH)/components/libraries/util/nrf_assert.c \
$(NORDIC_SDK_PATH)components/libraries/util/nrf_log.c \
$(NORDIC_SDK_PATH)/components/libraries/uart/retarget.c \
$(NORDIC_SDK_PATH)/components/libraries/sensorsim/sensorsim.c \
$(NORDIC_SDK_PATH)/external/segger_rtt/RTT_Syscalls_GCC.c \
$(NORDIC_SDK_PATH)/external/segger_rtt/SEGGER_RTT.c \
$(NORDIC_SDK_PATH)/external/segger_rtt/SEGGER_RTT_printf.c \
$(NORDIC_SDK_PATH)/external/freertos/source/croutine.c \
$(NORDIC_SDK_PATH)/external/freertos/source/event_groups.c \
$(NORDIC_SDK_PATH)/external/freertos/source/portable/MemMang/heap_1.c \
$(NORDIC_SDK_PATH)/external/freertos/source/list.c \
$(NORDIC_SDK_PATH)/external/freertos/portable/GCC/nrf51/port.c \
$(NORDIC_SDK_PATH)/external/freertos/portable/CMSIS/nrf51/port_cmsis.c \
$(NORDIC_SDK_PATH)/external/freertos/portable/CMSIS/nrf51/port_cmsis_systick.c \
$(NORDIC_SDK_PATH)/external/freertos/source/queue.c \
$(NORDIC_SDK_PATH)/external/freertos/source/tasks.c \
$(NORDIC_SDK_PATH)/external/freertos/source/timers.c \
$(NORDIC_SDK_PATH)/components/libraries/uart/app_uart.c \
$(NORDIC_SDK_PATH)/components/drivers_nrf/delay/nrf_delay.c \
$(NORDIC_SDK_PATH)/components/drivers_nrf/clock/nrf_drv_clock.c \
$(NORDIC_SDK_PATH)/components/drivers_nrf/common/nrf_drv_common.c \
$(NORDIC_SDK_PATH)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c \
$(NORDIC_SDK_PATH)/components/drivers_nrf/uart/nrf_drv_uart.c \
$(NORDIC_SDK_PATH)/components/drivers_nrf/pstorage/pstorage.c \
$(NORDIC_SDK_PATH)/examples/bsp/bsp.c \
$(NORDIC_SDK_PATH)/examples/bsp/bsp_btn_ble.c \
./main.c \
./ble_controller.c \
$(NORDIC_SDK_PATH)/components/ble/common/ble_advdata.c \
$(NORDIC_SDK_PATH)/components/ble/ble_advertising/ble_advertising.c \
$(NORDIC_SDK_PATH)/components/ble/ble_services/ble_bas/ble_bas.c \
$(NORDIC_SDK_PATH)/components/ble/common/ble_conn_params.c \
$(NORDIC_SDK_PATH)/components/ble/ble_services/ble_dis/ble_dis.c \
$(NORDIC_SDK_PATH)/components/ble/ble_services/ble_hrs/ble_hrs.c \
$(NORDIC_SDK_PATH)/components/ble/common/ble_srv_common.c \
$(NORDIC_SDK_PATH)/components/ble/device_manager/device_manager_peripheral.c \
$(NORDIC_SDK_PATH)/components/toolchain/system_nrf51.c \
$(NORDIC_SDK_PATH)/components/softdevice/common/softdevice_handler/softdevice_handler.c \

#assembly files common to all targets
ASM_SOURCE_FILES  = $(NORDIC_SDK_PATH)/components/toolchain/gcc/gcc_startup_nrf51.s

#includes common to all targets

INC_PATHS += -I./
INC_PATHS += -I./config
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/drivers_nrf/config
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/libraries/timer
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/libraries/fstorage/config
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/softdevice/s130/headers
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/drivers_nrf/delay
INC_PATHS += -I$(NORDIC_SDK_PATH)/external/freertos/source/include
INC_PATHS += -I$(NORDIC_SDK_PATH)/external/freertos/config
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/libraries/util
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/ble/device_manager
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/drivers_nrf/uart
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/ble/common
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/libraries/sensorsim
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/drivers_nrf/pstorage
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/ble/ble_services/ble_dis
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/device
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/libraries/uart
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/libraries/button
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/libraries/fstorage
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/libraries/experimental_section_vars
INC_PATHS += -I$(NORDIC_SDK_PATH)/external/freertos/portable/CMSIS/nrf51
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/drivers_nrf/gpiote
INC_PATHS += -I$(NORDIC_SDK_PATH)/external/segger_rtt
INC_PATHS += -I$(NORDIC_SDK_PATH)/examples/bsp
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/toolchain/CMSIS/Include
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/drivers_nrf/hal
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/toolchain/gcc
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/toolchain
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/drivers_nrf/common
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/ble/ble_advertising
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/drivers_nrf/clock
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/softdevice/s130/headers/nrf51
INC_PATHS += -I$(NORDIC_SDK_PATH)/external/freertos/portable/GCC/nrf51
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/libraries/trace
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/ble/ble_services/ble_bas
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/softdevice/common/softdevice_handler
INC_PATHS += -I$(NORDIC_SDK_PATH)/components/ble/ble_services/ble_hrs

OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#flags common to all targets
CFLAGS  = -D__STACK_SIZE=2048
CFLAGS += -DNRF51
CFLAGS += -D__HEAP_SIZE=1024
CFLAGS += -DBOARD_CUSTOM
CFLAGS += -DNRF_LOG_USES_UART=1
CFLAGS += -DFREERTOS
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DS130
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -O3 -g3
CFLAGS += -mfloat-abi=soft
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums
# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys

# Assembler flags
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -D__STACK_SIZE=2048
ASMFLAGS += -DNRF51
ASMFLAGS += -D__HEAP_SIZE=1024
ASMFLAGS += -DBOARD_PCA10028
ASMFLAGS += -DNRF_LOG_USES_UART=1
ASMFLAGS += -DFREERTOS
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DS130

#default target - first one defined
default: clean hrs_s130

#building all targets
all: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e hrs_s130

#target for printing all targets
help:
	@echo following targets are available:
	@echo 	hrs_s130
	@echo   flash_mk20

C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

hrs_s130: OUTPUT_FILENAME := hrs_s130
hrs_s130: LINKER_SCRIPT=ble_app_hrs_freertos_gcc_nrf51.ld

hrs_s130: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -lm -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Assembly file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<
# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -lm -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize: genhex mergehex

genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out fileclear
genhex:
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

## For Bear Lab's MK20, no softdevice on board so we need to include it!
mergehex: $(OUTPUT_BINARY_DIRECTORY)/hrs_s130.hex
	@echo Merging s130 softdevice with $<
	$(MERGEHEX) --merge $(S130_SOFTDEVICE) $< --output $(OUTPUT_BINARY_DIRECTORY)/hrs_s130_merged.hex

flash_mk20: hrs_s130
	@echo Flashing to MK20
	cp $(OUTPUT_BINARY_DIRECTORY)/hrs_s130_merged.hex $(MK20_PATH)/

echosize:
	-@echo ''
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ''

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o

flash: hrs_s130
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$<.hex
	nrfjprog --program $(OUTPUT_BINARY_DIRECTORY)/$<.hex -f nrf51  --sectorerase
	nrfjprog --reset -f nrf51

## Flash softdevice
flash_softdevice:
	@echo Flashing: s130_nrf51_2.0.0_softdevice.hex
	nrfjprog --program ../../../../../../components/softdevice/s130/hex/s130_nrf51_2.0.0_softdevice.hex -f nrf51 --chiperase
	nrfjprog --reset -f nrf51
