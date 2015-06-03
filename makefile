PROJECT_NAME := ble_app_beacon_s110_pca20006

export OUTPUT_FILENAME
#MAKEFILE_NAME := $(CURDIR)/$(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST))
MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) ) 


SDK_PATH = ../beacon_sdk_v1.1.0/SourceCode/
COMPONENTS_PATH = $(SDK_PATH)components/
TEMPLATE_PATH = $(COMPONENTS_PATH)toolchain/gcc
COMMON_PATH = $(SDK_PATH)nrf51_beacon/common/
EXAMPLES_PATH = $(SDK_PATH)examples/

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
CC       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc"
AS       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as"
AR       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar" -r
LD       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld"
NM       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm"
OBJDUMP  		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump"
OBJCOPY  		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy"
SIZE    		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size"

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#source common to all targets
#/Users/shawn42/code/beacon/beacon_sdk_v1.1.0/Source Code/nrf51_beacon/common/led_softblink/led_softblink.c
C_SOURCE_FILES += \
main.c \
twi_hw_master.c \
$(COMMON_PATH)led_softblink/led_softblink.c \
$(EXAMPLES_PATH)bsp/bsp.c \
$(COMPONENTS_PATH)toolchain/system_nrf51.c \
$(COMPONENTS_PATH)ble/common/ble_advdata.c \
$(COMPONENTS_PATH)ble/common/ble_conn_params.c \
$(COMPONENTS_PATH)ble/common/ble_srv_common.c \
$(COMPONENTS_PATH)drivers_nrf/hal/nrf_delay.c \
$(COMPONENTS_PATH)drivers_nrf/pstorage/pstorage.c \
$(COMPONENTS_PATH)libraries/bootloader_dfu/bootloader_util_gcc.c \
$(COMMON_PATH)sdk_modified/dfu_app_handler/dfu_app_handler_mod.c \
$(COMPONENTS_PATH)libraries/button/app_button.c \
$(COMPONENTS_PATH)libraries/util/app_error.c \
$(COMPONENTS_PATH)libraries/fifo/app_fifo.c \
$(COMPONENTS_PATH)libraries/gpiote/app_gpiote.c \
$(COMPONENTS_PATH)libraries/scheduler/app_scheduler.c \
$(COMPONENTS_PATH)libraries/timer/app_timer.c \
$(COMPONENTS_PATH)libraries/util/nrf_assert.c \
$(COMPONENTS_PATH)softdevice/common/softdevice_handler/softdevice_handler.c \
$(COMMON_PATH)ble_bcs/ble_bcs.c \
$(COMPONENTS_PATH)ble/ble_services/ble_dfu/ble_dfu.c \
$(COMPONENTS_PATH)ble/ble_services/ble_bas/ble_bas.c \
$(COMPONENTS_PATH)ble/ble_services/ble_hrs/ble_hrs.c \
$(COMPONENTS_PATH)ble/ble_services/ble_dis/ble_dis.c \
$(COMPONENTS_PATH)ble/ble_error_log/ble_error_log.c \
$(COMPONENTS_PATH)ble/ble_debug_assert_handler/ble_debug_assert_handler.c \
$(COMPONENTS_PATH)ble/device_manager/device_manager_peripheral.c \
$(COMPONENTS_PATH)libraries/sensorsim/ble_sensorsim.c \
$(COMPONENTS_PATH)libraries/trace/app_trace.c \
../research/nrf51-ADC-examples/adc_example_with_softdevice_and_UART/adc_example_with_softdevice_and_UART/ble_nus.c

#assembly files common to all targets
ASM_SOURCE_FILES  = $(COMPONENTS_PATH)toolchain/gcc/gcc_startup_nrf51.s

#includes common to all targets
INC_PATHS  = -I./config
INC_PATHS += -I$(COMPONENTS_PATH)toolchain/gcc
INC_PATHS += -I$(COMPONENTS_PATH)toolchain
INC_PATHS += -I$(COMPONENTS_PATH)libraries/button
INC_PATHS += -I$(COMPONENTS_PATH)softdevice/s110/headers
INC_PATHS += -I$(COMPONENTS_PATH)ble/common
INC_PATHS += -I$(COMPONENTS_PATH)ble/ble_services/ble_bas
INC_PATHS += -I$(COMPONENTS_PATH)ble/ble_services/ble_hrs
INC_PATHS += -I$(COMPONENTS_PATH)ble/ble_services/ble_dis
INC_PATHS += -I$(COMPONENTS_PATH)ble/ble_error_log
INC_PATHS += -I$(COMPONENTS_PATH)ble/ble_debug_assert_handler
INC_PATHS += -I$(COMPONENTS_PATH)ble/device_manager
INC_PATHS += -I$(COMPONENTS_PATH)ble/device_manager/config
INC_PATHS += -I$(COMPONENTS_PATH)libraries/sensorsim
INC_PATHS += -I$(COMPONENTS_PATH)libraries/trace
INC_PATHS += -I$(EXAMPLES_PATH)bsp
INC_PATHS += -I$(COMPONENTS_PATH)drivers_nrf/ble_flash
INC_PATHS += -I$(COMPONENTS_PATH)libraries/fifo
INC_PATHS += -I$(COMPONENTS_PATH)libraries/timer
INC_PATHS += -I$(COMPONENTS_PATH)libraries/gpiote
INC_PATHS += -I$(COMPONENTS_PATH)drivers_nrf/hal
INC_PATHS += -I$(COMPONENTS_PATH)softdevice/common/softdevice_handler
INC_PATHS += -I$(COMPONENTS_PATH)libraries/scheduler
INC_PATHS += -I$(COMPONENTS_PATH)libraries/util
INC_PATHS += -I$(COMPONENTS_PATH)drivers_nrf/pstorage
INC_PATHS += -I$(COMPONENTS_PATH)drivers_nrf/twi_master
INC_PATHS += -I$(COMMON_PATH)beacon
INC_PATHS += -I$(COMMON_PATH)ble_bcs
INC_PATHS += -I$(COMMON_PATH)led_softblink
INC_PATHS += -I$(COMPONENTS_PATH)ble/ble_services/ble_dfu
INC_PATHS += -I$(COMMON_PATH)sdk_modified/dfu_app_handler
INC_PATHS += -I$(COMPONENTS_PATH)libraries/bootloader_dfu
INC_PATHS += -I../research/nrf51-ADC-examples/adc_example_with_softdevice_and_UART/adc_example_with_softdevice_and_UART

OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#flags common to all targets
CFLAGS  = -DNRF51
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DS110
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DBOARD_PCA20006
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -Werror -g
CFLAGS += -mfloat-abi=soft
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -flto -fno-builtin

# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
LDFLAGS += -g
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys

# Assembler flags
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DNRF51
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DS110
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DBOARD_PCA20006
#default target - first one defined
default: clean nrf51822_xxaa_s110

#building all targets
all: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e nrf51822_xxaa_s110 

#target for printing all targets
help:
	@echo following targets are available:
	@echo 	nrf51822_xxaa_s110


C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

nrf51822_xxaa_s110: OUTPUT_FILENAME := nrf51822_xxaa_s110
nrf51822_xxaa_s110: LINKER_SCRIPT=$(COMPONENTS_PATH)toolchain/gcc/gcc_nrf51_s110_xxaa.ld
nrf51822_xxaa_s110: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
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
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<


# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out


## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize: genbin genhex echosize

genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
genhex: 
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

echosize:
	-@echo ""
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ""

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o

flash: $(MAKECMDGOALS)
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$<.hex
	nrfjprog --reset --program $(OUTPUT_BINARY_DIRECTORY)/$<.hex
