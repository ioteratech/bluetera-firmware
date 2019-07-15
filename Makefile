# Global defines
PROJECT_NAME     		:= bluetera
TARGETS          		:= bluetera
OUTPUT_DIRECTORY 		:= _build
INVN_LIB_NAME 			:= imu_driver

# Be careful when changing the DFU_APP_VERSION variable!
# If you upload a file with a version > 1 the bootloader will not let you
# perform OTA updates with packages generated with a lower version number!
# (all the packages available from the GitHub releases page will have version 1)
DFU_APP_VERSION			:= 1

SCM_COMMIT_HASH 		:= $(shell git rev-list --max-count=1 HEAD | cut -c1-8)

# Commands
MV := mv -f

# Directories
SDK_ROOT := $(NRF_SDK_ROOT)/nRF5_SDK_15.2.0_9412b96
PROJ_DIR := .
APP_DIR := $(PROJ_DIR)/application
EXTERNAL_DIR := $(PROJ_DIR)/external

# Hardware
BLUETERA_BOARD := BLUETERA_BOARD_V1
NRF_CHIP := NRF52832

# Invensense ICM20649 library
SRC_INVENSENSE = $(wildcard $(EXTERNAL_DIR)/invn/*.c) \
  $(wildcard $(EXTERNAL_DIR)/invn/Devices/*.c) \
  $(wildcard $(EXTERNAL_DIR)/invn/Devices/Drivers/Icm20649/*.c) \
  $(wildcard $(EXTERNAL_DIR)/invn/EmbUtils/*.c)
  
INC_INVENSENSE = $(EXTERNAL_DIR) \
  $(EXTERNAL_DIR)/invn 
  
# print variable
# $(info $$SRC_INVENSENSE is [${SRC_INVENSENSE}])

$(OUTPUT_DIRECTORY)/bluetera.out: \
  LINKER_SCRIPT  := bluetera_$(NRF_CHIP).ld

# Source files common to all targets
SRC_FILES += \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_rtt.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_serial.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_uart.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_default_backends.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_frontend.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_str_formatter.c \
  $(SDK_ROOT)/components/libraries/util/app_error.c \
  $(SDK_ROOT)/components/libraries/util/app_error_handler_gcc.c \
  $(SDK_ROOT)/components/libraries/util/app_error_weak.c \
  $(SDK_ROOT)/components/libraries/scheduler/app_scheduler.c \
  $(SDK_ROOT)/components/libraries/timer/app_timer.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/crc16/crc16.c \
  $(SDK_ROOT)/components/libraries/fds/fds.c \
  $(SDK_ROOT)/components/libraries/hardfault/hardfault_implementation.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(SDK_ROOT)/components/libraries/atomic_fifo/nrf_atfifo.c \
  $(SDK_ROOT)/components/libraries/atomic_flags/nrf_atflags.c \
  $(SDK_ROOT)/components/libraries/atomic/nrf_atomic.c \
  $(SDK_ROOT)/components/libraries/balloc/nrf_balloc.c \
  $(SDK_ROOT)/components/libraries/fstorage/nrf_fstorage.c \
  $(SDK_ROOT)/components/libraries/fstorage/nrf_fstorage_sd.c \
  $(SDK_ROOT)/components/libraries/memobj/nrf_memobj.c \
  $(SDK_ROOT)/components/libraries/pwr_mgmt/nrf_pwr_mgmt.c \
  $(SDK_ROOT)/components/libraries/experimental_section_vars/nrf_section_iter.c \
  $(SDK_ROOT)/components/libraries/strerror/nrf_strerror.c \
  $(SDK_ROOT)/components/libraries/ringbuf/nrf_ringbuf.c \
  $(SDK_ROOT)/components/libraries/bootloader/dfu/nrf_dfu_svci.c \
  $(SDK_ROOT)/components/ble/common/ble_advdata.c \
  $(SDK_ROOT)/components/ble/ble_advertising/ble_advertising.c \
  $(SDK_ROOT)/components/ble/common/ble_conn_params.c \
  $(SDK_ROOT)/components/ble/common/ble_conn_state.c \
  $(SDK_ROOT)/components/ble/common/ble_srv_common.c \
  $(SDK_ROOT)/components/ble/peer_manager/gatt_cache_manager.c \
  $(SDK_ROOT)/components/ble/peer_manager/gatts_cache_manager.c \
  $(SDK_ROOT)/components/ble/peer_manager/id_manager.c \
  $(SDK_ROOT)/components/ble/nrf_ble_gatt/nrf_ble_gatt.c \
  $(SDK_ROOT)/components/ble/nrf_ble_qwr/nrf_ble_qwr.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_data_storage.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_database.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_id.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_manager.c \
  $(SDK_ROOT)/components/ble/peer_manager/pm_buffer.c \
  $(SDK_ROOT)/components/ble/peer_manager/security_dispatcher.c \
  $(SDK_ROOT)/components/ble/peer_manager/security_manager.c \
  $(SDK_ROOT)/components/ble/ble_link_ctx_manager/ble_link_ctx_manager.c \
  $(SDK_ROOT)/components/ble/ble_services/ble_dis/ble_dis.c \
  $(SDK_ROOT)/components/ble/ble_services/ble_dfu/ble_dfu.c \
  $(SDK_ROOT)/components/ble/ble_services/ble_dfu/ble_dfu_bonded.c \
  $(SDK_ROOT)/components/ble/ble_services/ble_dfu/ble_dfu_unbonded.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_ble.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_soc.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_spi.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_clock.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_gpiote.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_power_clock.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/prs/nrfx_prs.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_uart.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_spim.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_timer.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_Syscalls_GCC.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_printf.c \
  $(SDK_ROOT)/external/fprintf/nrf_fprintf.c \
  $(SDK_ROOT)/external/fprintf/nrf_fprintf_format.c \
  $(SDK_ROOT)/external/nano-pb/pb_common.c \
  $(SDK_ROOT)/external/nano-pb/pb_decode.c \
  $(SDK_ROOT)/external/nano-pb/pb_encode.c \
  $(APP_DIR)/utilities/utils.c \
  $(APP_DIR)/messages/bluetera_messages.pb.c \
  $(APP_DIR)/messages/bluetera_messages.c \
  $(APP_DIR)/services/bus/ble_bus.c \
  $(APP_DIR)/modules/imu/imu_manager.c \
  $(APP_DIR)/main.c

# Include folders common to all targets
INC_FOLDERS += \
  $(SDK_ROOT)/components \
  $(SDK_ROOT)/components/libraries/pwm \
  $(SDK_ROOT)/components/libraries/log \
  $(SDK_ROOT)/components/libraries/fstorage \
  $(SDK_ROOT)/components/libraries/mutex \
  $(SDK_ROOT)/components/libraries/gpiote \
  $(SDK_ROOT)/components/libraries/log/src \
  $(SDK_ROOT)/components/libraries/memobj \
  $(SDK_ROOT)/components/libraries/experimental_task_manager \
  $(SDK_ROOT)/components/libraries/queue \
  $(SDK_ROOT)/components/libraries/pwr_mgmt \
  $(SDK_ROOT)/components/libraries/slip \
  $(SDK_ROOT)/components/libraries/delay \
  $(SDK_ROOT)/components/libraries/mpu \
  $(SDK_ROOT)/components/libraries/mem_manager \
  $(SDK_ROOT)/components/libraries/experimental_section_vars \
  $(SDK_ROOT)/components/libraries/low_power_pwm \
  $(SDK_ROOT)/components/libraries/scheduler \
  $(SDK_ROOT)/components/libraries/cli \
  $(SDK_ROOT)/components/libraries/crc16 \
  $(SDK_ROOT)/components/libraries/util \
  $(SDK_ROOT)/components/libraries/csense \
  $(SDK_ROOT)/components/libraries/balloc \
  $(SDK_ROOT)/components/libraries/ecc \
  $(SDK_ROOT)/components/libraries/hardfault \
  $(SDK_ROOT)/components/libraries/hci \
  $(SDK_ROOT)/components/libraries/timer \
  $(SDK_ROOT)/components/libraries/atomic \
  $(SDK_ROOT)/components/libraries/sortlist \
  $(SDK_ROOT)/components/libraries/stack_guard \
  $(SDK_ROOT)/components/libraries/strerror \
  $(SDK_ROOT)/components/libraries/crc32 \
  $(SDK_ROOT)/components/libraries/atomic_fifo \
  $(SDK_ROOT)/components/libraries/ringbuf \
  $(SDK_ROOT)/components/libraries/crypto \
  $(SDK_ROOT)/components/libraries/fds \
  $(SDK_ROOT)/components/libraries/atomic_flags \
  $(SDK_ROOT)/components/libraries/queue \
  $(SDK_ROOT)/components/libraries/bootloader/dfu \
  $(SDK_ROOT)/components/libraries/bootloader \
  $(SDK_ROOT)/components/libraries/bootloader/ble_dfu \
  $(SDK_ROOT)/components/libraries/svc \
  $(SDK_ROOT)/components/ble/ble_advertising \
  $(SDK_ROOT)/components/ble/ble_dtm \
  $(SDK_ROOT)/components/ble/common \
  $(SDK_ROOT)/components/ble/peer_manager \
  $(SDK_ROOT)/components/ble/nrf_ble_gatt \
  $(SDK_ROOT)/components/ble/nrf_ble_qwr \
  $(SDK_ROOT)/components/ble/ble_racp \
  $(SDK_ROOT)/components/ble/ble_link_ctx_manager \
  $(SDK_ROOT)/components/ble/ble_services/ble_dis \
  $(SDK_ROOT)/components/ble/ble_services/ble_dfu \
  $(SDK_ROOT)/components/softdevice/common \
  $(SDK_ROOT)/components/toolchain/cmsis/include \
  $(SDK_ROOT)/modules/nrfx/hal \
  $(SDK_ROOT)/modules/nrfx/drivers/include \
  $(SDK_ROOT)/modules/nrfx/mdk \
  $(SDK_ROOT)/modules/nrfx \
  $(SDK_ROOT)/integration/nrfx \
  $(SDK_ROOT)/integration/nrfx/legacy \
  $(SDK_ROOT)/external/fprintf \
  $(SDK_ROOT)/external/segger_rtt \
  $(SDK_ROOT)/external/nano-pb \
  $(APP_DIR) \
  $(APP_DIR)/config \
  $(APP_DIR)/utilities \
  $(APP_DIR)/services/bus \
  $(APP_DIR)/messages \
  $(APP_DIR)/boards \
  $(APP_DIR)/modules/imu \
  $(EXTERNAL_DIR)/invn
  

# Libraries common to all targets
LIB_FILES += $(EXTERNAL_DIR)/invn/$(INVN_LIB_NAME).a \

# Optimization flags
OPT = -O3 -g3
#OPT = -O0 -g
# Uncomment the line below to enable link time optimization
#OPT += -flto

# C flags common to all targets
CFLAGS += $(OPT)
CFLAGS += -D$(BLUETERA_BOARD)
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DCONFIG_NFCT_PINS_AS_GPIOS
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DFLOAT_ABI_HARD
CFLAGS += -DNRF_SD_BLE_API_VERSION=6
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DSWI_DISABLE0
CFLAGS += -DNRF_DFU_SVCI_ENABLED
CFLAGS += -DNRF_DFU_TRANSPORT_BLE=1
CFLAGS += -DSCM_COMMIT_HASH=\"$(SCM_COMMIT_HASH)\"
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in a separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin -fshort-enums
#CFLAGS += -DDEBUG
#CFLAGS += -Wall -Werror
CFLAGS += -Wall

# C++ flags common to all targets
CXXFLAGS += $(OPT)

# Assembler flags common to all targets
ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DFLOAT_ABI_HARD
ASMFLAGS += -DNRF_SD_BLE_API_VERSION=6
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -DNRF_DFU_SVCI_ENABLED
ASMFLAGS += -DNRF_DFU_TRANSPORT_BLE=1

# Linker flags
LDFLAGS += $(OPT)
LDFLAGS += -mthumb -mabi=aapcs -L$(SDK_ROOT)/modules/nrfx/mdk -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs

bluetera: CFLAGS += -D__HEAP_SIZE=8192
bluetera: CFLAGS += -D__STACK_SIZE=8192
bluetera: ASMFLAGS += -D__HEAP_SIZE=8192
bluetera: ASMFLAGS += -D__STACK_SIZE=8192

# chip specific
ifeq ($(NRF_CHIP), NRF52832)

SRC_FILES += $(SDK_ROOT)/modules/nrfx/mdk/system_nrf52.c \
	$(SDK_ROOT)/modules/nrfx/mdk/gcc_startup_nrf52.S

INC_FOLDERS += $(SDK_ROOT)/components/softdevice/s132/headers/nrf52 \
	$(SDK_ROOT)/components/softdevice/s132/headers

CFLAGS += -DNRF52
CFLAGS += -DNRF52832_XXAA
CFLAGS += -DNRF52_PAN_74
CFLAGS += -DS132

ASMFLAGS += -DNRF52
ASMFLAGS += -DNRF52832_XXAA
ASMFLAGS += -DNRF52_PAN_74
ASMFLAGS += -DS132

SOFTDEVICE_HEX := s132/hex/s132_nrf52_6.1.0_softdevice.hex
NRF_FAMILY := NRF52
DFU_SD_REQ := 0xAF

else

SRC_FILES += $(SDK_ROOT)/modules/nrfx/mdk/system_nrf52840.c \
	$(SDK_ROOT)/modules/nrfx/mdk/gcc_startup_nrf52840.S

INC_FOLDERS += $(SDK_ROOT)/components/softdevice/s140/headers/nrf52 \
  	$(SDK_ROOT)/components/softdevice/s140/headers

CFLAGS += -DNRF52840_XXAA
CFLAGS += -DNRF_CRYPTO_MAX_INSTANCE_COUNT=1
CFLAGS += -DS140

CFLAGS += -DuECC_ENABLE_VLI_API=0
CFLAGS += -DuECC_OPTIMIZATION_LEVEL=3
CFLAGS += -DuECC_SQUARE_FUNC=0
CFLAGS += -DuECC_SUPPORT_COMPRESSED_POINT=0
CFLAGS += -DuECC_VLI_NATIVE_LITTLE_ENDIAN=1

ASMFLAGS += -DNRF52840_XXAA
ASMFLAGS += -DNRF_CRYPTO_MAX_INSTANCE_COUNT=1
ASMFLAGS += -DS140
ASMFLAGS += -DuECC_ENABLE_VLI_API=0
ASMFLAGS += -DuECC_OPTIMIZATION_LEVEL=3
ASMFLAGS += -DuECC_SQUARE_FUNC=0
ASMFLAGS += -DuECC_SUPPORT_COMPRESSED_POINT=0
ASMFLAGS += -DuECC_VLI_NATIVE_LITTLE_ENDIAN=1

SOFTDEVICE_HEX := s140/hex/s140_nrf52_6.1.0_softdevice.hex
NRF_FAMILY := NRF52840
DFU_SD_REQ := 0xAE

endif

# Add standard libraries at the very end of the linker input, after all objects
# that may need symbols provided by these libraries.
LIB_FILES += -lc -lnosys -lm


.PHONY: default help

# Default target
# We only build invensense library if the library's sources are present
ifneq ("$(wildcard $(EXTERNAL_DIR)/invn/IDDVersion.h)","")
default: build_invn_lib bluetera generate_dfu_settings
else
default: bluetera generate_dfu_settings
endif

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo		bluetera (default)
	@echo		flash_softdevice - flash soft device
	@echo		flash - flashing binary
	@echo		build_invn_lib - build invensense library
	@echo 		gen_commit_hash - re-generate commit hash file

TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc


include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

.PHONY: flash flash_softdevice erase

# Generates the setting page for the bootloader. Needed when updating firmware via SWD,
# so the bootloader will have the new checksum and version
generate_dfu_settings: $(OUTPUT_DIRECTORY)/bluetera.hex
	@echo Generating bootloader setting page for firmware: $<
	nrfutil settings generate --family $(NRF_FAMILY) --application $< --application-version $(DFU_APP_VERSION) --bootloader-version 1 --bl-settings-version 1 _build/bootloader_settings.hex

# Flash the program
flash: $(OUTPUT_DIRECTORY)/bluetera.hex
	@echo Flashing: $<
	nrfjprog --program $< --sectorerase
	nrfjprog --program _build/bootloader_settings.hex --sectoranduicrerase
	nrfjprog -r

# Flash softdevice
flash_softdevice:
	@echo Flashing: $(SOFTDEVICE_HEX)
	nrfjprog -f nrf52 --program $(SDK_ROOT)/components/softdevice/$(SOFTDEVICE_HEX) --sectorerase
	nrfjprog -f nrf52 --reset

INVN_LIB_OBJ_LOC = $(addprefix $(OUTPUT_DIRECTORY)/imu_driver/, $(addsuffix .o, $(basename $(notdir $(SRC_INVENSENSE)))))

# Create a DFU package
generate_package: $(OUTPUT_DIRECTORY)/bluetera.hex
	@echo Generating DFU package
	nrfutil pkg generate --hw-version 52 --application-version $(DFU_APP_VERSION) --application $< --sd-req $(DFU_SD_REQ) --key-file bootloader\key\private_key.pem bluetera_dfu_package_$(SCM_COMMIT_HASH).zip

# Build Invensense library. Requires GNU tools
build_invn_lib:
	-$(MK) "$(OUTPUT_DIRECTORY)"
	-$(MK) "$(OUTPUT_DIRECTORY)/$(INVN_LIB_NAME)"
	$(CC) -c $(SRC_INVENSENSE) -I$(EXTERNAL_DIR) -I$(APP_DIR)/modules/imu -I$(APP_DIR)/utilities $(CFLAGS)
	$(MV) *.o "$(OUTPUT_DIRECTORY)/$(INVN_LIB_NAME)"
	"$(GNU_INSTALL_ROOT)$(GNU_PREFIX)-ar" rcs "$(OUTPUT_DIRECTORY)/$(INVN_LIB_NAME).a" $(INVN_LIB_OBJ_LOC)
	$(MV) "$(OUTPUT_DIRECTORY)/$(INVN_LIB_NAME).a" "$(EXTERNAL_DIR)/invn"

erase:
	nrfjprog -f nrf52 --eraseall

SDK_CONFIG_FILE := $(APP_DIR)/config/sdk_config.h
CMSIS_CONFIG_TOOL := $(SDK_ROOT)/external_tools/cmsisconfig/CMSIS_Configuration_Wizard.jar

sdk_config:
	java -jar $(CMSIS_CONFIG_TOOL) $(SDK_CONFIG_FILE)

.PHONY: get_commit_hash
