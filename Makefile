## Copyright (c) 2016, Devan Lai
##
## Permission to use, copy, modify, and/or distribute this software
## for any purpose with or without fee is hereby granted, provided
## that the above copyright notice and this permission notice
## appear in all copies.
##
## THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
## WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
## WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
## AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR
## CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
## LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
## NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
## CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q			  := @
NULL		   := 2>/dev/null
MAKE		   := $(MAKE) --no-print-directory
endif
export V

BUILD_DIR	  ?= ./build

SIZE           = arm-none-eabi-size

all: UMDK-EMB UMDK-RF DAP42 DAP42DC KITCHEN42 \
	 DAP103 DAP103-DFU \
	 DAP103-NUCLEO-STBOOT \
	 BRAINv3.3 \
	 DAP42K6U
clean:
	$(Q)$(RM) $(BUILD_DIR)/*.bin
	$(Q)$(MAKE) -C src/ clean

.PHONY = all clean

$(BUILD_DIR):
	$(Q)mkdir -p $(BUILD_DIR)
	
UMDK-RF: | $(BUILD_DIR)
	@printf "  BUILD $(@)\n"
	$(Q)$(MAKE) TARGET=UMDK-RF -C src/ clean
	$(Q)$(MAKE) TARGET=UMDK-RF -C src/
	$(Q)cp src/DAP42.bin $(BUILD_DIR)/$(@).bin
	$(Q)elf2dfuse src/DAP42.elf $(BUILD_DIR)/$(@).dfu
	$(Q)$(SIZE) src/DAP42.elf
    
UMDK-EMB: | $(BUILD_DIR)
	@printf "  BUILD $(@)\n"
	$(Q)$(MAKE) TARGET=UMDK-EMB -C src/ clean
	$(Q)$(MAKE) TARGET=UMDK-EMB -C src/
	$(Q)cp src/DAP42.bin $(BUILD_DIR)/$(@).bin
	$(Q)elf2dfuse src/DAP42.elf $(BUILD_DIR)/$(@).dfu
	$(Q)$(SIZE) src/DAP42.elf

DAP42: | $(BUILD_DIR)
	@printf "  BUILD $(@)\n"
	$(Q)$(MAKE) TARGET=STM32F042 -C src/ clean
	$(Q)$(MAKE) TARGET=STM32F042 -C src/
	$(Q)cp src/DAP42.bin $(BUILD_DIR)/$(@).bin
	$(Q)elf2dfuse src/DAP42.elf $(BUILD_DIR)/$(@).dfu
	$(Q)$(SIZE) src/DAP42.elf

DAP42DC: | $(BUILD_DIR)
	@printf "  BUILD $(@)\n"
	$(Q)$(MAKE) TARGET=DAP42DC -C src/ clean
	$(Q)$(MAKE) TARGET=DAP42DC -C src/
	$(Q)cp src/DAP42.bin $(BUILD_DIR)/$(@).bin
	$(Q)elf2dfuse src/DAP42.elf $(BUILD_DIR)/$(@).dfu
	$(Q)$(SIZE) src/DAP42.elf

KITCHEN42: | $(BUILD_DIR)
	@printf "  BUILD $(@)\n"
	$(Q)$(MAKE) TARGET=KITCHEN42 -C src/ clean
	$(Q)$(MAKE) TARGET=KITCHEN42 -C src/
	$(Q)cp src/DAP42.bin $(BUILD_DIR)/$(@).bin
	$(Q)elf2dfuse src/DAP42.elf $(BUILD_DIR)/$(@).dfu
	$(Q)$(SIZE) src/DAP42.elf

DAP103: | $(BUILD_DIR)
	@printf "  BUILD $(@)\n"
	$(Q)$(MAKE) TARGET=STM32F103 -C src/ clean
	$(Q)$(MAKE) TARGET=STM32F103 -C src/
	$(Q)cp src/DAP42.bin $(BUILD_DIR)/$(@).bin
	$(Q)elf2dfuse src/DAP42.elf $(BUILD_DIR)/$(@).dfu
	$(Q)$(SIZE) src/DAP42.elf

DAP103-DFU: | $(BUILD_DIR)
	@printf "  BUILD $(@)\n"
	$(Q)$(MAKE) TARGET=STM32F103-DFUBOOT -C src/ clean
	$(Q)$(MAKE) TARGET=STM32F103-DFUBOOT -C src/
	$(Q)cp src/DAP42.bin $(BUILD_DIR)/$(@).bin
	$(Q)elf2dfuse src/DAP42.elf $(BUILD_DIR)/$(@).dfu
	$(SIZE) src/DAP42.elf

BRAINv3.3: | $(BUILD_DIR)
	@printf "  BUILD $(@)\n"
	$(Q)$(MAKE) TARGET=BRAINV3.3 -C src/ clean
	$(Q)$(MAKE) TARGET=BRAINV3.3 -C src/
	$(Q)cp src/DAP42.bin $(BUILD_DIR)/$(@).bin
	$(Q)elf2dfuse src/DAP42.elf $(BUILD_DIR)/$(@).dfu
	$(Q)$(SIZE) src/DAP42.elf

DAP42K6U: | $(BUILD_DIR)
	@printf "  BUILD $(@)\n"
	$(Q)$(MAKE) TARGET=DAP42K6U -C src/ clean
	$(Q)$(MAKE) TARGET=DAP42K6U -C src/
	$(Q)cp src/DAP42.bin $(BUILD_DIR)/$(@).bin
	$(Q)elf2dfuse src/DAP42.elf $(BUILD_DIR)/$(@).dfu
	$(SIZE) src/DAP42.elf

DAP103-NUCLEO-STBOOT: | $(BUILD_DIR)
	@printf "  BUILD $(@)\n"
	$(Q)$(MAKE) TARGET=STLINKV2-1-STBOOT -C src/ clean
	$(Q)$(MAKE) TARGET=STLINKV2-1-STBOOT -C src/
	$(Q)cp src/DAP42.bin $(BUILD_DIR)/$(@).bin
	$(Q)elf2dfuse src/DAP42.elf $(BUILD_DIR)/$(@).dfu
	$(Q)$(SIZE) src/DAP42.elf
