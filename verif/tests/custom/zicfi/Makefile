RISCV_GCC     ?= riscv64-unknown-elf-gcc
RISCV_OBJCOPY ?= riscv64-unknown-elf-objcopy
CFLAGS        := -march=rv64imac_zicsr_zifencei -mabi=lp64 -mno-relax -Wl,--no-relax -nostdlib -nostartfiles -T link.ld

TESTS := \
	issue1_ssp_privilege_gating \
	issue2_commit_port1_sspopchk \
	issue3_debug_mode_elp_mpelp \
	issue4_lpad_label_slice \
	issue5_lpad_illegal_priority \
	issue6_jalr_x7_lpl_update \
	issue7_mseccfg_csr_read \
	issue8_commit_port1_elp_clear \
	issue9_sspush_csr_buffer_addr \
	issue10_sspush_back_to_back_hazard

ELFS := $(addsuffix .elf,$(TESTS))
BINS := $(addsuffix .bin,$(TESTS))

.PHONY: all clean

all: $(ELFS) $(BINS)

%.elf: %.S link.ld
	$(RISCV_GCC) $(CFLAGS) $< -o $@

%.bin: %.elf
	$(RISCV_OBJCOPY) -O binary $< $@

clean:
	rm -f $(ELFS) $(BINS)
