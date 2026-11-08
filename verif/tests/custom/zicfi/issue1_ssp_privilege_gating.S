# Subtest 1 (M-mode csrw/csrr ssp, menvcfg.SSE=1, senvcfg.SSE=0): Spike 0 traps (ssp=0x80004000); CVA6 2 traps (mcause=2).
# Subtest 2 (S-mode csrr ssp,      menvcfg.SSE=1, senvcfg.SSE=0): Spike 0 traps (ssp=0x80004000); CVA6 1 trap  (mcause=2).
# Subtest 3 (U-mode csrw/csrr ssp, menvcfg.SSE=1, senvcfg.SSE=0): Spike 2 traps (mcause=2);       CVA6 0 traps (ssp=0x80004888).
# Subtest 4 (U-mode ssrdp s10,     menvcfg.SSE=1, senvcfg.SSE=1): Spike s10=0x80004000;           CVA6 s10=0   (begin_signature[7]=0xbad).
# Build: riscv64-unknown-elf-gcc -march=rv64imac_zicsr_zifencei -mabi=lp64 -nostdlib -nostartfiles -T link.ld repro.S -o repro.elf
.section .text.init
.globl _start
_start:
  la   t0, trap_handler
  csrw mtvec, t0

  li   t0, -1
  csrw pmpaddr0, t0
  li   t0, 0x1f
  csrw pmpcfg0, t0

  li   t0, 0x8
  csrw 0x30a, t0
  csrw 0x10a, zero

  # Subtest 1: M-mode csrw/csrr ssp with menvcfg.SSE=1, senvcfg.SSE=0
  li   s0, 0
  li   s1, 0
  li   s3, 0
  li   t0, 0x80004000
  csrw 0x011, t0
  csrr s3, 0x011
  mv   s6, s0
  mv   s7, s1

  # Subtest 2: S-mode csrr ssp with menvcfg.SSE=1, senvcfg.SSE=0
  li   s0, 0
  li   s1, 0
  li   s4, 0
  li   t0, (1 << 11)
  csrw mstatus, t0
  la   t0, smode_entry
  csrw mepc, t0
  mret

smode_entry:
  csrr s4, 0x011
  mv   s8, s0
  li   a7, 1
  ecall

after_smode:
  # Subtest 3: U-mode csrw/csrr ssp with menvcfg.SSE=1, senvcfg.SSE=0
  li   s0, 0
  li   s1, 0
  li   s5, 0
  csrw 0x10a, zero
  csrw mstatus, zero
  la   t0, umode_disabled_entry
  csrw mepc, t0
  mret

umode_disabled_entry:
  li   t0, 0x80004888
  csrw 0x011, t0
  csrr s5, 0x011
  mv   s9, s0
  li   a7, 2
  ecall

after_umode_disabled:
  # Subtest 4: U-mode ssrdp s10 with menvcfg.SSE=1, senvcfg.SSE=1
  li   t0, 0x8
  csrw 0x30a, t0
  csrw 0x10a, t0
  li   t1, 0x80004000
  csrw 0x011, t1
  li   s0, 0
  csrw mstatus, zero
  la   t0, umode_enabled_entry
  csrw mepc, t0
  mret

umode_enabled_entry:
  li   s10, 0
  .word 0xcdc04d73
  li   a7, 3
  ecall

after_umode_enabled:
  la   t0, begin_signature
  sd   s6,  0(t0)
  sd   s3,  8(t0)
  sd   s8,  16(t0)
  sd   s4,  24(t0)
  sd   s9,  32(t0)
  sd   s5,  40(t0)
  sd   s10, 48(t0)

  bnez s6, bug_triggered
  bnez s8, bug_triggered
  li   t1, 2
  bne  s9, t1, bug_triggered
  beqz s10, bug_triggered
  j    spec_passed

.align 4
trap_handler:
  csrr t5, mcause
  li   t6, 8
  beq  t5, t6, handle_ecall
  li   t6, 9
  beq  t5, t6, handle_ecall

  addi s0, s0, 1
  mv   s1, t5
  csrr s2, mtval
  csrr t4, mepc
  addi t4, t4, 4
  csrw mepc, t4
  mret

handle_ecall:
  li   t6, 1
  beq  a7, t6, ecall_from_s
  li   t6, 2
  beq  a7, t6, ecall_from_u1
  la   t4, after_umode_enabled
  j    ecall_return_m
ecall_from_s:
  la   t4, after_smode
  j    ecall_return_m
ecall_from_u1:
  la   t4, after_umode_disabled
ecall_return_m:
  csrw mepc, t4
  li   t6, (3 << 11)
  csrw mstatus, t6
  mret

.org 0x200
.globl spec_passed
spec_passed:
  la   t0, begin_signature
  li   t1, 0x1
  sd   t1, 56(t0)
  la   t0, completion_word
  li   t1, 0xc0de
  sd   t1, 0(t0)
  la   t0, tohost
  li   t1, 1
  sd   t1, 0(t0)
1:j    1b

.org 0x300
.globl bug_triggered
bug_triggered:
  la   t0, begin_signature
  li   t1, 0xbad
  sd   t1, 56(t0)
  la   t0, completion_word
  li   t1, 0xc0de
  sd   t1, 0(t0)
  la   t0, tohost
  li   t1, 1
  sd   t1, 0(t0)
1:j    1b

.section .tohost,"aw",@progbits
.align 6
.globl tohost
tohost: .dword 0
.globl fromhost
fromhost: .dword 0

.section .data
.align 12
.globl begin_signature
begin_signature:
  .fill 8, 8, 0
.globl end_signature
end_signature:

.align 6
.globl completion_word
completion_word:
  .dword 0
