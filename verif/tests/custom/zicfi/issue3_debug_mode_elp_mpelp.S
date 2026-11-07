# Control Run (no Debug halt): mret with mstatus.MPELP=1 to non-lpad target traps with mcause=18 on Spike and CVA6.
# Subtest 1 (DMI halt/resume while mstatus.MPELP=1, ELP=0):
#   Spike: post-Debug MPELP=1, subsequent mret to non-lpad traps with mcause=18.
#   CVA6 (8bb74bef1274): post-Debug MPELP=0, subsequent mret takes 0 traps.
# Subtest 2 (DMI halt/resume while ELP=1, mstatus.MPELP=0):
#   Spike: post-Debug MPELP=0, non-lpad at dpc traps with mcause=18 (begin_signature[7]=0x1).
#   CVA6 (8bb74bef1274): post-Debug MPELP=1, non-lpad at dpc takes 0 traps (begin_signature[7]=0xbad).
# Build: riscv64-unknown-elf-gcc -march=rv64imac_zicsr_zifencei -mabi=lp64 -mno-relax -Wl,--no-relax -nostdlib -nostartfiles -T link.ld repro.S -o repro.elf
.option norvc
.section .text.init
.globl _start
_start:
  la   t0, trap_handler
  csrw mtvec, t0

  li   t0, 0x400
  csrs 0x747, t0

  # Phase 1: Control run (no Debug halt)
  li   s0, 0
  li   s1, 0
  li   s2, 0
  li   t0, (1 << 41) | (3 << 11)
  csrs mstatus, t0
  la   t1, ctrl_non_lpad_target
  csrw mepc, t1
  mret

.align 4
ctrl_non_lpad_target:
  addi t2, zero, 1
  mv   s6, s0
  mv   s7, s1

  # Phase 2: Subtest 1 (DMI halt/resume while mstatus.MPELP = 1, ELP = 0)
  li   s0, 0
  li   s1, 0
  li   s2, 0
  li   t0, (1 << 41) | (3 << 11)
  csrs mstatus, t0

  li   t3, 40
1:addi t3, t3, -1
  bnez t3, 1b

  csrr t4, mstatus
  srli s3, t4, 41
  andi s3, s3, 1

  li   t0, (3 << 11)
  csrs mstatus, t0
  la   t1, sub1_non_lpad_target
  csrw mepc, t1
  mret

.align 4
sub1_non_lpad_target:
  addi s4, zero, 1
  mv   s8, s0
  mv   s9, s1

  # Phase 3: Subtest 2 (DMI halt/resume while ELP = 1, mstatus.MPELP = 0)
  li   s0, 0
  li   s1, 0
  li   s2, 0
  li   t0, (1 << 41) | (3 << 11)
  csrs mstatus, t0
  la   t1, sub2_cold_target
  csrw mepc, t1
  mret

.org 0x100
sub2_cold_target:
  addi s5, zero, 1
  mv   s11, s0
  csrr t4, mstatus
  srli s10, t4, 41
  andi s10, s10, 1

  la   t0, begin_signature
  sd   s6,  0(t0)
  sd   s7,  8(t0)
  sd   s3,  16(t0)
  sd   s8,  24(t0)
  sd   s9,  32(t0)
  sd   s10, 40(t0)
  sd   s11, 48(t0)

  beqz s3, bug_triggered
  beqz s8, bug_triggered
  bnez s10, bug_triggered
  beqz s11, bug_triggered
  j    spec_passed

.align 4
trap_handler:
  .word 0x00000017
  addi s0, s0, 1
  csrr s1, mcause
  csrr s2, mtval
  li   t5, (1 << 41)
  csrc mstatus, t5
  csrr t4, mepc
  addi t4, t4, 4
  csrw mepc, t4
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
