# Directed test for core/lpad_port.sv:72-76 else-if chain between is_sbe_jalr_lpad and is_sbe_lplwrite:
#   1. Prior lui x7, 0x22 sets x7 = 0x22000 (lpl_q = 0x22000, label 0x00022).
#   2. jalr x7, 0(x2) at 0x80000100 arms ELP = LPAD_EXPECTED and writes x7 = 0x80000104
#      (x7[31:12] = 0x80000 != 0x00022).
#   3. Target instruction at target_stale_lpad is lpad 0x022 (.word 0x00022017):
#      - Spike / Sail: x7[31:12] is 0x80000 != 0x00022, so lpad 0x022 traps with
#        Software Check (mcause = 18, mtval = 2, s3 = 1, s4 = 18).
#      - CVA6 (8bb74bef1274): else-if skips lpl_o update on jalr x7, 0(x2), leaving
#        stale lpl_q = 0x22000, so lpad 0x022 retires with 0 traps (s3 = 0).

.option norvc
.section .text.init
.globl _start
_start:
  la   t0, trap_handler
  csrw mtvec, t0

  # Enable M-mode Zicfilp (mseccfg.MLPE = bit 10 = 0x400)
  li   t0, 0x400
  csrs 0x747, t0

  li   s0, 0
  li   s1, 0
  li   s2, 0
  lui  x7, 0x22
  la   x2, target_stale_lpad
  j    do_dual_jalr

.org 0x100
do_dual_jalr:
  jalr x7, 0(x2)

.align 2
target_stale_lpad:
  .word 0x00022017
  mv   s3, s0
  mv   s4, s1
  mv   s5, s2

  la   t0, begin_signature
  sd   s3, 0(t0)
  sd   s4, 8(t0)
  sd   s5, 16(t0)
  sd   x7, 24(t0)

  # Spec: jalr x7, 0(x2) overwrote x7 with 0x80000104 (x7[31:12] = 0x80000 != 0x022), so lpad 0x022 must trap (s3 == 1, s4 == 18)
  li   t1, 1
  bne  s3, t1, bug_triggered
  li   t1, 18
  bne  s4, t1, bug_triggered
  j    spec_passed

.align 4
trap_handler:
  addi s0, s0, 1
  csrr s1, mcause
  csrr s2, mtval
  li   t0, (1 << 41)
  csrc mstatus, t0
  csrr t0, mepc
  addi t0, t0, 4
  csrw mepc, t0
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
