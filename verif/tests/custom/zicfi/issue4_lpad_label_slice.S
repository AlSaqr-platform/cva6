# Directed test for core/lpad_port.sv:45, 75 slicing [19:0] instead of [31:12]:
#   - Case A (High-12-bit label alias bypass):
#     x7 = 0x00121000 (x7[31:12] = 0x121) -> jalr to lpad 0x021 (0x00021017).
#     Spike / Sail: traps with mcause = 18, mtval = 2 (s3 = 1, s4 = 18).
#     CVA6 (8bb74bef1274): result[19:0] == lpl_i[19:0] == 0x21000, so 0 traps (s3 = 0).
#   - Case B (Low-12-bit x7[11:0] false trap):
#     x7 = 0x00121001 (x7[31:12] = 0x121, x7[11:0] = 1) -> jalr to lpad 0x121 (0x00121017).
#     Spike / Sail: 0 traps (s5 = 0).
#     CVA6 (8bb74bef1274): result[19:0] (0x21000) != lpl_i[19:0] (0x21001), traps with mcause = 18, mtval = 2 (s5 = 1, s6 = 18).

.option norvc
.section .text.init
.globl _start
_start:
  la   t0, trap_handler
  csrw mtvec, t0

  # Enable M-mode Zicfilp (mseccfg.MLPE = bit 10 = 0x400)
  li   t0, 0x400
  csrs 0x747, t0

  # Case A: x7[31:12] = 0x121 vs lpad 0x021 (expected: 1 trap, mcause=18; CVA6: 0 traps)
  li   s0, 0
  li   s1, 0
  li   s2, 0
  lui  x7, 0x121
  la   x2, target_lpad_bypass
  jalr x0, 0(x2)

.align 2
target_lpad_bypass:
  .word 0x00021017
  mv   s3, s0
  mv   s4, s1

  # Case B: x7[31:12] = 0x121 (with x7[11:0] = 1) vs lpad 0x121 (expected: 0 traps; CVA6: 1 trap, mcause=18)
  li   s0, 0
  li   s1, 0
  li   s2, 0
  lui  x7, 0x121
  addi x7, x7, 1
  la   x2, target_lpad_valid
  jalr x0, 0(x2)

.align 2
target_lpad_valid:
  .word 0x00121017
  mv   s5, s0
  mv   s6, s1

  la   t0, begin_signature
  sd   s3, 0(t0)
  sd   s4, 8(t0)
  sd   s5, 16(t0)
  sd   s6, 24(t0)

  # Spec: Case A traps (s3 == 1, s4 == 18) and Case B does not trap (s5 == 0)
  li   t1, 1
  bne  s3, t1, bug_triggered
  li   t1, 18
  bne  s4, t1, bug_triggered
  bnez s5, bug_triggered
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
