# Directed test for core/lpad_port.sv:43 gating is_sbe_valid on ~sbe_i.ex.valid:
#   When ELP == LPAD_EXPECTED (jalr x0, 0(x2)) and the target instruction at target_fault is an
#   illegal opcode (.word 0xffffffff), core/decoder.sv sets sbe_i.ex.valid = 1'b1 with
#   sbe_i.ex.cause = riscv::ILLEGAL_INSTR (mcause = 2).
#   - Spike / Sail: traps with Software Check (mcause = 18, mtval = 2) and clears ELP = 0.
#   - CVA6 (8bb74bef1274): is_sbe_valid evaluates to 0, skipping the ELP == LPAD_EXPECTED
#     check and taking ILLEGAL_INSTR (mcause = 2, mtval = 0xffffffff) while leaving elp_q = 1.

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
  lui  x7, 0x33
  la   x2, target_fault
  jalr x0, 0(x2)

.align 2
target_fault:
  .word 0xffffffff
  .word 0x00000017
  mv   s3, s0
  mv   s4, s1
  mv   s5, s2

  la   t0, begin_signature
  sd   s3, 0(t0)
  sd   s4, 8(t0)
  sd   s5, 16(t0)

  # Spec: traps once at target_fault with mcause = 18, mtval = 2
  li   t1, 18
  bne  s4, t1, bug_triggered
  li   t1, 2
  bne  s5, t1, bug_triggered
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
