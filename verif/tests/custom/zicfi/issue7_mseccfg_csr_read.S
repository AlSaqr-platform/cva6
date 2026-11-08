# Directed test for core/csr_regfile.sv omitting CSR_MSECCFG (0x747) and CSR_MSECCFGH (0x757)
# from csr_read_process (lines 371-941):
#   Because csr_op_logic (line 2332) sets csr_read = 1'b1 for all CSR_WRITE/CSR_SET/CSR_CLEAR/CSR_READ
#   operations (even when rd == x0), M-mode csrrw x0, 0x747, x6 hits default: read_access_exception = 1'b1;
#   at line 938 and traps with ILLEGAL_INSTR (mcause = 2, mtval = 0x74731073):
#     - Spike / Sail: 0 traps (s3 = 0).
#     - CVA6 (8bb74bef1274): traps with mcause = 2, mtval = 0x74731073 (s3 = 1, s4 = 2).

.option norvc
.section .text.init
.globl _start
_start:
  la   t0, trap_handler
  csrw mtvec, t0
  li   s0, 0
  li   s1, 0
  li   s2, 0

  li   x6, 0x400
.global fault_insn
fault_insn:
  csrrw x0, 0x747, x6

  mv   s3, s0
  mv   s4, s1
  mv   s5, s2

  la   t0, begin_signature
  sd   s3, 0(t0)
  sd   s4, 8(t0)
  sd   s5, 16(t0)

  # Spec: M-mode write to mseccfg (0x747) succeeds with 0 traps (s3 == 0)
  bnez s3, bug_triggered
  j    spec_passed

.align 4
trap_handler:
  addi s0, s0, 1
  csrr s1, mcause
  csrr s2, mtval
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
