# Directed test for core/lpad_unit.sv:43-66 and core/lpad_port.sv:59-69:
#   1. lpad_unit.sv:47 instantiates lpad_port_i without connecting .commit_ack_i.
#   2. Even when commit_ack_i is connected to commit_stage_i.commit_ack_o[1:0], lpad_port.sv:59-69
#      (if (elp_i == LPAD_EXPECTED) ... elp_o = NO_LPAD_EXPECTED;) is not gated on commit_ack_i,
#      and lpad_unit.sv:64 unconditionally drives elp_o = tmp_elp[CVA6Cfg.NrCommitPorts] (tmp_elp[2]).
#   When a preceding load stalls Commit Port 0 so that jalr x0, 0(x2) (Port 0) and its predicted
#   non-LPAD target addi x31, x31, 1 (Port 1) reach lpad_unit in the same cycle, ports[1] sets
#   sbe_o[1].ex.valid = 1 (withholding commit_ack_o[1] = 0) while clearing tmp_elp[2] = NO_LPAD_EXPECTED,
#   allowing addi x31, x31, 1 to retire on Port 0 next cycle with 0 traps.

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
  la   sp, scratch_line
  lui  x7, 0x33
  la   x2, non_lpad_target
  li   x26, 0
  li   x31, 0
  li   x29, 0
  j    warm_entry

loop_top:
  ld   x27, 0(sp)
  addi x29, x29, 1
warm_entry:
  jalr x0, 0(x2)

non_lpad_target:
  addi x31, x31, 1
  bnez x26, done_loop
  li   x26, 1
  j    loop_top

done_loop:
  mv   s3, s0
  mv   s4, s1
  mv   s5, s2

  la   t0, begin_signature
  sd   s3, 0(t0)
  sd   s4, 8(t0)
  sd   s5, 16(t0)
  sd   x31, 24(t0)

  # Spec: both iterations jump via jalr to non_lpad_target (addi x31, x31, 1), so s3 must equal 2
  li   t1, 2
  bne  s3, t1, bug_triggered
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

.align 6
scratch_line:
  .dword 0x12345678
