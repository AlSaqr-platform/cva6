# Two-frame shadow stack at shadow_stack_base (0x80003000): [ssp+0] = 0x11112222 (x1), [ssp+8] = 0x66667777 (x5).
# Phase 1 (Port 0 control, no div): sspopchk x1 -> sspopchk x5 advances ssp by +16 with 0 traps on Spike and CVA6.
# Phase 2 (div + sspopchk x1 dual-commit):
#   Spike: 1st_delta = 8, 2nd_delta = 16, traps = 0 (begin_signature[7] = 0x1).
#   CVA6 (8bb74bef1274): div (Port 0) and sspopchk x1 (Port 1) commit in the same cycle; Port 1 drops ssp += 8
#   (1st_delta = 0), and sspopchk x5 traps with mcause = 18, mtval = 3 (begin_signature[7] = 0xbad).
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
  csrw 0x10a, t0

  la   s11, shadow_stack_base
  li   x1, 0x11112222
  li   x5, 0x66667777
  sd   x1, 0(s11)
  sd   x5, 8(s11)
  ld   t0, 0(s11)
  csrw 0x011, s11

  la   t0, pt_l2
  la   t1, pt_l1
  srli t2, t1, 12
  slli t2, t2, 10
  ori  t2, t2, 0x01
  sd   t2, 16(t0)

  la   t3, pt_l0
  srli t2, t3, 12
  slli t2, t2, 10
  ori  t2, t2, 0x01
  sd   t2, 0(t1)

  li   t4, 0x80000
  slli t5, t4, 10
  ori  t5, t5, 0xcf
  sd   t5, 0(t3)
  addi t4, t4, 1
  slli t5, t4, 10
  ori  t5, t5, 0xcf
  sd   t5, 8(t3)
  addi t4, t4, 1
  slli t5, t4, 10
  ori  t5, t5, 0xcf
  sd   t5, 16(t3)
  addi t4, t4, 1
  slli t5, t4, 10
  ori  t5, t5, 0xc5
  sd   t5, 24(t3)

  srli t2, t0, 12
  li   t4, (8 << 60)
  or   t2, t2, t4
  csrw satp, t2
  sfence.vma

  li   s0, 0
  li   t0, 1
  csrw 0xca0, t0
  bnez s0, enter_smode_for_spec
  j    run_tests

enter_smode_for_spec:
  li   s0, 0
  li   s1, 0
  li   s2, 0
  li   t0, (1 << 11)
  csrw mstatus, t0
  la   t0, run_tests
  csrw mepc, t0
  mret

.align 4
run_tests:
  # Phase 1: Port 0 control (no preceding div)
  li   s0, 0
  li   x1, 0x11112222
  li   x5, 0x66667777
  csrw 0x011, s11

  .word 0xcdc0c073
  .word 0xcdc049f3
  sub  s6, s3, s11

  .word 0xcdc2c073
  .word 0xcdc049f3
  sub  s6, s3, s11
  mv   s7, s0

  # Phase 2: Dual-commit (div on Port 0 + sspopchk x1 on Port 1)
  li   s0, 0
  li   s1, 0
  li   s2, 0
  li   t2, 0x12345678
  li   t3, 7
  csrw 0x011, s11

  div  t4, t2, t3
  .word 0xcdc0c073

  .word 0xcdc049f3
  sub  s4, s3, s11

  .word 0xcdc2c073

  .word 0xcdc04af3
  sub  s5, s5, s11

  li   a7, 99
  ecall

after_test:
  la   t0, begin_signature
  sd   s6, 0(t0)
  sd   s7, 8(t0)
  sd   s4, 16(t0)
  sd   s5, 24(t0)
  sd   s0, 32(t0)
  sd   s1, 40(t0)
  sd   s2, 48(t0)

  bnez s0, bug_triggered
  li   t1, 8
  bne  s4, t1, bug_triggered
  li   t1, 16
  bne  s5, t1, bug_triggered
  j    spec_passed

.align 4
trap_handler:
  csrr t5, mcause
  li   t6, 99
  beq  a7, t6, finish_ecall

  addi s0, s0, 1
  mv   s1, t5
  csrr s2, mtval
  csrr t4, mepc
  addi t4, t4, 4
  csrw mepc, t4
  mret

finish_ecall:
  csrw satp, zero
  sfence.vma
  la   t4, after_test
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

.align 12
.globl shadow_stack_base
shadow_stack_base:
  .fill 512, 8, 0

.align 12
.globl pt_l2
pt_l2:
  .fill 512, 8, 0
.align 12
.globl pt_l1
pt_l1:
  .fill 512, 8, 0
.align 12
.globl pt_l0
pt_l0:
  .fill 512, 8, 0
