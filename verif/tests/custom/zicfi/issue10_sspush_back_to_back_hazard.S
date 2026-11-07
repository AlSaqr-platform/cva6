# Directed test for core/issue_read_operands.sv:311-315:
#   SSPUSH computes its store address from committed ssp_i (ssp_q - 8) at the Issue stage,
#   while ssp_q is only decremented at the Commit stage (core/commit_stage.sv:211-215 ->
#   core/csr_regfile.sv:2342), with no scoreboard hazard check or forwarding for in-flight SSPUSH:
#     1. Set ssp = shadow_stack_top (0x80003080), x1 = 0x11112222, x5 = 0x66667777.
#     2. Execute back-to-back sspush x1 (.word 0xce104073) and sspush x5 (.word 0xce504073),
#        followed by sspopchk x5 (.word 0xcdc2c073):
#        - Spike / Sail: sspush x1 writes 0x11112222 to [0x80003078], sspush x5 writes
#          0x66667777 to [0x80003070], and sspopchk x5 pops 0x66667777 with 0 traps (s3 = 0).
#        - CVA6 (8bb74bef1274): both sspush instructions issue with ssp_i = 0x80003080 and
#          write to [0x80003078] ([0x80003078] = 0x66667777, [0x80003070] = 0), while ssp_q
#          is decremented twice at Commit to 0x80003070; sspopchk x5 reads 0 != 0x66667777 from
#          [0x80003070] and traps with mcause = 18, mtval = 3 (s3 = 1, s4 = 18).

.option norvc
.section .text.init
.globl _start
_start:
  la   t0, trap_handler
  csrw mtvec, t0

  # Configure PMP so S-mode has full RWX access to physical memory
  li   t0, -1
  csrw pmpaddr0, t0
  li   t0, 0x1f
  csrw pmpcfg0, t0

  # Enable Zicfiss in menvcfg.SSE (bit 3) and senvcfg.SSE (bit 3)
  li   t0, 0x8
  csrw 0x30a, t0
  csrw 0x10a, t0

  # Clear and warm D-Cache line for shadow_stack_top (0x80003080)
  la   s11, shadow_stack_top
  sd   zero, -16(s11)
  sd   zero, -8(s11)
  csrw 0x011, s11

  # Set up 3-level Sv39 page table with 0x80003000 as Shadow Stack page (PTE = 0xc5)
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

  li   t5, (0x80000 << 10) | 0xcf
  sd   t5, 0(t3)
  li   t5, (0x80001 << 10) | 0xcf
  sd   t5, 8(t3)
  li   t5, (0x80002 << 10) | 0xcf
  sd   t5, 16(t3)
  li   t5, (0x80003 << 10) | 0xc5
  sd   t5, 24(t3)

  srli t2, t0, 12
  li   t4, (8 << 60)
  or   t2, t2, t4
  csrw satp, t2
  sfence.vma

  li   s0, 0
  li   t0, 1
  csrw 0xca0, t0
  bnez s0, enter_smode

  j    run_test

enter_smode:
  li   s0, 0
  li   s1, 0
  li   s2, 0
  li   t0, (1 << 11)
  csrw mstatus, t0
  la   t0, run_test
  csrw mepc, t0
  mret

.align 4
run_test:
  li   s0, 0
  li   s1, 0
  li   s2, 0
  csrw 0x011, s11           # ssp = shadow_stack_top (0x80003080, also primes csr_buffer to 0x011)

  li   x1, 0x11112222
  li   x5, 0x66667777
  .word 0xce104073          # sspush x1
  .word 0xce504073          # sspush x5 (back-to-back!)
  ld   s5, -8(s11)          # s5 = mem[ssp_top - 8]  (Spec: 0x11112222; CVA6: 0x66667777)
  ld   s6, -16(s11)         # s6 = mem[ssp_top - 16] (Spec: 0x66667777; CVA6: 0x0)
  .word 0xcdc2c073          # sspopchk x5

  li   a7, 1
  ecall

.align 4
after_test:
  mv   s3, s0
  mv   s4, s1

  la   t0, begin_signature
  sd   s3, 0(t0)
  sd   s4, 8(t0)
  sd   s5, 16(t0)
  sd   s6, 24(t0)

  # Spec: 0 traps (s3 == 0), mem[ssp_top - 8] == 0x11112222, mem[ssp_top - 16] == 0x66667777
  bnez s3, bug_triggered
  li   t1, 0x11112222
  bne  s5, t1, bug_triggered
  li   t1, 0x66667777
  bne  s6, t1, bug_triggered
  j    spec_passed

.align 4
trap_handler:
  csrr t4, mcause
  li   t5, 8
  beq  t4, t5, handle_ecall
  li   t5, 9
  beq  t4, t5, handle_ecall
  li   t5, 11
  beq  t4, t5, handle_ecall

  addi s0, s0, 1
  mv   s1, t4
  csrr s2, mtval
  csrr t4, mepc
  addi t4, t4, 4
  csrw mepc, t4
  mret

handle_ecall:
  la   t4, after_test
  csrw mepc, t4
  li   t5, (3 << 11)
  csrs mstatus, t5
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
.globl shadow_stack_page
shadow_stack_page:
  .space 128
.globl shadow_stack_top
shadow_stack_top:
  .space 3968

.align 12
pt_l2:
  .space 4096
pt_l1:
  .space 4096
pt_l0:
  .space 4096
