# Directed test for core/commit_stage.sv:211-215 and core/csr_regfile.sv:1084, 2342-2343:
#   SSPUSH and SSPOPCHK update ssp via csr_update, which switches on csr_addr_i from
#   core/csr_buffer.sv:51, 62-65 (csr_reg_q.csr_address). Because SSPUSH executes in the LSU
#   (lsu_valid_q <= 1'b1 in core/issue_read_operands.sv:374-379, csr_valid_i == 0),
#   csr_reg_q.csr_address retains the address of whichever CSR was last accessed:
#     1. Set ssp = shadow_stack_top (0x80003080) and write sscratch = 0x123456 (0x140).
#     2. Execute sspush x1 (.word 0xce104073).
#     3. Read sscratch into s5 and ssp (ssrdp s6) into s6, then execute sspopchk x1:
#        - Spike / Sail: sscratch == 0x123456, ssp == 0x80003078, sspopchk x1 retires with 0 traps.
#        - CVA6 (8bb74bef1274): sspush x1 overwrites sscratch (0x140) with ssp - 8 (0x80003078),
#          leaves ssp unchanged at 0x80003080, and sspopchk x1 traps with mcause = 18, mtval = 3.

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

  # Warm D-Cache line for shadow_stack_base (0x80003000..0x80003080)
  la   s11, shadow_stack_top
  sd   zero, -8(s11)
  sd   zero, 0(s11)
  ld   t0, -8(s11)
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
  csrw 0x011, s11

  li   t0, 0x123456
  csrw sscratch, t0

  li   x1, 0x11112222
  .word 0xce104073          # sspush x1
  csrr s5, sscratch         # s5 = sscratch after sspush (Spec: 0x123456; CVA6: ssp - 8 = 0x80003078)
  .word 0xcdc04b73          # ssrdp s6 (s6 = ssp after sspush; Spec: 0x80003078; CVA6: 0x80003080)
  .word 0xcdc0c073          # sspopchk x1

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

  # Spec: 0 traps (s3 == 0) and sscratch (s5) == 0x123456
  bnez s3, bug_triggered
  li   t1, 0x123456
  bne  s5, t1, bug_triggered
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
