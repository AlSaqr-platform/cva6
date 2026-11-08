# Directed Zicfilp / Zicfiss Regression Tests (`pulp-v1-culsans`)

Standalone directed tests for the `Zicfilp` (Landing Pad) and `Zicfiss` (Shadow Stack) implementation in `pulp-v1-culsans` (`8bb74bef1274a7dcadece7ae29b1db322b1d6f7a`).

Each test builds with `make` (`riscv64-unknown-elf-gcc -march=rv64imac_zicsr_zifencei -mabi=lp64 -mno-relax -Wl,--no-relax -nostdlib -nostartfiles -T link.ld`) and writes its verdict to `begin_signature[7]` (`0x80002038`):
- `0x1` (`spec_passed` at `0x80000200`): expected behavior per the RISC-V `Zicfilp` / `Zicfiss` / `Sdext` specifications (verified on Spike `--isa=rv64imafdcvh_zicntr_zicsr_zifencei_zicfilp_zicfiss` and Sail).
- `0xbad` (`bug_triggered` at `0x80000300`): current behavior on `pulp-v1-culsans` (`8bb74bef1274`).

| Test | RTL Location | Summary |
|------|--------------|---------|
| `issue1_ssp_privilege_gating.S` | `core/csr_regfile.sv:416-420, 1116-1118`; `core/id_stage.sv:107-119` | `CSR_SSP (0x011)` privilege check uses `!= PRIV_LVL_U` instead of `== PRIV_LVL_U`, and non-virtualized U-mode hardcodes `xsse_o = 1'b0`. |
| `issue2_commit_port1_sspopchk.S` | `core/commit_stage.sv:169-178, 211-215, 297-339` | Commit Port 1 (`commit_instr_i[1]`) omits `SSP Logic` for `SSPOPCHK`, dropping `ssp += 8` on dual-commit; Port 0 `SSP Logic` overrides `commit_lsu_ready_i == 0`. |
| `issue3_debug_mode_elp_mpelp.S` (+ `dmi_debug_dpi.cc`) | `core/csr_regfile.sv:2049-2127, 2249-2260, 2300-2306` | Debug Mode entry (`DEBUG_REQUEST`) overwrites `mstatus.MPELP = elp_i`, omits `dcsr_d.pelp = elp_i`, and does not restore `elp_d = dcsr_q.pelp` on `dret`. |
| `issue4_lpad_label_slice.S` | `core/lpad_port.sv:44-46, 74-76` | `lpad_port` slices `sbe_i.result[19:0]` instead of `[31:12]` for `ZICFI_LPAD` label comparison and `x7` writeback (`lpl_o`). |
| `issue5_lpad_illegal_priority.S` | `core/lpad_port.sv:43-69` | `is_sbe_valid` is gated on `~sbe_i.ex.valid`, so an illegal non-`LPAD` target with `ELP == LPAD_EXPECTED` takes `mcause = 2` instead of `mcause = 18, mtval = 2`. |
| `issue6_jalr_x7_lpl_update.S` | `core/lpad_port.sv:46-47, 72-76` | `if (is_sbe_jalr_lpad) ... else if (is_sbe_lplwrite)` drops the `lpl_o` update on `jalr x7, 0(rs1)`. |
| `issue7_mseccfg_csr_read.S` | `core/csr_regfile.sv:375-941, 1631-1639` | `CSR_MSECCFG (0x747)` and `CSR_MSECCFGH (0x757)` are missing from `csr_read_process`, causing M-mode accesses to `mseccfg` to trap with `mcause = 2`. |
| `issue8_commit_port1_elp_clear.S` | `core/lpad_unit.sv:45-64`; `core/lpad_port.sv:59-69`; `core/commit_stage.sv:317` | `lpad_unit.sv:47` leaves `.commit_ack_i` unconnected, and `lpad_port.sv:68` clears `elp_o = NO_LPAD_EXPECTED` on Port 1 even when `commit_ack_o[1] == 0`. |
| `issue9_sspush_csr_buffer_addr.S` | `core/issue_read_operands.sv:374-376`; `core/csr_buffer.sv:51, 62-65`; `core/csr_regfile.sv:1080-1081, 2342-2343` | `SSPUSH` / `SSPOPCHK` updates `ssp` via `csr_addr_i` from `csr_buffer.csr_reg_q.csr_address`, overwriting the previously accessed CSR instead of `ssp`. |
| `issue10_sspush_back_to_back_hazard.S` | `core/issue_read_operands.sv:311-315`; `core/commit_stage.sv:211-215` | Back-to-back `SSPUSH` instructions read stale committed `ssp_q` at Issue without stalling or forwarding, writing both stores to `[ssp_q - 8]` while decrementing `ssp_q` by 16. |
