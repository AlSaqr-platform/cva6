// Copyright 2024 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Emanuele Parisi, University of Bologna
// Description: Control Transfer Records unit.


module ctr_unit
  import ariane_pkg::*;
#(
    parameter config_pkg::cva6_cfg_t CVA6Cfg = config_pkg::cva6_cfg_empty
) (
    // Subsystem clock - SUBSYSTEM
    input logic clk_i,
    // Asynchronous reset active low - SUBSYSTEM
    input logic rst_ni,
    // Input commit ports
    input       ctr_commit_port_t ctr_commit_port_1_i,
    input       ctr_commit_port_t ctr_commit_port_2_i,
    // Control Transfer Records source register - CTR_UNIT
    output      riscv::ctrsource_rv_t emitter_source_o,
    // Control Transfer Records target register - CTR_UNIT
    output      riscv::ctrtarget_rv_t emitter_target_o,
    // Control Transfer Records data register - CTR_UNIT
    output      riscv::ctr_type_t emitter_data_o,
    // C2ontrol Transfer Records instr register - CTR_UNIT
    output      logic [31:0] emitter_instr_o,
    // Privilege execution level - CTR_UNIT
    output      riscv::priv_lvl_t priv_lvl_o
);

  // Temporary storage for control transfers with unknown target.
  riscv::xlen_t pending_source_d, pending_source_q;
  riscv::ctr_type_t pending_type_d, pending_type_q;
  logic [31:0] pending_instr_d, pending_instr_q;
  riscv::priv_lvl_t pending_priv_lvl_d, pending_priv_lvl_q;
  logic pending_valid_d, pending_valid_q;

  ctr_commit_port_t  ctr_sbe_entry_out;

  localparam int ReqFifoWidth = $bits(ctr_commit_port_t) ;
  logic fifo_empty, fifo_full, fifo_out_valid;

  assign priv_lvl_o = pending_priv_lvl_q;
  assign fifo_out_valid = ~fifo_empty;


  // Dual port fifo to serialize CVA6 commit ports
  fifo_dp_v3 #(
     .FALL_THROUGH  ( 1'b1         ),
     .DATA_WIDTH    ( ReqFifoWidth ),
     .DEPTH         ( 4            )
  ) dual_port_fifo  (
     .clk_i         ( clk_i                                   ),
     .rst_ni        ( rst_ni                                  ),
     .flush_i       ( 1'b0                                    ),
     .testmode_i    ( 1'b0                                    ),
     .full_o        ( fifo_full                               ),
     .empty_o       ( fifo_empty                              ),
     .usage_o       (                                         ),
     .data_port_0_i ( ctr_commit_port_1_i                     ),
     .push_port_0_i ( ~fifo_full && ctr_commit_port_1_i.valid ),
     .data_port_1_i ( ctr_commit_port_2_i                     ),
     .push_port_1_i ( ~fifo_full && ctr_commit_port_2_i.valid ),
     .data_o        ( ctr_sbe_entry_out                       ),
     .pop_i         ( fifo_out_valid                          )
  );

  always_comb begin
    // By default, we don't have any control transfer pending.
    pending_source_d = 'b0;
    pending_type_d = riscv::CTR_TYPE_NONE;
    pending_valid_d = 'b0;
    pending_instr_d = 'b0;
    pending_priv_lvl_d = riscv::PRIV_LVL_M;
    if (~fifo_out_valid) begin
      // If no instructions are retired in the current cycle, keep the old values.
      pending_source_d = pending_source_q;
      pending_type_d = pending_type_q;
      pending_valid_d = pending_valid_q;
      pending_instr_d = pending_instr_q;
      pending_priv_lvl_d = pending_priv_lvl_q;
    end else begin
      // Record the most recent control transfer with unknown target address.
      pending_source_d = ctr_sbe_entry_out.ctr_source;
      pending_type_d = ctr_sbe_entry_out.ctr_type;
      pending_valid_d = fifo_out_valid;
      pending_instr_d = ctr_sbe_entry_out.ctr_instr;
      pending_priv_lvl_d = ctr_sbe_entry_out.priv_lvl;
    end
  end

  always_comb begin
    emitter_source_o = 'b0;
    emitter_target_o = 'b0;
    emitter_data_o = riscv::CTR_TYPE_NONE;
    emitter_instr_o = 'b0;
    if (pending_valid_q && fifo_out_valid) begin
      emitter_source_o.pc = pending_source_q[riscv::XLEN-1:1];
      emitter_source_o.v = pending_valid_q;
      // The MISP bit is unimplemented.
      emitter_target_o.pc = ctr_sbe_entry_out.ctr_source[riscv::XLEN-1:1];
      emitter_target_o.misp = 'b0;
      // Cycle counting is unimplemented.
      emitter_data_o = pending_type_q;
      emitter_instr_o = pending_instr_q;
    end
  end

  always_ff @(posedge clk_i, negedge rst_ni) begin
    if (~rst_ni) begin
      pending_source_q <= 'b0;
      pending_type_q <= riscv::CTR_TYPE_NONE;
      pending_instr_q <= 'b0;
      pending_valid_q <= 'b0;
      pending_priv_lvl_q <= riscv::PRIV_LVL_M;
    end else begin
      pending_source_q <= pending_source_d;
      pending_type_q <= pending_type_d;
      pending_priv_lvl_q <= pending_priv_lvl_d;
      pending_instr_q <= pending_instr_d;
      pending_valid_q <= pending_valid_d;
    end
  end

endmodule
