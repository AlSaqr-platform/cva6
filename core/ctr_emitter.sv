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
// Description: The control transfer records emitter.
//
// This module processes the sequence of instructions retired by the commit
// stage, it identifies the source and type of control transfers, and it
// extracts their target address to build the control transfer record.

module ctr_emitter
  import ariane_pkg::*;
#(
    parameter config_pkg::cva6_cfg_t CVA6Cfg = config_pkg::cva6_cfg_empty
) (
    // Subsystem clock - SUBSYSTEM
    input logic clk_i,
    // Asynchronous reset active low - SUBSYSTEM
    input logic rstn_i,
    // The address of the last instructions retired - CTR_UNIT
    input riscv::xlen_t [CVA6Cfg.NrCommitPorts-1:0] source_i,
    // The control transfer type of the last instructions retired - CTR_UNIT
    input riscv::ctr_type_t [CVA6Cfg.NrCommitPorts-1:0] type_i,
    // The module inputs are valid - CTR_UNIT
    input logic [CVA6Cfg.NrCommitPorts-1:0] valid_i,
    // Control Transfer Records source register - CTR_UNIT
    output riscv::ctrsource_rv_t [CVA6Cfg.NrCommitPorts-1:0] source_o,
    // Control Transfer Records target register - CTR_UNIT
    output riscv::ctrtarget_rv_t [CVA6Cfg.NrCommitPorts-1:0] target_o,
    // Control Transfer Records data register - CTR_UNIT
    output riscv::ctrdata_rv_t [CVA6Cfg.NrCommitPorts-1:0] data_o
);

  // Temporary storage for control transfers with unknown target.
  riscv::xlen_t pending_source_d, pending_source_q;
  riscv::ctr_type_t pending_type_d, pending_type_q;
  logic pending_valid_d, pending_valid_q;

  always_comb begin
    // By default, we don't have any control transfer pending.
    pending_source_d = 'b0;
    pending_type_d = riscv::CTR_TYPE_NONE;
    pending_valid_d = 'b0;
    if (~valid_i[0]) begin
      // If no instructions are retired in the current cycle, keep the old values.
      pending_source_d = pending_source_q;
      pending_type_d = pending_type_q;
      pending_valid_d = pending_valid_q;
    end else begin
      // Record the most recent control transfer with unknown target address.
      for (int unsigned i = 0; i < CVA6Cfg.NrCommitPorts-1; i++) begin
        if (valid_i[i] && type_i[i] != riscv::CTR_TYPE_NONE && ~valid_i[i+1]) begin
          pending_source_d = source_i[i];
          pending_type_d = type_i[i];
          pending_valid_d = valid_i[i];
          break;
        end
      end
    end
  end

  always_comb begin
    source_o = 'b0;
    target_o = 'b0;
    data_o = 'b0;
    if (pending_valid_q && valid_i[0]) begin
      source_o[0].pc = pending_source_q[riscv::XLEN-1:1];
      source_o[0].v = pending_valid_q;
      // The MISP bit is unimplemented.
      target_o[0].pc = source_i[0][riscv::XLEN-1:1];
      target_o[0].misp = 'b0;
      // Cycle counting is unimplemented.
      data_o[0].cftype = pending_type_q;
    end
    for (int unsigned i = 0; i < CVA6Cfg.NrCommitPorts-1; i++) begin
      if (valid_i[i+1]) begin
        source_o[i+1].pc = source_i[i][riscv::XLEN-1:1];
        source_o[i+1].v = valid_i[i];
        // The MISP bit is unimplemented.
        target_o[i+1].pc = source_i[i+1][riscv::XLEN-1:1];
        target_o[i+1].misp = 'b0;
        // Cycle counting is unimplemented.
        data_o[i+1].cftype = type_i[i];
      end
    end
  end

  always_ff @(posedge clk_i, negedge rstn_i) begin
    if (~rstn_i) begin
      pending_source_q <= 'b0;
      pending_type_q <= riscv::CTR_TYPE_NONE;
      pending_valid_q <= 'b0;
    end else begin
      pending_source_q <= pending_source_d;
      pending_type_q <= pending_type_d;
      pending_valid_q <= pending_valid_d;
    end
  end

endmodule
