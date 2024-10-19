// Copyright 2023 ETH Zurich and University of Bologna.
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
// Date: 17.04.2024
// Description: Implements landing pad enforcement.


module lpad_port
  import ariane_pkg::*;
#(
    parameter config_pkg::cva6_cfg_t CVA6Cfg = config_pkg::cva6_cfg_empty
) (
    //
    input logic lpe_i,
    //
    input elp_t elp_i,
    //
    input lpl_t lpl_i,
    //
    input scoreboard_entry_t sbe_i,
    //
    input logic commit_ack_i,
    //
    input logic bypass_i,
    //
    output elp_t elp_o,
    //
    output lpl_t lpl_o,
    //
    output scoreboard_entry_t sbe_o,
    //
    output logic bypass_o
);

  assign is_sbe_valid       = sbe_i.valid && ~sbe_i.ex.valid && lpe_i;
  assign is_sbe_lpad        = sbe_i.pc[1:0] == 2'b0 && sbe_i.op == ZICFI_LPAD;
  assign is_sbe_lplmismatch = sbe_i.result != 'b0 && sbe_i.result[LPAD_LABEL_BITS-1:0] != lpl_i;
  assign is_sbe_lplwrite    = sbe_i.rd == 'd7;
  assign is_sbe_jalr_lpad   = sbe_i.op == JALR && ~(sbe_i.rs1 inside {'d1, 'd5, 'd7});

  always_comb begin : control_flow_check
    elp_o = elp_i;
    lpl_o = lpl_i;
    sbe_o = sbe_i;
    bypass_o = bypass_i;
    if (is_sbe_valid) begin
      // The core is committing a valid scoreboard entry.
      if (~bypass_i) begin
        // All preceeding commit ports are valid, and all preceeding landing
        // pad ports do not containg CFI violations.
        if (elp_i == LPAD_EXPECTED) begin
          // A landing pad is expected, any instruction except for a valid
          // LPAD issues an exception.
          if (~is_sbe_lpad || is_sbe_lplmismatch) begin
            sbe_o.ex.cause = LPAD_EXCEPTION_CAUSE;
            sbe_o.ex.tval = LPAD_EXCEPTION_TVAL;
            sbe_o.ex.valid = 'b1;
            bypass_o = 'b1;
          end
          elp_o = NO_LPAD_EXPECTED;
        end else begin
          // If a landing pad is not expected, but we have an indirect jump, the
          // next instruction committed should be a landing pad.
          if (is_sbe_jalr_lpad && commit_ack_i) begin
            elp_o = LPAD_EXPECTED;
          end else if (is_sbe_lplwrite && commit_ack_i) begin
            lpl_o = sbe_i.result[LPAD_LABEL_BITS-1:0];
          end
        end
      end
    end else begin
      bypass_o = 'b1;
    end
  end

endmodule
