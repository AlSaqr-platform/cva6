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
    input logic rstn_i,
    // The instructions that the core will retire - COMMIT_STAGE
    input riscv::ctrsource_rv_t [CVA6Cfg.NrCommitPorts-1:0] ctr_source_i,
    // Acknowledges of instruction retirements - COMMIT_STAGE
    input riscv::ctr_type_t [CVA6Cfg.NrCommitPorts-1:0] ctr_type_i
);

endmodule
