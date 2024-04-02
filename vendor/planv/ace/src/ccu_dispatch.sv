// Copyright (c) 2014-2018 ETH Zurich, University of Bologna
// Copyright (c) 2023 PlanV GmbH
//
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

`include "ace/assign.svh"
`include "ace/typedef.svh"
module ccu_dispatch
  #(
  parameter int  NoPorts = '0,
  parameter type req_t   = logic,
  parameter type resp_t  = logic
) (
   input  logic                 clk_i,
   input  logic                 rst_ni,
   input  req_t  [NoPorts-1:0]  core_req_i,
   output resp_t [NoPorts-1:0]  core_resp_o,
   output req_t  [NoPorts-1:0]  ccu_req_o,
   input  resp_t [NoPorts-1:0]  ccu_resp_i
);

   for (genvar i = 0; i < NoPorts; i++) begin
      `ACE_ASSIGN_REQ_STRUCT(ccu_req_o[i],core_req_i[i])
      `ACE_ASSIGN_RESP_STRUCT(core_resp_o[i],ccu_resp_i[i])
   end

endmodule // ccu_dispatch
