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
`include "common_cells/assertions.svh"
`include "common_cells/registers.svh"

module ccu_dispatch
 import axi_pkg::*;
 import ariane_pkg::*;
 import ace_pkg::*;
 #(
  parameter int  NoPorts = '0,
  parameter int  AxiAddrWidth = 64,
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

   localparam int unsigned DCacheByteOffset = $clog2(ariane_pkg::DCACHE_LINE_WIDTH/8);

   typedef struct packed {
      logic                    valid;
      logic [AxiAddrWidth-1:0] start_addr;
      logic [AxiAddrWidth-1:0] end_addr;
   } trx_t;

   typedef struct packed {
      trx_t read;
      trx_t write;
   } ax_trx_t;

   ax_trx_t [NoPorts-1:0] inflight_trx_d, inflight_trx_q;
   logic [NoPorts-1:0] aw_lock_d, aw_lock_q;
   logic [NoPorts-1:0] ar_lock_d, ar_lock_q;

   always_comb begin : lock_ax_valids
      for (int p = 0; p < NoPorts; p++) begin : check_incoming_reqs
         aw_lock_d[p] = ccu_req_o[p].aw_valid & ~ccu_resp_i[p].aw_ready;
         ar_lock_d[p] = ccu_req_o[p].ar_valid & ~ccu_resp_i[p].ar_ready;
      end
   end

   always_comb begin : comb_check
      inflight_trx_d = inflight_trx_q;
      for (int i = 0; i < NoPorts; i++) begin : check_incoming_reqs
         logic [NoPorts-1:0] w_overlap;
         logic [NoPorts-1:0] aw_valids;
         logic [NoPorts-1:0] r_overlap;
         logic [NoPorts-1:0] ar_valids;
         ax_trx_t to_open_trx;
         w_overlap = '0;
         aw_valids = '0;
         r_overlap = '0;
         ar_valids = '0;
         to_open_trx = '0;
         ccu_req_o[i] = core_req_i[i];
         core_resp_o[i] = ccu_resp_i[i];
         for(int k = 0; k < NoPorts ; k++) begin
            aw_valids[k] = aw_lock_q[i] ? 1'b0 : ( |aw_lock_q ? 1'b1 : (core_req_i[k].aw_valid & (k<i)) );
            ar_valids[k] = ar_lock_q[i] ? 1'b0 : ( |ar_lock_q ? 1'b1 : (core_req_i[k].ar_valid & (k<i)) );
         end
         if(core_req_i[i].aw_valid) begin : aw_req
            to_open_trx.write.start_addr = ( axi_pkg::aligned_addr(core_req_i[i].aw.addr,core_req_i[i].aw.size) >> DCacheByteOffset ) << DCacheByteOffset;
            to_open_trx.write.start_addr[AxiAddrWidth-1:ariane_pkg::DCACHE_INDEX_WIDTH] = '0;
            to_open_trx.write.end_addr = (( axi_pkg::aligned_addr(core_req_i[i].aw.addr,core_req_i[i].aw.size) + (axi_pkg::num_bytes(core_req_i[i].aw.size) * (core_req_i[i].aw.len + 1)) ) >> DCacheByteOffset ) << DCacheByteOffset;
            to_open_trx.write.end_addr[AxiAddrWidth-1:ariane_pkg::DCACHE_INDEX_WIDTH] = '0;
            for (int j = 0; j < NoPorts ; j++) begin
               if ( i == j ) begin
                  w_overlap[j] = inflight_trx_q[j].read.valid;
               end else begin
                  if(inflight_trx_q[j].write.valid) begin
                     w_overlap[j] = ace_pkg::check_collision(to_open_trx.write.start_addr, to_open_trx.write.end_addr, inflight_trx_q[j].write.start_addr, inflight_trx_q[j].write.end_addr);
                  end
                  if(inflight_trx_q[j].read.valid) begin
                     w_overlap[j] = ace_pkg::check_collision(to_open_trx.write.start_addr, to_open_trx.write.end_addr, inflight_trx_q[j].read.start_addr, inflight_trx_q[j].read.end_addr);
                  end
               end // else: !if( i == j )
            end // for (genvar j = 0; j < NoPorts ; j++)
            ccu_req_o[i].aw_valid = core_req_i[i].aw_valid & ~(|w_overlap) & ~(|aw_valids) & ~(|ar_valids);
            core_resp_o[i].aw_ready = ccu_resp_i[i].aw_ready & ~(|w_overlap);
            if(ccu_resp_i[i].aw_ready & ccu_req_o[i].aw_valid) begin
               inflight_trx_d[i].write = to_open_trx.write;
               inflight_trx_d[i].write.valid = 1'b1;
            end
         end // if (core_req_i[i].aw_valid)
         if(ccu_resp_i[i].b_valid & ccu_req_o[i].b_ready) begin
            inflight_trx_d[i].write.valid = 1'b0;
         end
         if(core_req_i[i].ar_valid) begin : ar_req
            to_open_trx.read.start_addr = ( axi_pkg::aligned_addr(core_req_i[i].ar.addr,core_req_i[i].ar.size) >> DCacheByteOffset ) << DCacheByteOffset;
            to_open_trx.read.start_addr[AxiAddrWidth-1:ariane_pkg::DCACHE_INDEX_WIDTH] = '0;
            to_open_trx.read.end_addr = (( axi_pkg::aligned_addr(core_req_i[i].ar.addr,core_req_i[i].ar.size) + (axi_pkg::num_bytes(core_req_i[i].ar.size) * (core_req_i[i].ar.len + 1)) ) >> DCacheByteOffset ) << DCacheByteOffset;
            to_open_trx.read.end_addr[AxiAddrWidth-1:ariane_pkg::DCACHE_INDEX_WIDTH] = '0;
            for (int j = 0; j < NoPorts ; j++) begin
               if ( i == j ) begin
                  r_overlap[j] = inflight_trx_q[j].write.valid;
               end else begin
                  if(inflight_trx_q[j].write.valid) begin
                     r_overlap[j] = ace_pkg::check_collision(to_open_trx.read.start_addr, to_open_trx.read.end_addr, inflight_trx_q[j].write.start_addr, inflight_trx_q[j].write.end_addr);
                  end
                  if(inflight_trx_q[j].read.valid) begin
                     r_overlap[j] = ace_pkg::check_collision(to_open_trx.read.start_addr, to_open_trx.read.end_addr, inflight_trx_q[j].read.start_addr, inflight_trx_q[j].read.end_addr);
                  end
               end // else: !if( i == j )
            end // for (genvar j = 0; j < NoPorts ; j++)
            ccu_req_o[i].ar_valid = core_req_i[i].ar_valid & ~(|r_overlap) & ~(|aw_valids) & ~(|ar_valids);
            core_resp_o[i].ar_ready = ccu_resp_i[i].ar_ready & ~(|r_overlap);
            if(ccu_resp_i[i].ar_ready & ccu_req_o[i].ar_valid) begin
               inflight_trx_d[i].read = to_open_trx.read;
               inflight_trx_d[i].read.valid = 1'b1;
            end
         end // if (core_req_i[i].aw_valid)
         if(ccu_resp_i[i].r_valid & ccu_req_o[i].r_ready & ccu_resp_i[i].r.last) begin
            inflight_trx_d[i].read.valid = 1'b0;
         end
      end // for (genvar i = 0; i < NoPorts; i++)
   end // always_comb

   `FF(inflight_trx_q,inflight_trx_d,'0,clk_i,rst_ni)
   `FF(aw_lock_q,aw_lock_d,'0,clk_i,rst_ni)
   `FF(ar_lock_q,ar_lock_d,'0,clk_i,rst_ni)

endmodule // ccu_dispatch
