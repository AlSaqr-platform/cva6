// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Florian Zaruba, ETH Zurich
// Date: 13.10.2017
// Description: Nonblocking private L1 dcache


module std_nbdcache
  import std_cache_pkg::*;
  import ariane_pkg::*;
#(
    parameter config_pkg::cva6_cfg_t CVA6Cfg = config_pkg::cva6_cfg_empty,
    parameter int unsigned NumPorts = 4,
    parameter VLD_SRAM_SIM_INIT = "none",
    parameter type axi_req_t = logic,
    parameter type axi_rsp_t = logic
) (
    input logic clk_i,  // Clock
    input logic rst_ni,  // Asynchronous reset active low
    // Cache management
    input logic enable_i,  // from CSR
    input logic flush_i,  // high until acknowledged
    output logic flush_ack_o,  // send a single cycle acknowledge signal when the cache is flushed
    output logic miss_o,  // we missed on a LD/ST
    output logic busy_o,
    input logic stall_i,  // stall new memory requests
    input logic init_ni,
    output logic ongoing_write_o,
    // AMOs
    input amo_req_t amo_req_i,
    output amo_resp_t amo_resp_o,
    // Request ports
    input dcache_req_i_t [NumPorts-1:0] req_ports_i,  // request ports
    output dcache_req_o_t [NumPorts-1:0] req_ports_o,  // request ports
    // Cache AXI refill port
    output axi_req_t axi_data_o,
    input axi_rsp_t axi_data_i,
    output axi_req_t axi_bypass_o,
    input axi_rsp_t axi_bypass_i,
    output ariane_ace::snoop_resp_t snoop_port_o,
    input  ariane_ace::snoop_req_t snoop_port_i
);

  import std_cache_pkg::*;

  // -------------------------------
  // Controller <-> Arbiter
  // -------------------------------
  // 1. Miss handler
  // 2. Snoop
  // 3. PTW
  // 4. Load Unit
  // 5. Accelerator
  // 6. Store unit
  logic        [          NumPorts+1:0][  DCACHE_SET_ASSOC-1:0] req;
  logic        [          NumPorts+1:0][DCACHE_INDEX_WIDTH-1:0] addr;
  logic        [          NumPorts+1:0]                         gnt;
  cache_line_t [  DCACHE_SET_ASSOC-1:0]                         rdata;
  logic        [          NumPorts+1:0][  DCACHE_TAG_WIDTH-1:0] tag;

  cache_line_t [          NumPorts+1:0]                         wdata;
  logic        [          NumPorts+1:0]                         we;
  cl_be_t      [          NumPorts+1:0]                         be;
  logic        [  DCACHE_SET_ASSOC-1:0]                         hit_way;
  logic        [  DCACHE_SET_ASSOC-1:0]                         dirty_way;
  logic        [  DCACHE_SET_ASSOC-1:0]                         shared_way;
  // -------------------------------
  // Controller <-> Miss unit
  // -------------------------------
  logic        [            NumPorts:0]                         busy; // one per port + snoop
  logic        [          NumPorts-1:0][                  55:0] mshr_addr;
  logic        [          NumPorts-1:0]                         mshr_addr_matches;
  logic        [          NumPorts-1:0]                         mshr_index_matches;
  logic        [                  63:0]                         critical_word;
  logic                                                         critical_word_valid;

  logic        [          NumPorts-1:0][ $bits(miss_req_t)-1:0] miss_req;
  logic        [          NumPorts-1:0]                         miss_gnt;
  logic                                                         miss_write_done;
  logic        [          NumPorts-1:0]                         active_serving;
  logic                                                         flushing;
  logic                                                         serving_amo;
  logic        [                  63:0]                         serving_amo_addr;

  logic        [          NumPorts-1:0]                         bypass_gnt;
  logic        [          NumPorts-1:0]                         bypass_valid;
  logic        [          NumPorts-1:0][                  63:0] bypass_data;

  logic                                                         invalidate;
  logic        [                  63:0]                         invalidate_addr;
  // -------------------------------
  // Arbiter <-> Datram,
  // -------------------------------
  logic        [  DCACHE_SET_ASSOC-1:0]                         req_ram;
  logic        [DCACHE_INDEX_WIDTH-1:0]                         addr_ram;
  logic                                                         we_ram;
  cache_line_t                                                  wdata_ram;
  cache_line_t [  DCACHE_SET_ASSOC-1:0]                         rdata_ram;
  cl_be_t                                                       be_ram;
  vldrty_t     [  DCACHE_SET_ASSOC-1:0]                         be_valid_dirty_ram;

  // Busy signals
  logic                                                         miss_handler_busy;

  readshared_done_t                                             readshared_done;
  logic        [          NumPorts-1:0]                         updating_cache;

  assign busy_o = |busy | miss_handler_busy;

  // ------------------
  // Cache Controller
  // ------------------

  snoop_cache_ctrl i_snoop_cache_ctrl (
    .bypass_i             ( ~enable_i             ),
    .busy_o               ( busy              [0] ),
    // from ACE
    .snoop_port_i         ( snoop_port_i          ),
    .snoop_port_o         ( snoop_port_o          ),
    // to SRAM array
    .req_o                ( req               [1] ),
    .addr_o               ( addr              [1] ),
    .gnt_i                ( gnt               [1] ),
    .data_i               ( rdata                 ),
    .tag_o                ( tag               [1] ),
    .data_o               ( wdata             [1] ),
    .we_o                 ( we                [1] ),
    .be_o                 ( be                [1] ),
    .hit_way_i            ( hit_way               ),
    .dirty_way_i          ( dirty_way             ),
    .shared_way_i         ( shared_way            ),
    .invalidate_o         ( invalidate            ),
    .invalidate_addr_o    ( invalidate_addr       ),
    .readshared_done_o    ( readshared_done       ),
    .updating_cache_i     ( |updating_cache       ),
    .flushing_i           ( flushing              ),
    .amo_valid_i          ( serving_amo           ),
    .amo_addr_i           ( serving_amo_addr      ),
    .clean_invalid_hit_o  (                       ), // Not connected
    .clean_invalid_miss_o (                       ), // Not connected
    .*
  );

  generate
    for (genvar i = 0; i < NumPorts; i++) begin : master_ports
      cache_ctrl #(
          .CVA6Cfg(CVA6Cfg)
      ) i_cache_ctrl (
          .bypass_i  (~enable_i),
          .busy_o    (busy[i+1]),
          .hit_o     (),
          .unique_o  (),
          .stall_i   (stall_i | flush_i),
          // from core
          .req_port_i(req_ports_i[i]),
          .req_port_o(req_ports_o[i]),
          // to SRAM array
          .req_o     (req[i+2]),
          .addr_o    (addr[i+2]),
          .gnt_i     (gnt[i+2]),
          .data_i    (rdata),
          .tag_o     (tag[i+2]),
          .data_o    (wdata[i+2]),
          .we_o      (we[i+2]),
          .be_o      (be[i+2]),
          .hit_way_i (hit_way),
          .shared_way_i(shared_way),

          .miss_req_o           (miss_req[i]),
          .miss_gnt_i           (miss_gnt[i]),
          .miss_write_done_i    (miss_write_done),
          .active_serving_i     (active_serving[i]),
          .critical_word_i      (critical_word),
          .critical_word_valid_i(critical_word_valid),
          .bypass_gnt_i         (bypass_gnt[i]),
          .bypass_valid_i       (bypass_valid[i]),
          .bypass_data_i        (bypass_data[i]),

          .mshr_addr_o         (mshr_addr[i]),
          .mshr_addr_matches_i (mshr_addr_matches[i]),
          .mshr_index_matches_i(mshr_index_matches[i]),

          .snoop_invalidate_i     (invalidate),
          .snoop_invalidate_addr_i(invalidate_addr),
          .readshared_done_i      (readshared_done),
          .updating_cache_o       (updating_cache[i]),
          .*
      );
    end
  endgenerate

  // ------------------
  // Miss Handling Unit
  // ------------------
  miss_handler #(
      .CVA6Cfg  (CVA6Cfg),
      .NR_PORTS (NumPorts),
      .axi_req_t(axi_req_t),
      .axi_rsp_t(axi_rsp_t)
  ) i_miss_handler (
      .busy_o               (miss_handler_busy),
      .flush_i              (flush_i),
      .busy_i               (|busy),
      // AMOs
      .amo_req_i            (amo_req_i),
      .amo_resp_o           (amo_resp_o),
      .snoop_invalidate_i   (invalidate),
      .snoop_invalidate_addr_i(invalidate_addr),
      .miss_req_i           (miss_req),
      .miss_gnt_o           (miss_gnt),
      .miss_write_done_o    (miss_write_done),
      .bypass_gnt_o         (bypass_gnt),
      .bypass_valid_o       (bypass_valid),
      .bypass_data_o        (bypass_data),
      .critical_word_o      (critical_word),
      .critical_word_valid_o(critical_word_valid),
      .mshr_addr_i          (mshr_addr),
      .mshr_addr_matches_o  (mshr_addr_matches),
      .mshr_index_matches_o (mshr_index_matches),
      .active_serving_o     (active_serving),
      .flushing_o           (flushing),
      .serving_amo_o        (serving_amo),
      .serving_amo_addr_o   (serving_amo_addr),
      .req_o                (req[0]),
      .addr_o               (addr[0]),
      .data_i               (rdata),
      .be_o                 (be[0]),
      .data_o               (wdata[0]),
      .we_o                 (we[0]),
      .axi_bypass_o,
      .axi_bypass_i,
      .axi_data_o,
      .axi_data_i,
      .*
  );

  assign tag[0] = '0;

  // --------------
  // Memory Arrays
  // --------------
  for (genvar i = 0; i < DCACHE_SET_ASSOC; i++) begin : sram_block
    sram #(
        .DATA_WIDTH(DCACHE_LINE_WIDTH),
        .NUM_WORDS (DCACHE_NUM_WORDS)
    ) data_sram (
        .req_i  (req_ram[i]),
        .rst_ni (rst_ni),
        .we_i   (we_ram),
        .addr_i (addr_ram[DCACHE_INDEX_WIDTH-1:DCACHE_BYTE_OFFSET]),
        .wuser_i('0),
        .wdata_i(wdata_ram.data),
        .be_i   (be_ram.data),
        .ruser_o(),
        .rdata_o(rdata_ram[i].data),
        .*
    );

    sram #(
        .DATA_WIDTH(DCACHE_TAG_WIDTH),
        .NUM_WORDS (DCACHE_NUM_WORDS)
    ) tag_sram (
        .req_i  (req_ram[i]),
        .rst_ni (rst_ni),
        .we_i   (we_ram),
        .addr_i (addr_ram[DCACHE_INDEX_WIDTH-1:DCACHE_BYTE_OFFSET]),
        .wuser_i('0),
        .wdata_i(wdata_ram.tag),
        .be_i   (be_ram.tag),
        .ruser_o(),
        .rdata_o(rdata_ram[i].tag),
        .*
    );

  end

  // ----------------
  // Valid/Dirty Regs
  // ----------------

  vldrty_t [DCACHE_SET_ASSOC-1:0] dirty_wdata, dirty_rdata;

  for (genvar i = 0; i < DCACHE_SET_ASSOC; i++) begin
    assign dirty_wdata[i]               = '{dirty: wdata_ram.dirty, valid: wdata_ram.valid, shared: wdata_ram.shared};
    assign rdata_ram[i].dirty           = dirty_rdata[i].dirty;
    assign rdata_ram[i].valid           = dirty_rdata[i].valid;
    assign rdata_ram[i].shared          = dirty_rdata[i].shared;
    assign be_valid_dirty_ram[i].valid  = be_ram.vldrty[i].valid;
    assign be_valid_dirty_ram[i].dirty  = be_ram.vldrty[i].dirty;
    assign be_valid_dirty_ram[i].shared = be_ram.vldrty[i].shared;
  end

  sram #(
      .SIM_INIT(VLD_SRAM_SIM_INIT),
      .USER_WIDTH(1),
      .DATA_WIDTH(DCACHE_SET_ASSOC * $bits(vldrty_t)),
      .BYTE_WIDTH(1),
      .NUM_WORDS (DCACHE_NUM_WORDS)
  ) valid_dirty_sram (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .req_i  (|req_ram),
      .we_i   (we_ram),
      .addr_i (addr_ram[DCACHE_INDEX_WIDTH-1:DCACHE_BYTE_OFFSET]),
      .wuser_i('0),
      .wdata_i(dirty_wdata),
      .be_i   (be_valid_dirty_ram),
      .ruser_o(),
      .rdata_o(dirty_rdata)
  );

  // ------------------------------------------------
  // Tag Comparison and memory arbitration
  // ------------------------------------------------
  tag_cmp #(
      .CVA6Cfg         (CVA6Cfg),
      .NR_PORTS        (NumPorts + 2),
      .ADDR_WIDTH      (DCACHE_INDEX_WIDTH),
      .DCACHE_SET_ASSOC(DCACHE_SET_ASSOC)
  ) i_tag_cmp (
      .req_i    (req),
      .gnt_o    (gnt),
      .addr_i   (addr),
      .wdata_i  (wdata),
      .we_i     (we),
      .be_i     (be),
      .rdata_o  (rdata),
      .tag_i    (tag),
      .hit_way_o(hit_way),

      .req_o  (req_ram),
      .addr_o (addr_ram),
      .wdata_o(wdata_ram),
      .we_o   (we_ram),
      .be_o   (be_ram),
      .rdata_i(rdata_ram),
      .*
  );

  for (genvar j = 0; j < DCACHE_SET_ASSOC; j++) begin
    assign dirty_way[j] = |rdata_ram[j].dirty;
    assign shared_way[j] = rdata_ram[j].shared;
  end

  //pragma translate_off
  initial begin
    assert (DCACHE_LINE_WIDTH / CVA6Cfg.AxiDataWidth inside {2, 4, 8, 16})
    else $fatal(1, "Cache line size needs to be a power of two multiple of AxiDataWidth");
  end
  //pragma translate_on
endmodule
