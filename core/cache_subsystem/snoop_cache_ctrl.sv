// Copyright 2022 PlanV GmbH
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// Description: cache controller, driven by incoming snoop requests

module snoop_cache_ctrl import ariane_pkg::*; import std_cache_pkg::*; (
  input  logic                               clk_i, // Clock
  input  logic                               rst_ni, // Asynchronous reset active low
  input  logic                               stall_i,
  input  logic                               bypass_i, // enable cache
  output logic                               busy_o,
   // Snoop interface
  input  ariane_ace::snoop_req_t             snoop_port_i,
  output ariane_ace::snoop_resp_t            snoop_port_o,
   // SRAM interface
  output logic        [DCACHE_SET_ASSOC-1:0] req_o, // req is valid
  output logic      [DCACHE_INDEX_WIDTH-1:0] addr_o, // address into cache array
  input  logic                               gnt_i,
  output cache_line_t                        data_o,
  output cl_be_t                             be_o,
  output logic        [DCACHE_TAG_WIDTH-1:0] tag_o, //valid one cycle later
  input  cache_line_t [DCACHE_SET_ASSOC-1:0] data_i,
  output logic                               we_o,
  input  logic        [DCACHE_SET_ASSOC-1:0] hit_way_i,
  input  logic        [DCACHE_SET_ASSOC-1:0] dirty_way_i,
  input  logic        [DCACHE_SET_ASSOC-1:0] shared_way_i,
  // to/from miss handler
  output logic                               invalidate_o,
  output logic [63:0]                        invalidate_addr_o,
  input  logic                               flushing_i,
  input  logic                               amo_valid_i,
  input  logic [63:0]                        amo_addr_i,
  // to perf counters
  output logic                               clean_invalid_hit_o,
  output logic                               clean_invalid_miss_o,
  // to/from cache_ctrl
  input  logic                               updating_cache_i,
  output readshared_done_t                   readshared_done_o
);

  // pragma translate_off
  initial assert ((DCACHE_LINE_WIDTH == 128) && ($bits(snoop_port_o.cd.data) == 64)) else
    $error("Expected DCACHE_LINE_WIDTH == 128 (got %0d) and AXI_DATA_WITH == 64 (got %0d)", DCACHE_LINE_WIDTH, $bits(snoop_port_o.cd.data));
  // pragma translate_on


  typedef enum logic [2:0] {
    IDLE,            // 0
    WAIT_GNT,        // 1
    EVAL_FLAGS,      // 2
    UPDATE_SHARED,   // 3
    INVALIDATE,      // 4
    WAIT_SNOOP_PORT  // 5
  } state_t;

  state_t state_d, state_q;

  typedef struct packed {
    logic [DCACHE_INDEX_WIDTH-1:0] index;
    logic [DCACHE_TAG_WIDTH-1:0]   tag;
    logic [1:0]                    size;
  } mem_req_t;

  mem_req_t mem_req_d, mem_req_q;

  assign busy_o = (state_q != IDLE);
  assign tag_o = mem_req_d.tag;
  assign addr_o = mem_req_d.index;

  logic [DCACHE_SET_ASSOC-1:0]  hit_way_d, hit_way_q;
  logic [DCACHE_SET_ASSOC-1:0]  shared_way_d, shared_way_q;
  logic [DCACHE_SET_ASSOC-1:0]  dirty_way_d, dirty_way_q;

  logic [DCACHE_LINE_WIDTH-1:0] cache_data_d, cache_data_q;
  logic [DCACHE_LINE_WIDTH-1:0] cl_i;
  logic                         cacheline_word_sel_d, cacheline_word_sel_q;

  always_comb begin : way_select
    cl_i = '0;
    for (int unsigned i = 0; i < DCACHE_SET_ASSOC; i++)
      if (hit_way_i[i])
        cl_i = data_i[i].data;
  end

  logic dirty;

  assign dirty = |(dirty_way_i & hit_way_i);

  logic shared;

  assign shared = |(shared_way_i & hit_way_i);

  snoop_pkg::crresp_t cr_resp_d, cr_resp_q;

  snoop_pkg::acsnoop_t ac_snoop_d, ac_snoop_q;

  logic cr_done_q, cr_done_d;

  logic send_snoop_resp, snoop_port_done;

  logic snoop_port_busy_d, snoop_port_busy_q;

  logic update_shared;

  // FSM
  always_comb begin : cache_ctrl_fsm

    state_d = state_q;
    mem_req_d = mem_req_q;
    cache_data_d = cache_data_q;
    hit_way_d = hit_way_q;
    shared_way_d = shared_way_q;
    dirty_way_d = dirty_way_q;
    cr_resp_d = cr_resp_q;
    ac_snoop_d = ac_snoop_q;

    snoop_port_o.ac_ready = 1'b0;

    req_o = '0;

    update_shared = 1'b0;

    readshared_done_o = '0;

    invalidate_o      = 1'b0;
    invalidate_addr_o = {mem_req_q.tag, mem_req_q.index};

    clean_invalid_hit_o = 1'b0;
    clean_invalid_miss_o = 1'b0;

    send_snoop_resp = 1'b0;

    case (state_q)

      IDLE: begin
        cr_resp_d = '0;
        ac_snoop_d = '0;

        // we receive a snooping request
        if (snoop_port_i.ac_valid == 1'b1 && flushing_i == 1'b0 && !(amo_valid_i == 1'b1 && amo_addr_i[63:DCACHE_BYTE_OFFSET] == snoop_port_i.ac.addr[63:DCACHE_BYTE_OFFSET])) begin
          snoop_port_o.ac_ready = 1'b1;
          // save the request details
          mem_req_d.index = snoop_port_i.ac.addr[DCACHE_INDEX_WIDTH-1:0];
          mem_req_d.tag = snoop_port_i.ac.addr[DCACHE_INDEX_WIDTH+:DCACHE_TAG_WIDTH];
          mem_req_d.size = 2'b11;
          if (bypass_i) begin
            send_snoop_resp = 1'b1;
            state_d = snoop_port_done ? IDLE : WAIT_SNOOP_PORT;
          end
          else begin
            // invalidate request
            if (snoop_port_i.ac.snoop == snoop_pkg::CLEAN_INVALID ||
                snoop_port_i.ac.snoop == snoop_pkg::READ_SHARED ||
                snoop_port_i.ac.snoop == snoop_pkg::READ_ONCE ||
                snoop_port_i.ac.snoop == snoop_pkg::READ_UNIQUE) begin
              state_d = WAIT_GNT;
              ac_snoop_d = snoop_port_i.ac.snoop;
              // request the cache line (unless there is another cache controller which is uploading the cache content)
              req_o = !updating_cache_i ? '1 : '0;
            end
            // wrong request
            else begin
              send_snoop_resp = 1'b1;
              state_d = snoop_port_done ? IDLE : WAIT_SNOOP_PORT;
              cr_resp_d.error = 1'b1;
            end
          end
        end
      end

      WAIT_GNT: begin
        // request the cache line (unless there is another cache controller which is uploading the cache content)
        if (!updating_cache_i) begin
          req_o = '1;
          if (gnt_i)
            state_d = EVAL_FLAGS;
        end
      end

      EVAL_FLAGS: begin
        // keep the request to avoid interference from other cache controllers
        req_o = hit_way_i;
        hit_way_d = hit_way_i;
        shared_way_d = shared_way_i;
        dirty_way_d = dirty_way_i;
        if (|hit_way_i) begin
          cr_resp_d.dataTransfer = 1'b1;
          cr_resp_d.passDirty = 1'b0; //dirty;
          cache_data_d = cl_i;
          case (ac_snoop_q)
            snoop_pkg::CLEAN_INVALID: begin
              cr_resp_d.dataTransfer = dirty;
              cr_resp_d.passDirty = dirty;
              cr_resp_d.isShared = 1'b0;
              clean_invalid_hit_o = 1'b1;
              invalidate_o = 1'b1;
              if (gnt_i) begin
                send_snoop_resp = 1'b1;
                state_d = snoop_port_done ? IDLE : WAIT_SNOOP_PORT;
              end else begin
                state_d = INVALIDATE;
              end
            end
            snoop_pkg::READ_ONCE: begin
              cr_resp_d.isShared = shared;
              send_snoop_resp = 1'b1;
              state_d = snoop_port_done ? IDLE : WAIT_SNOOP_PORT;
            end
            snoop_pkg::READ_SHARED: begin
              cr_resp_d.isShared = 1'b1;
              update_shared = 1'b1;
              if (gnt_i) begin
                send_snoop_resp = 1'b1;
                state_d = snoop_port_done ? IDLE : WAIT_SNOOP_PORT;
                readshared_done_o.valid = 1'b1;
                readshared_done_o.addr = {mem_req_q.tag, mem_req_q.index};
              end else begin
                state_d = UPDATE_SHARED;
              end
            end
            default : begin // snoop_pkg::READ_UNIQUE
              cr_resp_d.passDirty = dirty;
              cr_resp_d.isShared = 1'b0;
              state_d = INVALIDATE;
              assert (ac_snoop_q == snoop_pkg::READ_UNIQUE) else
                $error("Unexpected snoop type");
            end
          endcase
        end
        // Miss
        else begin
          cr_resp_d.dataTransfer = 1'b0;
          cr_resp_d.passDirty = 1'b0;
          cr_resp_d.isShared = 1'b0;
          send_snoop_resp = 1'b1;
          state_d = snoop_port_done ? IDLE : WAIT_SNOOP_PORT;
          clean_invalid_miss_o = (ac_snoop_q == snoop_pkg::CLEAN_INVALID);
        end
      end

      UPDATE_SHARED: begin
        req_o = hit_way_q;
        update_shared = 1'b1;
        if (gnt_i) begin
          send_snoop_resp = 1'b1;
          state_d = snoop_port_done ? IDLE : WAIT_SNOOP_PORT;
          readshared_done_o.valid = 1'b1;
          readshared_done_o.addr = {mem_req_q.tag, mem_req_q.index};
        end
      end

      INVALIDATE: begin
        req_o = hit_way_q;
        // signal invalidate to the miss_handler
        // we are not blocked by the miss_handler here
        invalidate_o = 1'b1;
        if (gnt_i) begin
          send_snoop_resp = 1'b1;
          state_d = snoop_port_done ? IDLE : WAIT_SNOOP_PORT;
        end
      end

      WAIT_SNOOP_PORT: begin
        if (snoop_port_done) begin
          state_d = IDLE;
        end
      end

    endcase
  end

  always_comb begin : snoop_resp_port

    cacheline_word_sel_d = cacheline_word_sel_q;
    snoop_port_busy_d    = snoop_port_busy_q;
    cr_done_d            = cr_done_q;

    snoop_port_done      = 1'b0;

    snoop_port_o.cr_valid = 1'b0;
    snoop_port_o.cd_valid = 1'b0;

    snoop_port_o.cr_resp = cr_resp_q;

    snoop_port_o.cd.data = cacheline_word_sel_q ? cache_data_q[127:64] : cache_data_q[63:0];
    snoop_port_o.cd.last = cacheline_word_sel_q;

    if (snoop_port_busy_q) begin
      snoop_port_busy_d     = 1'b1;
      snoop_port_o.cr_valid = !cr_done_q;
      if (cr_resp_q.dataTransfer) begin
        snoop_port_o.cd_valid = 1'b1;
        cacheline_word_sel_d  = snoop_port_i.cd_ready || cacheline_word_sel_q;
        cr_done_d             = snoop_port_i.cr_ready || cr_done_q;
        if (snoop_port_i.cd_ready && cacheline_word_sel_q) begin
          cacheline_word_sel_d = 1'b0;
          snoop_port_busy_d    = 1'b0;
          cr_done_d            = 1'b0;
          snoop_port_done      = 1'b1;
        end
      end else if (snoop_port_i.cr_ready) begin
        cacheline_word_sel_d = 1'b0;
        snoop_port_busy_d    = 1'b0;
        cr_done_d            = 1'b0;
        snoop_port_done      = 1'b1;
      end
    end else if (send_snoop_resp) begin
      snoop_port_busy_d = 1'b1;
    end
  end

  always_comb begin : cache_update
    we_o = '0;
    be_o = '0;
    data_o = '0;
    if (update_shared) begin
      we_o = 1'b1;
      for (int unsigned i = 0; i < DCACHE_SET_ASSOC; i++) begin
        if (hit_way_d[i]) be_o.vldrty[i].shared = 1'b1;
      end
      // change shared the state
      data_o.shared = 1'b1;
    end else if (invalidate_o) begin
      we_o = 1'b1;
      for (int unsigned i = 0; i < DCACHE_SET_ASSOC; i++) begin
        if (hit_way_d[i]) be_o.vldrty[i] = '1;
      end
      data_o.dirty = '0;
      data_o.valid = 1'b0;
      data_o.shared = 1'b0;
    end
  end

  // Registers

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (~rst_ni) begin
      state_q <= IDLE;
      mem_req_q <= '0;
      cache_data_q <= '0;
      cacheline_word_sel_q <= 1'b0;
      hit_way_q <= '0;
      shared_way_q <= '0;
      dirty_way_q <= '0;
      cr_resp_q <= '0;
      ac_snoop_q <= '0;
      cr_done_q  <= '0;
      snoop_port_busy_q <= '0;
    end else begin
      state_q <= state_d;
      mem_req_q <= mem_req_d;
      cache_data_q <= cache_data_d;
      cacheline_word_sel_q <= cacheline_word_sel_d;
      hit_way_q <= hit_way_d;
      shared_way_q <= shared_way_d;
      dirty_way_q <= dirty_way_d;
      cr_resp_q <= cr_resp_d;
      ac_snoop_q <= ac_snoop_d;
      cr_done_q <= cr_done_d;
      snoop_port_busy_q <= snoop_port_busy_d;
    end
  end

endmodule
