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
// Date: 06.10.2017
// Description: Performance counters


module perf_counters import ariane_pkg::*; #(
  parameter int unsigned                NumPorts      = 3,    // number of miss ports
  parameter int unsigned                NumExtPerfEvts = 0
) (
  input  logic                                    clk_i,
  input  logic                                    rst_ni,
  input  logic                                    debug_mode_i, // debug mode
  // SRAM like interface
  input  logic [11:0]                             addr_i,   // read/write address (up to 6 counters possible)
  input  logic                                    we_i,     // write enable
  input  riscv::xlen_t                            data_i,   // data to write
  output riscv::xlen_t                            data_o,   // data to read
  // from commit stage
  input  scoreboard_entry_t [NR_COMMIT_PORTS-1:0] commit_instr_i,     // the instruction we want to commit
  input  logic [NR_COMMIT_PORTS-1:0]              commit_ack_i,       // acknowledge that we are indeed committing
  // from L1 caches
  input  logic                                    l1_icache_miss_i,
  input  logic                                    l1_dcache_miss_i,
  // from MMU
  input  logic                                    itlb_miss_i,
  input  logic                                    dtlb_miss_i,
  // from issue stage
  input  logic                                    sb_full_i,
  // from frontend
  input  logic                                    if_empty_i,
  // from PC Gen
  input  exception_t                              ex_i,
  input  logic                                    eret_i,
  input  bp_resolve_t                             resolved_branch_i,
  // for newly added events
  input  logic [NumExtPerfEvts-1:0]               perf_evt_i,
  input  logic                                    dc_hit_i,
  input  logic                                    dc_write_hit_unique_i,
  input  logic                                    dc_write_hit_shared_i,
  input  logic                                    dc_write_miss_i,
  input  logic                                    dc_clean_invalid_hit_i,
  input  logic                                    dc_clean_invalid_miss_i,
  input  logic                                    dc_flushing_i,
  input  logic                                    in_snoop_read_once_i,
  input  logic                                    in_snoop_read_shrd_i,
  input  logic                                    in_snoop_read_clean_i,
  input  logic                                    in_snoop_read_no_sd_i,
  input  logic                                    in_snoop_read_uniq_i,
  input  logic                                    in_snoop_clean_shrd_i,
  input  logic                                    in_snoop_clean_invld_i,
  input  logic                                    in_snoop_clean_uniq_i,
  input  logic                                    in_snoop_make_invld_i,
  input  logic                                    out_snoop_read_once_i,
  input  logic                                    out_snoop_read_shrd_i,
  input  logic                                    out_snoop_read_uniq_i,
  input  logic                                    out_snoop_read_nosnoop_i,
  input  logic                                    out_snoop_clean_uniq_i,
  input  logic                                    out_snoop_wr_uniq_i,
  input  logic                                    out_snoop_wr_nosnoop_i,
  input  logic                                    out_snoop_wr_back_i,
  input  exception_t                              branch_exceptions_i,  //Branch exceptions->execute unit-> branch_exception_o
  input  icache_dreq_i_t                          l1_icache_access_i,
  input  dcache_req_i_t[2:0]                      l1_dcache_access_i,
  input  logic [NumPorts-1:0][DCACHE_SET_ASSOC-1:0]miss_vld_bits_i,  //For Cache eviction (3ports-LOAD,STORE,PTW)
  input  logic                                    i_tlb_flush_i,
  input  logic                                    stall_issue_i,  //stall-read operands
  input  logic[31:0]                              mcountinhibit_i
);

  logic [63:0] generic_counter_d[MHPMCounterNum:1];
  logic [63:0] generic_counter_q[MHPMCounterNum:1];

  //internal signal to keep track of exception
  logic read_access_exception,update_access_exception;

  logic events[MHPMCounterNum:1];
  //internal signal for  MUX select line input
  localparam int unsigned MHPMEventWidth = 6;
  logic [MHPMEventWidth-1:0] mhpmevent_d[MHPMCounterNum:1];
  logic [MHPMEventWidth-1:0] mhpmevent_q[MHPMCounterNum:1];

  logic [NR_COMMIT_PORTS-1:0] load_event;
  logic [NR_COMMIT_PORTS-1:0] store_event;
  logic [NR_COMMIT_PORTS-1:0] branch_event;
  logic [NR_COMMIT_PORTS-1:0] call_event;
  logic [NR_COMMIT_PORTS-1:0] return_event;
  logic [NR_COMMIT_PORTS-1:0] int_event;
  logic [NR_COMMIT_PORTS-1:0] fp_event;

  //Multiplexer
   always_comb begin : Mux
      events[MHPMCounterNum:1]='{default:0};
      load_event = '0;
      store_event = '0;
      branch_event = '0;
      call_event = '0;
      return_event = '0;
      int_event = '0;
      fp_event = '0;

      for (int unsigned j = 0; j < NR_COMMIT_PORTS; j++) begin
         load_event[j] = commit_ack_i[j] & (commit_instr_i[j].fu == LOAD);
         store_event[j] = commit_ack_i[j] & (commit_instr_i[j].fu == STORE);
         branch_event[j] = commit_ack_i[j] & (commit_instr_i[j].fu == CTRL_FLOW);
         call_event[j] = commit_ack_i[j] & (commit_instr_i[j].fu == CTRL_FLOW && (commit_instr_i[j].op == ADD || commit_instr_i[j].op == JALR) && (commit_instr_i[j].rd == 'd1 || commit_instr_i[j].rd == 'd5));
         return_event[j] = commit_ack_i[j] & (commit_instr_i[j].op == JALR && commit_instr_i[j].rd == 'd0);
         int_event[j] = commit_ack_i[j] & (commit_instr_i[j].fu == ALU || commit_instr_i[j].fu == MULT);
         fp_event[j] = commit_ack_i[j] & (commit_instr_i[j].fu == FPU || commit_instr_i[j].fu == FPU_VEC);
      end

      for(int unsigned i = 1; i <= MHPMCounterNum; i++) begin
        case(mhpmevent_q[i])
           6'b000000 : events[i] = 0;
           6'b000001 : events[i] = l1_icache_miss_i;//L1 I-Cache misses
           6'b000010 : events[i] = l1_dcache_miss_i;//L1 D-Cache misses
           6'b000011 : events[i] = itlb_miss_i;//ITLB misses
           6'b000100 : events[i] = dtlb_miss_i;//DTLB misses
           6'b000101 : events[i] = |load_event;
           6'b000110 : events[i] = |store_event;
           6'b000111 : events[i] = ex_i.valid;//Exceptions
           6'b001000 : events[i] = eret_i;//Exception handler returns
           6'b001001 : events[i] = |branch_event;
           6'b001010 : events[i] = resolved_branch_i.valid && resolved_branch_i.is_mispredict;//Branch mispredicts
           6'b001011 : events[i] = branch_exceptions_i.valid;//Branch exceptions
                   // The standard software calling convention uses register x1 to hold the return address on a call
                   // the unconditional jump is decoded as ADD op
           6'b001100 : events[i] = |call_event;
           6'b001101 : events[i] = |return_event;
           6'b001110 : events[i] = sb_full_i;//MSB Full
           6'b001111 : events[i] = if_empty_i;//Instruction fetch Empty
           6'b010000 : events[i] = l1_icache_access_i.req;//L1 I-Cache accesses
           6'b010001 : events[i] = l1_dcache_access_i[0].data_req || l1_dcache_access_i[1].data_req || l1_dcache_access_i[2].data_req;//L1 D-Cache accesses
           6'b010010 : events[i] = (l1_dcache_miss_i && miss_vld_bits_i[0] == 8'hFF) || (l1_dcache_miss_i && miss_vld_bits_i[1] == 8'hFF) || (l1_dcache_miss_i && miss_vld_bits_i[2] == 8'hFF);//eviction
           6'b010011 : events[i] = i_tlb_flush_i;//I-TLB flush
           6'b010100 : events[i] = |int_event;
           6'b010101 : events[i] = |fp_event;
           6'b010110 : events[i] = stall_issue_i;//Pipeline bubbles
           6'b010111 : events[i] = in_snoop_read_once_i;
           6'b011000 : events[i] = in_snoop_read_shrd_i;
           6'b011001 : events[i] = in_snoop_read_clean_i;
           6'b011010 : events[i] = in_snoop_read_no_sd_i;
           6'b011011 : events[i] = in_snoop_read_uniq_i;
           6'b011110 : events[i] = in_snoop_clean_shrd_i;
           6'b011111 : events[i] = in_snoop_clean_invld_i;
           6'b100000 : events[i] = in_snoop_clean_uniq_i;
           6'b100001 : events[i] = in_snoop_make_invld_i;
           6'b100010 : events[i] = dc_hit_i;
           6'b100011 : events[i] = dc_write_hit_unique_i;
           6'b100100 : events[i] = dc_write_hit_shared_i;
           6'b100101 : events[i] = dc_write_miss_i;
           6'b100110 : events[i] = dc_clean_invalid_hit_i;
           6'b100111 : events[i] = dc_clean_invalid_miss_i;
           6'b101000 : events[i] = dc_flushing_i;
           6'b101001 : events[i] = out_snoop_read_once_i;
           6'b101010 : events[i] = out_snoop_read_shrd_i;
           6'b101011 : events[i] = out_snoop_read_uniq_i;
           6'b101100 : events[i] = out_snoop_read_nosnoop_i;
           6'b101101 : events[i] = out_snoop_clean_uniq_i;
           6'b101110 : events[i] = out_snoop_wr_uniq_i;
           6'b101111 : events[i] = out_snoop_wr_nosnoop_i;
           6'b110000 : events[i] = out_snoop_wr_back_i;
           default:   events[i] = ( (mhpmevent_q[i] > 6'b110000) && (mhpmevent_q[i] <= (6'b110000 + NumExtPerfEvts) ) ) ? (perf_evt_i[mhpmevent_q[i]-6'b110001]) : 1'b0;
         endcase
       end

    end

    typedef logic[11:0] csr_addr_t;

    always_comb begin : generic_counter
        generic_counter_d = generic_counter_q;
        data_o = 'b0;
        mhpmevent_d = mhpmevent_q;
	    read_access_exception =  1'b0;
	    update_access_exception =  1'b0;

      for(int unsigned i = 1; i <= MHPMCounterNum; i++) begin
         if ((!debug_mode_i) && (!we_i)) begin
             if ((events[i]) == 1 && (!mcountinhibit_i[i+2]))begin
                generic_counter_d[i] = generic_counter_q[i] + 1'b1;
             end
        end
      end

     //Read
     if( (addr_i >= csr_addr_t'(riscv::CSR_MHPM_COUNTER_3)) && (addr_i < ( csr_addr_t'(riscv::CSR_MHPM_COUNTER_3) + MHPMCounterNum)) ) begin
        if (riscv::XLEN == 32) begin
           data_o = generic_counter_q[addr_i-riscv::CSR_MHPM_COUNTER_3 + 1][31:0];
        end else begin
           data_o = generic_counter_q[addr_i-riscv::CSR_MHPM_COUNTER_3 + 1];
        end
     end else if( (addr_i >= csr_addr_t'(riscv::CSR_MHPM_COUNTER_3H)) && (addr_i < ( csr_addr_t'(riscv::CSR_MHPM_COUNTER_3H) + MHPMCounterNum)) ) begin
        if (riscv::XLEN == 32) begin
           data_o = generic_counter_q[addr_i-riscv::CSR_MHPM_COUNTER_3H + 1][63:32];
        end else begin
           read_access_exception = 1'b1;
        end
     end else if( (addr_i >= csr_addr_t'(riscv::CSR_MHPM_EVENT_3)) && (addr_i < (csr_addr_t'(riscv::CSR_MHPM_EVENT_3) + MHPMCounterNum)) ) begin
         data_o = mhpmevent_q[addr_i-riscv::CSR_MHPM_EVENT_3 + 1] ;
     end else if( (addr_i >= csr_addr_t'(riscv::CSR_HPM_COUNTER_3)) && (addr_i < (csr_addr_t'(riscv::CSR_HPM_COUNTER_3) + MHPMCounterNum)) ) begin
        if(riscv::XLEN == 32) begin
           data_o = generic_counter_q[addr_i-riscv::CSR_HPM_COUNTER_3 + 1][31:0];
        end else begin
           data_o = generic_counter_q[addr_i-riscv::CSR_HPM_COUNTER_3 + 1];
        end
     end else if( (addr_i > csr_addr_t'(riscv::CSR_HPM_COUNTER_3H)) && (addr_i < (csr_addr_t'(riscv::CSR_HPM_COUNTER_3H) + MHPMCounterNum)) ) begin
        if(riscv::XLEN == 32) begin
           data_o = generic_counter_q[addr_i-riscv::CSR_MHPM_COUNTER_3H + 1][63:32];
        end else begin
           read_access_exception = 1'b1;
        end
     end

     //Write
     if(we_i) begin
        if( (addr_i >= csr_addr_t'(riscv::CSR_MHPM_COUNTER_3)) && (addr_i < (csr_addr_t'(riscv::CSR_MHPM_COUNTER_3) + MHPMCounterNum)) ) begin
           if (riscv::XLEN == 32) begin
              generic_counter_d[addr_i-riscv::CSR_MHPM_COUNTER_3 + 1][31:0] = data_i;
           end else begin
              generic_counter_d[addr_i-riscv::CSR_MHPM_COUNTER_3 + 1] = data_i;
           end
        end else if( (addr_i >= csr_addr_t'(riscv::CSR_MHPM_COUNTER_3H)) && (addr_i < (csr_addr_t'(riscv::CSR_MHPM_COUNTER_3H) + MHPMCounterNum)) ) begin
           if (riscv::XLEN == 32) begin
              generic_counter_d[addr_i-riscv::CSR_MHPM_COUNTER_3H + 1][63:32] = data_i;
           end else begin
              update_access_exception = 1'b1;
           end
        end else if( (addr_i >= csr_addr_t'(riscv::CSR_MHPM_EVENT_3)) && (addr_i < csr_addr_t'(riscv::CSR_MHPM_EVENT_3) + MHPMCounterNum) ) begin
            mhpmevent_d[addr_i-riscv::CSR_MHPM_EVENT_3 + 1] = data_i;
        end
     end
    end

  //Registers
  always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            generic_counter_q <= '{default:0};
            mhpmevent_q       <= '{default:0};
        end else begin
            generic_counter_q <= generic_counter_d;
            mhpmevent_q       <= mhpmevent_d;
       end
   end


    `ifndef VERILATOR
        initial begin
            assert ( (6'b110000 + NumExtPerfEvts) < 2**MHPMEventWidth ) else $error ("Too many NumExtPerfEvts");
        end
    `endif

endmodule
