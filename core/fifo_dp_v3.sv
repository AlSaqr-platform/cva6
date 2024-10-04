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
// Authors: Maicol Ciani, University of Bologna
//
/////////////////////////////////////////////////////////////////
//  ______                                    _   _            //
// | ___ \                                  | | | |            //
// | |_/ /___ __      __ ___  _ __  ___   __| | | |            //
// |  __// _ \\ \ /\ / // _ \| '__|/ _ \ / _` | | '_ \ | | | | //
// | |  | (_) |\ V  V /|  __/| |  |  __/| (_| | | |_) || |_| | //
// \_|   \___/  \_/\_/  \___||_|   \___| \__,_| |_.__/  \__, | //
//  _____  _             _    _____ ______  _____        __/ | //
// /  __ \| |           | |  |  __ \| ___ \|_   _|      |___/  //
// | /  \/| |__    __ _ | |_ | |  \/| |_/ /  | |               //
// | |    | '_ \  / _` || __|| | __ |  __/   | |               //
// | \__/\| | | || (_| || |_ | |_\ \| |      | |               //
//  \____/|_| |_| \__,_| \__| \____/\_|      \_/               //
//                                                             //
/////////////////////////////////////////////////////////////////

module fifo_dp_v3 #(
    parameter bit          FALL_THROUGH = 1'b1, // fifo is in fall-through mode
    parameter int unsigned DATA_WIDTH   = 32,   // default data width if the fifo is of type logic
    parameter int unsigned DEPTH        = 8,    // depth can be arbitrary from 0 to 2**32
    parameter type dtype                = logic [DATA_WIDTH-1:0],
    // DO NOT OVERWRITE THIS PARAMETER
    parameter int unsigned ADDR_DEPTH   = (DEPTH > 1) ? $clog2(DEPTH) : 1
)(
    input  logic  clk_i,            // Clock
    input  logic  rst_ni,           // Asynchronous reset active low
    input  logic  flush_i,          // flush the queue
    input  logic  testmode_i,       // test_mode to bypass clock gating
    // status flags
    output logic  full_o,           // queue is full
    output logic  empty_o,          // queue is empty
    output logic  [ADDR_DEPTH-1:0] usage_o,  // fill pointer
    // as long as the queue is not full we can push new data
    input  dtype  data_port_0_i,           // data to push into the queue (first port)
    input  logic  push_port_0_i,           // data is valid and can be pushed to the queue (first port)
    input  dtype  data_port_1_i,          // data to push into the queue (second port)
    input  logic  push_port_1_i,          // data is valid and can be pushed to the queue (second port)
    // as long as the queue is not empty we can pop new elements
    output dtype  data_o,           // output data
    input  logic  pop_i             // pop head from queue
);
    // local parameter
    // FIFO depth - handle the case of pass-through, synthesizer will do constant propagation
    localparam int unsigned FifoDepth = (DEPTH > 0) ? DEPTH : 1;
    // clock gating control
    logic gate_clock;
    // pointer to the read and write section of the queue
    logic [ADDR_DEPTH - 1:0] read_pointer_n, read_pointer_q, write_pointer_n, write_pointer_q;
    // keep a counter to keep track of the current queue status
    // this integer will be truncated by the synthesis tool
    logic [ADDR_DEPTH:0] status_cnt_n, status_cnt_q;
    // actual memory
    dtype [FifoDepth - 1:0] mem_n, mem_q;

    // Available space in the FIFO
    logic [ADDR_DEPTH:0] space_available;
    // Handle passes and pushes
    logic [ADDR_DEPTH:0] temp_space_available;

    assign usage_o = status_cnt_q[ADDR_DEPTH-1:0];
    assign space_available = FifoDepth - status_cnt_q;
    assign full_o = (space_available == 0);
    assign empty_o = (status_cnt_q == 0) & ~(FALL_THROUGH & (push_port_0_i || push_port_1_i));

    // read and write queue logic
    always_comb begin : read_write_comb
        // default assignment
        read_pointer_n  = read_pointer_q;
        write_pointer_n = write_pointer_q;
        status_cnt_n    = status_cnt_q;
        mem_n           = mem_q;
        gate_clock      = 1'b1;
        data_o          = mem_q[read_pointer_q];

        temp_space_available = space_available;

        // Default data_o assignment in case of pass-through
        data_o = mem_q[read_pointer_q];

        // Pass-through mode handling
        if (FALL_THROUGH && (status_cnt_q == 0)) begin
            // Both push_port_0_i and push_port_1_i are asserted
            if (push_port_0_i && push_port_1_i) begin
                // Pass through data_port_0_i
                data_o = data_port_0_i;
                // Store data_port_1_i in the FIFO
                if (temp_space_available >= 1) begin
                    mem_n[write_pointer_n] = data_port_1_i;
                    gate_clock = 1'b0;
                    write_pointer_n = write_pointer_n + 1;
                    if (write_pointer_n == FifoDepth[ADDR_DEPTH-1:0])
                        write_pointer_n = '0;
                    status_cnt_n = status_cnt_n + 1;
                    temp_space_available = temp_space_available - 1;
                end
            end
            // Only push_port_0_i is asserted
            else if (push_port_0_i) begin
                data_o = data_port_0_i;
            end
            // Only push_port_1_i is asserted
            else if (push_port_1_i) begin
                data_o = data_port_1_i;
            end
            else begin
                // No pushes; output remains unchanged
                data_o = mem_q[read_pointer_q];
            end

            // Handle pop in pass-through mode
            if (pop_i && status_cnt_q > 0) begin
                if (read_pointer_q == FifoDepth[ADDR_DEPTH-1:0] - 1)
                    read_pointer_n = '0;
                else
                    read_pointer_n = read_pointer_q + 1;
                status_cnt_n = status_cnt_n - 1;
            end
        end
        else begin
            // Normal FIFO operation
            // Process first push port
            if (push_port_0_i && (temp_space_available >= 1)) begin
                mem_n[write_pointer_n] = data_port_0_i;
                gate_clock = 1'b0;
                write_pointer_n = write_pointer_n + 1;
                if (write_pointer_n == FifoDepth[ADDR_DEPTH-1:0])
                    write_pointer_n = '0;
                status_cnt_n = status_cnt_n + 1;
                temp_space_available = temp_space_available - 1;
            end

            // Process second push port
            if (push_port_1_i && (temp_space_available >= 1)) begin
                mem_n[write_pointer_n] = data_port_1_i;
                gate_clock = 1'b0;
                write_pointer_n = write_pointer_n + 1;
                if (write_pointer_n == FifoDepth[ADDR_DEPTH-1:0])
                    write_pointer_n = '0;
                status_cnt_n = status_cnt_n + 1;
                temp_space_available = temp_space_available - 1;
            end

            // Handle pop logic
            if (pop_i && ~empty_o) begin
                if (read_pointer_q == FifoDepth[ADDR_DEPTH-1:0] - 1)
                    read_pointer_n = '0;
                else
                    read_pointer_n = read_pointer_q + 1;
                status_cnt_n   = status_cnt_n - 1;
            end
        end
    end

    // sequential process
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if(~rst_ni) begin
            read_pointer_q  <= '0;
            write_pointer_q <= '0;
            status_cnt_q    <= '0;
            mem_q           <= '0;
        end else begin
            if (flush_i) begin
                read_pointer_q  <= '0;
                write_pointer_q <= '0;
                status_cnt_q    <= '0;
                mem_q           <= '0;
            end else begin
                read_pointer_q  <= read_pointer_n;
                write_pointer_q <= write_pointer_n;
                status_cnt_q    <= status_cnt_n;
                if (!gate_clock)
                    mem_q <= mem_n;
            end
        end
    end

`ifndef SYNTHESIS
`ifndef COMMON_CELLS_ASSERTS_OFF
    initial begin
        assert (DEPTH >= 0)             else $error("DEPTH must be greater than or equal to 0.");
    end

    full_write : assert property(
        @(posedge clk_i) disable iff (~rst_ni) (full_o |-> (~push_port_0_i && ~push_port_1_i)))
        else $fatal (1, "Trying to push new data although the FIFO is full.");

    empty_read : assert property(
        @(posedge clk_i) disable iff (~rst_ni) (empty_o |-> ~pop_i))
        else $fatal (1, "Trying to pop data although the FIFO is empty.");
`endif
`endif

endmodule // fifo_v3
