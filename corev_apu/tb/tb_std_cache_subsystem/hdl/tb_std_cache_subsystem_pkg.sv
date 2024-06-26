// Description: test utilities for the standard Ariane cache subsystem.
// main package definition
package tb_std_cache_subsystem_pkg;
    import ariane_pkg::*;
    import snoop_test::*;
    import std_cache_pkg::*;

    `define WAIT_CYC(CLK, N) \
        repeat(N) @(posedge(CLK));

    // definitions for dcache request and response
    typedef enum {WR_REQ, RD_REQ, RD_RESP, WR_RESP, EVICT, READBACK, UNDEF} dcache_trans_t;

    // definitions for dcache management transactions
    typedef enum {FLUSH_REQ} dcache_mgmt_trans_t;

    // definitions for amo request and response
    typedef enum {AMO_WR_REQ, AMO_RD_REQ, AMO_RD_RESP} amo_trans_t;

    // enum for snoop type to get better view in waveform
    typedef enum logic [3:0] {
        READ_ONCE             = snoop_pkg::READ_ONCE,
        READ_SHARED           = snoop_pkg::READ_SHARED,
        READ_CLEAN            = snoop_pkg::READ_CLEAN,
        READ_NOT_SHARED_DIRTY = snoop_pkg::READ_NOT_SHARED_DIRTY,
        READ_UNIQUE           = snoop_pkg::READ_UNIQUE,
        CLEAN_SHARED          = snoop_pkg::CLEAN_SHARED,
        CLEAN_INVALID         = snoop_pkg::CLEAN_INVALID,
        CLEAN_UNIQUE          = snoop_pkg::CLEAN_UNIQUE,
        MAKE_INVALID          = snoop_pkg::MAKE_INVALID,
        DVM_COMPLETE          = snoop_pkg::DVM_COMPLETE,
        DVM_MESSAGE           = snoop_pkg::DVM_MESSAGE
    } acsnoop_enum_t;


    //--------------------------------------------------------------------------
    // Helper functions
    //--------------------------------------------------------------------------

    // define min and max functions
    let max(a,b) = (a > b) ? a : b;
    let min(a,b) = (a < b) ? a : b;

    // get tag from address
    function automatic logic [DCACHE_TAG_WIDTH-1:0] addr2tag (input logic[63:0] addr);
        return addr[DCACHE_TAG_WIDTH+DCACHE_INDEX_WIDTH-1:DCACHE_INDEX_WIDTH];
    endfunction

    // get index from address
    function automatic logic [DCACHE_INDEX_WIDTH-1:0] addr2index (input logic[63:0] addr);
        return addr[DCACHE_INDEX_WIDTH-1:0];
    endfunction

    // get mem_idx from address
    function automatic logic [DCACHE_INDEX_WIDTH-DCACHE_BYTE_OFFSET-1:0] addr2mem_idx (input logic[63:0] addr);
        return addr[DCACHE_INDEX_WIDTH-1:DCACHE_BYTE_OFFSET];
    endfunction

    // get address from index and tag
    function automatic logic [63:0] tag_index2addr (
        input logic [DCACHE_TAG_WIDTH-1:0]   tag,
        input logic [DCACHE_INDEX_WIDTH-1:0] index
    );
        return {tag, index};
    endfunction

    // update part of cache line with <data> at <offset>
    function automatic void update_cache_line (
        inout cache_line_t                  cache_line,
        input riscv::xlen_t                 data,
        input logic [(riscv::XLEN/8)-1:0]   be,
        input int unsigned                  offset // in units of data width
    );
        logic [riscv::XLEN-1:0]         data_mask;
        logic [DCACHE_LINE_WIDTH-1:0]   line_mask;
        logic [DCACHE_LINE_WIDTH/8-1:0] be_int;

        for (int i=0; i<(riscv::XLEN/8); i++) begin
            data_mask[i*8 +: 8] = {8{be[i]}};
        end
        line_mask = data_mask; // zero-extend
        be_int    = be; // zero-extend

        cache_line.data  = ((line_mask & data) << (offset * riscv::XLEN))    | (cache_line.data & ~(line_mask << (offset * riscv::XLEN)));
        cache_line.dirty = be_int              << (offset * (riscv::XLEN/8)) | cache_line.dirty;

    endfunction
    
    function automatic void update_cache_line_data (
        inout logic [DCACHE_LINE_WIDTH-1:0]       cache_line,
        input riscv::xlen_t                 data,
        input logic [(riscv::XLEN/8)-1:0]   be,
        input int unsigned                  offset // in units of data width
    );
        logic [riscv::XLEN-1:0]         data_mask;
        logic [DCACHE_LINE_WIDTH-1:0]   line_mask;
        logic [DCACHE_LINE_WIDTH/8-1:0] be_int;

        for (int i=0; i<(riscv::XLEN/8); i++) begin
            data_mask[i*8 +: 8] = {8{be[i]}};
        end
        line_mask = data_mask; // zero-extend
        be_int    = be; // zero-extend

        cache_line  = ((line_mask & data) << (offset * riscv::XLEN))    | (cache_line & ~(line_mask << (offset * riscv::XLEN)));

    endfunction


    function automatic logic [63:0] get_rand_addr_from_cfg(ariane_cfg_t cfg);
        logic [63:0] start_addr, end_addr, range;
        logic [31:0] addr_msb, addr_lsb;
        int region;

        region = $urandom_range(2);
        case (region)
            0 : begin
                start_addr = cfg.ExecuteRegionAddrBase[0];
                end_addr   = cfg.ExecuteRegionAddrBase[0] + cfg.ExecuteRegionLength[0];
            end
            1 : begin
                start_addr = cfg.CachedRegionAddrBase[0];
                end_addr   = cfg.CachedRegionAddrBase[0] + cfg.CachedRegionLength[0];
            end
            2 : begin
                start_addr = cfg.SharedRegionAddrBase[0];
                end_addr   = cfg.SharedRegionAddrBase[0] + cfg.SharedRegionLength[0];
            end
        endcase

        range = end_addr - start_addr - 1;

        a_lsb_zero : assert (range[63:32] === 0) else
            $error("expected address range less than 32 bits");

        addr_msb = start_addr>>32;
        addr_lsb = start_addr[31:0] + $urandom_range(range);

        return {addr_msb, addr_lsb};

    endfunction


    //--------------------------------------------------------------------------
    // AMO request class
    //--------------------------------------------------------------------------
    class amo_req;
        amo_t        op;
        logic [63:0] addr; // address
        logic [63:0] data; // data as layouted in the register
        logic  [1:0] size;
        int          id;

        function string print_me();
            return $sformatf("id %0d, type %0s, address 0x%16h, data 0x%16h, size 0b%2b", id, op.name(), addr, data, size);
        endfunction
    endclass


    //--------------------------------------------------------------------------
    // AMO response class
    //--------------------------------------------------------------------------
    class amo_resp;
        amo_t        op;
        logic [63:0] data;
        int          id;

        function string print_me();
            return $sformatf("id %0d, type %0s, data 0x%16h", id, op.name(), data);
        endfunction
    endclass


    //--------------------------------------------------------------------------
    // Driver for the AMO interface
    //--------------------------------------------------------------------------
    class amo_driver;

        virtual amo_intf vif;
        string name;
        int verbosity;
        ariane_cfg_t cfg;

        function new (virtual amo_intf vif, ariane_cfg_t cfg, string name="amo_driver");
            this.vif = vif;
            vif.req = '0;
            this.name=name;
            verbosity = 0;
            this.cfg = cfg;
        endfunction

        // request with response output
        task req_resp (
            input  logic [63:0] data         = '0,
            input  logic [63:0] addr         = '0,
            input  logic  [1:0] size         = '1, // 2'b10 --> word operation, 2'b11 --> double word operation
            input  amo_t        op           = AMO_ADD,
            input  bit          rand_data    = 0,
            input  bit          rand_addr    = 0,
            input  bit          rand_op      = 0,
            input  bit          rand_size    = 0,
            input  bit          check_result = 1'b0,
            input  logic [63:0] exp_result   = '0,
            output logic [63:0] result
        );
            logic [63:0] addr_int;
            logic [63:0] data_int;
            logic  [1:0] size_int;
            amo_t        op_int;
            result = 'x;

            if (rand_addr) begin
                addr_int = get_rand_addr_from_cfg(cfg);
            end else begin
                addr_int = addr;
            end

            if (rand_data) begin
                data_int = {$urandom, $urandom};
            end else begin
                data_int = data;
            end

            if (rand_size) begin
                size_int = $urandom_range(3,2);
            end else begin
                size_int = size;
            end

            if (rand_op) begin
                if ($urandom_range(3) > 2) begin
                    // increase chance for AMO_LR
                    op_int = AMO_LR;
                end else begin
                    op_int = amo_t'($urandom_range(AMO_MINU, AMO_LR)); // avoid sending AMO_NONE and unsupported AMO_CAS1,AMO_CAS2
                end
            end else begin
                op_int = op;
            end

            if (verbosity > 0) begin
                $display("%t ns %s sending AMO request %s to address 0x%8h with data 0x%8h", $time, name, op_int.name(), addr_int, data_int);
            end

            #0;
            vif.req.req       = 1;        // this request is valid
            vif.req.amo_op    = op_int;   // atomic memory operation to perform
            vif.req.size      = size_int; // 2'b10 --> word operation, 2'b11 --> double word operation
            vif.req.operand_a = addr_int; // address
            vif.req.operand_b = data_int; // data

            do begin
                @(posedge vif.clk);
            end while (!vif.resp.ack);
            result = vif.resp.result;

            if (verbosity > 0) begin
                $display("%t ns %s got ack for AMO request %s to address 0x%8h", $time, name, op.name(), addr_int);
            end

            if (check_result) begin
                a_rd_check : assert (result == exp_result) else
                    $error("%s : data mismatch. Expected 0x%16h, got 0x%16h", name, exp_result, result);
            end

            #0;
            vif.req.req    = 1'b0;

        endtask

        // request without response
        task req (
            input  logic [63:0] data         = '0,
            input  logic [63:0] addr         = '0,
            input  logic  [1:0] size         = '1, // 2'b10 --> word operation, 2'b11 --> double word operation
            input  amo_t        op           = AMO_ADD,
            input  bit          rand_data    = 0,
            input  bit          rand_addr    = 0,
            input  bit          rand_op      = 0,
            input  bit          rand_size    = 0,
            input  bit          check_result = 1'b0,
            input  logic [63:0] exp_result   = '0
        );
            logic [63:0] dummy_result;
            this.req_resp (
                .data         ( data         ),
                .addr         ( addr         ),
                .size         ( size         ),
                .op           ( op           ),
                .rand_data    ( rand_data    ),
                .rand_addr    ( rand_addr    ),
                .rand_op      ( rand_op      ),
                .rand_size    ( rand_size    ),
                .check_result ( check_result ),
                .exp_result   ( exp_result   ),
                .result       ( dummy_result ));
        endtask

    endclass


    //--------------------------------------------------------------------------
    // Monitor for the AMO interface
    //--------------------------------------------------------------------------
    class amo_monitor;

        mailbox #(amo_req)  req_mbox;
        mailbox #(amo_resp) resp_mbox;

        virtual amo_intf    vif;

        string              name;
        int                 verbosity;
        int                 id;

        function new (virtual amo_intf vif, string name="amo_monitor");
            this.vif  = vif;
            this.name = name;
            verbosity = 0;
            id = 0;
        endfunction

        // get read requests and responses
        local task mon;
            $display("%t ns %s monitoring AMO requests and responses", $time, name);
            forever begin
                if (vif.req.req && vif.gnt) begin // got read request
                    amo_t    op;
                    amo_req  req;
                    amo_resp resp;

                    req = new();
                    req.op   = vif.req.amo_op;
                    req.addr = vif.req.operand_a;
                    req.data = vif.req.operand_b;
                    req.size = vif.req.size;
                    req.id   = id;
                    op       = vif.req.amo_op; // remember op

                    if (verbosity > 0) begin
                        $display("%t ns %s got AMO request : %s", $time, name, req.print_me());
                    end
                    req_mbox.put(req);

                    // wait for result
                    do begin
                        @(posedge vif.clk);
                    end while (!vif.resp.ack);

                    resp      = new();
                    resp.op   = op;
                    resp.data = vif.resp.result;
                    resp.id   = id;
                    this.id++;

                    #0; // add zero delay here to make sure read response is repoerted after read request if it gets served immediately
                    if (verbosity > 0) begin
                        $display("%t ns %s got AMO response : %s", $time, name, resp.print_me());
                    end
                    resp_mbox.put(resp);

                end else begin
                    @(posedge vif.clk);
                end
            end
        endtask

        task monitor;
            mon();
        endtask

    endclass


    //--------------------------------------------------------------------------
    // dcache request
    //--------------------------------------------------------------------------
    class dcache_req;
        dcache_trans_t                       trans_type;
        logic [DCACHE_INDEX_WIDTH-1:0]       address_index;
        logic [DCACHE_TAG_WIDTH-1:0]         address_tag;
        riscv::xlen_t                        data;
        logic [(riscv::XLEN/8)-1:0]          be;
        logic [1:0]                          size;
        // help variables
        int                                  port_idx;
        int                                  prio;
        bit                                  update_cache;
        bit                                  insert_readback;
        bit                                  r_dirty;
        bit                                  r_shared;
        int                                  data_offset; // data offset into cache line
        logic [DCACHE_LINE_WIDTH-1:0]        cache_line;  // for carrying an entire cache line from read response
        logic [$clog2(DCACHE_SET_ASSOC)-1:0] target_way;
        logic                                target_way_valid;
        logic                                redo_hit;
        int                                  id;

        function new();
            this.target_way_valid = 1'b0;
            this.redo_hit         = 1'b0;
        endfunction

        task set_data_offset;
            data_offset = address_index[3];
        endtask

        task add_to_cache_line (
            input riscv::xlen_t d
        );
            cache_line = {d, cache_line} >> (riscv::XLEN);

        endtask

        function logic [63:0] get_addr ();
            return tag_index2addr(.tag(this.address_tag), .index(this.address_index));
        endfunction

        function string print_me();
            if ((trans_type == WR_REQ) || (trans_type == RD_RESP)) begin
                return $sformatf("id %0d, type %0s, port idx %0d (prio %0d), tag 0x%11h, index 0x%3h, size %0d, be 0x%2h, data 0x%16h", id, trans_type.name(), port_idx, prio, address_tag, address_index, size, be, data);
            end else if (trans_type == READBACK) begin
                return $sformatf("id %0d, type %0s, port idx %0d (prio %0d), tag 0x%11h, index 0x%3h, size %0d, be 0x%2h, data 0x%16h_%16h", id, trans_type.name(), port_idx, prio, address_tag, address_index, size, be, cache_line[127:64], cache_line[63:0]);
            end else begin
                return $sformatf("id %0d, type %0s, port idx %0d (prio %0d), tag 0x%11h, index 0x%3h, size %0d, be 0x%2h", id, trans_type.name(), port_idx, prio, address_tag, address_index, size, be);
            end
        endfunction

    endclass


    //--------------------------------------------------------------------------
    // dcache response
    //--------------------------------------------------------------------------
    class dcache_resp;
        dcache_trans_t trans_type;
        riscv::xlen_t  data;
        int            id;

        function new();
            this.trans_type = UNDEF;
            this.data = 'x;
        endfunction

        function string print_me();
            if (trans_type == RD_RESP) begin
                return $sformatf("id %0d, type %0s, data 0x%16h", id, trans_type.name(), data);
            end else begin
                return $sformatf("id %0d, type %0s", id, trans_type.name());
            end
        endfunction

    endclass


    //--------------------------------------------------------------------------
    // Driver for the CPU / data cache interface
    //--------------------------------------------------------------------------
    class dcache_driver;

        virtual dcache_intf vif;
        ariane_cfg_t cfg;
        string name;
        int verbosity;
        logic kill_req;
        logic kill_armed;

        function new (virtual dcache_intf vif, ariane_cfg_t cfg, string name="dcache_driver");
            this.vif              = vif;
            vif.req               = '0;
            vif.req.address_tag   = $urandom;
            vif.req.address_index = $urandom;
            this.cfg              = cfg;
            this.name             = name;
            verbosity             = 0;
            kill_req              = 0;
            kill_armed            = 0;
        endfunction

        // read request
        task automatic rd_resp (
            input logic  [63:0] addr         = '0,
            input logic   [1:0] size         = 2'b11,
            input logic   [7:0] be           = '1,
            input bit           rand_size_be = 0,
            input bit           rand_addr    = 0,
            input int           rand_kill    = 0, // chance of killing request in percentage
            input bit           check_result = 1'b0,
            input logic  [63:0] exp_result   = '0,
            input bit           do_wait      = 1'b0,
            input bit           kill         = 1'b0,
            output logic [63:0] result
        );
            logic [63:0] addr_int;
            logic  [1:0] size_int;
            logic  [7:0] be_int;
            logic        kill_int;
            logic [63:0] bit_mask;

            if (rand_addr) begin
                addr_int = get_rand_addr_from_cfg(cfg);
            end else begin
                addr_int = addr;
            end

            if (rand_size_be) begin
                int size_bytes;
                size_int = $urandom_range(3);
                size_bytes = 2**size_int;
                be_int = ((2**size_bytes)-1) << $urandom_range(8 - size_bytes);
            end else begin
                be_int = be;
                size_int = size;
            end

            for (int i=0; i<8; i++) begin
                bit_mask[i*8 +:8] = {8{be_int[i]}};
            end

            if (kill) begin
                kill_int = 1'b1;
            end else begin
                kill_int = (rand_kill >= $urandom_range(100,1));
            end

            if (verbosity > 0) begin
                $display("%t ns %s: sending read request for address 0x%8h", $time, name, addr_int);
            end

            #0;
            vif.req.data_req      = 1'b1;
            vif.req.data_we       = 1'b0;
            vif.req.data_be       = be_int;
            vif.req.data_size     = size_int;
            vif.req.address_index = addr2index(addr_int);

            do begin
                @(posedge vif.clk);
            end while (!vif.resp.data_gnt);

            fork
                // send tag while allowing a new read to start
                begin

                    if (verbosity > 0) begin
                        $display("%t ns %s: got grant for read address 0x%8h, sending tag 0x%6h", $time, name, addr_int, addr2tag(addr_int));
                    end

                    #0;
                    vif.req.data_req    = 1'b0;

                    #0; // one more zero delay to "win" over an earlier read that sets tag_valid to 0
                    vif.req.tag_valid   = 1'b1;
                    vif.req.address_tag = addr2tag(addr_int);

                    do begin
                        if ((this.kill_req || kill_int) && !check_result) begin // don't kill transaction when we expect a result
                            if (verbosity > 0) begin
                                $display("%t ns %s: killing read request to address 0x%8h,", $time, name, addr_int);
                            end
                            vif.req.kill_req = 1'b1;
                            this.kill_req = 0;
                            kill_int = 0;
                        end
                        @(posedge vif.clk);
                        #0;
                        vif.req.tag_valid = '0;
                        vif.req.kill_req = 1'b0;
                    end while (!vif.resp.data_rvalid);

                    result = vif.resp.data_rdata;

                    if (verbosity > 0) begin
                        $display("%t ns %s: got rvalid for read address 0x%8h", $time, name, addr_int);
                    end

                    if (check_result) begin
                        a_rd_check : assert ((result & bit_mask) == exp_result) else
                        $error("%s: data mismatch. Expected 0x%16h, got 0x%16h", name, exp_result, result);
                    end

                end

                begin
                    if (do_wait)
                        wait (0); // avoid exiting fork
                end
            join_any
        endtask

        // wrapper to rd_resp without result output mapped
        task automatic rd (
            input logic  [63:0] addr         = '0,
            input logic   [1:0] size         = 2'b11,
            input logic   [7:0] be           = '1,
            input bit           rand_size_be = 0,
            input bit           rand_addr    = 0,
            input int           rand_kill    = 0, // chance of killing request in percentage
            input bit           check_result = 1'b0,
            input logic  [63:0] exp_result   = '0,
            input bit           do_wait      = 1'b0,
            input bit           kill         = 1'b0
        );
            logic [63:0] dummy_result;
            rd_resp (
                .addr         ( addr         ),
                .size         ( size         ),
                .be           ( be           ),
                .rand_size_be ( rand_size_be ),
                .rand_addr    ( rand_addr    ),
                .rand_kill    ( rand_kill    ),
                .check_result ( check_result ),
                .exp_result   ( exp_result   ),
                .do_wait      ( do_wait      ),
                .kill         ( kill         ),
                .result       ( dummy_result )
            );
        endtask



        // write request
        task automatic wr (
            input logic [63:0] data         = 0,
            input logic [63:0] addr         = '0,
            input logic  [1:0] size         = 2'b11,
            input logic  [7:0] be           = '1,
            input bit          rand_size_be = 0,
            input bit          rand_data    = 0,
            input bit          rand_addr    = 0
        );
            logic [63:0] addr_int;
            logic [63:0] data_int;
            logic  [1:0] size_int;
            logic  [7:0] be_int;

            if (rand_addr) begin
                addr_int = get_rand_addr_from_cfg(cfg);
            end else begin
                addr_int = addr;
            end

            if (rand_data) begin
                data_int = {$urandom,$urandom};
            end else begin
                data_int = data;
            end

            if (rand_size_be) begin
                int size_bytes;
                size_int = $urandom_range(3);
                size_bytes = 2**size_int;
                be_int = ((2**size_bytes)-1) << $urandom_range(8 - size_bytes);
            end else begin
                be_int = be;
                size_int = size;
            end

            if (verbosity > 0) begin
                $display("%t ns %s: sending write request for address 0x%8h with data 0x%8h", $time, name, addr_int, data_int);
            end

            #0;
            vif.req.data_req      = 1'b1;
            vif.req.data_we       = 1'b1;
            vif.req.data_be       = be_int;
            vif.req.data_size     = size_int;
            vif.req.data_wdata    = data_int;
            vif.req.address_index = addr2index(addr_int);
            vif.req.address_tag   = addr2tag(addr_int);
            vif.req.tag_valid     = 1'b1;

            do begin
                @(posedge vif.clk);
            end while (!vif.resp.data_gnt);

            #0;
            vif.req.data_req  = 1'b0;
            vif.req.data_we   = 1'b0;
            vif.req.tag_valid = 1'b0;

        endtask

        // wait between [min_wait] and [max_wait] cycles, then arm kill_req with a
        // probability of [prob] %
        task automatic arm_kill (
            input int min_wait = 100,
            input int max_wait = 500,
            input int prob     = 100
        );
            this.kill_armed = 1;
            fork
                begin
                    while (this.kill_armed) begin
                        `WAIT_CYC(vif.clk, $urandom_range(max_wait, min_wait));
                        if ($urandom_range(100,1) <= prob) begin
                            this.kill_req = kill_armed;
                        end
                    end
                end
            join_none
        endtask

        // stop the killing
        task automatic disarm_kill ();
            this.kill_armed = 0;
        endtask

    endclass


    //--------------------------------------------------------------------------
    // Monitor for the LSU / data cache interface
    //--------------------------------------------------------------------------
    class dcache_monitor;

        mailbox #(dcache_req)  req_mbox;
        mailbox #(dcache_resp) resp_mbox;

        virtual dcache_intf    vif;

        string                 name;
        int                    verbosity;
        int                    port_idx;
        int                    rd_req_cnt;
        int                    rd_kill_cnt;
        int                    rd_resp_cnt;
        int                    wr_req_cnt;
        int                    req_id;
        int                    resp_id;

        function new (virtual dcache_intf vif, int port_idx=0, string name="dcache_monitor");
            this.vif       = vif;
            this.name      = name;
            this.port_idx  = port_idx;
            verbosity = 0;

            rd_req_cnt = 0;
            rd_kill_cnt = 0;
            rd_resp_cnt = 0;
            wr_req_cnt = 0;
            req_id = 0;
            resp_id = 0;

        endfunction

        task print_stats;
          $display("%s: got %5d read requests, (%4d killed), %5d read responses, %5d write requests", name, rd_req_cnt, rd_kill_cnt, rd_resp_cnt, wr_req_cnt);
        endtask

        // get read requests
        local task automatic mon_rd_req;
            $display("%t ns %s: monitoring read requests", $time, name);
            forever begin
                if (vif.req.data_req && !vif.req.data_we) begin // got read request
                    automatic dcache_req rd_req;

                    while (!vif.resp.data_gnt) begin
                        @(posedge vif.clk);
                    end
                    if (verbosity > 0) begin
                        $display("%t ns %s: got request for read", $time, name);
                    end

                    rd_req = new();
                    rd_req.trans_type    = RD_REQ;
                    rd_req.address_index = vif.req.address_index;
                    rd_req.be            = vif.req.data_be;
                    rd_req.size          = vif.req.data_size;
                    rd_req.port_idx      = port_idx;
                    rd_req.id            = this.req_id;
                    this.rd_req_cnt++;
                    this.req_id++;

                    // need to use negedge here - tag is not clocked in cache ctrl
                    @(negedge vif.clk);
                    while (!vif.req.tag_valid) begin
                        @(negedge vif.clk);
                    end

                    rd_req.address_tag = vif.req.address_tag;
                    rd_req.set_data_offset();
                    if (vif.req.kill_req) begin
                        if (verbosity > 0) begin
                            $display("%t ns %s: read request killed", $time, name);
                        end
                        this.rd_kill_cnt++;
                    end else begin
                        if (verbosity > 0) begin
                            $display("%t ns %s: got request for read tag 0x%6h, index 0x%3h", $time, name, rd_req.address_tag, rd_req.address_index);
                        end
                        req_mbox.put(rd_req);

                        fork begin
                            while (!vif.resp.data_rvalid) begin
                                assert (!vif.req.kill_req) else $error("%s: Got kill req without rvalid",name);
                                @(posedge vif.clk);
                            end

                            if (vif.req.kill_req) begin
                                if (verbosity > 0) begin
                                    $display("%t ns %s: read request killed", $time, name);
                                end
                                this.rd_kill_cnt++;
                            end

                            if (verbosity > 0) begin
                                $display("%t ns %s: saw read response", $time, name);
                            end
                        end join_none

                    end

                end else begin
                    @(posedge vif.clk);
                end
            end
        endtask

        // get read responses
        local task mon_rd_resp;
            dcache_resp rd_resp;
            $display("%t ns %s monitoring read responses", $time, name);
            forever begin
                if (vif.resp.data_rvalid) begin // got read request
                    rd_resp = new();
                    rd_resp.trans_type = RD_RESP;
                    rd_resp.data = vif.resp.data_rdata;
                    rd_resp.id = this.resp_id;
                    this.rd_resp_cnt++;
                    this.resp_id++;
                    #0; // add zero delay here to make sure read response is repoerted after read request if it gets served immediately
                    if (verbosity > 0) begin
                        $display("%t ns %s got read response with data 0x%8h", $time, name, rd_resp.data);
                    end
                    resp_mbox.put(rd_resp);
                end
                @(posedge vif.clk);
            end
        endtask

        // get write requests
        local task mon_wr_req;
            dcache_req  wr_req;
            dcache_resp wr_resp;
            $display("%t ns %s monitoring write requests", $time, name);
            forever begin
                if (vif.req.data_req && vif.req.data_we) begin // got write request

                    while (!vif.wr_gnt) begin
                        @(posedge vif.clk);
                    end
                    this.wr_req_cnt++;
                    if (verbosity > 0) begin
                        $display("%t ns %s got request for write", $time, name);
                    end

                    wr_req = new();
                    wr_req.trans_type      = WR_REQ;
                    wr_req.address_index = vif.req.address_index;
                    wr_req.data          = vif.req.data_wdata;
                    wr_req.be            = vif.req.data_be;
                    wr_req.size          = vif.req.data_size;
                    wr_req.port_idx      = port_idx;
                    wr_req.id            = this.req_id;
                    wr_req.address_tag   = vif.req.address_tag;
                    wr_req.set_data_offset();

                    this.req_id++;

                    // add one more cycle here to get same timing as read requests
                    @(negedge vif.clk);

                    if (verbosity > 0) begin
                        $display("%t ns %s got request for write tag 0x%6h, index 0x%3h, data 0x%8h", $time, name, wr_req.address_tag, wr_req.address_index, wr_req.data);
                    end
                    req_mbox.put(wr_req);

                    while (!vif.resp.data_gnt) begin
                        @(posedge vif.clk);
                    end
                    wr_resp = new();
                    wr_resp.trans_type = WR_RESP;
                    wr_resp.id = this.resp_id;
                    this.resp_id++;

                    if (verbosity > 0) begin
                        $display("%t ns %s got write response %s", $time, name, wr_resp.print_me());
                    end
                    resp_mbox.put(wr_resp);
                end
                @(posedge vif.clk);
            end
        endtask

        task monitor;
            fork
                mon_rd_req();
                mon_rd_resp();
                mon_wr_req();
            join
        endtask

    endclass

    //--------------------------------------------------------------------------
    // icache request
    //--------------------------------------------------------------------------
    class icache_req;
        logic [riscv::VLEN-1:0] vaddr;
        // helper variables
        int                     id;

        function new(int id = 0);
            this.id = id;
        endfunction

        function string print_me();
            return $sformatf("id %0d, vaddr %0h", id, vaddr);
        endfunction
    endclass


    //--------------------------------------------------------------------------
    // Driver for the CPU / instruction cache interface
    //--------------------------------------------------------------------------
    class icache_driver;

        virtual icache_intf vif;
        ariane_cfg_t cfg;
        string name;
        int verbosity;
        logic en_random_req;
        logic random_req_running;

        function new (virtual icache_intf vif, ariane_cfg_t cfg, string name="icache_driver");
            this.vif      = vif;
            this.cfg      = cfg;
            vif.req       = '0;
            vif.req.vaddr = $urandom;
            this.name     = name;
            verbosity     = 0;
            en_random_req = 0;
            random_req_running = 0;
        endfunction

        // read request
        task automatic rd_resp (
            input logic  [63:0] addr      = '0,
            input bit           rand_addr = 0,
            output logic [63:0] result
        );
            logic [63:0] addr_int;

            if (rand_addr) begin
                addr_int = get_rand_addr_from_cfg(this.cfg);
            end else begin
                addr_int = addr;
            end

            if (verbosity > 0) begin
                $display("%t ns %s: sending read request for address 0x%8h", $time, name, addr_int);
            end

            #0;
            vif.req.req   = 1'b1;
            vif.req.vaddr = addr_int;

            do begin
                @(posedge vif.clk);
            end while (!vif.resp.ready);
            vif.req.req = 1'b0;

        endtask

        // wrapper to rd_resp without result output mapped
        task automatic rd (
            input logic  [63:0] addr      = '0,
            input bit           rand_addr = 0
        );
            logic [63:0] dummy_result;
            rd_resp (
                .addr      ( addr         ),
                .rand_addr ( rand_addr    ),
                .result    ( dummy_result )
            );
        endtask

        // start generation of random bursts of requests
        task automatic start_random_req (
            input int min_req_gap   = 0,     // minimum gap between requests (cycles)
            input int max_req_gap   = 10,    // maximum gap between requests (cycles)
            input int min_burst     = 5,     // minimum number of requests in a burst
            input int max_burst     = 20,    // maximum number of requests in a burst
            input int min_burst_gap = 100,   // minimum gap between bursts (cycles)
            input int max_burst_gap = 10000 // maximum gap between bursts (cycles)
        );
            this.en_random_req      = 1;
            this.random_req_running = 1;
            fork begin
                while (en_random_req) begin
                    repeat ($urandom_range(min_burst,max_burst)) begin
                        if (en_random_req) begin
                            rd(.rand_addr(1));
                            `WAIT_CYC(vif.clk,$urandom_range(min_req_gap, max_req_gap));
                        end
                    end
                    if (en_random_req) begin
                        `WAIT_CYC(vif.clk,$urandom_range(min_burst_gap, max_burst_gap));
                    end
                end
                random_req_running = 0;
            end join_none;

        endtask

        task automatic stop_random_req();
            this.en_random_req = 0;
            while (random_req_running) begin
                `WAIT_CYC(vif.clk,1);
            end
        endtask


    endclass



    //--------------------------------------------------------------------------
    // dcache management transaction
    //--------------------------------------------------------------------------
    class dcache_mgmt_trans;
        dcache_mgmt_trans_t            trans_type;

        function string print_me();
            return $sformatf("type %0s",trans_type.name());
        endfunction

    endclass


    //--------------------------------------------------------------------------
    // Driver for the dcache management interface
    //--------------------------------------------------------------------------
    class dcache_mgmt_driver;

        virtual dcache_mgmt_intf vif;
        string name;
        int verbosity;

        function new (virtual dcache_mgmt_intf vif, string name="dcache_driver");
            this.vif = vif;
            vif.dcache_enable = 1'b1;
            vif.dcache_flush  = 1'b0;
            this.name=name;
            verbosity = 0;
        endfunction

        // flush
        task flush ();

            #0;
            vif.dcache_flush = 1'b1;

            if (verbosity > 0) begin
                $display("%t ns %s requesting flush", $time, name);
            end

            do begin
                @(posedge vif.clk);
            end while (!vif.dcache_flush_ack);
            #0;
            vif.dcache_flush = 1'b0;

            if (verbosity > 0) begin
                $display("%t ns %s flush done", $time, name);
            end
        endtask
    endclass


    //--------------------------------------------------------------------------
    // Monitor for the dcache management interface
    //--------------------------------------------------------------------------
    class dcache_mgmt_monitor;

        mailbox #(dcache_mgmt_trans) mbox;

        virtual dcache_mgmt_intf     vif;

        string                       name;
        int                          verbosity;

        function new (virtual dcache_mgmt_intf vif, string name="dcache_mgmt_monitor");
            this.vif  = vif;
            this.name = name;
            verbosity = 0;
        endfunction

        // get flush requests
        local task mon_flush;
            dcache_mgmt_trans trans;
            $display("%t ns %s monitoring flush requests", $time, name);
            forever begin
                if (vif.dcache_flush) begin // got flush request
                    trans = new();
                    trans.trans_type = FLUSH_REQ;
                    if (verbosity > 0) begin
                        $display("%t ns %s got flush request", $time, name);
                    end

                    // wait for flushing to start
                    while (!vif.dcache_flushing) begin
                        @(posedge vif.clk);
                    end
                    mbox.put(trans);

                    // wait for ack
                    while (!vif.dcache_flush_ack) begin
                        @(posedge vif.clk);
                    end
                end
                @(posedge vif.clk);
            end
        endtask

        task monitor;
            mon_flush();
        endtask

    endclass



    //--------------------------------------------------------------------------
    // scoreboard
    //--------------------------------------------------------------------------
    class std_cache_scoreboard #(
        parameter int unsigned AXI_ADDR_WIDTH = 0,
        parameter int unsigned AXI_DATA_WIDTH = 0,
        parameter int unsigned AXI_ID_WIDTH   = 0,
        parameter int unsigned AXI_USER_WIDTH = 0
    );

        typedef ace_test::ace_driver #(
            .AW(AXI_ADDR_WIDTH), .DW(AXI_DATA_WIDTH), .IW(AXI_ID_WIDTH), .UW(AXI_USER_WIDTH)
        ) ace_driver_t;

        typedef snoop_test::snoop_driver #(
            .AW(AXI_ADDR_WIDTH), .DW(AXI_DATA_WIDTH)
        ) snoop_driver_t;

        typedef ace_driver_t::ax_ace_beat_t   ax_ace_beat_t;
        typedef ace_driver_t::w_beat_t        w_beat_t;
        typedef ace_driver_t::b_beat_t        b_beat_t;
        typedef ace_driver_t::r_ace_beat_t    r_ace_beat_t;

        typedef snoop_driver_t::ace_ac_beat_t ace_ac_beat_t;
        typedef snoop_driver_t::ace_cd_beat_t ace_cd_beat_t;
        typedef snoop_driver_t::ace_cr_beat_t ace_cr_beat_t;

        mailbox #(dcache_req)    dcache_req_mbox_prio;
        mailbox #(dcache_req)    dcache_req_mbox_prio_tmp;
        mailbox #(dcache_req)    dcache_req_mbox  [2:0];
        mailbox #(dcache_req)    dcache_req_mbox_fwd;

        mailbox #(dcache_resp)   dcache_resp_mbox_prio;
        mailbox #(dcache_resp)   dcache_resp_mbox_prio_tmp;
        mailbox #(dcache_resp)   dcache_resp_mbox [2:0];
        mailbox #(dcache_resp)   dcache_resp_mbox_fwd;

        mailbox #(dcache_req)    req_to_cache_update;

        mailbox #(dcache_req)    req_to_cache_check;
        mailbox #(ace_ac_beat_t) snoop_to_cache_update;

        mailbox #(amo_req)       amo_req_mbox, amo_req_mbox_fwd;
        mailbox #(amo_resp)      amo_resp_mbox, amo_resp_mbox_fwd;

        mailbox #(dcache_mgmt_trans) mgmt_mbox;

        // ACE mailboxes
        mailbox aw_mbx = new, w_mbx = new, b_mbx = new, ar_mbx = new, r_mbx = new;
        mailbox aw_mbx_pre_filt = new, w_mbx_pre_filt = new, b_mbx_pre_filt = new, ar_mbx_pre_filt = new, r_mbx_pre_filt = new;

        // Snoop mailboxes
        mailbox ac_mbx = new, ac_mbx_int = new, cd_mbx = new, cr_mbx = new;

        virtual dcache_sram_if sram_vif;
        virtual dcache_gnt_if  gnt_vif;

        string       name;
        ariane_cfg_t ArianeCfg;

        // Cache model
        cache_line_t [DCACHE_NUM_WORDS-1:0][DCACHE_SET_ASSOC-1:0] cache_status;
        logic                              [DCACHE_SET_ASSOC-1:0] lfsr;

        int cache_msg_timeout  =  1500;
        int snoop_msg_timeout  =  1000;
        int amo_msg_timeout    = 10000;
        int mgmt_trans_timeout = 10000;

        int verbosity;

        logic axi_id_per_port = 0; // set to 1 if the lower 2 bits of the AXI ID corresponds to port ID

        function new (
            virtual dcache_sram_if sram_vif,
            virtual dcache_gnt_if  gnt_vif,
            ariane_cfg_t               cfg,
            string                     name="std_cache_scoreboard"
        );
            this.sram_vif             = sram_vif;
            this.gnt_vif              = gnt_vif;
            this.name                 = name;
            this.ArianeCfg            = cfg;

            this.dcache_req_mbox_prio = new();
            this.dcache_req_mbox_prio_tmp = new();

            this.dcache_resp_mbox_prio = new();
            this.dcache_resp_mbox_prio_tmp = new();

            cache_status              = '0;
            lfsr                      = '0;

            req_to_cache_update = new();
            req_to_cache_check = new();
            snoop_to_cache_update = new();

            verbosity = 0;

        endfunction

        function void set_cache_msg_timeout(int t);
            cache_msg_timeout = t;
        endfunction

        function void set_snoop_msg_timeout(int t);
            snoop_msg_timeout = t;
        endfunction

        function void set_amo_msg_timeout(int t);
            amo_msg_timeout = t;
        endfunction

        function void set_mgmt_trans_timeout(int t);
            mgmt_trans_timeout = t;
        endfunction

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // cache check functions
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        function automatic bit isHit (input logic [63:0] addr);
            for (int i = 0; i < DCACHE_SET_ASSOC; i++) begin
                if (cache_status[addr2mem_idx(addr)][i].valid && cache_status[addr2mem_idx(addr)][i].tag == addr2tag(addr))
                    return 1'b1;
            end
            return 1'b0;
        endfunction

        function automatic bit isDirty (input logic [63:0] addr);
            for (int i = 0; i < DCACHE_SET_ASSOC; i++) begin
                if ((|cache_status[addr2mem_idx(addr)][i].dirty) && cache_status[addr2mem_idx(addr)][i].valid && cache_status[addr2mem_idx(addr)][i].tag == addr2tag(addr))
                    return 1'b1;
            end
            return 1'b0;
        endfunction

        function automatic bit isShared (input logic [63:0] addr);
            for (int i = 0; i < DCACHE_SET_ASSOC; i++) begin
                if (cache_status[addr2mem_idx(addr)][i].shared && cache_status[addr2mem_idx(addr)][i].valid && cache_status[addr2mem_idx(addr)][i].tag == addr2tag(addr))
                    return 1'b1;
            end
            return 1'b0;
        endfunction

        function automatic int getHitWay (input logic [63:0] addr);
            for (int i = 0; i < DCACHE_SET_ASSOC; i++) begin
                if (cache_status[addr2mem_idx(addr)][i].valid && cache_status[addr2mem_idx(addr)][i].tag == addr2tag(addr))
                    return i;
            end
            $error("No hit way found");
            return -1;
        endfunction

        function automatic bit isCleanUnique (input ax_ace_beat_t ar);
            if (ar.ax_snoop == 4'b1011 && ar.ax_bar[0] == 1'b0 && (ar.ax_domain == 2'b10 || ar.ax_domain == 2'b01))
                return 1'b1;
            else
                return 1'b0;
        endfunction

        function automatic bit isReadShared (input ax_ace_beat_t ar);
            if (ar.ax_snoop == 4'b0001 && ar.ax_bar[0] == 1'b0 && (ar.ax_domain == 2'b01 || ar.ax_domain == 2'b10))
                return 1'b1;
            else
                return 1'b0;
        endfunction

        function automatic bit isReadOnce (input ax_ace_beat_t ar);
            if (ar.ax_snoop == 4'b0000 && ar.ax_bar[0] == 1'b0 && (ar.ax_domain == 2'b01 || ar.ax_domain == 2'b10))
                return 1'b1;
            else
                return 1'b0;
        endfunction

        function automatic bit isReadUnique (input ax_ace_beat_t ar);
            if (ar.ax_snoop == 4'b0111 && ar.ax_bar[0] == 1'b0 && (ar.ax_domain == 2'b01 || ar.ax_domain == 2'b10))
                return 1'b1;
            else
                return 1'b0;
        endfunction

        function automatic bit isReadNoSnoop (input ax_ace_beat_t ar);
            if (ar.ax_snoop == 4'b0000 && ar.ax_bar[0] == 1'b0 && (ar.ax_domain == 2'b00 || ar.ax_domain == 2'b11))
                return 1'b1;
            else
                return 1'b0;
        endfunction

        function automatic bit isWriteBack (input ax_ace_beat_t aw);
            if (aw.ax_snoop == 3'b011 && aw.ax_bar[0] == 1'b0 && (aw.ax_domain == 2'b00 || aw.ax_domain == 2'b01 || aw.ax_domain == 2'b10))
                return 1'b1;
            else
                return 1'b0;
        endfunction

        function automatic bit isWriteUnique ( input ax_ace_beat_t aw );
            if (aw.ax_snoop == 3'b000 && aw.ax_bar[0] == 1'b0 && (aw.ax_domain == 2'b01 || aw.ax_domain == 2'b10))
                return 1'b1;
            else
                return 1'b0;
        endfunction

        function automatic bit isWriteNoSnoop( input ax_ace_beat_t aw );
            if (aw.ax_snoop == 3'b000 && aw.ax_bar[0] == 1'b0 && (aw.ax_domain == 2'b00 || aw.ax_domain == 2'b11))
                return 1'b1;
            else
                return 1'b0;
        endfunction

        function automatic bit isAmo( input ax_ace_beat_t ax );
            unique case (ax.ax_id)
                4'b1011: return 1;
                default: return 0;
            endcase
        endfunction

        function automatic bit isBypass( input ax_ace_beat_t ax );
            unique case (ax.ax_id)
                4'b1000, 4'b1001, 4'b1010, 4'b1011: return 1;
                default:                            return 0;
            endcase
        endfunction

        function automatic bit isDCache( input ax_ace_beat_t ax );
            unique case (ax.ax_id)
                4'b1100, 4'b1101, 4'b1110, 4'b1111: return 1;
                default:                            return 0;
            endcase
        endfunction

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // check if cache eviction is needed
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        function automatic bit mustEvict (input dcache_req msg);
            logic valid = 1'b1;
            logic [63:0] addr = msg.get_addr();
            // check if cache is full
            for (int i = 0; i < DCACHE_SET_ASSOC; i++) begin
                valid = valid & cache_status[addr2mem_idx(addr)][i].valid;
            end
            // check if target way is dirty
            if (!isHit(addr) && valid == 1'b1 && |cache_status[addr2mem_idx(addr)][msg.target_way].dirty) begin
                assert (msg.target_way_valid) else $error("mustEvict(): Expected valid target way");
                return 1'b1;
            end else begin
                return 1'b0;
            end
        endfunction


        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // calculate next lfsr
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        function automatic logic[7:0] nextLfsr (input logic[7:0] n);
            logic tmp;
            tmp = !(n[7] ^ n[3] ^ n[2] ^ n[1]);
            return {n[6:0], tmp};
        endfunction


        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // get target way and update lfsr
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        function automatic logic [$clog2(DCACHE_SET_ASSOC)-1:0] get_way_from_lfsr (
            inout logic [7:0] lfsr
        );
            logic [$clog2(DCACHE_SET_ASSOC)-1:0] result;

            result = lfsr[$clog2(DCACHE_SET_ASSOC)-1:0];
            lfsr       = nextLfsr(lfsr);

            return result;
        endfunction


        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // get target way from cache_status
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        function automatic logic get_way_from_cache (
            input  logic [63:0]                         addr,
            output logic [$clog2(DCACHE_SET_ASSOC)-1:0] way
        );
            logic [DCACHE_INDEX_WIDTH-DCACHE_BYTE_OFFSET-1:0] mem_idx_v;
            logic [DCACHE_SET_ASSOC-1:0]                      valid_v;

            mem_idx_v = addr2mem_idx(addr);
            for (int i=0; i<DCACHE_SET_ASSOC; i++) begin
                valid_v[i] = cache_status[mem_idx_v][i].valid;
            end
            way = one_hot_to_bin(get_victim_cl(~valid_v));
            return !(&valid_v);
        endfunction


        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // check target way in cache_status
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        function automatic logic check_way_from_cache (
            input logic [63:0]                         addr,
            input logic [$clog2(DCACHE_SET_ASSOC)-1:0] way
        );
            logic [DCACHE_INDEX_WIDTH-DCACHE_BYTE_OFFSET-1:0] mem_idx_v;
            mem_idx_v = addr2mem_idx(addr);
            return cache_status[mem_idx_v][way].valid;
        endfunction

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Check cache contents against real memory
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        function automatic bit checkCache (
            input logic [63:0]                         addr,
            input logic [$clog2(DCACHE_SET_ASSOC)-1:0] way,
            input string                               origin = ""
        );
            logic [DCACHE_INDEX_WIDTH-DCACHE_BYTE_OFFSET-1:0] mem_idx_v;
            logic [DCACHE_INDEX_WIDTH-1:0]                    idx_v;
            logic [DCACHE_TAG_WIDTH-1:0]                      tag_v;
            bit                                               OK;

            logic vld_sram_valid, vld_sram_shared;
            logic [DCACHE_LINE_WIDTH/8-1:0] vld_sram_dirty;

            OK        = 1'b1;
            mem_idx_v = addr2mem_idx(addr);
            idx_v     = addr2index(addr);
            tag_v     = addr2tag(addr);

            vld_sram_dirty  = sram_vif.get_dirty(.index(mem_idx_v), .way(way));
            vld_sram_shared = sram_vif.get_shared(.index(mem_idx_v), .way(way));
            vld_sram_valid  = sram_vif.get_valid(.index(mem_idx_v), .way(way));

            // check the target way
            a_valid : assert (cache_status[mem_idx_v][way].valid == vld_sram_valid) else
                $error("%s: Cache mismatch index %h tag %h way %h - valid bit: expected %d, actual %d", {name,".",origin}, idx_v, tag_v, way, cache_status[mem_idx_v][way].valid, vld_sram_valid);

            if (vld_sram_valid) begin

                a_dirty : assert (|cache_status[mem_idx_v][way].dirty == |vld_sram_dirty) else
                    $error("%s: Cache mismatch index %h tag %h way %h - dirty bits: expected %04h, actual %04h", {name,".",origin}, idx_v, tag_v, way, cache_status[mem_idx_v][way].dirty, vld_sram_dirty);

                a_shared : assert (cache_status[mem_idx_v][way].shared == vld_sram_shared) else
                    $error("%s: Cache mismatch index %h tag %h way %h - shared bit: expected %d, actual %d", {name,".",origin}, idx_v, tag_v, way, cache_status[mem_idx_v][way].shared, vld_sram_shared);
            end

            // check tags and data for valid entries
            for (int w=0;w<DCACHE_SET_ASSOC; w++) begin : WAY
                vld_sram_valid = sram_vif.get_valid(.index(mem_idx_v), .way(w));
                if (cache_status[mem_idx_v][w].valid) begin

                    a_tag : assert (cache_status[mem_idx_v][w].tag == sram_vif.tag_sram[w][mem_idx_v]) else
                        $error("%s: Cache mismatch index %h tag %h way %0h - tag: expected %h, actual %h", {name,".",origin}, idx_v, tag_v, w, cache_status[mem_idx_v][w].tag, sram_vif.tag_sram[w][mem_idx_v]);

                    a_data : assert (cache_status[mem_idx_v][w].data == sram_vif.data_sram[w][mem_idx_v]) else
                        $error("%s: Cache mismatch index %h tag %h way %h - data: expected 0x%16h_%16h, actual 0x%16h_%16h", {name,".",origin}, idx_v, tag_v, way, cache_status[mem_idx_v][way].data[127:64], cache_status[mem_idx_v][way].data[63:0], sram_vif.data_sram[way][mem_idx_v][127:64], sram_vif.data_sram[way][mem_idx_v][63:0]);

                end else begin
                    a_valid : assert (!vld_sram_valid) else
                        $error("%s: Cache mismatch index %h tag %h way %0h - valid: expected %h, actual %h", {name,".",origin}, idx_v, tag_v, w, cache_status[mem_idx_v][w].valid, vld_sram_valid);
                end
            end
            return OK;
        endfunction

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Get expected CR response from current cache contents
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        function automatic ace_cr_beat_t GetCRResp (
            input ace_ac_beat_t req
        );
            ace_cr_beat_t resp;
            resp         = new();
            resp.cr_resp = '0;

            if (req.ac_snoop != snoop_pkg::CLEAN_INVALID &&
                req.ac_snoop != snoop_pkg::READ_ONCE &&
                req.ac_snoop != snoop_pkg::READ_UNIQUE &&
                req.ac_snoop != snoop_pkg::READ_SHARED) begin
                resp.cr_resp.error = 1'b1;
            end

            if (isDirty(req.ac_addr) && (req.ac_snoop == snoop_pkg::READ_UNIQUE || req.ac_snoop == snoop_pkg::CLEAN_INVALID)) begin
                resp.cr_resp.passDirty = 1'b1;
            end

            if (isHit(req.ac_addr) && (req.ac_snoop != snoop_pkg::CLEAN_INVALID)) begin
                resp.cr_resp.dataTransfer = 1'b1;
            end

            if (isDirty(req.ac_addr) && (req.ac_snoop == snoop_pkg::CLEAN_INVALID)) begin
                resp.cr_resp.dataTransfer = 1'b1;
            end

            if (req.ac_snoop == snoop_pkg::READ_UNIQUE || req.ac_snoop == snoop_pkg::CLEAN_INVALID) begin
                resp.cr_resp.isShared = 1'b0;
            end else if (isHit(req.ac_addr) && req.ac_snoop == snoop_pkg::READ_SHARED) begin
                resp.cr_resp.isShared = 1'b1;
            end else begin // READ_ONCE
                resp.cr_resp.isShared = isShared(req.ac_addr);
            end

            return resp;

        endfunction

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Check CR response
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        function automatic bit checkCRResp (
            input ace_ac_beat_t req,
            input ace_cr_beat_t exp,
            input ace_cr_beat_t resp
        );
            bit OK;
            OK = 1'b1;

            assert (exp.cr_resp.error == resp.cr_resp.error) else
                $error("%s: CR.resp.error mismatch: expected %h, actual %h", name, exp.cr_resp.error, resp.cr_resp.error);


            if (exp.cr_resp.error == 1'b0) begin

                a_isShared : assert (exp.cr_resp.isShared == resp.cr_resp.isShared) else
                    $error("%s: CR.resp.isShared mismatch for address 0x%16h : expected %h, actual %h", name, req.ac_addr, exp.cr_resp.isShared, resp.cr_resp.isShared);

                a_passDirty : assert (exp.cr_resp.passDirty == resp.cr_resp.passDirty) else
                    $error("%s: CR.resp.passDirty mismatch for address 0x%16h : expected %h, actual %h", name, req.ac_addr, exp.cr_resp.passDirty, resp.cr_resp.passDirty);

                a_dataTransfer : assert (exp.cr_resp.dataTransfer == resp.cr_resp.dataTransfer) else
                    $error("%s: CR.resp.dataTransfer mismatch for address 0x%16h : expected %h, actual %h", name, req.ac_addr, exp.cr_resp.dataTransfer, resp.cr_resp.dataTransfer);
            end

            return OK;

        endfunction


        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // update cache model contents when receiving snoop
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        local task update_cache_from_snoop;
            // helper variables
            logic [DCACHE_SET_ASSOC-1:0]                      valid_v;
            logic [DCACHE_INDEX_WIDTH-DCACHE_BYTE_OFFSET-1:0] mem_idx_v;
            logic                                             hit_v;
            bit                                               CheckOK;
            ace_ac_beat_t                                     ac;
            logic [$clog2(DCACHE_SET_ASSOC)-1:0]              hit_way;
            int                                               cnt;

            forever begin
                snoop_to_cache_update.get(ac);

                mem_idx_v = addr2mem_idx(ac.ac_addr);
                hit_v     = 1'b0;

                // actual cache update takes 3 more cycles (with grant for some)
                cnt = 0;
                // 1. wait for grant to read cache
                while (!gnt_vif.gnt[1]) begin
                    $display("%t ns %s.update_cache_from_snoop: skipping cycle without grant for snoop", $time, name);
                    @(posedge sram_vif.clk); // skip cycles without grant
                    cnt++;
                    if (cnt > 1000) begin
                        $error("%t timeout while waiting for grant for snoop update", $time);
                        break;
                    end
                end
                $display("%t ns %s.update_cache_from_snoop: got grant for snoop", $time, name);
                @(posedge sram_vif.clk);

                // 2. wait for FSM
                @(posedge sram_vif.clk);

                // look for the right tag
                hit_v     = 1'b0;
                for (int i=0; i<DCACHE_SET_ASSOC; i++) begin
                    if (cache_status[mem_idx_v][i].valid && cache_status[mem_idx_v][i].tag == addr2tag(ac.ac_addr)) begin
                        hit_way = i;
                        hit_v   = 1'b1;
                        break;
                    end
                end

                // 3. wait for grant to write cache, if required
                if (hit_v && (ac.ac_snoop == snoop_pkg::READ_SHARED ||
                              ac.ac_snoop == snoop_pkg::READ_UNIQUE ||
                              ac.ac_snoop == snoop_pkg::CLEAN_INVALID)) begin
                    while (!gnt_vif.snoop_wr_gnt) begin
                        $display("%t ns %s.update_cache_from_snoop: skipping cycle without grant for snoop", $time, name);
                        @(posedge sram_vif.clk); // skip cycles without grant
                        cnt++;
                        if (cnt > 1000) begin
                            $error("%t timeout while waiting for grant for snoop update", $time);
                            break;
                        end
                    end
                end
                @(posedge sram_vif.clk);

                // send snoop message to do_hit()
                ac_mbx_int.put(ac);

                $display("%t ns %s.update_cache_from_snoop: updating cache status", $time, name);

                // check hit again, might have been invalidated by eviction
                hit_v = 1'b0;
                for (int i=0; i<DCACHE_SET_ASSOC; i++) begin
                    if (cache_status[mem_idx_v][i].valid && cache_status[mem_idx_v][i].tag == addr2tag(ac.ac_addr)) begin
                        hit_way = i;
                        hit_v   = 1'b1;
                        break;
                    end
                end

                if (hit_v) begin
                    case (ac.ac_snoop)
                        snoop_pkg::READ_SHARED: begin
                            $display("Update mem [%0d][%0d] from READ_SHARED", mem_idx_v, hit_way);
                            cache_status[mem_idx_v][hit_way].shared = 1'b1;
                            // the snoop cache controller will set all dirty bits if any is set
                            cache_status[mem_idx_v][hit_way].dirty = |cache_status[mem_idx_v][hit_way].dirty ? '1 : '0;
                        end
                        snoop_pkg::READ_UNIQUE: begin
                            $display("Update mem [%0d][%0d] from READ_UNIQUE", mem_idx_v, hit_way);
                            cache_status[mem_idx_v][hit_way].shared = 1'b0;
                            cache_status[mem_idx_v][hit_way].valid = 1'b0;
                            cache_status[mem_idx_v][hit_way].dirty = '0;
                        end
                        snoop_pkg::CLEAN_INVALID: begin
                            $display("Update mem [%0d][%0d] from CLEAN_INVALID", mem_idx_v, hit_way);
                            cache_status[mem_idx_v][hit_way].shared = 1'b0;
                            cache_status[mem_idx_v][hit_way].valid = 1'b0;
                            cache_status[mem_idx_v][hit_way].dirty = '0;
                        end
                        snoop_pkg::READ_ONCE: begin
                            $display("Update mem [%0d][%0d] from READ_ONCE", mem_idx_v, hit_way);
                        end
                        default: begin
                            $error("%t unexpected snoop type %0d", name, ac.ac_snoop);
                        end
                    endcase
                    if (cache_status[mem_idx_v][hit_way].valid) begin
                        $display("Updated cache_status[%0d][%0d]: valid : %0d, dirty : %04h, shared : %0d, tag : 0x%6h, data : 0x%16h_%16h", mem_idx_v, hit_way,
                            cache_status[mem_idx_v][hit_way].valid,
                            cache_status[mem_idx_v][hit_way].dirty,
                            cache_status[mem_idx_v][hit_way].shared,
                            cache_status[mem_idx_v][hit_way].tag,
                            cache_status[mem_idx_v][hit_way].data[127:64],
                            cache_status[mem_idx_v][hit_way].data[63:0]);
                    end else begin
                        $display("Updated cache_status[%0d][%0d]: valid : %0d, dirty : %04h, shared : %0d, tag : 0x%6h", mem_idx_v, hit_way,
                            cache_status[mem_idx_v][hit_way].valid,
                            cache_status[mem_idx_v][hit_way].dirty,
                            cache_status[mem_idx_v][hit_way].shared,
                            cache_status[mem_idx_v][hit_way].tag);
                    end
                    CheckOK = checkCache(ac.ac_addr, hit_way, "update_cache_from_snoop");
                end else begin
                    $display("No hit for addr %8h", ac.ac_addr);
                end

            end
        endtask



        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // update cache model contents when receiving dcache request
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        local task automatic update_cache_from_req;

            forever begin
                dcache_req   req_t;
                req_to_cache_update.get(req_t);

                fork
                    begin
                        logic [DCACHE_INDEX_WIDTH-DCACHE_BYTE_OFFSET-1:0] mem_idx_v;
                        logic [63:0]                                      addr_v;
                        bit                                               CheckOK;
                        logic [$clog2(DCACHE_SET_ASSOC)-1:0]              target_way;
                        logic [$clog2(DCACHE_SET_ASSOC)-1:0]              dut_way;
                        bit                                               hit;
                        dcache_req                                        req;

                        req       = new req_t;
                        addr_v    = tag_index2addr(.tag(req.address_tag), .index(req.address_index));
                        mem_idx_v = addr2mem_idx(addr_v);
                        hit       = isHit(addr_v);

                        // check that cache access is granted if needed
                        if (req.update_cache) begin
                            int cnt = 0;
                            while (!gnt_vif.gnt[req.prio]) begin

                                $display("%t ns %s.update_cache_from_req: skipping cycle without grant for dcache req : %s", $time, name, req.print_me());
                                @(posedge sram_vif.clk); // skip cycles without grant
                                cnt++;
                                if (cnt > cache_msg_timeout) begin
                                    $error("%s.update_cache_from_req:: Timeout while waiting for grant for dcache req : %s", name, req.print_me());
                                    break;
                                end
                                if (hit && !isHit(addr_v)) begin
                                    $display("%t ns %s.update_cache_from_req: hit status changed from hit to miss, calling do_miss for req : %s", $time, name, req.print_me());
                                    hit = 0;
                                    req.insert_readback = 0;
                                    do_miss(req);
                                end

                            end
                            $display("%t ns %s.update_cache_from_req: got grant for dcache req : %s", $time, name, req.print_me());
                            dut_way = one_hot_to_bin(gnt_vif.get_way(.use_be(req_t.prio==0)));
                            $display("%t ns %s.update_cache_from_req: got DUT target way %0d for dcache req : %s", $time, name, dut_way, req.print_me());
                            @(posedge sram_vif.clk);
                            $display("%t ns %s.update_cache_from_req: updating cache status from dcache req : %s", $time, name, req.print_me());
                        end else begin
                            $display("%t ns %s.update_cache_from_req: no cache update expected for dcache req : %s", $time, name, req.print_me());
                        end

                        $display("%t ns %s addr: 0x%16h, mem_idx: %0d", $time, name, addr_v, mem_idx_v);

                        if (hit) begin
                            // cache hit
                            $display("Cache hit");
                            target_way = getHitWay(addr_v);

                            if (target_way != dut_way)
                                $error("Mismatch between target way %0d DUT way %0d", target_way, dut_way);

                            if (req.trans_type == WR_REQ) begin
                                $display("cache_date[%0d][%0d]: before update: 0x%16h_%16h", mem_idx_v, target_way,
                                    cache_status[mem_idx_v][target_way].data[127:64],
                                    cache_status[mem_idx_v][target_way].data[63:0]
                                );
                                cache_status[mem_idx_v][target_way].shared = 1'b0;
                                update_cache_line(cache_status[mem_idx_v][target_way], req.data, req.be, req.data_offset);
                            end
                        end else begin
                            logic [DCACHE_SET_ASSOC-1:0] valid_v;
                            // cache miss
                            $display("Cache miss");
                            for (int i=0; i<DCACHE_SET_ASSOC; i++) begin
                                valid_v[i] = cache_status[mem_idx_v][i].valid;
                            end
                            if (&valid_v) begin
                                // all ways occupied
                                $display("No empty way");

                                if (req.target_way_valid) begin
                                    target_way = req.target_way;
                                end else begin
                                    target_way = get_way_from_lfsr(lfsr);
                                end

                                if (req.trans_type == EVICT) begin
                                    $display("Evict");
                                    cache_status[mem_idx_v][target_way].valid  = 1'b0;
                                    cache_status[mem_idx_v][target_way].dirty  = '0;
                                    cache_status[mem_idx_v][target_way].shared = 1'b0;
                                end else  if (req.trans_type == WR_REQ) begin
                                    cache_status[mem_idx_v][target_way].tag    = req.address_tag;
                                    cache_status[mem_idx_v][target_way].dirty  = req.r_dirty ? '1 : '0; // we got passDirty
                                    cache_status[mem_idx_v][target_way].shared = 1'b0;
                                    cache_status[mem_idx_v][target_way].data   = req.cache_line;
                                    update_cache_line(cache_status[mem_idx_v][target_way], req.data, req.be, req.data_offset);
                                end else  if (req.trans_type == READBACK || req.trans_type == RD_RESP) begin
                                    cache_status[mem_idx_v][target_way].tag    = req.address_tag;
                                    cache_status[mem_idx_v][target_way].dirty  = req.r_dirty ? '1 : '0;
                                    cache_status[mem_idx_v][target_way].shared = req.r_shared;
                                    cache_status[mem_idx_v][target_way].data   = req.cache_line;
                                end else begin
                                    $error("Didn't expect trans_type %s", req.trans_type.name());
                                end
                            end else begin
                                // there is an empty way
                                if (req.target_way_valid) begin
                                    $display("Using target way from request");
                                    target_way = req.target_way;
                                end else begin
                                    $display("Empty way found");
                                    target_way = one_hot_to_bin(get_victim_cl(~valid_v));
                                    if (target_way != dut_way)
                                        $warning("Adjusting target way %0d to match DUT way %0d", target_way, dut_way);
                                    target_way = dut_way;
                                end
                                cache_status[mem_idx_v][target_way].tag   = req.address_tag;

                                if (req.trans_type == EVICT) begin
                                    $display("Evict");
                                    cache_status[mem_idx_v][target_way].valid  = 1'b0;
                                    cache_status[mem_idx_v][target_way].dirty  = '0;
                                    cache_status[mem_idx_v][target_way].shared = 1'b0;
                                end else  if (req.trans_type == WR_REQ) begin
                                    cache_status[mem_idx_v][target_way].valid  = 1'b1;
                                    cache_status[mem_idx_v][target_way].dirty  = req.r_dirty ? '1 : '0; // we got passDirty
                                    cache_status[mem_idx_v][target_way].shared = 1'b0;
                                    cache_status[mem_idx_v][target_way].tag    = req.address_tag;
                                    cache_status[mem_idx_v][target_way].data   = req.cache_line;
                                    update_cache_line(cache_status[mem_idx_v][target_way], req.data, req.be, req.data_offset);
                                end else  if (req.trans_type == READBACK || req.trans_type == RD_RESP) begin
                                    cache_status[mem_idx_v][target_way].valid  = 1'b1;
                                    cache_status[mem_idx_v][target_way].dirty  = req.r_dirty ? '1 : '0;
                                    cache_status[mem_idx_v][target_way].shared = req.r_shared;
                                    cache_status[mem_idx_v][target_way].tag    = req.address_tag;
                                    cache_status[mem_idx_v][target_way].data   = req.cache_line;
                                end else begin
                                    $error("Didn't expect trans_type %s", req.trans_type.name());
                                end
                            end
                        end

                        if (hit && req.trans_type == RD_REQ) begin
                            assert (req.update_cache == 0) else $error("Didn't expect update for a read hit");
                        end else begin
                            assert (req.update_cache == 1) else $error("Expected cache update for a write or miss");
                        end

                        $display("Updated cache_status[%0d][%0d]: valid : %0d, dirty : %04h, shared : %0d, tag : 0x%6h, data : 0x%16h_%16h", mem_idx_v, target_way,
                                cache_status[mem_idx_v][target_way].valid,
                                cache_status[mem_idx_v][target_way].dirty,
                                cache_status[mem_idx_v][target_way].shared,
                                cache_status[mem_idx_v][target_way].tag,
                                cache_status[mem_idx_v][target_way].data[127:64],
                                cache_status[mem_idx_v][target_way].data[63:0]
                        );

                        CheckOK = checkCache(addr_v, target_way, "update_cache_from_req");

                    end

               join_none

            end
        endtask


        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // check behaviour when receiving snoop requests
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        local task automatic check_snoop;
            forever begin
                ace_ac_beat_t  ac;
                bit            timeout = 0;
                acsnoop_enum_t e;

                // wait for snoop request
                ac = new();
                ac_mbx.get(ac);
                e = acsnoop_enum_t'(ac.ac_snoop);
                $display("%t ns %s.check_snoop: Got snoop request %0s to address 0x%16h", $time, name, e.name(), ac.ac_addr);
                a_empty_ac : assert (ac_mbx.num() == 0) else $error ("%S.check_snoop : AC mailbox not empty", name);

                fork
                    begin
                        fork
                            begin
                                bit           CheckOK;
                                ace_cr_beat_t cr, cr_exp;

                                if (is_inside_cacheable_regions(ArianeCfg, ac.ac_addr)) begin
                                    snoop_to_cache_update.put(ac);
                                end

                                // wait for grant to read cache
                                while (!gnt_vif.gnt[1]) begin
                                    $display("%t ns %s.check_snoop: skipping cycle without grant for snoop", $time, name);
                                    @(posedge sram_vif.clk); // skip cycles without grant
                                end
                                @(posedge sram_vif.clk);

                                cr_exp = GetCRResp(ac);
                                $display("%t ns %s.check_snoop: Got expected response PassDirty : %1b, DataTransfer : %1b, Error : %1b for address %16h", $time, name, cr_exp.cr_resp.passDirty, cr_exp.cr_resp.dataTransfer, cr_exp.cr_resp.error, ac.ac_addr);

                                // wait for the response
                                cr_mbx.get(cr);
                                $display("%t ns %s.check_snoop: Got snoop response 0b%5b (WasUnique : %1b, isShared : %1b, PassDirty : %1b, Error : %1b, DataTransfer : %1b)", $time, name, cr.cr_resp, cr.cr_resp[4],cr.cr_resp[3],cr.cr_resp[2],cr.cr_resp[1],cr.cr_resp[0]);
                                a_empty_cr : assert (cr_mbx.num() == 0) else $error ("%S.check_snoop : CR mailbox not empty", name);

                                CheckOK = checkCRResp(.req(ac), .exp(cr_exp), .resp(cr));

                                // expect the data
                                $display("%t ns %s.check_snoop: CD mailbox size : %0d", $time, name, cd_mbx.num());

                                if (cr_exp.cr_resp.dataTransfer) begin
                                    ace_cd_beat_t cd;
                                    cd = new();
                                    cd.cd_last = 1'b0;
                                    while (!cd.cd_last) begin
                                        cd_mbx.get(cd);
                                        $display("%t ns %s.check_snoop: Got snoop data 0x%16h, last = %0d", $time, name, cd.cd_data,cd.cd_last);
                                    end
                                end
                                // check that no unexpected CD response has been generated
                                a_empty_cd : assert (cd_mbx.num() == 0) else $error ("%S.check_snoop : CD mailbox not empty", name);
                            end
                        join
                    end

                    // timeout
                    begin
                        automatic int cnt = 0;
                        while (cnt < snoop_msg_timeout) begin
                            @(posedge sram_vif.clk);
                            cnt++;
                        end
                        timeout = 1;
                    end

                join_any

                if (timeout) $error("%s.check_snoop : Timeout", name);

            end // forever
        endtask


        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // get cache requests in prio order
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        local task automatic get_cache_msg;
            $display("%t ns %s retreiving dcache messages", $time, name);
            forever begin
                for (int i=0; i<=2; i++) begin
                    dcache_req req;
                    dcache_resp resp;
                    if (dcache_req_mbox[i].try_get(req)) begin
                        dcache_req req_t;
                        req_t = new req;
                        dcache_req_mbox_prio_tmp.put(req_t);
                    end
                    if (dcache_resp_mbox[i].try_get(resp)) begin
                        dcache_resp resp_t;
                        resp_t = new resp;
                        dcache_resp_mbox_prio_tmp.put(resp_t);
                    end
                end
                @(posedge sram_vif.clk);
            end
        endtask

        local task automatic get_cache_req_tmp;
            dcache_req req;
            forever begin
                dcache_req_mbox_prio_tmp.get(req);
                fork
                    begin
                        dcache_req req_t;
                        req_t = new req;
                        dcache_req_mbox_prio.put(req_t);
                        check_cache_msg();
                    end
                    begin
                        @(posedge sram_vif.clk);
                    end
                join_any
            end
        endtask


        local task automatic get_cache_resp_tmp;
            dcache_resp resp;
            forever begin
                resp = new();
                dcache_resp_mbox_prio_tmp.get(resp);
                fork
                    automatic dcache_resp resp_tmp;
                    begin
                        resp_tmp = new resp;
                        // dcache_resp_mbox_prio.put(resp_tmp); // not used at the moment, put
                        // response directly in forwarding mailbox
                        dcache_resp_mbox_fwd.put(resp_tmp);
                    end
                    begin
                        @(posedge sram_vif.clk);
                    end
                join_any
            end
        endtask


        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // filter AXI bus
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        local task automatic axi_ar_filter;
            $display("%t ns %s filtering AXI AR transactions", $time, name);
            forever begin                ax_ace_beat_t ar_beat = new();
                ar_mbx_pre_filt.get(ar_beat);
                if (ar_beat.ax_id[3:0] != 0) begin
                    ar_mbx.put(ar_beat);
                end
            end
        endtask

        local task automatic axi_r_filter;
            $display("%t ns %s filtering AXI R transactions", $time, name);
            forever begin
                r_ace_beat_t  r_beat  = new();
                r_mbx_pre_filt.get(r_beat);
                if (r_beat.r_id[3:0] != 0) begin
                    r_mbx.put(r_beat);
                end
            end
        endtask


        local task automatic axi_write_filter;
            logic forward = 0;
            $display("%t ns %s filtering AXI write transactions", $time, name);
            forever begin
                ax_ace_beat_t aw_beat = new();
                b_beat_t      b_beat  = new();
                w_beat_t      w_beat  = new();

                forward = 0;
                aw_mbx_pre_filt.get(aw_beat);
                if (aw_beat.ax_id[3:0] != 0) begin
                    forward = 1;
                    aw_mbx.put(aw_beat);
                end

                while (!w_beat.w_last) begin
                    w_mbx_pre_filt.get(w_beat);
                    if (forward) begin
                        w_mbx.put(w_beat);
                    end
                end

                b_mbx_pre_filt.get(b_beat);
                if (forward) begin
                     b_mbx.put(b_beat);
                end

            end
        endtask


        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // subtask for hit case
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        local task automatic do_hit (input dcache_req msg);
            logic [63:0]  addr_v;
            int cnt = 0;
            logic mshr_match = 0;
            logic redo_hit;
            logic readunique_done = 0;

            $display("%t ns %s started hit task for message : %s", $time, name, msg.print_me());
            addr_v = tag_index2addr(.tag(msg.address_tag), .index(msg.address_index));
            redo_hit = msg.redo_hit;
            msg.redo_hit = 1'b0;

            // wait for possible MSHR match
            while (gnt_vif.mshr_match[msg.port_idx]) begin
                mshr_match = 1;
                if (cnt == 0 || verbosity > 0) begin
                    $display("%t ns %s.do_hit: wait for MSHR match for message : %s", $time, name, msg.print_me());
                end
                @(posedge sram_vif.clk);
                cnt++;
                if (cnt > cache_msg_timeout) begin
                    $error("%s.do_hit : Timeout while waiting for MSHR match for message : %s", name, msg.print_me());
                    break;
                end
                if (!gnt_vif.mshr_match[msg.port_idx]) begin
                    // if there was an MHSR match we need to get cache grant again
                    $display("%t ns %s.do_hit: MSHR match ended, wait for cache grant for message : %s", $time, name, msg.print_me());
                    while (!gnt_vif.rd_gnt[msg.prio]) begin
                        @(posedge sram_vif.clk); // skip cycles without grant
                        cnt++;
                        if (cnt > cache_msg_timeout) begin
                            $error("%s : Timeout while waiting for cache grant for message : %s", name, msg.print_me());
                            break;
                        end
                    end
                    $display("%t ns %s.do_hit: got cache grant for message : %s", $time, name, msg.print_me());
                    @(posedge sram_vif.clk); // wait one more cycle before reading cache status
                end
            end

            if (mshr_match) begin
                $display("%t ns %s.do_hit: MSHR match ended and got grant for message : %s", $time, name, msg.print_me());
            end else begin
                while (gnt_vif.req[msg.prio] && (!gnt_vif.rd_gnt[msg.prio])) begin
                    $display("%t ns %s.do_hit: wait for cache grant for message : %s", $time, name, msg.print_me());
                    @(posedge sram_vif.clk); // skip cycles without grant
                    cnt++;
                    if (cnt > cache_msg_timeout) begin
                        $error("%s : Timeout while waiting for cache grant for message : %s", name, msg.print_me());
                        break;
                    end
                end
                $display("%t ns %s.do_hit: got cache grant for message : %s", $time, name, msg.print_me());
            end

            if (msg.trans_type == WR_REQ && isHit(addr_v)) begin
                ace_ac_beat_t ac = new();
                msg.update_cache = 1'b1;

                if (redo_hit == 1'b1) begin
                    // The timing is a bit different when redoing hit - compensate for it here
                    @(posedge sram_vif.clk);
                end

                // empty snoop mailbox
                while (ac_mbx_int.try_get(ac));

                if (isShared(addr_v)) begin
                    ax_ace_beat_t ar_beat      = new();
                    ax_ace_beat_t ar_beat_peek = new();
                    r_ace_beat_t  r_beat       = new();
                    r_ace_beat_t  r_beat_peek  = new();

                    $display("%t ns %s.do_hit: Got status shared = %0d, redo_hit = %0d for message: %s", $time, name, isShared(addr_v), redo_hit, msg.print_me());

                    if (axi_id_per_port) begin
                        logic [3:0] exp_id = 4'hC + msg.port_idx;
                        // wait for AR beat with expected ID
                        while (ar_beat_peek.ax_id[3:0] != exp_id) begin
                            ar_mbx.peek(ar_beat_peek);
                            if (ar_beat_peek.ax_id != exp_id) begin
                                $display("%t ns %s.do_hit: ignoring AR beat with ID 0x%0h for message : %s", $time, name, ar_beat_peek.ax_id, msg.print_me());
                                @(posedge sram_vif.clk);
                            end
                        end
                    end else begin
                        if (msg.prio >= 2) begin
                            int cnt = 0;
                            // this is a request from a cache controller, wait for grant from miss handler
                            $display("%t ns %s.do_hit: wait for miss handler grant for message : %s", $time, name, msg.print_me());
                            while (!gnt_vif.miss_gnt[msg.port_idx]) begin
                                @(posedge sram_vif.clk); // skip cycles without grant
                                cnt++;
                                if (cnt > cache_msg_timeout) begin
                                    $error("%s : Timeout while waiting for miss handler grant for message : %s", name, msg.print_me());
                                    break;
                                end
                            end
                        end
                    end

                    // wait for AR beat
                    ar_mbx.get(ar_beat);
                    $display("%t ns %s.do_hit: got AR beat with ID 0x%0h for message : %s", $time, name, ar_beat.ax_id, msg.print_me());
                    // the valid flag of the cacheline might have changed in the meantime
                    // so both CleanUnique and ReadUnique are valid messages
                    if (!isCleanUnique(ar_beat) && !isReadUnique(ar_beat))
                        $error("%s Error CLEAN_UNIQUE or READ_UNIQUE expected for message : %s", name, msg.print_me());
                    readunique_done = isReadUnique(ar_beat);

                    // wait for R beat
                    while (!r_beat.r_last) begin
                        r_mbx.peek(r_beat_peek);
                        if (r_beat_peek.r_id == ar_beat.ax_id) begin
                            // this is our response
                            r_mbx.get(r_beat);
                            msg.add_to_cache_line(r_beat.r_data);
                            $display("%t ns %s.do_hit: got R beat with ID 0x%0h and last = %0d for message : %s", $time, name, r_beat.r_id, r_beat.r_last, msg.print_me());
                        end else begin
                            $display("%t ns %s.do_hit: ignoring R beat with ID 0x%0h for message : %s", $time, name, r_beat_peek.r_id, msg.print_me());
                            @(posedge sram_vif.clk);
                        end
                    end

                    msg.insert_readback = 1'b1;

                    // check if a ReadShared has arrived during writing
                    while (ac_mbx_int.try_get(ac)) begin
                        if (ac.ac_snoop == snoop_pkg::READ_SHARED && (ac.ac_addr >> DCACHE_BYTE_OFFSET) == (addr_v >> DCACHE_BYTE_OFFSET)) begin
                            $display("%t ns %s Got matching ReadShared during hit + write shared, calling hit routine for message : %s", $time, name, msg.print_me());
                            msg.redo_hit = 1'b1;
                        end
                    end

                end

            end

            if (readunique_done == 0 && !isHit(addr_v)) begin
                $display("%t ns %s Cache status changed from hit to miss, calling miss routine for message : %s", $time, name, msg.print_me());
                msg.prio = 0; // miss handler will handle this
                do_miss(msg);
                msg.redo_hit = 1'b0;                
            end
            else if (!isHit(addr_v)) begin
                msg.prio = 0; // miss handler will handle this
                msg.update_cache  = 1'b1;
                msg.r_dirty =  1'b1; // we're modifying the read cacheline anyway
                update_cache_line_data(msg.cache_line, msg.data, msg.be, msg.data_offset);
            end
        endtask

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // subtask for miss case
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        local task automatic do_miss (input dcache_req msg);
            dcache_req    readback_msg;
            logic [63:0]  addr_v;

            $display("%t ns %s started miss task for message : %s", $time, name, msg.print_me());
            addr_v = tag_index2addr(.tag(msg.address_tag), .index(msg.address_index));

            msg.update_cache = 1'b1;
            fork
                // Get target way and handle eviction  . . . . . . . . . . . . .
                begin : evict
                    ax_ace_beat_t aw_beat = new();
                    b_beat_t      b_beat  = new();
                    w_beat_t      w_beat  = new();
                    dcache_req    evict_msg;

                    int cnt = 0;
                    while (!gnt_vif.wr_gnt[msg.port_idx]) begin
                        // $display("%t ns %s.do_miss.evict: skipping cycle without miss handler processingfor message : %s", $time, name, msg.print_me());
                        @(posedge sram_vif.clk); // skip cycles without grant
                        cnt++;
                        if (cnt > cache_msg_timeout) begin
                            $error("%s : Timeout while waiting for miss handler processing for message : %s", name, msg.print_me());
                            break;
                        end
                    end
                    $display("%t ns %s.do_miss.evict: miss handler started processing message : %s", $time, name, msg.print_me());

                    msg.target_way_valid = get_way_from_cache(msg.get_addr(), msg.target_way);
                    if (msg.target_way_valid) begin
                        $display("%t ns %s.do_miss.evict: found empty target way %d for message : %s", $time, name, msg.target_way, msg.print_me());
                    end else begin
                        msg.target_way = get_way_from_lfsr(lfsr);
                        msg.target_way_valid = 1'b1;
                        $display("%t ns %s.do_miss.evict: all ways occupied, set target way %d for message : %s", $time, name, msg.target_way, msg.print_me());
                    end

                    // monitor if eviction is necessary
                    while (!mustEvict(msg)) begin
                        @(posedge sram_vif.clk);
                    end
                    $display("%t ns %s.do_miss.evict: Eviction needed for message %s", $time, name, msg.print_me());

                    // insert evict message
                    evict_msg            = new msg;
                    evict_msg.trans_type = EVICT;
                    evict_msg.prio       = 0; // miss handler updates the cache
                    $display("%t ns %s.do_miss.evict: New message created with target way %0d : %s", $time, name, evict_msg.target_way, evict_msg.print_me());

                    $display("%t ns %s.do_miss.evict: Wait for eviction AW beat for message : %s", $time, name, msg.print_me());
                    aw_mbx.get(aw_beat);
                    if (!isWriteBack(aw_beat))
                        $error("%s.do_miss.evict : WRITEBACK request expected after eviction for message : %s", name, msg.print_me());
                    a_empty_aw : assert (aw_mbx.num() == 0) else $error ("%S.do_miss : AW mailbox not empty", name);

                    $display("%t ns %s.do_miss.evict: sending evict message to cache update: %s", $time, name, evict_msg.print_me());
                    req_to_cache_update.put(evict_msg);

                    // wait for W beat
                    while (!w_beat.w_last) begin
                        w_mbx.get(w_beat);
                        $display("%t ns %s.do_miss.evict : got W beat with last = %0d for message %s", $time, name, w_beat.w_last, msg.print_me());
                    end
                    a_empty_w : assert (w_mbx.num() == 0) else $error ("%S.do_miss : W mailbox not empty", name);

                    // wait for B beat
                    b_mbx.get(b_beat);
                    $display("%t ns %s.do_miss.evict : got B beat for message %s", $time, name, msg.print_me());
                    a_empty_b : assert (b_mbx.num() == 0) else $error ("%S.do_miss : B mailbox not empty", name);

                    wait (0); // avoid exiting fork

                end

                // Check AXI transactions  . . . . . . . . . . . . . . . . . . .
                begin : check_axi
                    ax_ace_beat_t ar_beat      = new();
                    ax_ace_beat_t ar_beat_peek = new();
                    r_ace_beat_t  r_beat       = new();
                    r_ace_beat_t  r_beat_peek  = new();
                    int           r_cnt        = 0;
                    int cnt =0;
                    logic mshr_match = 0;

                    if (msg.prio >= 2) begin
                        int cnt = 0;
                        // this is a request from a cache controller, wait for grant from miss handler
                        $display("%t ns %s.do_miss.check_axi: wait for miss handler grant for message : %s", $time, name, msg.print_me());
                        while (!gnt_vif.miss_gnt[msg.port_idx]) begin
                            // $display("%t ns %s.do_miss.check_axi: skipping cycle without miss handler grant for message : %s", $time, name, msg.print_me());
                            @(posedge sram_vif.clk); // skip cycles without grant
                            cnt++;
                            if (cnt > cache_msg_timeout) begin
                                $error("%s : Timeout while waiting for miss handler grant for message : %s", name, msg.print_me());
                                break;
                            end
                        end
                        $display("%t ns %s.do_miss.check_axi: got miss handler grant for message : %s", $time, name, msg.print_me());
                    end

                    // wait for AR beat
                    $display("%t ns %s.do_miss.check_axi: waiting for AR beat for message : %s", $time, name, msg.print_me());
                    while (!isDCache(ar_beat)) begin
                        ar_mbx.peek(ar_beat_peek);
                        if (isDCache(ar_beat_peek)) begin
                            // this is our response
                            ar_mbx.get(ar_beat);
                        end else begin
                            $display("%t ns %s.do_miss.check_axi: ignoring AR beat with ID 0x%0h for message : %s", $time, name, ar_beat_peek.ax_id, msg.print_me());
                            @(posedge sram_vif.clk);
                        end
                    end
                    $display("%t ns %s.do_miss.check_axi: got AR beat with ID 0x%0h for message : %s", $time, name, ar_beat.ax_id, msg.print_me());

                    if (msg.trans_type == WR_REQ) begin
                        if (is_inside_shareable_regions(ArianeCfg, msg.get_addr())) begin
                            if (!isReadUnique(ar_beat)) begin
                                $error("%s.do_miss.check_axi: READ_UNIQUE request expected for message : %s", name, msg.print_me());
                            end
                        end else begin
                            if (!isReadNoSnoop(ar_beat)) begin
                                $error("%s.do_miss.check_axi: READ_NO_SNOOP request expected for message : %s", name, msg.print_me());
                            end
                        end
                    end else begin // RD_REQ
                        if (is_inside_shareable_regions(ArianeCfg, msg.get_addr())) begin
                            if (!isReadShared(ar_beat)) begin
                                $error("%s.do_miss.check_axi: READ_SHARED request expected for message : %s", name, msg.print_me());
                            end
                        end else begin
                            if (!isReadNoSnoop(ar_beat)) begin
                                $error("%s.do_miss.check_axi: READ_NO_SNOOP request expected for message : %s", name, msg.print_me());
                            end
                        end
                    end

                    // wait for R beat
                    while (!r_beat.r_last) begin
                        r_mbx.peek(r_beat_peek);
                        if (r_beat_peek.r_id == ar_beat.ax_id) begin
                            // this is our response
                            r_mbx.get(r_beat);
                            msg.add_to_cache_line(r_beat.r_data);
                            $display("%t ns %s.do_miss.check_axi: got R beat with ID 0x%0h and last = %0d for message : %s", $time, name, r_beat.r_id, r_beat.r_last, msg.print_me());
                            if (msg.trans_type == RD_REQ) begin
                                if (r_cnt == msg.data_offset) begin
                                    $display("%t ns %s.do_miss.check_axi: got R beat with valid data, changing type from RD_REQ to RD_RESP for message : %s", $time, name, msg.print_me());
                                    msg.trans_type = RD_RESP;
                                    msg.data       = r_beat.r_data;
                                end
                            end
                            r_cnt++;
                        end else begin
                            @(posedge sram_vif.clk);
                        end
                    end

                    msg.r_dirty  = r_beat.r_resp[2];
                    msg.r_shared = r_beat.r_resp[3];

                    if (msg.insert_readback) begin
                        // write readback data to cache
                        msg = new();
                        msg.trans_type    = READBACK;
                        msg.update_cache  = 1'b1;
                        msg.be            = '1;
                        msg.size          = 3;
                        msg.r_dirty =  1'b1; // we're modifying the read cacheline anyway
                        update_cache_line_data(msg.cache_line, msg.data, msg.be, msg.data_offset);

                        msg.insert_readback = 0;


                    end else begin
                        msg.prio = 0; // miss handler will do the final writeback
                    end

                end

                // Monitor hit status  . . . . . . . . . . . . . . . . . . . . .
                begin : mon_hit
                    // check if hit status changes, could be result of miss handler writeback
                    // in that case stop waiting for an AR beat
                    $display("%t ns %s.do_miss.mon_hit: monitoring hit status for message : %s", $time, name, msg.print_me());

                    while (!isHit(addr_v)) begin
                        while (!gnt_vif.rd_gnt[msg.port_idx + 2]) begin
                            // wait for read grant
                            @(posedge sram_vif.clk);
                        end
                        // got read grant, now wait one more cycle before checking hit status
                        @(posedge sram_vif.clk);
                    end

                    // status changed to hit, revert any changes in priority
                    msg.prio = msg.port_idx + 2;
                    if (msg.trans_type == WR_REQ) begin
                        msg.update_cache = 1'b1;
                    end else begin
                        msg.update_cache = 1'b0;
                    end

                    $display("%t ns %s.do_miss.mon_hit: Cache status changed from miss to hit, abort waiting for AR for message : %s", $time, name, msg.print_me());
                end
            join_any
            disable fork;


            if (isHit(addr_v) || msg.redo_hit) begin
                $display("%t ns %s.do_miss: Calling hit routine for message : %s", $time, name, msg.print_me());
                do_hit(msg);
            end

        endtask


        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // check behaviour when receiving dcache requests
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        local task automatic check_cache_msg;
            dcache_req    msg;
            dcache_req    readback_msg;
            logic [63:0]  addr_v;
            bit           CheckOK;
            bit timeout = 0;

            dcache_req_mbox_prio.get(msg);
            dcache_req_mbox_fwd.put(msg);

            // default
            msg.prio         = msg.port_idx + 2;
            msg.update_cache = 1'b0;

            $display("%t ns %s got dcache message : %s", $time, name, msg.print_me());
            addr_v = tag_index2addr(.tag(msg.address_tag), .index(msg.address_index));

            fork
                begin
                    // bypass
                    if (!is_inside_cacheable_regions(ArianeCfg, addr_v)) begin
                        automatic logic [3:0] exp_id = 4'h8 + msg.port_idx;
                        $display("%t ns %s message is outside cacheable region: %s", $time, name, msg.print_me());
                        if (msg.trans_type == WR_REQ) begin
                            b_beat_t b_beat            = new();
                            w_beat_t w_beat            = new();
                            ax_ace_beat_t aw_beat      = new();
                            ax_ace_beat_t aw_beat_peek = new();

                            if (is_inside_shareable_regions(ArianeCfg, addr_v)) begin
                                // wait for AW beat with expected ID
                                while (aw_beat_peek.ax_id[3:0] != exp_id) begin
                                    aw_mbx.peek(aw_beat_peek);
                                    if (aw_beat_peek.ax_id != exp_id) begin
                                        $display("%t ns %s.check_cache_msg: ignoring AW beat with ID 0x%0h for message : %s", $time, name, aw_beat_peek.ax_id, msg.print_me());
                                        @(posedge sram_vif.clk);
                                    end
                                end
                                aw_mbx.get(aw_beat);
                                $display("%t ns %s.check_cache_msg: got AW beat for message : %s", $time, name, msg.print_me());
                                if (!isWriteUnique(aw_beat))
                                    $error("%s.check_cache_msg : WRITE_UNIQUE request expected for message : %s", name, msg.print_me());
                            end else begin
                                int cnt = 0;

                                // wait for grant before checking AW, a snoop transaction may be active
                                $display("%t ns %s.check_cache_msg: wait for miss handler bypass grant for message : %s", $time, name, msg.print_me());
                                while (!gnt_vif.bypass_gnt[msg.port_idx]) begin
                                    @(posedge sram_vif.clk);
                                    cnt++;
                                    if (cnt > cache_msg_timeout) begin
                                        $error("%s.check_cache_msg : Timeout while waiting for grant before checking AW for message : %s", name, msg.print_me());
                                        break;
                                    end
                                end
                                $display("%t ns %s.check_cache_msg: got miss handler bypass grant for message : %s", $time, name, msg.print_me());

                                while (aw_beat_peek.ax_id[3:0] != exp_id) begin
                                    aw_mbx.peek(aw_beat_peek);
                                    if (aw_beat_peek.ax_id != exp_id) begin
                                        $display("%t ns %s.check_cache_msg: ignoring AW beat with ID 0x%0h for message : %s", $time, name, aw_beat_peek.ax_id, msg.print_me());
                                        @(posedge sram_vif.clk);
                                    end
                                end
                                aw_mbx.get(aw_beat);
                                $display("%t ns %s.check_cache_msg: got AW beat for message : %s", $time, name, msg.print_me());
                                if (!isWriteNoSnoop(aw_beat))
                                    $error("%s.check_cache_msg : WRITE_NO_SNOOP request expected for message : %s", name, msg.print_me());
                            end
                            a_empty_aw : assert (aw_mbx.num() == 0) else $error ("%S.check_cache_msg : AW mailbox not empty", name);

                            // wait for W beat
                            while (!w_beat.w_last) begin
                                w_mbx.get(w_beat);
                                $display("%t ns %s.check_cache_msg: got W beat with last = %0d for message : %s", $time, name, w_beat.w_last, msg.print_me());
                            end
                            a_empty_w : assert (w_mbx.num() == 0) else $error ("%S.check_cache_msg : W mailbox not empty", name);

                            // wait for B beat
                            b_mbx.get(b_beat);
                            a_empty_b : assert (b_mbx.num() == 0) else $error ("%S.check_cache_msg : B mailbox not empty", name);
                        end else begin
                            ax_ace_beat_t ar_beat      = new();
                            ax_ace_beat_t ar_beat_peek = new();
                            r_ace_beat_t  r_beat       = new();
                            r_ace_beat_t  r_beat_peek  = new();

                            int cnt = 0;
                            $display("%t ns %s.check_cache_msg: wait for miss handler bypass grant for message : %s", $time, name, msg.print_me());
                            while (!gnt_vif.bypass_gnt[msg.port_idx]) begin
                                @(posedge sram_vif.clk); // skip cycles without grant
                                cnt++;
                                if (cnt > cache_msg_timeout) begin
                                    $error("%s.check_cache_msg : Timeout while waiting for miss handler bypass grant for message : %s", name, msg.print_me());
                                    break;
                                end
                            end
                            $display("%t ns %s.check_cache_msg: got miss handler bypass grant for message : %s", $time, name, msg.print_me());

                            // wait for AR beat
                            $display("%t ns %s.check_cache_msg: waiting for AR beat for message : %s", $time, name, msg.print_me());
                            while (ar_beat_peek.ax_id[3:0] != exp_id) begin
                                ar_mbx.peek(ar_beat_peek);
                                if (ar_beat_peek.ax_id[3:0] == exp_id) begin
                                    // this is our response
                                    ar_mbx.get(ar_beat);
                                end else begin
                                    $display("%t ns %s.check_cache_msg: ignoring AR beat with ID 0x%0h for message : %s", $time, name, ar_beat_peek.ax_id, msg.print_me());
                                    @(posedge sram_vif.clk);
                                end
                            end
                            $display("%t ns %s.check_cache_msg: got AR beat for message : %s", $time, name, msg.print_me());

                            if (is_inside_shareable_regions(ArianeCfg, addr_v)) begin
                                if (!isReadOnce(ar_beat))
                                    $error("%s.check_cache_msg : READ_ONCE request expected for message : %s", name, msg.print_me());
                            end else begin
                                if (!isReadNoSnoop(ar_beat))
                                    $error("%s.check_cache_msg : READ_NO_SNOOP request expected for message : %s", name, msg.print_me());
                            end

                            // wait for R beat
                            $display("%t ns %s.check_cache_msg: waiting for R beat for message : %s", $time, name, msg.print_me());
                            while (!r_beat.r_last) begin
                                r_mbx.peek(r_beat_peek);
                                if (r_beat_peek.r_id == ar_beat.ax_id) begin
                                    // this is our response
                                    r_mbx.get(r_beat);
                                    $display("%t ns %s.check_cache_msg: got R beat with last = %0d and ID 0x%0h for message : %s", $time, name, r_beat.r_last, r_beat.r_id, msg.print_me());
                                end else begin
                                    $display("%t ns %s.check_cache_msg: ignoring R beat with ID 0x%0h for message : %s", $time, name, r_beat_peek.r_id, msg.print_me());
                                    @(posedge sram_vif.clk);
                                end
                            end

                            msg.r_dirty  = r_beat.r_resp[2];
                            msg.r_shared = r_beat.r_resp[3];

                        end
                    end
                    // cacheable
                    else begin
                        logic mshr_match = 0;
                        int cnt = 0;

                        // go to hit or miss routine
                        if (isHit(addr_v)) begin
                            do_hit(msg);
                        end else begin
                            do_miss(msg);
                        end

                        while (msg.redo_hit == 1) begin
                            // call hit routine again if requested
                            do_hit(msg);
                        end
                        $display("%t ns %s Sending message to cache update : %s", $time, name, msg.print_me());
                        req_to_cache_update.put(msg);

                    end

                end

                // timeout
                begin
                    automatic int cnt;
                    cnt = cache_msg_timeout;
                    while (cnt > 0) begin
                        cnt--;
                        @(posedge sram_vif.clk);
                    end
                    timeout = 1;
                end

            join_any

            if (timeout) $error("%s : Timeout in check_cache_msg for message : %s", name, msg.print_me());

        endtask



        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // check behaviour when flushing cache
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        local task automatic flush_cache;
            bit init     = 1;
            int init_cnt = 0;
            $display("%t ns %s.flush_cache: Flushing started", $time, name);

            // Wait for first grant before checking cache_status contents. Some other controller
            // may be updating the cache while the flush is pending.
            while (!gnt_vif.gnt[0]) begin
                $display("%t ns %s.flush_cache : waiting for first flush grant ", $time, name);
                @(posedge sram_vif.clk); // skip cycles without grant
                    init_cnt++;
                    if (init_cnt > 1000) begin
                        $error("%s.flush_cache : timeout while waiting for first flush grant", name);
                    break;
                end
            end

            for (int w = 0; w < DCACHE_NUM_WORDS; w++) begin
                int w_cnt = 0;
                for (int l = 0; l < DCACHE_SET_ASSOC; l++) begin
                    if (cache_status[w][l].valid && |cache_status[w][l].dirty) begin
                        fork
                            begin
                                automatic int ll = l;
                                automatic int ww = w;
                                // expect write back of dirty data
                                ax_ace_beat_t aw_beat = new();
                                b_beat_t      b_beat  = new();
                                w_beat_t      w_beat  = new();

                                // wait for AW beat
                                aw_mbx.get(aw_beat);
                                $display("%t ns %s.flush_cache: got AW beat for cache[%0d][%0d]", $time, name, ww, ll);
                                if (!isWriteBack(aw_beat))
                                    $error("%s.flush_cache : WRITEBACK request expected after eviction of cache[%0d][%0d]", name, ww, ll);
                                a_empty_aw : assert (aw_mbx.num() == 0) else $error ("%S.flush_cache : AW mailbox not empty", name);

                                // wait for W beat
                                while (!w_beat.w_last) begin
                                    w_mbx.get(w_beat);
                                    $display("%t ns %s.flush_cache: got W beat with last = %0d for cache[%0d][%0d]", $time, name, w_beat.w_last, ww, ll);
                                end
                                a_empty_w : assert (w_mbx.num() == 0) else $error ("%S.flush_cache : W mailbox not empty", name);

                                // wait for B beat
                                b_mbx.get(b_beat);
                                $display("%t ns %s.flush_cache: got B beat for cache[%0d][%0d]", $time, name, ww, ll);
                                a_empty_b : assert (b_mbx.num() == 0) else $error ("%S.flush_cache : B mailbox not empty", name);
                            end
                            begin
                                // expect clear of cache entry
                                automatic int ll  = l;
                                automatic int ww  = w;
                                automatic int cnt = 0;
                                while (!gnt_vif.gnt[0] && (init == 0)) begin
                                    $display("%t ns %s.flush_cache : skipping cycle without grant for flush of cache entry [%0d][%0d]", $time, name, ww, ll);
                                    @(posedge sram_vif.clk); // skip cycles without grant
                                    cnt++;
                                    if (cnt > 1000) begin
                                        $error("%s.flush_cache : timeout while waiting for grant for flush of cache entry [%0d][%0d]", name, ww, ll);
                                        break;
                                    end
                                end
                                init = 0;
                                @(posedge sram_vif.clk);

                                // clear entry in cache model
                                $display("%t ns %s.flush_cache: Flushing cache entry [%0d][%0d]", $time, name, ww, ll);
                                cache_status[ww][ll] = '0;
                            end
                        join_any
                    end
                end // l

                // expect clear of cache entry
                while (!gnt_vif.gnt[0] && (init == 0)) begin
                    $display("%t ns %s.flush_cache : skipping cycle without grant for clear of cache set [%0d]", $time, name, w);
                    @(posedge sram_vif.clk); // skip cycles without grant
                    w_cnt++;
                    if (w_cnt > 1000) begin
                        $error("%s.flush_cache : timeout while waiting for grant for clear of cache set [%0d]", name, w);
                        break;
                    end
                end
                init = 0;
                @(posedge sram_vif.clk);

                // clear entry in cache model
                $display("%t ns %s.flush_cache: Clear cache set [%0d]", $time, name, w);
                cache_status[w] = '0;

            end // w

        endtask

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // check behaviour when invalidating a cacheline
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        local task automatic invalidate (input logic[63:0] addr);
            int w_cnt = 0;
            for (int l = 0; l < DCACHE_SET_ASSOC; l++) begin
                int w = addr[DCACHE_INDEX_WIDTH-1:DCACHE_BYTE_OFFSET];
                if (cache_status[w][l].valid && |cache_status[w][l].dirty && cache_status[w][l].tag == addr[DCACHE_TAG_WIDTH+DCACHE_INDEX_WIDTH-1:DCACHE_INDEX_WIDTH]) begin
                    fork
                        begin
                            automatic int ll = l;
                            automatic int ww = w;
                            // expect write back of dirty data
                            ax_ace_beat_t aw_beat      = new();
                            ax_ace_beat_t aw_beat_peek = new();
                            b_beat_t      b_beat       = new();
                            w_beat_t      w_beat       = new();

                            while (!isDCache(aw_beat)) begin
                                aw_mbx.peek(aw_beat_peek);
                                if (isDCache(aw_beat_peek)) begin
                                    // this is our response
                                    aw_mbx.get(aw_beat);
                                end else begin
                                    $display("%t ns %s.invalidate: ignoring AW beat with ID 0x%0h for for cache[%0d][%0d]", $time, name, aw_beat_peek.ax_id, ww, ll);
                                    @(posedge sram_vif.clk);
                                end
                            end
                            $display("%t ns %s.invalidate: got AW beat for cache[%0d][%0d]", $time, name, ww, ll);

                            if (!isWriteBack(aw_beat))
                                $error("%s.invalidate : WRITEBACK request expected after eviction of cache[%0d][%0d]", name, ww, ll);
                            a_empty_aw : assert (aw_mbx.num() == 0) else $error ("%S.invalidate : AW mailbox not empty", name);

                            // wait for W beat
                            while (!w_beat.w_last) begin
                                w_mbx.get(w_beat);
                                $display("%t ns %s.invalidate: got W beat with last = %0d for cache[%0d][%0d]", $time, name, w_beat.w_last, ww, ll);
                            end
                            a_empty_w : assert (w_mbx.num() == 0) else $error ("%S.invalidate : W mailbox not empty", name);

                            // wait for B beat
                            b_mbx.get(b_beat);
                            $display("%t ns %s.invalidate: got B beat for cache[%0d][%0d]", $time, name, ww, ll);
                            a_empty_b : assert (b_mbx.num() == 0) else $error ("%S.invalidate : B mailbox not empty", name);
                        end

                        begin
                            // expect clear of cache entry
                            automatic int ll  = l;
                            automatic int ww  = w;
                            automatic int cnt = 0;
                            while (!gnt_vif.gnt[0]) begin
                                $display("%t ns %s.invalidate : skipping cycle without grant for evict of cache entry [%0d][%0d]", $time, name, ww, ll);
                                @(posedge sram_vif.clk); // skip cycles without grant
                                cnt++;
                                if (cnt > 1000) begin
                                    $error("%s.invalidate : timeout while waiting for grant for evict of cache entry [%0d][%0d]", name, ww, ll);
                                    break;
                                end
                            end
                            @(posedge sram_vif.clk);

                            // clear entry in cache model
                            $display("%t ns %s.invalidate: Evicting cache entry [%0d][%0d]", $time, name, ww, ll);
                            cache_status[ww][ll] = '0;
                        end
                    join_any

                    break;
                end
                else if (cache_status[w][l].valid && cache_status[w][l].tag == addr[DCACHE_TAG_WIDTH+DCACHE_INDEX_WIDTH-1:DCACHE_INDEX_WIDTH]) begin
                    // expect clear of cache entry
                    while (!gnt_vif.gnt[0]) begin
                        $display("%t ns %s.invalidate : skipping cycle without grant for clear of cache entry [%0d][%0d]", $time, name, w, l);
                        @(posedge sram_vif.clk); // skip cycles without grant
                        w_cnt++;
                        if (w_cnt > 1000) begin
                            $error("%s.invalidate : timeout while waiting for grant for clear of cache entry [%0d][%0d]", name, w, l);
                            break;
                        end
                    end
                    @(posedge sram_vif.clk);

                    // clear entry in cache model
                    $display("%t ns %s.invalidate: Evicting cache entry [%0d][%0d]", $time, name, w, l);
                    cache_status[w][l] = '0;

                    break;

                end
            end
        endtask

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // check behaviour when receiving AMO requests
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        local task automatic check_amo_msg;

            forever begin
                amo_req       msg;
                bit timeout = 0;

                amo_req_mbox.get(msg);
                $display("%t ns %s.check_amo_msg: Got amo message %s", $time, name, msg.print_me());
                amo_req_mbox_fwd.put(msg); // pass on to other checkers
                fork
                    begin
                        invalidate(msg.addr);
                        if (msg.op != AMO_LR) begin
                            ax_ace_beat_t aw_beat      = new();
                            ax_ace_beat_t aw_beat_peek = new();
                            b_beat_t      b_beat       = new();
                            b_beat_t      b_beat_peek  = new();
                            w_beat_t      w_beat       = new();
                            r_ace_beat_t  r_beat       = new();
                            r_ace_beat_t  r_beat_peek  = new();

                            // wait for AW beat
                            $display("%t ns %s.check_amo_msg: waiting for AW beat for message : %s", $time, name, msg.print_me());
                            while (!isAmo(aw_beat)) begin
                                aw_mbx.peek(aw_beat_peek);
                                if (isAmo(aw_beat_peek)) begin
                                    // this is our response
                                    aw_mbx.get(aw_beat);
                                end else begin
                                    $display("%t ns %s.check_amo_msg: ignoring AW beat with ID 0x%0h for message : %s", $time, name, aw_beat_peek.ax_id, msg.print_me());
                                    @(posedge sram_vif.clk);
                                end
                            end
                            $display("%t ns %s.check_amo_msg: got AW beat for message %s", $time, name, msg.print_me());

                            if (is_inside_shareable_regions(ArianeCfg, msg.addr)) begin
                                if (!isWriteUnique(aw_beat))
                                    $error("%s.check_amo_msg : WRITE_UNIQUE request expected for message %s", name, msg.print_me());
                            end else begin
                                if (!isWriteNoSnoop(aw_beat))
                                    $error("%s.check_amo_msg : WRITE_NO_SNOOP request expected for message %s", name, msg.print_me());
                            end
                            a_empty_aw : assert (aw_mbx.num() == 0) else $error ("%S.check_amo_msg : AW mailbox not empty", name);

                            // wait for W beat
                            while (!w_beat.w_last) begin
                                w_mbx.get(w_beat);
                                $display("%t ns %s.check_amo_msg: got W beat with last = %0d for message %s", $time, name, w_beat.w_last, msg.print_me());
                            end
                            a_empty_w : assert (w_mbx.num() == 0) else $error ("%S.check_amo_msg : W mailbox not empty", name);

                            // wait for B beat
                            while (b_beat_peek.b_id != aw_beat.ax_id) begin
                                b_mbx.peek(b_beat_peek);
                                if (b_beat_peek.b_id == aw_beat.ax_id) begin
                                    // this is our response
                                    b_mbx.get(b_beat);
                                    $display("%t ns %s.check_amo_msg: got B beat with ID 0x%0h for message %s", $time, name, b_beat.b_id, msg.print_me());
                                end else begin
                                    $display("%t ns %s.check_amo_msg: ignoring B beat with ID 0x%0h for message : %s", $time, name, b_beat_peek.b_id, msg.print_me());
                                    @(posedge sram_vif.clk);
                                end
                            end
                            a_empty_b : assert (b_mbx.num() == 0) else $error ("%S.check_amo_msg : B mailbox not empty", name);

                            if (msg.op != AMO_SC) begin // AMO_SC has no data response, only OK/ not OK decoded from B beat
                                // wait for R beat
                                while (!r_beat.r_last) begin
                                    r_mbx.peek(r_beat_peek);
                                    if (r_beat_peek.r_id == aw_beat.ax_id) begin
                                        // this is our response
                                        r_mbx.get(r_beat);
                                        $display("%t ns %s.check_amo_msg: got R beat with last = %0d for message %s", $time, name, r_beat.r_last, msg.print_me());
                                    end else begin
                                        @(posedge sram_vif.clk);
                                    end
                                end
                            end

                        end else begin
                            ax_ace_beat_t ar_beat      = new();
                            ax_ace_beat_t ar_beat_peek = new();
                            r_ace_beat_t  r_beat       = new();
                            r_ace_beat_t  r_beat_peek  = new();

                            $display("%t ns %s.check_amo_msg: waiting for AR beat for message : %s", $time, name, msg.print_me());
                            while (!isAmo(ar_beat)) begin
                                ar_mbx.peek(ar_beat_peek);
                                if (isAmo(ar_beat_peek)) begin
                                    // this is our response
                                    ar_mbx.get(ar_beat);
                                end else begin
                                    $display("%t ns %s.check_amo_msg: ignoring AR beat with ID 0x%0h for message : %s", $time, name, ar_beat_peek.ax_id, msg.print_me());
                                    @(posedge sram_vif.clk);
                                end
                            end
                            $display("%t ns %s.check_amo_msg: got AR beat for message %s", $time, name, msg.print_me());

                            if (is_inside_shareable_regions(ArianeCfg, msg.addr)) begin
                                if (!isReadOnce(ar_beat))
                                    $error("%s.check_amo_msg : READ_ONCE request expected for message %s",name, msg.print_me());
                            end else begin
                                if (!isReadNoSnoop(ar_beat))
                                    $error("%s.check_amo_msg : READ_NO_SNOOP request expected for message %s", name, msg.print_me());
                            end

                            // wait for R beat
                            while (!r_beat.r_last) begin
                                r_mbx.peek(r_beat_peek);
                                if (r_beat_peek.r_id == ar_beat.ax_id) begin
                                    // this is our response
                                    r_mbx.get(r_beat);
                                    $display("%t ns %s.check_amo_msg: got R beat with last = %0d for message %s", $time, name, r_beat.r_last, msg.print_me());
                                end else begin
                                    @(posedge sram_vif.clk);
                                end
                            end

                        end

                    end

                    // timeout
                    begin
                        automatic int cnt = 0;
                        while (cnt < amo_msg_timeout) begin
                            @(posedge sram_vif.clk);
                            cnt++;
                        end
                        timeout = 1;
                    end

                join_any
                if (timeout) $error("%s.check_amo_msg : Timeout for message %s", name, msg.print_me());
            end

        endtask

        // this task just forwards responses; no check done
        local task automatic check_amo_resp;
            forever begin
                amo_resp msg;
                amo_resp_mbox.get(msg);
                amo_resp_mbox_fwd.put(msg); // pass on to other checkers
            end
        endtask



        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Handle management transactions (currently only flush implemented)
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        local task automatic check_mgmt_trans;

            forever begin
                dcache_mgmt_trans trans;
                bit timeout = 0;

                mgmt_mbox.get(trans);
                $display("%t ns %s.check_mgmt_trans: Got management transaction %s", $time, name, trans.print_me());

                fork
                    begin
                        if (trans.trans_type == FLUSH_REQ) begin
                            flush_cache();
                        end
                    end

                    // timeout
                    begin
                        automatic int cnt = 0;
                        while (cnt < mgmt_trans_timeout) begin
                            @(posedge sram_vif.clk);
                            cnt++;
                        end
                        timeout = 1;
                    end

                join_any
                if (timeout) $error("%s.check_mgmt_trans : Timeout for transaction %s", name, trans.print_me());
            end

        endtask

        task run;
            fork
                get_cache_msg();
                get_cache_req_tmp();
                get_cache_resp_tmp();
                axi_ar_filter();
                axi_r_filter();
                axi_write_filter();
                check_snoop();
                check_amo_msg();
                check_amo_resp();
                check_mgmt_trans();
                update_cache_from_req();
                update_cache_from_snoop();
            join
        endtask

    endclass


    //--------------------------------------------------------------------------
    // dcache checker
    //--------------------------------------------------------------------------
    class std_dcache_checker #(
        parameter int unsigned NB_CORES        = 2,
        parameter int unsigned SRAM_DATA_WIDTH = 0,
        parameter int unsigned SRAM_NUM_WORDS  = 0
    );

        virtual dcache_sram_if                                                 dc_sram_vif [NB_CORES];
        virtual sram_intf #(DCACHE_SET_ASSOC, SRAM_DATA_WIDTH, SRAM_NUM_WORDS) sram_vif    [NB_CORES];
        int verbosity;

        string       name;
        ariane_cfg_t ArianeCfg;
        bit          enable_mem_check = 1;

        mailbox #(amo_req)  amo_req_mbox  [NB_CORES];
        mailbox #(amo_resp) amo_resp_mbox [NB_CORES];

        mailbox #(dcache_req)  dcache_req_mbox  [NB_CORES];
        mailbox #(dcache_resp) dcache_resp_mbox [NB_CORES];

        function new (
            virtual sram_intf #(DCACHE_SET_ASSOC, SRAM_DATA_WIDTH, SRAM_NUM_WORDS) sram_vif    [NB_CORES],
            virtual dcache_sram_if                                                 dc_sram_vif [NB_CORES],
            ariane_cfg_t                                                           cfg,
            string                                                                 name="std_dcache_checker"
        );
            this.name      = name;
            this.ArianeCfg = cfg;
            for (int c = 0; c < NB_CORES; c++) begin
                this.sram_vif[c]    = sram_vif[c];
                this.dc_sram_vif[c] = dc_sram_vif[c];
            end
            verbosity = 0;
        endfunction

        // check the cache contents vs main memory and other caches on every write to the cache status
        local task automatic mon_dcache;
            $display("%t ns %s monitoring dcache", $time, name);
            for (int c=0; c < NB_CORES; c++) begin
                fork
                    automatic int cc = c;

                    begin
                        forever begin
                            if (dc_sram_vif[cc].vld_req && dc_sram_vif[cc].vld_we) begin
                                logic [DCACHE_INDEX_WIDTH-DCACHE_BYTE_OFFSET-1:0] index;
                                index = dc_sram_vif[cc].vld_index;
                                $display("%t ns %s.monitor: Saw write to cache %0d, index 0x%3h",$time, name, cc, index);

                                @(posedge dc_sram_vif[cc].clk);
                                #0;

                                for (int cw=0; cw<DCACHE_SET_ASSOC; cw++) begin
                                    logic                         cc_valid, cc_dirty, cc_shared;
                                    logic [DCACHE_TAG_WIDTH:0]    cc_tag;
                                    logic [DCACHE_LINE_WIDTH-1:0] cc_data;
                                    logic [63:0]                  cc_addr;

                                    cc_dirty  = dc_sram_vif[cc].get_dirty_bit(.index(index), .way(cw));
                                    cc_valid  = dc_sram_vif[cc].get_valid(.index(index), .way(cw));
                                    cc_shared = dc_sram_vif[cc].get_shared(.index(index), .way(cw));
                                    cc_tag    = dc_sram_vif[cc].tag_sram[cw][index];
                                    cc_data   = dc_sram_vif[cc].data_sram[cw][index];
                                    cc_addr   = tag_index2addr(.tag(cc_tag), .index(index << DCACHE_BYTE_OFFSET));

                                    if (cc_valid && is_inside_shareable_regions(.Cfg(ArianeCfg), .address(cc_addr))) begin
                                        logic any_dirty;

                                        any_dirty = cc_dirty;
                                        // check entries in other caches
                                        for (int oc=0; oc < NB_CORES; oc++) begin
                                            if (oc != cc) begin
                                                for (int ow=0; ow<DCACHE_SET_ASSOC; ow++) begin
                                                    logic                         oc_valid, oc_dirty, oc_shared;
                                                    logic [DCACHE_TAG_WIDTH:0]    oc_tag;
                                                    logic [DCACHE_LINE_WIDTH-1:0] oc_data;

                                                    oc_dirty  = dc_sram_vif[oc].get_dirty_bit(.index(index), .way(ow));
                                                    oc_valid  = dc_sram_vif[oc].get_valid(.index(index), .way(ow));
                                                    oc_shared = dc_sram_vif[oc].get_shared(.index(index), .way(ow));
                                                    oc_tag    = dc_sram_vif[oc].tag_sram[ow][index];
                                                    oc_data   = dc_sram_vif[oc].data_sram[ow][index];

                                                    if (oc_valid && (oc_tag == cc_tag)) begin
                                                        any_dirty = any_dirty | oc_dirty;
                                                        $display("%t ns %s.monitor: Cache match for index 0x%3h, tag 0x%16h between way %0d in core %0d and way %0d in core %0d",$time, name, index, cc_tag, cw, cc, ow, oc);

                                                        // check that data matches
                                                        a_data : assert (cc_data == oc_data) else
                                                            $error("%s.monitor: Cache data mismatch for index %h, tag %h - core %0d, way %0d = 0x%16h_%16h, core %0d, way %0d = 0x%16h_%16h", name, index, cc_tag, cc, cw, cc_data[127:64], cc_data[63:0], oc, ow, oc_data[127:64], oc_data[63:0]);

                                                        // If data is present in both caches they should be marked shared.
                                                        // This will also implicitly check that a unique data is not present in
                                                        // any other chache.
                                                        a_cc_shared : assert (cc_shared) else
                                                            $error("%s.monitor: Expected shared = 1 for index 0x%3h, tag 0x%16h, way %0d, core %0d, got 0", name, index, cc_tag, cw, cc);
                                                        a_oc_shared : assert (oc_shared) else
                                                            $error("%s.monitor: Expected shared = 1 for index 0x%3h, tag 0x%16h, way %0d, core %0d, got 0", name, index, oc_tag, ow, oc);

                                                        // only one core could have the data marked as dirty
                                                        if (cc_dirty) begin
                                                            a_oc_clean : assert (!oc_dirty) else
                                                                $error("%s.monitor: Expected dirty = 0 for index 0x%3h, tag 0x%16h, way %0d, core %0d, got 1", name, index, oc_tag, ow, oc);
                                                        end


                                                    end
                                                end
                                            end
                                        end

                                        // check that data matches SRAM for globally clean entries
                                        if (enable_mem_check && (!any_dirty)) begin
                                            logic [63:0] addr;
                                            addr                  = {cc_tag, index};
                                            sram_vif[cc].addr[cw] = (addr - (ArianeCfg.ExecuteRegionAddrBase[3] >> DCACHE_BYTE_OFFSET)) << 1;
                                            #0
                                            a_mem_data : assert (cc_data == sram_vif[cc].data[cw]) else
                                                $error("%s.monitor: Cache vs Memory data mismatch for index %h, tag %h - core %0d, way %0d = 0x%16h_%16h, Memory[0x%16h] = 0x%16h_%16h", name, index, cc_tag, cc, cw, cc_data[127:64], cc_data[63:0], sram_vif[cc].addr[cw], sram_vif[cc].data[cw][1], sram_vif[cc].data[cw][0]);
                                        end
                                    end
                                end
                            end else begin
                                @(posedge dc_sram_vif[cc].clk);
                            end
                        end
                    end

                join_none
            end
            wait fork;
        endtask


        // custom checker to verify lock / unlock using amoswap
        task automatic check_amo_lock;

            logic locks [ logic[63:0] ];
            int ids [ logic[63:0] ];

            for (int c=0; c < NB_CORES; c++) begin
                fork
                    automatic int cc = c;
                    begin
                        fork
                            // AMO SWAP monitoring
                            begin
                                $display("%t ns %s.check_amo_lock: Monitoring AMO lock / unlock in core %0d", $time, name, cc);
                                forever begin
                                    automatic logic   attempt_lock    = 0;
                                    automatic logic   attempt_release = 0;
                                    automatic longint attempt_addr    = 0;
                                    amo_req  req  = new();
                                    amo_resp resp = new();

                                    amo_req_mbox[cc].get(req);
                                    if (req.op == AMO_SWAP) begin
                                        $display("%t ns %s.check_amo_lock: Got amo request from core %0d: %s", $time, name, cc, req.print_me());
                                        if (req.data == 1) begin
                                            attempt_lock = 1;
                                        end
                                        if (req.data == 0) begin
                                            attempt_release = 1;
                                        end
                                        attempt_addr = req.addr;
                                    end

                                    amo_resp_mbox[cc].get(resp);
                                    if (req.op == AMO_SWAP) begin
                                        $display("%t ns %s.check_amo_lock: Got amo response from core %0d: %s", $time, name, cc, resp.print_me());
                                    end

                                    if (attempt_lock) begin
                                        // initialize lock
                                        if (!locks.exists(attempt_addr)) begin
                                            locks[attempt_addr] = 0;
                                            ids[attempt_addr] = -1;
                                        end

                                        if (resp.data == 0) begin
                                            // lock granted
                                            $display("%t ns %s.check_amo_lock: Lock %8h granted to core %0d", $time, name, attempt_addr, cc);
                                            assert (locks[attempt_addr] == 0) else $error("%s.check_amo_lock: Lock %8h granted to core %0d but was already held by core %0d", name, attempt_addr, cc, ids[attempt_addr]);
                                            locks[attempt_addr] = 1;
                                            ids[attempt_addr] = cc;
                                        end else begin
                                            $display("%t ns %s.check_amo_lock: Lock %8h denied to core %0d", $time, name, attempt_addr, cc);
                                        end
                                    end

                                    if (attempt_release) begin
                                        if (resp.data == 1) begin
                                            // lock released
                                            $display("%t ns %s.check_amo_lock: Lock %8h released by core %0d", $time, name, attempt_addr, cc);
                                            assert ((locks[attempt_addr] == 1) && (ids[attempt_addr] == cc)) else
                                                $error("%s.check_amo_lock: Lock %8h released by AMO from core %0d but was already held by core %0d", name, attempt_addr, cc, ids[attempt_addr]);
                                            locks[attempt_addr] = 0;
                                            ids[attempt_addr] = -1;
                                        end else begin
                                            $error("%s.check_amo_lock: Lock %8h release to core %0d failed", name, attempt_addr, cc);
                                        end
                                    end

                                end
                            end

                            begin
                                forever begin
                                    automatic logic   attempt_lock    = 0;
                                    automatic logic   attempt_release = 0;
                                    automatic longint attempt_addr    = 0;
                                    automatic logic   init_addr;
                                    dcache_req  req;
                                    dcache_resp resp;

                                    init_addr = 0;
                                    dcache_req_mbox[cc].get(req);
                                    if (req.trans_type == WR_REQ) begin
                                        attempt_addr = req.get_addr();

                                        // initialize lock
                                        if (!locks.exists(attempt_addr)) begin
                                            locks[attempt_addr] = 0;
                                            ids[attempt_addr] = -1;
                                            init_addr = 1;
                                        end

                                        if (verbosity > 0) begin
                                            $display("%t ns %s.check_amo_lock: Got dcache request from core %0d: %s", $time, name, cc, req.print_me());
                                        end
                                        if (req.data == 1) begin
                                            attempt_lock = 1;
                                        end
                                        if (req.data == 0) begin
                                            attempt_release = 1;
                                        end

                                        if (attempt_lock) begin
                                            assert (locks[attempt_addr] == 0) else $error("%s.check_amo_lock: Core %0d write to locked address %8h",name, cc, attempt_addr);
                                            locks[attempt_addr] = 1;
                                            ids[attempt_addr] = cc;
                                        end

                                        do begin
                                            dcache_resp_mbox[cc].get(resp);
                                        end while (resp.trans_type != WR_RESP);
                                        if (verbosity > 0) begin
                                            $display("%t ns %s.check_amo_lock: Got dcache response from core %0d: %s", $time, name, cc, resp.print_me());
                                        end

                                        if (attempt_release) begin
                                            // lock released
                                            $display("%t ns %s.check_amo_lock: Lock %8h released by write from core %0d", $time, name, attempt_addr, cc);
                                            if (!init_addr) begin
                                                assert ((locks[attempt_addr] == 0) || ((locks[attempt_addr] == 1) && (ids[attempt_addr] == cc))) else $error("%s.check_amo_lock: Lock %8h released by write from core %0d but was already held by core %0d", name, attempt_addr, cc, ids[attempt_addr]);
                                            end
                                            locks[attempt_addr] = 0;
                                            ids[attempt_addr] = -1;
                                        end
                                    end
                                end
                            end
                        join_none
                    end
                join_none
            end
            wait fork;

        endtask


        task monitor;
            mon_dcache();
            // check_amo_lock() must be explicitly started from the tests that require it
        endtask

    endclass


endpackage
