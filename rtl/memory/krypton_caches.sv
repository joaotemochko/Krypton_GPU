//==============================================================================
// GPU Krypton Series-1 - Memory Subsystem
// L1 Instruction Cache, L1 Texture Cache, L2 Unified Cache
//==============================================================================

//------------------------------------------------------------------------------
// L1 Instruction Cache (32KB per Cluster)
//------------------------------------------------------------------------------

module krypton_l1_icache
    import krypton_pkg::*;
#(
    parameter int CACHE_SIZE_KB = 32,
    parameter int LINE_SIZE     = 64,
    parameter int ASSOCIATIVITY = 4
)(
    input  logic                        clk,
    input  logic                        rst_n,
    
    input  logic                        req_valid,
    output logic                        req_ready,
    input  logic [ADDR_WIDTH-1:0]       req_addr,
    
    output logic                        resp_valid,
    output logic [255:0]                resp_data,
    
    output logic                        l2_req,
    output logic [ADDR_WIDTH-1:0]       l2_addr,
    input  logic                        l2_valid,
    input  logic [511:0]                l2_data,
    
    output logic [31:0]                 hit_count,
    output logic [31:0]                 miss_count
);

    localparam int NUM_SETS = (CACHE_SIZE_KB * 1024) / (LINE_SIZE * ASSOCIATIVITY);
    localparam int INDEX_BITS = $clog2(NUM_SETS);
    localparam int OFFSET_BITS = $clog2(LINE_SIZE);
    localparam int TAG_BITS = ADDR_WIDTH - INDEX_BITS - OFFSET_BITS;
    
    logic [TAG_BITS-1:0]    tag_array [0:NUM_SETS-1][0:ASSOCIATIVITY-1];
    logic [LINE_SIZE*8-1:0] data_array [0:NUM_SETS-1][0:ASSOCIATIVITY-1];
    logic                   valid_array [0:NUM_SETS-1][0:ASSOCIATIVITY-1];
    logic [2:0]             lru_array [0:NUM_SETS-1][0:ASSOCIATIVITY-1];
    
    logic [TAG_BITS-1:0]    req_tag;
    logic [INDEX_BITS-1:0]  req_index;
    logic [OFFSET_BITS-1:0] req_offset;
    
    assign req_tag    = req_addr[ADDR_WIDTH-1:INDEX_BITS+OFFSET_BITS];
    assign req_index  = req_addr[INDEX_BITS+OFFSET_BITS-1:OFFSET_BITS];
    assign req_offset = req_addr[OFFSET_BITS-1:0];
    
    logic [ASSOCIATIVITY-1:0] hit_way;
    logic                     cache_hit;
    logic [1:0]               hit_way_idx;
    
    always_comb begin
        hit_way = '0;
        for (int w = 0; w < ASSOCIATIVITY; w++) begin
            if (valid_array[req_index][w] && tag_array[req_index][w] == req_tag)
                hit_way[w] = 1'b1;
        end
        cache_hit = |hit_way;
        
        hit_way_idx = '0;
        for (int w = 0; w < ASSOCIATIVITY; w++) begin
            if (hit_way[w])
                hit_way_idx = w[1:0];
        end
    end
    
    typedef enum logic [1:0] {
        ST_IDLE,
        ST_CHECK,
        ST_MISS,
        ST_FILL
    } state_t;
    
    state_t state;
    logic [1:0] victim_way;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            resp_valid <= 1'b0;
            l2_req <= 1'b0;
            hit_count <= '0;
            miss_count <= '0;
            
            for (int s = 0; s < NUM_SETS; s++) begin
                for (int w = 0; w < ASSOCIATIVITY; w++) begin
                    valid_array[s][w] <= 1'b0;
                    lru_array[s][w] <= w[2:0];
                end
            end
        end else begin
            resp_valid <= 1'b0;
            
            case (state)
                ST_IDLE: begin
                    if (req_valid)
                        state <= ST_CHECK;
                end
                
                ST_CHECK: begin
                    if (cache_hit) begin
                        resp_valid <= 1'b1;
                        resp_data <= data_array[req_index][hit_way_idx][req_offset*8 +: 256];
                        hit_count <= hit_count + 1;
                        
                        for (int w = 0; w < ASSOCIATIVITY; w++) begin
                            if (lru_array[req_index][w] < lru_array[req_index][hit_way_idx])
                                lru_array[req_index][w] <= lru_array[req_index][w] + 1;
                        end
                        lru_array[req_index][hit_way_idx] <= '0;
                        
                        state <= ST_IDLE;
                    end else begin
                        l2_req <= 1'b1;
                        l2_addr <= {req_addr[ADDR_WIDTH-1:OFFSET_BITS], {OFFSET_BITS{1'b0}}};
                        miss_count <= miss_count + 1;
                        
                        victim_way <= '0;
                        for (int w = 0; w < ASSOCIATIVITY; w++) begin
                            if (lru_array[req_index][w] == ASSOCIATIVITY - 1)
                                victim_way <= w[1:0];
                        end
                        
                        state <= ST_MISS;
                    end
                end
                
                ST_MISS: begin
                    if (l2_valid) begin
                        l2_req <= 1'b0;
                        state <= ST_FILL;
                    end
                end
                
                ST_FILL: begin
                    data_array[req_index][victim_way] <= l2_data;
                    tag_array[req_index][victim_way] <= req_tag;
                    valid_array[req_index][victim_way] <= 1'b1;
                    
                    for (int w = 0; w < ASSOCIATIVITY; w++) begin
                        if (lru_array[req_index][w] < lru_array[req_index][victim_way])
                            lru_array[req_index][w] <= lru_array[req_index][w] + 1;
                    end
                    lru_array[req_index][victim_way] <= '0;
                    
                    resp_valid <= 1'b1;
                    resp_data <= l2_data[req_offset*8 +: 256];
                    
                    state <= ST_IDLE;
                end
            endcase
        end
    end
    
    assign req_ready = (state == ST_IDLE);

endmodule : krypton_l1_icache

//------------------------------------------------------------------------------
// L1 Texture Cache (32KB per Cluster)
//------------------------------------------------------------------------------

module krypton_l1_tcache
    import krypton_pkg::*;
#(
    parameter int CACHE_SIZE_KB = 32,
    parameter int LINE_SIZE     = 64,
    parameter int ASSOCIATIVITY = 4
)(
    input  logic                        clk,
    input  logic                        rst_n,
    
    input  logic                        req_valid,
    output logic                        req_ready,
    input  logic [ADDR_WIDTH-1:0]       req_addr,
    
    output logic                        resp_valid,
    output logic [127:0]                resp_data,
    
    output logic                        l2_req,
    output logic [ADDR_WIDTH-1:0]       l2_addr,
    input  logic                        l2_valid,
    input  logic [511:0]                l2_data,
    
    output logic [31:0]                 hit_count,
    output logic [31:0]                 miss_count
);

    // Similar implementation to I-cache
    localparam int NUM_SETS = (CACHE_SIZE_KB * 1024) / (LINE_SIZE * ASSOCIATIVITY);
    localparam int INDEX_BITS = $clog2(NUM_SETS);
    localparam int OFFSET_BITS = $clog2(LINE_SIZE);
    localparam int TAG_BITS = ADDR_WIDTH - INDEX_BITS - OFFSET_BITS;
    
    logic [TAG_BITS-1:0]    tag_array [0:NUM_SETS-1][0:ASSOCIATIVITY-1];
    logic [LINE_SIZE*8-1:0] data_array [0:NUM_SETS-1][0:ASSOCIATIVITY-1];
    logic                   valid_array [0:NUM_SETS-1][0:ASSOCIATIVITY-1];
    logic [2:0]             lru_array [0:NUM_SETS-1][0:ASSOCIATIVITY-1];
    
    logic [TAG_BITS-1:0]    req_tag;
    logic [INDEX_BITS-1:0]  req_index;
    logic [OFFSET_BITS-1:0] req_offset;
    
    assign req_tag    = req_addr[ADDR_WIDTH-1:INDEX_BITS+OFFSET_BITS];
    assign req_index  = req_addr[INDEX_BITS+OFFSET_BITS-1:OFFSET_BITS];
    assign req_offset = req_addr[OFFSET_BITS-1:0];
    
    logic [ASSOCIATIVITY-1:0] hit_way;
    logic cache_hit;
    logic [1:0] hit_way_idx;
    
    always_comb begin
        hit_way = '0;
        for (int w = 0; w < ASSOCIATIVITY; w++) begin
            if (valid_array[req_index][w] && tag_array[req_index][w] == req_tag)
                hit_way[w] = 1'b1;
        end
        cache_hit = |hit_way;
        
        hit_way_idx = '0;
        for (int w = 0; w < ASSOCIATIVITY; w++) begin
            if (hit_way[w])
                hit_way_idx = w[1:0];
        end
    end
    
    typedef enum logic [1:0] {
        ST_IDLE,
        ST_CHECK,
        ST_MISS,
        ST_FILL
    } state_t;
    
    state_t state;
    logic [1:0] victim_way;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            resp_valid <= 1'b0;
            l2_req <= 1'b0;
            hit_count <= '0;
            miss_count <= '0;
            
            for (int s = 0; s < NUM_SETS; s++) begin
                for (int w = 0; w < ASSOCIATIVITY; w++) begin
                    valid_array[s][w] <= 1'b0;
                    lru_array[s][w] <= w[2:0];
                end
            end
        end else begin
            resp_valid <= 1'b0;
            
            case (state)
                ST_IDLE: begin
                    if (req_valid)
                        state <= ST_CHECK;
                end
                
                ST_CHECK: begin
                    if (cache_hit) begin
                        resp_valid <= 1'b1;
                        resp_data <= data_array[req_index][hit_way_idx][req_offset*8 +: 128];
                        hit_count <= hit_count + 1;
                        
                        for (int w = 0; w < ASSOCIATIVITY; w++) begin
                            if (lru_array[req_index][w] < lru_array[req_index][hit_way_idx])
                                lru_array[req_index][w] <= lru_array[req_index][w] + 1;
                        end
                        lru_array[req_index][hit_way_idx] <= '0;
                        
                        state <= ST_IDLE;
                    end else begin
                        l2_req <= 1'b1;
                        l2_addr <= {req_addr[ADDR_WIDTH-1:OFFSET_BITS], {OFFSET_BITS{1'b0}}};
                        miss_count <= miss_count + 1;
                        
                        victim_way <= '0;
                        for (int w = 0; w < ASSOCIATIVITY; w++) begin
                            if (lru_array[req_index][w] == ASSOCIATIVITY - 1)
                                victim_way <= w[1:0];
                        end
                        
                        state <= ST_MISS;
                    end
                end
                
                ST_MISS: begin
                    if (l2_valid) begin
                        l2_req <= 1'b0;
                        state <= ST_FILL;
                    end
                end
                
                ST_FILL: begin
                    data_array[req_index][victim_way] <= l2_data;
                    tag_array[req_index][victim_way] <= req_tag;
                    valid_array[req_index][victim_way] <= 1'b1;
                    
                    for (int w = 0; w < ASSOCIATIVITY; w++) begin
                        if (lru_array[req_index][w] < lru_array[req_index][victim_way])
                            lru_array[req_index][w] <= lru_array[req_index][w] + 1;
                    end
                    lru_array[req_index][victim_way] <= '0;
                    
                    resp_valid <= 1'b1;
                    resp_data <= l2_data[req_offset*8 +: 128];
                    
                    state <= ST_IDLE;
                end
            endcase
        end
    end
    
    assign req_ready = (state == ST_IDLE);

endmodule : krypton_l1_tcache

//------------------------------------------------------------------------------
// L2 Unified Cache (512KB)
//------------------------------------------------------------------------------

module krypton_l2_cache
    import krypton_pkg::*;
#(
    parameter int CACHE_SIZE_KB = 512,
    parameter int LINE_SIZE     = 64,
    parameter int ASSOCIATIVITY = 8,
    parameter int NUM_PORTS     = 8
)(
    input  logic                        clk,
    input  logic                        rst_n,
    
    input  logic [NUM_PORTS-1:0]        req_valid,
    output logic [NUM_PORTS-1:0]        req_ready,
    input  logic [ADDR_WIDTH-1:0]       req_addr [0:NUM_PORTS-1],
    input  logic [NUM_PORTS-1:0]        req_write,
    input  logic [511:0]                req_wdata [0:NUM_PORTS-1],
    
    output logic [NUM_PORTS-1:0]        resp_valid,
    output logic [511:0]                resp_data [0:NUM_PORTS-1],
    
    // AXI4 interface
    output logic                        m_axi_arvalid,
    input  logic                        m_axi_arready,
    output logic [ADDR_WIDTH-1:0]       m_axi_araddr,
    output logic [7:0]                  m_axi_arlen,
    
    input  logic                        m_axi_rvalid,
    output logic                        m_axi_rready,
    input  logic [AXI_DATA_WIDTH-1:0]   m_axi_rdata,
    input  logic                        m_axi_rlast,
    
    output logic                        m_axi_awvalid,
    input  logic                        m_axi_awready,
    output logic [ADDR_WIDTH-1:0]       m_axi_awaddr,
    output logic [7:0]                  m_axi_awlen,
    
    output logic                        m_axi_wvalid,
    input  logic                        m_axi_wready,
    output logic [AXI_DATA_WIDTH-1:0]   m_axi_wdata,
    output logic                        m_axi_wlast,
    
    input  logic                        m_axi_bvalid,
    output logic                        m_axi_bready,
    
    output logic [63:0]                 total_hits,
    output logic [63:0]                 total_misses
);

    localparam int NUM_SETS = (CACHE_SIZE_KB * 1024) / (LINE_SIZE * ASSOCIATIVITY);
    localparam int INDEX_BITS = $clog2(NUM_SETS);
    localparam int OFFSET_BITS = $clog2(LINE_SIZE);
    localparam int TAG_BITS = ADDR_WIDTH - INDEX_BITS - OFFSET_BITS;
    
    logic [TAG_BITS-1:0]    tag_array [0:NUM_SETS-1][0:ASSOCIATIVITY-1];
    logic [LINE_SIZE*8-1:0] data_array [0:NUM_SETS-1][0:ASSOCIATIVITY-1];
    logic                   valid_array [0:NUM_SETS-1][0:ASSOCIATIVITY-1];
    logic                   dirty_array [0:NUM_SETS-1][0:ASSOCIATIVITY-1];
    logic [3:0]             lru_array [0:NUM_SETS-1][0:ASSOCIATIVITY-1];
    
    logic [2:0] grant_idx;
    logic grant_valid;
    logic [2:0] arb_ptr;
    
    always_comb begin
        grant_valid = 1'b0;
        grant_idx = '0;
        for (int i = 0; i < NUM_PORTS; i++) begin
            int idx;
            idx = (arb_ptr + i) % NUM_PORTS;
            if (req_valid[idx] && !grant_valid) begin
                grant_valid = 1'b1;
                grant_idx = idx[2:0];
            end
        end
    end
    
    typedef enum logic [2:0] {
        ST_IDLE,
        ST_CHECK,
        ST_WRITEBACK,
        ST_FILL,
        ST_RESPOND
    } state_t;
    
    state_t state;
    logic [2:0] current_grant;
    logic [2:0] victim_way;
    logic [511:0] fill_buffer;
    
    logic [TAG_BITS-1:0] active_tag;
    logic [INDEX_BITS-1:0] active_index;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            arb_ptr <= '0;
            total_hits <= '0;
            total_misses <= '0;
            m_axi_arvalid <= 1'b0;
            m_axi_awvalid <= 1'b0;
            m_axi_wvalid <= 1'b0;
            m_axi_rready <= 1'b0;
            m_axi_bready <= 1'b1;
            
            for (int i = 0; i < NUM_PORTS; i++) begin
                resp_valid[i] <= 1'b0;
                req_ready[i] <= 1'b0;
            end
            
            for (int s = 0; s < NUM_SETS; s++) begin
                for (int w = 0; w < ASSOCIATIVITY; w++) begin
                    valid_array[s][w] <= 1'b0;
                    dirty_array[s][w] <= 1'b0;
                end
            end
        end else begin
            for (int i = 0; i < NUM_PORTS; i++) begin
                resp_valid[i] <= 1'b0;
                req_ready[i] <= 1'b0;
            end
            
            case (state)
                ST_IDLE: begin
                    if (grant_valid) begin
                        current_grant <= grant_idx;
                        active_tag <= req_addr[grant_idx][ADDR_WIDTH-1:INDEX_BITS+OFFSET_BITS];
                        active_index <= req_addr[grant_idx][INDEX_BITS+OFFSET_BITS-1:OFFSET_BITS];
                        state <= ST_CHECK;
                    end
                end
                
                ST_CHECK: begin
                    logic hit_found;
                    logic [2:0] hit_w;
                    
                    hit_found = 1'b0;
                    hit_w = '0;
                    for (int w = 0; w < ASSOCIATIVITY; w++) begin
                        if (valid_array[active_index][w] && 
                            tag_array[active_index][w] == active_tag) begin
                            hit_found = 1'b1;
                            hit_w = w[2:0];
                        end
                    end
                    
                    if (hit_found) begin
                        total_hits <= total_hits + 1;
                        resp_data[current_grant] <= data_array[active_index][hit_w];
                        state <= ST_RESPOND;
                    end else begin
                        total_misses <= total_misses + 1;
                        
                        // Find LRU victim
                        for (int w = 0; w < ASSOCIATIVITY; w++) begin
                            if (lru_array[active_index][w] == ASSOCIATIVITY - 1)
                                victim_way <= w[2:0];
                        end
                        
                        if (dirty_array[active_index][victim_way])
                            state <= ST_WRITEBACK;
                        else
                            state <= ST_FILL;
                    end
                end
                
                ST_WRITEBACK: begin
                    // Simplified writeback
                    m_axi_awvalid <= 1'b1;
                    m_axi_awaddr <= {tag_array[active_index][victim_way], 
                                    active_index, {OFFSET_BITS{1'b0}}};
                    m_axi_awlen <= 8'd1;
                    
                    if (m_axi_awready) begin
                        m_axi_awvalid <= 1'b0;
                        m_axi_wvalid <= 1'b1;
                        m_axi_wdata <= data_array[active_index][victim_way][255:0];
                        m_axi_wlast <= 1'b0;
                    end
                    
                    if (m_axi_wready && m_axi_wvalid) begin
                        if (!m_axi_wlast) begin
                            m_axi_wdata <= data_array[active_index][victim_way][511:256];
                            m_axi_wlast <= 1'b1;
                        end else begin
                            m_axi_wvalid <= 1'b0;
                            dirty_array[active_index][victim_way] <= 1'b0;
                            state <= ST_FILL;
                        end
                    end
                end
                
                ST_FILL: begin
                    m_axi_arvalid <= 1'b1;
                    m_axi_araddr <= {active_tag, active_index, {OFFSET_BITS{1'b0}}};
                    m_axi_arlen <= 8'd1;
                    m_axi_rready <= 1'b1;
                    
                    if (m_axi_arready)
                        m_axi_arvalid <= 1'b0;
                    
                    if (m_axi_rvalid) begin
                        if (!m_axi_rlast) begin
                            fill_buffer[255:0] <= m_axi_rdata;
                        end else begin
                            fill_buffer[511:256] <= m_axi_rdata;
                            m_axi_rready <= 1'b0;
                            
                            // Update cache
                            data_array[active_index][victim_way] <= {m_axi_rdata, fill_buffer[255:0]};
                            tag_array[active_index][victim_way] <= active_tag;
                            valid_array[active_index][victim_way] <= 1'b1;
                            
                            resp_data[current_grant] <= {m_axi_rdata, fill_buffer[255:0]};
                            state <= ST_RESPOND;
                        end
                    end
                end
                
                ST_RESPOND: begin
                    resp_valid[current_grant] <= 1'b1;
                    req_ready[current_grant] <= 1'b1;
                    arb_ptr <= (arb_ptr + 1) % NUM_PORTS;
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule : krypton_l2_cache
