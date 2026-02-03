//==============================================================================
// GPU Krypton Series-1 - Cache Coherence Controller
// Protocol: MSI (Modified-Shared-Invalid) - Simplified MESI for GPU
// Manages coherence between 4 shader cluster L1 caches and shared L2
//==============================================================================

module krypton_cache_coherence
    import krypton_pkg::*;
#(
    parameter int NUM_CLUSTERS      = 4,
    parameter int ADDR_WIDTH        = 40,
    parameter int LINE_SIZE         = 64,
    parameter int SNOOP_QUEUE_DEPTH = 8,
    parameter int TAG_BITS          = 24
)(
    input  logic                        clk,
    input  logic                        rst_n,
    
    //==========================================================================
    // L1 Cache Interfaces (per cluster)
    //==========================================================================
    
    // L1 to Coherence Controller requests
    input  logic [NUM_CLUSTERS-1:0]                 l1_req_valid,
    output logic [NUM_CLUSTERS-1:0]                 l1_req_ready,
    input  logic [ADDR_WIDTH-1:0]                   l1_req_addr [0:NUM_CLUSTERS-1],
    input  logic [NUM_CLUSTERS-1:0]                 l1_req_write,
    input  logic [NUM_CLUSTERS-1:0]                 l1_req_upgrade,     // S->M upgrade
    input  logic [LINE_SIZE*8-1:0]                  l1_req_wdata [0:NUM_CLUSTERS-1],
    
    // Coherence Controller to L1 responses
    output logic [NUM_CLUSTERS-1:0]                 l1_resp_valid,
    output logic [LINE_SIZE*8-1:0]                  l1_resp_data [0:NUM_CLUSTERS-1],
    output logic [1:0]                              l1_resp_state [0:NUM_CLUSTERS-1],  // New cache line state
    
    // Snoop interface (Coherence Controller to L1)
    output logic [NUM_CLUSTERS-1:0]                 snoop_req_valid,
    input  logic [NUM_CLUSTERS-1:0]                 snoop_req_ready,
    output logic [ADDR_WIDTH-1:0]                   snoop_addr,
    output logic [1:0]                              snoop_type,         // 0=Inv, 1=RdShared, 2=RdExcl
    
    input  logic [NUM_CLUSTERS-1:0]                 snoop_resp_valid,
    input  logic [NUM_CLUSTERS-1:0]                 snoop_resp_hit,
    input  logic [NUM_CLUSTERS-1:0]                 snoop_resp_dirty,
    input  logic [LINE_SIZE*8-1:0]                  snoop_resp_data [0:NUM_CLUSTERS-1],
    
    //==========================================================================
    // L2 Cache Interface
    //==========================================================================
    
    output logic                        l2_req_valid,
    input  logic                        l2_req_ready,
    output logic [ADDR_WIDTH-1:0]       l2_req_addr,
    output logic                        l2_req_write,
    output logic [LINE_SIZE*8-1:0]      l2_req_wdata,
    
    input  logic                        l2_resp_valid,
    input  logic [LINE_SIZE*8-1:0]      l2_resp_data,
    
    //==========================================================================
    // Directory/Snoop Filter
    //==========================================================================
    // Tracks which L1s have copies of each cache line
    
    output logic                        dir_update_valid,
    output logic [ADDR_WIDTH-1:0]       dir_update_addr,
    output logic [NUM_CLUSTERS-1:0]     dir_update_sharers,
    output logic [1:0]                  dir_update_owner,       // Exclusive owner (if any)
    output logic                        dir_update_has_owner,
    
    //==========================================================================
    // Fence/Barrier Support
    //==========================================================================
    
    input  logic [NUM_CLUSTERS-1:0]     fence_req,
    output logic [NUM_CLUSTERS-1:0]     fence_done,
    
    //==========================================================================
    // Performance Counters
    //==========================================================================
    
    output logic [31:0]                 coherence_transactions,
    output logic [31:0]                 invalidations_sent,
    output logic [31:0]                 writebacks_received
);

    //==========================================================================
    // Cache Line States (MSI Protocol)
    //==========================================================================
    
    typedef enum logic [1:0] {
        STATE_INVALID   = 2'b00,    // I: Line not present
        STATE_SHARED    = 2'b01,    // S: Clean copy, others may have copies
        STATE_MODIFIED  = 2'b10,    // M: Dirty, exclusive owner
        STATE_EXCLUSIVE = 2'b11     // E: Clean, exclusive (optional optimization)
    } cache_state_t;
    
    // Snoop types
    typedef enum logic [1:0] {
        SNOOP_INVALIDATE    = 2'b00,    // Invalidate line
        SNOOP_READ_SHARED   = 2'b01,    // Read for shared
        SNOOP_READ_EXCL     = 2'b10,    // Read for exclusive/modify
        SNOOP_WRITEBACK     = 2'b11     // Request writeback
    } snoop_type_t;
    
    //==========================================================================
    // Directory Entry Structure
    //==========================================================================
    
    typedef struct packed {
        logic                   valid;
        logic [TAG_BITS-1:0]    tag;
        logic [NUM_CLUSTERS-1:0] sharers;       // Bit vector of sharers
        logic [1:0]             owner;          // Owner cluster ID (if exclusive)
        logic                   has_owner;      // Has exclusive owner
        cache_state_t           state;          // Overall line state
    } directory_entry_t;
    
    // Directory (simplified: direct-mapped for demo, real impl would be set-assoc)
    localparam int DIR_SETS = 1024;
    localparam int DIR_INDEX_BITS = $clog2(DIR_SETS);
    
    directory_entry_t directory [0:DIR_SETS-1];
    
    //==========================================================================
    // Request Queue
    //==========================================================================
    
    typedef struct packed {
        logic                   valid;
        logic [1:0]             cluster_id;
        logic [ADDR_WIDTH-1:0]  addr;
        logic                   is_write;
        logic                   is_upgrade;
        logic [LINE_SIZE*8-1:0] wdata;
    } req_queue_entry_t;
    
    req_queue_entry_t req_queue [0:SNOOP_QUEUE_DEPTH-1];
    logic [$clog2(SNOOP_QUEUE_DEPTH)-1:0] req_head, req_tail;
    logic req_queue_full, req_queue_empty;
    
    assign req_queue_full = ((req_tail + 1) % SNOOP_QUEUE_DEPTH) == req_head;
    assign req_queue_empty = (req_head == req_tail);
    
    //==========================================================================
    // State Machine
    //==========================================================================
    
    typedef enum logic [3:0] {
        ST_IDLE,
        ST_DIR_LOOKUP,
        ST_SNOOP_SEND,
        ST_SNOOP_WAIT,
        ST_SNOOP_COLLECT,
        ST_L2_REQUEST,
        ST_L2_WAIT,
        ST_WRITEBACK,
        ST_RESPOND,
        ST_DIR_UPDATE,
        ST_FENCE_WAIT
    } state_t;
    
    state_t state;
    
    // Current transaction registers
    logic [1:0]             current_cluster;
    logic [ADDR_WIDTH-1:0]  current_addr;
    logic                   current_is_write;
    logic                   current_is_upgrade;
    logic [LINE_SIZE*8-1:0] current_wdata;
    logic [LINE_SIZE*8-1:0] response_data;
    
    // Directory lookup result
    directory_entry_t       dir_entry;
    logic [DIR_INDEX_BITS-1:0] dir_index;
    logic [TAG_BITS-1:0]    addr_tag;
    
    // Snoop state
    logic [NUM_CLUSTERS-1:0] snoop_pending;
    logic [NUM_CLUSTERS-1:0] snoop_received;
    logic [NUM_CLUSTERS-1:0] snoop_had_dirty;
    logic [LINE_SIZE*8-1:0]  snoop_dirty_data;
    
    // Fence state
    logic [NUM_CLUSTERS-1:0] fence_pending;
    
    //==========================================================================
    // Address Decomposition
    //==========================================================================
    
    function automatic logic [DIR_INDEX_BITS-1:0] get_dir_index(logic [ADDR_WIDTH-1:0] addr);
        return addr[DIR_INDEX_BITS+5:6];  // Skip offset bits (6 for 64B line)
    endfunction
    
    function automatic logic [TAG_BITS-1:0] get_tag(logic [ADDR_WIDTH-1:0] addr);
        return addr[ADDR_WIDTH-1:ADDR_WIDTH-TAG_BITS];
    endfunction
    
    //==========================================================================
    // Request Acceptance
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            req_tail <= '0;
            for (int i = 0; i < NUM_CLUSTERS; i++)
                l1_req_ready[i] <= 1'b1;
        end else begin
            // Accept requests into queue
            for (int i = 0; i < NUM_CLUSTERS; i++) begin
                l1_req_ready[i] <= !req_queue_full;
                
                if (l1_req_valid[i] && l1_req_ready[i] && !req_queue_full) begin
                    req_queue[req_tail].valid <= 1'b1;
                    req_queue[req_tail].cluster_id <= i[1:0];
                    req_queue[req_tail].addr <= l1_req_addr[i];
                    req_queue[req_tail].is_write <= l1_req_write[i];
                    req_queue[req_tail].is_upgrade <= l1_req_upgrade[i];
                    req_queue[req_tail].wdata <= l1_req_wdata[i];
                    req_tail <= (req_tail + 1) % SNOOP_QUEUE_DEPTH;
                end
            end
        end
    end
    
    //==========================================================================
    // Main Coherence State Machine
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            req_head <= '0;
            l2_req_valid <= 1'b0;
            dir_update_valid <= 1'b0;
            coherence_transactions <= '0;
            invalidations_sent <= '0;
            writebacks_received <= '0;
            
            for (int i = 0; i < NUM_CLUSTERS; i++) begin
                l1_resp_valid[i] <= 1'b0;
                snoop_req_valid[i] <= 1'b0;
                fence_done[i] <= 1'b0;
            end
            
            for (int i = 0; i < DIR_SETS; i++)
                directory[i].valid <= 1'b0;
                
            fence_pending <= '0;
        end else begin
            // Clear pulse signals
            dir_update_valid <= 1'b0;
            for (int i = 0; i < NUM_CLUSTERS; i++) begin
                l1_resp_valid[i] <= 1'b0;
                snoop_req_valid[i] <= 1'b0;
                fence_done[i] <= 1'b0;
            end
            
            //------------------------------------------------------------------
            // Fence Handling
            //------------------------------------------------------------------
            fence_pending <= fence_pending | fence_req;
            
            case (state)
                //--------------------------------------------------------------
                ST_IDLE: begin
                    // Check for pending fences first
                    if (|fence_pending) begin
                        // Need to ensure all pending transactions complete
                        if (req_queue_empty) begin
                            fence_done <= fence_pending;
                            fence_pending <= '0;
                        end else begin
                            state <= ST_FENCE_WAIT;
                        end
                    end
                    // Process next request from queue
                    else if (!req_queue_empty) begin
                        current_cluster <= req_queue[req_head].cluster_id;
                        current_addr <= req_queue[req_head].addr;
                        current_is_write <= req_queue[req_head].is_write;
                        current_is_upgrade <= req_queue[req_head].is_upgrade;
                        current_wdata <= req_queue[req_head].wdata;
                        
                        req_queue[req_head].valid <= 1'b0;
                        req_head <= (req_head + 1) % SNOOP_QUEUE_DEPTH;
                        
                        coherence_transactions <= coherence_transactions + 1;
                        state <= ST_DIR_LOOKUP;
                    end
                end
                
                //--------------------------------------------------------------
                ST_DIR_LOOKUP: begin
                    // Look up directory entry
                    dir_index <= get_dir_index(current_addr);
                    addr_tag <= get_tag(current_addr);
                    dir_entry <= directory[get_dir_index(current_addr)];
                    
                    state <= ST_SNOOP_SEND;
                end
                
                //--------------------------------------------------------------
                ST_SNOOP_SEND: begin
                    // Determine which clusters to snoop based on directory
                    snoop_pending <= '0;
                    snoop_received <= '0;
                    snoop_had_dirty <= '0;
                    
                    if (dir_entry.valid && dir_entry.tag == addr_tag) begin
                        // Line is tracked in directory
                        if (current_is_write || current_is_upgrade) begin
                            // Write request: need to invalidate all other sharers
                            logic [NUM_CLUSTERS-1:0] other_sharers;
                            other_sharers = dir_entry.sharers & ~(1 << current_cluster);
                            
                            if (|other_sharers) begin
                                // Send invalidations
                                for (int i = 0; i < NUM_CLUSTERS; i++) begin
                                    if (other_sharers[i]) begin
                                        snoop_req_valid[i] <= 1'b1;
                                        snoop_pending[i] <= 1'b1;
                                    end
                                end
                                snoop_addr <= current_addr;
                                snoop_type <= SNOOP_INVALIDATE;
                                invalidations_sent <= invalidations_sent + $countones(other_sharers);
                                state <= ST_SNOOP_WAIT;
                            end else if (dir_entry.has_owner && dir_entry.owner != current_cluster) begin
                                // Exclusive owner exists - need writeback
                                snoop_req_valid[dir_entry.owner] <= 1'b1;
                                snoop_pending[dir_entry.owner] <= 1'b1;
                                snoop_addr <= current_addr;
                                snoop_type <= SNOOP_INVALIDATE;
                                state <= ST_SNOOP_WAIT;
                            end else begin
                                // No other sharers, go to L2
                                state <= ST_L2_REQUEST;
                            end
                        end else begin
                            // Read request: check if someone has modified copy
                            if (dir_entry.has_owner && dir_entry.owner != current_cluster) begin
                                // Get data from owner
                                snoop_req_valid[dir_entry.owner] <= 1'b1;
                                snoop_pending[dir_entry.owner] <= 1'b1;
                                snoop_addr <= current_addr;
                                snoop_type <= SNOOP_READ_SHARED;
                                state <= ST_SNOOP_WAIT;
                            end else begin
                                // Can get from L2
                                state <= ST_L2_REQUEST;
                            end
                        end
                    end else begin
                        // Line not in directory - go to L2
                        state <= ST_L2_REQUEST;
                    end
                end
                
                //--------------------------------------------------------------
                ST_SNOOP_WAIT: begin
                    // Wait for snoop requests to be accepted
                    logic all_accepted;
                    all_accepted = 1'b1;
                    
                    for (int i = 0; i < NUM_CLUSTERS; i++) begin
                        if (snoop_pending[i] && !snoop_req_ready[i])
                            all_accepted = 1'b0;
                        if (snoop_pending[i] && snoop_req_ready[i])
                            snoop_req_valid[i] <= 1'b0;
                    end
                    
                    if (all_accepted)
                        state <= ST_SNOOP_COLLECT;
                end
                
                //--------------------------------------------------------------
                ST_SNOOP_COLLECT: begin
                    // Collect snoop responses
                    for (int i = 0; i < NUM_CLUSTERS; i++) begin
                        if (snoop_pending[i] && snoop_resp_valid[i]) begin
                            snoop_received[i] <= 1'b1;
                            if (snoop_resp_dirty[i]) begin
                                snoop_had_dirty[i] <= 1'b1;
                                snoop_dirty_data <= snoop_resp_data[i];
                                writebacks_received <= writebacks_received + 1;
                            end
                        end
                    end
                    
                    // Check if all responses received
                    if ((snoop_received | ~snoop_pending) == {NUM_CLUSTERS{1'b1}}) begin
                        if (|snoop_had_dirty) begin
                            // Got dirty data from snoop - may need to writeback to L2
                            if (current_is_write) begin
                                // Merge and use dirty data
                                response_data <= snoop_dirty_data;
                                state <= ST_RESPOND;
                            end else begin
                                // Writeback dirty data to L2
                                state <= ST_WRITEBACK;
                            end
                        end else begin
                            // No dirty data - get from L2
                            state <= ST_L2_REQUEST;
                        end
                    end
                end
                
                //--------------------------------------------------------------
                ST_L2_REQUEST: begin
                    l2_req_valid <= 1'b1;
                    l2_req_addr <= current_addr;
                    l2_req_write <= 1'b0;  // Always read from L2
                    
                    if (l2_req_ready) begin
                        l2_req_valid <= 1'b0;
                        state <= ST_L2_WAIT;
                    end
                end
                
                //--------------------------------------------------------------
                ST_L2_WAIT: begin
                    if (l2_resp_valid) begin
                        response_data <= l2_resp_data;
                        state <= ST_RESPOND;
                    end
                end
                
                //--------------------------------------------------------------
                ST_WRITEBACK: begin
                    // Writeback dirty data to L2
                    l2_req_valid <= 1'b1;
                    l2_req_addr <= current_addr;
                    l2_req_write <= 1'b1;
                    l2_req_wdata <= snoop_dirty_data;
                    
                    if (l2_req_ready) begin
                        l2_req_valid <= 1'b0;
                        response_data <= snoop_dirty_data;
                        state <= ST_RESPOND;
                    end
                end
                
                //--------------------------------------------------------------
                ST_RESPOND: begin
                    // Send response to requesting cluster
                    l1_resp_valid[current_cluster] <= 1'b1;
                    l1_resp_data[current_cluster] <= response_data;
                    
                    if (current_is_write) begin
                        l1_resp_state[current_cluster] <= STATE_MODIFIED;
                    end else begin
                        l1_resp_state[current_cluster] <= STATE_SHARED;
                    end
                    
                    state <= ST_DIR_UPDATE;
                end
                
                //--------------------------------------------------------------
                ST_DIR_UPDATE: begin
                    // Update directory entry
                    dir_update_valid <= 1'b1;
                    dir_update_addr <= current_addr;
                    
                    // Update sharers
                    if (current_is_write) begin
                        // Exclusive access - only requester has copy
                        dir_update_sharers <= (1 << current_cluster);
                        dir_update_owner <= current_cluster;
                        dir_update_has_owner <= 1'b1;
                        
                        directory[dir_index].valid <= 1'b1;
                        directory[dir_index].tag <= addr_tag;
                        directory[dir_index].sharers <= (1 << current_cluster);
                        directory[dir_index].owner <= current_cluster;
                        directory[dir_index].has_owner <= 1'b1;
                        directory[dir_index].state <= STATE_MODIFIED;
                    end else begin
                        // Shared access - add to sharers
                        dir_update_sharers <= dir_entry.sharers | (1 << current_cluster);
                        dir_update_owner <= dir_entry.owner;
                        dir_update_has_owner <= 1'b0;  // Downgrade owner
                        
                        directory[dir_index].valid <= 1'b1;
                        directory[dir_index].tag <= addr_tag;
                        directory[dir_index].sharers <= dir_entry.sharers | (1 << current_cluster);
                        directory[dir_index].has_owner <= 1'b0;
                        directory[dir_index].state <= STATE_SHARED;
                    end
                    
                    state <= ST_IDLE;
                end
                
                //--------------------------------------------------------------
                ST_FENCE_WAIT: begin
                    // Wait for queue to drain before completing fence
                    if (req_queue_empty) begin
                        fence_done <= fence_pending;
                        fence_pending <= '0;
                        state <= ST_IDLE;
                    end
                end
            endcase
        end
    end

endmodule : krypton_cache_coherence
