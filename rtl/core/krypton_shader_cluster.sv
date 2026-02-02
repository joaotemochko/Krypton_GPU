//==============================================================================
// GPU Krypton Series-1 - Shader Cluster
// Contains 4 Krypton Cores with Warp Scheduler and Shared Resources
//==============================================================================

module krypton_shader_cluster
    import krypton_pkg::*;
#(
    parameter int CLUSTER_ID = 0
)(
    input  logic                        clk,
    input  logic                        rst_n,
    
    // Work dispatch interface
    input  logic                        dispatch_valid,
    output logic                        dispatch_ready,
    input  logic [2:0]                  dispatch_type,     // 0=vertex, 1=pixel, 2=compute
    input  logic [ADDR_WIDTH-1:0]       dispatch_shader_pc,
    input  logic [31:0]                 dispatch_thread_count,
    input  logic [ADDR_WIDTH-1:0]       dispatch_input_ptr,
    input  logic [ADDR_WIDTH-1:0]       dispatch_output_ptr,
    
    // Work completion
    output logic                        complete_valid,
    input  logic                        complete_ready,
    output logic [15:0]                 complete_id,
    
    // L1 Instruction Cache interface
    output logic                        icache_req,
    output logic [ADDR_WIDTH-1:0]       icache_addr,
    input  logic                        icache_valid,
    input  logic [255:0]                icache_data,
    
    // L1 Texture Cache interface
    output logic                        tcache_req,
    output logic [ADDR_WIDTH-1:0]       tcache_addr,
    input  logic                        tcache_valid,
    input  logic [255:0]                tcache_data,
    
    // Memory interface (to L2)
    output logic                        mem_req,
    output logic                        mem_write,
    output logic [ADDR_WIDTH-1:0]       mem_addr,
    output logic [AXI_DATA_WIDTH-1:0]   mem_wdata,
    output logic [31:0]                 mem_wstrb,
    input  logic                        mem_ready,
    input  logic                        mem_valid,
    input  logic [AXI_DATA_WIDTH-1:0]   mem_rdata,
    
    // Texture unit interface
    output logic                        tmu_req [0:TMUS_PER_CLUSTER-1],
    output logic [3:0]                  tmu_sampler [0:TMUS_PER_CLUSTER-1],
    output vec2_t                       tmu_coord [0:TMUS_PER_CLUSTER-1],
    output logic [3:0]                  tmu_lod [0:TMUS_PER_CLUSTER-1],
    input  logic                        tmu_valid [0:TMUS_PER_CLUSTER-1],
    input  rgba8_t                      tmu_data [0:TMUS_PER_CLUSTER-1],
    
    // Performance counters
    output logic [63:0]                 perf_active_cycles,
    output logic [63:0]                 perf_stall_cycles,
    output logic [63:0]                 perf_instructions
);

    //==========================================================================
    // Local Parameters
    //==========================================================================
    
    localparam int NUM_CORES = CORES_PER_CLUSTER;
    localparam int MAX_WARPS = 32;
    localparam int WARP_SIZE = LANES_PER_CORE;
    
    //==========================================================================
    // Warp Pool
    //==========================================================================
    
    typedef struct packed {
        logic                   valid;
        logic                   active;
        logic [2:0]             type_id;      // shader type
        logic [ADDR_WIDTH-1:0]  pc;
        logic [ADDR_WIDTH-1:0]  input_ptr;
        logic [ADDR_WIDTH-1:0]  output_ptr;
        logic [31:0]            thread_mask;
        logic [15:0]            work_id;
    } warp_entry_t;
    
    warp_entry_t warp_pool [0:MAX_WARPS-1];
    
    logic [4:0]         free_warp_id;
    logic               free_warp_valid;
    logic [MAX_WARPS-1:0] warp_valid_mask;
    logic [MAX_WARPS-1:0] warp_ready_mask;
    
    //==========================================================================
    // Warp Scheduler
    //==========================================================================
    
    logic [4:0]         sched_warp_id [0:NUM_CORES-1];
    logic               sched_valid [0:NUM_CORES-1];
    logic [NUM_CORES-1:0] core_busy;
    logic [NUM_CORES-1:0] core_warp_done;
    logic [4:0]         core_done_warp_id [0:NUM_CORES-1];
    
    // Round-robin scheduler state
    logic [4:0]         sched_ptr;
    
    //==========================================================================
    // Vector Register File (128KB per cluster = 32K x 32-bit)
    //==========================================================================
    
    // Per-warp register allocation: 1024 VGPRs per warp (32 warps * 1024 = 32K)
    logic [DATA_WIDTH*WARP_SIZE-1:0] vgpr_mem [0:32*1024-1];
    
    // Read/Write ports for cores
    logic               vrf_rd_en [0:NUM_CORES-1];
    logic [14:0]        vrf_rd_addr [0:NUM_CORES-1];
    logic [DATA_WIDTH*WARP_SIZE-1:0] vrf_rd_data [0:NUM_CORES-1];
    
    logic               vrf_wr_en [0:NUM_CORES-1];
    logic [14:0]        vrf_wr_addr [0:NUM_CORES-1];
    logic [DATA_WIDTH*WARP_SIZE-1:0] vrf_wr_data [0:NUM_CORES-1];
    logic [WARP_SIZE-1:0] vrf_wr_mask [0:NUM_CORES-1];
    
    //==========================================================================
    // Scalar Register File (4KB per cluster)
    //==========================================================================
    
    logic [DATA_WIDTH-1:0] sgpr_mem [0:1023];
    
    logic               srf_rd_en [0:NUM_CORES-1];
    logic [9:0]         srf_rd_addr [0:NUM_CORES-1];
    logic [DATA_WIDTH-1:0] srf_rd_data [0:NUM_CORES-1];
    
    logic               srf_wr_en [0:NUM_CORES-1];
    logic [9:0]         srf_wr_addr [0:NUM_CORES-1];
    logic [DATA_WIDTH-1:0] srf_wr_data [0:NUM_CORES-1];
    
    //==========================================================================
    // Work Dispatch Logic
    //==========================================================================
    
    logic [15:0]        work_id_counter;
    logic [31:0]        remaining_threads;
    logic [4:0]         warps_launched;
    
    typedef enum logic [2:0] {
        DISP_IDLE,
        DISP_ALLOCATE,
        DISP_LAUNCH,
        DISP_WAIT
    } dispatch_state_t;
    
    dispatch_state_t disp_state;
    
    // Find free warp slot
    always_comb begin
        free_warp_id = '0;
        free_warp_valid = 1'b0;
        for (int i = 0; i < MAX_WARPS; i++) begin
            if (!warp_pool[i].valid) begin
                free_warp_id = i[4:0];
                free_warp_valid = 1'b1;
                break;
            end
        end
    end
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            disp_state <= DISP_IDLE;
            work_id_counter <= '0;
            remaining_threads <= '0;
            warps_launched <= '0;
            for (int i = 0; i < MAX_WARPS; i++) begin
                warp_pool[i] <= '0;
            end
        end else begin
            case (disp_state)
                DISP_IDLE: begin
                    if (dispatch_valid && dispatch_ready) begin
                        remaining_threads <= dispatch_thread_count;
                        warps_launched <= '0;
                        disp_state <= DISP_ALLOCATE;
                    end
                end
                
                DISP_ALLOCATE: begin
                    if (free_warp_valid && remaining_threads > 0) begin
                        warp_pool[free_warp_id].valid <= 1'b1;
                        warp_pool[free_warp_id].active <= 1'b0;
                        warp_pool[free_warp_id].type_id <= dispatch_type;
                        warp_pool[free_warp_id].pc <= dispatch_shader_pc;
                        warp_pool[free_warp_id].input_ptr <= dispatch_input_ptr + 
                            (warps_launched * WARP_SIZE * 16);
                        warp_pool[free_warp_id].output_ptr <= dispatch_output_ptr + 
                            (warps_launched * WARP_SIZE * 16);
                        warp_pool[free_warp_id].work_id <= work_id_counter;
                        
                        // Set thread mask
                        if (remaining_threads >= WARP_SIZE)
                            warp_pool[free_warp_id].thread_mask <= '1;
                        else
                            warp_pool[free_warp_id].thread_mask <= (1 << remaining_threads) - 1;
                        
                        remaining_threads <= (remaining_threads >= WARP_SIZE) ? 
                            remaining_threads - WARP_SIZE : '0;
                        warps_launched <= warps_launched + 1;
                        
                        disp_state <= DISP_LAUNCH;
                    end else if (remaining_threads == 0) begin
                        work_id_counter <= work_id_counter + 1;
                        disp_state <= DISP_WAIT;
                    end
                end
                
                DISP_LAUNCH: begin
                    disp_state <= (remaining_threads > 0) ? DISP_ALLOCATE : DISP_WAIT;
                end
                
                DISP_WAIT: begin
                    // Wait for all warps of this work item to complete
                    logic all_done;
                    all_done = 1'b1;
                    for (int i = 0; i < MAX_WARPS; i++) begin
                        if (warp_pool[i].valid && warp_pool[i].work_id == work_id_counter - 1)
                            all_done = 1'b0;
                    end
                    if (all_done)
                        disp_state <= DISP_IDLE;
                end
            endcase
            
            // Handle warp completions from cores
            for (int c = 0; c < NUM_CORES; c++) begin
                if (core_warp_done[c]) begin
                    warp_pool[core_done_warp_id[c]].valid <= 1'b0;
                end
            end
        end
    end
    
    assign dispatch_ready = (disp_state == DISP_IDLE);
    assign complete_valid = (disp_state == DISP_WAIT);
    assign complete_id = work_id_counter - 1;
    
    //==========================================================================
    // Warp Scheduler - Round Robin with Priority
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sched_ptr <= '0;
            for (int c = 0; c < NUM_CORES; c++) begin
                sched_valid[c] <= 1'b0;
                sched_warp_id[c] <= '0;
            end
        end else begin
            // Schedule warps to available cores
            for (int c = 0; c < NUM_CORES; c++) begin
                sched_valid[c] <= 1'b0;
                
                if (!core_busy[c]) begin
                    // Find next ready warp
                    for (int w = 0; w < MAX_WARPS; w++) begin
                        int warp_idx;
                        warp_idx = (sched_ptr + w) % MAX_WARPS;
                        
                        if (warp_pool[warp_idx].valid && !warp_pool[warp_idx].active) begin
                            sched_valid[c] <= 1'b1;
                            sched_warp_id[c] <= warp_idx[4:0];
                            warp_pool[warp_idx].active <= 1'b1;
                            sched_ptr <= (warp_idx + 1) % MAX_WARPS;
                            break;
                        end
                    end
                end
            end
        end
    end
    
    //==========================================================================
    // Instantiate Krypton Cores
    //==========================================================================
    
    // Instruction cache arbiter signals
    logic [NUM_CORES-1:0]       core_icache_req;
    logic [ADDR_WIDTH-1:0]      core_icache_addr [0:NUM_CORES-1];
    logic [NUM_CORES-1:0]       core_icache_valid;
    logic [31:0]                core_icache_data [0:NUM_CORES-1];
    
    // LSU signals
    logic [NUM_CORES-1:0]       core_lsu_req;
    logic [NUM_CORES-1:0]       core_lsu_write;
    logic [ADDR_WIDTH-1:0]      core_lsu_addr [0:NUM_CORES-1][0:31];
    logic [DATA_WIDTH-1:0]      core_lsu_wdata [0:NUM_CORES-1][0:31];
    logic [31:0]                core_lsu_mask [0:NUM_CORES-1];
    logic [NUM_CORES-1:0]       core_lsu_ready;
    logic [NUM_CORES-1:0]       core_lsu_valid;
    logic [DATA_WIDTH-1:0]      core_lsu_rdata [0:NUM_CORES-1][0:31];
    
    // Texture unit signals
    logic [NUM_CORES-1:0]       core_tex_req;
    logic [3:0]                 core_tex_unit [0:NUM_CORES-1];
    vec2_t                      core_tex_coord [0:NUM_CORES-1][0:31];
    logic [31:0]                core_tex_mask [0:NUM_CORES-1];
    logic [NUM_CORES-1:0]       core_tex_ready;
    logic [NUM_CORES-1:0]       core_tex_valid;
    rgba8_t                     core_tex_data [0:NUM_CORES-1][0:31];
    
    genvar core_idx;
    generate
        for (core_idx = 0; core_idx < NUM_CORES; core_idx++) begin : cores
            krypton_core #(
                .CORE_ID(core_idx)
            ) u_core (
                .clk            (clk),
                .rst_n          (rst_n),
                
                // Warp interface
                .warp_valid     (sched_valid[core_idx]),
                .warp_ready     (~core_busy[core_idx]),
                .warp_id        (sched_warp_id[core_idx]),
                .warp_pc        (warp_pool[sched_warp_id[core_idx]].pc),
                .warp_mask      (warp_pool[sched_warp_id[core_idx]].thread_mask),
                
                .warp_done      (core_warp_done[core_idx]),
                .warp_done_id   (core_done_warp_id[core_idx]),
                
                // Instruction cache
                .icache_req     (core_icache_req[core_idx]),
                .icache_addr    (core_icache_addr[core_idx]),
                .icache_valid   (core_icache_valid[core_idx]),
                .icache_data    (core_icache_data[core_idx]),
                
                // Vector register file
                .vrf_rd_en      (vrf_rd_en[core_idx]),
                .vrf_rd_addr    (vrf_rd_addr[core_idx][9:0]),
                .vrf_rd_data    (vrf_rd_data[core_idx]),
                .vrf_wr_en      (vrf_wr_en[core_idx]),
                .vrf_wr_addr    (vrf_wr_addr[core_idx][9:0]),
                .vrf_wr_data    (vrf_wr_data[core_idx]),
                .vrf_wr_mask    (vrf_wr_mask[core_idx]),
                
                // Scalar register file
                .srf_rd_en      (srf_rd_en[core_idx]),
                .srf_rd_addr    (srf_rd_addr[core_idx][7:0]),
                .srf_rd_data    (srf_rd_data[core_idx]),
                .srf_wr_en      (srf_wr_en[core_idx]),
                .srf_wr_addr    (srf_wr_addr[core_idx][7:0]),
                .srf_wr_data    (srf_wr_data[core_idx]),
                
                // LSU
                .lsu_req        (core_lsu_req[core_idx]),
                .lsu_write      (core_lsu_write[core_idx]),
                .lsu_addr       (core_lsu_addr[core_idx]),
                .lsu_wdata      (core_lsu_wdata[core_idx]),
                .lsu_mask       (core_lsu_mask[core_idx]),
                .lsu_ready      (core_lsu_ready[core_idx]),
                .lsu_valid      (core_lsu_valid[core_idx]),
                .lsu_rdata      (core_lsu_rdata[core_idx]),
                
                // Texture
                .tex_req        (core_tex_req[core_idx]),
                .tex_unit       (core_tex_unit[core_idx]),
                .tex_coord      (core_tex_coord[core_idx]),
                .tex_mask       (core_tex_mask[core_idx]),
                .tex_ready      (core_tex_ready[core_idx]),
                .tex_valid      (core_tex_valid[core_idx]),
                .tex_data       (core_tex_data[core_idx]),
                
                // SFU (placeholder)
                .sfu_req        (),
                .sfu_op         (),
                .sfu_operand    (),
                .sfu_mask       (),
                .sfu_ready      (1'b1),
                .sfu_valid      (1'b0),
                .sfu_result     ()
            );
            
            // Core busy tracking
            assign core_busy[core_idx] = ~(sched_valid[core_idx] && ~core_warp_done[core_idx]);
        end
    endgenerate
    
    //==========================================================================
    // Vector Register File Access
    //==========================================================================
    
    // Simple multi-port register file with arbitration
    always_ff @(posedge clk) begin
        for (int c = 0; c < NUM_CORES; c++) begin
            if (vrf_rd_en[c]) begin
                vrf_rd_data[c] <= vgpr_mem[vrf_rd_addr[c]];
            end
            if (vrf_wr_en[c]) begin
                for (int lane = 0; lane < WARP_SIZE; lane++) begin
                    if (vrf_wr_mask[c][lane]) begin
                        vgpr_mem[vrf_wr_addr[c]][lane*DATA_WIDTH +: DATA_WIDTH] <= 
                            vrf_wr_data[c][lane*DATA_WIDTH +: DATA_WIDTH];
                    end
                end
            end
        end
    end
    
    //==========================================================================
    // Scalar Register File Access
    //==========================================================================
    
    always_ff @(posedge clk) begin
        for (int c = 0; c < NUM_CORES; c++) begin
            if (srf_rd_en[c]) begin
                srf_rd_data[c] <= sgpr_mem[srf_rd_addr[c]];
            end
            if (srf_wr_en[c]) begin
                sgpr_mem[srf_wr_addr[c]] <= srf_wr_data[c];
            end
        end
    end
    
    //==========================================================================
    // Instruction Cache Arbiter
    //==========================================================================
    
    logic [1:0] icache_grant;
    logic       icache_grant_valid;
    
    always_comb begin
        icache_grant = '0;
        icache_grant_valid = 1'b0;
        icache_req = 1'b0;
        icache_addr = '0;
        
        for (int c = 0; c < NUM_CORES; c++) begin
            core_icache_valid[c] = 1'b0;
            core_icache_data[c] = '0;
        end
        
        // Simple priority arbiter
        for (int c = 0; c < NUM_CORES; c++) begin
            if (core_icache_req[c] && !icache_grant_valid) begin
                icache_grant = c[1:0];
                icache_grant_valid = 1'b1;
                icache_req = 1'b1;
                icache_addr = core_icache_addr[c];
            end
        end
        
        // Route response
        if (icache_valid) begin
            core_icache_valid[icache_grant] = 1'b1;
            core_icache_data[icache_grant] = icache_data[31:0];
        end
    end
    
    //==========================================================================
    // Memory Request Arbiter (LSU to L2)
    //==========================================================================
    
    // Simplified - real implementation would have proper coalescing
    logic [1:0] lsu_grant;
    
    always_comb begin
        mem_req = 1'b0;
        mem_write = 1'b0;
        mem_addr = '0;
        mem_wdata = '0;
        mem_wstrb = '0;
        
        for (int c = 0; c < NUM_CORES; c++) begin
            core_lsu_ready[c] = 1'b0;
            core_lsu_valid[c] = 1'b0;
        end
        
        // Simple priority arbiter
        for (int c = 0; c < NUM_CORES; c++) begin
            if (core_lsu_req[c] && !mem_req) begin
                lsu_grant = c[1:0];
                mem_req = 1'b1;
                mem_write = core_lsu_write[c];
                mem_addr = core_lsu_addr[c][0];  // First lane address
                // Coalesce write data from all lanes
                for (int lane = 0; lane < 8; lane++) begin
                    mem_wdata[lane*32 +: 32] = core_lsu_wdata[c][lane];
                    if (core_lsu_mask[c][lane])
                        mem_wstrb[lane*4 +: 4] = 4'hF;
                end
                core_lsu_ready[c] = mem_ready;
            end
        end
        
        // Route response
        if (mem_valid) begin
            core_lsu_valid[lsu_grant] = 1'b1;
            for (int lane = 0; lane < 8; lane++) begin
                core_lsu_rdata[lsu_grant][lane] = mem_rdata[lane*32 +: 32];
            end
        end
    end
    
    //==========================================================================
    // Performance Counters
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            perf_active_cycles <= '0;
            perf_stall_cycles <= '0;
            perf_instructions <= '0;
        end else begin
            // Active if any core is busy
            if (|core_busy)
                perf_active_cycles <= perf_active_cycles + 1;
            
            // Count instructions retired
            for (int c = 0; c < NUM_CORES; c++) begin
                if (core_warp_done[c])
                    perf_instructions <= perf_instructions + 1;
            end
        end
    end

endmodule : krypton_shader_cluster
