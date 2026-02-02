//==============================================================================
// GPU Krypton Series-1 - Top Level Module
// Codinome: Krypton-MP4
// Architecture: Tile-Based Deferred Rendering (TBDR)
// Configuration: 4 Shader Clusters, 512 ALUs, ~800 GFLOPS @ 1GHz
//==============================================================================

module krypton_gpu_top
    import krypton_pkg::*;
(
    input  logic                        clk,
    input  logic                        rst_n,
    
    //==========================================================================
    // AXI4 Master Interface (to System Memory)
    //==========================================================================
    
    // Write Address Channel
    output logic                        m_axi_awvalid,
    input  logic                        m_axi_awready,
    output logic [AXI_ID_WIDTH-1:0]     m_axi_awid,
    output logic [AXI_ADDR_WIDTH-1:0]   m_axi_awaddr,
    output logic [7:0]                  m_axi_awlen,
    output logic [2:0]                  m_axi_awsize,
    output logic [1:0]                  m_axi_awburst,
    output logic                        m_axi_awlock,
    output logic [3:0]                  m_axi_awcache,
    output logic [2:0]                  m_axi_awprot,
    
    // Write Data Channel
    output logic                        m_axi_wvalid,
    input  logic                        m_axi_wready,
    output logic [AXI_DATA_WIDTH-1:0]   m_axi_wdata,
    output logic [AXI_DATA_WIDTH/8-1:0] m_axi_wstrb,
    output logic                        m_axi_wlast,
    
    // Write Response Channel
    input  logic                        m_axi_bvalid,
    output logic                        m_axi_bready,
    input  logic [AXI_ID_WIDTH-1:0]     m_axi_bid,
    input  logic [1:0]                  m_axi_bresp,
    
    // Read Address Channel
    output logic                        m_axi_arvalid,
    input  logic                        m_axi_arready,
    output logic [AXI_ID_WIDTH-1:0]     m_axi_arid,
    output logic [AXI_ADDR_WIDTH-1:0]   m_axi_araddr,
    output logic [7:0]                  m_axi_arlen,
    output logic [2:0]                  m_axi_arsize,
    output logic [1:0]                  m_axi_arburst,
    output logic                        m_axi_arlock,
    output logic [3:0]                  m_axi_arcache,
    output logic [2:0]                  m_axi_arprot,
    
    // Read Data Channel
    input  logic                        m_axi_rvalid,
    output logic                        m_axi_rready,
    input  logic [AXI_ID_WIDTH-1:0]     m_axi_rid,
    input  logic [AXI_DATA_WIDTH-1:0]   m_axi_rdata,
    input  logic [1:0]                  m_axi_rresp,
    input  logic                        m_axi_rlast,
    
    //==========================================================================
    // APB Configuration Interface
    //==========================================================================
    
    input  logic                        apb_psel,
    input  logic                        apb_penable,
    input  logic                        apb_pwrite,
    input  logic [11:0]                 apb_paddr,
    input  logic [31:0]                 apb_pwdata,
    output logic [31:0]                 apb_prdata,
    output logic                        apb_pready,
    output logic                        apb_pslverr,
    
    //==========================================================================
    // Interrupt Output
    //==========================================================================
    
    output logic                        irq_fence,
    output logic                        irq_error,
    
    //==========================================================================
    // Debug/Status
    //==========================================================================
    
    output logic                        gpu_busy,
    output logic [3:0]                  gpu_state
);

    //==========================================================================
    // Internal Signals
    //==========================================================================
    
    // Configuration Registers
    logic [ADDR_WIDTH-1:0]      reg_cmd_buffer_base;
    logic [31:0]                reg_cmd_buffer_size;
    logic                       reg_cmd_start;
    logic                       reg_cmd_busy;
    
    logic [ADDR_WIDTH-1:0]      reg_fb_color_base;
    logic [ADDR_WIDTH-1:0]      reg_fb_depth_base;
    logic [31:0]                reg_fb_width;
    logic [31:0]                reg_fb_height;
    
    logic [ADDR_WIDTH-1:0]      reg_vertex_buffer;
    logic [31:0]                reg_vertex_stride;
    logic [ADDR_WIDTH-1:0]      reg_index_buffer;
    
    pipeline_state_t            reg_pipeline_state;
    
    // Command Processor signals
    logic                       cp_geo_cmd_valid;
    logic                       cp_geo_cmd_ready;
    command_packet_t            cp_geo_cmd;
    
    logic                       cp_rast_cmd_valid;
    logic                       cp_rast_cmd_ready;
    command_packet_t            cp_rast_cmd;
    
    logic                       cp_compute_cmd_valid;
    logic                       cp_compute_cmd_ready;
    command_packet_t            cp_compute_cmd;
    
    logic                       cp_fence_interrupt;
    logic                       cp_fence_ack;
    
    // Geometry Pipeline signals
    logic                       geo_tile_valid;
    logic                       geo_tile_ready;
    tile_descriptor_t           geo_tile_desc;
    
    logic                       geo_prim_valid;
    logic                       geo_prim_ready;
    triangle_t                  geo_prim_data;
    
    // Shader Cluster signals
    logic [NUM_CLUSTERS-1:0]    cluster_dispatch_valid;
    logic [NUM_CLUSTERS-1:0]    cluster_dispatch_ready;
    logic [2:0]                 cluster_dispatch_type [0:NUM_CLUSTERS-1];
    logic [ADDR_WIDTH-1:0]      cluster_shader_pc [0:NUM_CLUSTERS-1];
    logic [31:0]                cluster_thread_count [0:NUM_CLUSTERS-1];
    
    logic [NUM_CLUSTERS-1:0]    cluster_complete_valid;
    logic [NUM_CLUSTERS-1:0]    cluster_complete_ready;
    
    // L2 Cache interface
    logic [7:0]                 l2_req_valid;
    logic [7:0]                 l2_req_ready;
    logic [ADDR_WIDTH-1:0]      l2_req_addr [0:7];
    logic [7:0]                 l2_req_write;
    logic [511:0]               l2_req_wdata [0:7];
    logic [7:0]                 l2_resp_valid;
    logic [511:0]               l2_resp_data [0:7];
    
    // Performance Counters
    perf_counters_t             perf_counters;
    
    //==========================================================================
    // Command Processor Instance
    //==========================================================================
    
    // AXI signals for CP
    logic                       cp_axi_arvalid;
    logic                       cp_axi_arready;
    logic [ADDR_WIDTH-1:0]      cp_axi_araddr;
    logic                       cp_axi_rvalid;
    logic                       cp_axi_rready;
    logic [AXI_DATA_WIDTH-1:0]  cp_axi_rdata;
    
    krypton_command_processor u_command_processor (
        .clk                    (clk),
        .rst_n                  (rst_n),
        
        .cmd_buffer_base        (reg_cmd_buffer_base),
        .cmd_buffer_size        (reg_cmd_buffer_size),
        .cmd_start              (reg_cmd_start),
        .cmd_busy               (reg_cmd_busy),
        .cmd_done               (),
        
        .m_axi_arvalid          (cp_axi_arvalid),
        .m_axi_arready          (cp_axi_arready),
        .m_axi_araddr           (cp_axi_araddr),
        .m_axi_arlen            (),
        .m_axi_arsize           (),
        .m_axi_arburst          (),
        
        .m_axi_rvalid           (cp_axi_rvalid),
        .m_axi_rready           (cp_axi_rready),
        .m_axi_rdata            (cp_axi_rdata),
        .m_axi_rresp            (2'b00),
        .m_axi_rlast            (1'b1),
        
        .geo_cmd_valid          (cp_geo_cmd_valid),
        .geo_cmd_ready          (cp_geo_cmd_ready),
        .geo_cmd_packet         (cp_geo_cmd),
        
        .rast_cmd_valid         (cp_rast_cmd_valid),
        .rast_cmd_ready         (cp_rast_cmd_ready),
        .rast_cmd_packet        (cp_rast_cmd),
        
        .compute_cmd_valid      (cp_compute_cmd_valid),
        .compute_cmd_ready      (cp_compute_cmd_ready),
        .compute_cmd_packet     (cp_compute_cmd),
        
        .pipeline_state         (reg_pipeline_state),
        .pipeline_state_valid   (),
        
        .fence_interrupt        (cp_fence_interrupt),
        .fence_ack              (cp_fence_ack),
        
        .cmd_count              ()
    );
    
    //==========================================================================
    // Geometry Pipeline Instance
    //==========================================================================
    
    // Memory interface for geometry
    logic                       geo_mem_req;
    logic [ADDR_WIDTH-1:0]      geo_mem_addr;
    logic                       geo_mem_ready;
    logic                       geo_mem_valid;
    logic [AXI_DATA_WIDTH-1:0]  geo_mem_data;
    
    // Vertex shader dispatch
    logic                       geo_vs_dispatch_valid;
    logic                       geo_vs_dispatch_ready;
    logic [ADDR_WIDTH-1:0]      geo_vs_shader_pc;
    logic [31:0]                geo_vs_vertex_count;
    logic [ADDR_WIDTH-1:0]      geo_vs_input_ptr;
    logic [ADDR_WIDTH-1:0]      geo_vs_output_ptr;
    logic                       geo_vs_complete_valid;
    logic                       geo_vs_complete_ready;
    
    krypton_geometry_pipeline u_geometry_pipeline (
        .clk                    (clk),
        .rst_n                  (rst_n),
        
        .cmd_valid              (cp_geo_cmd_valid),
        .cmd_ready              (cp_geo_cmd_ready),
        .cmd_packet             (cp_geo_cmd),
        
        .vertex_buffer_base     (reg_vertex_buffer),
        .vertex_stride          (reg_vertex_stride),
        .index_buffer_base      (reg_index_buffer),
        .index_type             (2'b10),
        
        .mem_req                (geo_mem_req),
        .mem_addr               (geo_mem_addr),
        .mem_len                (),
        .mem_ready              (geo_mem_ready),
        .mem_valid              (geo_mem_valid),
        .mem_data               (geo_mem_data),
        
        .vs_dispatch_valid      (geo_vs_dispatch_valid),
        .vs_dispatch_ready      (geo_vs_dispatch_ready),
        .vs_shader_pc           (geo_vs_shader_pc),
        .vs_vertex_count        (geo_vs_vertex_count),
        .vs_input_ptr           (geo_vs_input_ptr),
        .vs_output_ptr          (geo_vs_output_ptr),
        
        .vs_complete_valid      (geo_vs_complete_valid),
        .vs_complete_ready      (geo_vs_complete_ready),
        
        .tile_valid             (geo_tile_valid),
        .tile_ready             (geo_tile_ready),
        .tile_desc              (geo_tile_desc),
        
        .prim_valid             (geo_prim_valid),
        .prim_ready             (geo_prim_ready),
        .prim_data              (geo_prim_data),
        
        .tl_write_req           (),
        .tl_write_addr          (),
        .tl_write_data          (),
        .tl_write_ready         (1'b1),
        
        .perf_vertices          (perf_counters.vertices_processed),
        .perf_primitives        (perf_counters.primitives_processed),
        .perf_culled            ()
    );
    
    //==========================================================================
    // Rasterizer Instance
    //==========================================================================
    
    logic                       rast_ps_dispatch_valid;
    logic                       rast_ps_dispatch_ready;
    logic [ADDR_WIDTH-1:0]      rast_ps_shader_pc;
    logic [31:0]                rast_ps_pixel_count;
    logic                       rast_ps_complete_valid;
    logic                       rast_ps_complete_ready;
    
    logic                       rast_fb_write_req;
    logic [ADDR_WIDTH-1:0]      rast_fb_write_addr;
    logic [AXI_DATA_WIDTH-1:0]  rast_fb_write_data;
    
    rgba8_t                     ps_color_out [0:TILE_WIDTH*TILE_HEIGHT-1];
    logic [23:0]                ps_depth_out [0:TILE_WIDTH*TILE_HEIGHT-1];
    
    krypton_rasterizer u_rasterizer (
        .clk                    (clk),
        .rst_n                  (rst_n),
        
        .tile_valid             (geo_tile_valid),
        .tile_ready             (geo_tile_ready),
        .tile_desc              (geo_tile_desc),
        
        .prim_valid             (geo_prim_valid),
        .prim_ready             (geo_prim_ready),
        .prim_data              (geo_prim_data),
        
        .pipeline_state         (reg_pipeline_state),
        
        .ps_dispatch_valid      (rast_ps_dispatch_valid),
        .ps_dispatch_ready      (rast_ps_dispatch_ready),
        .ps_shader_pc           (rast_ps_shader_pc),
        .ps_pixel_count         (rast_ps_pixel_count),
        .ps_tile_x              (),
        .ps_tile_y              (),
        
        .ps_complete_valid      (rast_ps_complete_valid),
        .ps_complete_ready      (rast_ps_complete_ready),
        .ps_color_out           (ps_color_out),
        .ps_depth_out           (ps_depth_out),
        
        .fb_write_req           (rast_fb_write_req),
        .fb_write_addr          (rast_fb_write_addr),
        .fb_write_data          (rast_fb_write_data),
        .fb_write_strb          (),
        .fb_write_ready         (m_axi_wready),
        
        .fb_color_base          (reg_fb_color_base),
        .fb_depth_base          (reg_fb_depth_base),
        .fb_width               (reg_fb_width),
        .fb_height              (reg_fb_height),
        .fb_format              (4'h0),
        
        .perf_pixels_in         (),
        .perf_pixels_out        (perf_counters.pixels_shaded),
        .perf_depth_pass        (),
        .perf_depth_fail        ()
    );
    
    //==========================================================================
    // Shader Clusters (4x)
    //==========================================================================
    
    genvar cluster_idx;
    generate
        for (cluster_idx = 0; cluster_idx < NUM_CLUSTERS; cluster_idx++) begin : shader_clusters
            
            // L1 caches per cluster
            logic                       l1i_req, l1i_ready, l1i_valid;
            logic [ADDR_WIDTH-1:0]      l1i_addr;
            logic [255:0]               l1i_data;
            
            logic                       l1t_req, l1t_ready, l1t_valid;
            logic [ADDR_WIDTH-1:0]      l1t_addr;
            logic [127:0]               l1t_data;
            
            logic                       l1i_l2_req, l1i_l2_valid;
            logic [ADDR_WIDTH-1:0]      l1i_l2_addr;
            logic [511:0]               l1i_l2_data;
            
            logic                       l1t_l2_req, l1t_l2_valid;
            logic [ADDR_WIDTH-1:0]      l1t_l2_addr;
            logic [511:0]               l1t_l2_data;
            
            krypton_l1_icache #(
                .CACHE_SIZE_KB      (L1_ICACHE_SIZE_KB)
            ) u_l1_icache (
                .clk                (clk),
                .rst_n              (rst_n),
                .req_valid          (l1i_req),
                .req_ready          (l1i_ready),
                .req_addr           (l1i_addr),
                .resp_valid         (l1i_valid),
                .resp_data          (l1i_data),
                .l2_req             (l1i_l2_req),
                .l2_addr            (l1i_l2_addr),
                .l2_valid           (l1i_l2_valid),
                .l2_data            (l1i_l2_data),
                .hit_count          (),
                .miss_count         ()
            );
            
            krypton_l1_tcache #(
                .CACHE_SIZE_KB      (L1_TCACHE_SIZE_KB)
            ) u_l1_tcache (
                .clk                (clk),
                .rst_n              (rst_n),
                .req_valid          (l1t_req),
                .req_ready          (l1t_ready),
                .req_addr           (l1t_addr),
                .resp_valid         (l1t_valid),
                .resp_data          (l1t_data),
                .l2_req             (l1t_l2_req),
                .l2_addr            (l1t_l2_addr),
                .l2_valid           (l1t_l2_valid),
                .l2_data            (l1t_l2_data),
                .hit_count          (),
                .miss_count         ()
            );
            
            // Connect L1 to L2 ports
            assign l2_req_valid[cluster_idx*2]     = l1i_l2_req;
            assign l2_req_addr[cluster_idx*2]      = l1i_l2_addr;
            assign l2_req_write[cluster_idx*2]     = 1'b0;
            assign l1i_l2_valid = l2_resp_valid[cluster_idx*2];
            assign l1i_l2_data  = l2_resp_data[cluster_idx*2];
            
            assign l2_req_valid[cluster_idx*2+1]   = l1t_l2_req;
            assign l2_req_addr[cluster_idx*2+1]    = l1t_l2_addr;
            assign l2_req_write[cluster_idx*2+1]   = 1'b0;
            assign l1t_l2_valid = l2_resp_valid[cluster_idx*2+1];
            assign l1t_l2_data  = l2_resp_data[cluster_idx*2+1];
            
            // Shader Cluster
            krypton_shader_cluster #(
                .CLUSTER_ID         (cluster_idx)
            ) u_shader_cluster (
                .clk                (clk),
                .rst_n              (rst_n),
                
                .dispatch_valid     (cluster_dispatch_valid[cluster_idx]),
                .dispatch_ready     (cluster_dispatch_ready[cluster_idx]),
                .dispatch_type      (cluster_dispatch_type[cluster_idx]),
                .dispatch_shader_pc (cluster_shader_pc[cluster_idx]),
                .dispatch_thread_count (cluster_thread_count[cluster_idx]),
                .dispatch_input_ptr ('0),
                .dispatch_output_ptr ('0),
                
                .complete_valid     (cluster_complete_valid[cluster_idx]),
                .complete_ready     (cluster_complete_ready[cluster_idx]),
                .complete_id        (),
                
                .icache_req         (l1i_req),
                .icache_addr        (l1i_addr),
                .icache_valid       (l1i_valid),
                .icache_data        (l1i_data),
                
                .tcache_req         (l1t_req),
                .tcache_addr        (l1t_addr),
                .tcache_valid       (l1t_valid),
                .tcache_data        ({128'b0, l1t_data}),
                
                .mem_req            (),
                .mem_write          (),
                .mem_addr           (),
                .mem_wdata          (),
                .mem_wstrb          (),
                .mem_ready          (1'b1),
                .mem_valid          (1'b0),
                .mem_rdata          ('0),
                
                .tmu_req            (),
                .tmu_sampler        (),
                .tmu_coord          (),
                .tmu_lod            (),
                .tmu_valid          ('{default: 1'b0}),
                .tmu_data           (),
                
                .perf_active_cycles (),
                .perf_stall_cycles  (),
                .perf_instructions  ()
            );
        end
    endgenerate
    
    //==========================================================================
    // Work Distribution to Shader Clusters
    //==========================================================================
    
    // Simple round-robin distribution
    logic [1:0] cluster_rr_ptr;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cluster_rr_ptr <= '0;
            for (int i = 0; i < NUM_CLUSTERS; i++) begin
                cluster_dispatch_valid[i] <= 1'b0;
            end
        end else begin
            // Clear previous dispatch
            for (int i = 0; i < NUM_CLUSTERS; i++) begin
                if (cluster_dispatch_ready[i])
                    cluster_dispatch_valid[i] <= 1'b0;
            end
            
            // Dispatch vertex shader work
            if (geo_vs_dispatch_valid && cluster_dispatch_ready[cluster_rr_ptr]) begin
                cluster_dispatch_valid[cluster_rr_ptr] <= 1'b1;
                cluster_dispatch_type[cluster_rr_ptr] <= 3'd0;  // Vertex
                cluster_shader_pc[cluster_rr_ptr] <= geo_vs_shader_pc;
                cluster_thread_count[cluster_rr_ptr] <= geo_vs_vertex_count;
                cluster_rr_ptr <= cluster_rr_ptr + 1;
            end
            // Dispatch pixel shader work
            else if (rast_ps_dispatch_valid && cluster_dispatch_ready[cluster_rr_ptr]) begin
                cluster_dispatch_valid[cluster_rr_ptr] <= 1'b1;
                cluster_dispatch_type[cluster_rr_ptr] <= 3'd1;  // Pixel
                cluster_shader_pc[cluster_rr_ptr] <= rast_ps_shader_pc;
                cluster_thread_count[cluster_rr_ptr] <= rast_ps_pixel_count;
                cluster_rr_ptr <= cluster_rr_ptr + 1;
            end
        end
    end
    
    assign geo_vs_dispatch_ready = cluster_dispatch_ready[cluster_rr_ptr];
    assign rast_ps_dispatch_ready = cluster_dispatch_ready[cluster_rr_ptr] && !geo_vs_dispatch_valid;
    
    // Completion signals
    assign geo_vs_complete_valid = |cluster_complete_valid;
    assign rast_ps_complete_valid = |cluster_complete_valid;
    
    always_comb begin
        for (int i = 0; i < NUM_CLUSTERS; i++) begin
            cluster_complete_ready[i] = geo_vs_complete_ready | rast_ps_complete_ready;
        end
    end
    
    //==========================================================================
    // L2 Cache Instance
    //==========================================================================
    
    krypton_l2_cache #(
        .CACHE_SIZE_KB          (L2_CACHE_SIZE_KB),
        .NUM_PORTS              (8)
    ) u_l2_cache (
        .clk                    (clk),
        .rst_n                  (rst_n),
        
        .req_valid              (l2_req_valid),
        .req_ready              (l2_req_ready),
        .req_addr               (l2_req_addr),
        .req_write              (l2_req_write),
        .req_wdata              (l2_req_wdata),
        
        .resp_valid             (l2_resp_valid),
        .resp_data              (l2_resp_data),
        
        .m_axi_arvalid          (m_axi_arvalid),
        .m_axi_arready          (m_axi_arready),
        .m_axi_araddr           (m_axi_araddr),
        .m_axi_arlen            (m_axi_arlen),
        
        .m_axi_rvalid           (m_axi_rvalid),
        .m_axi_rready           (m_axi_rready),
        .m_axi_rdata            (m_axi_rdata),
        .m_axi_rlast            (m_axi_rlast),
        
        .m_axi_awvalid          (m_axi_awvalid),
        .m_axi_awready          (m_axi_awready),
        .m_axi_awaddr           (m_axi_awaddr),
        .m_axi_awlen            (m_axi_awlen),
        
        .m_axi_wvalid           (m_axi_wvalid),
        .m_axi_wready           (m_axi_wready),
        .m_axi_wdata            (m_axi_wdata),
        .m_axi_wlast            (m_axi_wlast),
        
        .m_axi_bvalid           (m_axi_bvalid),
        .m_axi_bready           (m_axi_bready),
        
        .total_hits             (perf_counters.cache_hits_l2),
        .total_misses           (perf_counters.cache_misses_l2)
    );
    
    // Fixed AXI signals
    assign m_axi_awid    = '0;
    assign m_axi_awsize  = 3'b101;  // 32 bytes
    assign m_axi_awburst = 2'b01;   // INCR
    assign m_axi_awlock  = 1'b0;
    assign m_axi_awcache = 4'b0011;
    assign m_axi_awprot  = 3'b000;
    assign m_axi_wstrb   = '1;
    
    assign m_axi_arid    = '0;
    assign m_axi_arsize  = 3'b101;
    assign m_axi_arburst = 2'b01;
    assign m_axi_arlock  = 1'b0;
    assign m_axi_arcache = 4'b0011;
    assign m_axi_arprot  = 3'b000;
    
    //==========================================================================
    // APB Register Interface
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_cmd_buffer_base <= '0;
            reg_cmd_buffer_size <= '0;
            reg_cmd_start <= 1'b0;
            reg_fb_color_base <= '0;
            reg_fb_depth_base <= '0;
            reg_fb_width <= 32'd1920;
            reg_fb_height <= 32'd1080;
            reg_vertex_buffer <= '0;
            reg_vertex_stride <= 32'd32;
            reg_index_buffer <= '0;
            reg_pipeline_state <= '0;
            apb_prdata <= '0;
            apb_pready <= 1'b0;
            apb_pslverr <= 1'b0;
            cp_fence_ack <= 1'b0;
        end else begin
            reg_cmd_start <= 1'b0;
            cp_fence_ack <= 1'b0;
            apb_pready <= apb_psel && apb_penable;
            
            if (apb_psel && apb_penable) begin
                if (apb_pwrite) begin
                    case (apb_paddr[11:2])
                        10'h000: reg_cmd_buffer_base[31:0] <= apb_pwdata;
                        10'h001: reg_cmd_buffer_base[39:32] <= apb_pwdata[7:0];
                        10'h002: reg_cmd_buffer_size <= apb_pwdata;
                        10'h003: reg_cmd_start <= apb_pwdata[0];
                        10'h004: reg_fb_color_base[31:0] <= apb_pwdata;
                        10'h005: reg_fb_color_base[39:32] <= apb_pwdata[7:0];
                        10'h006: reg_fb_depth_base[31:0] <= apb_pwdata;
                        10'h007: reg_fb_depth_base[39:32] <= apb_pwdata[7:0];
                        10'h008: reg_fb_width <= apb_pwdata;
                        10'h009: reg_fb_height <= apb_pwdata;
                        10'h00A: reg_vertex_buffer[31:0] <= apb_pwdata;
                        10'h00B: reg_vertex_buffer[39:32] <= apb_pwdata[7:0];
                        10'h00C: reg_vertex_stride <= apb_pwdata;
                        10'h00D: reg_index_buffer[31:0] <= apb_pwdata;
                        10'h00E: reg_index_buffer[39:32] <= apb_pwdata[7:0];
                        10'h010: cp_fence_ack <= apb_pwdata[0];
                        default: ;
                    endcase
                end else begin
                    case (apb_paddr[11:2])
                        10'h000: apb_prdata <= reg_cmd_buffer_base[31:0];
                        10'h001: apb_prdata <= {24'b0, reg_cmd_buffer_base[39:32]};
                        10'h002: apb_prdata <= reg_cmd_buffer_size;
                        10'h003: apb_prdata <= {30'b0, reg_cmd_busy, 1'b0};
                        10'h020: apb_prdata <= perf_counters.cycles[31:0];
                        10'h021: apb_prdata <= perf_counters.cycles[63:32];
                        10'h022: apb_prdata <= perf_counters.vertices_processed[31:0];
                        10'h023: apb_prdata <= perf_counters.primitives_processed[31:0];
                        10'h024: apb_prdata <= perf_counters.pixels_shaded[31:0];
                        default: apb_prdata <= '0;
                    endcase
                end
            end
        end
    end
    
    //==========================================================================
    // Performance Counter - Cycle Counter
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            perf_counters.cycles <= '0;
        end else begin
            perf_counters.cycles <= perf_counters.cycles + 1;
        end
    end
    
    //==========================================================================
    // Status and Interrupts
    //==========================================================================
    
    assign gpu_busy = reg_cmd_busy;
    assign gpu_state = state_t'(4'h0);  // Could expose internal state
    
    assign irq_fence = cp_fence_interrupt;
    assign irq_error = 1'b0;

endmodule : krypton_gpu_top
