//==============================================================================
// GPU Krypton Series-1 - Global Package Definitions
// Architecture: Tile-Based Deferred Rendering (TBDR)
// ISA: RISC-V Vector Extensions (RVV) Customizada
// Target: TSMC 7nm FinFET
//==============================================================================

package krypton_pkg;

    //==========================================================================
    // Configuration Parameters
    //==========================================================================
    
    // Cluster Configuration (MP4 = 4 Shader Clusters)
    parameter int NUM_CLUSTERS          = 4;
    parameter int CORES_PER_CLUSTER     = 4;
    parameter int LANES_PER_CORE        = 32;  // SIMD/Warp width
    parameter int TOTAL_ALUS            = NUM_CLUSTERS * CORES_PER_CLUSTER * LANES_PER_CORE; // 512
    
    // Clock Configuration
    parameter int BASE_FREQ_MHZ         = 800;
    parameter int BOOST_FREQ_MHZ        = 1000;
    
    // Memory Configuration
    parameter int L1_ICACHE_SIZE_KB     = 32;
    parameter int L1_TCACHE_SIZE_KB     = 32;
    parameter int TILE_BUFFER_SIZE_KB   = 128;
    parameter int L2_CACHE_SIZE_KB      = 512;
    
    // Tile Configuration
    parameter int TILE_WIDTH            = 32;
    parameter int TILE_HEIGHT           = 32;
    parameter int MAX_TILES_X           = 64;  // Max 2048 pixels width
    parameter int MAX_TILES_Y           = 64;  // Max 2048 pixels height
    
    // Data Widths
    parameter int DATA_WIDTH            = 32;
    parameter int ADDR_WIDTH            = 40;
    parameter int AXI_DATA_WIDTH        = 256;
    parameter int AXI_ADDR_WIDTH        = 40;
    parameter int AXI_ID_WIDTH          = 8;
    
    // Register File Configuration
    parameter int VGPR_PER_CLUSTER      = 128 * 1024 / 4; // 128KB / 4 bytes = 32K registers
    parameter int SGPR_PER_CLUSTER      = 4096;
    
    // TMU Configuration
    parameter int TMUS_PER_CLUSTER      = 4;
    parameter int MAX_ANISO_LEVEL       = 4;
    
    //==========================================================================
    // Type Definitions
    //==========================================================================
    
    // Fixed-point types for graphics
    typedef logic [31:0] fp32_t;
    typedef logic [15:0] fp16_t;
    typedef logic [15:0] fixed_16_16_t;
    
    // Vector types
    typedef struct packed {
        fp32_t x;
        fp32_t y;
        fp32_t z;
        fp32_t w;
    } vec4_t;
    
    typedef struct packed {
        fp32_t x;
        fp32_t y;
        fp32_t z;
    } vec3_t;
    
    typedef struct packed {
        fp32_t x;
        fp32_t y;
    } vec2_t;
    
    // Color types
    typedef struct packed {
        logic [7:0] r;
        logic [7:0] g;
        logic [7:0] b;
        logic [7:0] a;
    } rgba8_t;
    
    typedef struct packed {
        fp16_t r;
        fp16_t g;
        fp16_t b;
        fp16_t a;
    } rgba16f_t;
    
    // Vertex data structure
    typedef struct packed {
        vec4_t position;
        vec4_t color;
        vec2_t texcoord;
        vec3_t normal;
    } vertex_t;
    
    // Triangle structure for rasterization
    typedef struct packed {
        vec4_t v0;
        vec4_t v1;
        vec4_t v2;
        logic [15:0] primitive_id;
    } triangle_t;
    
    // Tile descriptor
    typedef struct packed {
        logic [15:0] tile_x;
        logic [15:0] tile_y;
        logic [15:0] primitive_count;
        logic [39:0] list_ptr;
    } tile_descriptor_t;
    
    //==========================================================================
    // Command Structures
    //==========================================================================
    
    typedef enum logic [7:0] {
        CMD_NOP             = 8'h00,
        CMD_DRAW_INDEXED    = 8'h01,
        CMD_DRAW            = 8'h02,
        CMD_DISPATCH        = 8'h03,
        CMD_BIND_PIPELINE   = 8'h04,
        CMD_BIND_VERTEX     = 8'h05,
        CMD_BIND_INDEX      = 8'h06,
        CMD_BIND_UNIFORM    = 8'h07,
        CMD_BIND_TEXTURE    = 8'h08,
        CMD_SET_VIEWPORT    = 8'h09,
        CMD_SET_SCISSOR     = 8'h0A,
        CMD_BARRIER         = 8'h0B,
        CMD_COPY_BUFFER     = 8'h0C,
        CMD_COPY_IMAGE      = 8'h0D,
        CMD_BEGIN_RENDER    = 8'h10,
        CMD_END_RENDER      = 8'h11,
        CMD_FENCE           = 8'hFF
    } cmd_opcode_t;
    
    typedef struct packed {
        cmd_opcode_t opcode;
        logic [23:0] flags;
        logic [31:0] param0;
        logic [31:0] param1;
        logic [31:0] param2;
        logic [31:0] param3;
    } command_packet_t;
    
    //==========================================================================
    // Pipeline State
    //==========================================================================
    
    typedef enum logic [2:0] {
        BLEND_ZERO          = 3'b000,
        BLEND_ONE           = 3'b001,
        BLEND_SRC_ALPHA     = 3'b010,
        BLEND_INV_SRC_ALPHA = 3'b011,
        BLEND_DST_ALPHA     = 3'b100,
        BLEND_INV_DST_ALPHA = 3'b101
    } blend_factor_t;
    
    typedef enum logic [2:0] {
        COMPARE_NEVER       = 3'b000,
        COMPARE_LESS        = 3'b001,
        COMPARE_EQUAL       = 3'b010,
        COMPARE_LEQUAL      = 3'b011,
        COMPARE_GREATER     = 3'b100,
        COMPARE_NOTEQUAL    = 3'b101,
        COMPARE_GEQUAL      = 3'b110,
        COMPARE_ALWAYS      = 3'b111
    } compare_func_t;
    
    typedef enum logic [1:0] {
        CULL_NONE           = 2'b00,
        CULL_FRONT          = 2'b01,
        CULL_BACK           = 2'b10
    } cull_mode_t;
    
    typedef struct packed {
        // Depth state
        logic           depth_test_enable;
        logic           depth_write_enable;
        compare_func_t  depth_func;
        
        // Stencil state
        logic           stencil_enable;
        compare_func_t  stencil_func;
        logic [7:0]     stencil_ref;
        logic [7:0]     stencil_mask;
        
        // Blend state
        logic           blend_enable;
        blend_factor_t  src_blend;
        blend_factor_t  dst_blend;
        
        // Rasterizer state
        cull_mode_t     cull_mode;
        logic           front_ccw;
    } pipeline_state_t;
    
    //==========================================================================
    // Shader Instruction Format (RISC-V Vector Extension Based)
    //==========================================================================
    
    typedef enum logic [6:0] {
        OP_VLOAD        = 7'b0000111,  // Vector Load
        OP_VSTORE       = 7'b0100111,  // Vector Store
        OP_VARITH       = 7'b1010111,  // Vector Arithmetic
        OP_VFMA         = 7'b1000111,  // Vector FMA
        OP_BRANCH       = 7'b1100011,  // Branch
        OP_SAMPLE       = 7'b0001011,  // Texture Sample (Custom)
        OP_INTERP       = 7'b0101011,  // Interpolation (Custom)
        OP_SPECIAL      = 7'b1111011   // Special GPU ops
    } shader_opcode_t;
    
    typedef struct packed {
        logic [6:0]     opcode;
        logic [4:0]     rd;
        logic [2:0]     funct3;
        logic [4:0]     rs1;
        logic [4:0]     rs2;
        logic [6:0]     funct7;
    } shader_instr_t;
    
    //==========================================================================
    // AXI4 Interface Structures
    //==========================================================================
    
    typedef struct packed {
        logic [AXI_ID_WIDTH-1:0]    id;
        logic [AXI_ADDR_WIDTH-1:0]  addr;
        logic [7:0]                 len;
        logic [2:0]                 size;
        logic [1:0]                 burst;
    } axi_aw_t;
    
    typedef struct packed {
        logic [AXI_DATA_WIDTH-1:0]  data;
        logic [AXI_DATA_WIDTH/8-1:0] strb;
        logic                       last;
    } axi_w_t;
    
    typedef struct packed {
        logic [AXI_ID_WIDTH-1:0]    id;
        logic [1:0]                 resp;
    } axi_b_t;
    
    typedef struct packed {
        logic [AXI_ID_WIDTH-1:0]    id;
        logic [AXI_ADDR_WIDTH-1:0]  addr;
        logic [7:0]                 len;
        logic [2:0]                 size;
        logic [1:0]                 burst;
    } axi_ar_t;
    
    typedef struct packed {
        logic [AXI_ID_WIDTH-1:0]    id;
        logic [AXI_DATA_WIDTH-1:0]  data;
        logic [1:0]                 resp;
        logic                       last;
    } axi_r_t;
    
    //==========================================================================
    // Performance Counters
    //==========================================================================
    
    typedef struct packed {
        logic [63:0] cycles;
        logic [63:0] vertices_processed;
        logic [63:0] primitives_processed;
        logic [63:0] pixels_shaded;
        logic [63:0] texture_samples;
        logic [63:0] cache_hits_l1;
        logic [63:0] cache_misses_l1;
        logic [63:0] cache_hits_l2;
        logic [63:0] cache_misses_l2;
    } perf_counters_t;
    
    //==========================================================================
    // Utility Functions
    //==========================================================================
    
    function automatic logic [31:0] float_to_fixed(input fp32_t f);
        // Simplified conversion - in real implementation would handle IEEE 754
        return f;
    endfunction
    
    function automatic fp32_t fixed_to_float(input logic [31:0] fixed);
        return fixed;
    endfunction
    
    function automatic logic [15:0] clog2_func(input logic [31:0] val);
        logic [15:0] result;
        result = 0;
        while (val > 1) begin
            val = val >> 1;
            result = result + 1;
        end
        return result;
    endfunction

endpackage : krypton_pkg
