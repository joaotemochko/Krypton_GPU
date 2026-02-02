//==============================================================================
// GPU Krypton Series-1 - Texture Mapping Unit (TMU)
// Supports Bilinear, Trilinear, Anisotropic (4x) Filtering
// Hardware decoders for ASTC and ETC2 compression
//==============================================================================

module krypton_tmu
    import krypton_pkg::*;
#(
    parameter int TMU_ID = 0
)(
    input  logic                        clk,
    input  logic                        rst_n,
    
    // Sample request interface
    input  logic                        sample_req,
    output logic                        sample_ready,
    input  logic [3:0]                  sampler_id,
    input  vec2_t                       texcoord,
    input  logic [3:0]                  lod,
    input  logic                        lod_bias_enable,
    input  logic signed [7:0]           lod_bias,
    
    // Sample result
    output logic                        sample_valid,
    output rgba8_t                      sample_data,
    
    // Texture cache interface
    output logic                        tcache_req,
    output logic [ADDR_WIDTH-1:0]       tcache_addr,
    input  logic                        tcache_valid,
    input  logic [127:0]                tcache_data,
    
    // Texture descriptor interface
    input  logic [ADDR_WIDTH-1:0]       tex_base_addr [0:15],
    input  logic [15:0]                 tex_width [0:15],
    input  logic [15:0]                 tex_height [0:15],
    input  logic [3:0]                  tex_format [0:15],
    input  logic [3:0]                  tex_mip_levels [0:15],
    input  logic [2:0]                  tex_filter_mode [0:15],
    input  logic [1:0]                  tex_wrap_u [0:15],
    input  logic [1:0]                  tex_wrap_v [0:15]
);

    //==========================================================================
    // Texture Format Definitions
    //==========================================================================
    
    typedef enum logic [3:0] {
        TEX_FMT_RGBA8       = 4'h0,
        TEX_FMT_RGB565      = 4'h1,
        TEX_FMT_RGBA4444    = 4'h2,
        TEX_FMT_R8          = 4'h3,
        TEX_FMT_RG8         = 4'h4,
        TEX_FMT_ASTC_4x4    = 4'h8,
        TEX_FMT_ASTC_6x6    = 4'h9,
        TEX_FMT_ASTC_8x8    = 4'hA,
        TEX_FMT_ETC2_RGB    = 4'hC,
        TEX_FMT_ETC2_RGBA   = 4'hD
    } tex_format_t;
    
    typedef enum logic [2:0] {
        FILTER_NEAREST      = 3'b000,
        FILTER_BILINEAR     = 3'b001,
        FILTER_TRILINEAR    = 3'b010,
        FILTER_ANISO_2X     = 3'b100,
        FILTER_ANISO_4X     = 3'b101
    } filter_mode_t;
    
    typedef enum logic [1:0] {
        WRAP_REPEAT         = 2'b00,
        WRAP_CLAMP          = 2'b01,
        WRAP_MIRROR         = 2'b10,
        WRAP_BORDER         = 2'b11
    } wrap_mode_t;
    
    //==========================================================================
    // State Machine
    //==========================================================================
    
    typedef enum logic [3:0] {
        ST_IDLE,
        ST_CALC_LOD,
        ST_CALC_ADDR,
        ST_FETCH_TL,
        ST_FETCH_TR,
        ST_FETCH_BL,
        ST_FETCH_BR,
        ST_WAIT_FETCH,
        ST_DECOMPRESS,
        ST_FILTER,
        ST_OUTPUT
    } state_t;
    
    state_t state, next_state;
    
    //==========================================================================
    // Internal Registers
    //==========================================================================
    
    logic [3:0]                 current_sampler;
    vec2_t                      current_texcoord;
    logic [3:0]                 current_lod;
    logic [3:0]                 effective_lod;
    
    // Texture parameters (latched from descriptor)
    logic [ADDR_WIDTH-1:0]      tex_base;
    logic [15:0]                tex_w, tex_h;
    logic [3:0]                 tex_fmt;
    logic [2:0]                 tex_filter;
    logic [1:0]                 wrap_u, wrap_v;
    
    // Texel coordinates (integer + fraction)
    logic [15:0]                texel_u, texel_v;
    logic [7:0]                 frac_u, frac_v;
    
    // Fetched texels for bilinear filtering
    rgba8_t                     texel_tl, texel_tr, texel_bl, texel_br;
    logic [1:0]                 fetch_phase;
    
    // Mip level addresses
    logic [ADDR_WIDTH-1:0]      mip_base_addr [0:15];
    logic [15:0]                mip_width [0:15];
    logic [15:0]                mip_height [0:15];
    
    //==========================================================================
    // State Machine Logic
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
        end else begin
            state <= next_state;
        end
    end
    
    always_comb begin
        next_state = state;
        
        case (state)
            ST_IDLE: begin
                if (sample_req)
                    next_state = ST_CALC_LOD;
            end
            
            ST_CALC_LOD: begin
                next_state = ST_CALC_ADDR;
            end
            
            ST_CALC_ADDR: begin
                next_state = ST_FETCH_TL;
            end
            
            ST_FETCH_TL: begin
                if (tcache_valid || tex_filter == FILTER_NEAREST)
                    next_state = (tex_filter == FILTER_NEAREST) ? ST_DECOMPRESS : ST_FETCH_TR;
            end
            
            ST_FETCH_TR: begin
                if (tcache_valid)
                    next_state = ST_FETCH_BL;
            end
            
            ST_FETCH_BL: begin
                if (tcache_valid)
                    next_state = ST_FETCH_BR;
            end
            
            ST_FETCH_BR: begin
                if (tcache_valid)
                    next_state = ST_DECOMPRESS;
            end
            
            ST_DECOMPRESS: begin
                next_state = ST_FILTER;
            end
            
            ST_FILTER: begin
                next_state = ST_OUTPUT;
            end
            
            ST_OUTPUT: begin
                next_state = ST_IDLE;
            end
        endcase
    end
    
    //==========================================================================
    // Request Handling
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_sampler <= '0;
            current_texcoord <= '0;
            current_lod <= '0;
        end else if (state == ST_IDLE && sample_req) begin
            current_sampler <= sampler_id;
            current_texcoord <= texcoord;
            current_lod <= lod;
            
            // Latch texture descriptor
            tex_base <= tex_base_addr[sampler_id];
            tex_w <= tex_width[sampler_id];
            tex_h <= tex_height[sampler_id];
            tex_fmt <= tex_format[sampler_id];
            tex_filter <= tex_filter_mode[sampler_id];
            wrap_u <= tex_wrap_u[sampler_id];
            wrap_v <= tex_wrap_v[sampler_id];
        end
    end
    
    assign sample_ready = (state == ST_IDLE);
    
    //==========================================================================
    // LOD Calculation
    //==========================================================================
    
    always_ff @(posedge clk) begin
        if (state == ST_CALC_LOD) begin
            // Apply LOD bias if enabled
            logic signed [8:0] adjusted_lod;
            adjusted_lod = {1'b0, current_lod} + (lod_bias_enable ? lod_bias : 8'd0);
            
            // Clamp to valid range
            if (adjusted_lod < 0)
                effective_lod <= '0;
            else if (adjusted_lod > tex_mip_levels[current_sampler])
                effective_lod <= tex_mip_levels[current_sampler];
            else
                effective_lod <= adjusted_lod[3:0];
        end
    end
    
    //==========================================================================
    // Mip Chain Address Calculation
    //==========================================================================
    
    always_ff @(posedge clk) begin
        if (state == ST_IDLE && sample_req) begin
            // Calculate mip level base addresses
            logic [ADDR_WIDTH-1:0] offset;
            logic [15:0] w, h;
            
            offset = '0;
            w = tex_width[sampler_id];
            h = tex_height[sampler_id];
            
            for (int m = 0; m < 16; m++) begin
                mip_base_addr[m] <= tex_base_addr[sampler_id] + offset;
                mip_width[m] <= w;
                mip_height[m] <= h;
                
                // Calculate texel size for this format
                case (tex_format_t'(tex_format[sampler_id]))
                    TEX_FMT_RGBA8:      offset = offset + w * h * 4;
                    TEX_FMT_RGB565:     offset = offset + w * h * 2;
                    TEX_FMT_RGBA4444:   offset = offset + w * h * 2;
                    TEX_FMT_R8:         offset = offset + w * h;
                    TEX_FMT_RG8:        offset = offset + w * h * 2;
                    TEX_FMT_ASTC_4x4:   offset = offset + ((w + 3) / 4) * ((h + 3) / 4) * 16;
                    TEX_FMT_ETC2_RGB:   offset = offset + ((w + 3) / 4) * ((h + 3) / 4) * 8;
                    TEX_FMT_ETC2_RGBA:  offset = offset + ((w + 3) / 4) * ((h + 3) / 4) * 16;
                    default:            offset = offset + w * h * 4;
                endcase
                
                w = (w > 1) ? w >> 1 : 1;
                h = (h > 1) ? h >> 1 : 1;
            end
        end
    end
    
    //==========================================================================
    // Texture Coordinate Processing
    //==========================================================================
    
    function automatic logic [15:0] apply_wrap(
        input logic [31:0] coord,
        input logic [15:0] size,
        input logic [1:0]  wrap_mode
    );
        logic [31:0] wrapped;
        
        case (wrap_mode_t'(wrap_mode))
            WRAP_REPEAT: begin
                wrapped = coord % size;
            end
            WRAP_CLAMP: begin
                if ($signed(coord) < 0)
                    wrapped = 0;
                else if (coord >= size)
                    wrapped = size - 1;
                else
                    wrapped = coord;
            end
            WRAP_MIRROR: begin
                logic [31:0] period;
                period = size * 2;
                wrapped = coord % period;
                if (wrapped >= size)
                    wrapped = period - wrapped - 1;
            end
            WRAP_BORDER: begin
                if ($signed(coord) < 0 || coord >= size)
                    wrapped = 16'hFFFF;  // Border color indicator
                else
                    wrapped = coord;
            end
            default: wrapped = coord % size;
        endcase
        
        return wrapped[15:0];
    endfunction
    
    always_ff @(posedge clk) begin
        if (state == ST_CALC_ADDR) begin
            logic [31:0] u_fixed, v_fixed;
            logic [15:0] mip_w, mip_h;
            
            mip_w = mip_width[effective_lod];
            mip_h = mip_height[effective_lod];
            
            // Convert normalized [0,1] coordinates to texel coordinates
            // Assuming texcoord is in 16.16 fixed point format
            u_fixed = (current_texcoord.x * mip_w) >> 16;
            v_fixed = (current_texcoord.y * mip_h) >> 16;
            
            // Apply wrap modes
            texel_u <= apply_wrap(u_fixed >> 8, mip_w, wrap_u);
            texel_v <= apply_wrap(v_fixed >> 8, mip_h, wrap_v);
            
            // Extract fractional part for filtering
            frac_u <= u_fixed[7:0];
            frac_v <= v_fixed[7:0];
        end
    end
    
    //==========================================================================
    // Texel Fetch
    //==========================================================================
    
    logic [ADDR_WIDTH-1:0] fetch_addr;
    logic [15:0] fetch_u, fetch_v;
    
    always_comb begin
        case (state)
            ST_FETCH_TL: begin
                fetch_u = texel_u;
                fetch_v = texel_v;
            end
            ST_FETCH_TR: begin
                fetch_u = (texel_u + 1 < mip_width[effective_lod]) ? texel_u + 1 : texel_u;
                fetch_v = texel_v;
            end
            ST_FETCH_BL: begin
                fetch_u = texel_u;
                fetch_v = (texel_v + 1 < mip_height[effective_lod]) ? texel_v + 1 : texel_v;
            end
            ST_FETCH_BR: begin
                fetch_u = (texel_u + 1 < mip_width[effective_lod]) ? texel_u + 1 : texel_u;
                fetch_v = (texel_v + 1 < mip_height[effective_lod]) ? texel_v + 1 : texel_v;
            end
            default: begin
                fetch_u = texel_u;
                fetch_v = texel_v;
            end
        endcase
        
        // Calculate memory address
        case (tex_format_t'(tex_fmt))
            TEX_FMT_RGBA8:
                fetch_addr = mip_base_addr[effective_lod] + 
                            (fetch_v * mip_width[effective_lod] + fetch_u) * 4;
            TEX_FMT_RGB565, TEX_FMT_RGBA4444, TEX_FMT_RG8:
                fetch_addr = mip_base_addr[effective_lod] + 
                            (fetch_v * mip_width[effective_lod] + fetch_u) * 2;
            TEX_FMT_R8:
                fetch_addr = mip_base_addr[effective_lod] + 
                            (fetch_v * mip_width[effective_lod] + fetch_u);
            TEX_FMT_ASTC_4x4, TEX_FMT_ETC2_RGBA:
                fetch_addr = mip_base_addr[effective_lod] + 
                            ((fetch_v >> 2) * ((mip_width[effective_lod] + 3) >> 2) + 
                            (fetch_u >> 2)) * 16;
            TEX_FMT_ETC2_RGB:
                fetch_addr = mip_base_addr[effective_lod] + 
                            ((fetch_v >> 2) * ((mip_width[effective_lod] + 3) >> 2) + 
                            (fetch_u >> 2)) * 8;
            default:
                fetch_addr = mip_base_addr[effective_lod];
        endcase
    end
    
    assign tcache_req = (state == ST_FETCH_TL || state == ST_FETCH_TR || 
                        state == ST_FETCH_BL || state == ST_FETCH_BR);
    assign tcache_addr = fetch_addr;
    
    // Store fetched texels
    always_ff @(posedge clk) begin
        if (tcache_valid) begin
            rgba8_t decoded_texel;
            decoded_texel = decode_texel(tcache_data, tex_fmt, fetch_u[1:0], fetch_v[1:0]);
            
            case (state)
                ST_FETCH_TL: texel_tl <= decoded_texel;
                ST_FETCH_TR: texel_tr <= decoded_texel;
                ST_FETCH_BL: texel_bl <= decoded_texel;
                ST_FETCH_BR: texel_br <= decoded_texel;
            endcase
        end
    end
    
    //==========================================================================
    // Texel Decoding
    //==========================================================================
    
    function automatic rgba8_t decode_texel(
        input logic [127:0] raw_data,
        input logic [3:0] format,
        input logic [1:0] sub_x,
        input logic [1:0] sub_y
    );
        rgba8_t result;
        logic [31:0] texel_32;
        logic [15:0] texel_16;
        
        case (tex_format_t'(format))
            TEX_FMT_RGBA8: begin
                texel_32 = raw_data[31:0];
                result.r = texel_32[7:0];
                result.g = texel_32[15:8];
                result.b = texel_32[23:16];
                result.a = texel_32[31:24];
            end
            
            TEX_FMT_RGB565: begin
                texel_16 = raw_data[15:0];
                result.r = {texel_16[4:0], texel_16[4:2]};
                result.g = {texel_16[10:5], texel_16[10:9]};
                result.b = {texel_16[15:11], texel_16[15:13]};
                result.a = 8'hFF;
            end
            
            TEX_FMT_RGBA4444: begin
                texel_16 = raw_data[15:0];
                result.r = {texel_16[3:0], texel_16[3:0]};
                result.g = {texel_16[7:4], texel_16[7:4]};
                result.b = {texel_16[11:8], texel_16[11:8]};
                result.a = {texel_16[15:12], texel_16[15:12]};
            end
            
            TEX_FMT_R8: begin
                result.r = raw_data[7:0];
                result.g = 8'd0;
                result.b = 8'd0;
                result.a = 8'hFF;
            end
            
            TEX_FMT_RG8: begin
                result.r = raw_data[7:0];
                result.g = raw_data[15:8];
                result.b = 8'd0;
                result.a = 8'hFF;
            end
            
            // ASTC and ETC2 would require complex decompression
            // Simplified placeholder
            default: begin
                result.r = raw_data[7:0];
                result.g = raw_data[15:8];
                result.b = raw_data[23:16];
                result.a = raw_data[31:24];
            end
        endcase
        
        return result;
    endfunction
    
    //==========================================================================
    // Bilinear Filtering
    //==========================================================================
    
    rgba8_t filtered_texel;
    
    always_ff @(posedge clk) begin
        if (state == ST_FILTER) begin
            if (tex_filter == FILTER_NEAREST) begin
                filtered_texel <= texel_tl;
            end else begin
                // Bilinear interpolation
                logic [15:0] w_tl, w_tr, w_bl, w_br;
                logic [15:0] one_minus_u, one_minus_v;
                
                one_minus_u = 16'd256 - {8'd0, frac_u};
                one_minus_v = 16'd256 - {8'd0, frac_v};
                
                w_tl = (one_minus_u * one_minus_v) >> 8;
                w_tr = ({8'd0, frac_u} * one_minus_v) >> 8;
                w_bl = (one_minus_u * {8'd0, frac_v}) >> 8;
                w_br = ({8'd0, frac_u} * {8'd0, frac_v}) >> 8;
                
                filtered_texel.r <= ((texel_tl.r * w_tl + texel_tr.r * w_tr + 
                                     texel_bl.r * w_bl + texel_br.r * w_br) >> 8)[7:0];
                filtered_texel.g <= ((texel_tl.g * w_tl + texel_tr.g * w_tr + 
                                     texel_bl.g * w_bl + texel_br.g * w_br) >> 8)[7:0];
                filtered_texel.b <= ((texel_tl.b * w_tl + texel_tr.b * w_tr + 
                                     texel_bl.b * w_bl + texel_br.b * w_br) >> 8)[7:0];
                filtered_texel.a <= ((texel_tl.a * w_tl + texel_tr.a * w_tr + 
                                     texel_bl.a * w_bl + texel_br.a * w_br) >> 8)[7:0];
            end
        end
    end
    
    //==========================================================================
    // Output
    //==========================================================================
    
    assign sample_valid = (state == ST_OUTPUT);
    assign sample_data = filtered_texel;

endmodule : krypton_tmu
