//==============================================================================
// GPU Krypton Series-1 - Rasterizer and ROP
// Tile-Based Deferred Rendering with On-Chip Tile Buffer
//==============================================================================

module krypton_rasterizer
    import krypton_pkg::*;
(
    input  logic                        clk,
    input  logic                        rst_n,
    
    // Tile input from geometry pipeline
    input  logic                        tile_valid,
    output logic                        tile_ready,
    input  tile_descriptor_t            tile_desc,
    
    // Primitive input
    input  logic                        prim_valid,
    output logic                        prim_ready,
    input  triangle_t                   prim_data,
    
    // Pipeline state
    input  pipeline_state_t             pipeline_state,
    
    // Pixel shader dispatch
    output logic                        ps_dispatch_valid,
    input  logic                        ps_dispatch_ready,
    output logic [ADDR_WIDTH-1:0]       ps_shader_pc,
    output logic [31:0]                 ps_pixel_count,
    output logic [15:0]                 ps_tile_x,
    output logic [15:0]                 ps_tile_y,
    
    input  logic                        ps_complete_valid,
    output logic                        ps_complete_ready,
    input  rgba8_t                      ps_color_out [0:TILE_WIDTH*TILE_HEIGHT-1],
    input  logic [23:0]                 ps_depth_out [0:TILE_WIDTH*TILE_HEIGHT-1],
    
    // Tile buffer write to memory
    output logic                        fb_write_req,
    output logic [ADDR_WIDTH-1:0]       fb_write_addr,
    output logic [AXI_DATA_WIDTH-1:0]   fb_write_data,
    output logic [31:0]                 fb_write_strb,
    input  logic                        fb_write_ready,
    
    // Framebuffer configuration
    input  logic [ADDR_WIDTH-1:0]       fb_color_base,
    input  logic [ADDR_WIDTH-1:0]       fb_depth_base,
    input  logic [31:0]                 fb_width,
    input  logic [31:0]                 fb_height,
    input  logic [3:0]                  fb_format,        // 0=RGBA8, 1=RGB565, etc.
    
    // Performance counters
    output logic [63:0]                 perf_pixels_in,
    output logic [63:0]                 perf_pixels_out,
    output logic [63:0]                 perf_depth_pass,
    output logic [63:0]                 perf_depth_fail
);

    //==========================================================================
    // On-Chip Tile Buffer (OTB) - 128KB SRAM
    // Stores Color + Depth + Stencil for current tile
    //==========================================================================
    
    // Color buffer: 32x32 * 4 bytes = 4KB per tile
    rgba8_t                     otb_color [0:TILE_WIDTH*TILE_HEIGHT-1];
    
    // Depth buffer: 32x32 * 3 bytes = 3KB per tile (24-bit depth)
    logic [23:0]                otb_depth [0:TILE_WIDTH*TILE_HEIGHT-1];
    
    // Stencil buffer: 32x32 * 1 byte = 1KB per tile
    logic [7:0]                 otb_stencil [0:TILE_WIDTH*TILE_HEIGHT-1];
    
    // Coverage mask for current primitive
    logic [TILE_WIDTH*TILE_HEIGHT-1:0] coverage_mask;
    
    //==========================================================================
    // State Machine
    //==========================================================================
    
    typedef enum logic [3:0] {
        ST_IDLE,
        ST_LOAD_TILE,
        ST_WAIT_TILE_LOAD,
        ST_RASTERIZE,
        ST_EARLY_Z,
        ST_DISPATCH_PS,
        ST_WAIT_PS,
        ST_BLEND,
        ST_LATE_Z,
        ST_WRITEBACK,
        ST_WAIT_WB,
        ST_DONE
    } state_t;
    
    state_t state, next_state;
    
    //==========================================================================
    // Current Tile State
    //==========================================================================
    
    tile_descriptor_t           current_tile;
    logic [15:0]                prims_remaining;
    logic [15:0]                current_prim;
    
    triangle_t                  current_triangle;
    
    // Rasterization state
    logic [31:0]                edge0_a, edge0_b, edge0_c;
    logic [31:0]                edge1_a, edge1_b, edge1_c;
    logic [31:0]                edge2_a, edge2_b, edge2_c;
    
    logic [15:0]                pixel_x, pixel_y;
    logic [9:0]                 pixel_idx;
    logic [31:0]                pixels_covered;
    
    //==========================================================================
    // Writeback State
    //==========================================================================
    
    logic [9:0]                 wb_pixel_idx;
    logic [ADDR_WIDTH-1:0]      wb_addr;
    logic [3:0]                 wb_burst_cnt;
    
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
                if (tile_valid)
                    next_state = ST_LOAD_TILE;
            end
            
            ST_LOAD_TILE: begin
                next_state = ST_WAIT_TILE_LOAD;
            end
            
            ST_WAIT_TILE_LOAD: begin
                // For TBDR, we clear the tile buffer instead of loading
                next_state = ST_RASTERIZE;
            end
            
            ST_RASTERIZE: begin
                if (prim_valid && current_prim < current_tile.primitive_count)
                    next_state = ST_EARLY_Z;
                else if (current_prim >= current_tile.primitive_count)
                    next_state = ST_WRITEBACK;
            end
            
            ST_EARLY_Z: begin
                if (pixel_x >= TILE_WIDTH && pixel_y >= TILE_HEIGHT - 1)
                    next_state = ST_DISPATCH_PS;
            end
            
            ST_DISPATCH_PS: begin
                if (ps_dispatch_ready && pixels_covered > 0)
                    next_state = ST_WAIT_PS;
                else
                    next_state = ST_RASTERIZE;
            end
            
            ST_WAIT_PS: begin
                if (ps_complete_valid)
                    next_state = ST_BLEND;
            end
            
            ST_BLEND: begin
                if (pixel_idx >= TILE_WIDTH * TILE_HEIGHT)
                    next_state = ST_LATE_Z;
            end
            
            ST_LATE_Z: begin
                next_state = ST_RASTERIZE;
            end
            
            ST_WRITEBACK: begin
                if (fb_write_ready)
                    next_state = ST_WAIT_WB;
            end
            
            ST_WAIT_WB: begin
                if (wb_pixel_idx >= TILE_WIDTH * TILE_HEIGHT)
                    next_state = ST_DONE;
                else
                    next_state = ST_WRITEBACK;
            end
            
            ST_DONE: begin
                next_state = ST_IDLE;
            end
        endcase
    end
    
    //==========================================================================
    // Tile Loading / Clearing
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_tile <= '0;
            current_prim <= '0;
        end else begin
            case (state)
                ST_IDLE: begin
                    if (tile_valid) begin
                        current_tile <= tile_desc;
                        current_prim <= '0;
                    end
                end
                
                ST_WAIT_TILE_LOAD: begin
                    // Clear tile buffer (TBDR doesn't load from memory)
                    for (int i = 0; i < TILE_WIDTH * TILE_HEIGHT; i++) begin
                        otb_color[i] <= '{r: 8'd0, g: 8'd0, b: 8'd0, a: 8'd255};
                        otb_depth[i] <= 24'hFFFFFF;  // Far plane
                        otb_stencil[i] <= 8'd0;
                    end
                end
                
                ST_RASTERIZE: begin
                    if (prim_valid) begin
                        current_triangle <= prim_data;
                        current_prim <= current_prim + 1;
                    end
                end
            endcase
        end
    end
    
    assign tile_ready = (state == ST_IDLE);
    assign prim_ready = (state == ST_RASTERIZE);
    
    //==========================================================================
    // Edge Function Setup
    //==========================================================================
    
    // Edge equation: E(x,y) = a*x + b*y + c
    // Pixel is inside if all three edge functions are >= 0
    
    always_ff @(posedge clk) begin
        if (state == ST_RASTERIZE && prim_valid) begin
            // Edge 0: v0 -> v1
            edge0_a <= current_triangle.v0.y - current_triangle.v1.y;
            edge0_b <= current_triangle.v1.x - current_triangle.v0.x;
            edge0_c <= current_triangle.v0.x * current_triangle.v1.y - 
                       current_triangle.v1.x * current_triangle.v0.y;
            
            // Edge 1: v1 -> v2
            edge1_a <= current_triangle.v1.y - current_triangle.v2.y;
            edge1_b <= current_triangle.v2.x - current_triangle.v1.x;
            edge1_c <= current_triangle.v1.x * current_triangle.v2.y - 
                       current_triangle.v2.x * current_triangle.v1.y;
            
            // Edge 2: v2 -> v0
            edge2_a <= current_triangle.v2.y - current_triangle.v0.y;
            edge2_b <= current_triangle.v0.x - current_triangle.v2.x;
            edge2_c <= current_triangle.v2.x * current_triangle.v0.y - 
                       current_triangle.v0.x * current_triangle.v2.y;
        end
    end
    
    //==========================================================================
    // Rasterization (Coverage Test)
    //==========================================================================
    
    logic signed [63:0] e0, e1, e2;
    logic               inside;
    
    always_comb begin
        // Calculate edge functions at current pixel
        logic [31:0] px, py;
        px = (current_tile.tile_x << 5) + pixel_x;
        py = (current_tile.tile_y << 5) + pixel_y;
        
        e0 = $signed(edge0_a) * $signed({1'b0, px}) + 
             $signed(edge0_b) * $signed({1'b0, py}) + 
             $signed(edge0_c);
        e1 = $signed(edge1_a) * $signed({1'b0, px}) + 
             $signed(edge1_b) * $signed({1'b0, py}) + 
             $signed(edge1_c);
        e2 = $signed(edge2_a) * $signed({1'b0, px}) + 
             $signed(edge2_b) * $signed({1'b0, py}) + 
             $signed(edge2_c);
        
        inside = (e0 >= 0) && (e1 >= 0) && (e2 >= 0);
    end
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pixel_x <= '0;
            pixel_y <= '0;
            pixel_idx <= '0;
            coverage_mask <= '0;
            pixels_covered <= '0;
        end else begin
            case (state)
                ST_RASTERIZE: begin
                    pixel_x <= '0;
                    pixel_y <= '0;
                    pixel_idx <= '0;
                    coverage_mask <= '0;
                    pixels_covered <= '0;
                end
                
                ST_EARLY_Z: begin
                    // Test each pixel in tile
                    pixel_idx <= pixel_y * TILE_WIDTH + pixel_x;
                    
                    if (inside) begin
                        // Early depth test
                        logic [23:0] frag_depth;
                        frag_depth = interpolate_depth(e0, e1, e2);
                        
                        if (depth_test_pass(frag_depth, otb_depth[pixel_idx])) begin
                            coverage_mask[pixel_idx] <= 1'b1;
                            pixels_covered <= pixels_covered + 1;
                        end
                    end
                    
                    // Advance to next pixel
                    if (pixel_x < TILE_WIDTH - 1) begin
                        pixel_x <= pixel_x + 1;
                    end else begin
                        pixel_x <= '0;
                        pixel_y <= pixel_y + 1;
                    end
                end
            endcase
        end
    end
    
    // Depth interpolation (simplified)
    function automatic logic [23:0] interpolate_depth(
        input logic signed [63:0] w0, w1, w2
    );
        // Barycentric interpolation of depth
        logic [63:0] total_area;
        logic [63:0] depth_interp;
        
        total_area = w0 + w1 + w2;
        if (total_area == 0) return 24'hFFFFFF;
        
        depth_interp = (w0 * current_triangle.v2.z + 
                       w1 * current_triangle.v0.z + 
                       w2 * current_triangle.v1.z) / total_area;
        
        return depth_interp[23:0];
    endfunction
    
    // Depth test comparison
    function automatic logic depth_test_pass(
        input logic [23:0] frag_depth,
        input logic [23:0] buffer_depth
    );
        case (pipeline_state.depth_func)
            COMPARE_NEVER:   return 1'b0;
            COMPARE_LESS:    return (frag_depth < buffer_depth);
            COMPARE_EQUAL:   return (frag_depth == buffer_depth);
            COMPARE_LEQUAL:  return (frag_depth <= buffer_depth);
            COMPARE_GREATER: return (frag_depth > buffer_depth);
            COMPARE_NOTEQUAL: return (frag_depth != buffer_depth);
            COMPARE_GEQUAL:  return (frag_depth >= buffer_depth);
            COMPARE_ALWAYS:  return 1'b1;
            default:         return 1'b1;
        endcase
    endfunction
    
    //==========================================================================
    // Pixel Shader Dispatch
    //==========================================================================
    
    assign ps_dispatch_valid = (state == ST_DISPATCH_PS) && (pixels_covered > 0);
    assign ps_pixel_count = pixels_covered;
    assign ps_tile_x = current_tile.tile_x;
    assign ps_tile_y = current_tile.tile_y;
    assign ps_complete_ready = (state == ST_WAIT_PS);
    
    //==========================================================================
    // Blending
    //==========================================================================
    
    logic [9:0] blend_idx;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            blend_idx <= '0;
        end else begin
            case (state)
                ST_WAIT_PS: begin
                    blend_idx <= '0;
                end
                
                ST_BLEND: begin
                    if (coverage_mask[blend_idx]) begin
                        rgba8_t src_color;
                        rgba8_t dst_color;
                        rgba8_t blended;
                        
                        src_color = ps_color_out[blend_idx];
                        dst_color = otb_color[blend_idx];
                        
                        if (pipeline_state.blend_enable) begin
                            blended = blend_colors(src_color, dst_color);
                        end else begin
                            blended = src_color;
                        end
                        
                        otb_color[blend_idx] <= blended;
                        
                        // Update depth if test passed
                        if (pipeline_state.depth_write_enable) begin
                            otb_depth[blend_idx] <= ps_depth_out[blend_idx];
                        end
                    end
                    
                    blend_idx <= blend_idx + 1;
                end
            endcase
        end
    end
    
    // Alpha blending
    function automatic rgba8_t blend_colors(
        input rgba8_t src,
        input rgba8_t dst
    );
        logic [15:0] src_factor_r, src_factor_g, src_factor_b, src_factor_a;
        logic [15:0] dst_factor_r, dst_factor_g, dst_factor_b, dst_factor_a;
        rgba8_t result;
        
        // Get blend factors
        case (pipeline_state.src_blend)
            BLEND_ZERO:          begin src_factor_r = 0; src_factor_g = 0; src_factor_b = 0; src_factor_a = 0; end
            BLEND_ONE:           begin src_factor_r = 255; src_factor_g = 255; src_factor_b = 255; src_factor_a = 255; end
            BLEND_SRC_ALPHA:     begin src_factor_r = src.a; src_factor_g = src.a; src_factor_b = src.a; src_factor_a = src.a; end
            BLEND_INV_SRC_ALPHA: begin src_factor_r = 255-src.a; src_factor_g = 255-src.a; src_factor_b = 255-src.a; src_factor_a = 255-src.a; end
            default:             begin src_factor_r = 255; src_factor_g = 255; src_factor_b = 255; src_factor_a = 255; end
        endcase
        
        case (pipeline_state.dst_blend)
            BLEND_ZERO:          begin dst_factor_r = 0; dst_factor_g = 0; dst_factor_b = 0; dst_factor_a = 0; end
            BLEND_ONE:           begin dst_factor_r = 255; dst_factor_g = 255; dst_factor_b = 255; dst_factor_a = 255; end
            BLEND_SRC_ALPHA:     begin dst_factor_r = src.a; dst_factor_g = src.a; dst_factor_b = src.a; dst_factor_a = src.a; end
            BLEND_INV_SRC_ALPHA: begin dst_factor_r = 255-src.a; dst_factor_g = 255-src.a; dst_factor_b = 255-src.a; dst_factor_a = 255-src.a; end
            default:             begin dst_factor_r = 0; dst_factor_g = 0; dst_factor_b = 0; dst_factor_a = 0; end
        endcase
        
        // Blend equation: result = src * srcFactor + dst * dstFactor
        result.r = ((src.r * src_factor_r + dst.r * dst_factor_r) / 255)[7:0];
        result.g = ((src.g * src_factor_g + dst.g * dst_factor_g) / 255)[7:0];
        result.b = ((src.b * src_factor_b + dst.b * dst_factor_b) / 255)[7:0];
        result.a = ((src.a * src_factor_a + dst.a * dst_factor_a) / 255)[7:0];
        
        return result;
    endfunction
    
    //==========================================================================
    // Tile Writeback to Framebuffer
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wb_pixel_idx <= '0;
            wb_addr <= '0;
            wb_burst_cnt <= '0;
        end else begin
            case (state)
                ST_LATE_Z: begin
                    wb_pixel_idx <= '0;
                    wb_burst_cnt <= '0;
                end
                
                ST_WRITEBACK: begin
                    // Calculate framebuffer address
                    logic [31:0] fb_x, fb_y;
                    fb_x = (current_tile.tile_x << 5) + (wb_pixel_idx % TILE_WIDTH);
                    fb_y = (current_tile.tile_y << 5) + (wb_pixel_idx / TILE_WIDTH);
                    wb_addr <= fb_color_base + (fb_y * fb_width + fb_x) * 4;
                end
                
                ST_WAIT_WB: begin
                    if (fb_write_ready) begin
                        wb_pixel_idx <= wb_pixel_idx + 8;  // 8 pixels per burst
                        wb_burst_cnt <= wb_burst_cnt + 1;
                    end
                end
            endcase
        end
    end
    
    assign fb_write_req = (state == ST_WRITEBACK);
    assign fb_write_addr = wb_addr;
    
    // Pack 8 pixels into 256-bit write data
    always_comb begin
        for (int i = 0; i < 8; i++) begin
            fb_write_data[i*32 +: 32] = {otb_color[wb_pixel_idx + i].a,
                                         otb_color[wb_pixel_idx + i].b,
                                         otb_color[wb_pixel_idx + i].g,
                                         otb_color[wb_pixel_idx + i].r};
        end
        fb_write_strb = 32'hFFFFFFFF;
    end
    
    //==========================================================================
    // Performance Counters
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            perf_pixels_in <= '0;
            perf_pixels_out <= '0;
            perf_depth_pass <= '0;
            perf_depth_fail <= '0;
        end else begin
            if (state == ST_EARLY_Z && inside)
                perf_pixels_in <= perf_pixels_in + 1;
            
            if (state == ST_BLEND && coverage_mask[blend_idx])
                perf_pixels_out <= perf_pixels_out + 1;
            
            if (state == ST_EARLY_Z && inside) begin
                if (coverage_mask[pixel_idx])
                    perf_depth_pass <= perf_depth_pass + 1;
                else
                    perf_depth_fail <= perf_depth_fail + 1;
            end
        end
    end

endmodule : krypton_rasterizer
