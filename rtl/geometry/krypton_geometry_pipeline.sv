//==============================================================================
// GPU Krypton Series-1 - Geometry Pipeline
// Includes Vertex Fetch, Tiling Engine, and Early-Z
//==============================================================================

module krypton_geometry_pipeline
    import krypton_pkg::*;
(
    input  logic                        clk,
    input  logic                        rst_n,
    
    // Command interface
    input  logic                        cmd_valid,
    output logic                        cmd_ready,
    input  command_packet_t             cmd_packet,
    
    // Vertex buffer configuration
    input  logic [ADDR_WIDTH-1:0]       vertex_buffer_base,
    input  logic [31:0]                 vertex_stride,
    input  logic [ADDR_WIDTH-1:0]       index_buffer_base,
    input  logic [1:0]                  index_type,       // 0=u8, 1=u16, 2=u32
    
    // Memory interface
    output logic                        mem_req,
    output logic [ADDR_WIDTH-1:0]       mem_addr,
    output logic [7:0]                  mem_len,
    input  logic                        mem_ready,
    input  logic                        mem_valid,
    input  logic [AXI_DATA_WIDTH-1:0]   mem_data,
    
    // Shader cluster interface (for vertex shading)
    output logic                        vs_dispatch_valid,
    input  logic                        vs_dispatch_ready,
    output logic [ADDR_WIDTH-1:0]       vs_shader_pc,
    output logic [31:0]                 vs_vertex_count,
    output logic [ADDR_WIDTH-1:0]       vs_input_ptr,
    output logic [ADDR_WIDTH-1:0]       vs_output_ptr,
    
    input  logic                        vs_complete_valid,
    output logic                        vs_complete_ready,
    
    // Rasterizer interface (tile list output)
    output logic                        tile_valid,
    input  logic                        tile_ready,
    output tile_descriptor_t            tile_desc,
    
    // Primitive output for rasterization
    output logic                        prim_valid,
    input  logic                        prim_ready,
    output triangle_t                   prim_data,
    
    // Tile list memory write
    output logic                        tl_write_req,
    output logic [ADDR_WIDTH-1:0]       tl_write_addr,
    output logic [127:0]                tl_write_data,
    input  logic                        tl_write_ready,
    
    // Performance counters
    output logic [63:0]                 perf_vertices,
    output logic [63:0]                 perf_primitives,
    output logic [63:0]                 perf_culled
);

    //==========================================================================
    // State Machine
    //==========================================================================
    
    typedef enum logic [3:0] {
        ST_IDLE,
        ST_PARSE_CMD,
        ST_FETCH_INDEX,
        ST_WAIT_INDEX,
        ST_FETCH_VERTEX,
        ST_WAIT_VERTEX,
        ST_DISPATCH_VS,
        ST_WAIT_VS,
        ST_ASSEMBLE,
        ST_CULL_CLIP,
        ST_TILE_BIN,
        ST_WRITE_TILE,
        ST_DONE
    } state_t;
    
    state_t state, next_state;
    
    //==========================================================================
    // Draw Parameters
    //==========================================================================
    
    logic [31:0]                draw_vertex_count;
    logic [31:0]                draw_instance_count;
    logic [31:0]                draw_first_vertex;
    logic [31:0]                draw_first_instance;
    logic                       draw_indexed;
    logic [ADDR_WIDTH-1:0]      vs_program_pc;
    
    //==========================================================================
    // Index Buffer State
    //==========================================================================
    
    logic [31:0]                indices [0:255];
    logic [7:0]                 index_read_ptr;
    logic [7:0]                 index_count;
    
    //==========================================================================
    // Vertex Buffer State
    //==========================================================================
    
    logic [ADDR_WIDTH-1:0]      vertex_fetch_addr;
    logic [31:0]                vertices_fetched;
    logic [31:0]                vertices_to_fetch;
    
    // Vertex cache (post vertex shader)
    vertex_t                    vertex_cache [0:255];
    logic [7:0]                 vcache_write_ptr;
    logic [7:0]                 vcache_read_ptr;
    
    //==========================================================================
    // Primitive Assembly
    //==========================================================================
    
    logic [31:0]                prim_count;
    logic [31:0]                prim_idx;
    vertex_t                    v0, v1, v2;
    
    //==========================================================================
    // Culling/Clipping
    //==========================================================================
    
    logic                       cull_backface;
    logic                       cull_frustum;
    logic                       prim_visible;
    
    logic [31:0]                viewport_x, viewport_y;
    logic [31:0]                viewport_w, viewport_h;
    
    //==========================================================================
    // Tiling Engine
    //==========================================================================
    
    logic [15:0]                tile_prim_count [0:MAX_TILES_X*MAX_TILES_Y-1];
    logic [ADDR_WIDTH-1:0]      tile_list_ptr [0:MAX_TILES_X*MAX_TILES_Y-1];
    logic [ADDR_WIDTH-1:0]      tile_list_base;
    
    logic [15:0]                current_tile_x, current_tile_y;
    logic [15:0]                screen_tiles_x, screen_tiles_y;
    
    logic [15:0]                bb_min_x, bb_max_x;
    logic [15:0]                bb_min_y, bb_max_y;
    logic [15:0]                bb_min_tile_x, bb_max_tile_x;
    logic [15:0]                bb_min_tile_y, bb_max_tile_y;
    
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
                if (cmd_valid && cmd_ready)
                    next_state = ST_PARSE_CMD;
            end
            
            ST_PARSE_CMD: begin
                if (draw_indexed)
                    next_state = ST_FETCH_INDEX;
                else
                    next_state = ST_DISPATCH_VS;
            end
            
            ST_FETCH_INDEX: begin
                if (mem_ready)
                    next_state = ST_WAIT_INDEX;
            end
            
            ST_WAIT_INDEX: begin
                if (mem_valid)
                    next_state = ST_DISPATCH_VS;
            end
            
            ST_DISPATCH_VS: begin
                if (vs_dispatch_ready)
                    next_state = ST_WAIT_VS;
            end
            
            ST_WAIT_VS: begin
                if (vs_complete_valid)
                    next_state = ST_ASSEMBLE;
            end
            
            ST_ASSEMBLE: begin
                next_state = ST_CULL_CLIP;
            end
            
            ST_CULL_CLIP: begin
                if (prim_visible)
                    next_state = ST_TILE_BIN;
                else if (prim_idx < prim_count)
                    next_state = ST_ASSEMBLE;
                else
                    next_state = ST_WRITE_TILE;
            end
            
            ST_TILE_BIN: begin
                if (current_tile_y > bb_max_tile_y) begin
                    if (prim_idx < prim_count)
                        next_state = ST_ASSEMBLE;
                    else
                        next_state = ST_WRITE_TILE;
                end
            end
            
            ST_WRITE_TILE: begin
                if (current_tile_y >= screen_tiles_y)
                    next_state = ST_DONE;
            end
            
            ST_DONE: begin
                next_state = ST_IDLE;
            end
        endcase
    end
    
    //==========================================================================
    // Command Parsing
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            draw_vertex_count <= '0;
            draw_instance_count <= '0;
            draw_first_vertex <= '0;
            draw_first_instance <= '0;
            draw_indexed <= 1'b0;
            vs_program_pc <= '0;
        end else if (state == ST_IDLE && cmd_valid) begin
            case (cmd_packet.opcode)
                CMD_DRAW: begin
                    draw_vertex_count <= cmd_packet.param0;
                    draw_instance_count <= cmd_packet.param1;
                    draw_first_vertex <= cmd_packet.param2;
                    draw_first_instance <= cmd_packet.param3;
                    draw_indexed <= 1'b0;
                end
                CMD_DRAW_INDEXED: begin
                    draw_vertex_count <= cmd_packet.param0;
                    draw_instance_count <= cmd_packet.param1;
                    draw_first_vertex <= '0;
                    draw_first_instance <= cmd_packet.param3;
                    draw_indexed <= 1'b1;
                end
                default: begin
                    draw_vertex_count <= '0;
                end
            endcase
        end
    end
    
    assign cmd_ready = (state == ST_IDLE);
    
    //==========================================================================
    // Index Fetching
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            index_read_ptr <= '0;
            index_count <= '0;
        end else begin
            case (state)
                ST_PARSE_CMD: begin
                    index_read_ptr <= '0;
                    index_count <= (draw_vertex_count > 256) ? 8'd255 : draw_vertex_count[7:0];
                end
                
                ST_WAIT_INDEX: begin
                    if (mem_valid) begin
                        case (index_type)
                            2'b00: begin
                                for (int i = 0; i < 32; i++)
                                    indices[index_read_ptr + i] <= {24'b0, mem_data[i*8 +: 8]};
                                index_read_ptr <= index_read_ptr + 32;
                            end
                            2'b01: begin
                                for (int i = 0; i < 16; i++)
                                    indices[index_read_ptr + i] <= {16'b0, mem_data[i*16 +: 16]};
                                index_read_ptr <= index_read_ptr + 16;
                            end
                            2'b10: begin
                                for (int i = 0; i < 8; i++)
                                    indices[index_read_ptr + i] <= mem_data[i*32 +: 32];
                                index_read_ptr <= index_read_ptr + 8;
                            end
                            default: ;
                        endcase
                    end
                end
            endcase
        end
    end
    
    assign mem_req = (state == ST_FETCH_INDEX);
    assign mem_addr = index_buffer_base + (index_read_ptr << index_type);
    assign mem_len = 8'd0;
    
    //==========================================================================
    // Vertex Shader Dispatch
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vertices_fetched <= '0;
            vertices_to_fetch <= '0;
        end else begin
            case (state)
                ST_PARSE_CMD: begin
                    vertices_to_fetch <= draw_vertex_count;
                    vertices_fetched <= '0;
                end
                
                ST_WAIT_VS: begin
                    if (vs_complete_valid)
                        vertices_fetched <= vertices_fetched + vs_vertex_count;
                end
            endcase
        end
    end
    
    assign vs_dispatch_valid = (state == ST_DISPATCH_VS);
    assign vs_shader_pc = vs_program_pc;
    assign vs_vertex_count = (vertices_to_fetch - vertices_fetched > 256) ? 
                             32'd256 : (vertices_to_fetch - vertices_fetched);
    assign vs_input_ptr = vertex_buffer_base + (vertices_fetched * vertex_stride);
    assign vs_output_ptr = tile_list_base;
    assign vs_complete_ready = (state == ST_WAIT_VS);
    
    //==========================================================================
    // Primitive Assembly (Triangle List)
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prim_idx <= '0;
            prim_count <= '0;
            vcache_read_ptr <= '0;
        end else begin
            case (state)
                ST_WAIT_VS: begin
                    if (vs_complete_valid) begin
                        prim_count <= draw_vertex_count / 3;
                        prim_idx <= '0;
                        vcache_read_ptr <= '0;
                    end
                end
                
                ST_ASSEMBLE: begin
                    v0 <= vertex_cache[vcache_read_ptr];
                    v1 <= vertex_cache[vcache_read_ptr + 1];
                    v2 <= vertex_cache[vcache_read_ptr + 2];
                    vcache_read_ptr <= vcache_read_ptr + 3;
                    prim_idx <= prim_idx + 1;
                end
            endcase
        end
    end
    
    //==========================================================================
    // Backface and Frustum Culling
    //==========================================================================
    
    logic signed [63:0] signed_area;
    
    always_comb begin
        signed_area = ($signed(v1.position.x) - $signed(v0.position.x)) * 
                      ($signed(v2.position.y) - $signed(v0.position.y)) -
                      ($signed(v2.position.x) - $signed(v0.position.x)) * 
                      ($signed(v1.position.y) - $signed(v0.position.y));
        
        cull_backface = (signed_area <= 0);
        cull_frustum = ($signed(v0.position.z) < 0) && 
                       ($signed(v1.position.z) < 0) && 
                       ($signed(v2.position.z) < 0);
        
        prim_visible = !cull_backface && !cull_frustum;
    end
    
    //==========================================================================
    // Bounding Box Calculation
    //==========================================================================
    
    function automatic logic [15:0] min3(input logic [15:0] a, b, c);
        logic [15:0] tmp;
        tmp = (a < b) ? a : b;
        return (tmp < c) ? tmp : c;
    endfunction
    
    function automatic logic [15:0] max3(input logic [15:0] a, b, c);
        logic [15:0] tmp;
        tmp = (a > b) ? a : b;
        return (tmp > c) ? tmp : c;
    endfunction
    
    always_ff @(posedge clk) begin
        if (state == ST_CULL_CLIP && prim_visible) begin
            bb_min_x <= min3(v0.position.x[31:16], v1.position.x[31:16], v2.position.x[31:16]);
            bb_max_x <= max3(v0.position.x[31:16], v1.position.x[31:16], v2.position.x[31:16]);
            bb_min_y <= min3(v0.position.y[31:16], v1.position.y[31:16], v2.position.y[31:16]);
            bb_max_y <= max3(v0.position.y[31:16], v1.position.y[31:16], v2.position.y[31:16]);
            
            bb_min_tile_x <= bb_min_x >> 5;
            bb_max_tile_x <= bb_max_x >> 5;
            bb_min_tile_y <= bb_min_y >> 5;
            bb_max_tile_y <= bb_max_y >> 5;
        end
    end
    
    //==========================================================================
    // Tiling (Binning) Engine
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_tile_x <= '0;
            current_tile_y <= '0;
            for (int i = 0; i < MAX_TILES_X * MAX_TILES_Y; i++) begin
                tile_prim_count[i] <= '0;
                tile_list_ptr[i] <= '0;
            end
        end else begin
            case (state)
                ST_PARSE_CMD: begin
                    current_tile_x <= '0;
                    current_tile_y <= '0;
                    screen_tiles_x <= (viewport_w + TILE_WIDTH - 1) >> 5;
                    screen_tiles_y <= (viewport_h + TILE_HEIGHT - 1) >> 5;
                end
                
                ST_TILE_BIN: begin
                    if (current_tile_x >= bb_min_tile_x && current_tile_x <= bb_max_tile_x &&
                        current_tile_y >= bb_min_tile_y && current_tile_y <= bb_max_tile_y) begin
                        logic [15:0] tile_idx;
                        tile_idx = current_tile_y * screen_tiles_x + current_tile_x;
                        tile_prim_count[tile_idx] <= tile_prim_count[tile_idx] + 1;
                    end
                    
                    if (current_tile_x < bb_max_tile_x) begin
                        current_tile_x <= current_tile_x + 1;
                    end else begin
                        current_tile_x <= bb_min_tile_x;
                        current_tile_y <= current_tile_y + 1;
                    end
                end
                
                ST_WRITE_TILE: begin
                    if (tl_write_ready) begin
                        if (current_tile_x < screen_tiles_x - 1) begin
                            current_tile_x <= current_tile_x + 1;
                        end else begin
                            current_tile_x <= '0;
                            current_tile_y <= current_tile_y + 1;
                        end
                    end
                end
            endcase
        end
    end
    
    assign tl_write_req = (state == ST_WRITE_TILE);
    assign tl_write_addr = tile_list_base + 
                          ((current_tile_y * screen_tiles_x + current_tile_x) << 4);
    assign tl_write_data = {tile_prim_count[current_tile_y * screen_tiles_x + current_tile_x],
                           current_tile_y, current_tile_x, 80'b0};
    
    assign tile_valid = (state == ST_WRITE_TILE) && (current_tile_y < screen_tiles_y);
    assign tile_desc.tile_x = current_tile_x;
    assign tile_desc.tile_y = current_tile_y;
    assign tile_desc.primitive_count = tile_prim_count[current_tile_y * screen_tiles_x + current_tile_x];
    assign tile_desc.list_ptr = tile_list_ptr[current_tile_y * screen_tiles_x + current_tile_x];
    
    assign prim_valid = (state == ST_CULL_CLIP) && prim_visible;
    assign prim_data.v0 = v0.position;
    assign prim_data.v1 = v1.position;
    assign prim_data.v2 = v2.position;
    assign prim_data.primitive_id = prim_idx[15:0];
    
    //==========================================================================
    // Performance Counters
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            perf_vertices <= '0;
            perf_primitives <= '0;
            perf_culled <= '0;
        end else begin
            if (state == ST_WAIT_VS && vs_complete_valid)
                perf_vertices <= perf_vertices + vs_vertex_count;
            
            if (state == ST_CULL_CLIP) begin
                perf_primitives <= perf_primitives + 1;
                if (!prim_visible)
                    perf_culled <= perf_culled + 1;
            end
        end
    end

endmodule : krypton_geometry_pipeline
