//==============================================================================
// GPU Krypton Series-1 - Command Processor (CP)
// The "brain" of the GPU - fetches and decodes command buffers
//==============================================================================

module krypton_command_processor
    import krypton_pkg::*;
(
    input  logic                        clk,
    input  logic                        rst_n,
    
    // Configuration registers
    input  logic [ADDR_WIDTH-1:0]       cmd_buffer_base,
    input  logic [31:0]                 cmd_buffer_size,
    input  logic                        cmd_start,
    output logic                        cmd_busy,
    output logic                        cmd_done,
    
    // AXI Read interface for fetching commands
    output logic                        m_axi_arvalid,
    input  logic                        m_axi_arready,
    output logic [ADDR_WIDTH-1:0]       m_axi_araddr,
    output logic [7:0]                  m_axi_arlen,
    output logic [2:0]                  m_axi_arsize,
    output logic [1:0]                  m_axi_arburst,
    
    input  logic                        m_axi_rvalid,
    output logic                        m_axi_rready,
    input  logic [AXI_DATA_WIDTH-1:0]   m_axi_rdata,
    input  logic [1:0]                  m_axi_rresp,
    input  logic                        m_axi_rlast,
    
    // Geometry pipeline interface
    output logic                        geo_cmd_valid,
    input  logic                        geo_cmd_ready,
    output command_packet_t             geo_cmd_packet,
    
    // Rasterizer interface
    output logic                        rast_cmd_valid,
    input  logic                        rast_cmd_ready,
    output command_packet_t             rast_cmd_packet,
    
    // Compute dispatch interface
    output logic                        compute_cmd_valid,
    input  logic                        compute_cmd_ready,
    output command_packet_t             compute_cmd_packet,
    
    // Pipeline state output
    output pipeline_state_t             pipeline_state,
    output logic                        pipeline_state_valid,
    
    // Interrupt and fence
    output logic                        fence_interrupt,
    input  logic                        fence_ack,
    
    // Performance counters
    output logic [63:0]                 cmd_count
);

    //==========================================================================
    // State Machine
    //==========================================================================
    
    typedef enum logic [3:0] {
        ST_IDLE,
        ST_FETCH_REQ,
        ST_FETCH_WAIT,
        ST_DECODE,
        ST_DISPATCH_GEO,
        ST_DISPATCH_RAST,
        ST_DISPATCH_COMPUTE,
        ST_UPDATE_STATE,
        ST_FENCE_WAIT,
        ST_DONE
    } state_t;
    
    state_t state, next_state;
    
    //==========================================================================
    // Internal Registers
    //==========================================================================
    
    logic [ADDR_WIDTH-1:0]      cmd_ptr;
    logic [31:0]                cmd_remaining;
    logic [AXI_DATA_WIDTH-1:0]  fetch_buffer [0:3];
    logic [2:0]                 fetch_idx;
    logic [2:0]                 fetch_count;
    
    command_packet_t            current_cmd;
    logic                       cmd_decoded;
    
    pipeline_state_t            state_reg;
    
    //==========================================================================
    // Command FIFO
    //==========================================================================
    
    logic [159:0]               cmd_fifo [0:15];
    logic [3:0]                 cmd_fifo_wptr;
    logic [3:0]                 cmd_fifo_rptr;
    logic                       cmd_fifo_empty;
    logic                       cmd_fifo_full;
    
    assign cmd_fifo_empty = (cmd_fifo_wptr == cmd_fifo_rptr);
    assign cmd_fifo_full  = (cmd_fifo_wptr[3:0] == cmd_fifo_rptr[3:0]) && 
                            (cmd_fifo_wptr[3] != cmd_fifo_rptr[3]);
    
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
                if (cmd_start && cmd_buffer_size > 0)
                    next_state = ST_FETCH_REQ;
            end
            
            ST_FETCH_REQ: begin
                if (m_axi_arready)
                    next_state = ST_FETCH_WAIT;
            end
            
            ST_FETCH_WAIT: begin
                if (m_axi_rvalid && m_axi_rlast)
                    next_state = ST_DECODE;
            end
            
            ST_DECODE: begin
                if (cmd_decoded) begin
                    case (current_cmd.opcode)
                        CMD_DRAW_INDEXED, CMD_DRAW, CMD_BIND_VERTEX, CMD_BIND_INDEX:
                            next_state = ST_DISPATCH_GEO;
                        CMD_BEGIN_RENDER, CMD_END_RENDER, CMD_SET_VIEWPORT, CMD_SET_SCISSOR:
                            next_state = ST_DISPATCH_RAST;
                        CMD_DISPATCH:
                            next_state = ST_DISPATCH_COMPUTE;
                        CMD_BIND_PIPELINE:
                            next_state = ST_UPDATE_STATE;
                        CMD_FENCE:
                            next_state = ST_FENCE_WAIT;
                        CMD_NOP:
                            next_state = (cmd_remaining > 0) ? ST_FETCH_REQ : ST_DONE;
                        default:
                            next_state = (cmd_remaining > 0) ? ST_FETCH_REQ : ST_DONE;
                    endcase
                end
            end
            
            ST_DISPATCH_GEO: begin
                if (geo_cmd_ready)
                    next_state = (cmd_remaining > 0) ? ST_FETCH_REQ : ST_DONE;
            end
            
            ST_DISPATCH_RAST: begin
                if (rast_cmd_ready)
                    next_state = (cmd_remaining > 0) ? ST_FETCH_REQ : ST_DONE;
            end
            
            ST_DISPATCH_COMPUTE: begin
                if (compute_cmd_ready)
                    next_state = (cmd_remaining > 0) ? ST_FETCH_REQ : ST_DONE;
            end
            
            ST_UPDATE_STATE: begin
                next_state = (cmd_remaining > 0) ? ST_FETCH_REQ : ST_DONE;
            end
            
            ST_FENCE_WAIT: begin
                if (fence_ack)
                    next_state = (cmd_remaining > 0) ? ST_FETCH_REQ : ST_DONE;
            end
            
            ST_DONE: begin
                next_state = ST_IDLE;
            end
        endcase
    end
    
    //==========================================================================
    // Command Fetch Logic
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_ptr <= '0;
            cmd_remaining <= '0;
            fetch_idx <= '0;
            fetch_count <= '0;
            m_axi_arvalid <= 1'b0;
            m_axi_rready <= 1'b0;
        end else begin
            case (state)
                ST_IDLE: begin
                    if (cmd_start) begin
                        cmd_ptr <= cmd_buffer_base;
                        cmd_remaining <= cmd_buffer_size;
                        fetch_idx <= '0;
                    end
                end
                
                ST_FETCH_REQ: begin
                    m_axi_arvalid <= 1'b1;
                    m_axi_rready <= 1'b1;
                    if (m_axi_arready) begin
                        m_axi_arvalid <= 1'b0;
                    end
                end
                
                ST_FETCH_WAIT: begin
                    if (m_axi_rvalid) begin
                        fetch_buffer[fetch_idx] <= m_axi_rdata;
                        fetch_idx <= fetch_idx + 1'b1;
                        if (m_axi_rlast) begin
                            m_axi_rready <= 1'b0;
                            fetch_count <= fetch_idx + 1'b1;
                        end
                    end
                end
                
                ST_DECODE: begin
                    cmd_ptr <= cmd_ptr + 20; // Command packet size
                    cmd_remaining <= cmd_remaining - 1;
                    fetch_idx <= '0;
                end
            endcase
        end
    end
    
    // AXI read address channel
    assign m_axi_araddr  = cmd_ptr;
    assign m_axi_arlen   = 8'd0;  // Single beat
    assign m_axi_arsize  = 3'b101; // 32 bytes
    assign m_axi_arburst = 2'b01;  // INCR
    
    //==========================================================================
    // Command Decode Logic
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_cmd <= '0;
            cmd_decoded <= 1'b0;
        end else begin
            cmd_decoded <= 1'b0;
            
            if (state == ST_FETCH_WAIT && m_axi_rvalid && m_axi_rlast) begin
                // Extract command from fetched data
                current_cmd.opcode <= cmd_opcode_t'(m_axi_rdata[7:0]);
                current_cmd.flags  <= m_axi_rdata[31:8];
                current_cmd.param0 <= m_axi_rdata[63:32];
                current_cmd.param1 <= m_axi_rdata[95:64];
                current_cmd.param2 <= m_axi_rdata[127:96];
                current_cmd.param3 <= m_axi_rdata[159:128];
                cmd_decoded <= 1'b1;
            end
        end
    end
    
    //==========================================================================
    // Command Dispatch
    //==========================================================================
    
    assign geo_cmd_valid     = (state == ST_DISPATCH_GEO);
    assign geo_cmd_packet    = current_cmd;
    
    assign rast_cmd_valid    = (state == ST_DISPATCH_RAST);
    assign rast_cmd_packet   = current_cmd;
    
    assign compute_cmd_valid = (state == ST_DISPATCH_COMPUTE);
    assign compute_cmd_packet = current_cmd;
    
    //==========================================================================
    // Pipeline State Update
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_reg <= '0;
            pipeline_state_valid <= 1'b0;
        end else begin
            pipeline_state_valid <= 1'b0;
            
            if (state == ST_UPDATE_STATE) begin
                // Decode pipeline state from command parameters
                state_reg.depth_test_enable  <= current_cmd.param0[0];
                state_reg.depth_write_enable <= current_cmd.param0[1];
                state_reg.depth_func         <= compare_func_t'(current_cmd.param0[4:2]);
                state_reg.stencil_enable     <= current_cmd.param0[5];
                state_reg.stencil_func       <= compare_func_t'(current_cmd.param0[8:6]);
                state_reg.stencil_ref        <= current_cmd.param0[16:9];
                state_reg.stencil_mask       <= current_cmd.param0[24:17];
                state_reg.blend_enable       <= current_cmd.param1[0];
                state_reg.src_blend          <= blend_factor_t'(current_cmd.param1[3:1]);
                state_reg.dst_blend          <= blend_factor_t'(current_cmd.param1[6:4]);
                state_reg.cull_mode          <= cull_mode_t'(current_cmd.param1[8:7]);
                state_reg.front_ccw          <= current_cmd.param1[9];
                pipeline_state_valid <= 1'b1;
            end
        end
    end
    
    assign pipeline_state = state_reg;
    
    //==========================================================================
    // Fence and Interrupt Logic
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fence_interrupt <= 1'b0;
        end else begin
            if (state == ST_FENCE_WAIT && !fence_interrupt) begin
                fence_interrupt <= 1'b1;
            end else if (fence_ack) begin
                fence_interrupt <= 1'b0;
            end
        end
    end
    
    //==========================================================================
    // Status Outputs
    //==========================================================================
    
    assign cmd_busy = (state != ST_IDLE);
    assign cmd_done = (state == ST_DONE);
    
    //==========================================================================
    // Performance Counter
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_count <= '0;
        end else begin
            if (cmd_decoded && current_cmd.opcode != CMD_NOP)
                cmd_count <= cmd_count + 1'b1;
        end
    end

endmodule : krypton_command_processor
