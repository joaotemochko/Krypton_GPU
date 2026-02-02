//==============================================================================
// GPU Krypton Series-1 - Krypton Core (SIMT Execution Unit)
// Unified Shader Architecture - Processes Vertex, Pixel, and Compute Shaders
// ISA: RISC-V Vector Extensions (RVV) Customizada
//==============================================================================

module krypton_core
    import krypton_pkg::*;
#(
    parameter int CORE_ID = 0
)(
    input  logic                        clk,
    input  logic                        rst_n,
    
    // Warp scheduler interface
    input  logic                        warp_valid,
    output logic                        warp_ready,
    input  logic [4:0]                  warp_id,
    input  logic [ADDR_WIDTH-1:0]       warp_pc,
    input  logic [31:0]                 warp_mask,    // Active thread mask
    
    // Warp completion
    output logic                        warp_done,
    output logic [4:0]                  warp_done_id,
    
    // Instruction cache interface
    output logic                        icache_req,
    output logic [ADDR_WIDTH-1:0]       icache_addr,
    input  logic                        icache_valid,
    input  logic [31:0]                 icache_data,
    
    // Vector register file interface (external)
    output logic                        vrf_rd_en,
    output logic [9:0]                  vrf_rd_addr,
    input  logic [DATA_WIDTH*32-1:0]    vrf_rd_data,  // 32 lanes
    
    output logic                        vrf_wr_en,
    output logic [9:0]                  vrf_wr_addr,
    output logic [DATA_WIDTH*32-1:0]    vrf_wr_data,
    output logic [31:0]                 vrf_wr_mask,
    
    // Scalar register file interface
    output logic                        srf_rd_en,
    output logic [7:0]                  srf_rd_addr,
    input  logic [DATA_WIDTH-1:0]       srf_rd_data,
    
    output logic                        srf_wr_en,
    output logic [7:0]                  srf_wr_addr,
    output logic [DATA_WIDTH-1:0]       srf_wr_data,
    
    // Load/Store unit interface
    output logic                        lsu_req,
    output logic                        lsu_write,
    output logic [ADDR_WIDTH-1:0]       lsu_addr [0:31],
    output logic [DATA_WIDTH-1:0]       lsu_wdata [0:31],
    output logic [31:0]                 lsu_mask,
    input  logic                        lsu_ready,
    input  logic                        lsu_valid,
    input  logic [DATA_WIDTH-1:0]       lsu_rdata [0:31],
    
    // Texture unit interface
    output logic                        tex_req,
    output logic [3:0]                  tex_unit,
    output vec2_t                       tex_coord [0:31],
    output logic [31:0]                 tex_mask,
    input  logic                        tex_ready,
    input  logic                        tex_valid,
    input  rgba8_t                      tex_data [0:31],
    
    // Special function unit interface
    output logic                        sfu_req,
    output logic [3:0]                  sfu_op,
    output logic [DATA_WIDTH-1:0]       sfu_operand [0:31],
    output logic [31:0]                 sfu_mask,
    input  logic                        sfu_ready,
    input  logic                        sfu_valid,
    input  logic [DATA_WIDTH-1:0]       sfu_result [0:31]
);

    //==========================================================================
    // Pipeline Stages
    //==========================================================================
    
    typedef enum logic [2:0] {
        STAGE_IDLE,
        STAGE_FETCH,
        STAGE_DECODE,
        STAGE_OPERAND,
        STAGE_EXECUTE,
        STAGE_MEMORY,
        STAGE_WRITEBACK
    } pipeline_stage_t;
    
    //==========================================================================
    // Warp Context Structure
    //==========================================================================
    
    typedef struct packed {
        logic                   valid;
        logic [4:0]             id;
        logic [ADDR_WIDTH-1:0]  pc;
        logic [31:0]            active_mask;
        logic [31:0]            exec_mask;
        logic                   diverged;
        logic [3:0]             div_stack_ptr;
    } warp_context_t;
    
    // Divergence stack for handling branches
    typedef struct packed {
        logic [ADDR_WIDTH-1:0]  reconverge_pc;
        logic [31:0]            active_mask;
        logic [31:0]            exec_mask;
    } divergence_entry_t;
    
    //==========================================================================
    // Internal State
    //==========================================================================
    
    warp_context_t              current_warp;
    divergence_entry_t          div_stack [0:15];
    
    pipeline_stage_t            stage;
    
    // Fetched instruction
    logic [31:0]                instr;
    logic                       instr_valid;
    
    // Decoded instruction fields
    shader_instr_t              decoded;
    logic [4:0]                 rd, rs1, rs2, rs3;
    logic [31:0]                immediate;
    logic                       is_vector_op;
    logic                       is_memory_op;
    logic                       is_texture_op;
    logic                       is_branch_op;
    logic                       is_special_op;
    
    // Operand data
    logic [DATA_WIDTH-1:0]      src1_data [0:31];
    logic [DATA_WIDTH-1:0]      src2_data [0:31];
    logic [DATA_WIDTH-1:0]      src3_data [0:31];
    logic [DATA_WIDTH-1:0]      scalar_data;
    
    // Execution results
    logic [DATA_WIDTH-1:0]      exec_result [0:31];
    logic [31:0]                exec_mask;
    logic                       exec_done;
    
    // Branch evaluation
    logic [31:0]                branch_mask;
    logic                       branch_taken;
    logic                       branch_divergent;
    
    //==========================================================================
    // Main State Machine
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stage <= STAGE_IDLE;
            current_warp <= '0;
            instr_valid <= 1'b0;
            warp_done <= 1'b0;
        end else begin
            warp_done <= 1'b0;
            
            case (stage)
                STAGE_IDLE: begin
                    if (warp_valid && warp_ready) begin
                        current_warp.valid <= 1'b1;
                        current_warp.id <= warp_id;
                        current_warp.pc <= warp_pc;
                        current_warp.active_mask <= warp_mask;
                        current_warp.exec_mask <= warp_mask;
                        current_warp.diverged <= 1'b0;
                        current_warp.div_stack_ptr <= '0;
                        stage <= STAGE_FETCH;
                    end
                end
                
                STAGE_FETCH: begin
                    if (icache_valid) begin
                        instr <= icache_data;
                        instr_valid <= 1'b1;
                        stage <= STAGE_DECODE;
                    end
                end
                
                STAGE_DECODE: begin
                    stage <= STAGE_OPERAND;
                end
                
                STAGE_OPERAND: begin
                    if (vrf_rd_en || srf_rd_en) begin
                        // Wait for register read
                        stage <= STAGE_EXECUTE;
                    end else begin
                        stage <= STAGE_EXECUTE;
                    end
                end
                
                STAGE_EXECUTE: begin
                    if (is_memory_op || is_texture_op) begin
                        stage <= STAGE_MEMORY;
                    end else if (exec_done) begin
                        stage <= STAGE_WRITEBACK;
                    end
                end
                
                STAGE_MEMORY: begin
                    if ((lsu_valid && is_memory_op) || (tex_valid && is_texture_op)) begin
                        stage <= STAGE_WRITEBACK;
                    end
                end
                
                STAGE_WRITEBACK: begin
                    // Check for end of warp
                    if (decoded.opcode == 7'b1111111 || current_warp.active_mask == '0) begin
                        warp_done <= 1'b1;
                        warp_done_id <= current_warp.id;
                        stage <= STAGE_IDLE;
                    end else begin
                        // Continue to next instruction
                        if (is_branch_op && branch_taken) begin
                            // Handle branch
                            if (branch_divergent) begin
                                // Push divergence stack
                                div_stack[current_warp.div_stack_ptr].reconverge_pc <= 
                                    current_warp.pc + 4;
                                div_stack[current_warp.div_stack_ptr].active_mask <= 
                                    current_warp.active_mask;
                                div_stack[current_warp.div_stack_ptr].exec_mask <= 
                                    ~branch_mask & current_warp.exec_mask;
                                current_warp.div_stack_ptr <= current_warp.div_stack_ptr + 1;
                                current_warp.exec_mask <= branch_mask & current_warp.exec_mask;
                                current_warp.diverged <= 1'b1;
                            end
                            current_warp.pc <= current_warp.pc + {{ADDR_WIDTH-32{immediate[31]}}, immediate};
                        end else begin
                            current_warp.pc <= current_warp.pc + 4;
                        end
                        stage <= STAGE_FETCH;
                    end
                end
            endcase
        end
    end
    
    //==========================================================================
    // Instruction Fetch
    //==========================================================================
    
    assign icache_req = (stage == STAGE_FETCH) && !icache_valid;
    assign icache_addr = current_warp.pc;
    
    //==========================================================================
    // Instruction Decode
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            decoded <= '0;
            rd <= '0;
            rs1 <= '0;
            rs2 <= '0;
            rs3 <= '0;
            immediate <= '0;
            is_vector_op <= 1'b0;
            is_memory_op <= 1'b0;
            is_texture_op <= 1'b0;
            is_branch_op <= 1'b0;
            is_special_op <= 1'b0;
        end else if (stage == STAGE_DECODE && instr_valid) begin
            // RISC-V style decode
            decoded.opcode <= instr[6:0];
            decoded.rd <= instr[11:7];
            decoded.funct3 <= instr[14:12];
            decoded.rs1 <= instr[19:15];
            decoded.rs2 <= instr[24:20];
            decoded.funct7 <= instr[31:25];
            
            rd <= instr[11:7];
            rs1 <= instr[19:15];
            rs2 <= instr[24:20];
            rs3 <= instr[31:27];  // For FMA operations
            
            // Immediate extraction (I-type)
            immediate <= {{20{instr[31]}}, instr[31:20]};
            
            // Operation type detection
            case (shader_opcode_t'(instr[6:0]))
                OP_VLOAD, OP_VSTORE: begin
                    is_memory_op <= 1'b1;
                    is_vector_op <= 1'b1;
                end
                OP_VARITH, OP_VFMA: begin
                    is_vector_op <= 1'b1;
                end
                OP_BRANCH: begin
                    is_branch_op <= 1'b1;
                end
                OP_SAMPLE: begin
                    is_texture_op <= 1'b1;
                    is_vector_op <= 1'b1;
                end
                OP_INTERP: begin
                    is_vector_op <= 1'b1;
                end
                OP_SPECIAL: begin
                    is_special_op <= 1'b1;
                    is_vector_op <= 1'b1;
                end
                default: begin
                    is_vector_op <= 1'b0;
                end
            endcase
        end
    end
    
    //==========================================================================
    // Operand Fetch
    //==========================================================================
    
    assign vrf_rd_en = (stage == STAGE_OPERAND) && is_vector_op;
    assign vrf_rd_addr = {current_warp.id, rs1};
    
    assign srf_rd_en = (stage == STAGE_OPERAND) && !is_vector_op;
    assign srf_rd_addr = {3'b0, rs1};
    
    // Latch operands
    always_ff @(posedge clk) begin
        if (stage == STAGE_OPERAND) begin
            // Unpack vector data to lanes
            for (int i = 0; i < 32; i++) begin
                src1_data[i] <= vrf_rd_data[i*DATA_WIDTH +: DATA_WIDTH];
            end
            scalar_data <= srf_rd_data;
        end
    end
    
    //==========================================================================
    // Vector ALU - 32 Parallel Lanes
    //==========================================================================
    
    genvar lane;
    generate
        for (lane = 0; lane < 32; lane++) begin : alu_lanes
            krypton_alu #(
                .LANE_ID(lane)
            ) u_alu (
                .clk        (clk),
                .rst_n      (rst_n),
                .enable     (current_warp.exec_mask[lane] && stage == STAGE_EXECUTE),
                .opcode     (decoded.opcode),
                .funct3     (decoded.funct3),
                .funct7     (decoded.funct7),
                .operand_a  (src1_data[lane]),
                .operand_b  (src2_data[lane]),
                .operand_c  (src3_data[lane]),
                .result     (exec_result[lane]),
                .done       (/* per-lane done */)
            );
        end
    endgenerate
    
    // Execution complete when all active lanes done
    assign exec_done = (stage == STAGE_EXECUTE) && !is_memory_op && !is_texture_op;
    assign exec_mask = current_warp.exec_mask;
    
    //==========================================================================
    // Branch Evaluation
    //==========================================================================
    
    always_comb begin
        branch_mask = '0;
        branch_taken = 1'b0;
        branch_divergent = 1'b0;
        
        if (is_branch_op && stage == STAGE_EXECUTE) begin
            // Evaluate branch condition per lane
            for (int i = 0; i < 32; i++) begin
                if (current_warp.exec_mask[i]) begin
                    case (decoded.funct3)
                        3'b000: branch_mask[i] = (src1_data[i] == src2_data[i]); // BEQ
                        3'b001: branch_mask[i] = (src1_data[i] != src2_data[i]); // BNE
                        3'b100: branch_mask[i] = ($signed(src1_data[i]) < $signed(src2_data[i])); // BLT
                        3'b101: branch_mask[i] = ($signed(src1_data[i]) >= $signed(src2_data[i])); // BGE
                        3'b110: branch_mask[i] = (src1_data[i] < src2_data[i]); // BLTU
                        3'b111: branch_mask[i] = (src1_data[i] >= src2_data[i]); // BGEU
                        default: branch_mask[i] = 1'b0;
                    endcase
                end
            end
            
            // Check for divergence
            branch_taken = |branch_mask;
            branch_divergent = (branch_mask != current_warp.exec_mask) && 
                              (branch_mask != '0);
        end
    end
    
    //==========================================================================
    // Memory Interface (Load/Store Unit)
    //==========================================================================
    
    assign lsu_req = (stage == STAGE_MEMORY) && is_memory_op && !lsu_valid;
    assign lsu_write = (decoded.opcode == OP_VSTORE);
    assign lsu_mask = current_warp.exec_mask;
    
    // Calculate addresses per lane
    always_comb begin
        for (int i = 0; i < 32; i++) begin
            lsu_addr[i] = src1_data[i][ADDR_WIDTH-1:0] + {{ADDR_WIDTH-32{immediate[31]}}, immediate};
            lsu_wdata[i] = src2_data[i];
        end
    end
    
    //==========================================================================
    // Texture Interface
    //==========================================================================
    
    assign tex_req = (stage == STAGE_MEMORY) && is_texture_op && !tex_valid;
    assign tex_unit = decoded.funct7[3:0];
    assign tex_mask = current_warp.exec_mask;
    
    // Pack texture coordinates
    always_comb begin
        for (int i = 0; i < 32; i++) begin
            tex_coord[i].x = src1_data[i];
            tex_coord[i].y = src2_data[i];
        end
    end
    
    //==========================================================================
    // Writeback
    //==========================================================================
    
    assign vrf_wr_en = (stage == STAGE_WRITEBACK) && is_vector_op && (rd != '0);
    assign vrf_wr_addr = {current_warp.id, rd};
    assign vrf_wr_mask = current_warp.exec_mask;
    
    // Pack results to write data
    always_comb begin
        for (int i = 0; i < 32; i++) begin
            if (is_memory_op && !lsu_write) begin
                vrf_wr_data[i*DATA_WIDTH +: DATA_WIDTH] = lsu_rdata[i];
            end else if (is_texture_op) begin
                vrf_wr_data[i*DATA_WIDTH +: DATA_WIDTH] = {tex_data[i].a, tex_data[i].b, 
                                                          tex_data[i].g, tex_data[i].r};
            end else begin
                vrf_wr_data[i*DATA_WIDTH +: DATA_WIDTH] = exec_result[i];
            end
        end
    end
    
    assign srf_wr_en = (stage == STAGE_WRITEBACK) && !is_vector_op && (rd != '0);
    assign srf_wr_addr = {3'b0, rd};
    assign srf_wr_data = exec_result[0];  // Lane 0 for scalar ops
    
    //==========================================================================
    // Warp Ready Signal
    //==========================================================================
    
    assign warp_ready = (stage == STAGE_IDLE);

endmodule : krypton_core

//==============================================================================
// Krypton ALU - Single Lane Execution Unit
//==============================================================================

module krypton_alu
    import krypton_pkg::*;
#(
    parameter int LANE_ID = 0
)(
    input  logic                    clk,
    input  logic                    rst_n,
    input  logic                    enable,
    
    input  logic [6:0]              opcode,
    input  logic [2:0]              funct3,
    input  logic [6:0]              funct7,
    
    input  logic [DATA_WIDTH-1:0]   operand_a,
    input  logic [DATA_WIDTH-1:0]   operand_b,
    input  logic [DATA_WIDTH-1:0]   operand_c,
    
    output logic [DATA_WIDTH-1:0]   result,
    output logic                    done
);

    // FP32 components (simplified - real implementation would use proper FPU)
    logic        sign_a, sign_b, sign_c;
    logic [7:0]  exp_a, exp_b, exp_c;
    logic [22:0] mant_a, mant_b, mant_c;
    
    // Integer/bitwise results
    logic [DATA_WIDTH-1:0] int_result;
    logic [DATA_WIDTH-1:0] fp_result;
    
    // Decompose FP operands
    assign sign_a = operand_a[31];
    assign exp_a  = operand_a[30:23];
    assign mant_a = operand_a[22:0];
    
    assign sign_b = operand_b[31];
    assign exp_b  = operand_b[30:23];
    assign mant_b = operand_b[22:0];
    
    assign sign_c = operand_c[31];
    assign exp_c  = operand_c[30:23];
    assign mant_c = operand_c[22:0];
    
    // Combinational ALU
    always_comb begin
        int_result = '0;
        fp_result = '0;
        
        case (shader_opcode_t'(opcode))
            OP_VARITH: begin
                case (funct3)
                    3'b000: begin  // VADD / VSUB
                        if (funct7[5])
                            int_result = operand_a - operand_b;
                        else
                            int_result = operand_a + operand_b;
                    end
                    3'b001: int_result = operand_a << operand_b[4:0];  // VSLL
                    3'b010: int_result = ($signed(operand_a) < $signed(operand_b)) ? 32'd1 : 32'd0; // VSLT
                    3'b011: int_result = (operand_a < operand_b) ? 32'd1 : 32'd0; // VSLTU
                    3'b100: int_result = operand_a ^ operand_b;  // VXOR
                    3'b101: begin  // VSRL / VSRA
                        if (funct7[5])
                            int_result = $signed(operand_a) >>> operand_b[4:0];
                        else
                            int_result = operand_a >> operand_b[4:0];
                    end
                    3'b110: int_result = operand_a | operand_b;  // VOR
                    3'b111: int_result = operand_a & operand_b;  // VAND
                endcase
            end
            
            OP_VFMA: begin
                // Fused multiply-add: a * b + c
                // Simplified - real implementation would be proper FP
                fp_result = operand_a; // Placeholder
            end
            
            OP_INTERP: begin
                // Barycentric interpolation
                // result = a * w0 + b * w1 + c * w2
                fp_result = operand_a; // Placeholder
            end
            
            OP_SPECIAL: begin
                case (funct7[2:0])
                    3'b000: fp_result = operand_a;  // RCP (reciprocal)
                    3'b001: fp_result = operand_a;  // RSQ (reciprocal sqrt)
                    3'b010: fp_result = operand_a;  // LOG2
                    3'b011: fp_result = operand_a;  // EXP2
                    3'b100: fp_result = operand_a;  // SIN
                    3'b101: fp_result = operand_a;  // COS
                    default: fp_result = operand_a;
                endcase
            end
            
            default: int_result = '0;
        endcase
    end
    
    // Select result based on operation type
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            result <= '0;
            done <= 1'b0;
        end else if (enable) begin
            case (shader_opcode_t'(opcode))
                OP_VFMA, OP_INTERP, OP_SPECIAL:
                    result <= fp_result;
                default:
                    result <= int_result;
            endcase
            done <= 1'b1;
        end else begin
            done <= 1'b0;
        end
    end

endmodule : krypton_alu
