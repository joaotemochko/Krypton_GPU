//==============================================================================
// GPU Krypton Series-1 - Integrated Execution Unit
// Combines: Integer ALU + IEEE 754 FPU + Special Function Unit
// Replaces placeholder krypton_alu with functional implementation
//==============================================================================

module krypton_execution_unit
    import krypton_pkg::*;
#(
    parameter int LANE_ID = 0
)(
    input  logic                    clk,
    input  logic                    rst_n,
    
    // Control
    input  logic                    enable,
    input  logic [6:0]              opcode,
    input  logic [2:0]              funct3,
    input  logic [6:0]              funct7,
    
    // Operands
    input  logic [DATA_WIDTH-1:0]   operand_a,
    input  logic [DATA_WIDTH-1:0]   operand_b,
    input  logic [DATA_WIDTH-1:0]   operand_c,  // For FMA
    
    // Result
    output logic [DATA_WIDTH-1:0]   result,
    output logic                    result_valid,
    output logic                    busy,
    
    // Exception flags
    output logic                    exc_invalid,
    output logic                    exc_divzero,
    output logic                    exc_overflow,
    output logic                    exc_underflow,
    output logic                    exc_inexact
);

    //==========================================================================
    // Operation Classification
    //==========================================================================
    
    logic is_int_op;
    logic is_fp_simple;    // FADD, FSUB, FMUL
    logic is_fma_op;       // FMA variants
    logic is_special_op;   // RCP, RSQ, SIN, COS, LOG2, EXP2
    logic is_interp_op;    // Barycentric interpolation
    
    always_comb begin
        is_int_op     = (shader_opcode_t'(opcode) == OP_VARITH);
        is_fp_simple  = (shader_opcode_t'(opcode) == OP_VARITH) && funct7[0];  // FP bit
        is_fma_op     = (shader_opcode_t'(opcode) == OP_VFMA);
        is_special_op = (shader_opcode_t'(opcode) == OP_SPECIAL);
        is_interp_op  = (shader_opcode_t'(opcode) == OP_INTERP);
    end
    
    //==========================================================================
    // Integer ALU (1 cycle)
    //==========================================================================
    
    logic [DATA_WIDTH-1:0] int_result;
    logic int_valid;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            int_result <= '0;
            int_valid <= 1'b0;
        end else if (enable && is_int_op && !funct7[0]) begin
            int_valid <= 1'b1;
            case (funct3)
                3'b000: begin  // ADD/SUB
                    if (funct7[5])
                        int_result <= operand_a - operand_b;
                    else
                        int_result <= operand_a + operand_b;
                end
                3'b001: int_result <= operand_a << operand_b[4:0];  // SLL
                3'b010: int_result <= ($signed(operand_a) < $signed(operand_b)) ? 32'd1 : 32'd0;  // SLT
                3'b011: int_result <= (operand_a < operand_b) ? 32'd1 : 32'd0;  // SLTU
                3'b100: int_result <= operand_a ^ operand_b;  // XOR
                3'b101: begin  // SRL/SRA
                    if (funct7[5])
                        int_result <= $signed(operand_a) >>> operand_b[4:0];
                    else
                        int_result <= operand_a >> operand_b[4:0];
                end
                3'b110: int_result <= operand_a | operand_b;  // OR
                3'b111: int_result <= operand_a & operand_b;  // AND
            endcase
        end else begin
            int_valid <= 1'b0;
        end
    end
    
    //==========================================================================
    // FPU Instance (4-5 cycles)
    //==========================================================================
    
    logic [3:0]  fpu_op;
    logic        fpu_start;
    logic [31:0] fpu_result;
    logic        fpu_valid;
    logic        fpu_ready;
    
    // Map shader opcodes to FPU operations
    always_comb begin
        fpu_op = 4'h0;  // Default: ADD
        fpu_start = 1'b0;
        
        if (enable) begin
            if (is_fp_simple) begin
                fpu_start = 1'b1;
                case (funct3)
                    3'b000: fpu_op = funct7[5] ? 4'h1 : 4'h0;  // FSUB/FADD
                    3'b001: fpu_op = 4'h2;  // FMUL
                    3'b010: fpu_op = 4'h7;  // FMIN
                    3'b011: fpu_op = 4'h8;  // FMAX
                    3'b100: fpu_op = 4'h9;  // FEQ
                    3'b101: fpu_op = 4'hA;  // FLT
                    3'b110: fpu_op = 4'hB;  // FLE
                    default: fpu_op = 4'h0;
                endcase
            end else if (is_fma_op) begin
                fpu_start = 1'b1;
                case (funct3[1:0])
                    2'b00: fpu_op = 4'h3;  // FMA
                    2'b01: fpu_op = 4'h4;  // FMSUB
                    2'b10: fpu_op = 4'h5;  // FNMADD
                    2'b11: fpu_op = 4'h6;  // FNMSUB
                endcase
            end
        end
    end
    
    krypton_fpu_ieee754 u_fpu (
        .clk            (clk),
        .rst_n          (rst_n),
        .op             (fpu_op),
        .rounding       (2'b00),        // RNE (Round to Nearest Even)
        .a              (operand_a),
        .b              (operand_b),
        .c              (operand_c),
        .valid_in       (fpu_start),
        .ready          (fpu_ready),
        .result         (fpu_result),
        .valid_out      (fpu_valid),
        .flag_inexact   (exc_inexact),
        .flag_underflow (exc_underflow),
        .flag_overflow  (exc_overflow),
        .flag_div_by_zero(exc_divzero),
        .flag_invalid   (exc_invalid)
    );
    
    //==========================================================================
    // SFU Instance (4-6 cycles)
    //==========================================================================
    
    logic [2:0]  sfu_op;
    logic        sfu_start;
    logic [31:0] sfu_result;
    logic        sfu_valid;
    logic        sfu_error;
    
    // Map funct7 to SFU operations
    always_comb begin
        sfu_op = funct7[2:0];
        sfu_start = enable && is_special_op;
    end
    
    krypton_sfu u_sfu (
        .clk        (clk),
        .rst_n      (rst_n),
        .op         (sfu_op),
        .operand    (operand_a),
        .valid_in   (sfu_start),
        .result     (sfu_result),
        .valid_out  (sfu_valid),
        .error_flag (sfu_error)
    );
    
    //==========================================================================
    // Interpolator (3 cycles)
    //==========================================================================
    
    logic        interp_start;
    logic [31:0] interp_result;
    logic        interp_valid;
    
    // Simple perspective-correct interpolation
    // result = (a * w0 + b * w1 + c * w2) / (w0 + w1 + w2)
    // For now, simplified linear interpolation
    
    // Pipeline registers for interpolation
    logic [63:0] interp_mult0, interp_mult1, interp_mult2;
    logic [63:0] interp_sum;
    logic [1:0]  interp_stage;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            interp_stage <= '0;
            interp_valid <= 1'b0;
        end else begin
            interp_valid <= 1'b0;
            
            if (enable && is_interp_op) begin
                interp_stage <= 2'd1;
                // Stage 1: Multiply (simplified - uses integer mult)
                // In real implementation, would use FP mults
                interp_mult0 <= operand_a * operand_c[31:16];  // a * w0
                interp_mult1 <= operand_b * operand_c[15:0];   // b * w1
            end else if (interp_stage == 2'd1) begin
                interp_stage <= 2'd2;
                // Stage 2: Sum
                interp_sum <= interp_mult0 + interp_mult1;
            end else if (interp_stage == 2'd2) begin
                interp_stage <= 2'd0;
                // Stage 3: Normalize (simplified)
                interp_result <= interp_sum[47:16];
                interp_valid <= 1'b1;
            end
        end
    end
    
    //==========================================================================
    // Result Multiplexing
    //==========================================================================
    
    // Track which unit is active
    logic [2:0] active_unit;  // 0=none, 1=int, 2=fpu, 3=sfu, 4=interp
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active_unit <= 3'd0;
        end else if (enable) begin
            if (is_int_op && !funct7[0])
                active_unit <= 3'd1;
            else if (is_fp_simple || is_fma_op)
                active_unit <= 3'd2;
            else if (is_special_op)
                active_unit <= 3'd3;
            else if (is_interp_op)
                active_unit <= 3'd4;
        end else if (result_valid) begin
            active_unit <= 3'd0;
        end
    end
    
    // Result selection
    always_comb begin
        result = '0;
        result_valid = 1'b0;
        
        case (active_unit)
            3'd1: begin
                result = int_result;
                result_valid = int_valid;
            end
            3'd2: begin
                result = fpu_result;
                result_valid = fpu_valid;
            end
            3'd3: begin
                result = sfu_result;
                result_valid = sfu_valid;
            end
            3'd4: begin
                result = interp_result;
                result_valid = interp_valid;
            end
            default: begin
                result = '0;
                result_valid = 1'b0;
            end
        endcase
    end
    
    // Busy signal
    assign busy = (active_unit != 3'd0) && !result_valid;

endmodule : krypton_execution_unit


//==============================================================================
// Scoreboard for Hazard Detection
// Tracks register dependencies and stalls on RAW hazards
//==============================================================================

module krypton_scoreboard
#(
    parameter int NUM_REGS = 32,
    parameter int NUM_PENDING = 8
)(
    input  logic                    clk,
    input  logic                    rst_n,
    
    // Issue interface
    input  logic                    issue_valid,
    input  logic [4:0]              issue_rd,       // Destination register
    input  logic [4:0]              issue_rs1,      // Source register 1
    input  logic [4:0]              issue_rs2,      // Source register 2
    input  logic [4:0]              issue_rs3,      // Source register 3 (for FMA)
    input  logic [2:0]              issue_latency,  // Expected latency (cycles)
    output logic                    issue_stall,    // Stall due to hazard
    
    // Writeback interface
    input  logic                    wb_valid,
    input  logic [4:0]              wb_rd,
    
    // Flush
    input  logic                    flush
);

    // Pending write tracking
    typedef struct packed {
        logic        valid;
        logic [4:0]  rd;
        logic [2:0]  cycles_remaining;
    } pending_entry_t;
    
    pending_entry_t pending [0:NUM_PENDING-1];
    
    // Check for RAW hazards
    logic rs1_hazard, rs2_hazard, rs3_hazard;
    
    always_comb begin
        rs1_hazard = 1'b0;
        rs2_hazard = 1'b0;
        rs3_hazard = 1'b0;
        
        for (int i = 0; i < NUM_PENDING; i++) begin
            if (pending[i].valid) begin
                if (pending[i].rd == issue_rs1 && issue_rs1 != 5'd0)
                    rs1_hazard = 1'b1;
                if (pending[i].rd == issue_rs2 && issue_rs2 != 5'd0)
                    rs2_hazard = 1'b1;
                if (pending[i].rd == issue_rs3 && issue_rs3 != 5'd0)
                    rs3_hazard = 1'b1;
            end
        end
    end
    
    assign issue_stall = issue_valid && (rs1_hazard || rs2_hazard || rs3_hazard);
    
    // Pending entry management
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || flush) begin
            for (int i = 0; i < NUM_PENDING; i++)
                pending[i] <= '0;
        end else begin
            // Decrement cycle counters and clear completed
            for (int i = 0; i < NUM_PENDING; i++) begin
                if (pending[i].valid) begin
                    if (pending[i].cycles_remaining == 3'd0)
                        pending[i].valid <= 1'b0;
                    else
                        pending[i].cycles_remaining <= pending[i].cycles_remaining - 1;
                end
            end
            
            // Clear on explicit writeback
            if (wb_valid) begin
                for (int i = 0; i < NUM_PENDING; i++) begin
                    if (pending[i].valid && pending[i].rd == wb_rd)
                        pending[i].valid <= 1'b0;
                end
            end
            
            // Add new pending write (if not stalled)
            if (issue_valid && !issue_stall && issue_rd != 5'd0) begin
                for (int i = 0; i < NUM_PENDING; i++) begin
                    if (!pending[i].valid) begin
                        pending[i].valid <= 1'b1;
                        pending[i].rd <= issue_rd;
                        pending[i].cycles_remaining <= issue_latency;
                        break;
                    end
                end
            end
        end
    end

endmodule : krypton_scoreboard


//==============================================================================
// Barrier/Sync Unit for Workgroup Synchronization
//==============================================================================

module krypton_barrier_unit
#(
    parameter int MAX_WARPS = 32
)(
    input  logic                    clk,
    input  logic                    rst_n,
    
    // Barrier request from warps
    input  logic [MAX_WARPS-1:0]    barrier_req,
    input  logic [3:0]              barrier_id,     // Which barrier (0-15)
    input  logic [MAX_WARPS-1:0]    warp_active,    // Which warps are in workgroup
    
    // Barrier release
    output logic [MAX_WARPS-1:0]    barrier_release,
    output logic                    barrier_complete,
    
    // Memory fence
    input  logic [MAX_WARPS-1:0]    fence_req,
    output logic [MAX_WARPS-1:0]    fence_done,
    input  logic                    memory_idle     // From LSU - all writes committed
);

    // Barrier state per barrier_id
    logic [MAX_WARPS-1:0] barrier_arrived [0:15];
    
    // Check if all active warps have arrived
    logic all_arrived;
    assign all_arrived = (barrier_arrived[barrier_id] & warp_active) == warp_active;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 16; i++)
                barrier_arrived[i] <= '0;
            barrier_release <= '0;
            barrier_complete <= 1'b0;
            fence_done <= '0;
        end else begin
            barrier_release <= '0;
            barrier_complete <= 1'b0;
            fence_done <= '0;
            
            // Register barrier arrivals
            barrier_arrived[barrier_id] <= barrier_arrived[barrier_id] | barrier_req;
            
            // Release when all arrived
            if (all_arrived && |barrier_arrived[barrier_id]) begin
                barrier_release <= warp_active;
                barrier_complete <= 1'b1;
                barrier_arrived[barrier_id] <= '0;
            end
            
            // Memory fence - wait for memory idle
            if (|fence_req && memory_idle) begin
                fence_done <= fence_req;
            end
        end
    end

endmodule : krypton_barrier_unit
