//==============================================================================
// GPU Krypton Series-1 - Scoreboard and Hazard Detection Unit
// Detects and handles: RAW (Read After Write), WAW (Write After Write)
// Supports: Multi-cycle operations (FPU, SFU, TMU, LSU)
//==============================================================================

module krypton_scoreboard
    import krypton_pkg::*;
#(
    parameter int NUM_VGPR          = 256,      // Vector GPRs per warp
    parameter int NUM_SGPR          = 64,       // Scalar GPRs per warp
    parameter int MAX_PENDING_OPS   = 16,       // Max in-flight operations
    parameter int WARP_ID_WIDTH     = 5,
    parameter int REG_ADDR_WIDTH    = 8
)(
    input  logic                        clk,
    input  logic                        rst_n,
    
    //==========================================================================
    // Instruction Issue Interface
    //==========================================================================
    
    input  logic                        issue_valid,
    output logic                        issue_stall,
    output logic [2:0]                  stall_reason,
    
    input  logic [WARP_ID_WIDTH-1:0]    issue_warp_id,
    input  logic [6:0]                  issue_opcode,
    input  logic [REG_ADDR_WIDTH-1:0]   issue_rd,
    input  logic [REG_ADDR_WIDTH-1:0]   issue_rs1,
    input  logic [REG_ADDR_WIDTH-1:0]   issue_rs2,
    input  logic [REG_ADDR_WIDTH-1:0]   issue_rs3,
    input  logic                        issue_rd_valid,
    input  logic                        issue_rs1_valid,
    input  logic                        issue_rs2_valid,
    input  logic                        issue_rs3_valid,
    input  logic                        issue_is_vector,
    input  logic [3:0]                  issue_latency,
    input  logic [2:0]                  issue_exec_unit,
    
    //==========================================================================
    // Writeback Interface
    //==========================================================================
    
    input  logic                        wb_valid,
    input  logic [WARP_ID_WIDTH-1:0]    wb_warp_id,
    input  logic [REG_ADDR_WIDTH-1:0]   wb_rd,
    input  logic                        wb_is_vector,
    
    //==========================================================================
    // Execution Unit Status
    //==========================================================================
    
    input  logic                        alu_busy,
    input  logic                        fpu_busy,
    input  logic                        sfu_busy,
    input  logic                        lsu_busy,
    input  logic                        tmu_busy,
    
    //==========================================================================
    // Warp Control
    //==========================================================================
    
    input  logic                        warp_switch,
    input  logic [WARP_ID_WIDTH-1:0]    new_warp_id,
    input  logic                        flush,
    
    //==========================================================================
    // Interlock Signals
    //==========================================================================
    
    output logic                        interlock_active,
    output logic [MAX_PENDING_OPS-1:0]  pending_ops_bitmap,
    
    //==========================================================================
    // Performance Counters
    //==========================================================================
    
    output logic [31:0]                 raw_hazards_count,
    output logic [31:0]                 waw_hazards_count,
    output logic [31:0]                 structural_hazards_count
);

    //==========================================================================
    // Stall Reason Codes
    //==========================================================================
    
    typedef enum logic [2:0] {
        STALL_NONE          = 3'd0,
        STALL_RAW_RS1       = 3'd1,
        STALL_RAW_RS2       = 3'd2,
        STALL_RAW_RS3       = 3'd3,
        STALL_WAW           = 3'd4,
        STALL_STRUCTURAL    = 3'd5,
        STALL_QUEUE_FULL    = 3'd6
    } stall_reason_t;
    
    typedef enum logic [2:0] {
        EXEC_ALU    = 3'd0,
        EXEC_FPU    = 3'd1,
        EXEC_SFU    = 3'd2,
        EXEC_LSU    = 3'd3,
        EXEC_TMU    = 3'd4,
        EXEC_BRANCH = 3'd5
    } exec_unit_t;
    
    //==========================================================================
    // Pending Operation Entry
    //==========================================================================
    
    typedef struct packed {
        logic                           valid;
        logic [WARP_ID_WIDTH-1:0]       warp_id;
        logic [REG_ADDR_WIDTH-1:0]      rd;
        logic                           is_vector;
        logic [3:0]                     cycles_remaining;
        logic [2:0]                     exec_unit;
    } pending_op_t;
    
    pending_op_t pending_ops [0:MAX_PENDING_OPS-1];
    
    //==========================================================================
    // Register Dependency Tracking
    //==========================================================================
    
    logic [NUM_VGPR-1:0] vgpr_pending [0:31];
    logic [NUM_SGPR-1:0] sgpr_pending [0:31];
    
    //==========================================================================
    // Hazard Detection Logic
    //==========================================================================
    
    logic raw_hazard_rs1, raw_hazard_rs2, raw_hazard_rs3;
    logic waw_hazard;
    logic structural_hazard;
    logic queue_full;
    
    // Find free slot
    logic [$clog2(MAX_PENDING_OPS)-1:0] free_slot;
    logic free_slot_valid;
    
    always_comb begin
        free_slot_valid = 1'b0;
        free_slot = '0;
        
        for (int i = 0; i < MAX_PENDING_OPS; i++) begin
            if (!pending_ops[i].valid && !free_slot_valid) begin
                free_slot_valid = 1'b1;
                free_slot = i[$clog2(MAX_PENDING_OPS)-1:0];
            end
        end
    end
    
    assign queue_full = !free_slot_valid;
    
    // RAW hazard detection
    always_comb begin
        raw_hazard_rs1 = 1'b0;
        raw_hazard_rs2 = 1'b0;
        raw_hazard_rs3 = 1'b0;
        
        if (issue_valid) begin
            if (issue_rs1_valid) begin
                if (issue_is_vector)
                    raw_hazard_rs1 = vgpr_pending[issue_warp_id][issue_rs1];
                else
                    raw_hazard_rs1 = sgpr_pending[issue_warp_id][issue_rs1];
            end
            
            if (issue_rs2_valid) begin
                if (issue_is_vector)
                    raw_hazard_rs2 = vgpr_pending[issue_warp_id][issue_rs2];
                else
                    raw_hazard_rs2 = sgpr_pending[issue_warp_id][issue_rs2];
            end
            
            if (issue_rs3_valid) begin
                if (issue_is_vector)
                    raw_hazard_rs3 = vgpr_pending[issue_warp_id][issue_rs3];
                else
                    raw_hazard_rs3 = sgpr_pending[issue_warp_id][issue_rs3];
            end
        end
    end
    
    // WAW hazard detection
    always_comb begin
        waw_hazard = 1'b0;
        
        if (issue_valid && issue_rd_valid) begin
            if (issue_is_vector)
                waw_hazard = vgpr_pending[issue_warp_id][issue_rd];
            else
                waw_hazard = sgpr_pending[issue_warp_id][issue_rd];
        end
    end
    
    // Structural hazard detection
    always_comb begin
        structural_hazard = 1'b0;
        
        if (issue_valid) begin
            case (exec_unit_t'(issue_exec_unit))
                EXEC_ALU: structural_hazard = alu_busy;
                EXEC_FPU: structural_hazard = fpu_busy;
                EXEC_SFU: structural_hazard = sfu_busy;
                EXEC_LSU: structural_hazard = lsu_busy;
                EXEC_TMU: structural_hazard = tmu_busy;
                default:  structural_hazard = 1'b0;
            endcase
        end
    end
    
    // Combined stall signal
    assign issue_stall = raw_hazard_rs1 || raw_hazard_rs2 || raw_hazard_rs3 ||
                        waw_hazard || structural_hazard || queue_full;
    
    // Stall reason encoding
    always_comb begin
        if (raw_hazard_rs1)
            stall_reason = STALL_RAW_RS1;
        else if (raw_hazard_rs2)
            stall_reason = STALL_RAW_RS2;
        else if (raw_hazard_rs3)
            stall_reason = STALL_RAW_RS3;
        else if (waw_hazard)
            stall_reason = STALL_WAW;
        else if (structural_hazard)
            stall_reason = STALL_STRUCTURAL;
        else if (queue_full)
            stall_reason = STALL_QUEUE_FULL;
        else
            stall_reason = STALL_NONE;
    end
    
    //==========================================================================
    // Pending Operations Management
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < MAX_PENDING_OPS; i++)
                pending_ops[i] <= '0;
            
            for (int w = 0; w < 32; w++) begin
                vgpr_pending[w] <= '0;
                sgpr_pending[w] <= '0;
            end
            
            raw_hazards_count <= '0;
            waw_hazards_count <= '0;
            structural_hazards_count <= '0;
        end else if (flush) begin
            for (int i = 0; i < MAX_PENDING_OPS; i++)
                pending_ops[i].valid <= 1'b0;
            
            for (int w = 0; w < 32; w++) begin
                vgpr_pending[w] <= '0;
                sgpr_pending[w] <= '0;
            end
        end else begin
            // Decrement cycle counters
            for (int i = 0; i < MAX_PENDING_OPS; i++) begin
                if (pending_ops[i].valid) begin
                    if (pending_ops[i].cycles_remaining == 4'd0) begin
                        pending_ops[i].valid <= 1'b0;
                        
                        if (pending_ops[i].is_vector)
                            vgpr_pending[pending_ops[i].warp_id][pending_ops[i].rd] <= 1'b0;
                        else
                            sgpr_pending[pending_ops[i].warp_id][pending_ops[i].rd] <= 1'b0;
                    end else begin
                        pending_ops[i].cycles_remaining <= pending_ops[i].cycles_remaining - 1;
                    end
                end
            end
            
            // Handle writeback
            if (wb_valid) begin
                for (int i = 0; i < MAX_PENDING_OPS; i++) begin
                    if (pending_ops[i].valid &&
                        pending_ops[i].warp_id == wb_warp_id &&
                        pending_ops[i].rd == wb_rd &&
                        pending_ops[i].is_vector == wb_is_vector) begin
                        pending_ops[i].valid <= 1'b0;
                        
                        if (wb_is_vector)
                            vgpr_pending[wb_warp_id][wb_rd] <= 1'b0;
                        else
                            sgpr_pending[wb_warp_id][wb_rd] <= 1'b0;
                    end
                end
            end
            
            // Add new pending operation
            if (issue_valid && !issue_stall && issue_rd_valid) begin
                pending_ops[free_slot].valid <= 1'b1;
                pending_ops[free_slot].warp_id <= issue_warp_id;
                pending_ops[free_slot].rd <= issue_rd;
                pending_ops[free_slot].is_vector <= issue_is_vector;
                pending_ops[free_slot].cycles_remaining <= issue_latency;
                pending_ops[free_slot].exec_unit <= issue_exec_unit;
                
                if (issue_is_vector)
                    vgpr_pending[issue_warp_id][issue_rd] <= 1'b1;
                else
                    sgpr_pending[issue_warp_id][issue_rd] <= 1'b1;
            end
            
            // Update counters
            if (issue_valid) begin
                if (raw_hazard_rs1 || raw_hazard_rs2 || raw_hazard_rs3)
                    raw_hazards_count <= raw_hazards_count + 1;
                if (waw_hazard)
                    waw_hazards_count <= waw_hazards_count + 1;
                if (structural_hazard)
                    structural_hazards_count <= structural_hazards_count + 1;
            end
        end
    end
    
    assign interlock_active = |{pending_ops[0].valid, pending_ops[1].valid,
                               pending_ops[2].valid, pending_ops[3].valid};
    
    always_comb begin
        for (int i = 0; i < MAX_PENDING_OPS; i++)
            pending_ops_bitmap[i] = pending_ops[i].valid;
    end

endmodule : krypton_scoreboard
