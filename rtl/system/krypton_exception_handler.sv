//==============================================================================
// GPU Krypton Series-1 - Exception Handler Unit
// Handles: Page faults, illegal instructions, AXI errors, arithmetic exceptions
// Provides: irq_error signal, fault registers, gpu_state machine
//==============================================================================

module krypton_exception_handler
    import krypton_pkg::*;
#(
    parameter int NUM_CLUSTERS      = 4,
    parameter int EXCEPTION_QUEUE   = 8
)(
    input  logic                        clk,
    input  logic                        rst_n,
    
    //==========================================================================
    // Exception Sources
    //==========================================================================
    
    // MMU/GMMU Page Faults
    input  logic                        gmmu_fault_valid,
    input  logic [ADDR_WIDTH-1:0]       gmmu_fault_vaddr,
    input  logic [ADDR_WIDTH-1:0]       gmmu_fault_ptw_addr,
    input  logic [3:0]                  gmmu_fault_context,
    input  logic [3:0]                  gmmu_fault_code,
    input  logic [2:0]                  gmmu_fault_source,
    
    // Shader Core Exceptions (per cluster)
    input  logic [NUM_CLUSTERS-1:0]     core_exception_valid,
    input  logic [3:0]                  core_exception_code [0:NUM_CLUSTERS-1],
    input  logic [ADDR_WIDTH-1:0]       core_exception_pc [0:NUM_CLUSTERS-1],
    input  logic [4:0]                  core_exception_warp [0:NUM_CLUSTERS-1],
    input  logic [4:0]                  core_exception_lane [0:NUM_CLUSTERS-1],
    
    // AXI Bus Errors
    input  logic                        axi_read_error,
    input  logic                        axi_write_error,
    input  logic [ADDR_WIDTH-1:0]       axi_error_addr,
    input  logic [1:0]                  axi_error_resp,     // AXI response code
    input  logic [7:0]                  axi_error_id,
    
    // Arithmetic Exceptions (from FPU/SFU)
    input  logic [NUM_CLUSTERS-1:0]     fpu_exception_valid,
    input  logic [4:0]                  fpu_exception_flags [0:NUM_CLUSTERS-1],  // IEEE flags
    input  logic [4:0]                  fpu_exception_warp [0:NUM_CLUSTERS-1],
    
    // Timeout/Watchdog
    input  logic                        watchdog_timeout,
    input  logic [1:0]                  timeout_cluster,
    input  logic [4:0]                  timeout_warp,
    
    //==========================================================================
    // Control Interface (from APB)
    //==========================================================================
    
    input  logic                        exception_enable,   // Global enable
    input  logic [31:0]                 exception_mask,     // Which exceptions generate IRQ
    input  logic                        exception_ack,      // CPU acknowledges exception
    input  logic                        exception_clear,    // Clear exception registers
    
    //==========================================================================
    // GPU State Control
    //==========================================================================
    
    output logic [3:0]                  gpu_state,          // Current GPU state
    output logic                        gpu_halted,         // GPU execution halted
    output logic                        gpu_error,          // Unrecoverable error
    
    // Halt control to clusters
    output logic [NUM_CLUSTERS-1:0]     cluster_halt,
    output logic [NUM_CLUSTERS-1:0]     cluster_resume,
    
    //==========================================================================
    // Interrupt Output
    //==========================================================================
    
    output logic                        irq_error,          // Error interrupt to CPU
    output logic                        irq_pending,        // Any exception pending
    
    //==========================================================================
    // Exception Registers (readable via APB)
    //==========================================================================
    
    output logic [31:0]                 exc_status,         // Exception status bitmap
    output logic [31:0]                 exc_cause,          // Primary exception cause
    output logic [ADDR_WIDTH-1:0]       exc_addr,           // Fault address
    output logic [ADDR_WIDTH-1:0]       exc_pc,             // Faulting PC
    output logic [7:0]                  exc_cluster,        // Faulting cluster
    output logic [7:0]                  exc_warp,           // Faulting warp
    output logic [7:0]                  exc_context,        // GPU context ID
    output logic [31:0]                 exc_info,           // Additional info
    
    //==========================================================================
    // Performance/Debug
    //==========================================================================
    
    output logic [31:0]                 total_exceptions,
    output logic [31:0]                 page_fault_count,
    output logic [31:0]                 illegal_instr_count,
    output logic [31:0]                 axi_error_count
);

    //==========================================================================
    // Exception Codes
    //==========================================================================
    
    typedef enum logic [5:0] {
        // No exception
        EXC_NONE                = 6'h00,
        
        // Memory exceptions (0x01-0x0F)
        EXC_PAGE_FAULT_READ     = 6'h01,
        EXC_PAGE_FAULT_WRITE    = 6'h02,
        EXC_PAGE_FAULT_EXEC     = 6'h03,
        EXC_ACCESS_VIOLATION    = 6'h04,
        EXC_MISALIGNED_ACCESS   = 6'h05,
        EXC_OUT_OF_BOUNDS       = 6'h06,
        
        // AXI bus errors (0x10-0x1F)
        EXC_AXI_SLVERR          = 6'h10,
        EXC_AXI_DECERR          = 6'h11,
        EXC_AXI_TIMEOUT         = 6'h12,
        
        // Instruction exceptions (0x20-0x2F)
        EXC_ILLEGAL_INSTR       = 6'h20,
        EXC_UNDEFINED_OPCODE    = 6'h21,
        EXC_PRIVILEGED_INSTR    = 6'h22,
        
        // Arithmetic exceptions (0x30-0x3F)
        EXC_FP_INVALID          = 6'h30,
        EXC_FP_DIVZERO          = 6'h31,
        EXC_FP_OVERFLOW         = 6'h32,
        EXC_FP_UNDERFLOW        = 6'h33,
        EXC_FP_INEXACT          = 6'h34,
        EXC_INT_DIVZERO         = 6'h35,
        EXC_INT_OVERFLOW        = 6'h36,
        
        // System exceptions (0x38-0x3F)
        EXC_WATCHDOG_TIMEOUT    = 6'h38,
        EXC_STACK_OVERFLOW      = 6'h39,
        EXC_CONTEXT_INVALID     = 6'h3A,
        EXC_INTERNAL_ERROR      = 6'h3F
    } exception_code_t;
    
    //==========================================================================
    // GPU State Machine
    //==========================================================================
    
    typedef enum logic [3:0] {
        GPU_STATE_RESET         = 4'h0,
        GPU_STATE_IDLE          = 4'h1,
        GPU_STATE_RUNNING       = 4'h2,
        GPU_STATE_PAUSED        = 4'h3,
        GPU_STATE_EXCEPTION     = 4'h4,
        GPU_STATE_HALTED        = 4'h5,
        GPU_STATE_RECOVERY      = 4'h6,
        GPU_STATE_FATAL         = 4'hF
    } gpu_state_t;
    
    gpu_state_t current_state, next_state;
    
    //==========================================================================
    // Exception Queue
    //==========================================================================
    
    typedef struct packed {
        logic                   valid;
        exception_code_t        code;
        logic [ADDR_WIDTH-1:0]  addr;
        logic [ADDR_WIDTH-1:0]  pc;
        logic [3:0]             cluster;
        logic [4:0]             warp;
        logic [4:0]             lane;
        logic [3:0]             context_id;
        logic [31:0]            info;
        logic [31:0]            timestamp;
    } exception_entry_t;
    
    exception_entry_t exc_queue [0:EXCEPTION_QUEUE-1];
    logic [$clog2(EXCEPTION_QUEUE)-1:0] exc_head, exc_tail;
    logic exc_queue_full, exc_queue_empty;
    
    assign exc_queue_full = ((exc_tail + 1) % EXCEPTION_QUEUE) == exc_head;
    assign exc_queue_empty = (exc_head == exc_tail);
    
    // Timestamp counter
    logic [31:0] timestamp;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            timestamp <= '0;
        else
            timestamp <= timestamp + 1;
    end
    
    //==========================================================================
    // Exception Priority Encoder
    //==========================================================================
    
    logic           new_exception;
    exception_code_t new_exc_code;
    logic [ADDR_WIDTH-1:0] new_exc_addr;
    logic [ADDR_WIDTH-1:0] new_exc_pc;
    logic [3:0]     new_exc_cluster;
    logic [4:0]     new_exc_warp;
    logic [4:0]     new_exc_lane;
    logic [3:0]     new_exc_context;
    logic [31:0]    new_exc_info;
    
    always_comb begin
        new_exception = 1'b0;
        new_exc_code = EXC_NONE;
        new_exc_addr = '0;
        new_exc_pc = '0;
        new_exc_cluster = '0;
        new_exc_warp = '0;
        new_exc_lane = '0;
        new_exc_context = '0;
        new_exc_info = '0;
        
        // Priority order: Page faults > AXI errors > Core exceptions > FPU > Timeout
        
        if (gmmu_fault_valid) begin
            new_exception = 1'b1;
            case (gmmu_fault_code)
                4'h1: new_exc_code = EXC_PAGE_FAULT_READ;
                4'h2: new_exc_code = EXC_PAGE_FAULT_WRITE;
                4'h3: new_exc_code = EXC_PAGE_FAULT_EXEC;
                4'h4: new_exc_code = EXC_ACCESS_VIOLATION;
                4'h5: new_exc_code = EXC_MISALIGNED_ACCESS;
                default: new_exc_code = EXC_PAGE_FAULT_READ;
            endcase
            new_exc_addr = gmmu_fault_vaddr;
            new_exc_context = gmmu_fault_context;
            new_exc_info = {16'b0, gmmu_fault_ptw_addr[15:0]};
        end
        else if (axi_read_error || axi_write_error) begin
            new_exception = 1'b1;
            case (axi_error_resp)
                2'b10: new_exc_code = EXC_AXI_SLVERR;
                2'b11: new_exc_code = EXC_AXI_DECERR;
                default: new_exc_code = EXC_AXI_SLVERR;
            endcase
            new_exc_addr = axi_error_addr;
            new_exc_info = {24'b0, axi_error_id};
        end
        else if (|core_exception_valid) begin
            new_exception = 1'b1;
            // Find first cluster with exception
            for (int i = 0; i < NUM_CLUSTERS; i++) begin
                if (core_exception_valid[i]) begin
                    new_exc_cluster = i[3:0];
                    new_exc_warp = core_exception_warp[i];
                    new_exc_lane = core_exception_lane[i];
                    new_exc_pc = core_exception_pc[i];
                    
                    case (core_exception_code[i])
                        4'h1: new_exc_code = EXC_ILLEGAL_INSTR;
                        4'h2: new_exc_code = EXC_UNDEFINED_OPCODE;
                        4'h3: new_exc_code = EXC_PRIVILEGED_INSTR;
                        4'h4: new_exc_code = EXC_INT_DIVZERO;
                        4'h5: new_exc_code = EXC_STACK_OVERFLOW;
                        default: new_exc_code = EXC_ILLEGAL_INSTR;
                    endcase
                    break;
                end
            end
        end
        else if (|fpu_exception_valid) begin
            // FPU exceptions (lower priority, often masked)
            for (int i = 0; i < NUM_CLUSTERS; i++) begin
                if (fpu_exception_valid[i]) begin
                    new_exception = 1'b1;
                    new_exc_cluster = i[3:0];
                    new_exc_warp = fpu_exception_warp[i];
                    new_exc_info = {27'b0, fpu_exception_flags[i]};
                    
                    // IEEE flags: [4]=invalid, [3]=divzero, [2]=overflow, [1]=underflow, [0]=inexact
                    if (fpu_exception_flags[i][4])
                        new_exc_code = EXC_FP_INVALID;
                    else if (fpu_exception_flags[i][3])
                        new_exc_code = EXC_FP_DIVZERO;
                    else if (fpu_exception_flags[i][2])
                        new_exc_code = EXC_FP_OVERFLOW;
                    else if (fpu_exception_flags[i][1])
                        new_exc_code = EXC_FP_UNDERFLOW;
                    else
                        new_exc_code = EXC_FP_INEXACT;
                    break;
                end
            end
        end
        else if (watchdog_timeout) begin
            new_exception = 1'b1;
            new_exc_code = EXC_WATCHDOG_TIMEOUT;
            new_exc_cluster = {2'b0, timeout_cluster};
            new_exc_warp = timeout_warp;
        end
    end
    
    //==========================================================================
    // Exception Queue Management
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            exc_head <= '0;
            exc_tail <= '0;
            for (int i = 0; i < EXCEPTION_QUEUE; i++)
                exc_queue[i] <= '0;
        end else if (exception_clear) begin
            exc_head <= '0;
            exc_tail <= '0;
            for (int i = 0; i < EXCEPTION_QUEUE; i++)
                exc_queue[i].valid <= 1'b0;
        end else begin
            // Enqueue new exception
            if (new_exception && exception_enable && !exc_queue_full) begin
                exc_queue[exc_tail].valid <= 1'b1;
                exc_queue[exc_tail].code <= new_exc_code;
                exc_queue[exc_tail].addr <= new_exc_addr;
                exc_queue[exc_tail].pc <= new_exc_pc;
                exc_queue[exc_tail].cluster <= new_exc_cluster;
                exc_queue[exc_tail].warp <= new_exc_warp;
                exc_queue[exc_tail].lane <= new_exc_lane;
                exc_queue[exc_tail].context_id <= new_exc_context;
                exc_queue[exc_tail].info <= new_exc_info;
                exc_queue[exc_tail].timestamp <= timestamp;
                exc_tail <= (exc_tail + 1) % EXCEPTION_QUEUE;
            end
            
            // Dequeue on acknowledge
            if (exception_ack && !exc_queue_empty) begin
                exc_queue[exc_head].valid <= 1'b0;
                exc_head <= (exc_head + 1) % EXCEPTION_QUEUE;
            end
        end
    end
    
    //==========================================================================
    // GPU State Machine
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= GPU_STATE_RESET;
        end else begin
            current_state <= next_state;
        end
    end
    
    always_comb begin
        next_state = current_state;
        cluster_halt = '0;
        cluster_resume = '0;
        
        case (current_state)
            GPU_STATE_RESET: begin
                next_state = GPU_STATE_IDLE;
            end
            
            GPU_STATE_IDLE: begin
                // Wait for work to begin
                // (Transition controlled externally)
            end
            
            GPU_STATE_RUNNING: begin
                // Check for exceptions
                if (new_exception && exception_enable) begin
                    // Check if this is a fatal exception
                    if (new_exc_code inside {EXC_AXI_DECERR, EXC_INTERNAL_ERROR}) begin
                        next_state = GPU_STATE_FATAL;
                        cluster_halt = '1;
                    end else begin
                        next_state = GPU_STATE_EXCEPTION;
                        // Halt affected cluster
                        cluster_halt[new_exc_cluster] = 1'b1;
                    end
                end
            end
            
            GPU_STATE_EXCEPTION: begin
                // Wait for CPU to acknowledge and handle
                if (exception_ack) begin
                    next_state = GPU_STATE_RECOVERY;
                end
            end
            
            GPU_STATE_RECOVERY: begin
                // Resume after exception handled
                cluster_resume = '1;
                next_state = GPU_STATE_RUNNING;
            end
            
            GPU_STATE_HALTED: begin
                cluster_halt = '1;
                // Can only exit via reset
            end
            
            GPU_STATE_FATAL: begin
                cluster_halt = '1;
                // Unrecoverable error - requires reset
            end
            
            default: begin
                next_state = GPU_STATE_RESET;
            end
        endcase
    end
    
    //==========================================================================
    // Output Registers
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            exc_status <= '0;
            exc_cause <= '0;
            exc_addr <= '0;
            exc_pc <= '0;
            exc_cluster <= '0;
            exc_warp <= '0;
            exc_context <= '0;
            exc_info <= '0;
            total_exceptions <= '0;
            page_fault_count <= '0;
            illegal_instr_count <= '0;
            axi_error_count <= '0;
        end else if (exception_clear) begin
            exc_status <= '0;
        end else begin
            // Update status bitmap
            if (new_exception) begin
                exc_status[new_exc_code] <= 1'b1;
                total_exceptions <= total_exceptions + 1;
                
                // Update specific counters
                if (new_exc_code inside {EXC_PAGE_FAULT_READ, EXC_PAGE_FAULT_WRITE, EXC_PAGE_FAULT_EXEC})
                    page_fault_count <= page_fault_count + 1;
                if (new_exc_code inside {EXC_ILLEGAL_INSTR, EXC_UNDEFINED_OPCODE})
                    illegal_instr_count <= illegal_instr_count + 1;
                if (new_exc_code inside {EXC_AXI_SLVERR, EXC_AXI_DECERR})
                    axi_error_count <= axi_error_count + 1;
            end
            
            // Latch first exception details (until cleared)
            if (new_exception && exc_cause == '0) begin
                exc_cause <= {26'b0, new_exc_code};
                exc_addr <= new_exc_addr;
                exc_pc <= new_exc_pc;
                exc_cluster <= {4'b0, new_exc_cluster};
                exc_warp <= {3'b0, new_exc_warp};
                exc_context <= {4'b0, new_exc_context};
                exc_info <= new_exc_info;
            end
        end
    end
    
    //==========================================================================
    // Interrupt Generation
    //==========================================================================
    
    logic irq_condition;
    
    // Generate IRQ if exception is not masked
    assign irq_condition = new_exception && exception_enable && 
                          exception_mask[new_exc_code];
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            irq_error <= 1'b0;
        end else if (exception_ack) begin
            irq_error <= 1'b0;
        end else if (irq_condition) begin
            irq_error <= 1'b1;
        end
    end
    
    assign irq_pending = !exc_queue_empty;
    assign gpu_state = current_state;
    assign gpu_halted = (current_state == GPU_STATE_HALTED) || 
                       (current_state == GPU_STATE_FATAL);
    assign gpu_error = (current_state == GPU_STATE_FATAL);

endmodule : krypton_exception_handler
