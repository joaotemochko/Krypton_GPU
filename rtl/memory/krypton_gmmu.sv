//==============================================================================
// GPU Krypton Series-1 - Graphics Memory Management Unit (GMMU)
// Supports: SV39 (39-bit virtual address space, 3-level page tables)
// Features: TLB, Page Table Walker, Context Isolation, Fault Handling
//==============================================================================

module krypton_gmmu
    import krypton_pkg::*;
#(
    parameter int TLB_ENTRIES       = 64,       // Fully associative TLB
    parameter int PTW_QUEUE_DEPTH   = 4,        // Outstanding page walks
    parameter int NUM_CONTEXTS      = 16        // Max GPU contexts
)(
    input  logic                        clk,
    input  logic                        rst_n,
    
    //==========================================================================
    // Translation Request Interface (from L2 Cache / Shader Clusters)
    //==========================================================================
    input  logic                        req_valid,
    output logic                        req_ready,
    input  logic [ADDR_WIDTH-1:0]       req_vaddr,          // Virtual address
    input  logic [3:0]                  req_context_id,     // GPU context (process)
    input  logic                        req_write,          // Write access
    input  logic                        req_exec,           // Execute (shader fetch)
    input  logic [2:0]                  req_source_id,      // Requesting unit ID
    
    output logic                        resp_valid,
    output logic [ADDR_WIDTH-1:0]       resp_paddr,         // Physical address
    output logic                        resp_fault,         // Page fault occurred
    output logic [3:0]                  resp_fault_code,    // Fault reason
    output logic [2:0]                  resp_source_id,     // Echo back source
    
    //==========================================================================
    // Page Table Walker Memory Interface (to L2/Memory)
    //==========================================================================
    output logic                        ptw_req,
    output logic [ADDR_WIDTH-1:0]       ptw_addr,
    input  logic                        ptw_ready,
    input  logic                        ptw_valid,
    input  logic [63:0]                 ptw_data,
    input  logic                        ptw_error,          // AXI error on PTW
    
    //==========================================================================
    // Configuration Interface (from APB)
    //==========================================================================
    input  logic [ADDR_WIDTH-1:0]       page_table_base [0:NUM_CONTEXTS-1],
    input  logic [NUM_CONTEXTS-1:0]     context_valid,      // Context enabled
    input  logic                        gmmu_enable,        // Global enable
    input  logic                        bypass_mode,        // Pass-through (no translation)
    
    //==========================================================================
    // TLB Management
    //==========================================================================
    input  logic                        tlb_flush,
    input  logic [3:0]                  tlb_flush_ctx,      // Context to flush
    input  logic                        tlb_flush_all,      // Flush entire TLB
    input  logic [ADDR_WIDTH-1:0]       tlb_flush_addr,     // Specific address flush
    input  logic                        tlb_flush_addr_valid,
    
    //==========================================================================
    // Fault Reporting (to Exception Handler)
    //==========================================================================
    output logic                        fault_irq,
    output logic [ADDR_WIDTH-1:0]       fault_vaddr,
    output logic [ADDR_WIDTH-1:0]       fault_ptw_addr,     // Where PTW failed
    output logic [3:0]                  fault_context,
    output logic [3:0]                  fault_code,
    output logic [2:0]                  fault_source,
    
    //==========================================================================
    // Performance Counters
    //==========================================================================
    output logic [31:0]                 tlb_hits,
    output logic [31:0]                 tlb_misses,
    output logic [31:0]                 page_faults
);

    //==========================================================================
    // SV39 Page Table Entry Format (64-bit)
    //==========================================================================
    // [63:54] Reserved
    // [53:28] PPN[2] (26 bits) - Gigapage
    // [27:19] PPN[1] (9 bits)  - Megapage
    // [18:10] PPN[0] (9 bits)  - Normal page
    // [9:8]   RSW (reserved for software)
    // [7]     D (Dirty)
    // [6]     A (Accessed)
    // [5]     G (Global)
    // [4]     U (User)
    // [3]     X (Execute)
    // [2]     W (Write)
    // [1]     R (Read)
    // [0]     V (Valid)
    //==========================================================================
    
    typedef struct packed {
        logic [9:0]     reserved;
        logic [25:0]    ppn2;           // Gigapage PPN
        logic [8:0]     ppn1;           // Megapage PPN
        logic [8:0]     ppn0;           // Page PPN
        logic [1:0]     rsw;
        logic           dirty;
        logic           accessed;
        logic           global_bit;
        logic           user;
        logic           execute;
        logic           write;
        logic           read;
        logic           valid;
    } sv39_pte_t;
    
    // Page sizes
    typedef enum logic [1:0] {
        PAGE_4KB    = 2'b00,    // 4KB normal page
        PAGE_2MB    = 2'b01,    // 2MB megapage (level 1)
        PAGE_1GB    = 2'b10     // 1GB gigapage (level 0)
    } page_size_t;
    
    // Fault codes
    typedef enum logic [3:0] {
        FAULT_NONE              = 4'h0,
        FAULT_NOT_PRESENT       = 4'h1,
        FAULT_WRITE_PROTECT     = 4'h2,
        FAULT_EXEC_PROTECT      = 4'h3,
        FAULT_USER_PROTECT      = 4'h4,
        FAULT_MISALIGNED        = 4'h5,
        FAULT_PTW_ERROR         = 4'h6,  // AXI error during page walk
        FAULT_INVALID_CONTEXT   = 4'h7,
        FAULT_RESERVED_BITS     = 4'h8
    } fault_code_t;
    
    //==========================================================================
    // TLB Entry Structure
    //==========================================================================
    
    typedef struct packed {
        logic                   valid;
        logic [3:0]             context_id;
        logic [26:0]            vpn;            // Virtual page number (27 bits for SV39)
        logic [43:0]            ppn;            // Physical page number (44 bits)
        page_size_t             page_size;
        logic                   read;
        logic                   write;
        logic                   execute;
        logic                   user;
        logic                   global_bit;
        logic                   dirty;
        logic                   accessed;
    } tlb_entry_t;
    
    tlb_entry_t tlb [0:TLB_ENTRIES-1];
    logic [$clog2(TLB_ENTRIES)-1:0] tlb_lru [0:TLB_ENTRIES-1];
    
    //==========================================================================
    // Virtual Address Decomposition (SV39)
    //==========================================================================
    // VA[38:30] = VPN[2] (9 bits) - Index into level 0 (root)
    // VA[29:21] = VPN[1] (9 bits) - Index into level 1
    // VA[20:12] = VPN[0] (9 bits) - Index into level 2
    // VA[11:0]  = Page offset (12 bits)
    //==========================================================================
    
    logic [8:0]     req_vpn2, req_vpn1, req_vpn0;
    logic [11:0]    req_offset;
    
    assign req_vpn2   = req_vaddr[38:30];
    assign req_vpn1   = req_vaddr[29:21];
    assign req_vpn0   = req_vaddr[20:12];
    assign req_offset = req_vaddr[11:0];
    
    //==========================================================================
    // TLB Lookup Logic
    //==========================================================================
    
    logic                               tlb_hit;
    logic [$clog2(TLB_ENTRIES)-1:0]     tlb_hit_idx;
    tlb_entry_t                         tlb_hit_entry;
    
    always_comb begin
        tlb_hit = 1'b0;
        tlb_hit_idx = '0;
        tlb_hit_entry = '0;
        
        for (int i = 0; i < TLB_ENTRIES; i++) begin
            if (tlb[i].valid) begin
                logic ctx_match, vpn_match;
                
                // Context match (or global page)
                ctx_match = tlb[i].global_bit || (tlb[i].context_id == req_context_id);
                
                // VPN match depends on page size
                case (tlb[i].page_size)
                    PAGE_4KB: vpn_match = (tlb[i].vpn == req_vaddr[38:12]);
                    PAGE_2MB: vpn_match = (tlb[i].vpn[26:9] == req_vaddr[38:21]);
                    PAGE_1GB: vpn_match = (tlb[i].vpn[26:18] == req_vaddr[38:30]);
                    default:  vpn_match = 1'b0;
                endcase
                
                if (ctx_match && vpn_match) begin
                    tlb_hit = 1'b1;
                    tlb_hit_idx = i[$clog2(TLB_ENTRIES)-1:0];
                    tlb_hit_entry = tlb[i];
                end
            end
        end
    end
    
    //==========================================================================
    // Permission Check
    //==========================================================================
    
    logic           perm_read_ok, perm_write_ok, perm_exec_ok;
    logic           perm_ok;
    fault_code_t    perm_fault_code;
    
    always_comb begin
        perm_read_ok  = tlb_hit_entry.read;
        perm_write_ok = tlb_hit_entry.write;
        perm_exec_ok  = tlb_hit_entry.execute;
        
        perm_ok = 1'b1;
        perm_fault_code = FAULT_NONE;
        
        if (tlb_hit) begin
            // Check read permission (always needed)
            if (!perm_read_ok && !req_exec) begin
                perm_ok = 1'b0;
                perm_fault_code = FAULT_NOT_PRESENT;
            end
            // Check write permission
            else if (req_write && !perm_write_ok) begin
                perm_ok = 1'b0;
                perm_fault_code = FAULT_WRITE_PROTECT;
            end
            // Check execute permission
            else if (req_exec && !perm_exec_ok) begin
                perm_ok = 1'b0;
                perm_fault_code = FAULT_EXEC_PROTECT;
            end
        end
    end
    
    //==========================================================================
    // Physical Address Generation
    //==========================================================================
    
    logic [ADDR_WIDTH-1:0] translated_paddr;
    
    always_comb begin
        case (tlb_hit_entry.page_size)
            PAGE_4KB: translated_paddr = {tlb_hit_entry.ppn, req_offset};
            PAGE_2MB: translated_paddr = {tlb_hit_entry.ppn[43:9], req_vaddr[20:0]};
            PAGE_1GB: translated_paddr = {tlb_hit_entry.ppn[43:18], req_vaddr[29:0]};
            default:  translated_paddr = '0;
        endcase
    end
    
    //==========================================================================
    // Page Table Walker State Machine
    //==========================================================================
    
    typedef enum logic [3:0] {
        PTW_IDLE,
        PTW_LEVEL0,         // Fetch level 0 (root) PTE
        PTW_LEVEL0_WAIT,
        PTW_LEVEL1,         // Fetch level 1 PTE
        PTW_LEVEL1_WAIT,
        PTW_LEVEL2,         // Fetch level 2 PTE
        PTW_LEVEL2_WAIT,
        PTW_UPDATE_TLB,
        PTW_FAULT
    } ptw_state_t;
    
    ptw_state_t ptw_state;
    
    // PTW registers
    logic [ADDR_WIDTH-1:0]  ptw_vaddr_r;
    logic [3:0]             ptw_ctx_r;
    logic                   ptw_write_r;
    logic                   ptw_exec_r;
    logic [2:0]             ptw_source_r;
    sv39_pte_t              pte_level0, pte_level1, pte_level2;
    page_size_t             resolved_page_size;
    logic [43:0]            resolved_ppn;
    fault_code_t            ptw_fault_code;
    
    //==========================================================================
    // Main State Machine
    //==========================================================================
    
    typedef enum logic [2:0] {
        ST_IDLE,
        ST_TLB_LOOKUP,
        ST_PTW_START,
        ST_PTW_WAIT,
        ST_RESPOND,
        ST_FAULT
    } main_state_t;
    
    main_state_t state;
    
    // Registered request
    logic [ADDR_WIDTH-1:0]  req_vaddr_r;
    logic [3:0]             req_ctx_r;
    logic                   req_write_r;
    logic                   req_exec_r;
    logic [2:0]             req_source_r;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            ptw_state <= PTW_IDLE;
            resp_valid <= 1'b0;
            ptw_req <= 1'b0;
            fault_irq <= 1'b0;
            tlb_hits <= '0;
            tlb_misses <= '0;
            page_faults <= '0;
            
            for (int i = 0; i < TLB_ENTRIES; i++) begin
                tlb[i].valid <= 1'b0;
                tlb_lru[i] <= i[$clog2(TLB_ENTRIES)-1:0];
            end
        end else begin
            resp_valid <= 1'b0;
            fault_irq <= 1'b0;
            
            //------------------------------------------------------------------
            // TLB Flush Handling
            //------------------------------------------------------------------
            if (tlb_flush) begin
                if (tlb_flush_all) begin
                    for (int i = 0; i < TLB_ENTRIES; i++)
                        tlb[i].valid <= 1'b0;
                end else if (tlb_flush_addr_valid) begin
                    // Flush specific address
                    for (int i = 0; i < TLB_ENTRIES; i++) begin
                        if (tlb[i].valid && 
                            (tlb[i].context_id == tlb_flush_ctx || tlb[i].global_bit) &&
                            tlb[i].vpn == tlb_flush_addr[38:12])
                            tlb[i].valid <= 1'b0;
                    end
                end else begin
                    // Flush context
                    for (int i = 0; i < TLB_ENTRIES; i++) begin
                        if (tlb[i].valid && 
                            tlb[i].context_id == tlb_flush_ctx && 
                            !tlb[i].global_bit)
                            tlb[i].valid <= 1'b0;
                    end
                end
            end
            
            //------------------------------------------------------------------
            // Main State Machine
            //------------------------------------------------------------------
            case (state)
                ST_IDLE: begin
                    if (req_valid && gmmu_enable) begin
                        // Check context validity
                        if (!context_valid[req_context_id]) begin
                            state <= ST_FAULT;
                            ptw_fault_code <= FAULT_INVALID_CONTEXT;
                        end else if (bypass_mode) begin
                            // Pass-through mode
                            resp_valid <= 1'b1;
                            resp_paddr <= req_vaddr;
                            resp_fault <= 1'b0;
                            resp_source_id <= req_source_id;
                        end else begin
                            // Normal translation
                            req_vaddr_r <= req_vaddr;
                            req_ctx_r <= req_context_id;
                            req_write_r <= req_write;
                            req_exec_r <= req_exec;
                            req_source_r <= req_source_id;
                            state <= ST_TLB_LOOKUP;
                        end
                    end
                end
                
                ST_TLB_LOOKUP: begin
                    if (tlb_hit && perm_ok) begin
                        // TLB hit with valid permissions
                        resp_valid <= 1'b1;
                        resp_paddr <= translated_paddr;
                        resp_fault <= 1'b0;
                        resp_source_id <= req_source_r;
                        tlb_hits <= tlb_hits + 1;
                        
                        // Update LRU
                        for (int i = 0; i < TLB_ENTRIES; i++) begin
                            if (tlb_lru[i] < tlb_lru[tlb_hit_idx])
                                tlb_lru[i] <= tlb_lru[i] + 1;
                        end
                        tlb_lru[tlb_hit_idx] <= '0;
                        
                        // Set accessed/dirty bits
                        tlb[tlb_hit_idx].accessed <= 1'b1;
                        if (req_write_r)
                            tlb[tlb_hit_idx].dirty <= 1'b1;
                        
                        state <= ST_IDLE;
                    end else if (tlb_hit && !perm_ok) begin
                        // Permission fault
                        ptw_fault_code <= perm_fault_code;
                        state <= ST_FAULT;
                    end else begin
                        // TLB miss - start page table walk
                        tlb_misses <= tlb_misses + 1;
                        ptw_vaddr_r <= req_vaddr_r;
                        ptw_ctx_r <= req_ctx_r;
                        ptw_write_r <= req_write_r;
                        ptw_exec_r <= req_exec_r;
                        ptw_source_r <= req_source_r;
                        ptw_state <= PTW_LEVEL0;
                        state <= ST_PTW_WAIT;
                    end
                end
                
                ST_PTW_WAIT: begin
                    // Page Table Walker running
                    if (ptw_state == PTW_UPDATE_TLB) begin
                        state <= ST_RESPOND;
                    end else if (ptw_state == PTW_FAULT) begin
                        state <= ST_FAULT;
                    end
                end
                
                ST_RESPOND: begin
                    resp_valid <= 1'b1;
                    resp_paddr <= translated_paddr;
                    resp_fault <= 1'b0;
                    resp_source_id <= ptw_source_r;
                    state <= ST_IDLE;
                    ptw_state <= PTW_IDLE;
                end
                
                ST_FAULT: begin
                    resp_valid <= 1'b1;
                    resp_fault <= 1'b1;
                    resp_fault_code <= ptw_fault_code;
                    resp_source_id <= (ptw_state != PTW_IDLE) ? ptw_source_r : req_source_r;
                    
                    fault_irq <= 1'b1;
                    fault_vaddr <= (ptw_state != PTW_IDLE) ? ptw_vaddr_r : req_vaddr_r;
                    fault_context <= (ptw_state != PTW_IDLE) ? ptw_ctx_r : req_ctx_r;
                    fault_code <= ptw_fault_code;
                    fault_source <= (ptw_state != PTW_IDLE) ? ptw_source_r : req_source_r;
                    page_faults <= page_faults + 1;
                    
                    state <= ST_IDLE;
                    ptw_state <= PTW_IDLE;
                end
            endcase
            
            //------------------------------------------------------------------
            // Page Table Walker State Machine
            //------------------------------------------------------------------
            case (ptw_state)
                PTW_IDLE: begin
                    // Waiting for main SM to trigger
                end
                
                PTW_LEVEL0: begin
                    // Fetch root page table entry
                    ptw_req <= 1'b1;
                    // PTE address = SATP.PPN * 4096 + VPN[2] * 8
                    ptw_addr <= page_table_base[ptw_ctx_r] + {ptw_vaddr_r[38:30], 3'b000};
                    ptw_state <= PTW_LEVEL0_WAIT;
                end
                
                PTW_LEVEL0_WAIT: begin
                    if (ptw_valid) begin
                        ptw_req <= 1'b0;
                        pte_level0 <= sv39_pte_t'(ptw_data);
                        
                        if (ptw_error) begin
                            ptw_fault_code <= FAULT_PTW_ERROR;
                            fault_ptw_addr <= ptw_addr;
                            ptw_state <= PTW_FAULT;
                        end else if (!ptw_data[0]) begin
                            // Invalid PTE
                            ptw_fault_code <= FAULT_NOT_PRESENT;
                            ptw_state <= PTW_FAULT;
                        end else if (ptw_data[1] || ptw_data[2] || ptw_data[3]) begin
                            // Leaf PTE (R, W, or X bit set) - Gigapage
                            resolved_page_size <= PAGE_1GB;
                            resolved_ppn <= {ptw_data[53:10]};
                            
                            // Check alignment for gigapage
                            if (ptw_data[27:10] != 18'b0) begin
                                ptw_fault_code <= FAULT_MISALIGNED;
                                ptw_state <= PTW_FAULT;
                            end else begin
                                ptw_state <= PTW_UPDATE_TLB;
                            end
                        end else begin
                            // Non-leaf PTE - continue to level 1
                            ptw_state <= PTW_LEVEL1;
                        end
                    end
                end
                
                PTW_LEVEL1: begin
                    ptw_req <= 1'b1;
                    // Next level address from PTE
                    ptw_addr <= {pte_level0.ppn2, pte_level0.ppn1, pte_level0.ppn0, 12'b0} +
                               {ptw_vaddr_r[29:21], 3'b000};
                    ptw_state <= PTW_LEVEL1_WAIT;
                end
                
                PTW_LEVEL1_WAIT: begin
                    if (ptw_valid) begin
                        ptw_req <= 1'b0;
                        pte_level1 <= sv39_pte_t'(ptw_data);
                        
                        if (ptw_error) begin
                            ptw_fault_code <= FAULT_PTW_ERROR;
                            fault_ptw_addr <= ptw_addr;
                            ptw_state <= PTW_FAULT;
                        end else if (!ptw_data[0]) begin
                            ptw_fault_code <= FAULT_NOT_PRESENT;
                            ptw_state <= PTW_FAULT;
                        end else if (ptw_data[1] || ptw_data[2] || ptw_data[3]) begin
                            // Leaf PTE - Megapage (2MB)
                            resolved_page_size <= PAGE_2MB;
                            resolved_ppn <= {ptw_data[53:10]};
                            
                            // Check alignment for megapage
                            if (ptw_data[18:10] != 9'b0) begin
                                ptw_fault_code <= FAULT_MISALIGNED;
                                ptw_state <= PTW_FAULT;
                            end else begin
                                ptw_state <= PTW_UPDATE_TLB;
                            end
                        end else begin
                            // Non-leaf - continue to level 2
                            ptw_state <= PTW_LEVEL2;
                        end
                    end
                end
                
                PTW_LEVEL2: begin
                    ptw_req <= 1'b1;
                    ptw_addr <= {pte_level1.ppn2, pte_level1.ppn1, pte_level1.ppn0, 12'b0} +
                               {ptw_vaddr_r[20:12], 3'b000};
                    ptw_state <= PTW_LEVEL2_WAIT;
                end
                
                PTW_LEVEL2_WAIT: begin
                    if (ptw_valid) begin
                        ptw_req <= 1'b0;
                        pte_level2 <= sv39_pte_t'(ptw_data);
                        
                        if (ptw_error) begin
                            ptw_fault_code <= FAULT_PTW_ERROR;
                            fault_ptw_addr <= ptw_addr;
                            ptw_state <= PTW_FAULT;
                        end else if (!ptw_data[0]) begin
                            ptw_fault_code <= FAULT_NOT_PRESENT;
                            ptw_state <= PTW_FAULT;
                        end else if (!(ptw_data[1] || ptw_data[2] || ptw_data[3])) begin
                            // Must be leaf at level 2
                            ptw_fault_code <= FAULT_NOT_PRESENT;
                            ptw_state <= PTW_FAULT;
                        end else begin
                            // Normal 4KB page
                            resolved_page_size <= PAGE_4KB;
                            resolved_ppn <= {ptw_data[53:10]};
                            ptw_state <= PTW_UPDATE_TLB;
                        end
                    end
                end
                
                PTW_UPDATE_TLB: begin
                    // Find LRU entry to replace
                    logic [$clog2(TLB_ENTRIES)-1:0] victim;
                    victim = '0;
                    for (int i = 0; i < TLB_ENTRIES; i++) begin
                        if (tlb_lru[i] == TLB_ENTRIES - 1)
                            victim = i[$clog2(TLB_ENTRIES)-1:0];
                    end
                    
                    // Install new TLB entry
                    tlb[victim].valid <= 1'b1;
                    tlb[victim].context_id <= ptw_ctx_r;
                    tlb[victim].vpn <= ptw_vaddr_r[38:12];
                    tlb[victim].ppn <= resolved_ppn;
                    tlb[victim].page_size <= resolved_page_size;
                    
                    // Permissions from final PTE
                    case (resolved_page_size)
                        PAGE_1GB: begin
                            tlb[victim].read <= pte_level0.read;
                            tlb[victim].write <= pte_level0.write;
                            tlb[victim].execute <= pte_level0.execute;
                            tlb[victim].user <= pte_level0.user;
                            tlb[victim].global_bit <= pte_level0.global_bit;
                            tlb[victim].dirty <= pte_level0.dirty;
                            tlb[victim].accessed <= pte_level0.accessed;
                        end
                        PAGE_2MB: begin
                            tlb[victim].read <= pte_level1.read;
                            tlb[victim].write <= pte_level1.write;
                            tlb[victim].execute <= pte_level1.execute;
                            tlb[victim].user <= pte_level1.user;
                            tlb[victim].global_bit <= pte_level1.global_bit;
                            tlb[victim].dirty <= pte_level1.dirty;
                            tlb[victim].accessed <= pte_level1.accessed;
                        end
                        default: begin
                            tlb[victim].read <= pte_level2.read;
                            tlb[victim].write <= pte_level2.write;
                            tlb[victim].execute <= pte_level2.execute;
                            tlb[victim].user <= pte_level2.user;
                            tlb[victim].global_bit <= pte_level2.global_bit;
                            tlb[victim].dirty <= pte_level2.dirty;
                            tlb[victim].accessed <= pte_level2.accessed;
                        end
                    endcase
                    
                    // Update LRU
                    for (int i = 0; i < TLB_ENTRIES; i++) begin
                        if (tlb_lru[i] < tlb_lru[victim])
                            tlb_lru[i] <= tlb_lru[i] + 1;
                    end
                    tlb_lru[victim] <= '0;
                    
                    // State transition handled by main SM
                end
                
                PTW_FAULT: begin
                    // Fault state - handled by main SM
                end
            endcase
        end
    end
    
    assign req_ready = (state == ST_IDLE) && gmmu_enable;

endmodule : krypton_gmmu
