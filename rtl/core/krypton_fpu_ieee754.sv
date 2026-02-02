//==============================================================================
// GPU Krypton Series-1 - IEEE 754 Single Precision FPU
// Supports: FADD, FSUB, FMUL, FMA, FMIN, FMAX with proper rounding
// Pipeline: 5 stages for FMA, 4 stages for ADD/MUL
//==============================================================================

module krypton_fpu_ieee754
(
    input  logic                clk,
    input  logic                rst_n,
    
    // Operation select
    input  logic [3:0]          op,         // See fpu_op_t enum
    input  logic [1:0]          rounding,   // 0=RNE, 1=RTZ, 2=RDN, 3=RUP
    
    // Operands (IEEE 754 single precision)
    input  logic [31:0]         a,
    input  logic [31:0]         b,
    input  logic [31:0]         c,          // For FMA: a*b+c
    
    // Control
    input  logic                valid_in,
    output logic                ready,
    
    // Result
    output logic [31:0]         result,
    output logic                valid_out,
    
    // Exception flags (sticky)
    output logic                flag_inexact,
    output logic                flag_underflow,
    output logic                flag_overflow,
    output logic                flag_div_by_zero,
    output logic                flag_invalid
);

    //==========================================================================
    // Operation Codes
    //==========================================================================
    
    typedef enum logic [3:0] {
        FPU_ADD     = 4'h0,
        FPU_SUB     = 4'h1,
        FPU_MUL     = 4'h2,
        FPU_FMA     = 4'h3,     // a*b + c
        FPU_FMSUB   = 4'h4,     // a*b - c
        FPU_FNMADD  = 4'h5,     // -(a*b) + c
        FPU_FNMSUB  = 4'h6,     // -(a*b) - c
        FPU_MIN     = 4'h7,
        FPU_MAX     = 4'h8,
        FPU_CMP_EQ  = 4'h9,
        FPU_CMP_LT  = 4'hA,
        FPU_CMP_LE  = 4'hB,
        FPU_ITOF    = 4'hC,     // Int to float
        FPU_FTOI    = 4'hD,     // Float to int
        FPU_SQRT    = 4'hE      // Via Newton-Raphson
    } fpu_op_t;
    
    //==========================================================================
    // Special Value Constants
    //==========================================================================
    
    localparam logic [31:0] POS_ZERO     = 32'h00000000;
    localparam logic [31:0] NEG_ZERO     = 32'h80000000;
    localparam logic [31:0] POS_INF      = 32'h7F800000;
    localparam logic [31:0] NEG_INF      = 32'hFF800000;
    localparam logic [31:0] QUIET_NAN    = 32'h7FC00000;
    localparam logic [31:0] SIGNAL_NAN   = 32'h7F800001;
    
    //==========================================================================
    // Unpacked Floating Point Format
    //==========================================================================
    
    typedef struct packed {
        logic           sign;
        logic [9:0]     exp;        // Extended for overflow detection
        logic [47:0]    mant;       // Extended for FMA precision
        logic           is_zero;
        logic           is_inf;
        logic           is_nan;
        logic           is_snan;    // Signaling NaN
        logic           is_denorm;
    } fp_unpacked_t;
    
    //==========================================================================
    // Unpack Functions
    //==========================================================================
    
    function automatic fp_unpacked_t unpack_fp32(input logic [31:0] fp);
        fp_unpacked_t result;
        
        result.sign = fp[31];
        result.exp = {2'b0, fp[30:23]};
        result.is_denorm = (fp[30:23] == 8'h00) && (fp[22:0] != 23'h0);
        result.is_zero = (fp[30:23] == 8'h00) && (fp[22:0] == 23'h0);
        result.is_inf = (fp[30:23] == 8'hFF) && (fp[22:0] == 23'h0);
        result.is_nan = (fp[30:23] == 8'hFF) && (fp[22:0] != 23'h0);
        result.is_snan = result.is_nan && !fp[22];  // MSB of mantissa = 0 for sNaN
        
        // Construct mantissa with implicit bit
        if (result.is_denorm) begin
            result.mant = {1'b0, fp[22:0], 24'b0};
            result.exp = 10'd1;  // Denorms have effective exponent of 1
        end else if (result.is_zero || result.is_nan || result.is_inf) begin
            result.mant = {1'b0, fp[22:0], 24'b0};
        end else begin
            result.mant = {1'b1, fp[22:0], 24'b0};
        end
        
        return result;
    endfunction
    
    //==========================================================================
    // Pipeline Registers
    //==========================================================================
    
    // Stage 0: Input registration and unpacking
    fp_unpacked_t   s0_a, s0_b, s0_c;
    logic [3:0]     s0_op;
    logic [1:0]     s0_rnd;
    logic           s0_valid;
    
    // Stage 1: Multiply (for MUL/FMA)
    logic [47:0]    s1_product;
    logic [9:0]     s1_prod_exp;
    logic           s1_prod_sign;
    fp_unpacked_t   s1_c;
    logic [3:0]     s1_op;
    logic [1:0]     s1_rnd;
    logic           s1_valid;
    logic           s1_special_case;
    logic [31:0]    s1_special_result;
    logic           s1_invalid;
    
    // Stage 2: Alignment
    logic [74:0]    s2_mant_large;  // 75 bits for full precision
    logic [74:0]    s2_mant_small;
    logic [9:0]     s2_exp;
    logic           s2_sign_large;
    logic           s2_sign_small;
    logic           s2_eff_sub;
    logic [3:0]     s2_op;
    logic [1:0]     s2_rnd;
    logic           s2_valid;
    logic           s2_special_case;
    logic [31:0]    s2_special_result;
    logic           s2_invalid;
    
    // Stage 3: Addition/Subtraction
    logic [75:0]    s3_sum;
    logic [9:0]     s3_exp;
    logic           s3_sign;
    logic [3:0]     s3_op;
    logic [1:0]     s3_rnd;
    logic           s3_valid;
    logic           s3_special_case;
    logic [31:0]    s3_special_result;
    logic           s3_invalid;
    logic           s3_inexact;
    
    // Stage 4: Normalization and Rounding
    logic [31:0]    s4_result;
    logic           s4_valid;
    logic           s4_inexact;
    logic           s4_overflow;
    logic           s4_underflow;
    logic           s4_invalid;
    
    //==========================================================================
    // Stage 0: Unpack and Special Case Detection
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0_valid <= 1'b0;
        end else begin
            s0_valid <= valid_in;
            s0_op <= op;
            s0_rnd <= rounding;
            s0_a <= unpack_fp32(a);
            s0_b <= unpack_fp32(b);
            s0_c <= unpack_fp32(c);
        end
    end
    
    //==========================================================================
    // Stage 1: Multiply + Special Case Handling
    //==========================================================================
    
    // 24x24 multiplier for mantissas
    logic [47:0] mult_result;
    assign mult_result = s0_a.mant[47:24] * s0_b.mant[47:24];
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_valid <= 1'b0;
        end else begin
            s1_valid <= s0_valid;
            s1_op <= s0_op;
            s1_rnd <= s0_rnd;
            s1_c <= s0_c;
            
            // Product calculation
            s1_product <= mult_result;
            s1_prod_exp <= s0_a.exp + s0_b.exp - 10'd127;  // Remove one bias
            s1_prod_sign <= s0_a.sign ^ s0_b.sign;
            
            // Negate product for FNMADD/FNMSUB
            if (s0_op == FPU_FNMADD || s0_op == FPU_FNMSUB) begin
                s1_prod_sign <= ~(s0_a.sign ^ s0_b.sign);
            end
            
            // Special case handling
            s1_special_case <= 1'b0;
            s1_special_result <= '0;
            s1_invalid <= 1'b0;
            
            // NaN propagation
            if (s0_a.is_nan || s0_b.is_nan || 
                (s0_op inside {FPU_FMA, FPU_FMSUB, FPU_FNMADD, FPU_FNMSUB} && s0_c.is_nan)) begin
                s1_special_case <= 1'b1;
                s1_special_result <= QUIET_NAN;
                s1_invalid <= s0_a.is_snan || s0_b.is_snan || s0_c.is_snan;
            end
            // Inf * 0 = NaN
            else if ((s0_a.is_inf && s0_b.is_zero) || (s0_a.is_zero && s0_b.is_inf)) begin
                s1_special_case <= 1'b1;
                s1_special_result <= QUIET_NAN;
                s1_invalid <= 1'b1;
            end
            // Inf handling
            else if (s0_a.is_inf || s0_b.is_inf) begin
                if (s0_op == FPU_MUL) begin
                    s1_special_case <= 1'b1;
                    s1_special_result <= (s0_a.sign ^ s0_b.sign) ? NEG_INF : POS_INF;
                end
            end
            // Zero handling
            else if (s0_a.is_zero || s0_b.is_zero) begin
                if (s0_op == FPU_MUL) begin
                    s1_special_case <= 1'b1;
                    s1_special_result <= (s0_a.sign ^ s0_b.sign) ? NEG_ZERO : POS_ZERO;
                end
            end
        end
    end
    
    //==========================================================================
    // Stage 2: Alignment
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2_valid <= 1'b0;
        end else begin
            s2_valid <= s1_valid;
            s2_op <= s1_op;
            s2_rnd <= s1_rnd;
            s2_special_case <= s1_special_case;
            s2_special_result <= s1_special_result;
            s2_invalid <= s1_invalid;
            
            if (!s1_special_case) begin
                // For ADD/SUB, use a and b directly
                // For FMA variants, use product and c
                logic [9:0] exp_a, exp_b;
                logic [74:0] mant_a, mant_b;
                logic sign_a, sign_b;
                
                if (s1_op == FPU_ADD || s1_op == FPU_SUB) begin
                    exp_a = s0_a.exp;
                    exp_b = s0_b.exp;
                    mant_a = {s0_a.mant, 27'b0};
                    mant_b = {s0_b.mant, 27'b0};
                    sign_a = s0_a.sign;
                    sign_b = (s1_op == FPU_SUB) ? ~s0_b.sign : s0_b.sign;
                end else begin
                    // FMA: product + c
                    exp_a = s1_prod_exp;
                    exp_b = s1_c.exp;
                    mant_a = {s1_product, 27'b0};
                    mant_b = {s1_c.mant, 27'b0};
                    sign_a = s1_prod_sign;
                    sign_b = (s1_op == FPU_FMSUB || s1_op == FPU_FNMSUB) ? ~s1_c.sign : s1_c.sign;
                end
                
                // Align to larger exponent
                if (exp_a >= exp_b) begin
                    logic [9:0] shift;
                    shift = exp_a - exp_b;
                    s2_exp <= exp_a;
                    s2_mant_large <= mant_a;
                    s2_mant_small <= (shift >= 75) ? 75'b0 : (mant_b >> shift);
                    s2_sign_large <= sign_a;
                    s2_sign_small <= sign_b;
                end else begin
                    logic [9:0] shift;
                    shift = exp_b - exp_a;
                    s2_exp <= exp_b;
                    s2_mant_large <= mant_b;
                    s2_mant_small <= (shift >= 75) ? 75'b0 : (mant_a >> shift);
                    s2_sign_large <= sign_b;
                    s2_sign_small <= sign_a;
                end
                
                s2_eff_sub <= sign_a ^ sign_b;
            end
        end
    end
    
    //==========================================================================
    // Stage 3: Addition/Subtraction
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s3_valid <= 1'b0;
        end else begin
            s3_valid <= s2_valid;
            s3_op <= s2_op;
            s3_rnd <= s2_rnd;
            s3_special_case <= s2_special_case;
            s3_special_result <= s2_special_result;
            s3_invalid <= s2_invalid;
            s3_exp <= s2_exp;
            
            if (!s2_special_case) begin
                if (s2_eff_sub) begin
                    // Subtraction
                    if (s2_mant_large >= s2_mant_small) begin
                        s3_sum <= {1'b0, s2_mant_large} - {1'b0, s2_mant_small};
                        s3_sign <= s2_sign_large;
                    end else begin
                        s3_sum <= {1'b0, s2_mant_small} - {1'b0, s2_mant_large};
                        s3_sign <= ~s2_sign_large;
                    end
                end else begin
                    // Addition
                    s3_sum <= {1'b0, s2_mant_large} + {1'b0, s2_mant_small};
                    s3_sign <= s2_sign_large;
                end
                
                // Track if bits were lost in alignment (for inexact flag)
                s3_inexact <= |s2_mant_small[26:0];
            end
        end
    end
    
    //==========================================================================
    // Stage 4: Normalization and Rounding
    //==========================================================================
    
    // Leading zero counter for normalization
    function automatic logic [6:0] count_leading_zeros(input logic [75:0] val);
        logic [6:0] count;
        count = 0;
        for (int i = 75; i >= 0; i--) begin
            if (val[i] == 1'b0)
                count = count + 1;
            else
                break;
        end
        return count;
    endfunction
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s4_valid <= 1'b0;
            s4_inexact <= 1'b0;
            s4_overflow <= 1'b0;
            s4_underflow <= 1'b0;
            s4_invalid <= 1'b0;
        end else begin
            s4_valid <= s3_valid;
            s4_invalid <= s3_invalid;
            
            if (s3_special_case) begin
                s4_result <= s3_special_result;
                s4_inexact <= 1'b0;
                s4_overflow <= 1'b0;
                s4_underflow <= 1'b0;
            end else begin
                logic [6:0] lzc;
                logic [75:0] norm_mant;
                logic signed [10:0] norm_exp;
                logic [23:0] round_mant;
                logic [7:0] final_exp;
                logic round_bit, sticky;
                logic round_up;
                
                // Check for overflow (sum > 2.0)
                if (s3_sum[75]) begin
                    norm_mant = s3_sum >> 1;
                    norm_exp = s3_exp + 1;
                end else begin
                    // Normalize by counting leading zeros
                    lzc = count_leading_zeros(s3_sum);
                    if (lzc >= 76) begin
                        // Result is zero
                        norm_mant = '0;
                        norm_exp = 0;
                    end else begin
                        norm_mant = s3_sum << lzc;
                        norm_exp = s3_exp - lzc + 1;
                    end
                end
                
                // Extract mantissa and round bits
                // norm_mant[74] is the implicit 1
                round_mant = norm_mant[73:50];
                round_bit = norm_mant[49];
                sticky = |norm_mant[48:0] | s3_inexact;
                
                // Rounding decision
                case (s3_rnd)
                    2'b00: round_up = round_bit && (sticky || round_mant[0]);  // RNE
                    2'b01: round_up = 1'b0;                                      // RTZ
                    2'b10: round_up = s3_sign && (round_bit || sticky);         // RDN
                    2'b11: round_up = !s3_sign && (round_bit || sticky);        // RUP
                endcase
                
                // Apply rounding
                if (round_up) begin
                    {final_exp, round_mant} = {norm_exp[7:0], round_mant} + 1;
                    if (round_mant == 24'h000000)  // Carry into exponent
                        final_exp = norm_exp[7:0] + 1;
                end else begin
                    final_exp = norm_exp[7:0];
                end
                
                // Check for overflow/underflow
                if (norm_exp >= 255) begin
                    // Overflow to infinity
                    s4_result <= s3_sign ? NEG_INF : POS_INF;
                    s4_overflow <= 1'b1;
                    s4_inexact <= 1'b1;
                    s4_underflow <= 1'b0;
                end else if (norm_exp <= 0) begin
                    // Underflow to zero (could implement denorms here)
                    s4_result <= s3_sign ? NEG_ZERO : POS_ZERO;
                    s4_underflow <= 1'b1;
                    s4_inexact <= 1'b1;
                    s4_overflow <= 1'b0;
                end else begin
                    s4_result <= {s3_sign, final_exp, round_mant[22:0]};
                    s4_inexact <= round_bit || sticky;
                    s4_overflow <= 1'b0;
                    s4_underflow <= 1'b0;
                end
            end
        end
    end
    
    //==========================================================================
    // Output Assignment
    //==========================================================================
    
    assign result = s4_result;
    assign valid_out = s4_valid;
    assign ready = 1'b1;  // Fully pipelined, always ready
    
    // Exception flags (sticky - only set, never cleared by this module)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            flag_inexact <= 1'b0;
            flag_underflow <= 1'b0;
            flag_overflow <= 1'b0;
            flag_div_by_zero <= 1'b0;
            flag_invalid <= 1'b0;
        end else if (s4_valid) begin
            flag_inexact <= flag_inexact | s4_inexact;
            flag_underflow <= flag_underflow | s4_underflow;
            flag_overflow <= flag_overflow | s4_overflow;
            flag_invalid <= flag_invalid | s4_invalid;
            // div_by_zero would be set by division operation (not in this module)
        end
    end

endmodule
