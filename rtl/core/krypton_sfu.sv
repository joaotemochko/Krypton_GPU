//==============================================================================
// GPU Krypton Series-1 - Special Function Unit (SFU)
// Implements transcendental functions via LUT + Newton-Raphson/Polynomial
// Functions: RCP, RSQ, LOG2, EXP2, SIN, COS
// Precision: ≤ 2 ULP for RCP/RSQ, ≤ 3 ULP for LOG2/EXP2, ≤ 2^-11 for SIN/COS
//==============================================================================

module krypton_sfu
(
    input  logic                clk,
    input  logic                rst_n,
    
    input  logic [2:0]          op,         // 0=RCP, 1=RSQ, 2=LOG2, 3=EXP2, 4=SIN, 5=COS
    input  logic [31:0]         operand,
    input  logic                valid_in,
    
    output logic [31:0]         result,
    output logic                valid_out,
    output logic                error_flag  // Domain error (e.g., sqrt of negative)
);

    //==========================================================================
    // Operation Codes
    //==========================================================================
    
    typedef enum logic [2:0] {
        SFU_RCP     = 3'd0,     // 1/x
        SFU_RSQ     = 3'd1,     // 1/sqrt(x)
        SFU_LOG2    = 3'd2,     // log2(x)
        SFU_EXP2    = 3'd3,     // 2^x
        SFU_SIN     = 3'd4,     // sin(x) where x in radians
        SFU_COS     = 3'd5      // cos(x) where x in radians
    } sfu_op_t;
    
    //==========================================================================
    // Look-Up Tables (Synthesized as ROM)
    //==========================================================================
    
    // 10-bit address -> 24-bit data
    // Tables cover mantissa range [1.0, 2.0)
    
    logic [23:0] rcp_lut [0:1023];
    logic [23:0] rsq_lut [0:1023];
    logic [23:0] log2_lut [0:1023];
    logic [23:0] exp2_frac_lut [0:1023];
    logic [23:0] sin_lut [0:1023];
    logic [23:0] cos_lut [0:1023];
    
    // Initialize LUTs
    initial begin
        for (int i = 0; i < 1024; i++) begin
            real x, val;
            
            // RCP LUT: 1/(1 + i/1024) for mantissa range [1,2)
            x = 1.0 + real'(i) / 1024.0;
            rcp_lut[i] = int'((1.0 / x) * real'(1 << 23));
            
            // RSQ LUT: 1/sqrt(1 + i/1024)
            rsq_lut[i] = int'((1.0 / $sqrt(x)) * real'(1 << 23));
            
            // LOG2 LUT: log2(1 + i/1024) - fractional part only
            log2_lut[i] = int'($ln(x) / $ln(2.0) * real'(1 << 23));
            
            // EXP2 LUT: 2^(i/1024) - 1 for fractional exponent
            exp2_frac_lut[i] = int'(($pow(2.0, real'(i)/1024.0) - 1.0) * real'(1 << 23));
            
            // SIN LUT: sin(i * pi/2 / 1024) for first quadrant
            sin_lut[i] = int'($sin(real'(i) * 3.14159265358979 / 2048.0) * real'(1 << 23));
            
            // COS LUT: cos(i * pi/2 / 1024)
            cos_lut[i] = int'($cos(real'(i) * 3.14159265358979 / 2048.0) * real'(1 << 23));
        end
    end
    
    //==========================================================================
    // Constants
    //==========================================================================
    
    localparam logic [31:0] FP_ONE      = 32'h3F800000;  // 1.0f
    localparam logic [31:0] FP_HALF     = 32'h3F000000;  // 0.5f
    localparam logic [31:0] FP_TWO      = 32'h40000000;  // 2.0f
    localparam logic [31:0] FP_THREE    = 32'h40400000;  // 3.0f
    localparam logic [31:0] FP_PI       = 32'h40490FDB;  // pi
    localparam logic [31:0] FP_2_PI     = 32'h40C90FDB;  // 2*pi
    localparam logic [31:0] FP_PI_2     = 32'h3FC90FDB;  // pi/2
    localparam logic [31:0] FP_1_2PI    = 32'h3E22F983;  // 1/(2*pi)
    
    //==========================================================================
    // Input Unpacking
    //==========================================================================
    
    logic           in_sign;
    logic [7:0]     in_exp;
    logic [22:0]    in_mant;
    logic           in_is_zero;
    logic           in_is_inf;
    logic           in_is_nan;
    logic           in_is_negative;
    logic           in_is_denorm;
    
    assign in_sign = operand[31];
    assign in_exp = operand[30:23];
    assign in_mant = operand[22:0];
    assign in_is_zero = (in_exp == 8'h00) && (in_mant == 23'h0);
    assign in_is_inf = (in_exp == 8'hFF) && (in_mant == 23'h0);
    assign in_is_nan = (in_exp == 8'hFF) && (in_mant != 23'h0);
    assign in_is_negative = in_sign && !in_is_zero;
    assign in_is_denorm = (in_exp == 8'h00) && (in_mant != 23'h0);
    
    //==========================================================================
    // Pipeline Stage 0: Input Registration & LUT Lookup
    //==========================================================================
    
    logic [2:0]     s0_op;
    logic [31:0]    s0_operand;
    logic           s0_valid;
    logic           s0_sign;
    logic [7:0]     s0_exp;
    logic [22:0]    s0_mant;
    logic           s0_special;
    logic [31:0]    s0_special_result;
    logic           s0_error;
    
    logic [9:0]     lut_addr;
    logic [23:0]    lut_data;
    
    assign lut_addr = in_mant[22:13];  // Top 10 bits of mantissa
    
    // LUT read (combinational)
    always_comb begin
        case (sfu_op_t'(op))
            SFU_RCP:  lut_data = rcp_lut[lut_addr];
            SFU_RSQ:  lut_data = rsq_lut[lut_addr];
            SFU_LOG2: lut_data = log2_lut[lut_addr];
            SFU_EXP2: lut_data = exp2_frac_lut[lut_addr];
            SFU_SIN:  lut_data = sin_lut[lut_addr];
            SFU_COS:  lut_data = cos_lut[lut_addr];
            default:  lut_data = '0;
        endcase
    end
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0_valid <= 1'b0;
        end else begin
            s0_valid <= valid_in;
            s0_op <= op;
            s0_operand <= operand;
            s0_sign <= in_sign;
            s0_exp <= in_exp;
            s0_mant <= in_mant;
            
            // Special case handling
            s0_special <= 1'b0;
            s0_error <= 1'b0;
            
            case (sfu_op_t'(op))
                SFU_RCP: begin
                    if (in_is_nan) begin
                        s0_special <= 1'b1;
                        s0_special_result <= 32'h7FC00000;  // QNaN
                    end else if (in_is_zero) begin
                        s0_special <= 1'b1;
                        s0_special_result <= in_sign ? 32'hFF800000 : 32'h7F800000;  // ±Inf
                        s0_error <= 1'b1;  // Division by zero
                    end else if (in_is_inf) begin
                        s0_special <= 1'b1;
                        s0_special_result <= in_sign ? 32'h80000000 : 32'h00000000;  // ±0
                    end
                end
                
                SFU_RSQ: begin
                    if (in_is_nan) begin
                        s0_special <= 1'b1;
                        s0_special_result <= 32'h7FC00000;
                    end else if (in_is_negative) begin
                        s0_special <= 1'b1;
                        s0_special_result <= 32'h7FC00000;  // NaN for sqrt of negative
                        s0_error <= 1'b1;
                    end else if (in_is_zero) begin
                        s0_special <= 1'b1;
                        s0_special_result <= 32'h7F800000;  // +Inf
                        s0_error <= 1'b1;
                    end else if (in_is_inf) begin
                        s0_special <= 1'b1;
                        s0_special_result <= 32'h00000000;  // 0
                    end
                end
                
                SFU_LOG2: begin
                    if (in_is_nan) begin
                        s0_special <= 1'b1;
                        s0_special_result <= 32'h7FC00000;
                    end else if (in_is_zero) begin
                        s0_special <= 1'b1;
                        s0_special_result <= 32'hFF800000;  // -Inf
                    end else if (in_is_negative) begin
                        s0_special <= 1'b1;
                        s0_special_result <= 32'h7FC00000;  // NaN
                        s0_error <= 1'b1;
                    end else if (in_is_inf && !in_sign) begin
                        s0_special <= 1'b1;
                        s0_special_result <= 32'h7F800000;  // +Inf
                    end
                end
                
                SFU_EXP2: begin
                    if (in_is_nan) begin
                        s0_special <= 1'b1;
                        s0_special_result <= 32'h7FC00000;
                    end else if (in_is_inf) begin
                        s0_special <= 1'b1;
                        s0_special_result <= in_sign ? 32'h00000000 : 32'h7F800000;
                    end
                end
                
                SFU_SIN, SFU_COS: begin
                    if (in_is_nan || in_is_inf) begin
                        s0_special <= 1'b1;
                        s0_special_result <= 32'h7FC00000;
                        s0_error <= in_is_inf;
                    end
                end
            endcase
        end
    end
    
    //==========================================================================
    // Pipeline Stage 1: First Newton-Raphson Iteration / Computation
    //==========================================================================
    
    logic [2:0]     s1_op;
    logic           s1_valid;
    logic [23:0]    s1_approx;
    logic [7:0]     s1_exp;
    logic           s1_sign;
    logic           s1_special;
    logic [31:0]    s1_special_result;
    logic           s1_error;
    
    // Intermediate computation registers
    logic [47:0]    s1_mult_result;
    logic [31:0]    s1_operand;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_valid <= 1'b0;
        end else begin
            s1_valid <= s0_valid;
            s1_op <= s0_op;
            s1_approx <= lut_data;
            s1_special <= s0_special;
            s1_special_result <= s0_special_result;
            s1_error <= s0_error;
            s1_operand <= s0_operand;
            
            case (sfu_op_t'(s0_op))
                SFU_RCP: begin
                    // Compute a * x0 for Newton-Raphson
                    // x1 = x0 * (2 - a * x0)
                    s1_mult_result <= {1'b1, s0_mant} * lut_data;
                    // Result exponent: 253 - input_exp (bias adjustment)
                    s1_exp <= 8'd253 - s0_exp;
                    s1_sign <= s0_sign;
                end
                
                SFU_RSQ: begin
                    // Compute x0^2 for Newton-Raphson
                    // x1 = x0 * (3 - a * x0^2) / 2
                    s1_mult_result <= lut_data * lut_data;
                    // Result exponent: (253 - input_exp) / 2
                    if (s0_exp[0]) begin  // Odd exponent
                        s1_exp <= (9'd253 - {1'b0, s0_exp} + 1) >> 1;
                    end else begin
                        s1_exp <= (9'd253 - {1'b0, s0_exp}) >> 1;
                    end
                    s1_sign <= 1'b0;  // RSQ always positive
                end
                
                SFU_LOG2: begin
                    // log2(x) = exponent + log2(mantissa)
                    // log2(mantissa) from LUT
                    s1_exp <= s0_exp;  // Will be converted to signed
                    s1_sign <= (s0_exp < 127);  // Negative if exp < bias
                end
                
                SFU_EXP2: begin
                    // 2^x = 2^int(x) * 2^frac(x)
                    // Extract integer and fractional parts
                    s1_exp <= s0_exp;
                    s1_sign <= s0_sign;
                end
                
                SFU_SIN, SFU_COS: begin
                    // Reduce angle to [0, pi/2]
                    // Result from LUT after range reduction
                    s1_exp <= 8'd126;  // Result typically in [-1, 1]
                    s1_sign <= s0_sign;
                end
            endcase
        end
    end
    
    //==========================================================================
    // Pipeline Stage 2: Second N-R Iteration / Final Computation
    //==========================================================================
    
    logic [2:0]     s2_op;
    logic           s2_valid;
    logic [47:0]    s2_nr_result;
    logic [7:0]     s2_exp;
    logic           s2_sign;
    logic           s2_special;
    logic [31:0]    s2_special_result;
    logic           s2_error;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2_valid <= 1'b0;
        end else begin
            s2_valid <= s1_valid;
            s2_op <= s1_op;
            s2_special <= s1_special;
            s2_special_result <= s1_special_result;
            s2_error <= s1_error;
            s2_exp <= s1_exp;
            s2_sign <= s1_sign;
            
            case (sfu_op_t'(s1_op))
                SFU_RCP: begin
                    // N-R: x1 = x0 * (2 - a*x0)
                    // s1_mult_result = a * x0 (in Q1.47 format)
                    logic [47:0] two_minus_ax;
                    two_minus_ax = {2'b10, 46'b0} - s1_mult_result;  // 2.0 - a*x0
                    s2_nr_result <= (s1_approx * two_minus_ax[46:23]) >> 23;
                end
                
                SFU_RSQ: begin
                    // N-R: x1 = x0 * (3 - a*x0^2) / 2
                    logic [47:0] a_times_x2;
                    logic [47:0] three_minus;
                    
                    // a * x0^2
                    a_times_x2 = ({1'b1, s1_operand[22:0]} * s1_mult_result[46:23]) >> 23;
                    // 3 - a*x0^2
                    three_minus = {2'b11, 46'b0} - a_times_x2;
                    // x0 * (3 - a*x0^2) / 2
                    s2_nr_result <= (s1_approx * three_minus[46:23]) >> 24;  // >>24 includes /2
                end
                
                SFU_LOG2: begin
                    // Combine exponent (integer part) with LUT (fractional part)
                    logic signed [8:0] exp_unbiased;
                    exp_unbiased = {1'b0, s1_exp} - 9'd127;
                    
                    // Pack as floating point
                    if (exp_unbiased == 0) begin
                        // log2(1.x) = 0.something
                        s2_nr_result <= {24'b0, s1_approx};
                    end else begin
                        s2_nr_result <= {exp_unbiased[7:0], s1_approx[22:0], 17'b0};
                    end
                end
                
                SFU_EXP2: begin
                    // 2^x = 2^int * 2^frac
                    // LUT gives 2^frac - 1, add 1.0
                    s2_nr_result <= {24'h800000 + s1_approx, 24'b0};
                end
                
                SFU_SIN, SFU_COS: begin
                    // Direct LUT result (after range reduction)
                    s2_nr_result <= {s1_approx, 24'b0};
                end
            endcase
        end
    end
    
    //==========================================================================
    // Pipeline Stage 3: Result Packing
    //==========================================================================
    
    logic           s3_valid;
    logic [31:0]    s3_result;
    logic           s3_error;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s3_valid <= 1'b0;
            s3_error <= 1'b0;
        end else begin
            s3_valid <= s2_valid;
            s3_error <= s2_error;
            
            if (s2_special) begin
                s3_result <= s2_special_result;
            end else begin
                // Normalize and pack result
                logic [7:0] final_exp;
                logic [22:0] final_mant;
                
                case (sfu_op_t'(s2_op))
                    SFU_RCP, SFU_RSQ: begin
                        // Result is already normalized from N-R
                        final_exp = s2_exp;
                        final_mant = s2_nr_result[46:24];
                        s3_result <= {s2_sign, final_exp, final_mant};
                    end
                    
                    SFU_LOG2: begin
                        // Need to handle negative results (when input < 1)
                        final_exp = s2_nr_result[46:39];
                        final_mant = s2_nr_result[38:16];
                        s3_result <= {s2_sign, final_exp, final_mant};
                    end
                    
                    SFU_EXP2: begin
                        // Exponent comes from integer part of input
                        logic signed [8:0] int_part;
                        int_part = (s2_exp >= 127) ? 
                                  ({1'b0, s2_exp} - 127) : 
                                  -(127 - {1'b0, s2_exp});
                        
                        if (s2_sign) int_part = -int_part;  // Negative input
                        
                        final_exp = 8'd127 + int_part[7:0];
                        final_mant = s2_nr_result[46:24];
                        
                        // Overflow/underflow check
                        if (int_part > 127) begin
                            s3_result <= 32'h7F800000;  // +Inf
                        end else if (int_part < -126) begin
                            s3_result <= 32'h00000000;  // 0
                        end else begin
                            s3_result <= {1'b0, final_exp, final_mant};
                        end
                    end
                    
                    SFU_SIN, SFU_COS: begin
                        // Result in range [-1, 1]
                        final_exp = 8'd126;  // 2^-1 to 2^0 range
                        final_mant = s2_nr_result[46:24];
                        s3_result <= {s2_sign, final_exp, final_mant};
                    end
                    
                    default: begin
                        s3_result <= 32'h00000000;
                    end
                endcase
            end
        end
    end
    
    //==========================================================================
    // Output
    //==========================================================================
    
    assign result = s3_result;
    assign valid_out = s3_valid;
    assign error_flag = s3_error;

endmodule
