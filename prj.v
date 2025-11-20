`timescale 1ns / 1ps

module pr(
    input  wire clk,
    input  wire reset,
    input  wire start,
    output reg [6:0]  debiased_count,    
    // --- NEW ---
    // Output for the "Random Blink" Aliveness Check LED
    output reg        o_random_led
    // --- END NEW ---
);
    reg [87:0] raw_bits;
    reg [43:0] debiased_bits;
    reg [6:0]  raw_count;
    reg        bits_ready;
    // --- Q6.58 fixed-point (1 sign, 5 int, 58 frac) ---
    parameter SCALE_BITS = 58;
    parameter signed [63:0] SCALE = 64'd288230376151711744; // 2^58
    
    // Parameters (scaled to Q6.58)
    parameter signed [63:0] a     = 64'd432345564227567616; // 1.5
    parameter signed [63:0] b     = 64'd835868090839964032; // 2.9
    parameter signed [63:0] c     = 64'd201761263306198208; // 0.7
    parameter signed [63:0] d     = 64'd576460752303423488; // 2.0
    parameter signed [63:0] e     = 64'd340111843859021824; // 1.18
    parameter signed [63:0] ky    = 64'd28823037615171174;  // 0.1
    parameter signed [63:0] f     = 64'd852313813950022656; // 2.957
    parameter signed [63:0] g     = 64'd6629298651489369088; // 23.0
    parameter signed [63:0] h     = 64'd141232884314338752; // 0.49
    parameter signed [63:0] alpha = 64'd288230376151711744; // 1.0
    parameter signed [63:0] beta  = 64'd288230376151711744; // 1.0
    parameter signed [63:0] m     = 64'd144115188075855872; // 0.5
    parameter signed [63:0] n     = 64'd288230376151711744; // 1.0
    parameter signed [63:0] r     = 64'd432345564227567616; // 1.5
    parameter signed [63:0] h_step= 64'd2882303761517117;     // 0.01

    // --- Parameters for seeding, division, and bit extraction ---
    parameter signed [63:0] BASE_X = 64'd288806836904015168; // 1.002
    parameter signed [63:0] BASE_Y = 64'd288316836904015168; // 1.0003
    parameter signed [63:0] BASE_Z = 64'd28823037615171;     // 0.0001
    parameter signed [63:0] BASE_W = 64'd1441151880758558;   // 0.005
    parameter signed [63:0] ONE_SIXTH = 64'd48038396025285290; // 1/6
    
    parameter signed [63:0] K_1E14_Q48_16 = 64'd6553600000000000000;

    // --- State variables ---
    reg signed [63:0] x, y, z, w;
    reg signed [63:0] k1_x, k1_y, k1_z, k1_w;
    reg signed [63:0] k2_x, k2_y, k2_z, k2_w;
    reg signed [63:0] k3_x, k3_y, k3_z, k3_w;
    reg signed [63:0] k4_x, k4_y, k4_z, k4_w;

    // --- Temporary registers for serial FSM ---
    reg signed [63:0] xt, yt, zt, wt; // Inputs for derivative calc
    reg signed [63:0] dx, dy, dz, dw; // Derivative results
    reg signed [63:0] h_dx, h_dy, h_dz, h_dw; // h_step * derivative
    
    // --- Temporary registers for serial multiplication ---
    reg signed [63:0] tanh_x_pipe, tanh_y_pipe, tanh_z_pipe, W_pipe;
    reg signed [63:0] temp_a, temp_b, temp_c, temp_d, temp_e, temp_ky;
    reg signed [63:0] temp_f, temp_g, temp_h, temp_m, temp_n;
    
    // --- FSM State Registers ---
    reg [2:0] k_stage; // 0=Idle, 1=k1, 2=k2, 3=k3, 4=k4, 5=Update, 6=Extract
    reg [4:0] op_stage; // 0-23: Sub-stages for serial calculation

    // --- LFSR ---
    reg [63:0] lfsr;
    always @(posedge clk or posedge reset) begin
        if (reset) lfsr <= 64'hA5A5A5A5A5A5A5A5;
        else lfsr <= {lfsr[62:0], lfsr[63]^lfsr[62]^lfsr[60]^lfsr[59]};
    end
    wire signed [63:0] noise = {48'd0, lfsr[15:0]};

    // --- Reusable Hardware Functions (unchanged) ---
    function signed [63:0] mult_q58;
        input signed [63:0] a, b;
        reg signed [127:0] temp;
        begin
            temp = a * b;
            mult_q58 = temp[121:SCALE_BITS];
        end
    endfunction

    function signed [63:0] div_by_6;
        input signed [63:0] v;
        begin
            div_by_6 = mult_q58(v, ONE_SIXTH);
        end
    endfunction

    function signed [63:0] approx_tanh_r;
        input signed [63:0] i;
        reg signed [63:0] ri;
        begin
            ri = mult_q58(r, i);
            if (ri >= d) approx_tanh_r = alpha;
            else if (ri <= -d) approx_tanh_r = -alpha;

            // --- FIX 3 ---
            // Use Arithmetic Shift (>>) for signed numbers, not (>>>)
            else if (ri >= 0) approx_tanh_r = mult_q58(ri, (alpha - (ri >> 2)));
            else approx_tanh_r = mult_q58(ri, (alpha + (ri >> 2)));
            // --- END FIX 3 ---
            
        end
    endfunction

    function signed [63:0] W;
        input signed [63:0] phi;
        reg signed [63:0] abs_phi;
        begin
            abs_phi = (phi < 0) ? -phi : phi;
            W = alpha - mult_q58(beta, abs_phi);
        end
    endfunction

    // ========== RE-ARCHITECTED SERIAL FSM ==========
    always @(posedge clk or posedge reset) begin
        if(reset) begin
            // --- FIX 1 ---
            // Use ADD for seeding, not XOR
            x <= BASE_X + {48'd0, lfsr[15:0]};
            y <= BASE_Y + {48'd0, lfsr[31:16]};
            z <= BASE_Z + {48'd0, lfsr[47:32]};
            w <= BASE_W + {48'd0, lfsr[63:48]};
            // --- END FIX 1 ---
            
            // Reset all state registers
            k_stage <= 0;
            op_stage <= 0;
            k1_x <= 0; k1_y <= 0; k1_z <= 0; k1_w <= 0;
            k2_x <= 0; k2_y <= 0; k2_z <= 0; k2_w <= 0;
            k3_x <= 0; k3_y <= 0; k3_z <= 0; k3_w <= 0;
            k4_x <= 0; k4_y <= 0; k4_z <= 0; k4_w <= 0;
        end 
        else if (start) begin
            // This FSM manages the main RK4 stages (k1, k2, k3, k4, update)
            case (k_stage)
                // --- STAGE 0: IDLE ---
                0: begin
                    xt <= x; yt <= y; zt <= z; wt <= w; // Load inputs for k1
                    k_stage <= 1; // Start k1 calculation
                    op_stage <= 0; // Reset sub-stage FSM
                end
                
                // --- STAGE 1: Calculate k1 ---
                1: begin
                    if (op_stage == 23) begin // k1 calc is done
                        k1_x <= h_dx; k1_y <= h_dy; k1_z <= h_dz; k1_w <= h_dw;
                        // Load inputs for k2: x + 0.5*k1
                        xt <= x + (h_dx >>> 1);
                        yt <= y + (h_dy >>> 1);
                        zt <= z + (h_dz >>> 1);
                        wt <= w + (h_dw >>> 1);
                        k_stage <= 2; // Move to k2
                        op_stage <= 0; // Reset sub-stage
                    end
                end
                
                // --- STAGE 2: Calculate k2 ---
                2: begin
                    if (op_stage == 23) begin // k2 calc is done
                        k2_x <= h_dx; k2_y <= h_dy; k2_z <= h_dz; k2_w <= h_dw;
                        // Load inputs for k3: x + 0.5*k2
                        xt <= x + (h_dx >>> 1);
                        yt <= y + (h_dy >>> 1);
                        zt <= z + (h_dz >>> 1);
                        wt <= w + (h_dw >>> 1);
                        k_stage <= 3; // Move to k3
                        op_stage <= 0; // Reset sub-stage
                    end
                end

                // --- STAGE 3: Calculate k3 ---
                3: begin
                    if (op_stage == 23) begin // k3 calc is done
                        k3_x <= h_dx; k3_y <= h_dy; k3_z <= h_dz; k3_w <= h_dw;
                        // Load inputs for k4: x + k3
                        xt <= x + h_dx;
                        yt <= y + h_dy;
                        zt <= z + h_dz;
                        wt <= w + h_dw;
                        k_stage <= 4; // Move to k4
                        op_stage <= 0; // Reset sub-stage
                    end
                end
                
                // --- STAGE 4: Calculate k4 ---
                4: begin
                    if (op_stage == 23) begin // k4 calc is done
                        k4_x <= h_dx; k4_y <= h_dy; k4_z <= h_dz; k4_w <= h_dw;
                        k_stage <= 5; // Move to Update
                        op_stage <= 0; // Not needed, but good practice
                    end
                end
                
                // --- STAGE 5: Final RK4 Update ---
                5: begin
                    // --- FIX 2 ---
                    // Use ADD for noise, not XOR
                    x <= x + div_by_6(k1_x + (k2_x << 1) + (k3_x << 1) + k4_x) + noise;
                    y <= y + div_by_6(k1_y + (k2_y << 1) + (k3_y << 1) + k4_y) + {noise[62:0], noise[63]};
                    z <= z + div_by_6(k1_z + (k2_z << 1) + (k3_z << 1) + k4_z) + {noise[61:0], noise[63:62]};
                    w <= w + div_by_6(k1_w + (k2_w << 1) + (k3_w << 1) + k4_w) + {noise[60:0], noise[63:61]};
                    // --- END FIX 2 ---
                    
                    k_stage <= 6; // Move to Bit Extraction
                end
                
                // --- STAGE 6: Bit Extraction ---
                // This stage is pipelined with the start of the next k1 calc
                6: begin
                    k_stage <= 0; // Go back to IDLE to start next iteration
                end

                default: k_stage <= 0;
            endcase

            // This FSM serializes the 23-step derivative calculation
            // It runs continuously as long as k_stage is 1, 2, 3, or 4
            if (k_stage >= 1 && k_stage <= 4) begin
                case(op_stage)
                    // --- Tanh/W Calculations ---
                    0:  tanh_x_pipe <= approx_tanh_r(xt);
                    1:  tanh_y_pipe <= approx_tanh_r(yt);
                    2:  tanh_z_pipe <= approx_tanh_r(zt);
                    3:  W_pipe      <= W(wt);
                    // --- Serial Multiplications ---
                    4:  temp_a <= mult_q58(a, tanh_x_pipe);
                    5:  temp_b <= mult_q58(b, tanh_y_pipe);
                    6:  temp_c <= mult_q58(c, tanh_z_pipe);
                    7:  temp_d <= mult_q58(d, tanh_x_pipe);
                    8:  temp_e <= mult_q58(e, tanh_y_pipe);
                    9:  temp_ky<= mult_q58(ky, W_pipe);
                    10: temp_f <= mult_q58(f, tanh_x_pipe);
                    11: temp_g <= mult_q58(g, tanh_y_pipe);
                    12: temp_h <= mult_q58(h, tanh_z_pipe);
                    13: temp_m <= mult_q58(m, yt); // Note: uses yt
                    14: temp_n <= mult_q58(n, wt); // Note: uses wt
                    // --- Derivative Calculations ---
                    15: dx <= -xt + temp_a + temp_b - temp_c;
                    16: dy <= -yt - temp_d + temp_e + temp_ky;
                    17: dz <= -zt + temp_f - temp_g + temp_h;
                    18: dw <= temp_m - temp_n;
                    // --- h_step * Derivative Multiplications ---
                    19: h_dx <= mult_q58(h_step, dx);
                    20: h_dy <= mult_q58(h_step, dy);
                    21: h_dz <= mult_q58(h_step, dz);
                    22: h_dw <= mult_q58(h_step, dw);
                    // --- Loop ---
                    23: op_stage <= 0; // Will be incremented to 0 by default
                    default: op_stage <= 0;
                endcase
                op_stage <= op_stage + 1; // Increment sub-stage
            end
            
        end // if(start)
    end // always


    // ========== BIT EXTRACTION (Pipelined) ==========
    // (This logic is unchanged)
    reg signed [127:0] scaled_x_128, scaled_y_128, scaled_z_128, scaled_w_128;
    reg signed [53:0] int_x, int_y, int_z, int_w;
    reg [87:0] internal_raw_bits;
    reg bits_extracted;

    always @(posedge clk or posedge reset) begin
        if(reset) begin
            internal_raw_bits <= 0;
            bits_extracted <= 0;
            raw_count <= 0;
            scaled_x_128 <= 0; scaled_y_128 <= 0; scaled_z_128 <= 0; scaled_w_128 <= 0;
            int_x <= 0; int_y <= 0; int_z <= 0; int_w <= 0;
        end 
        else if (start && k_stage == 6) begin 
            scaled_x_128 = $signed(x) * $signed(K_1E14_Q48_16);
            scaled_y_128 = $signed(y) * $signed(K_1E14_Q48_16);
            scaled_z_128 = $signed(z) * $signed(K_1E14_Q48_16);
            scaled_w_128 = $signed(w) * $signed(K_1E14_Q48_16);

            int_x = scaled_x_128[127:74];
            int_y = scaled_y_128[127:74];
            int_z = scaled_z_128[127:74];
            int_w = scaled_w_128[127:74];
            
            internal_raw_bits[0]  = int_x[0];  internal_raw_bits[1]  = int_y[0];
            internal_raw_bits[2]  = int_z[0];  internal_raw_bits[3]  = int_w[0];
            internal_raw_bits[4]  = int_x[1];  internal_raw_bits[5]  = int_y[1];
            internal_raw_bits[6]  = int_z[1];  internal_raw_bits[7]  = int_w[1];
            internal_raw_bits[8]  = int_x[2];  internal_raw_bits[9]  = int_y[2];
            internal_raw_bits[10] = int_z[2];  internal_raw_bits[11] = int_w[2];
            internal_raw_bits[12] = int_x[3];  internal_raw_bits[13] = int_y[3];
            internal_raw_bits[14] = int_z[3];  internal_raw_bits[15] = int_w[3];
            internal_raw_bits[16] = int_x[4];  internal_raw_bits[17] = int_y[4];
            internal_raw_bits[18] = int_z[4];  internal_raw_bits[19] = int_w[4];
            internal_raw_bits[20] = int_x[5];  internal_raw_bits[21] = int_y[5];
            internal_raw_bits[22] = int_z[5];  internal_raw_bits[23] = int_w[5];
            internal_raw_bits[24] = int_x[6];  internal_raw_bits[25] = int_y[6];
            internal_raw_bits[26] = int_z[6];  internal_raw_bits[27] = int_w[6];
            internal_raw_bits[28] = int_x[7];  internal_raw_bits[29] = int_y[7];
            internal_raw_bits[30] = int_z[7];  internal_raw_bits[31] = int_w[7];
            internal_raw_bits[32] = int_x[8];  internal_raw_bits[33] = int_y[8];
            internal_raw_bits[34] = int_z[8];  internal_raw_bits[35] = int_w[8];
            internal_raw_bits[36] = int_x[9];  internal_raw_bits[37] = int_y[9];
            internal_raw_bits[38] = int_z[9];  internal_raw_bits[39] = int_w[9];
            internal_raw_bits[40] = int_x[10]; internal_raw_bits[41] = int_y[10];
            internal_raw_bits[42] = int_z[10]; internal_raw_bits[43] = int_w[10];
            internal_raw_bits[44] = int_x[11]; internal_raw_bits[45] = int_y[11];
            internal_raw_bits[46] = int_z[11]; internal_raw_bits[47] = int_w[11];
            internal_raw_bits[48] = int_x[12]; internal_raw_bits[49] = int_y[12];
            internal_raw_bits[50] = int_z[12]; internal_raw_bits[51] = int_w[12];
            internal_raw_bits[52] = int_x[13]; internal_raw_bits[53] = int_y[13];
            internal_raw_bits[54] = int_z[13]; internal_raw_bits[55] = int_w[13];
            internal_raw_bits[56] = int_x[14]; internal_raw_bits[57] = int_y[14];
            internal_raw_bits[58] = int_z[14]; internal_raw_bits[59] = int_w[14];
            internal_raw_bits[60] = int_x[15]; internal_raw_bits[61] = int_y[15];
            internal_raw_bits[62] = int_z[15]; internal_raw_bits[63] = int_w[15];
            internal_raw_bits[64] = int_x[16]; internal_raw_bits[65] = int_y[16];
            internal_raw_bits[66] = int_z[16]; internal_raw_bits[67] = int_w[16];
            internal_raw_bits[68] = int_x[17]; internal_raw_bits[69] = int_y[17];
            internal_raw_bits[70] = int_z[17]; internal_raw_bits[71] = int_w[17];
            internal_raw_bits[72] = int_x[18]; internal_raw_bits[73] = int_y[18];
            internal_raw_bits[74] = int_z[18]; internal_raw_bits[75] = int_w[18];
            internal_raw_bits[76] = int_x[19]; internal_raw_bits[77] = int_y[19];
            internal_raw_bits[78] = int_z[19]; internal_raw_bits[79] = int_w[19];
            internal_raw_bits[80] = int_x[20]; internal_raw_bits[81] = int_y[20];
            internal_raw_bits[82] = int_z[20]; internal_raw_bits[83] = int_w[20];
            internal_raw_bits[84] = int_x[21]; internal_raw_bits[85] = int_y[21];
            internal_raw_bits[86] = int_z[21]; internal_raw_bits[87] = int_w[21];
            
            raw_bits <= internal_raw_bits;
            raw_count <= 88;
            bits_extracted <= 1; // Signal the debiaser
        end else begin
            bits_extracted <= 0;
        end
    end


    // ========== VON NEUMANN DEBIASING (Pipelined) ==========
    // This logic runs in parallel with k_stage 0
    integer i, j;
    reg [87:0] internal_debiased_bits; 

    always @(posedge clk or posedge reset) begin
        if(reset) begin
            internal_debiased_bits <= 0;
            debiased_count <= 0;
            debiased_bits <= 0;
            bits_ready <= 0;
            
            // --- NEW ---
            o_random_led <= 0; // Reset the LED
            // --- END NEW ---
            
        end 
        // When the 'bits_extracted' pulse arrives...
        else if (bits_extracted) begin 
            internal_debiased_bits = 0;
            j = 0;
            
            // Perform Von Neumann debiasing
            for(i = 0; i < 88; i = i + 2) begin
                if(internal_raw_bits[i] !== internal_raw_bits[i+1]) begin
                    if (j < 44) begin 
                        internal_debiased_bits[j] = internal_raw_bits[i];
                        j = j + 1;
                    end
                end
            end
            
            debiased_count <= j;
            debiased_bits <= internal_debiased_bits[43:0];
            bits_ready <= 1; // Signal testbench that a result is ready
            
            // --- NEW ---
            // Update the LED with the XOR sum of all new raw bits
            // This will make it flicker randomly if the PRNG is working
            o_random_led <= ^internal_raw_bits;
            // --- END NEW ---
            
        end else begin
            bits_ready <= 0;
        end
    end
    
endmodule