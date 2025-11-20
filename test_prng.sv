`timescale 1ns / 1ps

module test_prng();

    parameter CLK_PERIOD = 10;
    // Set this to a low number (e.g., 100) for fast testing
    // Set to 1200000 for the full run
    parameter TARGET_ITERATIONS = 1200000;  
    parameter string OUTPUT_PATH = "C:/Users/dell/OneDrive/Documents/prng_output.txt";
    
    logic clk = 1'b0;
    logic reset = 1'b1;
    logic start = 1'b0;
    logic [87:0] raw_bits;
    logic [43:0] debiased_bits;
    logic [6:0] debiased_count;
    logic [6:0] raw_count;
    logic bits_ready;
    
    int output_file;
    int iteration_count = 0;
    int i;
    string bits_line;
    int ones_count;
    
    // Instantiate the Unit Under Test (UUT)
    pr uut (
        .clk(clk),  
        .reset(reset),  
        .start(start),
        .raw_bits(raw_bits),  
        .debiased_bits(debiased_bits),
        .debiased_count(debiased_count),  
        .raw_count(raw_count),
        .bits_ready(bits_ready)
    );
    
    // Clock generator
    always #(CLK_PERIOD/2) clk = ~clk;
    
    initial begin
        $display("\n========================================");
        $display("PRNG Generation - CORRECTED VERSION");
        $display("========================================");
        $display("Target iterations: %0d", TARGET_ITERATIONS);
        $display("Output file: %s", OUTPUT_PATH);
        $display("========================================\n");
        
        output_file = $fopen(OUTPUT_PATH, "w");
        if (output_file == 0) begin
            $display("ERROR: Cannot open file: %s", OUTPUT_PATH);
            $finish;
        end
        
        $display("File opened successfully!\n");
        
        // Release reset
        #100 reset = 1'b0;
        $display("[%0t] Reset released", $time);
        
        // Start RK4 execution
        #50 start = 1'b1;
        $display("[%0t] Start asserted\n", $time);
        
        // Main loop - keep checking for bits_ready pulse
        while (iteration_count < TARGET_ITERATIONS) begin
            @(posedge clk);
            
            // bits_ready is a PULSE signal - only HIGH for 1 clock cycle
            if (bits_ready) begin
                iteration_count++;
                
                // Decide which bits to use based on debiased_count
                // This logic matches the Python fallback
                if (debiased_count < 64) begin
                    // Use RAW bits (88 bits)
                    bits_line = "";
                    ones_count = 0;
                    
                    for (i = 0; i < 88; i++) begin
                        bits_line = {bits_line, (raw_bits[i] ? "1" : "0")};
                        if (raw_bits[i]) ones_count++;
                    end
                    
                    // Show first 5 iterations with statistics
                    if (iteration_count <= 5) begin
                        $display("Iteration %0d: RAW (debiased_count=%0d < 64)", 
                                 iteration_count, debiased_count);
                        $display("  Ones: %0d/88 | Output: %s", ones_count, bits_line);
                    end
                    
                end else begin
                    // Use DEBIASED bits (variable length)
                    bits_line = "";
                    ones_count = 0;
                    
                    for (i = 0; i < debiased_count; i++) begin
                        bits_line = {bits_line, (debiased_bits[i] ? "1" : "0")};
                        if (debiased_bits[i]) ones_count++;
                    end
                    
                    // Show first 5 iterations with statistics
                    if (iteration_count <= 5) begin
                        $display("Iteration %0d: DEBIASED (count=%0d bits)", 
                                 iteration_count, debiased_count);
                        $display("  Ones: %0d/%0d | Output: %s", ones_count, debiased_count, bits_line);
                    end
                end
                
                // Write to file
                $fwrite(output_file, "%s\n", bits_line);
                
                // Progress update
                if (iteration_count % 20000 == 0) begin // Changed to 20k for less spam
                    $display("[%0t] Progress: %0d iterations", $time, iteration_count);
                end
            end
        end
        
        $fclose(output_file);
        
        $display("\n========================================");
        $display("PRNG GENERATION COMPLETE!");
        $display("========================================");
        $display("Total iterations: %0d", iteration_count);
        $display("Output file: %s", OUTPUT_PATH);
        $display("========================================\n");
        
        $finish;
    end
    
    // Timeout watchdog (e.g., 5 minutes for a long run)
    initial begin
        #300_000ms; // 300 seconds = 5 minutes
        $display("\nTIMEOUT at %0d iterations (5 minutes elapsed)", iteration_count);
        if (output_file != 0) $fclose(output_file);
        $finish;
    end

endmodule