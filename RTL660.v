`timescale 1ns / 1ps
// ---------------------------------------------------------------------------
// plfsr660_chip
//  - 660-bit programmable Fibonacci-style LFSR
//  - Serial-programmable tap mask P[659:0] and seed I[659:0]
//  - 660 FFs for state_reg; 660 for P_reg; 660 for seed_reg
//  - 30-bit multiplexed output Q[29:0] over 22 cycles per 660-bit state
//
//  Indexing convention:
//    state_reg[659] = MSB (position 1)
//    state_reg[0]   = LSB (position 660)
//
//  Taps used in TB (through programmable P_reg):
//    indices 0,1,18,19  →  positions 660,659,642,641 from MSB side
//
//  LFSR update in RUN mode (once every 22 clocks):
//    feedback_bit = XOR over all state_reg[i] where P_reg[i] = 1
//    next_state   = { feedback_bit, state_reg[659:1] }
//
//  With seed = 1 (only LSB = 1) this gives:
//    1 → 2^659 → 2^658 → 2^657 → ...
//
//  Scan chain:
//    When TEST_MODE=1 and SCAN_EN=1,
//    all flops form one scan chain:
//      {flag_err, chunk_ctr, state_reg, seed_reg, P_reg}
//    Shift-right structure:
//      MSB  <= SCAN_IN
//      LSB  -> SCAN_OUT
// ---------------------------------------------------------------------------
module plfsr660_chip (
    // Pin  1
    input  wire        RESET_N,       // active-low reset

    // Pins 2-3
    input  wire [1:0]  MODE,          // 00=IDLE, 01=LOAD_P, 10=LOAD_I, 11=RUN

    // Pin 4
    input  wire        SER_IN,        // serial input for P and I (MSB-first)

    // Pin 5
    output wire        BIT_OUT,       // LSB of current LFSR state

    // Pins 6-7
    output wire        FLAG_ZERO,     // 1 if state_reg == 0
    output wire        FLAG_RUNNING,  // 1 if MODE == RUN

    // Pin 8
    input  wire        CLK,           // main clock

    // Pin 9
    output wire        FLAG_ERROR,    // sticky error (zero lock)

    // Pin 10
    input  wire        TEST_MODE,     // 1 => scan enabled

    // Pin 11
    input  wire        SCAN_CLK,      // present for pins; not used logically

    // Pin 12
    input  wire        SCAN_EN,       // 1 => shift scan chain

    // Pins 13-14
    input  wire        SCAN_IN,       // serial scan input
    output wire        SCAN_OUT,      // serial scan output

    // Pins 15-22 and 24-45 → 30 data outputs
    output wire [29:0] Q,

    // Pin 23
    input  wire        GND,           // not used logically

    // Pin 46
    output wire        MON_CLK_REMOTE,

    // Pin 47
    output wire        MON_FEEDBACK,

    // Pins 48-51
    output wire        MON_Q3,
    output wire        MON_Q7,
    output wire        MON_Q15,
    output wire        MON_Q23,

    // Pin 52
    output wire        MON_P3,

    // Pin 53
    input  wire        VDD,           // not used logically

    // Pins 54-56
    output wire        MON_P7,
    output wire        MON_P15,
    output wire        MON_P23,

    // Pins 57-60
    output wire        MON_SEED3,
    output wire        MON_SEED7,
    output wire        MON_SEED15,
    output wire        MON_SEED23
);

    // -----------------------------------------------------------------------
    // Parameters
    // -----------------------------------------------------------------------
    localparam integer N            = 660;
    localparam integer CHUNK_BITS   = 30;
    localparam integer CHUNKS       = N / CHUNK_BITS; // 22 chunks
    localparam integer CHUNK_CTR_W  = 5;              // enough for 0..21

    localparam [1:0]   MODE_IDLE    = 2'b00;
    localparam [1:0]   MODE_LOAD_P  = 2'b01;
    localparam [1:0]   MODE_LOAD_I  = 2'b10;
    localparam [1:0]   MODE_RUN     = 2'b11;

    // Scan length: P + seed + state + chunk_ctr + flag_err
    localparam integer SCAN_LEN     = (3*N) + CHUNK_CTR_W + 1; // 1986 bits

    // -----------------------------------------------------------------------
    // Registers
    // -----------------------------------------------------------------------
    reg [N-1:0]           P_reg;       // tap mask (programmable)
    reg [N-1:0]           seed_reg;    // initialization value
    reg [N-1:0]           state_reg;   // 660-bit LFSR state
    reg [CHUNK_CTR_W-1:0] chunk_ctr;   // 0..21 → 22 chunks per state
    reg                   flag_err;    // sticky error flag

    // -----------------------------------------------------------------------
    // Feedback: XOR of bits selected by P_reg
    // -----------------------------------------------------------------------
    wire [N-1:0] tapped_bits;
    wire         feedback_bit;

    assign tapped_bits  = state_reg & P_reg;
    assign feedback_bit = ^tapped_bits;

    // -----------------------------------------------------------------------
    // 30-bit chunked output over 22 cycles
    // -----------------------------------------------------------------------
    reg  [CHUNK_BITS-1:0]   Q_window;
    wire [CHUNK_CTR_W+4:0]  base_index; // enough bits for 0..659

    assign base_index = chunk_ctr * CHUNK_BITS;

    always @* begin
        Q_window = state_reg[ base_index +: CHUNK_BITS ];
    end

    assign Q            = Q_window;
    assign BIT_OUT      = state_reg[0];
    assign FLAG_ZERO    = (state_reg == {N{1'b0}});
    assign FLAG_RUNNING = (MODE == MODE_RUN);
    assign FLAG_ERROR   = flag_err;

    // -----------------------------------------------------------------------
    // Monitor outputs
    // -----------------------------------------------------------------------
    assign MON_CLK_REMOTE = CLK;
    assign MON_FEEDBACK   = feedback_bit;

    assign MON_Q3    = state_reg[3];
    assign MON_Q7    = state_reg[7];
    assign MON_Q15   = state_reg[15];
    assign MON_Q23   = state_reg[23];

    assign MON_P3    = P_reg[3];
    assign MON_P7    = P_reg[7];
    assign MON_P15   = P_reg[15];
    assign MON_P23   = P_reg[23];

    assign MON_SEED3   = seed_reg[3];
    assign MON_SEED7   = seed_reg[7];
    assign MON_SEED15  = seed_reg[15];
    assign MON_SEED23  = seed_reg[23];

    // -----------------------------------------------------------------------
    // Functional next-state logic
    // -----------------------------------------------------------------------
    reg [N-1:0]           P_next;
    reg [N-1:0]           seed_next;
    reg [N-1:0]           state_next;
    reg [CHUNK_CTR_W-1:0] chunk_next;
    reg                   flag_err_next;

    always @* begin
        // defaults: hold
        P_next        = P_reg;
        seed_next     = seed_reg;
        state_next    = state_reg;
        chunk_next    = chunk_ctr;
        flag_err_next = flag_err;

        case (MODE)
            MODE_IDLE: begin
                // hold everything
            end

            MODE_LOAD_P: begin
                // SHIFT-LEFT, SER_IN into LSB.
                // TB sends MSB-first: after N clocks P_reg == P_val.
                P_next     = {P_reg[N-2:0], SER_IN};
                chunk_next = {CHUNK_CTR_W{1'b0}};
            end

            MODE_LOAD_I: begin
                // SHIFT-LEFT, SER_IN into LSB.
                // After N clocks: seed_reg == I_seed, state_reg == I_seed.
                seed_next  = {seed_reg[N-2:0], SER_IN};
                state_next = {state_reg[N-2:0], SER_IN};
                chunk_next = {CHUNK_CTR_W{1'b0}};
            end

            MODE_RUN: begin
                // One new N-bit state every CHUNKS (22) RUN clocks.
                if (chunk_ctr == CHUNKS-1) begin
                    // Fibonacci-like right shift:
                    //   new MSB   = feedback_bit
                    //   [N-2:0]   = old [N-1:1]
                    state_next = {feedback_bit, state_reg[N-1:1]};
                    chunk_next = {CHUNK_CTR_W{1'b0}};

                    if ({feedback_bit, state_reg[N-1:1]} == {N{1'b0}})
                        flag_err_next = 1'b1;
                end else begin
                    chunk_next = chunk_ctr + {{(CHUNK_CTR_W-1){1'b0}},1'b1};
                end
            end

            default: begin
                // treat as IDLE
            end
        endcase
    end

    // -----------------------------------------------------------------------
    // Scan chain: {flag_err, chunk_ctr, state_reg, seed_reg, P_reg}
    // -----------------------------------------------------------------------
    wire [SCAN_LEN-1:0] scan_vec_in;
    assign scan_vec_in = {flag_err, chunk_ctr, state_reg, seed_reg, P_reg};

    assign SCAN_OUT = scan_vec_in[0]; // LSB

    // -----------------------------------------------------------------------
    // Sequential logic with scan override (uses CLK, not SCAN_CLK)
    // -----------------------------------------------------------------------
    always @(posedge CLK or negedge RESET_N) begin
        if (!RESET_N) begin
            P_reg      <= {N{1'b0}};
            seed_reg   <= {N{1'b0}};
            state_reg  <= {N{1'b0}};
            chunk_ctr  <= {CHUNK_CTR_W{1'b0}};
            flag_err   <= 1'b0;
        end else if (TEST_MODE && SCAN_EN) begin
            // Scan shift-right: SCAN_IN enters MSB
            {flag_err, chunk_ctr, state_reg, seed_reg, P_reg} <=
                {SCAN_IN, scan_vec_in[SCAN_LEN-1:1]};
        end else begin
            P_reg      <= P_next;
            seed_reg   <= seed_next;
            state_reg  <= state_next;
            chunk_ctr  <= chunk_next;
            flag_err   <= flag_err_next;
        end
    end

endmodule
