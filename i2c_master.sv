module i2c_master(
    input logic clk, reset,
    input logic [7:0] din,
    input logic [15:0] dvsr,
    input logic [2:0] cmd,
    input logic wr_i2c,
    output tri scl,
    inout tri sda,
    output logic ready, done_tick, ack,
    output logic [7:0] dout
  );
  // commands
  localparam START_CMD    = 3'b000;
  localparam WR_CMD       = 3'b001;
  localparam RD_CMD       = 3'b010;
  localparam STOP_CMD     = 3'b011;
  localparam RESTART_CMD  = 3'b100;
  // fsm state types
  typedef enum {
            idle, hold, start1, start2, data1, data2, data3, data4,
            data_end, restart, stop1, stop2
          } state_type;
  //declaration
  state_type state_reg, state_next;
  logic [15:0] c_reg, c_next;
  logic [15:0] qutr, half;
  logic [8:0] tx_reg, tx_next;
  logic [8:0] rx_reg, rx_next;
  logic [2:0] cmd_reg, cmd_next;
  logic [3:0] bit_reg, bit_next;
  logic sda_out, scl_out, sda_reg, scl_reg, data_phase;
  logic done_tick_i, ready_i;
  logic into, nack;

  //******************************************************
  // output control logic
  //******************************************************
  // buffer for sda and scl lines
  always_ff @(posedge clk, posedge reset)
    if (reset)
    begin
      sda_reg <= 1'b1;
      scl_reg <= 1'b1;
    end
    else
    begin
      sda_reg <= sda_out;
      scl_reg <= scl_out;
    end
    // only master drives scl lines
    assign scl = (scl_reg) ? 1'bz : 1'b0;
    //sda are with pull-up resistors and become high when not driven 
    // "into" signal asserted when sda into master
    assign into =   (data_phase && cmd_reg==RD_CMD && bit_reg<8) || 
                    (data_phase && cmd_reg==WR_CMD && bit_reg==8);
    assign sda =   (into || sda_reg) ? 1'bz : 1'b0;
    // output
    assign dout = rx_reg[8:1];
    assign ack = rx_reg[0]; // obtained from slave in write
    assign nack = din[0];
    //***************************************************
    // fsmd for transmitting three bytes
    //***************************************************
    // registers 
    always_ff @(posedge clk, posedge reset)
        if (reset) begin
            state_reg <= idle;
            c_reg <= 0;
            bit_reg <= 0;
            cmd_reg <= 0;
            tx_reg <= 0;
            rx_reg <= 0;

        end
        else begin
            state_reg <= state_next;
            c_reg <= c_next;
            bit_reg <= bit_next;
            cmd_reg <= cmd_next;
            tx_reg <= tx_next;
            rx_reg <= rx_next;
        end
    assign qutr = dvsr;
    assign half = {qutr[14:0],1'b0}; // half = 2* qutr

    // next-state logic
    always_comb
    begin
        state_next = state_reg;
        c_next = c_reg + 1;     // timer counts continuously
        bit_next = bit_reg;
        tx_next = tx_reg;
        rx_next = rx_reg;
        cmd_next = cmd_reg;
        done_tick_i = 1'b0;
        ready_i = 1'b0;
        scl_out = 1'b1;
        sda_out = 1'b1;
        data_phase = 1'b0;
        case (state_reg)
            idle: begin
                ready_i = 1'b1;
                if (wr_i2c && cmd==START_CMD) begin // start
                    state_next = start1;
                    c_next = 0
                end
            end
            start1: begin       // start condition
                sda_out = 1'b0;
                if(c_reg==half) begin
                    c_next = 0;
                    state_next = start2;
                end
            end
            start2: begin
                sda_out = 1'b0;
                scl_out = 1'b0;
                if (c_reg==qutr)begin
                    c_next = 0;
                    state_next = hold;
                end
            end
            hold: begin                         // in progress: prepared for next op
                ready_i = 1'b1;
                sda_out = 1'b0;
                scl_out = 1'b0;
                if (wr_i2c) begin
                    cmd_next = cmd;
                    c_next = 0;
                    case (cmd)
                        RESTART_CMD, START_CMD:         // start; error (restart?)
                            state_next = restart;
                        STOP_CMD:                       // stop
                            state_next = stop1;
                        default: begin                  // read/write a byte
                            bit_next = 0;
                            state_next = data1;
                            tx_next = {din,nack};       // nack used as NACK in read.
                        end
                    endcase
                end // end if
            end
            data1: begin
                sda_out = tx_reg[8];
                scl_out = 1'b0;
                data_phase = 1'b1;
                if (c_reg==qutr) begin
                    c_next = 0;
                    state_next = data2;
                end 
            end
            data2: begin
                sda_out = tx_reg[8];
                data_phase = 1'b1;
                if (c_reg==qutr) begin
                    c_next = 0;
                    state_next = data3;
                    rx_next = {rx_reg[7:0], sda}; // shift data in
                end // end if
            end
            data3: begin
                sda_out = tx_reg[8];
                data_phase = 1'b1;
                if (c_reg==qutr) begin
                    c_next = 0;
                    state_next = data4;
                end // end if
            end
            data4: begin
                sda_out = tx_reg[8];
                scl_out = 1'b0;
                data_phase = 1'b1;
                if (c_reg==qutr) begin
                    c_next = 0;
                    if (bit_reg==8) begin  // done with 8 data bits + 1 ack
                        state_next = data_end; // hold
                        done_tick_i = 1'b1;
                    end
                    else begin
                        tx_next = {tx_reg[7:0],1'b0};
                        bit_next = bit_reg + 1;
                        state_next = data1;
                    end  // end else
                end // end if
            end
            data_end: begin
                sda_out = 1'b0;
                scl_out = 1'b0;
                if (c_reg==qutr) begin
                    c_next = 0;
                    state_next = hold;
                end // end if
            end
            restart:                // generate idle condition
                if (c_reg==half) begin
                    c_next = 0;
                    state_next = start1;
                end
            stop1: begin
                sda_out = 1'b0;
                if (c_reg==half)begin
                    c_next = 0;
                    state_next = stop2;
                end // end if
            end
            default:    // stop2 (for turnaround time)
                if (c_reg==half)
                    state_next = idle;
        endcase
        assign done_tick = done_tick_i;
        assign ready = ready_i;     

    end
endmodule
