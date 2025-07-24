// Baud Rate Generator for 115200 baud with 16x oversampling (12 MHz clock)
module baud_gen_115200 (
    input  wire clk,       // 12 MHz system clock
    input  wire rst,       // synchronous reset (active high)
    output reg  s_tick     // 16x baud tick (1.8432 MHz when 115200 baud)
);
    reg [9:0] acc; // accumulator

    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            acc    <= 0;
            s_tick <= 0;
        end else begin
            // Add fractional value (96) each clock
            {s_tick, acc} <= acc + 10'd96;
            if (acc + 10'd96 >= 10'd625) begin
                // When sum ≥ 625, output a tick and subtract 625
                s_tick <= 1;
                acc    <= acc + 10'd96 - 10'd625;
            end else begin
                s_tick <= 0;
            end
        end
    end
endmodule

/////////////////////////////////////////////////////////////////////////////////////////////

// UART Receiver (8 data bits, no parity, 1 stop bit) with 16x sampling
module uart_rx #(
    parameter DBIT = 8,           // # of data bits
    parameter SB_TICK = 16        // # of 16x ticks for stop bit (16 = 1 stop bit)
)(
    input  wire       clk,        // system clock (12 MHz)
    input  wire       rst,        // synchronous reset
    input  wire       rx,         // serial input (from PC)
    input  wire       s_tick,     // baud-rate 16x tick
    output reg [7:0]  dout,		// received byte
	 output 					tb_led,
    output reg        rx_done_tick// high for 1 clk when byte is received
);
    // State encoding
    localparam [1:0]
        IDLE  = 2'b00,
        START = 2'b01,
        DATA  = 2'b10,
        STOP  = 2'b11;
    reg [1:0] state;
    reg [3:0] s_cnt;       // oversample tick counter (needs at least 4 bits to count up to 16)
    reg [2:0] bit_index;   // data bit index 0..7
    reg rx_sync0, rx_sync1; 

    // Double-register the RX input to synchronize
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            rx_sync0 <= 1'b1;
            rx_sync1 <= 1'b1;
        end else begin
            rx_sync0 <= rx;
            rx_sync1 <= rx_sync0;
        end
    end

assign tb_led = (state == DATA) ? 1'b1 : 1'b0;
    // UART receive FSM
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            state        <= IDLE;
            s_cnt        <= 0;
            bit_index    <= 0;
            dout         <= 8'b0;
            rx_done_tick <= 1'b0;
        end else begin
            rx_done_tick <= 1'b0; // default: no done tick
            case (state)
                IDLE: begin
                    // Wait for start bit (falling edge)
                    if (rx_sync1 == 1'b1 && rx_sync0 == 1'b0) begin
                        state     <= START;
                        s_cnt     <= 0;
                    end
                end

                START: begin
                    // Count 8 ticks to middle of start bit
                    if (s_tick) begin
                        s_cnt <= s_cnt + 1;
                        if (s_cnt == 4'd7) begin
                            // Mid-start reached; move to DATA
                            state     <= DATA;
                            s_cnt     <= 0;
                            bit_index <= 0;
                        end
                    end
                end

                DATA: begin
                    // Sample 8 data bits, one every 16 ticks
                    if (s_tick) begin
                        s_cnt <= s_cnt + 1;
                        if (s_cnt == 4'd15) begin
                            // Every 16 ticks: latch data bit
                            dout[bit_index] <= rx_sync0;
                            s_cnt <= 0;
                            if (bit_index == DBIT-1) begin
                                state <= STOP;
                            end else begin
                                bit_index <= bit_index + 1;
                            end
                        end
                    end
                end

                STOP: begin
                    // Wait full stop bit (16 ticks)
                    if (s_tick) begin
                        s_cnt <= s_cnt + 1;
                        if (s_cnt == (SB_TICK-1)) begin
                            // Stop bit done: output byte
                            state        <= IDLE;
                            rx_done_tick <= 1'b1;
                        end
                    end
                end

            endcase
        end
    end
endmodule

/////////////////////////////////////////////////////////////////////
/*
// Top-level: instantiates Baud Generator and UART_RX, drives 8 LEDs
module uart_3 (
    input  wire       clk,      // 12 MHz clock
    input  wire       rst,      // reset
    input  wire       uart_rx,  // serial input
	 output 				 tb_led,
    output reg [7:0]  led       // LEDs showing received byte
);
    wire s_tick;
    wire [7:0] data_byte;
    wire rx_done;

    // Generate 16x baud tick
    baud_gen_115200 baud_gen_inst (
        .clk(clk), .rst(rst), .s_tick(s_tick)
    );

    // UART receiver
    uart_rx uart_rx_inst (
        .clk(clk), .rst(rst), .rx(uart_rx),
        .s_tick(s_tick),
        .dout(data_byte),
		  .tb_led(tb_led),
        .rx_done_tick(rx_done)
    );


    // Latch received byte to LEDs when a new byte arrives
    always @(posedge clk or negedge rst) begin
        if (!rst) begin
            led <= 8'b0;
        end else if (rx_done) begin
            led <= data_byte;
        end
    end
endmodule 
*/
/////////////////////////////////////////////////////////////////////////

module uart_3 (
    input uart_rx,
    input  logic clk,         // Clock 12MHz on CYC1000
    input  logic rst_n,       // Asynchronous active-low reset
    output logic SCLK,        // Serial Clock
    output logic RCLK,        // Register Clock (Latch)
    output logic DIO          // Data Input/Output
);

logic [7:0] bcd_input;
logic [1:0] digit_pos = 0;    // Vị trí thanh ghi hiện tại (0-3)
logic [3:0] registers [0:3];  // 4 thanh ghi lưu giá trị
logic new_data_flag = 0;      // Cờ báo có dữ liệu mới

// 7-segment LED encoding table (common cathode)
logic [7:0] LED_0F [0:15] = '{
  8'hC0, // 0
  8'hF9, // 1
  8'hA4, // 2
  8'hB0, // 3
  8'h99, // 4
  8'h92, // 5
  8'h82, // 6
  8'hF8, // 7
  8'h80, // 8
  8'h90, // 9
  8'h88, // A (10)
  8'h83, // b (11)
  8'hC6, // C (12)
  8'hA1, // d (13)
  8'h86, // E (14)
  8'h8E  // F (15)
};

// 4-digit display data
logic [3:0] LED [0:3] = '{0, 0, 0, 0}; // Mặc định hiển thị 0000

// Control signals
logic [7:0] shift_data;
logic [3:0] bit_cnt;
logic [1:0] digit_index;
logic [19:0] counter = 20'd0;

// Digit selection (active-low)
logic [7:0] digit_select [0:3] = '{
  8'b11111000, // Select digit 0 (units)
  8'b11110100, // Select digit 1 (tens)
  8'b11110010, // Select digit 2 (hundreds)
  8'b11110001  // Select digit 3 (thousands)
};

// FSM states
typedef enum logic [1:0] {
  IDLE,
  SEND_SEGMENT,
  SEND_DIGIT,
  LATCH
} display_state_t;

display_state_t current_state;

// Clock divider for LED scanning (~6kHz)
logic [15:0] fast_counter = 0;
logic fast_clk = 0;

// Buffer để phát hiện dữ liệu UART mới
logic [7:0] prev_bcd_input = 0;

uart_top uart_top_inst (
  .clk(clk), .rst(rst_n), .uart_rx(uart_rx), .led(bcd_input)
);

// Xử lý dữ liệu UART nhận được
always_ff @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    digit_pos <= 0;
    registers <= '{0, 0, 0, 0}; // Reset các thanh ghi
    new_data_flag <= 0;
    prev_bcd_input <= 0;
  end else begin
    // Phát hiện dữ liệu UART mới
    new_data_flag <= (prev_bcd_input != bcd_input) ? 1 : 0;
    prev_bcd_input <= bcd_input;
    
    if (new_data_flag) begin
      // Kiểm tra tín hiệu đặc biệt ENTER (0x0D)
      if (bcd_input == 8'h0D) begin
        // Kích hoạt hiển thị: copy giá trị từ thanh ghi ra LED
        LED[0] <= registers[0];
        LED[1] <= registers[1];
        LED[2] <= registers[2];
        LED[3] <= registers[3];
      end
      // Khi nhận số từ 0-9 (ASCII '0' to '9')
      else if (bcd_input >= 8'h30 && bcd_input <= 8'h39) begin
        // Chuyển ASCII sang giá trị số (0-9)
        registers[digit_pos] <= bcd_input - 8'h30;
        // Di chuyển đến vị trí tiếp theo
        digit_pos <= (digit_pos == 3) ? 0 : digit_pos + 1;
      end
    end
  end
end

// Clock divider for LED scanning (~6kHz)
always_ff @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    fast_counter <= 0;
    fast_clk <= 0;
  end else begin
    if (fast_counter >= 16'd200) begin  // Chia tần số chính xác hơn
      fast_counter <= 0;
      fast_clk <= ~fast_clk;
    end else begin
      fast_counter <= fast_counter + 1;
    end
  end
end

// Display FSM (use fast clock)
logic [3:0] delay_cnt;
always_ff @(posedge fast_clk or negedge rst_n) begin
  if (!rst_n) begin
    current_state <= IDLE;
    SCLK <= 1;
    RCLK <= 1;
    DIO <= 0;
    digit_index <= 0;
    bit_cnt <= 0;
    delay_cnt <= 0;
  end else begin
    SCLK <= 1;
    RCLK <= 1;
    DIO <= 0;
    delay_cnt <= 0;

    case (current_state)
      IDLE: begin
        // Hiển thị từ trái (LED3) sang phải (LED0)
        shift_data <= LED_0F[LED[3 - digit_index]]; 
        bit_cnt <= 0;
        current_state <= SEND_SEGMENT;
      end

      SEND_SEGMENT: begin
        if (bit_cnt < 8) begin
          SCLK <= 0;
          DIO <= shift_data[7];
          if (delay_cnt < 2) begin  // Giảm delay để tăng tốc độ
            delay_cnt <= delay_cnt + 1;
          end else begin
            SCLK <= 1;
            shift_data <= shift_data << 1;
            bit_cnt <= bit_cnt + 1;
            delay_cnt <= 0;
          end
        end else begin
          shift_data <= digit_select[digit_index];
          bit_cnt <= 0;
          current_state <= SEND_DIGIT;
        end
      end

      SEND_DIGIT: begin
        if (bit_cnt < 8) begin
          SCLK <= 0;
          DIO <= shift_data[7];
          if (delay_cnt < 2) begin  // Giảm delay để tăng tốc độ
            delay_cnt <= delay_cnt + 1;
          end else begin
            SCLK <= 1;
            shift_data <= shift_data << 1;
            bit_cnt <= bit_cnt + 1;
            delay_cnt <= 0;
          end
        end else begin
          RCLK <= 0;
          if (delay_cnt < 5) begin  // Giảm delay để tăng tốc độ
            delay_cnt <= delay_cnt + 1;
          end else begin
            RCLK <= 1;
            current_state <= LATCH;
          end
        end
      end

      LATCH: begin
        digit_index <= (digit_index == 3) ? 0 : digit_index + 1;
        current_state <= IDLE;
      end
    endcase
  end
end

endmodule



