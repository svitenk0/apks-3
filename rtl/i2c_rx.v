// =============================================================================
// i2c_rx.v — приймач (Rx) одного байта I2C Master
// -----------------------------------------------------------------------------
// Лабораторна робота №3, варіант 20 (I2C, Verilog). Світенко П.І., КІ-408.
//
// Майстер генерує 9 тактів SCL і приймає байт від веденого (MSB first):
//   такти 1..8 — SDA відпущена, біт фіксується в середині SCL = 1;
//   такт 9     — майстер сам відповідає: ACK (SDA = 0), якщо хоче ще байт,
//                або NACK (SDA = 1) після останнього байта читання.
//
// Фази такту SCL такі самі, як у i2c_tx:
//   0 — SCL = 0: відпустити SDA (дані) або виставити ACK/NACK (9-й такт)
//   1 — відпустити SCL (наростаючий фронт)
//   2 — середина SCL = 1: зсунути біт SDA в регістр
//   3 — притиснути SCL до 0
// =============================================================================
`timescale 1ns / 1ps
`default_nettype none

module i2c_rx (
    input  wire       clk,       // системна тактова частота
    input  wire       rst_n,     // синхронне скидання, активний рівень 0
    input  wire       qtick,     // імпульс фази від генератора фаз
    input  wire [1:0] phase,     // номер фази для цього qtick
    input  wire       start,     // імпульс: почати прийом байта
    input  wire       ack_send,  // 1 = відповісти ACK, 0 = NACK (останній байт)
    input  wire       sda_i,     // синхронізований стан лінії SDA
    output reg        scl_oe,    // 1 = притиснути SCL до 0
    output reg        sda_oe,    // 1 = притиснути SDA до 0
    output reg        busy,      // 1 = іде прийом байта
    output reg        done,      // імпульс: байт прийнято і ACK/NACK видано
    output reg  [7:0] dout       // прийнятий байт (дійсний разом з done)
);

    reg [7:0] shreg;   // зсувний регістр, біти входять справа (MSB first)
    reg [3:0] bitn;    // 0..7 — біти даних, 8 — такт ACK/NACK

    always @(posedge clk) begin
        if (!rst_n) begin
            scl_oe <= 1'b1;
            sda_oe <= 1'b0;
            busy   <= 1'b0;
            done   <= 1'b0;
            dout   <= 8'h00;
            shreg  <= 8'h00;
            bitn   <= 4'd0;
        end else begin
            done <= 1'b0;

            if (start) begin
                bitn   <= 4'd0;
                busy   <= 1'b1;
                scl_oe <= 1'b1;
                sda_oe <= 1'b0;      // відпускаємо SDA — дані виставляє ведений
            end else if (busy && qtick) begin
                case (phase)
                    2'd0: // SCL = 0: на 9-му такті майстер видає ACK або NACK
                        sda_oe <= (bitn == 4'd8) ? ack_send : 1'b0;
                    2'd1: // наростаючий фронт SCL
                        scl_oe <= 1'b0;
                    2'd2: // середина SCL = 1: дані стабільні, зсуваємо біт
                        if (bitn < 4'd8) shreg <= {shreg[6:0], sda_i};
                    2'd3: begin // спадаючий фронт SCL
                        scl_oe <= 1'b1;
                        if (bitn == 4'd8) begin
                            busy <= 1'b0;
                            done <= 1'b1;
                            dout <= shreg;
                        end else begin
                            bitn <= bitn + 4'd1;
                        end
                    end
                endcase
            end
        end
    end

endmodule

`default_nettype wire
