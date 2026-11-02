// =============================================================================
// i2c_master_top.v — верхній рівень I2C Master з виводами відкритого стоку
// -----------------------------------------------------------------------------
// Лабораторна робота №3, варіант 20 (I2C, Verilog). Світенко П.І., КІ-408.
//
// Обгортка над ядром i2c_master: перетворює пари (вхід, керування) у
// двонаправлені виводи SCL/SDA. Логічну 1 на шині формують зовнішні
// підтягувальні резистори (типово 4.7 кОм для 100 кГц, 2.2 кОм для 400 кГц).
// У ПЛІС такі виводи синтезуються у тристабільні буфери вводу-виводу (IOB).
// =============================================================================
`timescale 1ns / 1ps
`default_nettype none

module i2c_master_top #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer SCL_FREQ = 100_000
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       cmd_start,
    input  wire [6:0] cmd_addr,
    input  wire       cmd_rw,
    input  wire [7:0] cmd_len,
    input  wire [7:0] tx_data,
    output wire       tx_req,
    output wire [7:0] rx_data,
    output wire       rx_valid,
    output wire       busy,
    output wire       done,
    output wire       ack_err,
    inout  wire       scl,          // лінія тактування I2C
    inout  wire       sda           // лінія даних I2C
);

    wire scl_oe, sda_oe;

    // Відкритий стік: або 0, або високоімпедансний стан (Z)
    assign scl = scl_oe ? 1'b0 : 1'bz;
    assign sda = sda_oe ? 1'b0 : 1'bz;

    i2c_master #(.CLK_FREQ(CLK_FREQ), .SCL_FREQ(SCL_FREQ)) u_core (
        .clk(clk), .rst_n(rst_n),
        .cmd_start(cmd_start), .cmd_addr(cmd_addr), .cmd_rw(cmd_rw), .cmd_len(cmd_len),
        .tx_data(tx_data), .tx_req(tx_req),
        .rx_data(rx_data), .rx_valid(rx_valid),
        .busy(busy), .done(done), .ack_err(ack_err),
        .scl_i(scl), .scl_oe(scl_oe),
        .sda_i(sda), .sda_oe(sda_oe)
    );

endmodule

`default_nettype wire
