// =============================================================================
// tb_i2c_master.v — самоперевірний тестбенч I2C Master
// -----------------------------------------------------------------------------
// Лабораторна робота №3, варіант 20 (I2C, Verilog). Світенко П.І., КІ-408.
//
// Середовище:
//   * генератор clk (50 МГц) і скидання rst_n;
//   * шина I2C з підтягувальними резисторами (pullup) на SCL і SDA;
//   * DUT i2c_master_top;
//   * i2c_slave_model (адреса 0x50) — інший пристрій: приймає дані запису
//     (перевірка Tx) і сам передає дані під час читання (перевірка Rx);
//   * i2c_monitor — незалежний декодер шини.
//
// Тести:
//   T1 запис 4 байтів у комірки 0x02..0x05
//   T2 зворотне читання тих самих 4 байтів (ACK, ACK, ACK, NACK)
//   T3 читання даних, наперед записаних у модель (шаблон ~addr)
//   T4 звернення до відсутньої адреси 0x23 -> NACK, ack_err = 1
//   T5 запис із розтягуванням SCL веденим (clock stretching 10 мкс)
//   T6 address probe (cmd_len = 0)
//   T7 часові параметри SCL: f_SCL, t_LOW, t_HIGH (UM10204)
//
// Запуск: iverilog -g2005 -o tb.vvp rtl/*.v tb/*.v && vvp tb.vvp +vcd=wave.vcd
// Параметр SCL_FREQ можна змінити: iverilog -P tb_i2c_master.SCL_FREQ=400000
// =============================================================================
`timescale 1ns / 1ps

module tb_i2c_master;

    parameter integer CLK_FREQ = 50_000_000;
    parameter integer SCL_FREQ = 100_000;
    localparam real   CLK_PERIOD = 1.0e9 / CLK_FREQ;       // 20 нс
    localparam [6:0]  SLAVE_ADDR = 7'h50;

    // ---------------- тактування і скидання ----------------
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #(CLK_PERIOD / 2.0) clk = ~clk;

    // ---------------- шина I2C ----------------
    wire scl, sda;
    pullup (scl);
    pullup (sda);

    // ---------------- командний інтерфейс DUT ----------------
    reg        cmd_start = 1'b0;
    reg  [6:0] cmd_addr  = 7'd0;
    reg        cmd_rw    = 1'b0;
    reg  [7:0] cmd_len   = 8'd0;
    wire       tx_req, rx_valid, busy, done, ack_err;
    wire [7:0] rx_data;

    reg  [7:0] wbuf [0:15];        // дані для запису
    reg  [3:0] widx = 4'd0;        // індекс поточного байта запису
    wire [7:0] tx_data = wbuf[widx];
    reg  [7:0] rbuf [0:15];        // прийняті дані
    integer    rcnt = 0;

    // NETLIST: симуляція нетліста після синтезу (параметри вже зафіксовані в ньому)
`ifdef NETLIST
    i2c_master_top dut (
`else
    i2c_master_top #(.CLK_FREQ(CLK_FREQ), .SCL_FREQ(SCL_FREQ)) dut (
`endif
        .clk(clk), .rst_n(rst_n),
        .cmd_start(cmd_start), .cmd_addr(cmd_addr), .cmd_rw(cmd_rw), .cmd_len(cmd_len),
        .tx_data(tx_data), .tx_req(tx_req),
        .rx_data(rx_data), .rx_valid(rx_valid),
        .busy(busy), .done(done), .ack_err(ack_err),
        .scl(scl), .sda(sda)
    );

    i2c_slave_model #(.ADDR(SLAVE_ADDR)) slave (.scl(scl), .sda(sda));
    i2c_monitor mon (.scl(scl), .sda(sda));

    // наступний байт запису після tx_req; збір прийнятих байтів
    always @(posedge clk) begin
        if (tx_req) widx <= widx + 4'd1;
        if (rx_valid) begin
            rbuf[rcnt] <= rx_data;
            rcnt       <= rcnt + 1;
        end
    end

    // ---------------- допоміжні задачі ----------------
    integer n_checks = 0, n_errors = 0;

    task check(input cond, input [8*64-1:0] msg);
        begin
            n_checks = n_checks + 1;
            if (cond) $display("  [PASS] %0s", msg);
            else begin
                n_errors = n_errors + 1;
                $display("  [FAIL] %0s", msg);
            end
        end
    endtask

    // одна транзакція: подати команду і дочекатися done
    task xfer(input [6:0] addr, input rw, input [7:0] len);
        begin
            @(negedge clk);
            widx      = 4'd0;
            rcnt      = 0;
            cmd_addr  = addr;
            cmd_rw    = rw;
            cmd_len   = len;
            cmd_start = 1'b1;
            @(negedge clk);
            cmd_start = 1'b0;
            @(posedge done);
            @(negedge clk);
        end
    endtask

    // перевірка журналу монітора: байт № idx має значення val і відповідь ack
    task expect_bus(input integer idx, input [7:0] val, input ack);
        begin
            check(mon.bytes[idx] === val && mon.acks[idx] === ack,
                  "monitor: bus byte matches expected value and ACK/NACK");
            if (!(mon.bytes[idx] === val && mon.acks[idx] === ack))
                $display("         byte[%0d]: got 0x%02h/%0d, expected 0x%02h/%0d",
                         idx, mon.bytes[idx], mon.acks[idx], val, ack);
        end
    endtask

    // ---------------- сценарій ----------------
    reg [8*256-1:0] vcdfile;
    integer base, j;
    realtime t0;

    initial begin
        if (!$value$plusargs("vcd=%s", vcdfile)) vcdfile = "tb_i2c_master.vcd";
        $dumpfile(vcdfile);
        $dumpvars(0, tb_i2c_master);

        for (j = 0; j < 16; j = j + 1) wbuf[j] = 8'h00;
        $display("==== I2C master testbench: CLK = %0d Hz, SCL = %0d Hz ====", CLK_FREQ, SCL_FREQ);
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);
        check(scl === 1'b1 && sda === 1'b1 && !busy, "after reset: bus idle (SCL = SDA = 1)");

        // ---- T1: запис ----
        $display("-- T1: write 4 bytes to 0x50 @ 0x02");
        wbuf[0] = 8'h02; wbuf[1] = 8'hDE; wbuf[2] = 8'hAD; wbuf[3] = 8'hBE; wbuf[4] = 8'hEF;
        base = mon.nbytes;
        xfer(SLAVE_ADDR, 1'b0, 8'd5);
        check(!ack_err, "T1 write: no ack_err");
        expect_bus(base + 0, {SLAVE_ADDR, 1'b0}, 1'b1);
        for (j = 0; j < 5; j = j + 1) expect_bus(base + 1 + j, wbuf[j], 1'b1);
        check(slave.mem[2] == 8'hDE && slave.mem[3] == 8'hAD &&
              slave.mem[4] == 8'hBE && slave.mem[5] == 8'hEF,
              "T1 write: slave memory 0x02..0x05 = DE AD BE EF");

        // ---- T2: зворотне читання ----
        $display("-- T2: set pointer 0x02, read 4 bytes back");
        wbuf[0] = 8'h02;
        xfer(SLAVE_ADDR, 1'b0, 8'd1);
        base = mon.nbytes;
        xfer(SLAVE_ADDR, 1'b1, 8'd4);
        check(!ack_err && rcnt == 4, "T2 read: 4 bytes received, no ack_err");
        check(rbuf[0] == 8'hDE && rbuf[1] == 8'hAD && rbuf[2] == 8'hBE && rbuf[3] == 8'hEF,
              "T2 read: rx_data = DE AD BE EF");
        expect_bus(base + 0, {SLAVE_ADDR, 1'b1}, 1'b1);
        expect_bus(base + 1, 8'hDE, 1'b1);
        expect_bus(base + 4, 8'hEF, 1'b0);   // останній байт — NACK від майстра

        // ---- T3: читання даних, сформованих веденим ----
        $display("-- T3: read 3 bytes prepared by the slave @ 0x40");
        wbuf[0] = 8'h40;
        xfer(SLAVE_ADDR, 1'b0, 8'd1);
        xfer(SLAVE_ADDR, 1'b1, 8'd3);
        check(rbuf[0] == 8'hBF && rbuf[1] == 8'hBE && rbuf[2] == 8'hBD,
              "T3 read: rx_data = BF BE BD (slave pattern ~addr)");

        // ---- T4: відсутній пристрій ----
        $display("-- T4: access to absent address 0x23 (expect NACK)");
        base = mon.nbytes;
        wbuf[0] = 8'h00; wbuf[1] = 8'h11;
        xfer(7'h23, 1'b0, 8'd2);
        check(ack_err === 1'b1, "T4 NACK: ack_err = 1");
        expect_bus(base, {7'h23, 1'b0}, 1'b0);
        check(mon.nbytes == base + 1, "T4 NACK: transfer aborted after address byte");
        check(scl === 1'b1 && sda === 1'b1 && !busy, "T4 NACK: STOP issued, bus released");

        // ---- T5: clock stretching ----
        $display("-- T5: write with clock stretching (slave holds SCL low 10 us)");
        slave.stretch_ns = 10000;
        mon.t_low_max    = 0;
        t0 = $realtime;
        wbuf[0] = 8'h10; wbuf[1] = 8'h55; wbuf[2] = 8'hAA;
        xfer(SLAVE_ADDR, 1'b0, 8'd3);
        slave.stretch_ns = 0;
        check(!ack_err && slave.mem[8'h10] == 8'h55 && slave.mem[8'h11] == 8'hAA,
              "T5 stretching: data 55 AA written correctly");
        check(mon.t_low_max >= 10000.0, "T5 stretching: SCL low extended by slave (>= 10 us)");
        $display("         max t_LOW = %0.2f us, transaction time = %0.2f us",
                 mon.t_low_max / 1000.0, ($realtime - t0) / 1000.0);

        // ---- T6: address probe ----
        $display("-- T6: address probe (len = 0)");
        xfer(SLAVE_ADDR, 1'b0, 8'd0);
        check(!ack_err, "T6 probe: device 0x50 present (ACK)");

        // ---- T7: часові параметри SCL ----
        $display("-- T7: SCL timing");
        $display("         f_SCL(max) = %0.1f kHz, t_LOW(min) = %0.3f us, t_HIGH(min) = %0.3f us",
                 1.0e6 / mon.t_per_min, mon.t_low_min / 1000.0, mon.t_high_min / 1000.0);
        check(1.0e9 / mon.t_per_min <= SCL_FREQ && 1.0e9 / mon.t_per_min >= 0.95 * SCL_FREQ,
              "T7 timing: f_SCL within [0.95; 1.00] x nominal");
        if (SCL_FREQ <= 100_000)
            check(mon.t_low_min >= 4700.0 && mon.t_high_min >= 4000.0,
                  "T7 timing: Standard-mode t_LOW >= 4.7 us, t_HIGH >= 4.0 us");
        else
            check(mon.t_low_min >= 1300.0 && mon.t_high_min >= 600.0,
                  "T7 timing: Fast-mode t_LOW >= 1.3 us, t_HIGH >= 0.6 us");

        $display("==== bus: %0d START, %0d STOP, %0d bytes; slave wrote %0d, sent %0d bytes ====",
                 mon.nstart, mon.nstop, mon.nbytes, slave.n_wr, slave.n_rd);
        if (n_errors == 0) $display("==== TEST PASSED (%0d checks) ====", n_checks);
        else               $display("==== TEST FAILED (%0d of %0d checks failed) ====", n_errors, n_checks);
        #2000;
        $finish;
    end

    // сторожовий таймер
    initial begin
        #50_000_000;
        $display("==== TEST FAILED (timeout) ====");
        $finish;
    end

endmodule
