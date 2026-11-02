// =============================================================================
// i2c_monitor.v — пасивний аналізатор шини I2C (лише для симуляції)
// -----------------------------------------------------------------------------
// Незалежно від DUT декодує умови START/STOP і 9-бітні кадри (байт + ACK),
// зберігає їх у журнал і друкує протокол обміну. Тестбенч порівнює журнал
// з очікуваним потоком бітів — так перевіряється коректність передавача (Tx).
// Також вимірює часові параметри SCL: мінімальні t_LOW, t_HIGH і період.
// =============================================================================
`timescale 1ns / 1ps

module i2c_monitor (
    input wire scl,
    input wire sda
);

    reg  [7:0] bytes [0:255];   // журнал байтів
    reg        acks  [0:255];   // 1 = ACK, 0 = NACK
    integer    nbytes = 0;      // кількість байтів у журналі
    integer    nstart = 0, nstop = 0;

    reg        in_frame = 1'b0;
    reg  [8:0] sh;
    integer    nb = 0;

    // ---- часові вимірювання ----
    realtime t_rise = 0, t_fall = 0, t_prev_rise = 0;
    realtime t_low_min = 1.0e9, t_high_min = 1.0e9, t_per_min = 1.0e9;
    realtime t_low_max = 0;

    always @(negedge sda) if (scl === 1'b1) begin
        nstart   = nstart + 1;
        in_frame = 1'b1;
        nb       = 0;
        $display("[MON] %9.2f us  START", $realtime / 1000.0);
    end

    always @(posedge sda) if (scl === 1'b1 && in_frame) begin
        nstop    = nstop + 1;
        in_frame = 1'b0;
        $display("[MON] %9.2f us  STOP", $realtime / 1000.0);
    end

    always @(posedge scl) begin
        if (t_fall > 0 && in_frame && ($realtime - t_fall) < t_low_min) t_low_min = $realtime - t_fall;
        if (t_fall > 0 && in_frame && ($realtime - t_fall) > t_low_max) t_low_max = $realtime - t_fall;
        if (t_prev_rise > 0 && in_frame && ($realtime - t_prev_rise) < t_per_min)
            t_per_min = $realtime - t_prev_rise;
        t_prev_rise = $realtime;
        t_rise      = $realtime;
        if (in_frame) begin
            sh = {sh[7:0], sda};
            nb = nb + 1;
            if (nb == 9) begin
                bytes[nbytes] = sh[8:1];
                acks[nbytes]  = ~sh[0];
                $display("[MON] %9.2f us  byte 0x%02h  %s", $realtime / 1000.0, sh[8:1], sh[0] ? "NACK" : "ACK");
                nbytes = nbytes + 1;
                nb     = 0;
            end
        end
    end

    always @(negedge scl) begin
        if (in_frame && t_rise > 0 && ($realtime - t_rise) < t_high_min) t_high_min = $realtime - t_rise;
        t_fall = $realtime;
    end

endmodule
