// =============================================================================
// i2c_slave_model.v — поведінкова модель веденого I2C (лише для симуляції)
// -----------------------------------------------------------------------------
// Імітує пам'ять на кшталт EEPROM 24Cxx з 7-бітною адресою ADDR:
//   запис : S | ADDR+W | A | ptr | A | d0 | A | d1 | A ... | P
//           перший байт — вказівник комірки, далі mem[ptr++] = d;
//   читання: S | ADDR+R | A | mem[ptr++] | A ... | mem[ptr++] | N | P
// Чужа адреса -> модель мовчить (NACK).
// stretch_ns > 0 -> після кожного ACK модель утримує SCL у 0 (clock stretching).
// Дані модель змінює через T_DATA нс після спаду SCL (t_HD;DAT > 0).
// =============================================================================
`timescale 1ns / 1ps

module i2c_slave_model #(
    parameter [6:0]   ADDR   = 7'h50,
    parameter integer T_DATA = 300          // затримка зміни SDA після спаду SCL, нс
) (
    inout wire scl,
    inout wire sda
);

    reg sda_low = 1'b0;                     // 1 = модель притискає SDA
    reg scl_low = 1'b0;                     // 1 = модель притискає SCL (stretching)
    assign sda = sda_low ? 1'b0 : 1'bz;
    assign scl = scl_low ? 1'b0 : 1'bz;

    reg [7:0] mem [0:255];                  // пам'ять пристрою
    reg [7:0] ptr = 8'h00;                  // вказівник комірки
    integer   stretch_ns = 0;               // тривалість розтягування SCL (встановлює тестбенч)
    integer   n_wr = 0, n_rd = 0;           // лічильники байтів для звіту

    integer i;
    initial for (i = 0; i < 256; i = i + 1) mem[i] = ~i[7:0];  // відомий шаблон для читання

    // ---- прийом байта; ev: 0 = байт, 1 = STOP, 2 = повторний START ----
    task automatic recv_byte(output [7:0] b, output [1:0] ev);
        integer k;
        reg     bitv;
        begin
            ev = 2'd0;
            b  = 8'h00;
            k  = 7;
            while (k >= 0 && ev == 2'd0) begin
                @(posedge scl) bitv = sda;
                @(negedge scl or sda);              // спад SCL або зміна SDA при SCL = 1
                if (scl === 1'b1) ev = (sda === 1'b0) ? 2'd2 : 2'd1;
                else              b[k] = bitv;
                k = k - 1;
            end
        end
    endtask

    // ---- 9-й такт: видати ACK (викликається одразу після спаду 8-го такту) ----
    task automatic send_ack;
        begin
            #(T_DATA) sda_low = 1'b1;
            @(posedge scl);
            @(negedge scl);
        end
    endtask

    // ---- розтягування такту після ACK ----
    task automatic do_stretch;
        begin
            if (stretch_ns > 0) begin
                scl_low = 1'b1;
                #(stretch_ns) scl_low = 1'b0;
            end
        end
    endtask

    // ---- головний процес моделі ----
    reg [7:0] b;
    reg [1:0] ev;
    reg       first, fin, mack, restart;
    integer   k;

    initial begin
        restart = 1'b0;
        forever begin
            // очікування START: спад SDA при SCL = 1
            if (!restart) begin : wait_start
                forever begin
                    @(negedge sda);
                    if (scl === 1'b1) disable wait_start;
                end
            end
            restart = 1'b0;
            @(negedge scl);

            recv_byte(b, ev);
            if (ev == 2'd2) restart = 1'b1;
            else if (ev == 2'd0 && b[7:1] == ADDR) begin
                send_ack;
                if (b[0] == 1'b0) begin
                    // ------------------ запис ------------------
                    first = 1'b1;
                    fin   = 1'b0;
                    while (!fin) begin
                        #(T_DATA) sda_low = 1'b0;            // відпустити SDA після ACK
                        do_stretch;
                        recv_byte(b, ev);
                        if (ev != 2'd0) begin
                            fin = 1'b1;
                            if (ev == 2'd2) restart = 1'b1;
                        end else begin
                            if (first) ptr = b;
                            else begin
                                mem[ptr] = b;
                                ptr = ptr + 8'd1;
                                n_wr = n_wr + 1;
                            end
                            first = 1'b0;
                            send_ack;
                        end
                    end
                end else begin
                    // ------------------ читання ------------------
                    #(T_DATA) sda_low = 1'b0;
                    do_stretch;
                    fin = 1'b0;
                    while (!fin) begin
                        for (k = 7; k >= 0; k = k - 1) begin
                            sda_low = ~mem[ptr][k];
                            @(posedge scl);
                            @(negedge scl);
                            #(T_DATA);
                        end
                        sda_low = 1'b0;                      // відпустити SDA на такт ACK
                        ptr  = ptr + 8'd1;
                        n_rd = n_rd + 1;
                        @(posedge scl) mack = (sda === 1'b0);
                        @(negedge scl);
                        if (!mack) fin = 1'b1;               // NACK -> кінець читання
                        else #(T_DATA);
                    end
                end
            end
            // інакше: чужа адреса — NACK (SDA не чіпаємо), чекаємо наступний START
        end
    end

endmodule
