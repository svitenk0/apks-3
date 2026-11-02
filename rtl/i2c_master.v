// =============================================================================
// i2c_master.v — контролер I2C Master (ядро без тристабільних буферів)
// -----------------------------------------------------------------------------
// Лабораторна робота №3, варіант 20 (I2C, Verilog). Світенко П.І., КІ-408.
//
// Склад:
//   * синхронізатори входів SCL/SDA (2 тригери — захист від метастабільності);
//   * генератор фаз: ділить clk і формує 4 імпульси qtick на кожен такт SCL,
//     утримує лічильник, поки ведений розтягує SCL (clock stretching);
//   * головний автомат (FSM) транзакції:
//       IDLE -> START -> ADDR -> {WRITE | READ}* -> STOP -> BUF -> IDLE
//   * i2c_tx — передавач байта (адреса, дані запису), i2c_rx — приймач байта.
//
// Транзакція: START, адреса (7 біт) + R/W, `cmd_len` байтів даних, STOP.
//   запис (cmd_rw = 0): байти беруться з tx_data; імпульс tx_req означає,
//                       що поточний байт узято і треба виставити наступний;
//   читання (cmd_rw = 1): кожен прийнятий байт видається на rx_data з
//                       імпульсом rx_valid; на останньому байті майстер дає NACK;
//   cmd_len = 0        : лише перевірка присутності пристрою (address probe).
// NACK від веденого на адресі чи даних -> ack_err = 1 і негайний STOP.
//
// Часові параметри (Standard-mode 100 кГц / Fast-mode 400 кГц, UM10204):
//   такт SCL = 2 фази низького рівня (LOW_Q) + 2 фази високого (HIGH_Q);
//   LOW ≈ 60 %, HIGH ≈ 40 % періоду, щоб виконати t_LOW >= 4.7 / 1.3 мкс
//   і t_HIGH >= 4.0 / 0.6 мкс.
// =============================================================================
`timescale 1ns / 1ps
`default_nettype none

module i2c_master #(
    parameter integer CLK_FREQ = 50_000_000,   // частота clk, Гц
    parameter integer SCL_FREQ = 100_000       // цільова частота SCL, Гц
) (
    input  wire       clk,
    input  wire       rst_n,        // синхронне скидання, активний 0

    // ---- командний інтерфейс ----
    input  wire       cmd_start,    // імпульс: почати транзакцію (ігнорується, якщо busy)
    input  wire [6:0] cmd_addr,     // 7-бітна адреса веденого
    input  wire       cmd_rw,       // 0 = запис, 1 = читання
    input  wire [7:0] cmd_len,      // кількість байтів даних (0 = address probe)
    input  wire [7:0] tx_data,      // байт для запису
    output reg        tx_req,       // імпульс: tx_data узято, виставити наступний
    output reg  [7:0] rx_data,      // прийнятий байт
    output reg        rx_valid,     // імпульс: rx_data дійсний
    output wire       busy,         // 1 = шина зайнята транзакцією
    output reg        done,         // імпульс: транзакцію завершено (після STOP)
    output reg        ack_err,      // 1 = ведений не підтвердив (NACK)

    // ---- шина I2C (відкритий стік, розділені вхід/керування) ----
    input  wire       scl_i,        // стан лінії SCL
    output wire       scl_oe,       // 1 = притиснути SCL до 0
    input  wire       sda_i,        // стан лінії SDA
    output wire       sda_oe        // 1 = притиснути SDA до 0
);

    // -------------------------------------------------------------------------
    // Параметри дільника: кількість тактів clk у фазі низького/високого рівня
    // -------------------------------------------------------------------------
    localparam integer BIT_CLKS = CLK_FREQ / SCL_FREQ;            // тактів clk на біт
    localparam integer LOW_Q    = (BIT_CLKS * 3 + 9) / 10;        // 30 % біта, вгору
    localparam integer HIGH_Q   = (BIT_CLKS - 2 * LOW_Q) / 2;     // решта / 2, вниз
    localparam integer CW       = $clog2(LOW_Q + 1);              // розрядність лічильника

    // Перевірка параметрів під час елаборації: занадто мала частота clk.
    generate
        if (HIGH_Q < 1) begin : g_bad_params
            ERROR_CLK_FREQ_too_low_for_SCL_FREQ u_param_check ();
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Стани головного автомата
    // -------------------------------------------------------------------------
    localparam [2:0] S_IDLE  = 3'd0,   // шина вільна, очікування cmd_start
                     S_START = 3'd1,   // формування умови START
                     S_ADDR  = 3'd2,   // передача адреси + R/W (i2c_tx)
                     S_WRITE = 3'd3,   // передача байтів даних (i2c_tx)
                     S_READ  = 3'd4,   // прийом байтів даних (i2c_rx)
                     S_STOP  = 3'd5,   // формування умови STOP
                     S_BUF   = 3'd6;   // t_BUF — пауза вільної шини перед IDLE

    reg [2:0] state;

    // -------------------------------------------------------------------------
    // Синхронізатори входів (лінії шини асинхронні до clk)
    // -------------------------------------------------------------------------
    reg [1:0] scl_sync, sda_sync;
    wire scl_s = scl_sync[1];
    wire sda_s = sda_sync[1];

    always @(posedge clk) begin
        if (!rst_n) begin
            scl_sync <= 2'b11;
            sda_sync <= 2'b11;
        end else begin
            scl_sync <= {scl_sync[0], scl_i};
            sda_sync <= {sda_sync[0], sda_i};
        end
    end

    // -------------------------------------------------------------------------
    // Генератор фаз (дільник частоти) з підтримкою clock stretching
    // -------------------------------------------------------------------------
    // phase — номер фази, яку буде виконано на наступному qtick:
    //   інтервал перед фазами 0 і 1 — SCL = 0 (тривалість LOW_Q),
    //   інтервал перед фазами 2 і 3 — SCL = 1 (тривалість HIGH_Q).
    reg  [CW-1:0] qcnt;
    reg  [1:0]    phase;
    wire [CW-1:0] qlen    = (phase[1] == 1'b0) ? LOW_Q - 1 : HIGH_Q - 1;
    // майстер відпустив SCL, а лінія ще в 0 -> ведений розтягує такт: чекаємо
    wire          stretch = ~scl_oe & ~scl_s;
    wire          qtick   = (state != S_IDLE) && !stretch && (qcnt == qlen);

    always @(posedge clk) begin
        if (!rst_n || state == S_IDLE) begin
            qcnt  <= {CW{1'b0}};
            phase <= 2'd0;
        end else if (!stretch) begin
            if (qcnt == qlen) begin
                qcnt  <= {CW{1'b0}};
                phase <= phase + 2'd1;
            end else begin
                qcnt  <= qcnt + 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Передавач і приймач байта
    // -------------------------------------------------------------------------
    reg        tx_go, rx_go;     // імпульси запуску
    reg  [7:0] tx_byte;          // байт, що передається
    wire       tx_scl_oe, tx_sda_oe, tx_done, tx_ack;
    wire       rx_scl_oe, rx_sda_oe, rx_done;
    wire [7:0] rx_byte;

    reg  [6:0] addr_r;           // зафіксовані параметри команди
    reg        rw_r;
    reg  [7:0] len_r;
    reg  [7:0] cnt;              // номер поточного байта даних
    wire       last_byte = (cnt == len_r - 8'd1);

    i2c_tx u_tx (
        .clk(clk), .rst_n(rst_n), .qtick(qtick), .phase(phase),
        .start(tx_go), .din(tx_byte), .sda_i(sda_s),
        .scl_oe(tx_scl_oe), .sda_oe(tx_sda_oe),
        .busy(), .done(tx_done), .ack(tx_ack)
    );

    i2c_rx u_rx (
        .clk(clk), .rst_n(rst_n), .qtick(qtick), .phase(phase),
        .start(rx_go), .ack_send(~last_byte), .sda_i(sda_s),
        .scl_oe(rx_scl_oe), .sda_oe(rx_sda_oe),
        .busy(), .done(rx_done), .dout(rx_byte)
    );

    // -------------------------------------------------------------------------
    // Головний автомат транзакції
    // -------------------------------------------------------------------------
    reg scl_oe_r, sda_oe_r;      // керування лініями у станах START/STOP/BUF/IDLE

    always @(posedge clk) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            scl_oe_r <= 1'b0;
            sda_oe_r <= 1'b0;
            tx_go    <= 1'b0;
            rx_go    <= 1'b0;
            tx_req   <= 1'b0;
            tx_byte  <= 8'h00;
            rx_data  <= 8'h00;
            rx_valid <= 1'b0;
            done     <= 1'b0;
            ack_err  <= 1'b0;
            addr_r   <= 7'd0;
            rw_r     <= 1'b0;
            len_r    <= 8'd0;
            cnt      <= 8'd0;
        end else begin
            // імпульсні сигнали за замовчуванням
            tx_go    <= 1'b0;
            rx_go    <= 1'b0;
            tx_req   <= 1'b0;
            rx_valid <= 1'b0;
            done     <= 1'b0;

            case (state)
                // ---- шина вільна: обидві лінії відпущені ----
                S_IDLE: begin
                    scl_oe_r <= 1'b0;
                    sda_oe_r <= 1'b0;
                    if (cmd_start) begin
                        addr_r  <= cmd_addr;
                        rw_r    <= cmd_rw;
                        len_r   <= cmd_len;
                        cnt     <= 8'd0;
                        ack_err <= 1'b0;
                        state   <= S_START;
                    end
                end

                // ---- START: SDA 1->0 при SCL = 1, потім SCL -> 0 ----
                S_START: if (qtick) begin
                    case (phase)
                        2'd0: begin scl_oe_r <= 1'b0; sda_oe_r <= 1'b0; end
                        2'd1: sda_oe_r <= 1'b1;              // умова START
                        2'd2: ;                              // t_HD;STA
                        2'd3: begin
                            scl_oe_r <= 1'b1;                // SCL -> 0
                            tx_byte  <= {addr_r, rw_r};      // байт адреси
                            tx_go    <= 1'b1;
                            state    <= S_ADDR;
                        end
                    endcase
                end

                // ---- адреса передана: аналіз ACK ----
                S_ADDR: if (tx_done) begin
                    if (!tx_ack) begin
                        ack_err <= 1'b1;                     // немає пристрою
                        state   <= S_STOP;
                    end else if (len_r == 8'd0) begin
                        state   <= S_STOP;                   // address probe
                    end else if (!rw_r) begin
                        tx_byte <= tx_data;                  // перший байт запису
                        tx_req  <= 1'b1;
                        tx_go   <= 1'b1;
                        state   <= S_WRITE;
                    end else begin
                        rx_go   <= 1'b1;                     // перший байт читання
                        state   <= S_READ;
                    end
                end

                // ---- запис: байт передано ----
                S_WRITE: if (tx_done) begin
                    if (!tx_ack) begin
                        ack_err <= 1'b1;                     // ведений відмовив
                        state   <= S_STOP;
                    end else if (last_byte) begin
                        state   <= S_STOP;
                    end else begin
                        cnt     <= cnt + 8'd1;
                        tx_byte <= tx_data;
                        tx_req  <= 1'b1;
                        tx_go   <= 1'b1;
                    end
                end

                // ---- читання: байт прийнято ----
                S_READ: if (rx_done) begin
                    rx_data  <= rx_byte;
                    rx_valid <= 1'b1;
                    if (last_byte) begin
                        state <= S_STOP;                     // NACK уже видано
                    end else begin
                        cnt   <= cnt + 8'd1;
                        rx_go <= 1'b1;
                    end
                end

                // ---- STOP: SDA 0->1 при SCL = 1 ----
                S_STOP: if (qtick) begin
                    case (phase)
                        2'd0: begin scl_oe_r <= 1'b1; sda_oe_r <= 1'b1; end // SCL=0, SDA=0
                        2'd1: scl_oe_r <= 1'b0;              // SCL -> 1
                        2'd2: ;                              // t_SU;STO
                        2'd3: begin
                            sda_oe_r <= 1'b0;                // умова STOP
                            state    <= S_BUF;
                        end
                    endcase
                end

                // ---- t_BUF: шина вільна щонайменше один такт SCL ----
                S_BUF: if (qtick && phase == 2'd3) begin
                    state <= S_IDLE;
                    done  <= 1'b1;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Мультиплексор керування лініями: хто володіє шиною в поточному стані
    // -------------------------------------------------------------------------
    wire own_tx = (state == S_ADDR) || (state == S_WRITE);
    wire own_rx = (state == S_READ);

    assign scl_oe = own_tx ? tx_scl_oe : own_rx ? rx_scl_oe : scl_oe_r;
    assign sda_oe = own_tx ? tx_sda_oe : own_rx ? rx_sda_oe : sda_oe_r;
    assign busy   = (state != S_IDLE);

endmodule

`default_nettype wire
