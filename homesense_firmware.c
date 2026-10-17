/*
 * =============================================================================
 *  HomeSense Node - CO2 Monitor Firmware 
 *  File    : homesense_firmware.c      
 *  Author  : Mangesh Biradar             Program : Yuva Intern - Embedded Systems
 * =============================================================================
 

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#if defined(__AVR__)
  #define TARGET_AVR 1
  #ifndef F_CPU
    #define F_CPU 16000000UL
  #endif
  #include <avr/io.h>
  #include <avr/interrupt.h>
  #include <avr/sleep.h>
#elif defined(HOST_SIM)
  #include <stdio.h>
  #include <stdlib.h>
#else
  #error "Build with -DHOST_SIM for the PC simulation, or with avr-gcc -mmcu=atmega328p"
#endif

/* =============================================================================
 * SECTION 1 - CONFIGURATION
 * ========================================================================== */
#define FW_VERSION        "1.0"
#define FILTER_LEN        4      /* moving-average window (samples)            */
#define FAIR_PPM          1000   /* at or above this the air is "FAIR"         */
#define TH_DEFAULT        1500   /* default alert threshold (ppm)              */
#define TH_MIN            1100   /* allowed range for the TH command           */
#define TH_MAX            5000
#define HYST_PPM          50     /* alert clears this far below the threshold  */
#define PERSIST_SAMPLES   3      /* samples above threshold before alert       */
#define FAULT_LIMIT       3      /* failed reads in a row before FAULT         */
#define REPORT_DEFAULT_S  5      /* seconds between periodic reports           */
#define DEBOUNCE_TICKS    3      /* 3 x 10 ms = 30 ms stable before accepting  */
#define RX_SIZE           64     /* UART ring buffer size, MUST be power of 2  */
#define LINE_MAX_LEN      24     /* longest accepted command line (chars)      */

#define LED_G 0x01u              /* bit masks used by hal_led_set()            */
#define LED_Y 0x02u
#define LED_R 0x04u

typedef enum { LVL_GOOD = 0, LVL_FAIR, LVL_POOR } level_t;
typedef enum { SRC_ADC = 0, SRC_PPM, SRC_FAIL } source_t;

/* =============================================================================
 * SECTION 2 - HAL INTERFACE (implemented in Sections 4 and 5)
 * ========================================================================== */
static void     hal_init(void);                 /* configure pins/timer/UART/ADC */
static void     hal_led_set(uint8_t mask);      /* LED_G | LED_Y | LED_R         */
static void     hal_buzzer(bool on);
static int16_t  hal_adc_read(void);             /* 0..1023, or -1 on failure     */
static bool     hal_button_pressed(void);       /* true while button is down     */
static void     hal_uart_putc(char c);          /* blocking transmit of 1 byte   */
static uint8_t  hal_irq_save(void);             /* disable interrupts, old state */
static void     hal_irq_restore(uint8_t state);

/* =============================================================================
 * SECTION 3 - CORE FIRMWARE
 * ========================================================================== */

/* ---- 3.1 Interrupt-shared data ------------------------------------------ */
/* Flags are set in the ISR and cleared in app_step(). 'volatile' tells the
 * compiler they can change at any time. 8-bit accesses are atomic on AVR.   */
static volatile uint8_t f_10ms, f_250ms, f_1s;
static uint16_t c_10, c_250, c_1000;            /* ISR-private down-counters    */

/* Called by the hardware timer every 1 ms. Keeps work to an absolute minimum:
 * only counts and raises flags. If the main loop is slow, flags simply merge. */
void timer_tick_isr(void)
{
    if (++c_10   >= 10)   { c_10   = 0; f_10ms  = 1; }
    if (++c_250  >= 250)  { c_250  = 0; f_250ms = 1; }
    if (++c_1000 >= 1000) { c_1000 = 0; f_1s    = 1; }
}

/* UART receive ring buffer. The ISR is the only writer of rx_head, the main
 * loop is the only writer of rx_tail, so no lock is needed.                  */
static volatile uint8_t rx_buf[RX_SIZE];
static volatile uint8_t rx_head, rx_tail;
static volatile uint8_t rx_overflow;            /* bytes dropped when full      */

/* Called by the hardware on every received byte. */
void uart_rx_isr(uint8_t c)
{
    uint8_t next = (uint8_t)((rx_head + 1u) & (RX_SIZE - 1u));
    if (next == rx_tail) {                      /* buffer full: drop the byte   */
        if (rx_overflow < 255u) rx_overflow++;
        return;
    }
    rx_buf[rx_head] = c;
    rx_head = next;
}

static bool rx_get(uint8_t *c)
{
    if (rx_head == rx_tail) return false;
    *c = rx_buf[rx_tail];
    rx_tail = (uint8_t)((rx_tail + 1u) & (RX_SIZE - 1u));
    return true;
}

/* ---- 3.2 Application state (main-loop only) ---------------------------- */
static uint32_t s_uptime;                       /* seconds since start          */
static uint16_t s_fbuf[FILTER_LEN];             /* moving-average history       */
static uint8_t  s_fcnt, s_fidx;
static uint16_t s_inst = 400;                   /* latest instantaneous ppm     */
static uint16_t s_filt = 400;                   /* filtered ppm                 */
static level_t  s_level = LVL_GOOD;
static bool     s_alert;
static uint8_t  s_persist;
static uint16_t s_th = TH_DEFAULT;
static uint8_t  s_report_s = REPORT_DEFAULT_S;
static uint8_t  s_rep_cnt;
static bool     s_muted;
static bool     s_blink;                        /* toggles every 250 ms         */
static bool     s_fault;
static uint8_t  s_fault_cnt;
static source_t s_src = SRC_ADC;
static uint16_t s_sim_ppm;
static bool     s_btn_raw, s_btn_stable;        /* debounce state               */
static uint8_t  s_btn_cnt;
static char     s_line[LINE_MAX_LEN + 1];       /* command line being typed     */
static uint8_t  s_line_len;
static bool     s_line_discard;                 /* skip rest of an overlong line*/

/* ---- 3.3 Small output helpers (no printf, so it fits a small MCU) ------- */
static void put_str(const char *s) { while (*s) hal_uart_putc(*s++); }
static void put_nl(void)           { hal_uart_putc('\r'); hal_uart_putc('\n'); }

static void put_u32(uint32_t v)
{
    char b[11];
    uint8_t i = 0;
    if (v == 0) { hal_uart_putc('0'); return; }
    while (v) { b[i++] = (char)('0' + (v % 10u)); v /= 10u; }
    while (i)  hal_uart_putc(b[--i]);
}

static void put_ts(void)                        /* "[t=12s] " time stamp        */
{
    put_str("[t="); put_u32(s_uptime); put_str("s] ");
}

static const char *level_name(level_t l)
{
    return (l == LVL_GOOD) ? "GOOD" : (l == LVL_FAIR) ? "FAIR" : "POOR";
}

static const char *src_name(source_t s)
{
    return (s == SRC_ADC) ? "ADC" : (s == SRC_PPM) ? "PPM" : "FAIL";
}

/* Print "[t=..s] EVT <msg> CO2=..ppm" */
static void event_ppm(const char *msg)
{
    put_ts(); put_str("EVT "); put_str(msg);
    put_str(" CO2="); put_u32(s_filt); put_str("ppm"); put_nl();
}

/* ---- 3.4 Sensor conversion and filter ----------------------------------- */
/* Linear map of the 10-bit ADC value onto 400..2000 ppm.                     */
static uint16_t adc_to_ppm(uint16_t adc)
{
    return (uint16_t)(400u + ((uint32_t)adc * 1600u) / 1023u);
}

static void filter_push(uint16_t v)
{
    uint32_t sum = 0;
    uint8_t i;
    s_fbuf[s_fidx] = v;
    s_fidx = (uint8_t)((s_fidx + 1u) % FILTER_LEN);
    if (s_fcnt < FILTER_LEN) s_fcnt++;
    for (i = 0; i < s_fcnt; i++) sum += s_fbuf[i];
    s_filt = (uint16_t)(sum / s_fcnt);
}

/* ---- 3.5 Alert state machine (persistence + hysteresis) ----------------- */
/*   normal --(filt >= TH for 3 samples)--> ALERT
 *   ALERT  --(filt + 50 < TH)-----------> normal                             */
static void alert_eval(void)
{
    if (!s_alert) {
        if (s_filt >= s_th) {
            if (++s_persist >= PERSIST_SAMPLES) {
                s_alert = true;
                s_persist = 0;
                event_ppm("ALERT ON");
            }
        } else {
            s_persist = 0;                      /* streak broken                */
        }
    } else if ((uint16_t)(s_filt + HYST_PPM) < s_th) {
        s_alert = false;
        event_ppm("ALERT OFF");
    }

    if (s_alert)                    s_level = LVL_POOR;
    else if (s_filt >= FAIR_PPM)    s_level = LVL_FAIR;
    else                            s_level = LVL_GOOD;
}

/* ---- 3.6 LED and buzzer outputs ----------------------------------------- */
static void outputs_apply(void)
{
    uint8_t mask = 0;
    bool buz = false;

    if (s_fault) {                              /* all LEDs blink together      */
        mask = s_blink ? (uint8_t)(LED_G | LED_Y | LED_R) : 0u;
    } else if (s_level == LVL_GOOD) {
        mask = LED_G;
    } else if (s_level == LVL_FAIR) {
        mask = LED_Y;
    } else {                                    /* POOR: red blink + beep       */
        mask = s_blink ? LED_R : 0u;
        buz  = s_blink && !s_muted;
    }
    hal_led_set(mask);
    hal_buzzer(buz);
}

/* ---- 3.7 Sampling task (runs once per second) --------------------------- */
static void sample_task(void)
{
    bool ok = true;
    uint16_t ppm = 0;

    s_uptime++;

    if (s_src == SRC_FAIL) {
        ok = false;                             /* injected failure (SIM FAIL)  */
    } else if (s_src == SRC_PPM) {
        ppm = s_sim_ppm;                        /* injected value (SIM <ppm>)   */
    } else {
        int16_t raw = hal_adc_read();
        if (raw < 0 || raw > 1023) ok = false;  /* plausibility check           */
        else ppm = adc_to_ppm((uint16_t)raw);
    }

    if (!ok) {
        if (s_fault_cnt < 255u) s_fault_cnt++;
        if (s_fault_cnt >= FAULT_LIMIT && !s_fault) {
            s_fault = true;
            event_ppm("SENSOR FAULT");
        }
    } else {
        if (s_fault) { s_fault = false; event_ppm("SENSOR OK"); }
        s_fault_cnt = 0;
        s_inst = ppm;
        filter_push(ppm);
        alert_eval();
    }

    outputs_apply();

    if (++s_rep_cnt >= s_report_s) {            /* periodic report              */
        s_rep_cnt = 0;
        put_ts();
        if (s_fault) {
            put_str("RPT FAULT");
        } else {
            put_str("RPT CO2="); put_u32(s_filt);
            put_str("ppm inst="); put_u32(s_inst);
            put_str(" LVL="); put_str(level_name(s_level));
        }
        put_nl();
    }
}

/* ---- 3.8 Button task (runs every 10 ms) --------------------------------- */
/* The raw level must stay the same for DEBOUNCE_TICKS reads in a row before it
 * is accepted, which removes contact bounce. A accepted PRESS toggles mute.  */
static void button_task(void)
{
    bool raw = hal_button_pressed();
    if (raw == s_btn_raw) {
        if (s_btn_cnt < 255u) s_btn_cnt++;
    } else {
        s_btn_raw = raw;
        s_btn_cnt = 0;
    }
    if (s_btn_cnt == DEBOUNCE_TICKS && s_btn_raw != s_btn_stable) {
        s_btn_stable = s_btn_raw;
        if (s_btn_stable) {                     /* press edge                   */
            s_muted = !s_muted;
            put_ts(); put_str("EVT BUTTON MUTE="); put_u32(s_muted ? 1u : 0u);
            put_nl();
            outputs_apply();
        }
    }
}

/* ---- 3.9 UART command interpreter --------------------------------------- */
static bool parse_u32(const char *s, uint32_t *out)
{
    uint32_t v = 0;
    if (*s == '\0') return false;
    while (*s) {
        if (*s < '0' || *s > '9') return false;
        v = v * 10u + (uint32_t)(*s - '0');
        if (v > 100000u) return false;
        s++;
    }
    *out = v;
    return true;
}

static void cmd_status(void)
{
    put_str("CO2="); put_u32(s_filt);
    put_str("ppm LVL="); put_str(level_name(s_level));
    put_str(" ALERT="); put_u32(s_alert ? 1u : 0u);
    put_str(" TH="); put_u32(s_th);
    put_str(" INT="); put_u32(s_report_s);
    put_str("s MUTE="); put_u32(s_muted ? 1u : 0u);
    put_str(" SRC="); put_str(src_name(s_src));
    put_str(" FAULT="); put_u32(s_fault ? 1u : 0u);
    put_nl();
}

static void cmd_exec(char *line)
{
    char *arg = line;
    uint32_t n;
    uint8_t i;

    for (i = 0; line[i]; i++)                   /* upper-case: not case sensitive*/
        if (line[i] >= 'a' && line[i] <= 'z') line[i] = (char)(line[i] - 32);

    while (*arg && *arg != ' ') arg++;          /* split "CMD ARG"              */
    if (*arg) { *arg++ = '\0'; while (*arg == ' ') arg++; }

    if (!strcmp(line, "HELP")) {
        put_str("CMDS: HELP STATUS TH <ppm> INT <s> MUTE UNMUTE SIM <ppm|ADC|FAIL>");
        put_nl();
    } else if (!strcmp(line, "STATUS")) {
        cmd_status();
    } else if (!strcmp(line, "TH")) {
        if (parse_u32(arg, &n) && n >= TH_MIN && n <= TH_MAX) {
            s_th = (uint16_t)n;
            put_str("OK TH="); put_u32(n); put_nl();
        } else {
            put_str("ERR range 1100-5000"); put_nl();
        }
    } else if (!strcmp(line, "INT")) {
        if (parse_u32(arg, &n) && n >= 1u && n <= 60u) {
            s_report_s = (uint8_t)n;
            s_rep_cnt = 0;
            put_str("OK INT="); put_u32(n); put_nl();
        } else {
            put_str("ERR range 1-60"); put_nl();
        }
    } else if (!strcmp(line, "MUTE") || !strcmp(line, "UNMUTE")) {
        s_muted = (line[0] == 'M');
        outputs_apply();
        put_str("OK MUTE="); put_u32(s_muted ? 1u : 0u); put_nl();
    } else if (!strcmp(line, "SIM")) {
        if (!strcmp(arg, "ADC"))       { s_src = SRC_ADC;  put_str("OK SIM=ADC"); put_nl(); }
        else if (!strcmp(arg, "FAIL")) { s_src = SRC_FAIL; put_str("OK SIM=FAIL"); put_nl(); }
        else if (parse_u32(arg, &n) && n >= 400u && n <= 5000u) {
            s_src = SRC_PPM; s_sim_ppm = (uint16_t)n;
            put_str("OK SIM="); put_u32(n); put_nl();
        } else {
            put_str("ERR range 400-5000"); put_nl();
        }
    } else {
        put_str("ERR unknown cmd (try HELP)"); put_nl();
    }
}

/* Collects characters into a line; executes it when Enter arrives. */
static void line_input(uint8_t c)
{
    if (c == '\r' || c == '\n') {
        if (s_line_discard) { s_line_discard = false; }
        else if (s_line_len > 0) { s_line[s_line_len] = '\0'; cmd_exec(s_line); }
        s_line_len = 0;
        return;
    }
    if (s_line_discard) return;
    if (s_line_len >= LINE_MAX_LEN) {           /* too long: drop whole line    */
        put_str("ERR line too long"); put_nl();
        s_line_discard = true;
        s_line_len = 0;
        return;
    }
    s_line[s_line_len++] = (char)c;
}

/* ---- 3.10 Init and scheduler ------------------------------------------- */
static void app_init(void)
{
    s_blink = false;
    outputs_apply();                            /* start with green LED on      */
    put_str("HomeSense CO2 node v" FW_VERSION " READY (type HELP)");
    put_nl();
}

/* One pass of the main loop: takes snapshots of the ISR flags, then runs the
 * tasks that are due. Called continuously by main(), or once per simulated
 * millisecond by the PC test harness.                                       */
static void app_step(void)
{
    uint8_t a, b, c, ch, st;

    st = hal_irq_save();                        /* read-and-clear flags safely  */
    a = f_10ms; b = f_250ms; c = f_1s;
    f_10ms = 0; f_250ms = 0; f_1s = 0;
    hal_irq_restore(st);

    while (rx_get(&ch)) line_input(ch);         /* handle UART commands         */

    if (a) button_task();
    if (b) { s_blink = !s_blink; outputs_apply(); }
    if (c) sample_task();
}

/* =============================================================================
 * SECTION 4 - HAL FOR REAL HARDWARE: ATmega328P (Arduino Uno, 16 MHz)
 *   Pin map: D8..D10 = PB0..PB2 LEDs, D11 = PB3 buzzer, D2 = PD2 button,
 *            A0 = ADC0 potentiometer, UART0 = USB serial at 9600 baud.
 * ========================================================================== */
#ifdef TARGET_AVR

static void hal_init(void)
{
    DDRB  |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3); /* outputs      */
    DDRD  &= (uint8_t)~(1 << PD2);                              /* button input */
    PORTD |= (1 << PD2);                                        /* pull-up on   */

    /* Timer1, CTC mode, prescaler 64: 16 MHz / 64 = 250 kHz; 250 counts = 1 ms */
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
    OCR1A  = 249;
    TIMSK1 = (1 << OCIE1A);

    /* UART0: 9600 baud, 8N1, RX interrupt on. UBRR = F_CPU/16/9600 - 1 = 103 */
    UBRR0H = 0;
    UBRR0L = (uint8_t)(F_CPU / 16UL / 9600UL - 1UL);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

    /* ADC: AVcc reference, channel 0, prescaler 128 (125 kHz ADC clock) */
    ADMUX  = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    set_sleep_mode(SLEEP_MODE_IDLE);
}

static void hal_led_set(uint8_t mask)
{
    PORTB = (uint8_t)((PORTB & ~0x07u) | (mask & 0x07u));
}

static void hal_buzzer(bool on)
{
    if (on) PORTB |= (1 << PB3); else PORTB &= (uint8_t)~(1 << PB3);
}

static int16_t hal_adc_read(void)
{
    ADCSRA |= (1 << ADSC);                      /* start conversion             */
    while (ADCSRA & (1 << ADSC)) { }            /* wait (about 104 us)          */
    return (int16_t)ADC;
}

static bool    hal_button_pressed(void) { return !(PIND & (1 << PD2)); }
static void    hal_uart_putc(char c)    { while (!(UCSR0A & (1 << UDRE0))) { } UDR0 = (uint8_t)c; }
static uint8_t hal_irq_save(void)       { uint8_t s = SREG; cli(); return s; }
static void    hal_irq_restore(uint8_t s) { SREG = s; }

ISR(TIMER1_COMPA_vect) { timer_tick_isr(); }
ISR(USART_RX_vect)     { uart_rx_isr(UDR0); }

int main(void)
{
    hal_init();
    app_init();
    sei();                                      /* enable interrupts            */
    for (;;) {
        app_step();
        sleep_mode();                           /* idle until next interrupt    */
    }
}

#endif /* TARGET_AVR */

/* =============================================================================
 * SECTION 5 - HAL FOR PC SIMULATION + AUTOMATED SELF-TESTS
 *   The "hardware" is plain variables. A test harness plays the role of the
 *   timer (calling timer_tick_isr() once per simulated millisecond) and of the
 *   UART receiver (calling uart_rx_isr() for each typed byte).
 * ========================================================================== */
#ifdef HOST_SIM

static uint8_t  sim_led;                        /* current LED mask             */
static bool     sim_buz;
static int16_t  sim_adc = 512;                  /* simulated potentiometer      */
static bool     sim_btn;                        /* simulated button level       */
static uint32_t sim_ms;                         /* simulated time               */
static bool     sim_trace, sim_echo = true, sim_bol = true;
static char     cap[16384];                     /* copy of everything sent      */
static size_t   caplen;
/* hardware activity counters used by the tests (cleared by hw_reset()) */
static uint8_t  led_or;                         /* OR of every mask shown       */
static uint16_t led_zero, buz_on;               /* times LED all-off / buzzer on*/

static void hal_init(void) { }
static uint8_t hal_irq_save(void) { return 0; }
static void hal_irq_restore(uint8_t s) { (void)s; }
static int16_t hal_adc_read(void) { return sim_adc; }
static bool hal_button_pressed(void) { return sim_btn; }

static void hal_led_set(uint8_t mask)
{
    if (sim_trace && mask != sim_led)
        printf("  HW> t=%6lums LED G=%d Y=%d R=%d\n", (unsigned long)sim_ms,
               !!(mask & LED_G), !!(mask & LED_Y), !!(mask & LED_R));
    sim_led = mask;
    led_or |= mask;
    if (mask == 0) led_zero++;
}

static void hal_buzzer(bool on)
{
    if (sim_trace && on != sim_buz)
        printf("  HW> t=%6lums BUZZER %s\n", (unsigned long)sim_ms, on ? "ON" : "OFF");
    sim_buz = on;
    if (on) buz_on++;
}

static void hal_uart_putc(char c)
{
    if (caplen < sizeof cap - 1) { cap[caplen++] = c; cap[caplen] = '\0'; }
    if (sim_echo && c != '\r') {
        if (sim_bol) fputs("  UART> ", stdout);
        putchar(c);
        sim_bol = (c == '\n');
    }
}

/* ---- test harness helpers ---------------------------------------------- */
static void run_ms(uint32_t ms)                 /* advance simulated time       */
{
    while (ms--) { sim_ms++; timer_tick_isr(); app_step(); }
}

static void uart_send(const char *s)            /* type text on the "terminal"  */
{
    printf("  YOU > %s\n", s);
    while (*s) uart_rx_isr((uint8_t)*s++);
    uart_rx_isr('\n');
    app_step();
}

static void cap_clear(void) { caplen = 0; cap[0] = '\0'; }
static bool cap_has(const char *s) { return strstr(cap, s) != NULL; }
static int  cap_count(const char *s)
{
    int n = 0;
    const char *p = cap;
    while ((p = strstr(p, s)) != NULL) { n++; p += strlen(s); }
    return n;
}
static void hw_reset(void) { led_or = 0; led_zero = 0; buz_on = 0; }

static int tests, passed;
#define CHECK(cond, name) do { tests++; if (cond) { passed++; \
    printf("[PASS] %s\n", name); } else { \
    printf("[FAIL] %s   (line %d)\n", name, __LINE__); } } while (0)

static int run_tests(void)
{
    /* -- T0: boot ------------------------------------------------------- */
    printf("\n== T0 Boot banner and initial outputs ==\n");
    CHECK(cap_has("READY"), "boot banner printed over UART");
    CHECK(sim_led == LED_G, "green LED on after boot");

    /* -- T1: timer interrupt time base ---------------------------------- */
    printf("\n== T1 Timer interrupt time base ==\n");
    cap_clear();
    run_ms(999);
    CHECK(s_uptime == 0, "no 1 s tick after 999 ms");
    run_ms(1);
    CHECK(s_uptime == 1, "exactly one 1 s tick after 1000 ms");
    run_ms(9000);
    CHECK(s_uptime == 10, "10 ticks after 10 s");

    /* -- T2: ADC to ppm conversion -------------------------------------- */
    printf("\n== T2 ADC to ppm conversion ==\n");
    CHECK(adc_to_ppm(0) == 400,    "ADC 0    -> 400 ppm");
    CHECK(adc_to_ppm(512) == 1200, "ADC 512  -> 1200 ppm");
    CHECK(adc_to_ppm(1023) == 2000,"ADC 1023 -> 2000 ppm");

    /* -- T3: GOOD level --------------------------------------------------- */
    printf("\n== T3 Good air: green LED, periodic report ==\n");
    cap_clear();
    sim_adc = 100;                              /* 556 ppm */
    run_ms(5000);
    CHECK(s_level == LVL_GOOD && sim_led == LED_G, "556 ppm -> GOOD, green LED solid");
    CHECK(!s_alert, "no alert at 556 ppm");
    CHECK(cap_has("RPT CO2=556ppm"), "periodic report shows filtered value");

    /* -- T4: FAIR level --------------------------------------------------- */
    printf("\n== T4 Fair air: yellow LED ==\n");
    sim_adc = 400;                              /* 1025 ppm */
    run_ms(5000);
    CHECK(s_level == LVL_FAIR && sim_led == LED_Y, "1025 ppm -> FAIR, yellow LED solid");

    /* -- T5: alert persistence ------------------------------------------- */
    printf("\n== T5 Alert needs 3 consecutive samples above threshold ==\n");
    cap_clear();
    sim_adc = 900;                              /* 1807 ppm */
    run_ms(4000);
    CHECK(!s_alert, "no alert yet after 4 s (filter ramping, persistence counting)");
    run_ms(1000);
    CHECK(s_alert, "alert active after 5 s");
    CHECK(cap_has("EVT ALERT ON"), "ALERT ON event printed");
    hw_reset();
    run_ms(1000);
    CHECK(led_or == LED_R && led_zero > 0, "red LED blinks (on and off) during alert");
    CHECK(buz_on > 0, "buzzer sounds during alert");

    /* -- T6: mute command ------------------------------------------------- */
    printf("\n== T6 MUTE / UNMUTE over UART ==\n");
    cap_clear();
    uart_send("MUTE");
    CHECK(cap_has("OK MUTE=1"), "MUTE acknowledged");
    hw_reset();
    run_ms(1000);
    CHECK(buz_on == 0 && led_zero > 0, "buzzer silent but red LED still blinks");
    uart_send("unmute");                        /* lower case on purpose */
    hw_reset();
    run_ms(1000);
    CHECK(buz_on > 0, "UNMUTE restores buzzer (command is case-insensitive)");

    /* -- T7: button debounce --------------------------------------------- */
    printf("\n== T7 Button debounce (contact bounce must count once) ==\n");
    cap_clear();
    sim_btn = true;  run_ms(10);                /* bounce: 10 ms pulses */
    sim_btn = false; run_ms(10);
    sim_btn = true;  run_ms(10);
    sim_btn = false; run_ms(10);
    CHECK(!s_muted && cap_count("EVT BUTTON") == 0, "short bounce pulses are ignored (no event)");
    sim_btn = true;  run_ms(100);               /* real press */
    CHECK(s_muted, "stable press toggles mute ON");
    sim_btn = false; run_ms(100);               /* release */
    CHECK(s_muted && cap_count("EVT BUTTON") == 1, "release does nothing; exactly one event");
    sim_btn = true;  run_ms(100);
    sim_btn = false; run_ms(100);
    CHECK(!s_muted, "second press toggles mute OFF");

    /* -- T8: hysteresis --------------------------------------------------- */
    printf("\n== T8 Hysteresis: alert holds until 50 ppm below threshold ==\n");
    cap_clear();
    sim_adc = 691;                              /* 1480 ppm: below TH but inside hysteresis band */
    run_ms(6000);
    uart_send("STATUS");
    CHECK(cap_has("CO2=1480ppm") && cap_has("ALERT=1"), "1480 ppm keeps the alert active");
    sim_adc = 600;                              /* 1338 ppm */
    cap_clear();
    run_ms(6000);
    CHECK(!s_alert && cap_has("EVT ALERT OFF"), "1338 ppm clears the alert");
    CHECK(sim_led == LED_Y, "LED returns to solid yellow");

    /* -- T9: UART commands ----------------------------------------------- */
    printf("\n== T9 UART command interpreter ==\n");
    cap_clear(); uart_send("TH 2000");
    CHECK(cap_has("OK TH=2000") && s_th == 2000, "TH 2000 accepted");
    cap_clear(); uart_send("TH 50");
    CHECK(cap_has("ERR range") && s_th == 2000, "TH 50 rejected, value unchanged");
    cap_clear(); uart_send("TH abc");
    CHECK(cap_has("ERR range"), "non-numeric argument rejected");
    uart_send("TH 1500");
    cap_clear(); uart_send("INT 2");
    CHECK(cap_has("OK INT=2"), "INT 2 accepted");
    cap_clear();
    run_ms(6000);
    CHECK(cap_count("RPT") == 3, "6 s at 2 s interval -> exactly 3 reports");
    uart_send("INT 5");
    cap_clear(); uart_send("FOO");
    CHECK(cap_has("ERR unknown"), "unknown command reported");
    cap_clear(); uart_send("HELP");
    CHECK(cap_has("CMDS:"), "HELP lists the commands");
    cap_clear();
    uart_send("THISLINEISFARTOOLONGTOBEACOMMAND");
    CHECK(cap_has("ERR line too long"), "over-long line rejected safely");
    cap_clear(); uart_send("STATUS");
    CHECK(cap_has("CO2="), "interpreter still works after the long line");

    /* -- T10: SIM override ------------------------------------------------ */
    printf("\n== T10 SIM test mode ==\n");
    cap_clear(); uart_send("SIM 3000");
    CHECK(cap_has("OK SIM=3000"), "SIM 3000 accepted");
    run_ms(8000);
    CHECK(s_alert, "forced 3000 ppm raises the alert");
    cap_clear(); uart_send("SIM 100");
    CHECK(cap_has("ERR range"), "SIM 100 rejected (below 400)");
    uart_send("SIM ADC");
    run_ms(8000);
    CHECK(!s_alert && s_src == SRC_ADC, "back on ADC input, alert cleared");

    /* -- T11: sensor fault ------------------------------------------------ */
    printf("\n== T11 Sensor fault detection and recovery ==\n");
    cap_clear(); uart_send("SIM FAIL");
    hw_reset();
    run_ms(3000);
    CHECK(s_fault && cap_has("EVT SENSOR FAULT"), "3 failed reads declare a FAULT");
    CHECK((led_or & 7u) == 7u, "all three LEDs blink together during fault");
    uart_send("SIM ADC");
    cap_clear();
    run_ms(2000);
    CHECK(!s_fault && cap_has("EVT SENSOR OK"), "fault clears when the sensor returns");

    /* -- T12: UART buffer overflow --------------------------------------- */
    printf("\n== T12 UART ring buffer overflow handling ==\n");
    {
        int i;
        for (i = 0; i < 100; i++) uart_rx_isr('A');   /* burst, main loop not run */
        CHECK(rx_overflow > 0, "overflowing bytes are counted and dropped, no crash");
        cap_clear();
        app_step();                                    /* drain */
        uart_rx_isr('\n');
        app_step();
        uart_send("STATUS");
        CHECK(cap_has("CO2="), "firmware recovers and answers STATUS afterwards");
    }

    printf("\n==============================================\n");
    printf(" RESULT: %d / %d checks passed\n", passed, tests);
    printf("==============================================\n");
    return (passed == tests) ? 0 : 1;
}

/* Interactive mode: lets the user drive the simulated device by hand. */
static void interactive(void)
{
    char ln[96];
    size_t n;
    puts("Interactive mode. Text = UART command. @run N | @adc N | @btn 0/1 | @quit");
    while (fgets(ln, sizeof ln, stdin)) {
        n = strlen(ln);
        while (n && (ln[n - 1] == '\n' || ln[n - 1] == '\r')) ln[--n] = '\0';
        if (!strncmp(ln, "@run ", 5))       run_ms((uint32_t)strtoul(ln + 5, NULL, 10));
        else if (!strncmp(ln, "@adc ", 5))  sim_adc = (int16_t)strtol(ln + 5, NULL, 10);
        else if (!strncmp(ln, "@btn ", 5))  sim_btn = (ln[5] == '1');
        else if (!strcmp(ln, "@quit"))      break;
        else if (n)                         uart_send(ln);
    }
}

int main(int argc, char **argv)
{
    int i;
    bool inter = false;
    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--trace"))       sim_trace = true;
        if (!strcmp(argv[i], "--interactive")) inter = true;
    }
    hal_init();
    app_init();
    if (inter) { interactive(); return 0; }
    return run_tests();
}

#endif /* HOST_SIM */
