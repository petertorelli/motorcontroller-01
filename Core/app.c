/**
 * Every 4-byte packet from the UART channel triggers an interrupt.
 * Byte 0: Command. Always 0x55
 * Byte 1: Motor # mask: M1 = (1 << 0) || M2 = (1 << 1)
 * Byte 2: int16. Speed and direction. Negative = backward.
 *         Encoding from -1000, 1000 => +/-100.0% in .1% incre,emts/
 *
 * Notes:
 *
 * 1. Use PB_7 for I2C1 Data doesn't work. This pin is always pulled low.
 *    Switching to pin PA_10 fixes this. It even screws up with USART1!
 * 2. When sending a fixed number of bytes, clock stretching must be
 *    disabled on the peripheral I2C device (e.g., this one).
 * 3. Switched to UART because couldn't track down 3~5 minute ARLO bug.
 *
 * PWM and Motors:
 *
 * This is written for the "goBILDA Yellowjacket 223 rpm" DC motors. They
 * consist of a ~6000 rpm 12V motor with a gearbox at 26.9:1 reduction.
 * Below ~35 KHz, the response curve becomes very nonlinear. This app is
 * set to drive them at 40 KHz, with an 80 MHz MCU.
 *
 * At 40 KHz, the motor doesn't turn until a duty cycle of ~50%. Above 90%
 * it becomes a flatter response. So [55%, 90%] is a very good line.
 *
 * When sampling the encoder every 100 ms, all six motors have different
 * max pulse readings at 100% duty cycle. I'm assuming 240 pulses at 100%
 * is the maximum for all of them. The CCR for the PWMs is set to 1000. The
 * conversion from the magnitude of the reques to the PWM should follow:
 *
 * Request Magnitude 0 -> 1000
 *
 * 	0     =   0.0%
 * 	1     =  55.0%
 * 	1000  =  90.0%
 *
 * There is no feed-forward on the PID to avoid large di/dt spikes. Instead
 * we rely just on the "I" term to bring the motor up to speed.
 *
 * The encoders are shared with the PWM for both motors. The PWM sets the CRR
 * which is 1000. These 223 rpm motors at full speed 10ms sampling produce
 * ~240 pulses per sample. This means resetting the counter to 500 will at
 * most vary between 260 and 740 counts per sample, when they are reset. So
 * there will not be any overflow/underflow of the encoder at 10ms for these
 * motors. (Changing the motors or sample rate will impact this.)
 *
 *
 * Concerns:
 *
 * - PID filter "I" mode is laggy. Needs better tuning.
 *
 * - The direction of the INA/B motor controllers can change before the pid has
 * a chance to respond. This means if we go from 100.0 to -1.0 it will
 * immediately switch to -100.0 until the PID can respond. We should probably
 * use the quadrature to set the direction control bits and not the requested
 * sign. This discontinuity should be checked for ... somewhere?
 *
 *
 * Bugs:
 * 1. ARLO/AF bugs at 10 msec updates. Removed all the printfs and they went
 * away. Still not driving motors though. Let's try that.
 */

#include "app.h"

extern UART_HandleTypeDef huart1;

extern TIM_HandleTypeDef M1_ENC_TIM;
extern TIM_HandleTypeDef M1_PWM_TIM;
extern TIM_HandleTypeDef M2_ENC_TIM;
extern TIM_HandleTypeDef M2_PWM_TIM;

#define STANDARD_MOTOR_COMMAND 0x55u
#define RX_BUFFER_SIZE         4u
#define TIM_CTR_PERIOD         0x10000u
#define CTR_RESET              (TIM_CTR_PERIOD >> 1)
#define MIN_PWM                500u
#define MAX_PWM                1000u
#define MIN_SPEED              -1000
#define MAX_SPEED              1000
#define MAX_PPR                240u
#define WDT_TIMEOUT_MS         1000u

// From main.c
void reset_uart(void);

// A packet has been DMA'd over UART, process it in main loop
static volatile bool g_pkt_ready = false;

// Don't overwrite buffer data if computing
static volatile bool g_computing_pid = false;

// Expecting periodic packets, timeout of we don't get them
static bool g_in_wdt_timeout = false;

// WDT count
static uint32_t g_wdt = 0u;

// RX is the incoming buffer for the DMA
static uint8_t g_rx_buffer[RX_BUFFER_SIZE] = { 0u, 0u, 0u, 0u };

// RX data is copied to SAVE before before processing to prevent overwrite
static uint8_t g_save_buffer[RX_BUFFER_SIZE] = { 0u, 0u, 0u, 0u };

// Number of packets that have come in since exiting WDT
static uint32_t g_pkt_count = 0u;

// Our pid control
static pidctl_t g_pids[2];

// TODO: Had a plan to use indices, but it failed. Remove these four lines.
static GPIO_TypeDef *g_a_ports[] = { M1_INA_GPIO_Port, M2_INA_GPIO_Port };
static GPIO_TypeDef *g_b_ports[] = { M1_INB_GPIO_Port, M2_INB_GPIO_Port };
static uint32_t      g_a_pins[]  = { M1_INA_Pin, M2_INA_Pin };
static uint32_t      g_b_pins[]  = { M1_INB_Pin, M2_INB_Pin };

static void
prime_uart(void)
{
    HAL_StatusTypeDef stat;

    printf("prime_uart: priming\n");
    stat = HAL_UART_Receive_DMA(&huart1, g_rx_buffer, RX_BUFFER_SIZE);
    if (HAL_OK != stat)
    {
        printf(
            "prime_uart(): ERROR: DMA failed to start (%d) (ErrorCode = %08lx, "
            "rxState = %08lx)\n",
            stat,
            huart1.ErrorCode,
            huart1.RxState);
    }
}

void
HAL_UART_RxCpltCallback(UART_HandleTypeDef *phuart)
{
    if (phuart == &huart1)
    {
        // Might not be necessary, but don't overwrite old data until ready
        if (g_computing_pid != true)
        {
            g_save_buffer[0] = g_rx_buffer[0];
            g_save_buffer[1] = g_rx_buffer[1];
            g_save_buffer[2] = g_rx_buffer[2];
            g_save_buffer[3] = g_rx_buffer[3];
            g_pkt_ready      = true;
        }
    }
}

static void
reset_pids(void)
{
    memset(&(g_pids[0]), 0, sizeof(pidctl_t));

    g_pids[0].target   = 0.0f;
    g_pids[0].kp       = 0.3f;
    g_pids[0].ki       = 0.9f;
    g_pids[0].kd       = 0.00f;
    g_pids[0].ymin     = (float)MIN_PWM;
    g_pids[0].ymax     = (float)MAX_PWM;
    g_pids[0].period_s = 0.10f;

    memcpy(&(g_pids[1]), &(g_pids[0]), sizeof(pidctl_t));
}

static void
hard_shutdown(void)
{
    // Turn off PWMs before anything else (even printf)
    M2_PWM_TIM.Instance->M2_CCR = 0;
    M1_PWM_TIM.Instance->M1_CCR = 0;

    printf("hard_shutdown(): now\n");

    // Reset motor direction GPIOs
    HAL_GPIO_WritePin(g_a_ports[0], g_a_pins[0], GPIO_PIN_RESET);
    HAL_GPIO_WritePin(g_b_ports[0], g_b_pins[0], GPIO_PIN_RESET);
    HAL_GPIO_WritePin(g_a_ports[1], g_a_pins[1], GPIO_PIN_RESET);
    HAL_GPIO_WritePin(g_b_ports[1], g_b_pins[1], GPIO_PIN_RESET);

    // Reset PIDs so that if we re-start we don't start w/high PWM
    reset_pids();
}

/**
 * Change the target setpoint for the motor's PID.
 *
 * TODO: once we settle on a pinmap, use GPIO(port)->B[S]RR registers.
 * TODO: should we handle radical changes in direction / magnitude?
 */
static void
set_motor(unsigned mask, int16_t speed_dir)
{
    GPIO_PinState a;
    GPIO_PinState b;
    float         t;

    a = GPIO_PIN_RESET;
    b = GPIO_PIN_RESET;

    if (speed_dir < 0)
    {
        b = GPIO_PIN_SET;
    }
    else if (speed_dir > 0)
    {
        a = GPIO_PIN_SET;
    }

#ifdef HARD0
    // Don't scale out of zero.
    if (speed_dir == 0)
    {
        t = 0.0f;
        if (mask & 1)
        {
            printf("Stop 1\n");
            M1_PWM_TIM.Instance->M1_CCR = 0;
            HAL_GPIO_WritePin(g_a_ports[0], g_a_pins[0], GPIO_PIN_RESET);
            HAL_GPIO_WritePin(g_b_ports[0], g_b_pins[0], GPIO_PIN_RESET);
        }
        if (mask & 2)
        {
            printf("Stop 2\n");
            M2_PWM_TIM.Instance->M2_CCR = 0;
            HAL_GPIO_WritePin(g_a_ports[1], g_a_pins[1], GPIO_PIN_RESET);
            HAL_GPIO_WritePin(g_b_ports[1], g_b_pins[1], GPIO_PIN_RESET);
        }
    }
    else
    {
        t = abs(speed_dir);
        t = (t / MAX_SPEED) * MAX_PPR;
    }
#else
    t = abs(speed_dir);
    t = (t / MAX_SPEED) * MAX_PPR;
#endif

    // TODO: Changing direction before setting the PID is discontinuous
    if (mask & 1)
    {
        //    	printf("Set target 1 to %f (%d)\n", t, speed_dir);
        g_pids[0].target = t;
        HAL_GPIO_WritePin(g_a_ports[0], g_a_pins[0], a);
        HAL_GPIO_WritePin(g_b_ports[0], g_b_pins[0], b);
    }
    if (mask & 2)
    {
        //   	printf("Set target 2 to %f (%d)\n", t, speed_dir);
        g_pids[1].target = t;
        HAL_GPIO_WritePin(g_a_ports[1], g_a_pins[1], a);
        HAL_GPIO_WritePin(g_b_ports[1], g_b_pins[1], b);
    }
}

static void
parse_pkt(void)
{
    uint8_t command   = g_save_buffer[0];
    uint8_t mask      = g_save_buffer[1];
    int16_t speed_dir = (g_save_buffer[3] << 8) | g_save_buffer[2];

    ++g_pkt_count;

    if (g_pkt_count % 100 == 0)
    {
        printf("%lu\n", g_pkt_count);
    }

    if ((command != STANDARD_MOTOR_COMMAND) || (mask < 1 || mask > 3)
        || (speed_dir < MIN_SPEED || speed_dir > MAX_SPEED))
    {
        printf("parse_pkt: ERROR 0x%02x %d %d (%02x %02x %02x %02x)\n",
               command,
               mask,
               speed_dir,
               g_save_buffer[0],
               g_save_buffer[1],
               g_save_buffer[2],
               g_save_buffer[3]);
        //hard_shutdown();
        HAL_Delay(100);
        reset_uart();
        prime_uart();
        HAL_Delay(100);
        // Error_Handler();
    }
    else
    {
        // printf("parse_pkt: Set mask %d to %d / 10\n", mask, speed_dir);
        set_motor(mask, speed_dir);
    }
}

void
app_init(void)
{
    printf("app_init: begin\n");

    printf("app_init: reset PIDs\n");
    reset_pids();

    printf("app_init: start encoders\n");
    M1_ENC_TIM.Instance->CNT = CTR_RESET;
    M2_ENC_TIM.Instance->CNT = CTR_RESET;
    // Don't forget to set T1T2 encoder mode for QUADRATURE
    HAL_TIM_Encoder_Start(&M1_ENC_TIM, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&M2_ENC_TIM, TIM_CHANNEL_1 | TIM_CHANNEL_2);

    printf("app_init: setting motors to free-spin\n");
    HAL_GPIO_WritePin(g_a_ports[0], g_a_pins[0], GPIO_PIN_RESET);
    HAL_GPIO_WritePin(g_b_ports[0], g_b_pins[0], GPIO_PIN_RESET);
    HAL_GPIO_WritePin(g_a_ports[1], g_a_pins[1], GPIO_PIN_RESET);
    HAL_GPIO_WritePin(g_b_ports[1], g_b_pins[1], GPIO_PIN_RESET);

    printf("app_init: start PWMs\n");
    M1_PWM_TIM.Instance->M1_CCR = 0;
    M2_PWM_TIM.Instance->M2_CCR = 0;
    HAL_TIM_PWM_Start(&M1_PWM_TIM, M1_PWM_CH);
    HAL_TIM_PWM_Start(&M2_PWM_TIM, M2_PWM_CH);

    printf("app_init: waiting for %u DMA bytes over UART\n", RX_BUFFER_SIZE);
    prime_uart();

    printf("app_init: end\n");
}

void
update_pids(void)
{
    float m1x;
    float m1y;
    float m2x;
    float m2y;

    // Update both pids on the same interval
    if (pidctl_needs_update(&(g_pids[0])))
    {
        g_computing_pid = true;
        // Read our 'x' magnitudes...
        m1x = abs(M1_ENC_TIM.Instance->CNT - CTR_RESET);
        m2x = abs(M2_ENC_TIM.Instance->CNT - CTR_RESET);
        // Get our 'y' values...
        m1y = pidctl_compute(&(g_pids[0]), m1x);
        m2y = pidctl_compute(&(g_pids[1]), m2x);
        // Update the PWM CCRs...
        M1_PWM_TIM.Instance->M1_CCR = (uint32_t)m1y;
        M2_PWM_TIM.Instance->M2_CCR = (uint32_t)m2y;
        // Clear the encoders...
        M1_ENC_TIM.Instance->CNT = CTR_RESET;
        M2_ENC_TIM.Instance->CNT = CTR_RESET;

        /*
        printf("t[%5.1f] m1 %5.1f -> %5.1f er = %5.1f ", g_pids[0].target, m1x,
        m1y, g_pids[0].error); printf("t[%5.1f] m2 %5.1f -> %5.1f er = %5.1f ",
        g_pids[1].target, m2x, m2y, g_pids[1].error); printf("\n");
        */
        // printf("%f\n", m1x);
        g_computing_pid = false;
    }
}

// See SysTick handler in stem3l4xx_it.c
void
my_systick(void)
{
    ++g_wdt;
}

void
app_loop(void)
{
    // If we don't see a packet after WDT_TIMEOUT, reset errything
    if (g_wdt >= WDT_TIMEOUT_MS && !g_in_wdt_timeout)
    {
        g_wdt       = 0;
        g_pkt_count = 0;
        printf("app_loop: Enter WDT timeout\n");
        hard_shutdown();
        HAL_Delay(100);
        reset_uart();
        HAL_Delay(100);
        prime_uart();
        g_in_wdt_timeout = true;
    }

    if (true == g_pkt_ready)
    {
        if (g_in_wdt_timeout)
        {
            printf("app_loop_ Exit WDT timeout\n");
            g_in_wdt_timeout = false;
        }
        g_wdt       = 0;
        g_pkt_ready = false;
        parse_pkt();
    }

    update_pids();
}
