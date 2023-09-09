/**
 * Every 4-byte packet from the I2C channel triggers an interrupt.
 * Byte 0: Command. Always 0x55
 * Byte 1: Motor number. Must be 1 or 2
 * Byte 2: int16. Speed and direction. Negative = backward.
 *         Encoding from -1000, 1000 => +/-100.0% in .1% incre,emts/
 *
 * Notes:
 *
 * 1. Use PB_7 for I2C1 Data doesn't work. This pin is always pulled low.
 *    Switching to pin PA_10 fixes this.
 * 2. When sending a fixed number of bytes, clock stretching must be
 *    disabled on the peripheral I2C device (e.g., this one).
 *
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
 * - The direction of the INA/B motor controllers can change before the pid has a chance
 *   to respond. This means if we go from 100.0 to -1.0 it will immediately switch to
 *   -100.0 until the PID can respond. We should probably use the quadrature to set the
 *   direction control bits and not the requested sign. This discontinuity should be
 *   checked for ... somewhere?
 */

#include "app.h"

extern I2C_HandleTypeDef SERVER_I2C;

extern TIM_HandleTypeDef M1_ENC_TIM;
extern TIM_HandleTypeDef M1_PWM_TIM;
extern TIM_HandleTypeDef M2_ENC_TIM;
extern TIM_HandleTypeDef M2_PWM_TIM;

#define STANDARD_MOTOR_COMMAND 0x55u
#define RX_BUFFER_SIZE         4u
#define TIM_CTR_PERIOD         0x10000u
#define CTR_RESET              (TIM_CTR_PERIOD >> 1)
#define MIN_PWM                0u
#define MAX_PWM                1000u
#define MIN_SPEED              -1000
#define MAX_SPEED              1000
#define MAX_PPR                240u

#define NMOTOR_1 0u
#define NMOTOR_2 1u

static volatile bool g_i2c_data                  = false;
uint8_t              g_rx_buffer[RX_BUFFER_SIZE] = { 0u, 0u, 0u, 0u };

static GPIO_TypeDef *g_a_ports[] = { M1_INA_GPIO_Port, M2_INA_GPIO_Port };
static GPIO_TypeDef *g_b_ports[] = { M1_INB_GPIO_Port, M2_INB_GPIO_Port };
static uint32_t      g_a_pins[]  = { M1_INA_Pin, M2_INA_Pin };
static uint32_t      g_b_pins[]  = { M1_INB_Pin, M2_INB_Pin };

static pidctl_t g_pids[2];

// TODO: Wonder how much making these global improves perfmance?
static float m1x;
static float m2x;
static float m1y;
static float m2y;

void
HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &SERVER_I2C)
    {
        g_i2c_data = true;
    }
}

static void
hard_shutdown(void)
{
    printf("hard_shutdown(): now\n");
	M1_PWM_TIM.Instance->M1_CCR = 0;
	HAL_GPIO_WritePin(g_a_ports[0], g_a_pins[0], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(g_b_ports[0], g_b_pins[0], GPIO_PIN_RESET);
	M2_PWM_TIM.Instance->M2_CCR = 0;
	HAL_GPIO_WritePin(g_a_ports[1], g_a_pins[1], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(g_b_ports[1], g_b_pins[1], GPIO_PIN_RESET);
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
        a = GPIO_PIN_SET;
    }
    else if (speed_dir > 0)
    {
        b = GPIO_PIN_SET;
    }

    // Don't scale out of zero.
    if (speed_dir == 0)
    {
        t = 0.0f;
        if (mask & 1)
        {
			M1_PWM_TIM.Instance->M1_CCR = 0;
			HAL_GPIO_WritePin(g_a_ports[0], g_a_pins[0], GPIO_PIN_RESET);
			HAL_GPIO_WritePin(g_b_ports[0], g_b_pins[0], GPIO_PIN_RESET);
        }
        if (mask & 2)
        {
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
    // TODO: Changing direction before setting the PID is discontinuous
    if (mask & 1)
    {
    	g_pids[0].target = t;
        HAL_GPIO_WritePin(g_a_ports[0], g_a_pins[0], a);
        HAL_GPIO_WritePin(g_b_ports[0], g_b_pins[0], b);

    }
    if (mask & 2)
    {
    	g_pids[1].target = t;
        HAL_GPIO_WritePin(g_a_ports[1], g_a_pins[1], a);
        HAL_GPIO_WritePin(g_b_ports[1], g_b_pins[1], b);

    }
}

static void
parse_i2c(void)
{
    uint8_t command   = g_rx_buffer[0];
    uint8_t mask      = g_rx_buffer[1];
    int16_t speed_dir = (g_rx_buffer[3] << 8) | g_rx_buffer[2];

    if ((command != STANDARD_MOTOR_COMMAND) || (mask < 1 || mask > 3)
        || (speed_dir < MIN_SPEED || speed_dir > MAX_SPEED))
    {
        printf("parse_i2c: ERROR 0x%02x %d %d\n", command, mask, speed_dir);
        hard_shutdown();
        Error_Handler();
    }
    else
    {
        printf("parse_i2c: Set mask %d to %d / 10\n", mask, speed_dir);
        set_motor(mask, speed_dir);
    }
}

void
app_init(void)
{
    HAL_StatusTypeDef stat = HAL_OK;

    printf("app_init: begin\n");

    printf("app_init: priming i2c\n");
    stat = HAL_I2C_Slave_Receive_IT(&SERVER_I2C, g_rx_buffer, RX_BUFFER_SIZE);
    if (stat != HAL_OK)
    {
        printf("app_init: ERROR Failed to set receive: 0x%02x\n", stat);
        Error_Handler();
    }

    memset(&(g_pids[0]), 0, sizeof(pidctl_t));

    g_pids[0].target   = 0.0f;
    g_pids[0].kp       = 0.0f;
    g_pids[0].ki       = 1.0f;
    g_pids[0].kd       = 0.0f;
    g_pids[0].ymin     = (float)MIN_PWM;
    g_pids[0].ymax     = (float)MAX_PWM;
    g_pids[0].period_s = 0.10f;

    memcpy(&(g_pids[1]), &(g_pids[0]), sizeof(pidctl_t));

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

    printf("app_init: end\n");
}

void
update_pids(void)
{
    // Update both pids on the same interval
    if (pidctl_needs_update(&(g_pids[0])))
    {
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
    }
}

void
app_loop(void)
{
    HAL_StatusTypeDef stat = HAL_OK;

    if (g_i2c_data == true)
    {
        g_i2c_data = false;
        parse_i2c();
        stat = HAL_I2C_Slave_Receive_IT(
            &SERVER_I2C, g_rx_buffer, RX_BUFFER_SIZE);
        if (stat != HAL_OK)
        {
            printf("app_loop: ERROR: Failed to set receive: 0x%02x\n", stat);
            hard_shutdown();
            Error_Handler();
        }
    }
    else
    {
        update_pids();
    }
}
