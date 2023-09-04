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
 * it becomes a flatter response. So [50%, 90%] is a very good line.
 *
 * When sampling the encoder every 100 ms, all six motors have different
 * max pulse readings at 100% duty cycle. I'm assuming 240 pulses at 100%
 * is the maximum for all of them. The CCR for the PWMs is set to 1000. The
 * conversion from the magnitude of the reques to the PWM should follow:
 *
 * Request Magnitude 0 -> 1000
 *
 * 	0     =   0.0%
 * 	1     =  50.0%
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
 */

#include "app.h"

extern I2C_HandleTypeDef hi2c1;

#define STANDARD_MOTOR_COMMAND 0x55u
#define RX_BUFFER_SIZE         4u
// The encoder is shared with the PWMs, and the count is 1000
#define CTR_RESET (1000 >> 1)

static volatile bool g_i2c_data                  = false;
uint8_t              g_rx_buffer[RX_BUFFER_SIZE] = { 0u, 0u, 0u, 0u };

static GPIO_TypeDef * g_a_ports[] = { M1_INA_GPIO_Port, M2_INA_GPIO_Port };
static GPIO_TypeDef * g_b_ports[] = { M1_INB_GPIO_Port, M2_INB_GPIO_Port };
static uint32_t g_a_pins[]  = { M1_INA_Pin, M2_INA_Pin };
static uint32_t g_b_pins[]  = { M1_INB_Pin, M2_INB_Pin };

static pidctl_t g_pids[2];

// TODO: Wonder how much making these global improves perfmance?
static float m1x;
static float m2x;
static float m1y;
static float m2y;

void
HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1)
    {
        g_i2c_data = true;
    }
}

/**
 * Probably a good idea to stop all the motors if something goes wrong?
 * At least put them in "no drive" mode so they just spin.
 */
static void
hard_shutdown(void)
{
    printf("hard_shutdown(): now\n");
}
/**
 * Change the target setpoint for the motor's PID.
 *
 * TODO: once we settle on a pinmap, use GPIO(port)->B[S]RR registers.
 * TODO: should we handle radical changes in direction / magnitude?
 */
static void
set_motor(unsigned nmotor, int16_t speed_dir)
{
    GPIO_PinState a;
    GPIO_PinState b;

    printf("Setting motor %d to %d / 10\n", nmotor, speed_dir);

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

    HAL_GPIO_WritePin(g_a_ports[nmotor - 1], g_a_pins[nmotor], a);
    HAL_GPIO_WritePin(g_b_ports[nmotor - 1], g_b_pins[nmotor], b);

    g_pids[nmotor - 1].target = abs(speed_dir);
}

static void
process_command(void)
{
    uint8_t command   = g_rx_buffer[0];
    uint8_t nmotor    = g_rx_buffer[1];
    int16_t speed_dir = (g_rx_buffer[3] << 8) | g_rx_buffer[2];

    if ((command != STANDARD_MOTOR_COMMAND) || (nmotor < 1 || nmotor > 2)
        || (speed_dir < -1000 || speed_dir > 1000))
    {
        printf("Invalid packet: 0x%02x %d -> %d\n", command, nmotor, speed_dir);
        // hard_shutdown();
        // Error_Handler();
    }
    else
    {
        set_motor(nmotor, speed_dir);
    }
}

void
app_init(void)
{
    HAL_StatusTypeDef stat = HAL_OK;

    printf("app_init: begin\n");
    stat = HAL_I2C_Slave_Receive_IT(&hi2c1, g_rx_buffer, RX_BUFFER_SIZE);
    if (stat != HAL_OK)
    {
        printf("ERROR: Failed to set initial receive: 0x%02x\n", stat);
        Error_Handler();
    }
    printf("app_init: end\n");

    memset(&(g_pids[0]), 0, sizeof(pidctl_t));

    g_pids[0].target   = 0.0f;
    g_pids[0].kp       = 0.0f;
    g_pids[0].ki       = 4.0f;
    g_pids[0].kd       = 0.0f;
    g_pids[0].ymin     = 550.0f;
    g_pids[0].ymax     = 900.0f;
    g_pids[0].period_s = 0.010f;

    memcpy(&(g_pids[1]), &(g_pids[0]), sizeof(pidctl_t));
}

void
update_pids(void)
{
    // Update both pids on the same interval
    if (pidctl_needs_update(&(g_pids[0])))
    {
        // TODO: Motor 1 is TIM2 and Motor 2 is TIM1... seriously dude?

        // Read our 'x' magnitudes...
        m1x = abs(TIM2->CNT - CTR_RESET);
        m2x = abs(TIM1->CNT - CTR_RESET);
        // Get our 'y' values...
        m1y = pidctl_compute(&(g_pids[0]), m1x);
        m2y = pidctl_compute(&(g_pids[1]), m2x);
        // Update the PWM CCRs...
        TIM1->CCR1 = (uint32_t)m2y;
        TIM2->CCR1 = (uint32_t)m1y;
        // Clear the encoders...
        TIM2->CNT = CTR_RESET;
        TIM1->CNT = CTR_RESET;
    }
}

void
app_loop(void)
{
    HAL_StatusTypeDef stat = HAL_OK;

    if (g_i2c_data == true)
    {
        g_i2c_data = false;
        process_command();
        stat = HAL_I2C_Slave_Receive_IT(&hi2c1, g_rx_buffer, RX_BUFFER_SIZE);
        if (stat != HAL_OK)
        {
            printf("ERROR: Failed subsequent receive: 0x%02x\n", stat);
            hard_shutdown();
            Error_Handler();
        }
    }
    else
    {
        update_pids();
    }
}
