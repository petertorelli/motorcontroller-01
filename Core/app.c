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
 */

#include "app.h"

extern I2C_HandleTypeDef hi2c1;

#define STANDARD_MOTOR_COMMAND 0x55u
#define RX_BUFFER_SIZE 4u

static volatile bool g_i2c_data     = false;
uint8_t g_rx_buffer[RX_BUFFER_SIZE] = { 0u, 0u, 0u, 0u };




void
HAL_I2C_SlaveRxCpltCallback (I2C_HandleTypeDef * hi2c)
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
 */
static void
set_motor(unsigned nmotor, int16_t speed_dir)
{
	printf("Setting motor %d to %d / 10\n", nmotor, speed_dir);
}

static void
process_command(void)
{
	uint8_t command   = g_rx_buffer[0];
	uint8_t nmotor    = g_rx_buffer[1];
	int16_t speed_dir = (g_rx_buffer[3] << 8) | g_rx_buffer[2];

	if ((command != STANDARD_MOTOR_COMMAND) ||
		(nmotor < 1 || nmotor > 2) ||
		(speed_dir < -1000 || speed_dir > 1000))
	{
		printf("Invalid packet: 0x%02x %d -> %d\n", command, nmotor, speed_dir);
		//hard_shutdown();
		//Error_Handler();
	}
	else
	{
		set_motor(0x1 /* magic # TODO */, speed_dir);
	}
}


static pidctl_t g_pid_motor_1;
static pidctl_t g_pid_motor_2;

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

	memset(&g_pid_motor_1, 0, sizeof(pidctl_t));
	memset(&g_pid_motor_2, 0, sizeof(pidctl_t));

	g_pid_motor_1.kp = 0.0f;
	g_pid_motor_1.ki = 4.0f;
	g_pid_motor_1.kd = 0.0f;
	g_pid_motor_2.kp = 0.0f;
	g_pid_motor_2.ki = 4.0f;
	g_pid_motor_2.kd = 0.0f;


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
	}
}
