/*
 * retargetio.c
 *
 *  Created on: Aug 27, 2023
 *      Author: petertorelli
 */
#include "main.h"

extern UART_HandleTypeDef huart2;

int
__io_putchar(int ch)
{
    uint8_t ch8 = (uint8_t)ch;
    HAL_UART_Transmit(&huart2, &ch8, 1, HAL_MAX_DELAY);
    return ch;
}

int
__io_getchar()
{
    uint8_t ch8;
    HAL_UART_Receive(&huart2, &ch8, 1, HAL_MAX_DELAY);
    return (int)ch8;
}
