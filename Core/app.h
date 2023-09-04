/*
 * app.h
 *
 *  Created on: Aug 27, 2023
 *      Author: petertorelli
 */

#ifndef APP_H_
#define APP_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "main.h"
#include "pidctl.h"

void app_init(void);
void app_loop(void);

#endif /* APP_H_ */
