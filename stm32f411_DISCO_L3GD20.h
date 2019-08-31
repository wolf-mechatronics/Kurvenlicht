/*
 * stm32f411_DISCO_L3GD20.h
 *
 *  Created on: 26.08.2019
 *      Author: Laeuterer
 */

#ifndef STM32F411_DISCO_L3GD20_H_
#define STM32F411_DISCO_L3GD20_H_



void init_L3GD20(void);

void Messwerte_L3GD20(int16_t *roll,int16_t *pitch, int16_t *yaw);


#endif /* STM32F411_DISCO_L3GD20_H_ */
