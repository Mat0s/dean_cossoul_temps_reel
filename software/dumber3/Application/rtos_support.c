/**
 ******************************************************************************
 * @file motors.c
 * @brief motors driver body
 * @author S. DI MERCURIO (dimercur@insa-toulouse.fr)
 * @date December 2023
 *
 ******************************************************************************
 * @copyright Copyright 2023 INSA-GEI, Toulouse, France. All rights reserved.
 * @copyright This project is released under the Lesser GNU Public License (LGPL-3.0-only).
 *
 * @copyright This file is part of "Dumber" project
 *
 * @copyright This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * @copyright This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.

 * @copyright You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 ******************************************************************************
 */

#include "rtos_support.h"

#include "stm32l0xx_ll_tim.h"

/** @addtogroup Application_Software
  * @{
  */

/** @addtogroup RTOS_SUPPORT
 * RTOS support module provide several functions for task run time measurement.
 * @{
 */

/** @addtogroup RTOS_SUPPORT_Private Private
 * @{
 */

uint16_t RTOS_SUPPORT_counter16bitUpper=0;

/**
 * @brief TIM7 Initialization Function
 */
void RTOS_SUPPORT_Init(void) {
	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_HandleTypeDef htim7;

	__HAL_RCC_TIM7_CLK_ENABLE();

	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 0;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 65535;
	htim7.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
	{
		Error_Handler();
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim7, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}

	/* TIM7 interrupt Init */
	LL_TIM_EnableIT_UPDATE(TIM7);
	HAL_NVIC_SetPriority(TIM7_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(TIM7_IRQn);

	RTOS_SUPPORT_counter16bitUpper=0;
	LL_TIM_EnableCounter(TIM7);
}

/**
 * @brief Get 32bit emulated timer based on TIM7
 *
 * @return 32bit timer
 */
uint32_t RTOS_SUPPORT_GetTimer(void) {

	uint16_t currentVal=LL_TIM_GetCounter(TIM7);

	return (uint32_t)((((uint32_t)RTOS_SUPPORT_counter16bitUpper)<<16) + (uint32_t)currentVal);
}

/**
  * @brief This function handles TIM7 update interrupt for 32bit
  */
void TIM7_IRQHandler(void) {
  LL_TIM_ClearFlag_UPDATE(TIM7);

  RTOS_SUPPORT_counter16bitUpper++;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
