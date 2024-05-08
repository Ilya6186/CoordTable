/*
 * Debounce.cpp
 *
 *  Created on: Nov 4, 2023
 *      Author: Dom
 */

#include "Debounce.h"

Debounce::Debounce(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,  uint32_t timeDelay)
{
	// TODO Auto-generated constructor stub
	m_pGPIOx = GPIOx;
	m_timeDelay = timeDelay;
	m_GPIO_Pin = GPIO_Pin;
	m_timeOutFromPressed = 0;

	 m_flag_key_press = 1;
	 m_time_key_press = 0;
}

bool Debounce:: CheckButton()
{

	if(HAL_GPIO_ReadPin(m_pGPIOx, m_GPIO_Pin) == GPIO_PIN_SET && m_flag_key_press) // записываем время нажатия
	{
	m_flag_key_press = 0;
	m_time_key_press = HAL_GetTick();
	}
	else if (HAL_GPIO_ReadPin(m_pGPIOx, m_GPIO_Pin) == GPIO_PIN_SET && !m_flag_key_press)	// если кнопка всё ещё нажата но флаг уже не 1 (проверяем не первый раз)
		{
		m_timeOutFromPressed = HAL_GetTick() - m_time_key_press;						// суммируем время которое конопка нажата
	}
	else if (HAL_GPIO_ReadPin(m_pGPIOx, m_GPIO_Pin) == GPIO_PIN_RESET)					// если кнопка сброшена то всё по дефолту, суть в том что если хоть раз проскочил низкий уровень то считатем заново
		{
		m_timeOutFromPressed = 0;

		 m_flag_key_press = 1;
		 m_time_key_press = 0;
	}

	if(checkDelayPress())
	{
		return 1;
	}

return 0;
}

bool Debounce:: checkDelayPress()
{
if(m_timeOutFromPressed >  m_timeDelay)
{
	m_flag_key_press = 0;
	return 1;
}

else
	return 0;
}

void Debounce:: setTimeDelayButton(uint32_t timeDelay)
{
	m_timeDelay = timeDelay;
}


Debounce::~Debounce() {
	// TODO Auto-generated destructor stub
}

