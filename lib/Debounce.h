/*
 * Debounce.h
 *
 */

#include "stm32f103xb.h"
#include "stm32f1xx_hal.h"

	class Debounce {
	    public:
		Debounce(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,  uint32_t timeDelay);
		~Debounce();

	    private:

		 GPIO_TypeDef *m_pGPIOx;
		 uint32_t m_timeDelay;
		 uint16_t m_GPIO_Pin;

		 uint32_t m_timeOutFromPressed;

		 uint8_t m_flag_key_press;
		 uint32_t m_time_key_press;

	    private:
		inline bool checkDelayPress();

	    public:
		bool CheckButton();					// if return 1 - button is press	return 0 - button removed
		inline void setTimeDelayButton(uint32_t  timeDelay);




	};


