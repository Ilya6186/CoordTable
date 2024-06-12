#include "StepMotor.h"


	StepMotor::StepMotor(GPIO_TypeDef *GPIOx_Enable, uint16_t pin_Enable,GPIO_TypeDef *GPIOx_Dir,
			uint16_t pin_Dir, TIM_HandleTypeDef *htim_PWM, uint32_t Channel)
	{
		//---------------------------
		// pinout
		m_GPIOx_Enable = GPIOx_Enable;
		m_GPIOx_Dir = GPIOx_Dir;
		p_htim_PWM = htim_PWM;			// pointer to taimer
		m_Channel = Channel;			// chanel taimer
		m_pin_enable = pin_Enable;		// pin enable
		m_pin_Dir = pin_Dir;

		//---------------------------
		m_speed = 0;					// now speed
		m_MaxSpeed = -1;					// max speed
		m_MinSpeed = 1;					// min speed
		//---------------------------
		m_direction = 0;				// direction 1 - left, 0 - right
		m_counterSteps = 0;				// counter steps
		m_nStepsForMotion = 0;			// all steps
		//---------------------------
		m_Retention = 0;				// retention
		//---------------------------
		m_typeMotion = NO_MOTION;
		//---------------------------
		m_OneStepAcceleration = 0;			// значение на которое увеличивается скорость за один шаг
		m_stepEndAcceleration = 0;		// шаг конца ускорения
		m_OneStepBrake = 0;				// значение на которое уменьшается скорость за один шаг
		m_stepStartBrake = 0;			// шаг начала торможения
		m_stepsAcceleration = 0;


		htim_PWM-> Instance->CR1 |=  TIM_CR1_ARPE;
		//stopMotion();
	//	this->setMinSpeed(1);
	}

	void StepMotor::motorService()		//вызывается в коллбэке
	{
		if(typeMotorControl == DC_MOTOR && m_typeMotion == READY_TO_START)
		{
			m_counterSteps++;
			accelerationDCService(m_counterSteps);
		}

		else if(typeMotorControl == STEP_MOTOR && m_typeMotion == READY_TO_START)
		{
			if(m_counterSteps < m_nStepsForMotion)
				{
					if(m_counterSteps <= m_stepEndAcceleration)
					{
						accelerationService(m_counterSteps);
					}

					if(m_counterSteps >= m_stepStartBrake)
					{
						brakeService(m_counterSteps);
					}

					//m_typeMotion = MOTION;
					m_counterSteps++;
				}
				else
				{
					m_AccelIteration = 1;
					m_speed = 0;
					m_counterSteps = 0;
					stopMotion();
				}
		}

	}

	void  StepMotor::accelerationService(int i)
	{
		if(i % m_OneStepAcceleration == 0)
		{
			if(m_AccelIteration < STEPS_ACCEL_BRAKE)
			{
				setSpeed(arr_motionAccel[m_AccelIteration]);
				m_AccelIteration++;

			}
			else
				return;
		}
	}

	void StepMotor::accelerationDCService(int i)
	{
		if(i % m_OneStepAcceleration == 0)
		{
			if(m_AccelIteration < STEPS_ACCEL_BRAKE)
			{
				setSpeed(arr_motionAccel[m_AccelIteration]);
				m_AccelIteration++;

			}
			else
				return;
		}
	}
	void StepMotor::brakeService(int i)
	{
		if(i % m_OneStepAcceleration == 0)
		{
			if(m_AccelIteration > 0)
			{
				setSpeed(arr_motionAccel[m_AccelIteration - 1]);
				m_AccelIteration--;

			}
			else
				return;

		}
	}

	void StepMotor::checkMotorInCallback(TIM_HandleTypeDef *htim)
	{
		if(htim == this->p_htim_PWM)
			{
				motorService();
			}
	}

	void StepMotor::setDirection(uint8_t in_direction)
	{
		m_direction = in_direction;
	}

	uint8_t StepMotor::getDirection()
	{
		return m_direction;
	}


	void StepMotor::prepareCalcMotion(uint32_t steps, uint32_t maxSpeed, uint8_t procentAccel)
	{
		typeMotorControl = STEP_MOTOR;

		if(m_typeMotion == NO_MOTION)
		{

			m_MaxSpeed = maxSpeed;

			HAL_GPIO_WritePin(m_GPIOx_Enable, m_pin_enable, GPIO_PIN_RESET);

			if(m_direction)
			{
				HAL_GPIO_WritePin(m_GPIOx_Dir, m_pin_Dir, GPIO_PIN_SET);
			}
			else if(!m_direction)
			{
				HAL_GPIO_WritePin(m_GPIOx_Dir, m_pin_Dir, GPIO_PIN_RESET);
			}

			m_nStepsForMotion = steps;

			uint32_t pointStartBrake_Acceleration = m_MaxSpeed * procentAccel / 100;			// 15% ШАГОВ ИСПОЛЬЗУЕМ ДЛЯ РАЗГОНА И ТОРМОЖЕНИЯ

			if(steps < pointStartBrake_Acceleration)
			{
				m_MaxSpeed = steps / 2;
				pointStartBrake_Acceleration = m_MaxSpeed;
			}


			this->setAccelerationStep(STEPS_ACCEL_BRAKE, pointStartBrake_Acceleration);
			this->setBrakeMotorStep(STEPS_ACCEL_BRAKE, pointStartBrake_Acceleration);

			calculateFreqBrakeStep();
			calculateFreqAccelerationStep();


			accelerationVelCalculate();


			m_typeMotion = READY_TO_MOTION;
		}
	}

	void StepMotor::prepareDC_Motion(uint16_t maxSpeed, uint16_t procentAccel)
	{
		typeMotorControl = DC_MOTOR;


		m_MaxSpeed = maxSpeed;
		uint32_t pointStartBrake_Acceleration = m_MaxSpeed * procentAccel / 100;

		this->setAccelerationStep(STEPS_ACCEL_BRAKE, pointStartBrake_Acceleration);

		calculateFreqAccelerationStep();	// до какого шага увеличение частоты
		accelerationVelCalculate();

		m_typeMotion = READY_TO_MOTION;
		//accelerationService(m_counterSteps);

	}

	void StepMotor::start()
	{
		if(m_typeMotion == READY_TO_MOTION)
		{
			m_typeMotion = READY_TO_START;
			HAL_TIM_PWM_Start_IT(p_htim_PWM, m_Channel);
		}

		else
			return;
	}

	void StepMotor::stopMotion()
	{
		HAL_TIM_PWM_Stop(p_htim_PWM, m_Channel); 				// остановить шим
		m_typeMotion = NO_MOTION;
		m_counterSteps = 0;
		//this->setSpeed(1);
	}

/*	void StepMotor::setMaxSpeed(uint32_t maxSpeed)
	{
		m_MaxSpeed = maxSpeed;
	}

	uint32_t StepMotor::getSpeed()
	{
		return m_speed;
	}
*/
	uint32_t StepMotor::getMaxSpeed()
	{
		return m_MaxSpeed;
	}


/*	void StepMotor::setMinSpeed(uint32_t speed)
	{
		m_MinSpeed = speed;
	}

	uint32_t StepMotor::getMinSpeed()
	{
		return m_MinSpeed;
	}
*/
	void StepMotor::setRetention(bool Retention)
	{
		m_Retention = Retention;

		if(m_Retention)
			HAL_GPIO_WritePin(m_GPIOx_Enable, m_pin_enable, GPIO_PIN_RESET);
		else if(!m_Retention)
		{
			HAL_GPIO_WritePin(m_GPIOx_Enable, m_pin_enable, GPIO_PIN_SET);
			this->setSpeed(1);
			//this->stopMotion();
		}

	}

	void StepMotor::setSpeed(uint32_t ARR)
	{

		p_htim_PWM -> Instance -> ARR = ARR - 1;
		p_htim_PWM -> Instance -> CCR1 = ARR / 2 - 1;


	}

	void StepMotor::setAccelerationStep(uint32_t steps, uint32_t stepEndAcceleration)		// до какого шага увеличение частоты
	{
		m_OneStepAcceleration = stepEndAcceleration / steps;
		m_stepEndAcceleration = stepEndAcceleration;
		m_stepsAcceleration = steps;
	}

	void StepMotor::calculateFreqAccelerationStep()		// до какого шага увеличение частоты
	{
		m_stepFreqAcceleration = m_MaxSpeed / m_stepsAcceleration;
	}

	void StepMotor::calculateFreqBrakeStep()		// до какого шага увеличение частоты
	{
		m_stepFreqBrake = m_MaxSpeed / m_stepsBrake;
	}

	void StepMotor::accelerationVelCalculate()
	{

		for(int i = 1; i < STEPS_ACCEL_BRAKE; i++ )
		{
		//	arr_motionAccel[0] = 1;

			if(i == STEPS_ACCEL_BRAKE - 1)
			{
				arr_motionAccel[STEPS_ACCEL_BRAKE - 1] = (HAL_RCC_GetSysClockFreq() / p_htim_PWM->Instance->PSC) /getMaxSpeed();
				return;
			}
			int speed = m_stepFreqAcceleration * i;
			arr_motionAccel[i] = (HAL_RCC_GetSysClockFreq() / p_htim_PWM->Instance->PSC) / speed ;
		}

	}

	void StepMotor::setBrakeMotorStep(uint32_t stepsBrake, uint32_t stepsForEndBraking)
	{
		m_OneStepBrake = stepsForEndBraking / stepsBrake;
		m_stepStartBrake = m_nStepsForMotion - stepsForEndBraking;
		m_stepsBrake = stepsBrake;
	}


	int StepMotor::getMotorState()
	{
		return m_typeMotion;
	}

