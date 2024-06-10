/*
 * StepMotor.h
 *
 *  Created on: Nov 28, 2023
 *      Author: user
 */

#include "stm32f103xb.h"
#include "stm32f1xx_hal.h"
#include "main.h"

#define STEPS_ACCEL_BRAKE 150

class StepMotor{

public:
	StepMotor(GPIO_TypeDef *GPIOx_Enable, uint16_t pin_Enable,GPIO_TypeDef *GPIOx_Dir,
			uint16_t pin_Dir, TIM_HandleTypeDef *htim_PWM, uint32_t Channel);


	uint32_t m_counterSteps;

private:

	enum TYPE_MOTION{
		NO_MOTION = 0,
		READY_TO_MOTION = 1,
		READY_TO_START = 2,
		MOTION = 3,
		ERROR = 9
	};

	enum TYPE_MOTOR_CONTROL{
		DC_MOTOR = 0,
		STEP_MOTOR = 1
	};

	private:

	GPIO_TypeDef *m_GPIOx_Enable;
	GPIO_TypeDef *m_GPIOx_Dir;
	GPIO_TypeDef *m_GPIOx_Step;
	uint16_t m_pin_enable;
	uint16_t m_pin_Dir;
	uint16_t m_pin_step;

	TIM_HandleTypeDef *p_htim_PWM;

	TYPE_MOTOR_CONTROL typeMotorControl;

	int m_AccelIteration = 1; // переделать

	uint32_t m_Channel;
	bool m_direction;
	uint32_t m_nStepsForMotion;
	uint32_t m_MaxSpeed;
	uint32_t m_MinSpeed;
	bool m_Retention;
	uint32_t m_speed;
	uint32_t m_stepEndAcceleration;
	uint32_t m_stepStartBrake;
	uint32_t m_OneStepBrake;
	uint32_t m_OneStepAcceleration;
	TYPE_MOTION m_typeMotion;
	uint32_t m_stepFreqAcceleration;
	uint32_t m_stepFreqBrake;
	uint32_t m_stepsAcceleration;
	uint32_t m_stepsBrake;
	int arr_motionAccel [STEPS_ACCEL_BRAKE];
	int arr_motionBrake [STEPS_ACCEL_BRAKE];

	public:
	void setDirection(uint8_t direction);
	void checkMotorInCallback(TIM_HandleTypeDef *htim);
	inline uint8_t getDirection();
	void prepareCalcMotion(uint32_t steps, uint32_t maxSpeed, uint8_t procentAccelBrake,  uint16_t nStepsAccelBrake);
	//void setMaxSpeed(uint32_t speed);
	//uint32_t getSpeed();
	void setAccelerationStep(uint32_t step,  uint32_t stepEndAccep);
	void setSpeed(uint32_t speed);
	uint32_t getMaxSpeed();
//	void setMinSpeed(uint32_t speed);
//	uint32_t getMinSpeed();
	void setRetention(bool);
	void setBrakeMotorStep(uint32_t stepsBrake, uint32_t pointStartBraking);
	void stopMotion();
	inline int getMotorState();
	void startDC_Motion( uint16_t stepsInOneAccelStep);
	void start();
	void accelerationVelCalculate();

	private:
	void calculateFreqBrakeStep();
	void calculateFreqAccelerationStep();
	void accelerationService(int i);
	void motorService();
	void brakeService(int i);
	void accelerationDCService(uint16_t stepsAccelerate, uint32_t stepMotorInStepAccel);
};




