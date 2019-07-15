#include "Drv_PwmOut.h"
#include "pwm.h"
#include "hw_types.h"
#include "hw_gpio.h"

void Drv_PwmOutInit(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);	
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);	
	ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOA );
	ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOB );
	ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOF );
	/* Set divider to 80M/64=0.8us ����Ϊ0.8*/
	ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_64); 
	ROM_SysCtlDelay(2);
	/*GPIO������*/
	ROM_GPIOPinConfigure(M0TO_PWM1_FUNCTION);
	ROM_GPIOPinConfigure(M0TO_PWM2_FUNCTION);
	ROM_GPIOPinConfigure(M0TO_PWM3_FUNCTION);
	ROM_GPIOPinConfigure(M0TO_PWM4_FUNCTION);
	ROM_GPIOPinTypePWM(GPIOB_BASE, GPIO_PIN_6);//M0PWM0
	ROM_GPIOPinTypePWM(GPIOB_BASE, GPIO_PIN_7);//M0PWM1
	ROM_GPIOPinTypePWM(GPIOB_BASE, GPIO_PIN_4);//M0PWM2
	ROM_GPIOPinTypePWM(GPIOB_BASE, GPIO_PIN_5);//M0PWM3
	/*PF0��������*/
	HWREG(GPIOF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; 
	HWREG(GPIOF_BASE + GPIO_O_CR) = GPIO_PIN_0;
	HWREG(GPIOF_BASE + GPIO_O_LOCK) = 0x00;
	ROM_GPIOPinConfigure(M0TO_PWM5_FUNCTION);
	ROM_GPIOPinConfigure(M0TO_PWM6_FUNCTION);
	ROM_GPIOPinConfigure(M0TO_PWM7_FUNCTION);
	ROM_GPIOPinConfigure(M0TO_PWM8_FUNCTION);
	ROM_GPIOPinTypePWM(GPIOF_BASE, GPIO_PIN_0);//M1PWM4
	ROM_GPIOPinTypePWM(GPIOF_BASE, GPIO_PIN_1);//M1PWM5
	ROM_GPIOPinTypePWM(GPIOF_BASE, GPIO_PIN_2);//M1PWM6
	ROM_GPIOPinTypePWM(GPIOF_BASE, GPIO_PIN_3);//M1PWM7
	
	ROM_GPIOPinConfigure(HEAT_PWM_FUNCTION);
	ROM_GPIOPinTypePWM(GPIOA_BASE, GPIO_PIN_7);//M1PWM3
	/*��PWM����������Ϊ����ʱģʽ�����������²���*/
	ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
	ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	/*����Ϊ0.8us*3125=2500us=2.5ms(400 Hz)*/
	ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, PWM_PERIOD_MAX); 
	ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, PWM_PERIOD_MAX);
	ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, PWM_PERIOD_MAX); 
	ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, PWM_PERIOD_MAX); 
	ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, PWM_PERIOD_MAX);
	/*ʹ�ܶ�ʱ��*/	
	ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_0);
	ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_1);
	ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_1);
	ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_2);
	ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_3);
	/* ʹ����� */
	ROM_PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
	ROM_PWMOutputState(PWM1_BASE, PWM_OUT_4_BIT | PWM_OUT_5_BIT | PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);
	
	ROM_PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
	
	/* ����������� */	
	Drv_HeatSet(0);
	for ( u8 i=0; i<8; i++)
	{
		Drv_MotorPWMSet(i, 0);
	}
}
/**********************************************************************************************************
*�� �� ��: Drv_MotorPWMSet
*����˵��: ���PWM���ֵ����
*��    ��: PWMֵ��0-1000��
*�� �� ֵ: ��
*��    ע:����������(������Ϊ400hz 2.5ms)
*         ����������ռ�ձ�(����PWMЭ��Ӧ��Ϊ1250/3125     ~   2500/3125)
											40%(�������) ~   80%(�������)
**********************************************************************************************************/
void Drv_MotorPWMSet(uint8_t Motor,uint16_t PwmValue)
{
	/*ʹ��PWMЭ�����*/
  u16 tempval ;
	if(PwmValue>999) PwmValue = 999;
	tempval = 1.25f*PwmValue+1250.0f;//0-1000��Ӧ1250-2500
	/*���ñȽϲ���Ĵ�����Ԥװ��ֵ*/
	if(Motor == 0)
		ROM_PWMPulseWidthSet(PWM1_BASE,PWM_OUT_4,tempval);
	if(Motor == 1)
		ROM_PWMPulseWidthSet(PWM1_BASE,PWM_OUT_5,tempval);
	if(Motor == 2)	
		ROM_PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,tempval);
	if(Motor == 3)	
		ROM_PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,tempval);
//	if(Motor == 4)	
//			ROM_PWMPulseWidthSet(PWM0_BASE,PWM_OUT_0,tempval);

//	if(Motor == 5)	
//		ROM_PWMPulseWidthSet(PWM0_BASE,PWM_OUT_1,tempval);

//	if(Motor == 6)	
//		ROM_PWMPulseWidthSet(PWM1_BASE,PWM_OUT_6,tempval);
//	if(Motor == 7)	
//		ROM_PWMPulseWidthSet(PWM1_BASE,PWM_OUT_7,tempval);
}
/**********************************************************************************************************
*�� �� ��: Drv_HeatSet
*����˵��: ����PWM���ֵ����
*��    ��: PWMֵ��0-1000��
*�� �� ֵ: ��
*��    ע:����������(������Ϊ400hz 2.5ms)
**********************************************************************************************************/
void Drv_HeatSet(u16 val)
{
	u16 tmpval = PWM_PERIOD_MAX  * val / 1000;
	if(tmpval > (PWM_PERIOD_MAX-1))
		tmpval = (PWM_PERIOD_MAX-1);
	ROM_PWMPulseWidthSet(PWM1_BASE,PWM_OUT_3,tmpval);	
}
