#include "stm32f10x.h"
#include "stdbool.h"
//#include "FreeRTOS.h"

/*uint16_t SysTick_CNT = 0;

 void SysTick_Handler(void){

	if (SysTick_CNT > 0) 
	{
		SysTick_CNT--;
	}
}
	
void delayms (uint16_t ms){
	
	SysTick_CNT = ms;
	while (SysTick_CNT>0){}
}
*/

void Encoder_init (void);
bool flag = 0;
void EXTI0_IRQHandler (void){
	EXTI->PR |= EXTI_PR_PR0;
	flag = !flag;
	GPIOC->ODR ^= GPIO_ODR_ODR13;
}

int main (void){
NVIC_EnableIRQ(EXTI0_IRQn);
	//SysTick_Config (72000);
	Encoder_init();
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	
	GPIOC->CRH &= ~GPIO_CRH_CNF13;
	GPIOC->CRH |= GPIO_CRH_MODE13;
	GPIOA->CRL &= ~GPIO_CRL_CNF0;
	GPIOA->CRL |= GPIO_CRL_CNF0_1;
	AFIO->EXTICR[0] &= ~AFIO_EXTICR1_EXTI0;
	EXTI->IMR |= EXTI_IMR_MR0;
	EXTI->RTSR |= EXTI_RTSR_TR0;
	GPIOC->ODR ^= GPIO_ODR_ODR13;
	volatile uint32_t test = TIM3->CNT;;
	//osDelay(100);
	while (1) 
		{
			if (!flag)
			{
			GPIOC->ODR = GPIO_ODR_ODR13;
			for (uint32_t i=0; i<140*(100-(TIM3->CNT)) ;i++);
			GPIOC->ODR &= ~GPIO_ODR_ODR13;
			for (uint32_t i=0; i<140*TIM3->CNT ;i++);
				
			}
			test = TIM3->CNT;
		}
	}

	
	void Encoder_init (void) 
	{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	GPIOA->CRL &= ~(GPIO_CRL_CNF6 | GPIO_CRL_CNF7);
	GPIOA->CRL |= (GPIO_CRL_CNF6_1 | GPIO_CRL_CNF7_1);
	GPIOA->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_MODE7);
	GPIOA->ODR |= (GPIO_ODR_ODR6 | GPIO_ODR_ODR7);
	TIM3->SMCR |= TIM_SMCR_SMS_0;
	TIM3->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
	TIM3->CCMR1 |= TIM_CCMR1_IC1F_3 | TIM_CCMR1_IC1F_1;
	TIM3->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
	TIM3->PSC = 0;   				// Выставляем время 1 тика таймера
	TIM3->ARR = 100;   					// До скольки считаем
	TIM3->CR1 |= TIM_CR1_CEN;
	TIM3->CNT = 50;
	}