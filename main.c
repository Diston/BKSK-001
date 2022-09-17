#include "stm32f10x.h"


uint16_t SysTick_CNT = 0;
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
	


void EXTI0_IRQHandler (void){
	EXTI->PR |= EXTI_PR_PR0;
	GPIOC->ODR ^= GPIO_ODR_ODR13;
}

int main (void){
	NVIC_EnableIRQ(EXTI0_IRQn);
	SysTick_Config (72000);
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
	
	while (1) {
		GPIOC->ODR &= ~GPIO_ODR_ODR13;
  	delayms (500);
		GPIOC->ODR |= GPIO_ODR_ODR13;
		delayms (500);
		}
	}