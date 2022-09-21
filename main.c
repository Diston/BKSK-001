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
volatile uint8_t portlcd;

void Encoder_init (void);
void I2C1_Init (void);
void LCDSendDec (uint8_t LCDAddr, uint32_t LCDData);
void LCDSendReg (uint8_t LCDAddr, uint32_t LCDData);
void LCDClear (uint8_t LCDAddr);
void LCDSendString (uint8_t LCDAddr, char* LCDData);
void LCDSendChar (uint8_t LCDAddr, char LCDData);
void LCDLight (void);
void LCDSendByte (uint8_t LCDAddr, uint8_t LCDData, uint8_t mode);
void LCDSendHalfByte (uint8_t LCDAddr, uint8_t LCDData);
void LCD_Send (uint8_t LCDAddr, uint8_t LCDData);
volatile uint32_t regis;

bool flag = 0;
void EXTI0_IRQHandler (void){
	EXTI->PR |= EXTI_PR_PR0;
	flag = !flag;
	GPIOC->ODR ^= GPIO_ODR_ODR13;
}

int main (void){
	RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_PPRE1)) | RCC_CFGR_PPRE1_2;
	regis = RCC->CFGR;
NVIC_EnableIRQ(EXTI0_IRQn);
	//SysTick_Config (72000);
	Encoder_init();
	I2C1_Init();
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
	
	for (uint32_t i=0; i<150000; i++);
	LCDSendHalfByte(0x4E, 0x03);
	for (uint32_t i=0; i<20000; i++);
	LCDSendHalfByte(0x4E, 0x03);
	for (uint32_t i=0; i<3300; i++);
	LCDSendHalfByte(0x4E, 0x03);
	for (uint32_t i=0; i<3300; i++);
	LCDSendHalfByte(0x4E, 0x02);
	for (uint32_t i=0; i<3300; i++);
	LCDSendByte(0x4e, 0x28, 0);
	for (uint32_t i=0; i<3300; i++);
	LCDSendByte(0x4e, 0x0C, 0);
	for (uint32_t i=0; i<3300; i++);
	LCDSendByte(0x4e, 0x01, 0);
	for (uint32_t i=0; i<3300; i++);
	LCDSendByte(0x4e, 0x06, 0);
	for (uint32_t i=0; i<3300; i++);
	LCDSendByte(0x4e, 0x02, 0);
	for (uint32_t i=0; i<3300; i++);
	LCD_Send (0x4E, portlcd|=0x08);
	for (uint32_t i=0; i<3300; i++);
	LCDSendByte(0x4E, 0x84, 0);
		LCDSendString (0x4E, "Hello WORLD");;
		LCDSendByte(0x4E, 0xC5, 0);
		LCDSendString (0x4E, "DiSToN Inc");
		LCDSendByte(0x4E, 0x80+27, 0);
		LCDSendString (0x4E, "Present");
		LCDSendByte(0x4E, 0xC0+24, 0);
		LCDSendString (0x4E, "STM32F103C8T6");
	
	//osDelay(100);
	while (1) 
		{
			GPIOC->ODR ^= GPIO_ODR_ODR13;
			for (uint32_t i=0; i<3300000; i++);
	/*		if (!flag)
			{
		//	GPIOC->ODR = GPIO_ODR_ODR13;
		//	for (uint32_t i=0; i<140*(100-(TIM3->CNT)) ;i++);
		//	GPIOC->ODR &= ~GPIO_ODR_ODR13;
		//	for (uint32_t i=0; i<140*TIM3->CNT ;i++);
				
			}
			test = TIM3->CNT;
	*/
		}
	}

	
	void Encoder_init (void) {
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
	
	
	void I2C1_Init (void){
	
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	GPIOB->CRL |= (GPIO_CRL_CNF6 | GPIO_CRL_CNF7);
	GPIOB->CRL |= (GPIO_CRL_MODE6 | GPIO_CRL_MODE7);
//	GPIOB->ODR |=  (GPIO_ODR_ODR10 | GPIO_ODR_ODR11);
	I2C1->CR1 &= ~I2C_CR1_PE;																		// Отключаем шину I2C
	I2C1->CR2 |= ((I2C1->CR2 & ~I2C_CR2_FREQ) | 36);								// Устанавливаем частоту шины APB2 36 МГц (110000)
	I2C1->CCR |= 0xB4;																					// Устанавливаем частоту шины I2C 100 кГц (standart) (f.APB/x = 200000)
	I2C1->TRISE = ((I2C1->TRISE & ~I2C_TRISE_TRISE) | 33);    // Устанавливаем время нарастания сигнала (rise time) 1000 нс (1000 нс / (1/f.AFP))
	I2C1->CR1 &= ~I2C_CR1_NOSTRETCH;															// Включаем ожидание ответа
	I2C1->CR1 |= I2C_CR1_PE;
	I2C1->CR1 |= I2C_CR1_ACK;
	}
	
	
	void LCD_Send (uint8_t LCDAddr, uint8_t LCDData){
	
	I2C1->CR1 &= ~I2C_CR1_POS;
	
	I2C1->CR1 |= I2C_CR1_START;
		while (!(I2C1->SR1 & I2C_SR1_SB));

	I2C1->DR = ((I2C1->DR & ~I2C_DR_DR) | LCDAddr);
	while (!(I2C1->SR1 & I2C_SR1_ADDR));
	
	volatile uint16_t LCDReadSR2 = I2C1->SR2;
	
	I2C1->DR = (I2C1->DR & ~I2C_DR_DR) | LCDData;
  while (!(I2C1->SR1 & I2C_SR1_TXE));
	
	I2C1->CR1 |= I2C_CR1_STOP;
}
	
	void LCDSendHalfByte (uint8_t LCDAddr, uint8_t LCDData){
	LCDData<<=4;
	LCD_Send (LCDAddr, portlcd|=0x04);
	LCD_Send (LCDAddr, portlcd|LCDData);
	LCD_Send (LCDAddr, portlcd&=~0x04);
}

void LCDSendByte (uint8_t LCDAddr, uint8_t LCDData, uint8_t mode){
	if (mode==0)  LCD_Send (LCDAddr, portlcd&=~0x01);
	else LCD_Send (LCDAddr, portlcd|=0x01);
	uint8_t hc=0;
	hc=LCDData>>4;
	LCDSendHalfByte (LCDAddr, hc);
	LCDSendHalfByte (LCDAddr, LCDData);
}

void LCDLight (void) {
	
	LCD_Send (0x4E, portlcd|=0x08);

}

void LCDSendChar (uint8_t LCDAddr, char LCDData){
	
	LCDSendByte (LCDAddr, (uint8_t) LCDData, 1);
	
}

void LCDSendString (uint8_t LCDAddr, char* LCDData){
	
	uint8_t i=0;
	
	while (LCDData[i]!=0)
	{
		LCDSendByte (LCDAddr, LCDData[i], 1);
		i++;
	}
}

void LCDClear (uint8_t LCDAddr){
	LCDSendByte (LCDAddr, 0x01, 0);
}

void LCDSendReg (uint8_t LCDAddr, uint32_t LCDData){
	for (int8_t i=28; i>=0; i=i-4){
		int32_t LCDReg = LCDData>>i;
		LCDReg &= 0xF;
		if (LCDReg <=0x9) {
			LCDSendChar (LCDAddr, LCDReg + '0');
		}
		else {
			LCDSendChar (LCDAddr, LCDReg - 0x9 + '@');
		}
	}
}

void LCDSendDec (uint8_t LCDAddr, uint32_t LCDData){

	int8_t i=0;
	uint32_t LCDData2 = LCDData;
	uint8_t LCDData3[10];
	
	while (LCDData2>0){
		LCDData3[i] = LCDData2 % 10;
		LCDData2 = LCDData2 / 10;
		i++;
	}
	if (i>0) i--;
	
	while (i>=0){
		LCDSendChar (LCDAddr, LCDData3[i]+'0');
		i--;
	}
}