#include "NST-28037B.h"

#define COMMAND_SET		0x14	// CTRL-T

#define COMMAND_MODE	0x01
uint8_t StatusFlags;

#define UART_RX_SIZE   2048
#define UART_TX_SIZE   16
char RxBuffer[UART_RX_SIZE];
__IO int  RxGet, RxPut;

char TxBuffer[UART_TX_SIZE];
__IO int  TxGet, TxPut;

__IO uint16_t	TimingDelay;
__IO uint16_t	CursorTimer;
__IO uint8_t	AdcStartDelay;

/**************************************************************
 *	SysTick interrupt handler
 *
 */
void SysTick_Handler(void)
{
	if (AdcStartDelay != 0)
	{
		--AdcStartDelay;
		if (AdcStartDelay == 0)
			ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	}

	if (TimingDelay != 0)
		--TimingDelay;
	if (CursorTimer != 0)
		--CursorTimer;
}

/**************************************************************
 *	Delay in milliseconds with interrupts
 */
void Delay_ms(__IO uint16_t ms)
{
	TimingDelay = ms;
	while (TimingDelay != 0) { }
}

/**************************************************************
 *	USART1 interrupt handler
 */
void USART1_IRQHandler()
{
	int next;
	char data;

	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		data = USART_ReceiveData(USART1);
		if (USART_GetFlagStatus(USART1, (USART_FLAG_FE | USART_FLAG_NE | USART_FLAG_ORE)) != RESET)
		{
			data = USART_ReceiveData(USART1);
		}
		else
		{
			next = RxPut + 1;
			if(next == UART_RX_SIZE)
				next = 0;
			if (next != RxGet)
			{
				RxBuffer[RxPut] = data;
				RxPut = next;
			}
			else
			{
				// No space for data, ignore it
			}
		}
	}

	if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
	{
		if (TxGet != TxPut)
		{
			USART_SendData(USART1, TxBuffer[TxGet]);
			TxGet++;
			if (TxGet == UART_TX_SIZE)
				TxGet = 0;
		}
		if (TxGet == TxPut)
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
	}
}

void BSOD(const char *text)
{
	__IO uint32_t delay;

	SetForeground(WHITE);
	SetBackground(BRIGHTBLUE);
	ClearScreen();
	SetFontScale(2);
	DisplayTextAt(0, 0, "Fatal Error:");
	DisplayTextAt(0, 2, text);
	DisplayTextAt(0, 3, "Reboot...");

	for(delay = 8000000; delay != 0; --delay)
	{
		__NOP();	__NOP();
		__NOP();	__NOP();
		__NOP();	__NOP();
		__NOP();	__NOP();
	}
	NVIC_SystemReset();
}

void NMI_Handler()			{	BSOD("NMI");			}
void HardFault_Handler()	{	BSOD("Hard Fault");		}
void MemManage_Handler()	{	BSOD("Mem Manage");		}
void BusFault_Handler()		{	BSOD("Bus Fault");		}
void UsageFault_Handler()	{	BSOD("Usage Fault");	}
void SVC_Handler()			{	BSOD("SVC");			}
void DebugMon_Handler()		{	BSOD("Debug Mon");		}
void PendSV_Handler()		{	BSOD("PendSV");			}

void SetupHardware()
{
	NVIC_InitTypeDef NVIC_InitStructure;
	ADC_InitTypeDef  ADC_InitStructure;

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA	|
							RCC_APB2Periph_GPIOB	|
							RCC_APB2Periph_ADC1		|
							RCC_APB2Periph_AFIO,
							ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	//	--------------
	//	Initialize ADC
	//
#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
	/* ADCCLK = PCLK2/2 */
	RCC_ADCCLKConfig(RCC_PCLK2_Div2);
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_IRQn;
#else
	/* ADCCLK = PCLK2/4 */
	RCC_ADCCLKConfig(RCC_PCLK2_Div4); 
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
#endif

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	ADC_DeInit(ADC1);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Cmd(ADC1, ENABLE);

	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));

	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//	Initialize SysTick
	TimingDelay = 0;
	if (SysTick_Config(SystemCoreClock / 1000))
		BSOD("SysTick not init");
}

int UsartAvailable()
{
	return (RxGet == RxPut ? 0 : 1);
}

void UsartFlush()
{
	__disable_irq();
	TxGet = TxPut = RxGet = RxPut = 0;
	__enable_irq();
}

char UsartRead()
{
	char c;
	if (RxGet == RxPut)
		return 0;
	c = RxBuffer[RxGet];
	__disable_irq();
	RxGet++;
	if (RxGet == UART_RX_SIZE)
		RxGet = 0;
	__enable_irq();
	return c;
}

void UsartWrite(const char * src)
{
	int next;
	char c;
	while ((c = *src++) != 0)
	{
		next = TxPut + 1;
		if (next == UART_TX_SIZE)
			next = 0;
		while (next == TxGet) { }
		TxBuffer[TxPut] = c;
		TxPut = next;
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	}
}

void UsartInit(uint32_t speed)
{
	USART_InitTypeDef USART_InitStructure;

	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
	UsartFlush();

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	USART_DeInit(USART1);

	USART_InitStructure.USART_BaudRate = speed;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	pinMode(TXD, PinMode_AF_PP);
	pinMode(RXD, PinMode_In_PU);

	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

int main(void)
{
	char c;

	StatusFlags = 0;

	SetupHardware();
	DisplayInit();

	DisplayTextLine("Speed: 115200");
	UsartInit(115200);

	while(1)
	{
		c = UsartRead();
		if (c != 0)
		{
			if (StatusFlags & COMMAND_MODE)
			{
				if (c == 'X')
					NVIC_SystemReset();
				if (c == 'E')
					ClearScreen();
				StatusFlags &= ~COMMAND_MODE;
			}
			else if (c == COMMAND_SET)
				StatusFlags |= COMMAND_MODE;
			else
				DisplayChar(c);
		}
		DisplayIdleApps();
	}
}
