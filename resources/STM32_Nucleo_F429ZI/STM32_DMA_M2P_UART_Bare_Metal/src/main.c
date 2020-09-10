/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "stm32f4xx.h"


void button_init(void);
void uart3_init(void);
void dma1_init(void);
void dma1_enable_stream(void);
void dma1_enable_interrupt(void);
void test_uart(char *test_data);
void restart_dma_transfer(void);
void red_led_init(void);
void red_led_on(void);
void red_led_off(void);

char dma_msg[]="Hurray DMA Worked!!";

void RCC_DeInit(void)
{
	/* Set HSION bit */
	RCC->CR |= (uint32_t)0x00000001;

	/* Reset CFGR register */
	RCC->CFGR = 0x00000000;

	/* Reset HSEON, CSSON, PLLON, PLLI2S and PLLSAI(STM32F42xxx/43xxx/446xx/469xx/479xx devices) bits */
	RCC->CR &= (uint32_t)0xEAF6FFFF;

	/* Reset PLLCFGR register */
	RCC->PLLCFGR = 0x24003010;

	/* Reset PLLI2SCFGR register */
	RCC->PLLI2SCFGR = 0x20003000;
	/* STM32F40_41xxx || STM32F427_437xx || STM32F429_439xx || STM32F401xx || STM32F411xE || STM32F446xx || STM32F413_423xx || STM32F469_479xx */

	/* Reset PLLSAICFGR register, only available for STM32F42xxx/43xxx/446xx/469xx/479xx devices */
	RCC->PLLSAICFGR = 0x24003000;
	/* STM32F40_41xxx || STM32F427_437xx || STM32F429_439xx || STM32F446xx || STM32F469_479xx */

	/* Reset HSEBYP bit */
	RCC->CR &= (uint32_t)0xFFFBFFFF;

	/* Disable all interrupts */
	RCC->CIR = 0x00000000;

	/* Disable Timers clock prescalers selection, only available for STM32F42/43xxx and STM32F413_423xx devices */
	RCC->DCKCFGR = 0x00000000;

}

int main(void)
{
	/** Do we need clock init**/

	/** Not required - default clock - internal RC oscillator**/
	/** 16 Mhz default **/
	/** No need to initialize PLL engine to increase the clock **/
	/** no configuration is required, micro-controller will run it
	 * by default
	 */
	RCC_DeInit(); // Important otherwise I need to calculate the baud rate for UART correctly.
	// By de-initializing the  RCC, we have made it to run at 16 Mhz.
	button_init();
	uart3_init();
	red_led_init();
	dma1_init();
	dma1_enable_interrupt();
	dma1_enable_stream();

	while(1)
	{
		//forever loops
	}

	return 0;

}

void button_init()
{
	/** button is connected to PC13 **/
	/** User blue button **/
	// These steps are generic one in order to configure the gpio
	//1. Enable the clock for the GPIO C peripheral
	//2. keep the GPIO pin in input mode
	//3. Keep the GPIO pin in edge detection
	//4. Enable the interrupt over that GPIO pin
	//5. Enable the IRQ related to that gpio pin in NVIC of the processor

	//Start coding
	//1. - Enable the clock
	// reference manual
	//GPIOC - is connected to AHB1
	// RCC - Enable bit 2 - (RCC_AHB1ENR) // page 180
	RCC->AHB1ENR |= ( 1u << 2);
	//2. keep the GPIO pin in input mode
	// Page 281 - pin - 13 , 26 and 27 bit to zero.
	GPIOC->MODER &= ~(0x3 << 26);
	/** Configure SYSCFG since it is EXTI line.
//Page 66 - SYSCFG is connected to APB2 bus.
 Enable clock to APB2 now.
	 */
	//Page 248
	RCC->APB2ENR |= (0x1 <<14);
	/*Page 299 **/
	//SYSCFG_EXTICR4 bit 4,5,6,7.0010 for PC
	SYSCFG->EXTICR[3] &= ~(0xF << 4); //clearing
	SYSCFG->EXTICR[3] |= (0x2 << 4); //setting
	//3 Keep the GPIO pin in edge detection
	//  External Interrupt registers.
	// Page 385 -
	// PC 13 i.e. 13th pin.
	EXTI->RTSR &= ~(0x1 << 13); // disabling -- the rising edge
	EXTI->FTSR |= (0x1 << 13); // enabling - the falling edge
	//Interrupt mask register - unmask your register
	EXTI->IMR |= (0x1 <<13);

	//Page 371 - vector table -- 40 - IRQ number.
	//EXTI15_10_IRQn is actually 40.
	//Enables the interrupt.s
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}

//startup_stm32.s -- weak ISR
// now creating the custom IRQ handler
void EXTI15_10_IRQHandler()
{
	//test uart -- will be disabled since we are using DMA.
	//test_uart("testing..uart\r\n");
	//UART request DMA
	//Page 1017 - USART_CR3 - DMA- xsfer enable
	//bit 7
	USART3->CR3 |= (0x1 << 7);
	//once the dma transfer is done
	// the NDTR will become zero and
	//DMA will automatically disable the stream
	// therefore on pressing again this button
	// you won't see anything on the terminal
	//clear the pending bit.
	//page 386.//EXTI_PR - pending register.//clear by setting it to 1.
	if(EXTI->PR & (0x1 << 13))
	{
		EXTI->PR |= (0x1 <<13);
	}

}

void uart3_init()
{
	// 1. Enable the peripheral clock for uart3 peripheral.
	// 2. Configure the gpio pins for UART TX and UART Rx functionality
	// 3. COnfigure the baudrate
	// 4. Configure the datawidth, number of stop bits, parity
	// 5. Enable the uart peripheral
	// 6. Enable the tx engine of the uart.

	//USART3 is hanging on APB1 bus //Page 245
	RCC->APB1ENR |= (0x1 << 18);
	//USART3 is used like a Virtual com port support.
	//AF table check -
	//Look into data sheet rather than reference manual
	// Page . 74 Table 12 -- Alternat function table.
	//PD8 - AF7 --USART3 Tx
	//PD9 - AF7 --USART3 Rx
	//Enable clock for GPIO D peripheral.
	//change the mode of PD8 and PD9 to alternate function
	//Enable or disable pull up resistor if required.

	//GPIO D is connected to AHB1
	//Page 242 - ABH1 ENR
	RCC->AHB1ENR |= (0x1 << 3);
	//PD 8 to AF Page 281.
	//AF mode .
	GPIOD->MODER &= ~(0x3 << 16); //clear the bits
	GPIOD->MODER |= (0x2 << 16); //AF is 10 binary i.e 0x2

	//GPIO - AF -- mode PD8 - AF7.
	//Page 286. //AF7 i.e 0111
	GPIOD->AFR[1] &= ~(0xF << 0);
	GPIOD->AFR[1] |= (0x7 << 0);
	///very high speed 0x11
	// GPIOD->OSPEEDR |= (0x3 <<16);
	// GPIOD->OSPEEDR |= (0x3 <<18);

	GPIOD->PUPDR &= ~(0x3 << 16); //clear the bits
	//Page 283.. Pull up 01 i.e 1.
	GPIOD->PUPDR |= (0x1 << 16); // for serial communication we should have Pull Up

	//Similarly -- PD9
	GPIOD->MODER &= ~(0x3 << 18); //clear the bits
	GPIOD->MODER |= (0x2 << 18); //AF is 10 binary i.e 0x2

	GPIOD->AFR[1] &= ~(0xF << 4);
	GPIOD->AFR[1] |= (0x7 << 4);

	GPIOD->PUPDR &= ~(0x3 << 18); //clear the bits
	//Page 283.. Pull up 01 i.e 1.
	GPIOD->PUPDR |= (0x1 << 18);  // for serial communication we should have Pull Up

	//USART 3 -- Baud rate .
	//by default the micro-controller clock is 16Mhz.
	//Table 136. 115.2Kbps - 8.6875
	// 8 in hex is 0x8
	// 0.6875 is 0.6875 *16 = 11 i.e. B
	USART3->BRR = 0x8B; //very important, you will see some garbage on the screen if the UART baud rate is
	//not set properly
	//if USART settings are not done properly then you will see garbage on the terminal/putty.
	// by default - 1 start bit, 8 databits and n stop bits
	// USART CR1 page . 1014
	//bit 12
	// by default parity is even. i.e. bit 9.
	// bit 3 xmit enable
	USART3->CR1 |= (0x1 << 3);
	//USART engine is on now.
	USART3->CR1 |= (0x1 << 13);
}

void dma1_init()
{
	//1. Enable the peripheral clock for dma1
	//2. Identify the stream which is suitable for your peripheral
	//3. Identify the channel number on which uart 3 peripheral sends DMA request
	//4. Program src address
	//5. Program des address
	//6. Number of data items to send.
	//7. direction of transfer i.e. transfer mode
	//8. Program the source and destination data width,byte by byte
	//9. Select direct mode or fifo mode
	//10. select the fifo threshold
	//11.  Circular mode if required
	//12. Single transfer or burst transfer
	//13. Stream Priority
	//14. Enable the stream

	//1.Page 180 bit 21 //DMA1 connected to AHB1
	RCC->AHB1ENR |= (0x1 <<21);
	//Page 307 - DMA request mapping - under DMA section
	//Channel 4 USART3_TX Stream 3
	//DMA stream pointer
	//Page 328 -- select the channel i.e. 4.
	//bit 27:25 -- 100
	//Take DMA Stream 3 register
	DMA1_Stream3->CR &= ~(0x7 << 25);
	DMA1_Stream3->CR |= (0x4 <<25);  // channel 4
	//Memory increment - bit 10
	DMA1_Stream3->CR |= (0x1 <<10);
	//Peripheral increment - bit 9 should be 0
	DMA1_Stream3->CR &= ~(0x1 <<9);

	//Program the src address i.e. memory address - MOAR
	DMA1_Stream3->M0AR = (uint32_t)&dma_msg;
	//Program the  dest address i.e. uart3 --PAR - peripheral address
	DMA1_Stream3->PAR = (uint32_t)&(USART3->DR);
	//number of bytes to transfer.
	DMA1_Stream3->NDTR = strlen(dma_msg);
	//direction
	//direction Bit 7:6 -- 01 - memory to peripheral
	// on reset this register is set to 0. i.e CR
	// i.e on power cycle the content of this register is 0x000
	// check the reference sheet - about this register
	DMA1_Stream3->CR &= ~(0x3 <<6);
	DMA1_Stream3->CR |= (0x1 <<6);
	//transfer byte by byte - page 329 bits 14:13 - memory size
	DMA1_Stream3->CR &=  ~(0x3 << 13);   //00 is byte by byte
	//peripheral size - 00 i.e. byte by byte
	//bits 12:11
	DMA1_Stream3->CR &= ~(0x3 << 11);
	//FIFO mode - Page 334 i.e. FIFO register.
	//bit 2 direct mode is disabled - set to 1.
	DMA1_Stream3->FCR |= (0x1 <<2);
	//FIFO threshold. i.e. FULL
	//bits 1:0
	//11 is full fifo
	DMA1_Stream3->FCR |= (0x3 <<0);
	//disable circular mode
	//Page 330 Bit 8 - circular mode disable i.e. 0
	DMA1_Stream3->CR &= ~(0x1 << 8);
	//burst disabled - bits 24:23 -- 00 - single transfer
	DMA1_Stream3->CR &= ~(0x3 <<23);
	//stream priority
	// bits 17:16 - priority level
	// 11 very high
	DMA1_Stream3->CR |= (0x3 <<16);

}

void test_uart(char *test_data)
{
	// char test_data[] = "Shreyas..very nice..\r\n";
	uint32_t len = strlen(test_data);

	for(uint32_t index = 0; index < len; index++)
	{
		//Page 1011, until and unless TXE bit 1 means it is ready to send 7th bit.
		while(!(USART3->SR & (0x1 <<7)) );
		USART3->DR = test_data[index];
	}
}

void dma1_enable_stream()
{
	//Stream enabled bit 0
	DMA1_Stream3->CR |= (0x1 <<0);
}

void dma1_enable_interrupt()
{
	//enable dma1 interrupts.
	//HTIE - half transfer interrupt enable
	//TCIE - transfer complete
	//TEIE - transfer error
	//FIFO - overrun/underrun -FEIE
	//DMEIE - direct mode error
	//Page 328 --bit 1 -- DMEIE, bit 2- TEIE, bit 3 -HTIE, bit 4 - TCIE.
	DMA1_Stream3->CR |= (0x1 << 1);
	DMA1_Stream3->CR |= (0x1 << 2);
	DMA1_Stream3->CR |= (0x1 << 3);
	DMA1_Stream3->CR |= (0x1 << 4);
	NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}

//DMA1 stream3 IRQ
void DMA1_Stream3_IRQHandler()
{
	//Page 325 -- Stream 3 so low interrupt status register.
	//Page 327 -- Stream 3 clear pending interrupts.

	if ( DMA1->LISR & (0x1 <<22) )
	{
		//Direct Mode error
		//set the bit
		test_uart("FIFO error\r\n");
		DMA1->LIFCR |= (0x1 << 22);
	}
	else if ( DMA1->LISR & (0x1 <<24) )
	{
		//Direct Mode error
		//set the bit
		test_uart("Direct Mode error\r\n");
		DMA1->LIFCR |= (0x1 << 24);
	}
	else if(DMA1->LISR & (0x1 <<25))
	{
		//transfer error.
		test_uart("transfer error\r\n");
		DMA1->LIFCR |= (0x1 << 25);
	}
	else if(DMA1->LISR & (0x1 <<26))
	{
		//half transfer complete
		//if you write something on uart while the DMA is going on, it will corrupt UART.
		//test_uart("half transfer complete\r\n");
		DMA1->LIFCR |= (0x1 << 26);
	}
	else if(DMA1->LISR & (0x1 <<27))
	{
		//transfer complete
		test_uart("transfer complete\r\n");
		DMA1->LIFCR |= (0x1 << 27);
		restart_dma_transfer();
	}
	else
	{
		//never come here.
		test_uart("None!! can't be ..really..\r\n");
	}
}


void restart_dma_transfer()
{
	static int count = 0;
	if(count % 2 == 0)
		red_led_on();
	else
		red_led_off();
	count++;
	test_uart("Restart DMA..again.\r\n");
	//disable uart3 interrupt
	USART3->CR3 &= ~(0x1 << 7);
	//re-initialize NDTR register after DMA complete this has become zero.
	DMA1_Stream3->NDTR = strlen(dma_msg);
	//re-enable the stream after DMA complete the stream gets disabled
	dma1_enable_stream();
}

void red_led_init()
{
  //PB-14
  // turn the clock for GPIOB.
  RCC->AHB1ENR |= (0x1 <<1);
  //01 - output mode. bit 28,29
  GPIOB->MODER &= ~(0x3 <<28);
  GPIOB->MODER |= (0x1 <<28);

}
void red_led_on(void)
{
	// turn on the 14th pin.
	GPIOB->ODR |= (0x1 << 14);
}
void red_led_off(void)
{
	// turn off the 14th pin.
	GPIOB->ODR &= ~(0x1 << 14);
}
