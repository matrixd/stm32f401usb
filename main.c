/*
 * main.c
 *
 *  Created on: Aug 25, 2014
 *      Author: matrixd
 */
#include "usbd_core.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "stm32f4xx.h"
extern USB_OTG_CORE_HANDLE           USB_OTG_dev;
static uint8_t *USBD_HID_GetPos (void);
extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

volatile uint16_t delay_time = 0;
typedef struct {
    uint8_t read;
    uint8_t write;
    uint8_t buf[1024];
} BUF;
BUF to_usb;
BUF to_spi;

void ledInit(void){
    RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIODEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR12 | GPIO_OSPEEDER_OSPEEDR14);
    GPIOD->MODER &= ~(GPIO_MODER_MODER12 | GPIO_OSPEEDER_OSPEEDR14);
    GPIOD->MODER |= GPIO_MODER_MODER12_0 | GPIO_OSPEEDER_OSPEEDR14_0;
}

void usartInit(void){
    RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOBEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR6 | GPIO_MODER_MODER7);
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6 | GPIO_MODER_MODER7;
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7);
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0;
    GPIOB->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOB->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
    GPIOB->AFR[0] &= ~0x77000000;
    GPIOB->AFR[0] |= 0x77000000;
    USART1->CR1 = USART_CR1_UE;
    USART1->BRR = 0x2d9; // 45.5625
    USART1->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE;
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_SetPriority(USART1_IRQn, 2);
}
void spiInit(void){
    RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR &= RCC_APB2ENR_SPI1EN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR5
            | GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7);
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR5
            | GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7;
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4 | GPIO_PUPDR_PUPDR5
            | GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7);
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_0 | GPIO_PUPDR_PUPDR5_0
            | GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0;
    GPIOA->MODER &= ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5
                     | GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOA->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1
                         | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
    GPIOA->AFR[0] &= ~0x55550000;
    GPIOA->AFR[0] |= 0x55550000;
    SPI1->CR1 = SPI_CR1_BR;
    SPI1-> CR2 = 0x10 | SPI_CR2_RXNEIE;
    SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_SPE;
    NVIC_EnableIRQ(SPI1_IRQn);
    NVIC_SetPriority(SPI1_IRQn, 5);
}

void delay(uint16_t time){
    delay_time = time;
    while(delay_time != 0);
}
void blink(void){
    GPIOD->ODR &= ~GPIO_ODR_ODR_12;
    GPIOD->ODR |= GPIO_ODR_ODR_12;
    delay(100);
    GPIOD->ODR &= ~GPIO_ODR_ODR_12;
    delay(100);
}

int main(void){
    ledInit();
    to_usb.write = 0;
    to_usb.read = 0;
    to_spi.read = 0;
    to_spi.write = 0;
    SysTick_Config(SystemCoreClock / 1000);
    USBD_Init(&USB_OTG_dev,
                USB_OTG_FS_CORE_ID,
                &USR_desc,
                &USBD_CDC_cb,
                &USR_cb);
    //spiInit();
    uint8_t buf[] = "hell0hell0hell0hell0hell0hell0";
    uint8_t co = 0;
    while(1) {

        /*USBD_HID_SendReport (&USB_OTG_dev,
                                      buf,
                                        4);*/
        DCD_EP_Tx (&USB_OTG_dev, HID_IN_EP, buf, 30);
        //blink();
        /*//to_spi.write = 1;
        //to_spi.buf[0] = 0x07;
        to_usb.write = 4;
        to_usb.buf[0] = 0x07;
        to_usb.buf[1] = 0x07;
        to_usb.buf[2] = 0x07;
        to_usb.buf[3] = 0x07;
        if(to_spi.write > 0){
            while(to_spi.write > to_spi.read){
                            SPI1->DR = to_spi.buf[to_spi.read]-0x30;
                            to_spi.read++;
                            while(!(SPI1->SR & SPI_SR_TXE));
                        }
                        to_spi.write = 0;
                        to_spi.read = 0;
        }
        if(to_usb.write > 0){
            USBD_HID_SendReport (&USB_OTG_dev,
                                       to_usb.buf,
                                       to_usb.write);
            blink();
            to_usb.write = 0;
        }*/
    }
}

/*void OTG_FS_WKUP_IRQHandler(void)
{
  if(USB_OTG_dev.cfg.low_power)
  {
    // Reset SLEEPDEEP and SLEEPONEXIT bits
    SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));

    //After wake-up from sleep mode, reconfigure the system clock
    SystemInit();
    USB_OTG_UngateClock(&USB_OTG_dev);
  }
  EXTI_ClearITPendingBit(EXTI_Line18);
}*/
void OTG_FS_IRQHandler(void)
{
  USBD_OTG_ISR_Handler (&USB_OTG_dev);
}

/*void OTG_HS_EP1_IN_IRQHandler(void){
    USBD_HID_SendReport (&USB_OTG_dev,
                                          buf,
                                            4);
    blink();
}
void OTG_HS_EP1_OUT_IRQHandler(void){
    blink();blink();
}*/

void SysTick_Handler(void){
    if(delay_time != 0){
        delay_time--;
    }
}

void SPI1_IRQHandler(void){
    if(SPI1->SR & SPI_SR_RXNE){
        to_usb.buf[to_usb.write] = SPI1->DR;
        to_usb.write++;
    }
}
