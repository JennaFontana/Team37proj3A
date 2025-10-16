/***********************************************
* TEAM 37: J. PARK and J. FONTANA
* CPEG222 Project1B, 10/3/25
* NucleoF446RE CMSIS Sequence PMOD SSDs
***********************************************/
#include "stm32f4xx.h"
#include "SSD_Array.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

/* Function Prototypes */
int main(void) __attribute__((used));

// USART2 definitions for serial output
#define USART_TX_PIN 2  // PA2
#define USART_RX_PIN 3  // PA3
#define USART_BAUDRATE 115200

#define BTN_PIN 13 //PC13
#define BTN_PORT GPIOC
#define FREQUENCY 16000000

#define TRIG_PORT GPIOA
#define TRIG_PIN 4 // PA4


volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
volatile uint32_t pulse_width_us = 0;

volatile bool displayCm = true; // true for cm, false for inches
volatile float displayValue = 0.0;
volatile int DigitSelect = 0; // Current digit to update on SSD
volatile float distance = 0.0; 

#define ECHO_PORT GPIOB      // Using GPIOB for PB0
#define ECHO_PIN 0    // PB0


volatile uint32_t currentEdge = 0;
volatile uint32_t seconds = 0;
volatile uint8_t trigger_distance = 0;

float measure_distance(void) {
    if (displayCm){
        distance = pulse_width_us / 58.3; 
    } else {
        distance = pulse_width_us / 148.0;
    }
    if (distance > 99.99) {
        distance = 99.99; // Cap at 99.99
    }
    return distance;
}

void update_display(void);

void systick_init(void) {
  SysTick_Config(SystemCoreClock/500); // Exactly 0.5 second or 500 ms
  NVIC_SetPriority(SysTick_IRQn, 0);
}


void SysTick_Handler(void) {
   trigger_distance = 1; // tell main to trigger distance measurement
   TIM5->CNT = TIM_CR1_CEN; // Start timer
   TRIG_PORT->ODR |= (1 << TRIG_PIN); // Set the trigger pin high
}


void tim5_init(void) {
   RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
   TIM5->PSC = 15999; // Prescaler for 1 MHz timer clock (16 MHz / 16)
   TIM5->ARR = 0xFFFFFFFF; // Auto-reload for 50 ms at 16 MHz / 16
   TIM5->CR1 |= TIM_CR1_CEN; // Start timer
    TIM5->DIER |= TIM_DIER_UIE; // Enable update interrupt

   NVIC_EnableIRQ(TIM5_IRQn);
   NVIC_SetPriority(TIM5_IRQn, 1); // Lower priority than TIM2
}
void TIM5_IRQHandler(void) {
   if (TIM5->SR & TIM_SR_UIF) { // check if update interrupt flag is set
    TIM5->CR1 &= ~TIM_CR1_CEN; // Stop timer
    TRIG_PORT->ODR &= ~(1 << TRIG_PIN); // Ensure trigger pin is low
       TIM5->SR &= ~TIM_SR_UIF; // clear the update interrupt flag
   }
}


void EXTI0_IRQHandler(void) {
   if(EXTI->PR & (1 << 0)) {  // Check pending bit for PB0
       if (ECHO_PORT->IDR & ECHO_PIN) { // Rising edge detected
           start_time = TIM5->CNT; // Capture start time
       } else { // Falling edge detected
           end_time = TIM5->CNT; // Capture end time
           if (end_time >= start_time) {
               pulse_width_us = end_time - start_time; // Calculate pulse width in microseconds
           } else {
               // Handle timer overflow
               pulse_width_us = (0xFFFFFFFF - start_time) + end_time;
           }
           pulse_width_us /= 16; // Convert timer ticks to microseconds (1 tick = 1us)
           trigger_distance = 1; // Signal main loop to process distance


       }
       EXTI->PR |= (1 << ECHO_PIN);  // Clear pending bit
   }
}


void EXTI15_10_IRQHandler(void) {
   if(EXTI->PR & (1 << BTN_PIN)) {  // Check pending bit for PC13
       // Button press detected
       displayCm = !displayCm; // Toggle display mode
       EXTI->PR |= (1 << BTN_PIN);  // Clear pending bit
   }
}


void configure_button_interrupt(void) {
   RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable system configuration controller clock
   SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC; // Map EXTI13 to PC13
   EXTI->IMR |= EXTI_IMR_MR13; // Unmask EXTI13
   EXTI->FTSR |= EXTI_FTSR_TR13; // Trigger on falling edge
   NVIC_EnableIRQ(EXTI15_10_IRQn); // Enable EXTI line[15:10] interrupts in NVIC
   NVIC_SetPriority(EXTI15_10_IRQn, 2); // Set priority (lower than TIM2 and TIM5)
}


void tim2_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 15;
    TIM2->ARR = 499;
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->SR &= ~TIM_SR_UIF;
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 1);
    TIM2->CR1 |= TIM_CR1_CEN; // Start timer (only once)
}

void TIM2_IRQHandler(void){
   if (TIM2->SR & TIM_SR_UIF) { 
        SSD_update(DigitSelect, (int)(measure_distance() * 100) , 2);
        DigitSelect = (DigitSelect + 1) % 4; // Cycle through 0-3
       TIM2->SR &= ~TIM_SR_UIF; // clear the update interrupt flag
       }
   }


void USART2_init(void) {
   // Enable USART2 clock
   RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable GPIOA clock
  
   // Configure USART2 pins (PA2 and PA3)
   GPIOA->MODER &= ~(0x3 << (USART_TX_PIN * 2) | 0x3 << (USART_RX_PIN * 2));
   GPIOA->MODER |= 0x2 << (USART_TX_PIN * 2) | 0x2 << (USART_RX_PIN * 2); // Alternate function
   GPIOA->AFR[0] |= 0x7 << (USART_TX_PIN * 4) | 0x7 << (USART_RX_PIN * 4); // AF7 for USART2
  
   // Configure USART2
   USART2->BRR = FREQUENCY / USART_BAUDRATE; // Set baud rate (APB1 clock is SystemCoreClock/4)
   USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; // Enable TX, RX, and USART
}

void update_display(void) {
   int displayInt = (int)(displayValue * 100); // Convert to hundredths
  
   // Handle leading zero suppression for values < 10.00
   if(displayValue < 10.00f) {
       // First digit should be blank
       SSD_update(0, 15, 0); // 15 is the blank pattern
       // Display remaining digits
       SSD_update(1, (displayInt / 100) % 10, 1); // First displayed digit with decimal point
       SSD_update(2, (displayInt / 10) % 10, 0);
       SSD_update(3, displayInt % 10, 0);
   } else {
       // Normal display for values >= 10.00
       SSD_update(0, (displayInt / 1000) % 10, 0);
       SSD_update(1, (displayInt / 100) % 10, 1); // Second digit with decimal point
       SSD_update(2, (displayInt / 10) % 10, 0);
       SSD_update(3, displayInt % 10, 0);
   }
   
}

int main(void) {
   // Initialize all peripherals
   USART2_init();
   systick_init();
   tim2_init();   
   tim5_init();
   configure_button_interrupt();
   SSD_init();
  
   // Initial display setup
   update_display();



   while(1) {
       if (trigger_distance) {
           trigger_distance = 0;
           measure_distance(); // Calculate distance from pulse width
           // Display updates happen in TIM2 interrupt
       }
   }
   return 0; // Never reached in embedded systems, but satisfies compiler
}