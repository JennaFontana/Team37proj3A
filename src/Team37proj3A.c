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


// USART2 definitions for serial output
#define USART_TX_PIN 2  // PA2
#define USART_RX_PIN 3  // PA3
#define USART_BAUDRATE 115200


#define BTN_PIN 13 //PC13
#define BTN_PORT GPIOC
#define FREQUENCY 16000000


#define TRIG_GPIO GPIOA      // Example
#define TRIG_PIN  (1 << 4) // PA4


volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
volatile uint32_t pulse_width_us = 0;
volatile bool displayCm = true; // true for cm, false for inches
volatile bool displayInches = false;
volatile float displayValue = 0.0;
#define ECHO_GPIO GPIOB      // Using GPIOB for PB0
#define ECHO_PIN (1 << 0)    // PB0
#define ECHO_PIN_NUMBER 0    // Pin number for EXTI configuration
#define TRIG_PORT GPIOA


volatile uint32_t currentEdge = 0;
volatile uint32_t seconds = 0;
volatile uint8_t trigger_distance = 0;


void update_display(void);
void systick_init(void) {
  SysTick_Config(SystemCoreClock/500); // Exactly 0.5 second or 500 ms
  NVIC_SetPriority(SysTick_IRQn, 0);
}


void SysTick_Handler(void) {
   trigger_distance = 1; // tell main to trigger distance measurement
   TRIG_PORT->ODR |= (1 << TRIG_PIN); // Set the trigger pin high
   currentEdge = TIM5->CNT; // Get the current timer count
   while ((TIM5->CNT - currentEdge) < 10); // Wait for 10us
   TRIG_PORT->ODR &= ~(1 << TRIG_PIN); // Set the trigger pin low
  
}


void tim5_init(void) {
   RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
   TIM5->PSC = 15999; // Prescaler for 1 MHz timer clock (16 MHz / 16)
   TIM5->ARR = 0xFFFFFFFF; // Auto-reload for 50 ms at 16 MHz / 16
   TIM5->CR1 |= TIM_CR1_CEN; // Start timer
   TIM5->DIER |= TIM_DIER_UIE; // Enable update interrupt
   TIM5->SR &= ~TIM_SR_UIF; // Clear update flag
   NVIC_EnableIRQ(TIM5_IRQn);
   NVIC_SetPriority(TIM5_IRQn, 1); // Lower priority than TIM2
}
void TIM5_IRQHandler(void) {
   if (TIM5->SR & TIM_SR_UIF) { // check if update interrupt flag is set
       TIM5->SR &= ~TIM_SR_UIF; // clear the update interrupt flag
       seconds++; // Increment seconds counter
   }
}


void EXTI0_IRQHandler(void) {
   if(EXTI->PR & (1 << ECHO_PIN_NUMBER)) {  // Check pending bit for PB0
       if (ECHO_GPIO->IDR & ECHO_PIN) { // Rising edge detected
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
       EXTI->PR |= (1 << ECHO_PIN_NUMBER);  // Clear pending bit
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
void TIM2_IRQHandler(void) {
   static uint32_t update_count = 0;
  
   if (TIM2->SR & TIM_SR_UIF) { // check if update interrupt flag is set
       TIM2->SR &= ~TIM_SR_UIF; // clear the update interrupt flag
      
       update_count++;
       if(update_count >= 4) { // Every 4 * 0.5ms = 2ms
           update_count = 0;
           update_display(); // Update SSD display
       }
   }
}


void USART2_init(void) {
   // Enable USART2 clock
   RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  
   // Configure USART2 pins (PA2 and PA3)
   GPIOA->MODER &= ~(0x3 << (USART_TX_PIN * 2) | 0x3 << (USART_RX_PIN * 2));
   GPIOA->MODER |= 0x2 << (USART_TX_PIN * 2) | 0x2 << (USART_RX_PIN * 2); // Alternate function
   GPIOA->AFR[0] |= 0x7 << (USART_TX_PIN * 4) | 0x7 << (USART_RX_PIN * 4); // AF7 for USART2
  
   // Configure USART2
   USART2->BRR = SystemCoreClock / 4 / USART_BAUDRATE; // Set baud rate (APB1 clock is SystemCoreClock/4)
   USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; // Enable TX, RX, and USART
}


void send_to_usart(void) {
   char buffer[50];
   int len = snprintf(buffer, sizeof(buffer), "Distance: %.2f %s\r\n",
                     displayValue, displayCm ? "cm" : "in");
  
   for(int i = 0; i < len; i++) {
       while(!(USART2->SR & USART_SR_TXE)); // Wait for empty transmit buffer
       USART2->DR = buffer[i];
   }
}


void measure_distance(void) {
   // Calculate distance from pulse width
   float distance_cm = (pulse_width_us * 0.0343f) / 2.0f;
   float distance_inches = distance_cm / 2.54f;
  
   displayValue = displayCm ? distance_cm : distance_inches;
  
   // Cap at 99.99
   if(displayValue > 99.99f) {
       displayValue = 99.99f;
   }
}


void update_display(void) {
   int displayInt = (int)(displayValue * 100); // Convert to hundredths
  
   // Handle leading zero suppression for values < 10.00
   if(displayValue < 10.00f) {
       // First digit should be blank
       SSD_update(0, 15, 0); // 15 is the blank pattern
       // Display remaining digits
       SSD_update(1, (displayInt / 100) % 10, 1); // Digit with decimal point
       SSD_update(2, (displayInt / 10) % 10, 0);
       SSD_update(3, displayInt % 10, 0);
   } else {
       // Normal display for values >= 10.00
       SSD_update(0, (displayInt / 1000) % 10, 0);
       SSD_update(1, (displayInt / 100) % 10, 1); // Digit with decimal point
       SSD_update(2, (displayInt / 10) % 10, 0);
       SSD_update(3, displayInt % 10, 0);
   }
}


void GPIO_init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN; // Enable GPIOA, GPIOB and GPIOC clocks
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable SYSCFG for EXTI
  
  // Configure TRIG_PIN (PA4) as output
  TRIG_GPIO->MODER &= ~(0x3 << (4 * 2)); // Clear mode bits for PA4
  TRIG_GPIO->MODER |= (0x1 << (4 * 2));  // Set PA4 as general purpose output
  TRIG_GPIO->OTYPER &= ~(1 << 4);        // Set PA4 as push-pull
  TRIG_GPIO->OSPEEDR |= (0x3 << (4 * 2)); // Set PA4 as high speed
  TRIG_GPIO->PUPDR &= ~(0x3 << (4 * 2)); // No pull-up, pull-down for PA4


  // Configure ECHO_PIN (PB0) as input with EXTI
  ECHO_GPIO->MODER &= ~(0x3 << (ECHO_PIN_NUMBER * 2)); // Clear mode bits for PB0
  ECHO_GPIO->PUPDR &= ~(0x3 << (ECHO_PIN_NUMBER * 2)); // No pull-up, pull-down for PB0
  
  // Configure EXTI for ECHO_PIN (PB0)
  SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0; // Clear EXTI0 bits
  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PB; // Map PB0 to EXTI0
  EXTI->IMR |= (1 << ECHO_PIN_NUMBER); // Enable interrupt
  EXTI->RTSR |= (1 << ECHO_PIN_NUMBER); // Enable rising edge trigger
  EXTI->FTSR |= (1 << ECHO_PIN_NUMBER); // Enable falling edge trigger
  
  // Enable EXTI0 interrupt in NVIC
  NVIC_EnableIRQ(EXTI0_IRQn);
  NVIC_SetPriority(EXTI0_IRQn, 0); // Highest priority for accurate timing


  // Configure BTN_PIN (PC13) as input
  BTN_PORT->MODER &= ~(0x3 << (BTN_PIN * 2)); // Clear mode bits for PC13
  BTN_PORT->PUPDR |= (0x2 << (BTN_PIN * 2));  // Set PC13 with pull-down resistor
}
int main(void) {
   // Initialize all peripherals
   GPIO_init();
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
           send_to_usart();   // Send to serial monitor
           // Display updates happen in TIM2 interrupt
       }
   }
   return 0; // Never reached in embedded systems, but satisfies compiler
}

