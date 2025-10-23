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
void send_trigger_pulse(void);

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

//Part B
volatile uint32_t pulse_width = 0;
#define SERVO3_PIN 6 // PC6
#define SERV0_PORT GPIOA
int angle =0;

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

void systick_init(void) {
  SysTick_Config(FREQUENCY/2); // Exactly 0.5 second or 500 ms
  NVIC_SetPriority(SysTick_IRQn, 0);
}

void SysTick_Handler(void) {
    send_trigger_pulse();  // trigger the sensor
}

void tim5_init(void) {
   RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
   TIM5->PSC = 15; // Prescaler for 1 MHz timer clock (16 MHz / 16)
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
       if (ECHO_PORT->IDR & (1 << ECHO_PIN)) { // Rising edge detected
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

void trig_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable GPIOA clock
    TRIG_PORT->MODER &= ~(3 << (TRIG_PIN * 2)); // Clear mode bits
    TRIG_PORT->MODER |= (1 << (TRIG_PIN * 2));  // Set as output
}
void send_trigger_pulse(void) {
    TRIG_PORT->ODR |= (1 << TRIG_PIN);     // Set TRIG high
    for (volatile int i = 0; i < 160; i++); // Delay ~10 µs (assuming 16 MHz CPU)
    TRIG_PORT->ODR &= ~(1 << TRIG_PIN);    // Set TRIG low
}

void configure_button_interrupt(void) {
   RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable system configuration controller clock
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // Enable GPIOC clock
   EXTI->IMR |= (1 << BTN_PIN); // Unmask EXTI13
   EXTI->FTSR |= (1 << BTN_PIN); // Trigger on falling edge
   BTN_PORT->MODER &= ~(3 << (BTN_PIN * 2)); // Set BTN_PIN as input by clearing MODER bits
   SYSCFG->EXTICR[3] |= (2<< (1*4));
   NVIC_EnableIRQ(EXTI15_10_IRQn); // Enable EXTI line[15:10] interrupts in NVIC
   NVIC_SetPriority(EXTI15_10_IRQn, 0); // Set priority (lower than TIM2 and TIM5)
}

void configure_echo_interrupt(void) {
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable GPIOB clock
   RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable system configuration controller clock
   EXTI->IMR |= (1 << ECHO_PIN); // Unmask EXTI0
   EXTI->RTSR |= (1 << ECHO_PIN); // Trigger on rising edge
   EXTI->FTSR |= (1 << ECHO_PIN); // Trigger on falling edge
   ECHO_PORT->MODER &= ~(3 << (ECHO_PIN * 2)); // Set ECHO_PIN as input by clearing MODER bits
   SYSCFG->EXTICR[0] &= ~(0xF << (0 * 4)); // Clear EXTI0 bits
   SYSCFG->EXTICR[0] |= (1 << (0 * 4));    // Map EXTI0 to PB0
   NVIC_EnableIRQ(EXTI0_IRQn); // Enable EXTI0 interrupt in NVIC
   NVIC_SetPriority(EXTI0_IRQn, 1); // Set priority (lower than TIM2)
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
//Part B
void tim8_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN; // Enable TIM8 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable GPIOA clock

    GPIOC->MODER &= ~(0x3 << (SERVO3_PIN * 2));
	GPIOC->MODER |=  (0x2 << (SERVO3_PIN * 2)); // Alternate function
	GPIOC->AFR[0] &= ~(0xF << (SERVO3_PIN * 4));
	GPIOC->AFR[0] |=  (0x2 << (SERVO3_PIN * 4));

    TIM8->PSC = (FREQUENCY/1000000)-1; // Prescaler for 1 us
    TIM8->ARR = 19999; // Auto-reload for 20 ms period
    TIM8->CCR1 = 1500; // how long the signal stays on (1.5 ms) 

    TIM8->CCMR1 &= ~(TIM_CCMR1_OC1M); //clears the bits
    TIM8->CCMR1 |= (6 << 4); // PWM mode 1 on channel 1 !!!!!!!!!!!!!
    TIM8->CCMR1 |= TIM_CCMR1_OC1PE; // Enable preload for CCR1
    TIM8->CCER |= TIM_CCER_CC1E; // Enables channel 1

    TIM8->CR1 |= TIM_CR1_CEN; // Start timer
    TIM8->CR1 |= TIM_CR1_ARPE; // Enable auto-reload preload
    TIM8->EGR |= TIM_EGR_UG; // Generate an update event to load the registers (so PWM doesn't glitch in the middle of a pulse)
    TIM8->BDTR |= TIM_BDTR_MOE; // Main output enable (necessary for advanced-control timers like TIM8)
}


void set_servo_angle(int32_t angle) {
    pulse_width = 1500- 1000 *(angle / 90.0); // Map angle to pulse width (1 ms to 2 ms)
    TIM8->CCR1 = pulse_width; // Update CCR1 with new pulse width
}



int main(void) {
   // Initialize all peripherals
   USART2_init();
   systick_init();
   tim2_init();   
   tim5_init();
   configure_button_interrupt();
   SSD_init();
   trig_init();
   configure_echo_interrupt();

    tim8_init(); // Part B: Initialize TIM3 for servo control
    set_servo_angle(angle); // Set initial servo angle to 0 degrees
   while(1) {
    	for (angle = -45; angle <= 45; angle += 5) { 
            set_servo_angle(angle); 
            for (volatile int i = 0; i < 1000000; i++); // Delay (wait to get to angle)
            send_trigger_pulse(); 
            for (volatile int i = 0; i < 30000; i++); // Delay 
            } 
            for (angle = 45; angle >= -45; angle -= 5) 
            { set_servo_angle(angle); 
            for (volatile int i = 0; i < 1000000; i++); // Delay 
            send_trigger_pulse(); 
            for (volatile int i = 0; i < 30000; i++); // Delay } 
            }
        }
    }