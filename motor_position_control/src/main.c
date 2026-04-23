/* Importo los módulos necesarios para controlar los perifericos de la stm y realizar operaciones con entradas y salidas */
#include "stm32f051x8.h"
#include <stdint.h>
#include <math.h>

/* Definición de cttes */
#define PPR         500
#define MAX_RPM     300.0f
#define DT          0.01f
#define MAX_TURNS   3
#define MAX_PULSES  (MAX_TURNS * PPR)
#define MAX_VEL 300

/* Definición de variables */
volatile uint16_t ADC_value = 0; //handler ADC
volatile uint8_t  flag = 0; //handler interrupción

int32_t prev_count = 0; // handler conteo previo encoder

// Posicion
float   pos_d      = 0.0f;
float   vel_d      = 0.0f;
volatile int32_t pos_actual = 0;
float   error_pos  = 0.0f;

// Velocidad
float omega          = 0.0f;
float omega_d        = 0.0f;
float error_vel      = 0.0f;
float error_vel_prev = 0.0f;
float integral_vel   = 0.0f;

// Ganancias empleadas
float Kp_pos = 0.5f;
float Kp     = 1.3f;
float Ki     = 5.0f;

float ADC_filtrado = 0.0f;//handler de filtrado

/* Prototipos de función */
void initPer(void);
void initADC(void);
uint16_t getADC(void);
void initPWM(void);
void initEncoder(void);
void initIR(void);



void TIM14_IRQHandler(void){ // handler del timer
    if(TIM14->SR & TIM_SR_UIF){
        TIM14->SR &= ~TIM_SR_UIF; //Limpiar bandera de interrupción
        flag = 1;

        }
}

int main(void){//función main

    initPer();
    initADC();
    initPWM();
    initEncoder();
    initIR();
    while (1)
    {
        if (flag){
			flag = 0;

			/* ===== 1. ADC ===== */
			ADC_value=(float)getADC();
			ADC_filtrado = 0.9f * ADC_filtrado + 0.1f * ADC_value;

			vel_d = (ADC_filtrado / 4095.0f) * (float)MAX_VEL;

			/* ===== 2. POSICION DESEADA ===== */
			pos_d = (ADC_filtrado / 4095.0f) * (float)MAX_PULSES;

			/* ===== 3. LECTURA DEL ENCODER ===== */
			int32_t current = (int32_t)TIM2->CNT;
			pos_actual = current;

			int32_t delta = current - prev_count;
			prev_count = current;

			/* ===== 4. LAZO EXTERNO: CONTROL DE POSICION ===== */
			error_pos = pos_d - (float)pos_actual;
			float vel_ref = Kp_pos * error_pos;

			/* Zona muerta */
			if (fabsf(error_pos) < 100.0f)
			{
				vel_ref = 0.0f;
			}

			/* Saturacion de referencia */
			if (vel_ref > MAX_RPM)  vel_ref = MAX_RPM;
			if (vel_ref < -MAX_RPM) vel_ref = -MAX_RPM;

			/* ===== 5. MEDIR VELOCIDAD ===== */
			omega = ((float)delta / (float)PPR) * (60.0f / DT);
			/* ===== 6. LAZO INTERNO: PID DE VELOCIDAD ===== */
			omega_d = vel_ref;

			/*omega_d = vel_d;
			if (omega_d < 8){
				omega_d =0;

			}*/
			error_vel = omega_d - omega;

			/* Anti-windup simple */
			float u_unsat = (Kp * error_vel) + (Ki * integral_vel);
			float u = u_unsat;

			if (u > 999.0f)  u = 999.0f;
			if (u < -999.0f) u = -999.0f;

			if ((u == u_unsat) || ((u > 0.0f) != (error_vel > 0.0f)))
			{
				integral_vel += error_vel * DT;
			}

			error_vel_prev = error_vel;

			/* ===== 7. SALIDA PWM ===== */
			uint16_t pwm = (uint16_t)fabsf(u);
			if (pwm > 999U) pwm = 999U;



			for (volatile int i = 0; i < 50; i++);

			if (u > 0.0f)
			{
				TIM3->CCR4 = 0;
				TIM3->CCR3 = pwm;   /* Adelante */

			}
			else if (u < 0.0f)
			{
				TIM3->CCR3 = 0;
				TIM3->CCR4 = pwm;   /* Reversa */
			}
			else
			{
				TIM3->CCR3 = 0;
				TIM3->CCR4 = 0;
			}
        }
    }
}

/* perifericos */
void initPer(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN;

    /* PA2 = analogico (ADC IN2) */
    GPIOA->MODER &= ~(3U << (2U * 2U));
    GPIOA->MODER |=  (3U << (2U * 2U));
}

/* adc */
void initADC(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /* HSI14 para ADC */
    RCC->CR2 |= RCC_CR2_HSI14ON;
    while (!(RCC->CR2 & RCC_CR2_HSI14RDY));

    /* Asegurar ADC deshabilitado antes de calibrar */
    if (ADC1->CR & ADC_CR_ADEN)
    {
        ADC1->CR |= ADC_CR_ADDIS;
        while (ADC1->CR & ADC_CR_ADEN);
    }

    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL);

    ADC1->ISR |= ADC_ISR_ADRDY;   /* limpiar flag si quedó pendiente */
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY));

    ADC1->CHSELR = ADC_CHSELR_CHSEL2;
    ADC1->SMPR = 3U;   // tiempo máximo de sampleo
}

// obtener el adc
uint16_t getADC(void)
{
    ADC1->CR |= ADC_CR_ADSTART;
    while (!(ADC1->ISR & ADC_ISR_EOC));
    return (uint16_t)ADC1->DR;
}

/* pwm */
void initPWM(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->AHBENR  |= RCC_AHBENR_GPIOCEN;

    /* PC8 = TIM3_CH3, PC9 = TIM3_CH4, AF0 */
    GPIOC->MODER &= ~((3U << (8U * 2U)) | (3U << (9U * 2U)));
    GPIOC->MODER |=  ((2U << (8U * 2U)) | (2U << (9U * 2U)));

    GPIOC->AFR[1] &= ~((0xFU << ((8U - 8U) * 4U)) | (0xFU << ((9U - 8U) * 4U)));
    /* AF0 => no hace falta poner bits */

    /* PWM 1 kHz: 8 MHz / (8 * 1000) = 1000 Hz */
    TIM3->PSC = 7;
    TIM3->ARR = 999;

    /* CH3 PWM mode 1 */
    TIM3->CCMR2 &= ~(7U << 4);
    TIM3->CCMR2 |=  (6U << 4);
    TIM3->CCMR2 |=  TIM_CCMR2_OC3PE;

    /* CH4 PWM mode 1 */
    TIM3->CCMR2 &= ~(7U << 12);
    TIM3->CCMR2 |=  (6U << 12);
    TIM3->CCMR2 |=  TIM_CCMR2_OC4PE;

    TIM3->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM3->CR1  |= TIM_CR1_ARPE;

    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;

    TIM3->EGR |= TIM_EGR_UG;
    TIM3->CR1 |= TIM_CR1_CEN;
}

/* Encoder */
void initEncoder(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;

    // PA0 canal 1 y PA1 canal 2
    GPIOA->MODER &= ~((3U << (0U * 2U)) | (3U << (1U * 2U)));
    GPIOA->MODER |=  ((2U << (0U * 2U)) | (2U << (1U * 2U)));

    GPIOA->PUPDR &= ~((3U << (0U * 2U)) | (3U << (1U * 2U)));
    GPIOA->PUPDR |=  ((1U << (0U * 2U)) | (1U << (1U * 2U)));   /* pull-up */

    GPIOA->AFR[0] &= ~((0xFU << (0U * 4U)) | (0xFU << (1U * 4U)));
    GPIOA->AFR[0] |=  ((2U << (0U * 4U)) | (2U << (1U * 4U)));

    /* Reset limpio de TIM2 */
    TIM2->CR1   = 0;
    TIM2->SMCR  = 0;
    TIM2->CCMR1 = 0;
    TIM2->CCER  = 0;
    TIM2->CNT   = 0;

    /* Entradas mapeadas a TI1 y TI2 */
    TIM2->CCMR1 |= (1U << 0);   /* CC1S = 01 */
    TIM2->CCMR1 |= (1U << 8);   /* CC2S = 01 */

    /* Encoder mode 3 */
    TIM2->SMCR |= 3U;

    /* Opcional: sin invertir polaridad */
    TIM2->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);

    TIM2->ARR = 0xFFFFFFFFU;
    TIM2->CR1 |= TIM_CR1_CEN;
}

//inicilizar timer 14 para interrupción cada 10ms
void initIR(void){
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;

    TIM14->PSC = 7;      // 8 MHz / 8000 = 1 kHz (1 ms por tick)
    TIM14->ARR = 9999;         // 10 ms periodo
    TIM14->DIER |= TIM_DIER_UIE;
    TIM14->CR1  |= TIM_CR1_CEN;

    NVIC_SetPriority(TIM14_IRQn, 3);
    NVIC_EnableIRQ(TIM14_IRQn);
}


