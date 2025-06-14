//Importación de librerias
#include "stm32f051x8.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Definiciones generales
#define PWM_MAX_DUTY 100 //Ciclo de trabajo máximo
#define UART_BUFFER_SIZE 128 //Tamaño de buffer para recepción de texto por USART
#define NUM_DIFFUSE_SENSORS 5 //Sensores difusos
#define NUM_LIMIT_SWITCHES 2 //Sensores de final de carrera

// Configuración de Encoders
#define ENCODER_CLK_PIN 10  // PA10
#define ENCODER_DT_PIN 11  // PA11
#define PULSOS_POR_REV 500/2  // 500 PPR × 2 flancos
#define RADIO_MM 101.6f  // 4 pulgadas = 101.6 mm
#define PERIMETRO_MM (2 * 3.1416f * RADIO_MM)  // ≈ 638.37 mm
#define MM_POR_PULSO (PERIMETRO_MM / PULSOS_POR_REV) //Distancia por cada pulso
#define REV_POR_PULSO (1 / PULSOS_POR_REV) //Rpm por pulso


// Estructura para estados de sensores (diccionario)
typedef struct {
    uint8_t diffuse[NUM_DIFFUSE_SENSORS];//Arreglo con los estados de sensores de presencia
    uint8_t limits[NUM_LIMIT_SWITCHES];//Arreglo con los estados de sensores de límite
} SensorStates;

// Variables globales
volatile SensorStates sensor_states = {0};//Inicializar en 0 cada valor del diccionario
volatile uint8_t motion_state = 1; // bandera de movimiento AGV 1 = movimiento permitido
volatile uint8_t motion_state_L = 1; // bandera de movimiento Tijera 1 = movimiento permitido
volatile char uart_buffer[UART_BUFFER_SIZE]; //Arreglo char para el almacenado de datos bluetooth
volatile uint8_t uart_head = 0, uart_tail = 0; //Variables para byte de incio y parada del mensaje USART

// Variables de encoder
// Variables globales (volatile para ISR)
volatile int32_t encoder_counter = 0; //Contador en tiempo real de pulsos
volatile int32_t last_encoder_count = 0; //Ultimos pulsos contados previo a la interrupción
volatile uint32_t uwTick = 0;    // Contador de tiempo global del micro en ms

// Prototipos de funciones//

//Inicialización
void GPIO_Init(void);
void USART2_Init(void);
void PWM_Init(void);
void Encoder_Init(void);
void Motion_Sensors_Init(void);

//Procesos
void USART2_SendString(const char *str);
void USART2_SendChar(char c);
void PWM_SetDuty(uint8_t channel, uint8_t duty);
void Check_Sensors(void);
void Process_Command(const char* cmd);
void Motion_ControlLoop(void);
void Leds_Init(void);
uint32_t GetTick(void);

// Variables globales para control de LEDs
volatile uint32_t led_last_time = 0;
volatile uint8_t leds_state = 0;
void Update_Warning_Leds(int time);

// **********************
// ** Inicializaciones **
// **********************

void GPIO_Init(void) {//Habilita los puertos GPIO de los buses A,B y C
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
}

void Leds_Init(void){//Define pines GPIO's como salidas generales para encendido de leds
	GPIOB->MODER&=~(((0x3)<<(1*2)));
	GPIOB->MODER|=((1<<(1*2)));
	GPIOC->MODER&=~((0x3)<<(11*2));
	GPIOC->MODER|=(1<<(11*2));
}
void Update_Warning_Leds(int time) {//Define la rutina de activación de leds y luces para el estado de movimiento del AGV
									//y el paro al detectar un obstáculo
    static uint8_t last_motion_state = 1;//Bandera local para guardar el último estado de movimiento del AGV
    static uint8_t last_motion_state_L = 1;//Bandera local para guardar el último estado de movimiento de la tijera

    uint32_t now = GetTick();//Call a la función para obtener el tiempo del micro

    if(motion_state != last_motion_state) {// Si hay un cambio respecto al estado actual del AGV...
        last_motion_state = motion_state;//Actualizar bandera con nuevo estado
        led_last_time = now;
        leds_state = 1; // Comenzar con LEDs encendidos

        if(motion_state) {
            // Si el sistema está en movimiento, encender LED's
            GPIOB->ODR &= ~((1<<1));
            GPIOC->ODR |= (1<<11);
            return;
        }
    }
    if(motion_state_L != last_motion_state_L) {// Si hay un cambio respecto al estado actual de la tijera...
            last_motion_state_L = motion_state_L;
            led_last_time = now;
            leds_state = 1; // Comenzar con LEDs encendidos

            if(motion_state_L) {
                // Si el sistema está en movimiento, encender LED's
                GPIOB->ODR &= ~((1<<1));
                GPIOC->ODR |= (1<<11);
                return;
            }
        }
    if(!motion_state||!motion_state_L) {
        // Si cualquiera de los sistemas está detenido, hacer parpadear LEDs y encender buzzer
        if(now - led_last_time >= time) {//Rutina de parpadeo cada cierto tiempo definido en la función
            leds_state = !leds_state;//Bandera para cambia estado de encendido y apagado

            if(leds_state) {//estado encendido
                GPIOB->ODR |= (1<<1);
                GPIOC->ODR |= (1<<11);
            } else {//estado apagado
                GPIOB->ODR &= ~((1<<1));
                GPIOC->ODR &= ~(1<<11);
            }

            led_last_time = now;//Actualizar el tiempo
        }
    }
}

void USART2_Init(void) {//Configuración de pines para comunicación USART
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;//Habilitar Reloj para el periférico de comunicación USART

    // Configurar pines PA2 (TX) y PA3 (RX) como función alterna
    GPIOA->MODER |= (GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1);
    GPIOA->AFR[0] |= (1 << (2*4)) | (1 << (3*4));

    USART2->BRR = 8000000 / 9600;  // Baudrate a 9600 (8 MHz clock)
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_RXNEIE;//Habilitar transmisor, receptor, USART y  la interrupción para recepción

    NVIC_SetPriority(USART2_IRQn, 2);//Definir prioridd de interrupción media
    NVIC_EnableIRQ(USART2_IRQn);//Habilitar rutina de interrupción
}

void PWM_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;//Habilitar reloj del TIM 3

    // Configurar PC6-PC9 como AF para pwm
    GPIOC->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1 |
                    GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1);
    GPIOC->AFR[0] &= ~(0xFF << 24); // PC6-PC7: AF0
    GPIOC->AFR[1] &= ~0xFF;         // PC8-PC9: AF0

    // Configurar TIM3 (1 kHz PWM)
    TIM3->PSC = 7;
    TIM3->ARR = 999;
    TIM3->CCMR1 = (6 << 4) | (1 << 3) | (6 << 12) | (1 << 11);
    TIM3->CCMR2 = (6 << 4) | (1 << 3) | (6 << 12) | (1 << 11);
    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;//Hablitar los cuatro channels
    TIM3->CR1 = TIM_CR1_CEN;//Habilitar el conteo del reloj

    // Configurar PB10 y PB11 como salidas
    GPIOB->MODER |= (1 << (10*2)) | (1 << (11*2));
}

void Encoder_Init(void) {
    GPIOA->MODER &= ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11);  // Entradas
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR10_0 | GPIO_PUPDR_PUPDR11_0);  // Pull-up

    // Configurar EXTI para PA10 (flanco de subida/bajada)
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PA;
    EXTI->IMR |= (1 << ENCODER_CLK_PIN);
    EXTI->RTSR |= (1 << ENCODER_CLK_PIN);   // Flanco de subida
    EXTI->FTSR |= (1 << ENCODER_CLK_PIN);   // Flanco de bajada
    NVIC_SetPriority(EXTI4_15_IRQn, 0);
    NVIC_EnableIRQ(EXTI4_15_IRQn);
}
void Motion_Sensors_Init(void) {
    // Configurar sensores difusos (PA0-PA1, PA4-PA7) como entrada
    GPIOA->MODER &= ~(GPIO_MODER_MODER1 |
                     GPIO_MODER_MODER4 | GPIO_MODER_MODER5 |
                     GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    //Habilitar para cada pin su resistencia pull up (colector abierto)
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0 |
                    GPIO_PUPDR_PUPDR4_0 | GPIO_PUPDR_PUPDR5_0 |
                    GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0);

    // Configurar pines de limit switches (PA8-PA11) como entrada
        GPIOA->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);

        // Configurar pull-up para limit switches (NC)
        GPIOA->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
        GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR9_0);



    // Configurar EXTI para las interrupciones de los sensores de precencia
    SYSCFG->EXTICR[0] |=  SYSCFG_EXTICR1_EXTI1_PA;
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PA | SYSCFG_EXTICR2_EXTI5_PA |
                        SYSCFG_EXTICR2_EXTI6_PA | SYSCFG_EXTICR2_EXTI7_PA;

    // Configurar EXTI para para las interrupciones de los sensores de limite
    SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI8_PA | SYSCFG_EXTICR3_EXTI9_PA;


    EXTI->IMR |= (1<<1) | (1<<4) | (1<<5) | (1<<6) | (1<<7) | (1<<8) | (1<<9);//Habilitar las interrupciones para cada pin

    //Habilitar flanco de bajada y de subida para sensores de presencia
    EXTI->FTSR |= (1<<1) | (1<<4) | (1<<5) | (1<<6) | (1<<7);
    EXTI->RTSR |= (1<<1) | (1<<4) | (1<<5) | (1<<6) | (1<<7);

    //Habilitar flanco de bajada y de subida para sensores de limite
       EXTI->FTSR |= (1<<8) | (1<<9);    // flanco bajada
       EXTI->RTSR |= ((1<<8) | (1<<9)); // flanco subida


    NVIC_SetPriority(EXTI0_1_IRQn, 0);//Definir prioridad de rutina de interrupción como alta
    NVIC_SetPriority(EXTI4_15_IRQn, 0);
    NVIC_EnableIRQ(EXTI0_1_IRQn);//Habilitar rutinas de interrumpción
    NVIC_EnableIRQ(EXTI4_15_IRQn);
}

// **********************
// ** Funciones de Interrupción **
// **********************

void SysTick_Handler(void) {//Función para conteo del tiempo global del micro
    uwTick++;
}

void USART2_IRQHandler(void) {//Algoritmo de buffer circular
    if(USART2->ISR & USART_ISR_RXNE) {//Verifica si se recibe un dato al activarse la flag de interrupción
        uart_buffer[uart_head] = USART2->RDR;//Almacena el último dato recibido en el buffer
        uart_head = (uart_head + 1) % UART_BUFFER_SIZE;//Actualiza el indice y evita desbordamiento
    }
}
void USART2_SendChar(char c) {
    while(!(USART2->ISR & USART_ISR_TXE));//Espera a que el registro este vacio, activando la flag
    USART2->TDR = c;//Se escribe el dato en el registro de transmición
}

void USART2_SendString(const char *str) {//Función para envíar strings
    while(*str) {//Mientras no se llegue al final de la cadena \0
        USART2_SendChar(*str++);//Enviar el caracter deseado
    }
}

void EXTI0_1_IRQHandler(void) {//Rutina de interrupción para sensor de precencia del pin A1
    if(EXTI->PR & (EXTI_PR_PR1)) {//Espera a que se presente una interrupción
        EXTI->PR = EXTI_PR_PR1;//Establece en alto la bandera para que la interrupción no se repita
        Check_Sensors();//Call a check sensors
    }
}

void EXTI4_15_IRQHandler(void) {
    // Manejo de encoders
	if (EXTI->PR & (1 << ENCODER_CLK_PIN)) {//Si ocurre una interupcción en el registro del pin del encoder...

	        if (1) { //step
	            uint8_t clk = (GPIOA->IDR >> ENCODER_CLK_PIN) & 1;//Variable local con el estado (alto o bajo) del canal A
	            uint8_t dt = (GPIOA->IDR >> ENCODER_DT_PIN) & 1;;//Variable local con el estado (alto o bajo) del canal B
	            encoder_counter += (dt != clk) ? -1 : 1;//Contador de pulsos, negativo cuando A y B no están en cuadratura
	        }
	        EXTI->PR = (1 << ENCODER_CLK_PIN);  // Limpiar flag para evitar que vuelva a dispararse
	    }

    // Manejo de sensores
    uint32_t pending = EXTI->PR & 0xFFF0;//Guarda el estado de las interrupciones de todos los pines
    for(int i = 4; i <= 7; i++) {
        if(pending & (1 << i)) {//Evalua el estado de las interrupciones de todos los pines
            EXTI->PR = (1 << i);//Limpia la flag
            sensor_states.diffuse[i-3] = !(GPIOA->IDR & (1 << i));//Pone el estado del sensor en el arrglo
        }
    }

    // Limit switches PA8-PA9
        for(int i = 8; i <= 9; i++) {
            if(pending & (1 << i)) {
                EXTI->PR = (1 << i); // Limpiar flag escribiendo 1
                sensor_states.limits[i-8] = (GPIOA->IDR & (1 << i)) ? 1 : 0;
            }
        }

    Check_Sensors();//Call a check sensors
}

// **********************
// ** Funciones Principales **
// **********************

void Check_Sensors(void) {
	//Banderas locales de detección
	uint8_t obstacle_detected = 0;
	uint8_t obstacle_detected_L = 0;

	// Verificar sensores difusos (PA1 y PA4-PA7)
	sensor_states.diffuse[0] = !(GPIOA->IDR & GPIO_IDR_1) ? 1 : 0;
	sensor_states.diffuse[1] = !(GPIOA->IDR & GPIO_IDR_4) ? 1 : 0;
	sensor_states.diffuse[2] = !(GPIOA->IDR & GPIO_IDR_5) ? 1 : 0;
	sensor_states.diffuse[3] = !(GPIOA->IDR & GPIO_IDR_6) ? 1 : 0;
	sensor_states.diffuse[4] = !(GPIOA->IDR & GPIO_IDR_7) ? 1 : 0;

	// Verificar limit switches (PA8-PA9)
	sensor_states.limits[0] = (GPIOA->IDR & GPIO_IDR_8) ? 1 : 0;
	sensor_states.limits[1] = (GPIOA->IDR & GPIO_IDR_9) ? 1 : 0;


	// Comprobar si hay algún obstáculo en agv
	for(int i = 0; i < NUM_DIFFUSE_SENSORS; i++) {
		if(sensor_states.diffuse[i]) {
			obstacle_detected = 1;
			break;
		}
	}
	if(sensor_states.limits[0]) {
				obstacle_detected = 1;
			}

	if(sensor_states.limits[1]) {
		obstacle_detected_L = 1;
	}


	// Banderas para actualizar estados de movimiento
	uint8_t new_motion_state = !obstacle_detected;
	uint8_t new_motion_state_L = !obstacle_detected_L;

	// Detener motores si cambiamos de estado a detenido
	if(motion_state && !new_motion_state) {
		for(int i = 1; i <= 4; i++) PWM_SetDuty(i, 0);
	}
	motion_state = new_motion_state;//Actualizar estado de movimiento
	if(motion_state_L && !new_motion_state_L) {
		for(int i = 5; i <= 6; i++) PWM_SetDuty(i, 0);
	}
	motion_state_L = new_motion_state_L;

}

void PWM_SetDuty(uint8_t channel, uint8_t duty) {//Handler para pwm
    if(duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;//Evitamos overflow de ciclo
    uint16_t value = duty * (TIM3->ARR + 1) / 100;//Ponemos el ciclo en escala de 0 a 100

    switch(channel) {//Dependiendo el comando recibido
        case 1: TIM3->CCR1 = value; TIM3->CCR3 = value; break;//Adelante
        case 2: TIM3->CCR2 = value; TIM3->CCR3 = value; break;//Derecha
        case 3: TIM3->CCR1 = value; TIM3->CCR4 = value; break;//Izquierda
        case 4: TIM3->CCR2 = value; TIM3->CCR4 = value; break;//Atras
        case 5: GPIOB->ODR = (duty) ? (GPIOB->ODR | (1 << 10)) : (GPIOB->ODR & ~(1 << 10)); break;//Tijera
        case 6: GPIOB->ODR = (duty) ? (GPIOB->ODR | (1 << 11)) : (GPIOB->ODR & ~(1 << 11)); break;//Tijera
    }
}

void Process_Command(const char* cmd) {//Función para procesar comando de acción
    int duty, channel;
    if(sscanf(cmd, "%d %d", &duty, &channel) == 2) {//Escanea del buffer los argumentos para el set duty
        if(channel >= 1 && channel <= 6 && motion_state) {//Validación de comando
            PWM_SetDuty(channel, duty);//Call a set duty
        }
    }
}

void Motion_ControlLoop(void) {//Loop principal
    static char cmd_buffer[32];//Buffer para lectura de recepción
    static uint8_t idx = 0;//Indice

    // Leer y guardar comandos UART
    while(uart_head != uart_tail) {//Si tenemos datos nuevos
        char c = uart_buffer[uart_tail];//Leemos el dato almacenado en el buffer
        uart_tail = (uart_tail + 1) % UART_BUFFER_SIZE;//Actualizamos posición

        if(c == '\r' || c == '\n') {//Se detecta salto de linea
            cmd_buffer[idx] = '\0';//Cierra la cadena para string válido
            Process_Command(cmd_buffer);//Procesa el comando
            idx = 0;//Reinicia el indice
        }
        else if(idx < sizeof(cmd_buffer)-1) {//Evita overflow
            cmd_buffer[idx++] = c;//Guarda caracter en la posición y luego cambia posición
        }
    }
    // Mostrar velocidad cada 100ms
    static uint32_t last_print = 0;
    uint32_t now = GetTick();
    if(now - last_print >= 100) {
        last_print = now;//Actulizar tiempo

        int32_t delta = encoder_counter - last_encoder_count;//Cantidad de pulsos del encoder
        last_encoder_count = encoder_counter;//Actulizar último conteo

        // Convierte pulsos a velocidad (mm/s)
	   float speed_mm_per_sec = (delta * MM_POR_PULSO) / 0.1f;  // 100 ms = 0.1s

	   //Envía la velocidad a la aplicación
	   char buffer[64];
	   snprintf(buffer, sizeof(buffer), "Vel: %.2f mm/s\r\n", speed_mm_per_sec);
	   USART2_SendString(buffer);
    }
}

// **********************
// ** Funciones de Utilidad **
// **********************

uint32_t GetTick(void) {//Función para obtner el tiempo global del micro
    return uwTick;
}

// **********************
// ** Main **
// **********************

int main(void) {
    // Inicializaciones
    GPIO_Init();
    USART2_Init();
    PWM_Init();
    Encoder_Init();
    Motion_Sensors_Init();
    Leds_Init();


    // Configurar SysTick para 1ms
    SysTick_Config(8000000 / 1000);

    uint32_t last_led_update = GetTick(); // Variable para control de timing


    USART2_SendString("Sistema iniciado\n");//Inicio

    while(1) {
	   // 1. Procesamiento principal (siempre activo)
			Motion_ControlLoop();

		// 2. Actualización de LEDs (no bloqueante, cada 300ms)
		uint32_t now = GetTick();
		if(now - last_led_update >= 300) {
			Update_Warning_Leds(300); // Parpadeo cada 300ms
			last_led_update = now;
		}

    }
}
