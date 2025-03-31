#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "UART/UART.h"

// Definición de pines para los botones
#define UP_PIN        PD7
#define DOWN_PIN      PD6
#define LEFT_PIN      PD5
#define RIGHT_PIN     PD4
#define ACTION_A_PIN  PD3
#define ACTION_B_PIN  PD2

// Máscaras para los botones
#define UP_BUTTON     (1 << UP_PIN)
#define DOWN_BUTTON   (1 << DOWN_PIN)
#define LEFT_BUTTON   (1 << LEFT_PIN)
#define RIGHT_BUTTON  (1 << RIGHT_PIN)
#define A_BUTTON      (1 << ACTION_A_PIN)
#define B_BUTTON      (1 << ACTION_B_PIN)

// Tiempo de debounce en milisegundos
#define DEBOUNCE_DELAY 20

// Prototipos de funciones
void Buttons_Init(void);
uint8_t Read_Buttons(void);
void Send_Button_Command(uint8_t button);

int main(void)
{
	// Inicialización de periféricos
	UART_Init(103); // 9600 baudios para 16MHz
	Buttons_Init();
	
	uint8_t last_button = 0;
	uint8_t current_button = 0;
	
	while(1)
	{
		current_button = Read_Buttons();
		
		// Solo enviar comando si hay un cambio de estado
		if(current_button != last_button)
		{
			if(current_button != 0)
			{
				Send_Button_Command(current_button);
				_delay_ms(DEBOUNCE_DELAY); // Espera para debounce
			}
			last_button = current_button;
		}
		
		_delay_ms(50); // Pequeña pausa entre lecturas
	}
}

// Función para inicializar los botones
void Buttons_Init(void)
{
	// Configurar pines de botones como entrada
	DDRD &= ~(UP_BUTTON | DOWN_BUTTON | LEFT_BUTTON | RIGHT_BUTTON | A_BUTTON | B_BUTTON);
	
	// Activar resistencias pull-up internas
	PORTD |= (UP_BUTTON | DOWN_BUTTON | LEFT_BUTTON | RIGHT_BUTTON | A_BUTTON | B_BUTTON);
}

// Función para leer el estado de los botones
uint8_t Read_Buttons(void)
{
	uint8_t button_state = PIND;
	uint8_t pressed_button = 0;
	
	// Comprobar cada botón (prioridad en orden de comprobación)
	if (!(button_state & UP_BUTTON)) {
		pressed_button = 'u'; // Arriba
	}
	else if (!(button_state & DOWN_BUTTON)) {
		pressed_button = 'd'; // Abajo
	}
	else if (!(button_state & LEFT_BUTTON)) {
		pressed_button = 'l'; // Izquierda
	}
	else if (!(button_state & RIGHT_BUTTON)) {
		pressed_button = 'r'; // Derecha
	}
	else if (!(button_state & A_BUTTON)) {
		pressed_button = 'a'; // Acción A
	}
	else if (!(button_state & B_BUTTON)) {
		pressed_button = 'b'; // Acción B
	}
	
	return pressed_button;
}

// Función para enviar el comando del botón por UART
void Send_Button_Command(uint8_t button)
{
	UART_Transmit(button); // Envía el carácter correspondiente al botón
	
	// Opcional: enviar un salto de línea para mejor legibilidad en terminal
	// UART_Transmit('\n');
}