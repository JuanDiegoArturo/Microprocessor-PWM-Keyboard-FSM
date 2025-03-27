#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include <stdio.h>

/***************************
 * Definitions
 **************************/
#define BOARD_LED_GPIO     BOARD_INITPINS_LED_RED_GPIO // GPIOE

// LED -> Salida
#define BOARD_LED_GPIO_PIN BOARD_INITPINS_LED_RED_PIN  // E2
// Filas -> Salidas
#define BOARD_FIL1_GPIO_PIN BOARD_INITPINS_FIL1_PIN    // E3
#define BOARD_FIL2_GPIO_PIN BOARD_INITPINS_FIL2_PIN    // E4
#define BOARD_FIL3_GPIO_PIN BOARD_INITPINS_FIL3_PIN    // E5
#define BOARD_FIL4_GPIO_PIN BOARD_INITPINS_FIL4_PIN    // E6

// Columnas -> Entradas
#define BOARD_COL1_GPIO_PIN BOARD_INITPINS_COL1_PIN    // E9
#define BOARD_COL2_GPIO_PIN BOARD_INITPINS_COL2_PIN    // E10
#define BOARD_COL3_GPIO_PIN BOARD_INITPINS_COL3_PIN    // E11
#define BOARD_COL4_GPIO_PIN BOARD_INITPINS_COL4_PIN    // E12

//Variables
int col = 0, fil = 0;
int cont = 1;
int DutyCycle, Dmax, Ton, Toff;
int NUM, DEC; // Num = unidades, Dec = decenas
int check_col1, check_col2, check_col3, check_col4;

enum Estados {INICIO, PWM, DUTYCYCLE};

/***************************
 * Variables
 **************************/
volatile uint32_t g_systickCounter;

/***************************
 * Code
 **************************/
void SysTick_Handler(void)
{
    if (g_systickCounter != 0U)
    {
        g_systickCounter--;
    }
}

void SysTick_DelayTicks(uint32_t n)
{
    g_systickCounter = n;
    while (g_systickCounter != 0U)
    {
    }
}

// Función para leer el teclado
void Teclado(void)
{
    for (int fila = 1; fila <= 4; fila++)
    {
        // Apagar todas las filas
        GPIO_PortSet(BOARD_LED_GPIO, 1u << BOARD_FIL1_GPIO_PIN);
        GPIO_PortSet(BOARD_LED_GPIO, 1u << BOARD_FIL2_GPIO_PIN);
        GPIO_PortSet(BOARD_LED_GPIO, 1u << BOARD_FIL3_GPIO_PIN);
        GPIO_PortSet(BOARD_LED_GPIO, 1u << BOARD_FIL4_GPIO_PIN);

        // Activar la fila actual
        if (fila == 1) GPIO_PortClear(BOARD_LED_GPIO, 1u << BOARD_FIL1_GPIO_PIN);
        if (fila == 2) GPIO_PortClear(BOARD_LED_GPIO, 1u << BOARD_FIL2_GPIO_PIN);
        if (fila == 3) GPIO_PortClear(BOARD_LED_GPIO, 1u << BOARD_FIL3_GPIO_PIN);
        if (fila == 4) GPIO_PortClear(BOARD_LED_GPIO, 1u << BOARD_FIL4_GPIO_PIN);

        // Leer el estado de todas las columnas
        check_col1 = GPIO_PinRead(BOARD_LED_GPIO, BOARD_COL1_GPIO_PIN);
        check_col2 = GPIO_PinRead(BOARD_LED_GPIO, BOARD_COL2_GPIO_PIN);
        check_col3 = GPIO_PinRead(BOARD_LED_GPIO, BOARD_COL3_GPIO_PIN);
        check_col4 = GPIO_PinRead(BOARD_LED_GPIO, BOARD_COL4_GPIO_PIN);

        // Actualiza fil y col si se presionó algún botón
        if (check_col1 == 0 || check_col2 == 0 || check_col3 == 0 || check_col4 == 0)
        {
            if (check_col1 == 0)
            {
                col = 1;
                fil = fila;
            }
            else if (check_col2 == 0)
            {
                col = 2;
                fil = fila;
            }
            else if (check_col3 == 0)
            {
                col = 3;
                fil = fila;
            }
            else if (check_col4 == 0)
            {
                col = 4;
                fil = fila;
            }
        }
    }
}

int main(void)
{
    /* Board pin init */
    BOARD_InitPins();
    BOARD_InitBootClocks();

    // Condiciones Iniciales PWM
    DutyCycle = 99; // De 0 a 99
    Dmax = 100U;    // Periodo máximo con D = 100%
    Ton = (Dmax * DutyCycle) / 100;
    Toff = Dmax - Ton;

    void pwm(void)
    {
        GPIO_PortClear(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN); //Enciende LED
        SysTick_DelayTicks((Dmax * DutyCycle) / 100); // Tiempo encendido
        GPIO_PortToggle(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN); //Apaga LED
        SysTick_DelayTicks(Dmax - (Dmax * DutyCycle) / 100); // Tiempo apagado
    }

    /* Set systick reload value to generate 10 us interrupt */
    if (SysTick_Config(SystemCoreClock / 100000U)) // interrupción cada 10 microsegundos
    {
        while (1)
        {
        }
    }

    enum Estados Estado_actual = INICIO; //Estado inicial 

    while (1)
    {
        switch (Estado_actual) // Máquina de estados que revisa el estado actual
        {
        case INICIO:
            Teclado();
            if (col == 4 && fil == 1) // A
            {
                Estado_actual = PWM;
                GPIO_PortSet(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN); // Apaga led
            }
            else
            {
                Estado_actual = INICIO;
                GPIO_PortSet(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN); // Apaga led
            }
            col = 0, fil = 0; // Limpia teclado
            break;

        case PWM:
            Teclado();
            if (col == 4 && fil == 2){ // B
                Estado_actual = INICIO;
                GPIO_PortSet(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN); // Apaga led
            }
            else if (col < 4 && col != 0){ // Si no es letra
                if ((col == 1 || col == 3) && fil == 4){ // * o #
                    Estado_actual = PWM;
                    pwm();
                }
                else if (col == 2 && fil == 4){ // 0
                    DEC = 0;
                    col = 0;
                    fil = 0;
                    while (col == 0 && fil == 0){ //Esperar a siguiente dígito
                        pwm();
                        Teclado();
                    }
                    if (col == 2 && fil == 4){ // 0
                        NUM = 0;
                        pwm();
                        Estado_actual = DUTYCYCLE;
                    }
                    else{ //Guardar dígito
                        NUM = (fil - 1) * 3 + col;
                        pwm();
                        Estado_actual = DUTYCYCLE;
                    }
                }
                else // != 0
                {
                    DEC = (fil - 1) * 3 + col; // Guardar dígito en decena
                    col = 0;
                    fil = 0;
                    while (col == 0 && fil == 0){ // Esperar siguiente dígito
                        Teclado();
                    }
                    if (col == 2 && fil == 4){ // 0
                        NUM = 0;
                        pwm();
                        Estado_actual = DUTYCYCLE;
                    }
                    else {
                        NUM = (fil - 1) * 3 + col; // Guardar dígito en unidad
                        pwm();
                        Estado_actual = DUTYCYCLE;
                    }
                }
            }
            else{ // A, C, D o nada
                Estado_actual = PWM;
                pwm();
            }
            col = 0, fil = 0; // Limpia teclado
            break;

        case DUTYCYCLE:
            Teclado();
            if (col == 4 && fil == 2){ // B
                Estado_actual = INICIO;
                GPIO_PortSet(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN); // Apaga led
            }
            else if (col == 4 && fil == 3){ // C
                pwm();
                Estado_actual = PWM;
            }
            else if (col == 4 && fil == 4){ // D
                DutyCycle = (DEC * 10) + NUM; // De 0 a 99
                Ton = (Dmax * DutyCycle) / 100;
                Toff = Dmax - Ton;
                Estado_actual = PWM;
            }
            else{
                pwm();
                Estado_actual = DUTYCYCLE;
            }
            col = 0, fil = 0; // Limpia teclado
            break;
        }
    }
    return 0;
}
