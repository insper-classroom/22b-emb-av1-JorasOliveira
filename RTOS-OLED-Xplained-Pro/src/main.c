#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* Botao da placa */
#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

//definindo os botoes da placa EXT1
//Botao 1
#define BUT1_PIO     PIOD    //perifferico que controla o botao1
#define BUT1_PIO_ID  ID_PIOD //ID do periferico PIOD
#define BUT1_PIO_IDX 28	    //ID do botao1 no PIO
#define BUT1_PIO_IDX_MASK (1u << BUT1_PIO_IDX) //mascara para lermos o botao1
//Botao 2
#define BUT2_PIO     PIOC    //perifferico que controla o botao2
#define BUT2_PIO_ID  ID_PIOC //ID do periferico PIOC
#define BUT2_PIO_IDX 31	    //ID do botao2 no PIO
#define BUT2_PIO_IDX_MASK (1u << BUT2_PIO_IDX) //mascara para lermos o botao2
//Botao 3
#define BUT3_PIO     PIOA    //perifferico que controla o botao3
#define BUT3_PIO_ID  ID_PIOA //ID do periferico PIOA
#define BUT3_PIO_IDX 19	    //ID do botao3 no PIO
#define BUT3_PIO_IDX_MASK (1u << BUT3_PIO_IDX) //mascara para lermos o botao3

//Definindo os pinos do motor
//fase 1
#define IN_1_PIO     PIOD    //Pino para a fase 1 do motor
#define IN_1_PIO_ID  ID_PIOD //ID do periferico PIOD
#define IN_1_PIO_IDX 30	    //ID do pino no PIO
#define IN_1_PIO_IDX_MASK (1 << IN_1_PIO_IDX) //mascara para ativar a fase 1 do motor
//fase 2
#define IN_2_PIO     PIOA    //Pino para a fase 2 do motor
#define IN_2_PIO_ID  ID_PIOA //ID do periferico PIOC
#define IN_2_PIO_IDX 6	    //ID do pino no PIO
#define IN_2_PIO_IDX_MASK (1 << IN_2_PIO_IDX) //mascara para ativar a fase 1 do motor
//fase 3
#define IN_3_PIO     PIOC    //Pino para a fase 3 do motor
#define IN_3_PIO_ID  ID_PIOC //ID do periferico PIOA
#define IN_3_PIO_IDX 19	    //ID do pino no PIO
#define IN_3_PIO_IDX_MASK (1 << IN_3_PIO_IDX) //mascara para ativar a fase 1 do motor
//fase 4
#define IN_4_PIO     PIOA    //Pino para a fase 4 do motor
#define IN_4_PIO_ID  ID_PIOA //ID do periferico PIOA
#define IN_4_PIO_IDX 2	    //ID do pino no PIO
#define IN_4_PIO_IDX_MASK (1 << IN_4_PIO_IDX) //mascara para ativar a fase 4 do motor

//_pio_set_input
/*  Default pin configuration (no attribute). */
#define PIO_DEFAULT             (0u << 0)
/*  The internal pin pull-up is active. */
#define PIO_PULLUP              (1u << 0)
/*  The internal glitch filter is active. */
#define PIO_DEGLITCH            (1u << 1)
/*  The internal debouncing filter is active. */
#define PIO_DEBOUNCE            (1u << 3)


/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_MODO (4096 / sizeof(portSTACK_TYPE))
#define TASK_MODO (tskIDLE_PRIORITY)

#define TASK_MOTOR (4096 / sizeof(portSTACK_TYPE))
#define TASK_MOTOR (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback(void);
static void BUT_init(void);
volatile int but_1_flag;

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/
//filas
QueueHandle_t xQueueModo;
QueueHandle_t xQueueSteps;

//semaforos
SemaphoreHandle_t xSemaphoreRTT;
SemaphoreHandle_t xSemaphoreBut;

typedef struct {
	uint value;
} adcData;


extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

	// BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	 //xSemaphoreGiveFromISR(xSemaphoreRTT, &xHigherPriorityTaskWoken);
void but_callback(void) {
	uint32_t modo = 01;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreBut, &xHigherPriorityTaskWoken);
	xQueueSend(xQueueModo, (void *)&modo, 10);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_modo(void *pvParameters) {
	gfx_mono_ssd1306_init();
  gfx_mono_draw_string("fuck", 0, 0, &sysfont);
  gfx_mono_draw_string("me", 0, 20, &sysfont);
  
  uint32_t msg = 0;
  uint32_t modo = 0;

	for (;;)  {
		
	/* verifica se chegou algum dado na queue, e espera por 0 ticks */
    if (xQueueReceive(xQueueModo, &msg, 0)) {
      /* chegou novo valor, atualiza delay ! */
      /* aqui eu poderia verificar se msg faz sentido (se esta no range certo)
       */
      /* converte ms -> ticks */
      modo = msg; // portTICK_PERIOD_MS;
		char text[5];
		sprintf(text, "%d", modo);
		gfx_mono_draw_string(text, 50,16, &sysfont);
    }
	
	if (xSemaphoreTake(xSemaphoreBut, 0)){
		char text[5];
		sprintf(text, "%d", 180);
		gfx_mono_draw_string(text, 50,16, &sysfont);
	}

    /* suspende por delayMs */
    //vTaskDelay(delayMs);
  }
}

static void task_motor(void *pvParameters) {
  
  uint32_t msg = 0;
  uint32_t modo = 0;
  adcData adc;
	for (;;)  {
		
		/* verifica se chegou algum dado na queue, e espera por 0 ticks */
    if (xQueueReceive(xQueueModo, &msg, (TickType_t) 0)) {
      /* chegou novo valor, atualiza delay ! */
      /* aqui eu poderia verificar se msg faz sentido (se esta no range certo)
       */
      /* converte ms -> ticks */
      modo = msg / portTICK_PERIOD_MS;
		char text[5];
		sprintf(text, "%d", modo);
		gfx_mono_draw_string(text, 50,16, &sysfont);
    }

    /* suspende por delayMs */
    //vTaskDelay(delayMs);
  }
}


  
/************************************************************************/
/* funcoes                                                              */
/************************************************************************/


static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

static void BUT_init(void) {
	/* configura prioridae */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);

	/* conf botão como entrada */
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	
	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback);
	
	// Ativa interrupção e limpa primeira IRQ gerada na ativacao	
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_get_interrupt_status(BUT1_PIO);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4); // Prioridade 4
	
	//Botoes EXT1
	//BUT1
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pio_set_input(BUT1_PIO, BUT1_PIO_IDX_MASK, PIO_DEFAULT);
	pio_pull_up(BUT1_PIO, BUT1_PIO_IDX_MASK, 1);
	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 60);
	
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_FALL_EDGE , but_callback);
	
	//BUT2
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pio_set_input(BUT2_PIO, BUT2_PIO_IDX_MASK, PIO_DEFAULT);
	pio_pull_up(BUT2_PIO, BUT2_PIO_IDX_MASK, 1);
	pio_set_debounce_filter(BUT2_PIO, BUT2_PIO_IDX_MASK, 60);
	
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_IDX_MASK, PIO_IT_FALL_EDGE , but_callback);
	//BUT3
	pmc_enable_periph_clk(BUT3_PIO_ID);
	pio_set_input(BUT3_PIO, BUT3_PIO_IDX_MASK, PIO_DEFAULT);
	pio_pull_up(BUT3_PIO, BUT3_PIO_IDX_MASK, 1);
	pio_set_debounce_filter(BUT3_PIO, BUT3_PIO_IDX_MASK, 60);
}

void motor_init(void){
	//IN 1
	pmc_enable_periph_clk(IN_1_PIO_ID);
	pio_set_output(IN_1_PIO, IN_1_PIO_IDX_MASK, 0, 0, 0);
	//IN 2
	pmc_enable_periph_clk(IN_2_PIO_ID);
	pio_set_output(IN_2_PIO, IN_2_PIO_IDX_MASK, 0, 0, 0);
	//IN 3
	pmc_enable_periph_clk(IN_3_PIO_ID);
	pio_set_output(IN_3_PIO, IN_3_PIO_IDX_MASK, 0, 0, 0);
	//IN 4
	pmc_enable_periph_clk(IN_4_PIO_ID);
	pio_set_output(IN_4_PIO, IN_4_PIO_IDX_MASK, 0, 0, 0);
}




/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	motor_init();
	BUT_init();

	/* Initialize the console uart */
	configure_console();

	/* Create task to control oled */
	/*
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create oled task\r\n");
	}
	*/
	
	
	xQueueModo = xQueueCreate(32, sizeof(uint32_t));
	if (xQueueModo == NULL){
		printf("falha em criar a queue xQueueADC \n");
	}
	
	xSemaphoreBut = xSemaphoreCreateBinary();
	if (xSemaphoreBut == NULL){
		printf("falha em criar o semaforo \n");
	}
	
	if (xTaskCreate(task_modo, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create oled task\r\n");
	}
	if (xTaskCreate(task_motor, "help", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create oled task\r\n");
	}


	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
