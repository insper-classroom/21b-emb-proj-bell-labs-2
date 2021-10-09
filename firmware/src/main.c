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

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)
#define TASK_LCD_STACK_SIZE					(1024/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY	            (tskIDLE_PRIORITY)

/* AFEC */
#define AFEC_POT AFEC0
#define AFEC_POT_ID ID_AFEC0
#define AFEC_POT_CHANNEL 0 // Canal do pino PD30

/** The conversion data is done flag */
volatile bool g_is_conversion_done = false;

/** The conversion data value */
volatile uint32_t g_ul_value = 0;

typedef struct {
	uint value;
} adcData;

QueueHandle_t xQueueADC;
adcData adc;

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback(void);
static void BUT_init(void);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) {
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but_callback(void) {
}

static void AFEC_pot_Callback(void){
	g_ul_value = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL);
	/*BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(g_is_conversion_done, &xHigherPriorityTaskWoken);*/
	
	adcData adc;
	adc.value = g_ul_value;
	xQueueSendFromISR(xQueueADC, &adc, 0);
	g_is_conversion_done = true;
}

static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel, afec_callback_t callback){
  /*************************************
  * Ativa e configura AFEC
  *************************************/
  /* Ativa AFEC - 0 */
  afec_enable(afec);

  /* struct de configuracao do AFEC */
  struct afec_config afec_cfg;

  /* Carrega parametros padrao */
  afec_get_config_defaults(&afec_cfg);

  /* Configura AFEC */
  afec_init(afec, &afec_cfg);

  /* Configura trigger por software */
  afec_set_trigger(afec, AFEC_TRIG_SW);

  /*** Configuracao específica do canal AFEC ***/
  struct afec_ch_config afec_ch_cfg;
  afec_ch_get_config_defaults(&afec_ch_cfg);
  afec_ch_cfg.gain = AFEC_GAINVALUE_0;
  afec_ch_set_config(afec, afec_channel, &afec_ch_cfg);

  /*
  * Calibracao:
  * Because the internal ADC offset is 0x200, it should cancel it and shift
  down to 0.
  */
  afec_channel_set_analog_offset(afec, afec_channel, 0x200);

  /***  Configura sensor de temperatura ***/
  struct afec_temp_sensor_config afec_temp_sensor_cfg;

  afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
  afec_temp_sensor_set_config(afec, &afec_temp_sensor_cfg);
  
  /* configura IRQ */
  afec_set_callback(afec, afec_channel,	callback, 1);
  NVIC_SetPriority(afec_id, 4);
  NVIC_EnableIRQ(afec_id);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

void task_oled(void){
	gfx_mono_ssd1306_init();
	gfx_mono_draw_string("Exemplo RTOS", 0, 0, &sysfont);

	for (;;) {

		// Busca um novo valor na fila do ADC!
		// formata e imprime no LCD o dado
		// xQueueReceive( xQueueADC, &(adc), ( TickType_t )  100 / portTICK_PERIOD_MS)
		if (xQueueReceiveFromISR( xQueueADC, &(adc), ( TickType_t )  500 / portTICK_PERIOD_MS) == pdTRUE) {
			/*char b[10];
			sprintf(b, "%04d", adc.value);
			gfx_mono_draw_string(b, 50, 20, &sysfont);*/
			/*float width = (float)adc.value/max;
			gfx_mono_draw_filled_rect(0, 20, 128, 10, GFX_PIXEL_CLR);
			gfx_mono_draw_filled_rect(0, 20, 128*width, 10, GFX_PIXEL_SET);*/
			printf("%d\n", adc.value);
		}
	}
}

void task_adc(void){

	/* inicializa e configura adc */
	config_AFEC_pot(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL, AFEC_pot_Callback);

	/* Selecina canal e inicializa conversão */
	afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
	afec_start_software_conversion(AFEC_POT);
	
        
	/*if (g_is_conversion_done == NULL)
		printf("Falha em criar o semaforo\n");*/
	
	/*xQueueADC = xQueueCreate( 5, sizeof( adcData ));*/

	while(1){
		// xSemaphoreTake(g_is_conversion_done, ( TickType_t ) 500 / portTICK_PERIOD_MS) == pdTRUE
		if(g_is_conversion_done){
			g_is_conversion_done = 0;

			adc.value = g_ul_value;
			xQueueSend(xQueueADC, &adc, 0);

			vTaskDelay(( TickType_t ) (1/31025) / portTICK_PERIOD_MS);
			/*vTaskDelay(( TickType_t ) 1 / portTICK_PERIOD_MS);*/

			/* Selecina canal e inicializa conversão */
			afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
			afec_start_software_conversion(AFEC_POT);
		}
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
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_DEFAULT);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	configure_console();
	
	xQueueADC = xQueueCreate( 5, sizeof( adcData ));
	/*g_is_conversion_done = xSemaphoreCreateBinary();*/

	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create oled task\r\n");
	}
	
	/* Create task to handler LCD */
	if (xTaskCreate(task_adc, "adc", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test adc task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();
    
  /* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
