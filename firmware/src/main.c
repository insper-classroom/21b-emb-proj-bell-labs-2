#include <asf.h>
#include <string.h>
#include "conf_board.h"
#include <stdlib.h>

/* Defines */
#define TS 11000
#define SOUND_LEN TS*3

#define LED					PIOC
#define LED_ID				ID_PIOC
#define LED_IDX				8
#define LED_IDX_MASK		(1u << LED_IDX)

#define GATE				PIOA
#define GATE_ID				ID_PIOA
#define GATE_IDX			6
#define GATE_IDX_MASK		(1u << GATE_IDX)

#define BUT_PIO				PIOA
#define BUT_PIO_ID			ID_PIOA
#define BUT_PIO_PIN			11
#define BUT_PIO_PIN_MASK	(1 << BUT_PIO_PIN)

#define DEBUG_SERIAL

#ifdef DEBUG_SERIAL
#define USART_COM USART1
#define USART_COM_ID ID_USART1
#else
#define USART_COM USART0
#define USART_COM_ID ID_USART0
#endif

/* RTOS  */
#define TASK_LCD_STACK_SIZE					(1024/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY	            (tskIDLE_PRIORITY)

#define TASK_BLUETOOTH_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_BLUETOOTH_STACK_PRIORITY        (tskIDLE_PRIORITY)

/* AFEC */
#define AFEC_POT AFEC0
#define AFEC_POT_ID ID_AFEC0
#define AFEC_POT_CHANNEL 0 // Canal do pino PD30

SemaphoreHandle_t xSemaphore;      

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

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

volatile uint32_t g_sdram_cnt = 0 ;
uint32_t *g_sdram = (uint32_t *)BOARD_SDRAM_ADDR;
volatile char but_flag = 0;
volatile int bluetooth_init = 0;

void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
		pio_clear(pio, mask);
	else
		pio_set(pio,mask);
}

void but_callback(void) {
	if(but_flag == 0) but_flag = 1;
	pin_toggle(LED, LED_IDX_MASK);
}

void TC0_Handler(void){    
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 0);
	printf("[Debug] TC0 IRQ \n");

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

}

static void AFEC_pot_Callback(void){
	if(but_flag == 1) {
		if(g_sdram_cnt < SOUND_LEN){
			// pin_toggle(LED, LED_IDX_MASK);
			*(g_sdram + g_sdram_cnt) = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL);
			g_sdram_cnt++;
		} else {
			pin_toggle(LED, LED_IDX_MASK);
			afec_disable_interrupt(AFEC_POT, AFEC_POT_CHANNEL);
			xSemaphoreGiveFromISR(xSemaphore, 0);
			g_sdram_cnt = 0;
		}
	}
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrup?c?o no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	
	//PMC->PMC_SCER = 1 << 14;
	ul_tcclks = 1;
	
	tc_init(TC, TC_CHANNEL, ul_tcclks 
							| TC_CMR_WAVE /* Waveform mode is enabled */
							| TC_CMR_ACPA_SET /* RA Compare Effect: set */
							| TC_CMR_ACPC_CLEAR /* RC Compare Effect: clear */
							| TC_CMR_CPCTRG /* UP mode with automatic trigger on RC Compare */
	);
	
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq /8 );
	tc_write_ra(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq / 8 / 2);

	tc_start(TC, TC_CHANNEL);
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
	afec_set_trigger(AFEC0, AFEC_TRIG_TIO_CH_0);
	AFEC0->AFEC_MR |= 3;

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

void task_adc(void){        
	while(1){
		if( xSemaphoreTake(xSemaphore, 500 / portTICK_PERIOD_MS) == pdTRUE ){
			printf("A\n");
			taskENTER_CRITICAL();
			for(uint32_t i =0; i< SOUND_LEN; i++) {
				printf("%d\n", *(g_sdram + i));
			}
			taskEXIT_CRITICAL();
			printf("X\n");
			vTaskDelay(500 / portTICK_PERIOD_MS);
			but_flag = 0;
			afec_enable_interrupt(AFEC_POT, AFEC_POT_CHANNEL);
		}
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
		#endif
		.paritytype = CONF_UART_PARITY,
		#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
		#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	#if defined(__GNUC__)
	setbuf(stdout, NULL);
	#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	* emits one character at a time.
	*/
	#endif
}

static void init(void) {
	sysclk_init();
	board_init();
	configure_console();
	
	/* inicializa e configura adc */
	config_AFEC_pot(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL, AFEC_pot_Callback);

	/* Selecina canal e inicializa conversão */
	afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
	TC_init(TC0, ID_TC0, 0, TS);
	
	/*pmc_enable_periph_clk(GATE_ID);
	pio_configure(GATE, PIO_INPUT, GATE_IDX_MASK, PIO_DEFAULT);*/
	
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);

	/* conf botão como entrada */
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_DEFAULT);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback);
	
	pmc_enable_periph_clk(LED_ID);
	pio_configure(LED, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);
	pin_toggle(LED, LED_IDX_MASK);
	
	xSemaphore = xSemaphoreCreateBinary();
	
	/* Complete SDRAM configuration */
	pmc_enable_periph_clk(ID_SDRAMC);
	sdramc_init((sdramc_memory_dev_t *)&SDRAM_ISSI_IS42S16100E,	sysclk_get_cpu_hz());
	sdram_enable_unaligned_support();
	SCB_CleanInvalidateDCache();
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	
	/* Initialize the SAM system */
	init();
	
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
