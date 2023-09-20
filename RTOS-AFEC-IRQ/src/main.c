#include "conf_board.h"
#include <asf.h>

/************************************************************************/
/* BOARD CONFIG                                                         */
/************************************************************************/

#define USART_COM_ID ID_USART1
#define USART_COM USART1

#define AFEC_POT AFEC0
#define AFEC_POT_ID ID_AFEC0
#define AFEC_POT_CHANNEL 0 // Canal do pino PD30

#define AFEC_POT2 AFEC1
#define AFEC_POT2_ID ID_AFEC1
#define AFEC_POT2_CHANNEL 6 // Canal do pino PD30

/************************************************************************/
/* RTOS                                                                */
/************************************************************************/

#define TASK_ADC_STACK_SIZE (1024 * 10 / sizeof(portSTACK_TYPE))
#define TASK_ADC_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_PROC_STACK_SIZE (1024 * 10 / sizeof(portSTACK_TYPE))
#define TASK_PROC_STACK_PRIORITY (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/************************************************************************/
/* recursos RTOS                                                        */
/************************************************************************/

TimerHandle_t xTimer;
TimerHandle_t xTimer2;

/** Queue for msg log send data */
QueueHandle_t xQueueADC;
QueueHandle_t xQueueADC2;
QueueHandle_t xQueueMean;

typedef struct {
  uint value;
} adcData;

/************************************************************************/
/* prototypes local                                                     */
/************************************************************************/

static void USART1_init(void);
static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel,
                            afec_callback_t callback);
static void configure_console(void);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName) {
  printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
  /* If the parameters have been corrupted then inspect pxCurrentTCB to
   * identify which task has overflowed its stack.
   */
  for (;;) {
  }
}

extern void vApplicationIdleHook(void) { pmc_sleep(SAM_PM_SMODE_SLEEP_WFI); }

extern void vApplicationTickHook(void) {}

extern void vApplicationMallocFailedHook(void) {
  /* Called if a call to pvPortMalloc() fails because there is insufficient
  free memory available in the FreeRTOS heap.  pvPortMalloc() is called
  internally by FreeRTOS API functions that create tasks, queues, software
  timers, and semaphores.  The size of the FreeRTOS heap is set by the
  configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

  /* Force an assert. */
  configASSERT((volatile void *)NULL);
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

static void AFEC_pot_callback(void) {
BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	  
  adcData adc;
  adc.value = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL);
  xQueueSendFromISR(xQueueADC, &adc, &xHigherPriorityTaskWoken);
  
  adcData adc2;
  adc2.value = afec_channel_get_value(AFEC_POT2, AFEC_POT2_CHANNEL);
  xQueueSendFromISR(xQueueADC2, &adc2, &xHigherPriorityTaskWoken);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

void vTimerCallback(TimerHandle_t xTimer) {
	
  afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
  afec_start_software_conversion(AFEC_POT);
  
  afec_channel_enable(AFEC_POT2, AFEC_POT2_CHANNEL);
  afec_start_software_conversion(AFEC_POT2);
  
}

static void task_proc(void *pvParameters){
	config_AFEC_pot(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL, AFEC_pot_callback);
	config_AFEC_pot(AFEC_POT2, AFEC_POT2_ID, AFEC_POT2_CHANNEL, AFEC_pot_callback);
	
	xTimer = xTimerCreate(
                        "Timer",
                        100,
                        pdTRUE,
                        (void *)0,
                        /* Timer callback */
                        vTimerCallback);
	xTimerStart(xTimer, 0);
	
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	adcData adc;
	adcData adc2;
	int mean;
	int mean2;
	int values[10];
	int values2[10];
	
	while (1){
	if (xQueueReceive(xQueueADC2, &(adc2), 1000)){
		if (xQueueReceive(xQueueADC, &(adc), 1000)){
			values2[9] = values2[8];
			values2[8] = values2[7];
			values2[7] = values2[6];
			values2[6] = values2[5];
			values2[5] = values2[4];
			values2[4] = values2[3];
			values2[3] = values2[2];
			values2[2] = values2[1];
			values2[1] = values2[0];
			values2[0] = adc2.value;
			int mean2 = (values2[0] + values2[1] + values2[2] + values2[3] + values2[4] + values2[5] + values2[6] + values2[7] + values2[8] + values2[9])/10;
			values[9] = values[8];
			values[8] = values[7];
			values[7] = values[6];
			values[6] = values[5];
			values[5] = values[4];
			values[4] = values[3];
			values[3] = values[2];
			values[2] = values[1];
			values[1] = values[0];
			values[0] = adc.value;
			int mean = (values[0] + values[1] + values[2] + values[3] + values[4] + values[5] + values[6] + values[7] + values[8] + values[9])/10;
			int mf = (mean + mean2)/2;
			xQueueSendFromISR(xQueueMean, &mf, &xHigherPriorityTaskWoken);
	}
	}
	}
}


static void task_adc(void *pvParameters) {
	int mean;
	while (1){
    if (xQueueReceive(xQueueMean, &(mean), 1000)) {
      printf("Media: %d \n", mean);
    }
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

/**
 * \brief Configure the console UART.
 */
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

static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel,
                            afec_callback_t callback) {
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
  afec_set_callback(afec, afec_channel, callback, 1);
  NVIC_SetPriority(afec_id, 4);
  NVIC_EnableIRQ(afec_id);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

/**
 *  \brief FreeRTOS Real Time Kernel example entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main(void) {
  sysclk_init();
  board_init();
  configure_console();

  xQueueADC = xQueueCreate(100, sizeof(adcData));
  if (xQueueADC == NULL)
    printf("falha em criar a queue xQueueADC \n");
	
  xQueueADC2 = xQueueCreate(100, sizeof(adcData));
  if (xQueueADC2 == NULL)
  printf("falha em criar a queue xQueueADC2 \n");
	
  xQueueMean = xQueueCreate(100, sizeof(int));
  if (xQueueMean == NULL)
  printf("falha em criar a queue xQueueMean \n");

  if (xTaskCreate(task_adc, "ADC", TASK_ADC_STACK_SIZE, NULL,
                  TASK_ADC_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create test ADC task\r\n");
  }
  
   if (xTaskCreate(task_proc, "Proc", TASK_PROC_STACK_SIZE, NULL,
   TASK_PROC_STACK_PRIORITY, NULL) != pdPASS) {
	   printf("Failed to create test Proc task\r\n");
   }

  vTaskStartScheduler();

  while (1) {
  }

  /* Will only get here if there was insufficient memory to create the idle
   * task. */
  return 0;
}
