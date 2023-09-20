#include "conf_board.h"
#include <asf.h>

/************************************************************************/
/* BOARD CONFIG                                                         */
/************************************************************************/

#define BUT_PIO PIOA
#define BUT_PIO_ID ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

#define BUT2_PIO PIOD
#define BUT2_PIO_ID ID_PIOD
#define BUT2_PIO_PIN 28
#define BUT2_PIO_PIN_MASK (1 << BUT2_PIO_PIN)

#define BUT4_PIO PIOA
#define BUT4_PIO_ID ID_PIOA
#define BUT4_PIO_PIN 19
#define BUT4_PIO_PIN_MASK (1 << BUT4_PIO_PIN)

#define BUT3_PIO PIOC
#define BUT3_PIO_ID ID_PIOC
#define BUT3_PIO_PIN 31
#define BUT3_PIO_PIN_MASK (1 << BUT3_PIO_PIN)

#define LED_PIO PIOC
#define LED_PIO_ID ID_PIOC
#define LED_PIO_IDX 8
#define LED_IDX_MASK (1 << LED_PIO_IDX)

#define USART_COM_ID ID_USART1
#define USART_COM USART1

/************************************************************************/
/* RTOS                                                                */
/************************************************************************/

#define TASK_LED_STACK_SIZE (4096 / sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY (tskIDLE_PRIORITY)
#define TASK_BUT_STACK_SIZE (4096 / sizeof(portSTACK_TYPE))
#define TASK_BUT_STACK_PRIORITY (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/************************************************************************/
/* recursos RTOS                                                        */
/************************************************************************/

/** Semaforo a ser usado pela task led */
QueueHandle_t xQueueBut;

/** Queue for msg log send data */
QueueHandle_t xQueueLedFreq;

/************************************************************************/
/* prototypes local                                                     */
/************************************************************************/

void but_callback(void);
void but2_callback(void);
void but3_callback(void);
void but4_callback(void);
static void BUT_init(void);
void pin_toggle(Pio *pio, uint32_t mask);
static void USART1_init(void);
void LED_init(int estado);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName) {
  printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
  /* If the parameters have been corrupted then inspect pxCurrentTCB to
   * identify which task has overflowed its stack.
   */
  for (;;) {
  }
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void) { pmc_sleep(SAM_PM_SMODE_SLEEP_WFI); }

/**
 * \brief This function is called by FreeRTOS each tick
 */
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

void but_callback(void) {
	uint32_t butID = 1;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(xQueueBut, (void *)&butID, &xHigherPriorityTaskWoken);
	printf("Callback %d", butID);
}

void but2_callback(void){
	uint32_t butID = 2;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(xQueueBut, (void *)&butID, &xHigherPriorityTaskWoken);
	printf("Callback %d", butID);
}

void but3_callback(void){
	uint32_t butID = 3;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(xQueueBut, (void *)&butID, &xHigherPriorityTaskWoken);
	printf("Callback %d", butID);
}

void but4_callback(void){
	uint32_t butID = 4;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(xQueueBut, (void *)&butID, &xHigherPriorityTaskWoken);
	printf("Callback %d", butID);
	}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_led(void *pvParameters) {

  LED_init(1);

  uint32_t msg = 0;
  uint32_t delayMs = 200;

  for (;;) {
    if (xQueueReceive(xQueueLedFreq, &msg, (TickType_t) 0)) {

      delayMs = msg / portTICK_PERIOD_MS;
      printf("task_led: %d \n", delayMs);
    }

    pin_toggle(LED_PIO, LED_IDX_MASK);

    vTaskDelay(delayMs);
  }
}

static void task_but(void *pvParameters) {

  BUT_init();

  uint32_t delayTicks = 200;
  uint32_t msgB = 0;

  for (;;) {
	  
    if (xQueueReceive(xQueueBut, &msgB, (TickType_t) 0)) {
		
      if (msgB == 1){
		  delayTicks += 10;
	  } else if (msgB == 2){
		  delayTicks -= 10;
	  } else if (msgB == 3){
		  delayTicks -= 5;
	  } else if (msgB == 4){
		  delayTicks -= 1;
	  }
	  
      if (delayTicks > 200) {
	      delayTicks = 100;
      }
      if (delayTicks < 50) {
	      delayTicks = 100;
      }

	 printf("task_but: %d \n", delayTicks);
	 xQueueSend(xQueueLedFreq, (void *)&delayTicks, 10);
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

void pin_toggle(Pio *pio, uint32_t mask) {
  if (pio_get_output_data_status(pio, mask))
    pio_clear(pio, mask);
  else
    pio_set(pio, mask);
}

void LED_init(int estado){
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_set_output(LED_PIO, LED_IDX_MASK, estado, 0, 0);
};


static void BUT_init(void) {
	
  pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
  pio_handler_set(BUT_PIO,
                  BUT_PIO_ID,
                  BUT_PIO_PIN_MASK,
                  PIO_IT_FALL_EDGE,
                  but_callback);
  pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
  pio_get_interrupt_status(BUT_PIO);
  NVIC_EnableIRQ(BUT_PIO_ID);
  NVIC_SetPriority(BUT_PIO_ID, 4);
  
  pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
  pio_handler_set(BUT2_PIO,
    BUT2_PIO_ID,
    BUT2_PIO_PIN_MASK,
    PIO_IT_FALL_EDGE,
    but2_callback);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_PIN_MASK);
	pio_get_interrupt_status(BUT2_PIO);
 NVIC_EnableIRQ(BUT2_PIO_ID);
 NVIC_SetPriority(BUT2_PIO_ID, 4); 

  pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
  pio_handler_set(BUT3_PIO,
  BUT3_PIO_ID,
  BUT3_PIO_PIN_MASK,
  PIO_IT_FALL_EDGE,
  but3_callback);
  pio_enable_interrupt(BUT3_PIO, BUT3_PIO_PIN_MASK);
  pio_get_interrupt_status(BUT3_PIO);
  NVIC_EnableIRQ(BUT3_PIO_ID);
  NVIC_SetPriority(BUT3_PIO_ID, 4);
  
  pio_configure(BUT4_PIO, PIO_INPUT, BUT4_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
  pio_handler_set(BUT4_PIO,
  BUT4_PIO_ID,
  BUT4_PIO_PIN_MASK,
  PIO_IT_FALL_EDGE,
  but4_callback);
  pio_enable_interrupt(BUT4_PIO, BUT4_PIO_PIN_MASK);
  pio_get_interrupt_status(BUT4_PIO);
  NVIC_EnableIRQ(BUT4_PIO_ID);
  NVIC_SetPriority(BUT4_PIO_ID, 4);
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
  /* Initialize the SAM system */
  sysclk_init();
  board_init();
  configure_console();
	
  printf("Sys init ok \n");
  
  xQueueBut = xQueueCreate(32,sizeof(uint32_t));
  if (xQueueBut == NULL)
    printf("falha em criar o semaforo \n");

  /* cria queue com 32 "espacos" */
  /* cada espaço possui o tamanho de um inteiro*/
  xQueueLedFreq = xQueueCreate(32, sizeof(uint32_t));
  if (xQueueLedFreq == NULL)
    printf("falha em criar a queue \n");

  /* Create task to make led blink */
  if (xTaskCreate(task_led, "Led", TASK_LED_STACK_SIZE, NULL,
                  TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create test led task\r\n");
  } else {
     printf("task led created \r\n");
	  
  }
  /* Create task to monitor processor activity */
  if (xTaskCreate(task_but, "BUT", TASK_BUT_STACK_SIZE, NULL,
                  TASK_BUT_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create UartTx task\r\n");
  } else {
     printf("task led but \r\n");  
  }

  /* Start the scheduler. */
  vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
  while (1) {
  }

  /* Will only get here if there was insufficient memory to create the idle
   * task. */
  return 0;
}
