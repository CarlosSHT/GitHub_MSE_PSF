/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "lwip.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#include "arm_math.h"
#include "arm_common_tables.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define START_ID		1
#define NUM_SAMPLES		500
#define SAMPLE_FREQ		2000
#define MIDLEVEL_ADC	512
#define MIDLEVEL_ADC2	8

#define MAX_CLOCK_SYS	168000000
#define MAX_VAL_16BITS	65535


#define DEST_IP_ADDR0   192
#define DEST_IP_ADDR1   168
//#define DEST_IP_ADDR2   5
//#define DEST_IP_ADDR3   229
#define DEST_IP_ADDR2   60
#define DEST_IP_ADDR3   125

#define UDP_SERVER_PORT    61454   /* define the UDP local connection port */


typedef bool bool_t;
struct header_struct {
   char     head[4];
   uint32_t id;
   uint16_t N;
   uint16_t fs ;
   char     tail[4];
} header={"head",START_ID-1,NUM_SAMPLES,SAMPLE_FREQ,"tail"};


uint32_t h_maxIndex;
uint32_t h_minIndex;
q15_t    h_maxValue;
q15_t    h_minValue;
q15_t    h_rms;


typedef struct
{
	uint16_t	prescaler;
	uint16_t	period;
}mxTimConfs;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int cont=0;
mxTimConfs confs_var;
bool_t	irq_timer;
uint32_t medicion, adc_val;
bool_t	sampling_signal;


uint16_t sample;
uint8_t udp_buffer[sizeof(header)+NUM_SAMPLES*sizeof(uint16_t)];

extern struct netif gnetif;
bool_t notify_newip;
uint32_t curr_ipdev;
uint32_t prev_ipdev;
struct udp_pcb *upcb;
u8_t   data[100];
__IO uint32_t message_count = 0;

int16_t checktest;
q15_t asf=0x0000;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

mxTimConfs calculate_SettingsTimer(uint32_t freq);
void setOwner_TimerConfs(uint16_t prescaler, uint16_t period);
void udpApp_client_Initialization(void);
void udpApp_receive_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
void udpApp_client_send(char *data_msg, uint16_t datasize);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  int16_t adc[header.N];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_LWIP_Init();
  /* USER CODE BEGIN 2 */
  printf("Inicio programa ADC sending\n");
  irq_timer=false;
  adc_val=0;
  sample = 0;
  notify_newip=true;
  curr_ipdev=0;
  prev_ipdev=-1;
  sampling_signal=true;
  confs_var=calculate_SettingsTimer(header.fs);

  udpApp_client_Initialization();
  setOwner_TimerConfs(confs_var.prescaler-1, confs_var.period-1);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_ADC_Start_DMA(&hadc1, &medicion, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  MX_LWIP_Process();
	  if (irq_timer && sampling_signal) {
		  adc[sample] = (int16_t) (medicion & 0x03FF) - MIDLEVEL_ADC;
//		  adc[sample] = (int16_t) ((medicion & 0x03FF)>>6) - MIDLEVEL_ADC2;

		  checktest=adc[sample];
		  sample++;

		  if (sample==header.N) {
			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

			arm_max_q15 ( adc, header.N, &h_maxValue, &h_maxIndex );
			arm_min_q15 ( adc, header.N, &h_minValue, &h_minIndex );
			arm_rms_q15 ( adc, header.N, &h_rms);

			sample=0;
			header.id++;


			// Format 1
//			memcpy(&udp_buffer[0], &adc[0], sizeof(adc));
//			memcpy(&udp_buffer[sizeof(adc)], &header, sizeof(header));
//			udpApp_client_send((char*)udp_buffer, sizeof(adc)+sizeof(header));

			// Format 2
			memcpy(&udp_buffer[0], &header, sizeof(header));
			memcpy(&udp_buffer[sizeof(header)], &adc[0], sizeof(adc));
//			udpApp_client_send((char*)udp_buffer, sizeof(header));
			udpApp_client_send((char*)udp_buffer, sizeof(header)+ sizeof(adc));

//			printf("Se envia el array completa\n");
//			printf("Se envia el header completa\n");
		  }
		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		  adc_val=medicion;
		  irq_timer=false;
	  }
	  curr_ipdev=(uint32_t)netif_ip4_addr(&gnetif)->addr;
	  if (curr_ipdev!=prev_ipdev) {
		  printf("Nueva IP obtenida : %lu.%lu.%lu.%lu\n", curr_ipdev>>0 & 0xFF, curr_ipdev>>8 & 0xFF, curr_ipdev>>16 & 0xFF, curr_ipdev>>24 & 0xFF );
		  prev_ipdev=curr_ipdev;

	  }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

mxTimConfs calculate_SettingsTimer(uint32_t freq)
{
	mxTimConfs aux;
	uint32_t res, mod, div;
	int var;

	div=MAX_CLOCK_SYS/freq;
	mod=MAX_VAL_16BITS;
	for (var = MAX_VAL_16BITS; var >0; --var) {

		if (div%var==0) {
			res=div/var;
			break;
		}
		else {
			if (div%var<mod) {
				mod=div%var;
				res=div/var;
			}
		}
	}
	aux.prescaler = res;
	aux.period=var;
	return aux;
}

void setOwner_TimerConfs(uint16_t prescaler, uint16_t period)
{
	htim1.Init.Prescaler = prescaler;
	htim1.Init.Period = period;
	HAL_TIM_Base_Init(&htim1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim1 && !irq_timer)
  {
	  irq_timer=true;
  }
}


void udpApp_client_Initialization(void)
{
  struct ip4_addr DestIPaddr;
  err_t err;

  /* Create a new UDP control block  */
  upcb = udp_new();

  if (upcb!=NULL)
  {
    /*assign destination IP address */
    IP4_ADDR( &DestIPaddr, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2, DEST_IP_ADDR3 );

    /* configure destination IP address and port */
    err= udp_connect(upcb, &DestIPaddr, UDP_SERVER_PORT);

    if (err == ERR_OK)
    {
      /* Set a receive callback for the upcb */
      udp_recv(upcb, udpApp_receive_callback, NULL);
//      udpApp_client_send();
    }
	else
	{
		/* remove connection */
		udp_remove(upcb);
	}
  }
}

void udpApp_client_send(char *data_msg, uint16_t datasize)
{
  struct pbuf *p;

//  sprintf((char*)data, "sending udp client message %d", (int)message_count);

  /* allocate pbuf from pool*/
  p = pbuf_alloc(PBUF_TRANSPORT, datasize, PBUF_POOL);

  if (p != NULL)
  {
    /* copy data to pbuf */
    pbuf_take(p, data_msg, datasize);

    /* send udp data */
    udp_send(upcb, p);

    /* free pbuf */
    pbuf_free(p);
  }
}

void udpApp_receive_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	//  /*increment message count */
	  message_count++;
//  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  /* Free receive pbuf */
  pbuf_free(p);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_13) // If The INT Source Is EXTI Line9 (A9 Pin)
    {
    	sampling_signal=!sampling_signal;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
