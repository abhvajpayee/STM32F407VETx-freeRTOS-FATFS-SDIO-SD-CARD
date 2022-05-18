/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 16,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#include "stdio.h"
#include "retarget.h"
FATFS fs;                 // Work area (file system object) for logical drive
FIL fil;                  // file objects

uint32_t byteswritten;                /* File write counts */
uint32_t bytesread;                   /* File read counts */
uint8_t wtext[] = "This is STM32 working with FatFs\r\n\0"; /* File write buffer */
uint8_t rtext[100];                     /* File read buffers */
char filename[] = "STM32cube.txt";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
HAL_SD_CardInfoTypeDef  SDCardInfo;
extern SD_HandleTypeDef hsd;

FATFS fs; // working space
FIL fil; //  Documentation items
void printf_sdcard_info(void)
{

	uint64_t CardCap;      	//SD Card capacity
	HAL_SD_CardCIDTypeDef SDCard_CID;

	HAL_SD_GetCardCID(&hsd,&SDCard_CID);	// obtain CID
	HAL_SD_GetCardInfo(&hsd,&SDCardInfo);                    // obtain SD Card information
	CardCap=(uint64_t)(SDCardInfo.LogBlockNbr)*(uint64_t)(SDCardInfo.LogBlockSize);	// Calculation SD Card capacity
	switch(SDCardInfo.CardType)
	{

		case CARD_SDSC:
		{

			if(SDCardInfo.CardVersion == CARD_V1_X)
				printf("Card Type:SDSC V1\r\n");
			else if(SDCardInfo.CardVersion == CARD_V2_X)
				printf("Card Type:SDSC V2\r\n");
		}
		break;
		case CARD_SDHC_SDXC:printf("Card Type:SDHC\r\n");break;
		default:break;
	}

    printf("Card ManufacturerID: %d \r\n",SDCard_CID.ManufacturerID);				// Manufacturer ID
 	printf("CardVersion: %ld \r\n",(uint32_t)(SDCardInfo.CardVersion));		// Card version number
	printf("Class: %ld \r\n",(uint32_t)(SDCardInfo.Class));		    //
 	printf("Card RCA(RelCardAdd):%ld \r\n",SDCardInfo.RelCardAdd);					// Card relative address
	printf("Card BlockNbr: %ld \r\n",SDCardInfo.BlockNbr);						// Number of pieces
 	printf("Card BlockSize: %ld \r\n",SDCardInfo.BlockSize);					// Block size
	printf("LogBlockNbr: %ld \r\n",(uint32_t)(SDCardInfo.LogBlockNbr));		// Number of logical blocks
	printf("LogBlockSize: %ld \r\n",(uint32_t)(SDCardInfo.LogBlockSize));		// Logical block size
	printf("Card Capacity: %ld MB\r\n",(uint32_t)(CardCap>>20));				// Card capacity
}

void InitFatFas(void)
{

    int retSD = f_mount(&fs, "", 0);
    if(retSD)
    {

        printf("mount error : %d \r\n",retSD);
      // Error_Handler();
    }
    else
    printf("mount sucess!!! \r\n");
}

void creat_file(char * filename)
{

    int retSD = f_open(&fil, filename, FA_CREATE_ALWAYS | FA_WRITE); // Open file , Permissions include creating 、 Write （ If there is no such document , The file will be created ）
    if(retSD==FR_OK) printf("\r\ncreater file sucess!!! \r\n");
    else printf("\r\ncreater file error : %d\r\n",retSD);
   // f_close(&fil); // Close the file
    //HAL_Delay(100);
}
void write_file(char * data,uint32_t len)
{

    uint32_t byteswritten;
    /*##-3- Write data to the text files ###############################*/
    int retSD = f_write(&fil, data, len, (void *)&byteswritten);
    if(retSD)
        printf(" write file error : %d\r\n",retSD);
    else
    {

        printf(" write file sucess!!! \r\n");
        printf(" write Data[%ld] : %s\r\n",byteswritten,data);
    }
    /*##-4- Close the open text files ################################*/
    retSD = f_close(&fil);
    if(retSD)
        printf(" close error : %d\r\n",retSD);
    else
        printf(" close sucess!!! \r\n");
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  BSP_SD_Init();

  RetargetInit(&huart3);
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 2;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	printf("\r\n ****** FatFs Example ******\r\n\r\n");
	printf_sdcard_info();

    /*##-1- Register the file system object to the FatFs module ##############*/
    retSD = f_mount(&fs, "/", 1);
    if(retSD)
    {
        printf(" mount error : %d \r\n",retSD);
        Error_Handler();
    }
    else{
  	  printf(" mount sucess!!! \r\n");
  	  //osDelay(100);
  	  //##-2- Create and Open new text file objects with write access ######
  	  retSD = f_open(&fil, filename, FA_CREATE_ALWAYS | FA_WRITE);
  	  if(retSD)
  		  printf(" open file error : %d\r\n",retSD);
  	  else{
  		  printf(" open file sucess!!! \r\n");
  		  //##-3- Write data to the text files ###############################
  		  retSD = f_write(&fil, wtext, strlen(wtext), (void *)&byteswritten);
  		  printf(" byteswritten: %d \r\n", byteswritten);
  		  if(retSD)
  			  printf(" write file error : %d\r\n",retSD);
  		  else
  		  {
  			  printf(" write file sucess!!! \r\n");
  			  printf(" write Data : %s\r\n",wtext);
  		  }
  		  //##-4- Close the open text files ################################
  		  retSD = f_close(&fil);
  		  if(retSD)
  			  printf(" close error : %d\r\n",retSD);
  		  else
  			  printf(" close sucess!!! \r\n");
  	  }

  	  //##-5- Open the text files object with read access ##############
  	  retSD = f_open(&fil, filename, FA_READ);
  	  if(retSD)
  		  printf(" open file error : %d\r\n",retSD);
  	  else{
  		  printf(" open file sucess!!! \r\n");
  		  //##-6- Read data from the text files ##########################
  		  retSD = f_read(&fil, rtext, sizeof(rtext), (UINT*)&bytesread);
  		  if(retSD)
  			  printf(" read error!!! %d\r\n",retSD);
  		  else
  		  {
  			  printf(" read sucess!!! \r\n");
  			  printf(" read Data : %s\r\n",rtext);
  		  }
  		  //##-7- Close the open text files ############################
  		  retSD = f_close(&fil);
  		  if(retSD)
  			  printf(" close error!!! %d\r\n",retSD);
  		  else
  			  printf(" close sucess!!! \r\n");
  	  }
  	  //##-8- Compare read data with the expected data ############
  	  if(bytesread == byteswritten)
  	  {
  		  printf(" FatFs is working well!!!\r\n");
  	  }
  	  retSD = f_mount(NULL, "/", 1);
  	  if (retSD == FR_OK) printf ("SD CARD UNMOUNTED successfully...\n\n\n");
  	  else printf("ERROR!!! in UNMOUNTING SD CARD\n\n\n");
    }
  for(;;)
  {
	  //HAL_UART_Transmit(&huart3, "#######\r\n", 9, 10);
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
	  osDelay(100);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
