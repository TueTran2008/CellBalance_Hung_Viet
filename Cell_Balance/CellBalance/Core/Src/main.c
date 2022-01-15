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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_debug.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef void(*p_function)(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EPSILON_VALUE       5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */
//uint32_t Cell_Value[4] = {0};
uint16_t adc_value = 0;
uint16_t cell_raw_value[4] = {0};
uint8_t m_is_cell_need_discharge = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t Cell_Voltage_Read(void);

void findMinimumMaximum(uint16_t arr[], uint16_t N,uint16_t *MinVal, uint16_t *MaxVal)
{
    uint16_t i;
    uint16_t min = arr[0], max = arr[0];
    // Traverse the given array
    for (i = 0; i < N; i++)
    {
        // If current element is smaller
        // than min then update it
        if (arr[i] < min)
        {
            min = arr[i];
        }
        // If current element is greater
        // than max then update it
        if (arr[i] > max)
        {
            max = arr[i];
        }
    }
    *MinVal = min;
    *MaxVal = max;
}


void ADC_Select_Channel1()
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    DEBUG_ERROR("Cannot Config ADC \r\n");
    Error_Handler();
  }
}
  /** Configure Regular Channel
  */
void ADC_Select_Channel2()
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    DEBUG_ERROR("Cannot Config ADC \r\n");
    Error_Handler();
  }
}
  /** Configure Regular Channel
  */
void ADC_Select_Channel3()
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    DEBUG_ERROR("Cannot Config ADC \r\n");

    Error_Handler();
  }
}
void ADC_Select_Channel4()
{
  /** Configure Regular Channel
  */
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    DEBUG_ERROR("Cannot Config ADC \r\n");
    Error_Handler();
  }
}

p_function ADC_Array_Circle[4] = {ADC_Select_Channel1,ADC_Select_Channel2,ADC_Select_Channel3,ADC_Select_Channel4};

void ADC_Polling_Scan()
{
      for (uint8_t adc_channel_index = 0; adc_channel_index < 4; adc_channel_index++)
      {
          if(ADC_Array_Circle[adc_channel_index] != NULL)
          {
              ADC_Array_Circle[adc_channel_index]();
              HAL_ADC_Start(&hadc1);
              HAL_ADC_PollForConversion(&hadc1, 1000);
              cell_raw_value[adc_channel_index] = HAL_ADC_GetValue(&hadc1);
              DEBUG_INFO("ADC%d: %d\r\n",adc_channel_index + 1,cell_raw_value[adc_channel_index]);
              //HAL_Delay(200);
          }
          else
          {
              DEBUG_ERROR("Cannot Read ADC Channel\r\n");
              break;
          }
      } 
}


void DischargeCell1()
{
    HAL_GPIO_WritePin(Discharge_C1_GPIO_Port,Discharge_C1_Pin,0); /* Pull Low to Discharge*/
    HAL_Delay(1000);
    HAL_GPIO_WritePin(Discharge_C1_GPIO_Port,Discharge_C1_Pin,1);
}
void DischargeCell2()
{
    HAL_GPIO_WritePin(Dischage_C2_GPIO_Port,Dischage_C2_Pin,0); /* Pull Low to Discharge*/
    HAL_Delay(1000);
    HAL_GPIO_WritePin(Dischage_C2_GPIO_Port,Dischage_C2_Pin,1);
}
void DischargeCell3()
{
    HAL_GPIO_WritePin(Dischage_C3_GPIO_Port,Dischage_C3_Pin,0); /* Pull Low to Discharge*/
    HAL_Delay(1000);
    HAL_GPIO_WritePin(Dischage_C3_GPIO_Port,Dischage_C3_Pin,1);
}
void DischargeCell4()
{
    HAL_GPIO_WritePin(Dischage_C4_GPIO_Port,Dischage_C4_Pin,0); /* Pull Low to Discharge*/
    HAL_Delay(1000);
    HAL_GPIO_WritePin(Dischage_C4_GPIO_Port,Dischage_C4_Pin,1);
}


p_function Discharge_Cell_Arr[4] = {DischargeCell1,DischargeCell2,DischargeCell3,DischargeCell4};
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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
   app_debug_add_callback_print_function(SEGGER_RTT_Write);
   DEBUG_INFO("Balance Cells Build %s %s\r\n", __DATE__, __TIME__);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  uint32_t data[4] = {0};
  //HAL_ADC_Start_DMA(&hadc1,data,4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
     uint16_t CellMin = 0;
     uint16_t CellMax = 0;
     //uint8_t MaxCellIndex = 0;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    ADC_Polling_Scan();
    findMinimumMaximum(cell_raw_value,4,&CellMin,&CellMax);
    if( CellMax - CellMin > EPSILON_VALUE)
    {
        m_is_cell_need_discharge = 1;
    }
    if( m_is_cell_need_discharge )
    {
       uint8_t find_max_index;
       for(find_max_index = 0 ; find_max_index < 4; find_max_index++)
       {
           if(CellMax == cell_raw_value[find_max_index])
           {
               DEBUG_INFO("Cell %d's Voltage is the highest\r\n",find_max_index + 1);
               Discharge_Cell_Arr[find_max_index]();
               DEBUG_INFO("Cell %d was discharged\r\n",find_max_index + 1);
               break;
           }
       }
    }
    else
    {
        DEBUG_INFO("Cells are balance, No need to Discharge\r\n");
    }
    HAL_Delay(1000);
      
  
      //HAL_Delay(1000);
	 // HAL_ADC_Stop(&hadc1);
       //DEBUG_INFO("ADC1:%d\r\nADC2:%d\r\nADC3:%d\r\nADC4:%d\r\n",data[0],data[1],data[2],data[3]);
     // HAL_Delay(1000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
//  sConfig.Channel = ADC_CHANNEL_1;
//  sConfig.Rank = ADC_REGULAR_RANK_1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure Regular Channel
//  */
//  sConfig.Channel = ADC_CHANNEL_2;
//  sConfig.Rank = ADC_REGULAR_RANK_2;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure Regular Channel
//  */
//  sConfig.Channel = ADC_CHANNEL_3;
//  sConfig.Rank = ADC_REGULAR_RANK_3;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure Regular Channel
//  */
//  sConfig.Channel = ADC_CHANNEL_4;
//  sConfig.Rank = ADC_REGULAR_RANK_4;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Discharge_C1_Pin|Dischage_C2_Pin|Dischage_C3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Dischage_C4_GPIO_Port, Dischage_C4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : Discharge_C1_Pin Dischage_C2_Pin Dischage_C3_Pin */
  GPIO_InitStruct.Pin = Discharge_C1_Pin|Dischage_C2_Pin|Dischage_C3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Dischage_C4_Pin */
  GPIO_InitStruct.Pin = Dischage_C4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Dischage_C4_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */


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

