/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
/*! \brief Wymagane do konwersji int na ASCII (itoa()) */
#include <stdlib.h>
/*! \brief Biblioteka potrzebna do operacji tekstowych */
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim1_ch1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/*! \brief Liczba znakow w buforze odbiorczym uwzględniająca znak końca tekstu '\0' */
#define BUFFER_SIZE (9u)

/*! \brief Typ wyliczeniowy opisujący stan odebranej komendy przez UART */
typedef enum
{
   eNotReady, /*!< Odebrana komenda nie jest gotowa do odczytu */
   eReady /*!< Odebrana komenda jest gotowa do odczytu */
}ReceivedCmdState_t;

typedef enum
{
   PwmChannel_eInvalid,
   PwmChannel_eCh1,
   PwmChannel_eCh2,
}PwmChannel_t;

typedef enum
{
   eDutyCycleValid,
   eDutyCycleInvalid_0,
   eDutyCycleInvalid_100
}DutyCycleState_t;

/* Struktura zawierajaca otrzymana komende i stan tej komendy */
struct
{
   uint8_t u8Buf[BUFFER_SIZE]; /*!< Bufor przechowujący odebrane i przetworzone znaki */
   uint8_t u8Idx; /*!< Indeks kolejnego elementu w buforze */
   ReceivedCmdState_t ReceivedCmdState; /*!< Stan odebranego polecenia */
}sRxCmd; /*!< Stworzenie struktury o nazwie sRxCmd */

/* Bufor przechowujący odebrane i przetworzone znaki */
uint8_t u8RxChar;

/* Duty cycle state */
volatile DutyCycleState_t DutyCycleStatePwmIn1;
DutyCycleState_t DutyCycleStatePwmIn2;

/* TIM1 (PWM1) duty cycle */
uint32_t u32Tim1DutyCycle = 0u;
/* TIM1 (PWM1) duty cycle */
uint32_t u32Tim2DutyCycle = 0u;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static uint8_t GetDutyCycle(void);
/* Reads duty cycle from input UART buffer and returns decimal value */
static PwmChannel_t GetPwmChannel(void);
/* Przesyla przez uart parametry wejsciowych PWM */
static void PrintInputPwmParameters(void);
/* Wysyla Period i duty cycle przez uart */
static void SendPerdiodAndDutyCycle(uint8_t u8Period, uint8_t u8DutyCycle);
/* Function updates duty cycle */
static void UpdateDutyCycle(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
   u32Tim1DutyCycle = 0u;  //u32 wymagane przez funkcje DMA
   u32Tim2DutyCycle = 0u;  //u32 wymagane przez funkcje DMA

   DutyCycleStatePwmIn1 = eDutyCycleInvalid_0;
   DutyCycleStatePwmIn2 = eDutyCycleInvalid_0;

   //Inicjalizacja zmiennych od odbieranych komend
   sRxCmd.u8Idx = 0u;
   sRxCmd.ReceivedCmdState = eNotReady;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
  /* Wlacz przerwanie od odebranego znaku z UART */
  HAL_UART_Receive_IT(&huart2, &u8RxChar, 1);
  /* Wlacz PWM wykorzystujacy DMA */
  HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, &u32Tim1DutyCycle, 1u);
  /* Wlacz PWM wykorzystujacy DMA */
  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, &u32Tim2DutyCycle, 1u);
  /* Wlacz odczyt parametrow PWM */
  //HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_1);
  //HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  /* Wlacz odczyt parametrow PWM */
  HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_2);
  /* Wlacz przerwanie dla TIM3 i TIM4 */
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
     //Jezeli odebrano komende
     if(sRxCmd.ReceivedCmdState == eReady)
     {
        //Sprawdz format koemndy "PWMx_xxx"
        if(strcmp((char *)sRxCmd.u8Buf, "PWM") > 1)
        {
           UpdateDutyCycle();
        }
        else if(strcmp((char*)sRxCmd.u8Buf, "GET") == 0)
        {
           PrintInputPwmParameters();
        }

        sRxCmd.ReceivedCmdState = eNotReady;
        sRxCmd.u8Idx = 0u;
     }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 4200-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_IC_InitTypeDef sConfigIC;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 42000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim3, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_IC_InitTypeDef sConfigIC;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 42000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim4, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static uint8_t GetDutyCycle(void)
{
   uint8_t u8DutyCycle = 0u;

   /* PWMX_X */
   if ((sRxCmd.u8Buf[5u] >= '0') && (sRxCmd.u8Buf[5u] <= '9'))
   {
      u8DutyCycle = sRxCmd.u8Buf[5u] - '0';

      /* PWMX_XX */
      if ((sRxCmd.u8Buf[6u] >= '0') && (sRxCmd.u8Buf[6u] <= '9'))
      {
         u8DutyCycle *= 10u;
         u8DutyCycle += sRxCmd.u8Buf[6u] - '0';

         /* PWMX_XXX */
         if ((sRxCmd.u8Buf[7u] >= '0') && (sRxCmd.u8Buf[7u] <= '9'))
         {
            u8DutyCycle *= 10u;
            u8DutyCycle += sRxCmd.u8Buf[7u] - '0';
         }
      }

      /* Ograniczenie maksymalnego duty cycle w przypadku gdy odebrane jest > 100 */
      if (u8DutyCycle > 100u)
      {
         u8DutyCycle = 100u;
      }
   }

   return u8DutyCycle;
}

static PwmChannel_t GetPwmChannel(void)
{
   PwmChannel_t ePwmChannel;

   switch(sRxCmd.u8Buf[3u])
   {
      case '1':
         ePwmChannel = PwmChannel_eCh1;
         break;
      case '2':
         ePwmChannel = PwmChannel_eCh2;
         break;
      default:
         ePwmChannel = PwmChannel_eInvalid;
   }

   return ePwmChannel;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
   if(TIM3 == htim->Instance)
   {
      DutyCycleStatePwmIn1 = eDutyCycleValid;
   }
   if(TIM4 == htim->Instance)
   {
      DutyCycleStatePwmIn2 = eDutyCycleValid;
   }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
   if(TIM3 == htim->Instance)
   {
      if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET)
      {
         DutyCycleStatePwmIn1 = eDutyCycleInvalid_100;
      }
      else
      {
         DutyCycleStatePwmIn1 = eDutyCycleInvalid_0;
      }
   }
//   if(TIM4 == htim->Instance)
//   {
//      if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET)
//      {
//         DutyCycleStatePwmIn2 = eDutyCycleInvalid_100;
//      }
//      else
//      {
//         DutyCycleStatePwmIn2 = eDutyCycleInvalid_0;
//      }
//   }
}

/*! \brief Funkcja wywolywana po odebraniu zaku z UART */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   //Jezeli odebrany znak to ktorys z konczacych znakow - zakoncz bufor i ustaw flage: "komenda odebrana i do sprawdzenia"
   if((u8RxChar == '\r') || (u8RxChar == '\n') || (u8RxChar == '\0'))
   {
      sRxCmd.u8Buf[sRxCmd.u8Idx] = '\0';   //Zakoncz
      sRxCmd.ReceivedCmdState = eReady; //Zmien stan bufora na: odebrano pelna komende
   }
   //Dodaj znak o bufora, zwieksz indeks bufora
   else
   {
      //Dodaj nowy znak do bufora
      sRxCmd.u8Buf[sRxCmd.u8Idx] = u8RxChar;

      //Jezeli bufor nie jest pelny to inkrementuj indeks
      if(sRxCmd.u8Idx <= (BUFFER_SIZE - 1))
      {
         sRxCmd.u8Idx++;
      }
   }

   //Wlacz nasluchiwanie na uart
   HAL_UART_Receive_IT(huart, &u8RxChar, 1u);
}

static void PrintInputPwmParameters(void)
{
   uint8_t u8DutyCycle;
   uint8_t u8Period;

   /* Zablokuj odbior sygnalow przez uart */
      HAL_UART_AbortReceive_IT(&huart2);

   /* PWM input on PA6 */
   switch(DutyCycleStatePwmIn1)
   {
      case eDutyCycleValid:
         u8Period = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1) + 1u;
         u8DutyCycle = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2) + 1u;
         break;
      case eDutyCycleInvalid_0:
         u8Period = 0;
         u8DutyCycle = 0;
         break;
      case eDutyCycleInvalid_100:
         u8Period = 0;
         u8DutyCycle = 100;
   }
   /* Period */
   HAL_UART_Transmit(&huart2, (uint8_t*)"PWM on PA6:\n", 13u, 1000u);
   SendPerdiodAndDutyCycle(u8Period, u8DutyCycle);

   /* PWM input on PB6 */
   switch(DutyCycleStatePwmIn2)
   {
      case eDutyCycleValid:
         u8Period = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1) + 1u;
         u8DutyCycle = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_2) + 1u;
         break;
      case eDutyCycleInvalid_0:
         u8Period = 0;
         u8DutyCycle = 0;
         break;
      case eDutyCycleInvalid_100:
         u8Period = 0;
         u8DutyCycle = 100;
   }
   /* Period */
   HAL_UART_Transmit(&huart2, (uint8_t*)"PWM on PB6:\n", 13u, 1000u);
   SendPerdiodAndDutyCycle(u8Period, u8DutyCycle);

   /* Wlacz przerwania */
   HAL_UART_Receive_IT(&huart2, &u8RxChar, 1u);
}

static void SendPerdiodAndDutyCycle(uint8_t u8Period, uint8_t u8DutyCycle)
{
   uint8_t u8TxtBuffer[6u];

   HAL_UART_Transmit(&huart2, (uint8_t*)"  Period [ms]: ", 15u, 1000u);
   itoa(u8Period, (char *)u8TxtBuffer, 10);
   HAL_UART_Transmit(&huart2, u8TxtBuffer, strlen((const char*)u8TxtBuffer), 1000u);
   HAL_UART_Transmit(&huart2, (uint8_t*)"\n", 1u, 1000u);

   /* Duty cycle */
   HAL_UART_Transmit(&huart2, (uint8_t*)"  Duty Cycle [ms]: ", 19u, 1000u);
   itoa(u8DutyCycle, (char *)u8TxtBuffer, 10);
   HAL_UART_Transmit(&huart2, u8TxtBuffer, strlen((const char*)u8TxtBuffer), 1000u);
   HAL_UART_Transmit(&huart2, (uint8_t*)"\n", 1u, 1000u);
}

static void UpdateDutyCycle(void)
{
   uint8_t u8DutyCycle;   HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);

   PwmChannel_t ePwmChannel;

   ePwmChannel = GetPwmChannel();
   u8DutyCycle = GetDutyCycle();

   switch(ePwmChannel)
   {
      case PwmChannel_eCh1:
         u32Tim1DutyCycle = (uint32_t)u8DutyCycle;
         break;
      case PwmChannel_eCh2:
         u32Tim2DutyCycle = (uint32_t)u8DutyCycle;
         break;
      default:
         break;
   }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
