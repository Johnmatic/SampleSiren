
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#define AUDIO_BUFFER_SIZE 4800
#define DAC_ADDRESS 0x94
#define DAC_REG_POWER 0x02
#define DAC_REG_SAI 0x06
#define DAC_REG_MSTAVOL 0x20
#define DAC_REG_MSTBVOL 0x21

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

SAI_HandleTypeDef hsai_BlockA1;
DMA_HandleTypeDef hdma_sai1_a;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint16_t SineTable [960] = {32768,32982,33196,33411,33625,33840,34054,34268, //Generated using
							34482,34697,34911,35125,35338,35552,35766,35979, //http://www.daycounter.com/Calculators/Sine-Generator-Calculator.phtml
							36193,36406,36619,36832,37045,37257,37469,37682, //Sine lookup table generated for 50Hz @48kHz sample rate
							37893,38105,38317,38528,38739,38950,39160,39370,
							39580,39790,39999,40208,40417,40625,40833,41041,
							41248,41455,41662,41868,42074,42279,42484,42689,
							42893,43097,43300,43503,43706,43907,44109,44310,
							44510,44710,44910,45109,45307,45505,45702,45899,
							46095,46291,46486,46680,46874,47068,47260,47452,
							47644,47834,48025,48214,48403,48591,48778,48965,
							49151,49337,49521,49705,49888,50071,50253,50434,
							50614,50793,50972,51150,51327,51504,51679,51854,
							52028,52201,52373,52545,52715,52885,53054,53222,
							53389,53555,53720,53885,54048,54211,54373,54533,
							54693,54852,55010,55167,55323,55478,55632,55785,
							55938,56089,56239,56388,56536,56683,56829,56974,
							57118,57261,57403,57544,57684,57823,57961,58097,
							58233,58367,58500,58633,58764,58894,59023,59150,
							59277,59402,59527,59650,59772,59893,60013,60131,
							60249,60365,60480,60594,60706,60818,60928,61037,
							61145,61252,61357,61461,61564,61666,61766,61866,
							61964,62060,62156,62250,62343,62435,62525,62614,
							62702,62789,62874,62958,63041,63122,63202,63281,
							63359,63435,63510,63583,63656,63726,63796,63864,
							63931,63997,64061,64124,64186,64246,64305,64362,
							64418,64473,64527,64579,64630,64679,64727,64774,
							64819,64863,64905,64947,64986,65025,65062,65097,
							65132,65164,65196,65226,65255,65282,65308,65332,
							65355,65377,65398,65416,65434,65450,65465,65478,
							65490,65501,65510,65517,65524,65529,65532,65534,
							65535,65534,65532,65529,65524,65517,65510,65501,
							65490,65478,65465,65450,65434,65416,65398,65377,
							65355,65332,65308,65282,65255,65226,65196,65164,
							65132,65097,65062,65025,64986,64947,64905,64863,
							64819,64774,64727,64679,64630,64579,64527,64473,
							64418,64362,64305,64246,64186,64124,64061,63997,
							63931,63864,63796,63726,63656,63583,63510,63435,
							63359,63281,63202,63122,63041,62958,62874,62789,
							62702,62614,62525,62435,62343,62250,62156,62060,
							61964,61866,61766,61666,61564,61461,61357,61252,
							61145,61037,60928,60818,60706,60594,60480,60365,
							60249,60131,60013,59893,59772,59650,59527,59402,
							59277,59150,59023,58894,58764,58633,58500,58367,
							58233,58097,57961,57823,57684,57544,57403,57261,
							57118,56974,56829,56683,56536,56388,56239,56089,
							55938,55785,55632,55478,55323,55167,55010,54852,
							54693,54533,54373,54211,54048,53885,53720,53555,
							53389,53222,53054,52885,52715,52545,52373,52201,
							52028,51854,51679,51504,51327,51150,50972,50793,
							50614,50434,50253,50071,49888,49705,49521,49337,
							49151,48965,48778,48591,48403,48214,48025,47834,
							47644,47452,47260,47068,46874,46680,46486,46291,
							46095,45899,45702,45505,45307,45109,44910,44710,
							44510,44310,44109,43907,43706,43503,43300,43097,
							42893,42689,42484,42279,42074,41868,41662,41455,
							41248,41041,40833,40625,40417,40208,39999,39790,
							39580,39370,39160,38950,38739,38528,38317,38105,
							37893,37682,37469,37257,37045,36832,36619,36406,
							36193,35979,35766,35552,35338,35125,34911,34697,
							34482,34268,34054,33840,33625,33411,33196,32982,
							32768,32553,32339,32124,31910,31695,31481,31267,
							31053,30838,30624,30410,30197,29983,29769,29556,
							29342,29129,28916,28703,28490,28278,28066,27853,
							27642,27430,27218,27007,26796,26585,26375,26165,
							25955,25745,25536,25327,25118,24910,24702,24494,
							24287,24080,23873,23667,23461,23256,23051,22846,
							22642,22438,22235,22032,21829,21628,21426,21225,
							21025,20825,20625,20426,20228,20030,19833,19636,
							19440,19244,19049,18855,18661,18467,18275,18083,
							17891,17701,17510,17321,17132,16944,16757,16570,
							16384,16198,16014,15830,15647,15464,15282,15101,
							14921,14742,14563,14385,14208,14031,13856,13681,
							13507,13334,13162,12990,12820,12650,12481,12313,
							12146,11980,11815,11650,11487,11324,11162,11002,
							10842,10683,10525,10368,10212,10057,9903,9750,
							9597,9446,9296,9147,8999,8852,8706,8561,
							8417,8274,8132,7991,7851,7712,7574,7438,
							7302,7168,7035,6902,6771,6641,6512,6385,
							6258,6133,6008,5885,5763,5642,5522,5404,
							5286,5170,5055,4941,4829,4717,4607,4498,
							4390,4283,4178,4074,3971,3869,3769,3669,
							3571,3475,3379,3285,3192,3100,3010,2921,
							2833,2746,2661,2577,2494,2413,2333,2254,
							2176,2100,2025,1952,1879,1809,1739,1671,
							1604,1538,1474,1411,1349,1289,1230,1173,
							1117,1062,1008,956,905,856,808,761,
							716,672,630,588,549,510,473,438,
							403,371,339,309,280,253,227,203,
							180,158,137,119,101,85,70,57,
							45,34,25,18,11,6,3,1,
							0,1,3,6,11,18,25,34,
							45,57,70,85,101,119,137,158,
							180,203,227,253,280,309,339,371,
							403,438,473,510,549,588,630,672,
							716,761,808,856,905,956,1008,1062,
							1117,1173,1230,1289,1349,1411,1474,1538,
							1604,1671,1739,1809,1879,1952,2025,2100,
							2176,2254,2333,2413,2494,2577,2661,2746,
							2833,2921,3010,3100,3192,3285,3379,3475,
							3571,3669,3769,3869,3971,4074,4178,4283,
							4390,4498,4607,4717,4829,4941,5055,5170,
							5286,5404,5522,5642,5763,5885,6008,6133,
							6258,6385,6512,6641,6771,6902,7035,7168,
							7302,7438,7574,7712,7851,7991,8132,8274,
							8417,8561,8706,8852,8999,9147,9296,9446,
							9597,9750,9903,10057,10212,10368,10525,10683,
							10842,11002,11162,11324,11487,11650,11815,11980,
							12146,12313,12481,12650,12820,12990,13162,13334,
							13507,13681,13856,14031,14208,14385,14563,14742,
							14921,15101,15282,15464,15647,15830,16014,16198,
							16384,16570,16757,16944,17132,17321,17510,17701,
							17891,18083,18275,18467,18661,18855,19049,19244,
							19440,19636,19833,20030,20228,20426,20625,20825,
							21025,21225,21426,21628,21829,22032,22235,22438,
							22642,22846,23051,23256,23461,23667,23873,24080,
							24287,24494,24702,24910,25118,25327,25536,25745,
							25955,26165,26375,26585,26796,27007,27218,27430,
							27642,27853,28066,28278,28490,28703,28916,29129,
							29342,29556,29769,29983,30197,30410,30624,30838,
							31053,31267,31481,31695,31910,32124,32339,32553};
uint16_t Pots[3];
int16_t AudioBuffer[AUDIO_BUFFER_SIZE];
int16_t BuffPointer = -1;
double OscClk = -1;
int CountUp = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SAI1_Init(void);
static void MX_TIM6_Init(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM6) {
		if (CountUp == 1) {
			OscClk += 0.1;
			if (OscClk > 1) CountUp = 0;
		}
		else {
			OscClk -= 0.1;
			if (OscClk < -1) CountUp = 1;
		}
	}
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef* hsai) {
	HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
	BuffPointer = 0;
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef* hsai) {
	HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	BuffPointer = AUDIO_BUFFER_SIZE / 2;
}

void HAL_SAI_ErrorCallback (SAI_HandleTypeDef* hsai) {
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_SAI1_Init();
  MX_TIM6_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)Pots, 3);

  uint8_t toSend = 0x00;

  //initDac();

  HAL_GPIO_WritePin(DAC_RESET_GPIO_Port, DAC_RESET_Pin, GPIO_PIN_SET);
  HAL_I2C_Mem_Read(&hi2c2, DAC_ADDRESS, DAC_REG_POWER, 1, &toSend, 1, 100);
  toSend = 0x99;
  HAL_I2C_Mem_Write(&hi2c2, DAC_ADDRESS, 0x00, 1, &toSend, 1, 100);
  toSend = 0x80;
  HAL_I2C_Mem_Write(&hi2c2, DAC_ADDRESS, 0x47, 1, &toSend, 1, 100);
  HAL_I2C_Mem_Read(&hi2c2, DAC_ADDRESS, 0x32, 1, &toSend, 1, 100);
  toSend = toSend | (1 << 7);
  HAL_I2C_Mem_Write(&hi2c2, DAC_ADDRESS, 0x32, 1, &toSend, 1, 100);
  toSend = toSend & ~(1 << 7);
  HAL_I2C_Mem_Write(&hi2c2, DAC_ADDRESS, 0x32, 1, &toSend, 1, 100);
  toSend = 0x00;
  HAL_I2C_Mem_Write(&hi2c2, DAC_ADDRESS, 0x00, 1, &toSend, 1, 100);
  toSend = 0xAF;
  HAL_I2C_Mem_Write(&hi2c2, DAC_ADDRESS, 0x04, 1, &toSend, 1, 100);
  toSend = 0x07;
  HAL_I2C_Mem_Write(&hi2c2, DAC_ADDRESS, DAC_REG_SAI, 1, &toSend, 1, 100);
  toSend = 0x9E;
  HAL_I2C_Mem_Write(&hi2c2, DAC_ADDRESS, DAC_REG_POWER, 1, &toSend, 1, 100);
  toSend = 0xD0;
  HAL_I2C_Mem_Write(&hi2c2, DAC_ADDRESS, DAC_REG_MSTAVOL, 1, &toSend, 1, 100); //protect your ears
  toSend = 0xD0;
  HAL_I2C_Mem_Write(&hi2c2, DAC_ADDRESS, DAC_REG_MSTBVOL, 1, &toSend, 1, 100); //both ears
  toSend = 0x00;

  for(int i = 0; i < AUDIO_BUFFER_SIZE; i++){ //init buffer
	  AudioBuffer[i] = 0;
  }

  if (HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *)AudioBuffer, AUDIO_BUFFER_SIZE) != HAL_OK) {
	  _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_Base_Start_IT(&htim6);

  int k = 0; //used to sync sine in buffer

  while (1)
  {
	  while(BuffPointer == -1);
	  int position = BuffPointer;
	  BuffPointer = -1;
	  double Freq; //50 - 870 +- osc
	  double osc; //0 - 20
	  double volume; //-1 - 1
	  osc = Pots[1] / 200;
	  Freq = (Pots[0] / 5) + 50 + (osc * OscClk);
	  volume = Pots[2] / 4096.0;

	  for (int i = 0; i < AUDIO_BUFFER_SIZE / 2; i++){
		  int16_t temp = volume * SineTable[(int)(959 * ((double)k / Freq))] - 32768;
		  AudioBuffer[i + position] = temp;
		  AudioBuffer[++i + position] = temp; //duplicate every sample for both channels
		  k++;
		  if (k > Freq) k = 0;
	  }

	  if (HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin) == GPIO_PIN_SET) {
		  toSend = 0xC0;
		  HAL_I2C_Mem_Write(&hi2c2, DAC_ADDRESS, 0x1E, 1, &toSend, 1, 100);
	 }

	  if(BuffPointer == -1) {
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	  }
	  //else HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 10;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI_PLLSAI;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 5;
  PeriphClkInitStruct.PLLSAIDivQ = 5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/8000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SAI1 init function */
static void MX_SAI1_Init(void)
{

  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_16;
  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_RISINGEDGE;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA1.FrameInit.FrameLength = 32;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 16;
  hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
  hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA1.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;
  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
  hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_16B;
  hsai_BlockA1.SlotInit.SlotNumber = 2;
  hsai_BlockA1.SlotInit.SlotActive = 0x0000FFFF;
  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 36000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PG14   ------> USART6_TX
     PE1   ------> FMC_NBL1
     PE0   ------> FMC_NBL0
     PB8   ------> I2C1_SCL
     PB3   ------> I2S3_CK
     PC12   ------> SDIO_CK
     PB9   ------> I2C1_SDA
     PB6   ------> QUADSPI_BK1_NCS
     PG15   ------> FMC_SDNCAS
     PD0   ------> FMC_D2_DA2
     PC11   ------> SDIO_D3
     PC10   ------> SDIO_D2
     PA12   ------> USB_OTG_FS_DP
     PI4   ------> FMC_NBL2
     PD1   ------> FMC_D3_DA3
     PI3   ------> FMC_D27
     PI2   ------> FMC_D26
     PA11   ------> USB_OTG_FS_DM
     PI5   ------> FMC_NBL3
     PI7   ------> FMC_D29
     PI10   ------> FMC_D31
     PI6   ------> FMC_D28
     PG9   ------> USART6_RX
     PD2   ------> SDIO_CMD
     PH15   ------> FMC_D23
     PI1   ------> FMC_D25
     PA10   ------> USB_OTG_FS_ID
     PF1   ------> FMC_A1
     PI9   ------> FMC_D30
     PH13   ------> FMC_D21
     PH14   ------> FMC_D22
     PI0   ------> FMC_D24
     PA9   ------> USB_OTG_FS_VBUS
     PC9   ------> SDIO_D1
     PF2   ------> FMC_A2
     PC8   ------> SDIO_D0
     PF3   ------> FMC_A3
     PG8   ------> FMC_SDCLK
     PF4   ------> FMC_A4
     PH3   ------> FMC_SDNE0
     PF7   ------> QUADSPI_BK1_IO2
     PF6   ------> QUADSPI_BK1_IO3
     PF5   ------> FMC_A5
     PH2   ------> FMC_SDCKE0
     PD15   ------> FMC_D1_DA1
     PD10   ------> FMC_D15_DA15
     PF10   ------> QUADSPI_CLK
     PF9   ------> QUADSPI_BK1_IO1
     PF8   ------> QUADSPI_BK1_IO0
     PD14   ------> FMC_D0_DA0
     PD9   ------> FMC_D14_DA14
     PD8   ------> FMC_D13_DA13
     PC0   ------> FMC_SDNWE
     PF12   ------> FMC_A6
     PG1   ------> FMC_A11
     PF15   ------> FMC_A9
     PD13   ------> S_TIM4_CH2
     PH12   ------> FMC_D20
     PF13   ------> FMC_A7
     PG0   ------> FMC_A10
     PE8   ------> FMC_D5_DA5
     PG5   ------> FMC_A15_BA1
     PG4   ------> FMC_A14_BA0
     PH9   ------> FMC_D17
     PH11   ------> FMC_D19
     PF14   ------> FMC_A8
     PJ2   ------> DSIHOST_TE
     PF11   ------> FMC_SDNRAS
     PE9   ------> FMC_D6_DA6
     PE11   ------> FMC_D8_DA8
     PE14   ------> FMC_D11_DA11
     PB10   ------> USART3_TX
     PH8   ------> FMC_D16
     PH10   ------> FMC_D18
     PE7   ------> FMC_D4_DA4
     PE10   ------> FMC_D7_DA7
     PE12   ------> FMC_D9_DA9
     PE15   ------> FMC_D12_DA12
     PE13   ------> FMC_D10_DA10
     PB11   ------> USART3_RX
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|DAC_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED3_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 DAC_RESET_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_3|DAC_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PG14 PG9 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PE1 PE0 PE8 PE9 
                           PE11 PE14 PE7 PE10 
                           PE12 PE15 PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_7|GPIO_PIN_10 
                          |GPIO_PIN_12|GPIO_PIN_15|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC12 PC11 PC10 PC9 
                           PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_11|GPIO_PIN_10|GPIO_PIN_9 
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QSPI;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG15 PG8 PG1 PG0 
                           PG5 PG4 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_8|GPIO_PIN_1|GPIO_PIN_0 
                          |GPIO_PIN_5|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD15 PD10 
                           PD14 PD9 PD8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_15|GPIO_PIN_10 
                          |GPIO_PIN_14|GPIO_PIN_9|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA12 PA11 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_11|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PI4 PI3 PI2 PI5 
                           PI7 PI10 PI6 PI1 
                           PI9 PI0 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_5 
                          |GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_6|GPIO_PIN_1 
                          |GPIO_PIN_9|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : LED3_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LED4_Pin */
  GPIO_InitStruct.Pin = LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PH15 PH13 PH14 PH3 
                           PH2 PH12 PH9 PH11 
                           PH8 PH10 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_3 
                          |GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_9|GPIO_PIN_11 
                          |GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PF1 PF2 PF3 PF4 
                           PF5 PF12 PF15 PF13 
                           PF14 PF11 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_12|GPIO_PIN_15|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PF7 PF6 PF10 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QSPI;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PF9 PF8 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QSPI;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PG2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PJ5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PJ2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DSI;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
