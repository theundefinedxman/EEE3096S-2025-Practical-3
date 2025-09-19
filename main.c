/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAX_ITER 100
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//TODO: Define variables you think you might need
// - Performance timing variables (e.g execution time, throughput, pixels per second, clock cycles)
#define SCALE 1000000LL // Fixed-point scale factor (S = 10^6)
#define SCALE 10000LL // Fixed-point scale factor (S = 10^4) // Task 7
#define SCALE 1000LL // Fixed-point scale factor (S = 10^3) // Task 7
#define MAX_MEASUREMENTS 5  // 5 iterations × 5 different test cases // Task 8

uint16_t image_Dimensions[5] = {128,160,192,224,256}; // Task 1,Task 5
uint16_t max_iterators[5] = {100,250,500,750,1000}; //Task 2
uint16_t image_Dimensions_Task_4_width[5] = {320,416,512,640,704,832,960,1024,1152,1216,1920}; // Task 3
uint16_t image_Dimensions_Task_4_height[5] = {320,416,512,640,704,832,960,1024,1080,1080,1080}; // Task 3

uint32_t start_time = 0;
uint32_t end_time = 0;
uint32_t execution_time = 0;
uint64_t checksum = 0;

int width  = 0;
int height = 0;
int maxIter = 0;

uint32_t cycles = 0;

// Task 8 Structure for organized power measurement data
typedef struct {
    int test_case_id;
    int max_iterations;
    int image_width;
    int image_height;
    uint32_t execution_time_ms;
    float power_consumption_mW;
    float current_consumption_mA;
    float supply_voltage_V;
    uint64_t checksum;
    uint32_t cpu_cycles;
    float pixels_per_second;
    uint32_t timestamp_ms;
} PowerMeasurement_t;

// Task 8 Power measurement storage arrays
float power_measurements_mW[MAX_MEASUREMENTS];
float current_measurements_mA[MAX_MEASUREMENTS];
uint32_t voltage_measurements_mV[MAX_MEASUREMENTS];
uint32_t measurement_timestamps[MAX_MEASUREMENTS];
int measurement_count = 0;

// Task 8 Structured power measurement data
PowerMeasurement_t power_data[MAX_MEASUREMENTS];
int power_data_count = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
//TODO: Define any function prototypes you might need such as the calculate Mandelbrot function among others
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_float(int width, int height, int max_iterations); 
void measure_and_store_power(int test_case_id, int iter_index); // Task 8
void get_measurement_summary(void); // Task 8
float read_current_sensor(void); // Task 8
void reset_measurement_arrays(void); // Task 8
void store_power_measurement(int test_id, int max_iter, int w, int h,
                           uint32_t exec_time, float power, float current,
                           uint64_t cs, uint32_t cycles); // Task 8
void access_power_data_arrays(void); // Task 8
float estimate_power_consumption(uint32_t cpu_cycles, int max_iter); // Task 8


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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  // Task 8 
   // for(int i = 0; i < 5; i++){
   //        // Visual indicator: Turn on LED0 to signal processing start
   //        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

   //        // Start power measurement trigger (for external measurement)
   //        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

   //        // Benchmark and Profile Performance
   //        start_time = HAL_GetTick();

   //        // Visual indicator: Turn on LED1 to signal processing start
   //        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

   //        // Calculate Mandelbrot and get checksum
   //        checksum = calculate_mandelbrot_fixed_point_arithmetic(width, height, maxIter[i]);

   //        // End timing measurement
   //        end_time = HAL_GetTick();
   //        execution_time = end_time - start_time;

   //        // End power measurement trigger
   //        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

   //        // Measure and store power consumption data
   //        measure_and_store_power(0, i); // Test case 0, iteration i

   //        // Keep the LEDs ON for 2s
   //        HAL_Delay(2000);

   //        // Turn OFF LEDs
   //        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);
   //    }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //TODO: Visual indicator: Turn on LED0 to signal processing start
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // Turn on LED0 by setting pin.

	  //TODO: Benchmark and Profile Performance
      start_time = HAL_GetTick(); //Record the start time
	  //Task 1
	  checksum  =  calculate_mandelbrot_fixed_point_arithmetic(image_Dimensions[5],image_Dimensions[5],100);
	  //checksum  =  calculate_mandelbrot_double(image_Dimensions[5],image_Dimensions[5],100);
	  //Task 2
	  checksum  =  calculate_mandelbrot_fixed_point_arithmetic(image_Dimensions[5],image_Dimensions[5],max_iterators[5]);
	  //checksum  =  calculate_mandelbrot_double(image_Dimensions[5],image_Dimensions[5],max_iterators[5]);  
	  //Task 3
	  //DWT_Init();
      //uint32_t start = DWT->CYCCNT;
	  checksum  =  calculate_mandelbrot_fixed_point_arithmetic(image_Dimensions[5],image_Dimensions[5],100);
	  //checksum  =  calculate_mandelbrot_double(image_Dimensions[5],image_Dimensions[5],100);
	  //uint32_t end = DWT->CYCCNT;
      //uint32_t cycles = end - start
      //end_time = HAL_GetTick();
	  //cycles = end - start;
	  //Task 4
	  checksum  =  calculate_mandelbrot_fixed_point_arithmetic(image_Dimensions_Task_4_width[5],image_Dimensions_Task_4_height[5],100);
	  //checksum  =  calculate_mandelbrot_double(image_Dimensions_Task_4_width[5],image_Dimensions_Task_4_height[5],100);
	  //Task 5
	  checksum = calculate_mandelbrot_float(image_Dimensions[5],image_Dimensions[5],100);
	  //checksum = calculate_mandelbrot_double(image_Dimensions[5],image_Dimensions[5],100);
	  //Task 6
	  checksum  =  calculate_mandelbrot_fixed_point_arithmetic(image_Dimensions[5],image_Dimensions[5],100);
	  //checksum  =  calculate_mandelbrot_double(image_Dimensions[5],image_Dimensions[5],100);
	  //Task 7
	  checksum  =  calculate_mandelbrot_fixed_point_arithmetic(image_Dimensions[5],image_Dimensions[5],100);
	  //TODO: Visual indicator: Turn on LED1 to signal processing start
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // Turn on LED1 by setting pin.


	  //TODO: Keep the LEDs ON for 2s
      HAL_Delay(2000);
	  // TODO: Turn OFF LEDs
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET); // Turning OFF LED's.



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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3
                           PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}
void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable DWT
    DWT->CYCCNT = 0;                                // Reset counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Enable counter
}

/* USER CODE BEGIN 4 */
//TODO: Function signatures you defined previously , implement them here
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations){
  uint64_t mandelbrot_sum = 0;
    //TODO: Complete the function implementation
    for(int y = 0 ; y < height; y++){
    	for(int x = 0 ; x < width ; x++ ){
    		int64_t x0 = (((int64_t)((x * 3500000LL) / width)) - 2500000LL); // Fixed-point scale factor (S = 10^6)
			//int64_t x0 = (((int64_t)((x * 35000LL) / width)) - 25000LL);    // Fixed-point scale factor (S = 10^4)
			//int64_t x0 = (((int64_t)((x * 3500LL) / width)) - 2500LL);      // Fixed-point scale factor (S = 10^3)
    		int64_t y0 = (((int64_t)((y * 2000000LL) / height)) - 1000000LL); // Fixed-point scale factor (S = 10^6)
			//int64_t y0 = (((int64_t)((y * 2000LL) / height)) - 1000LL);     // Fixed-point scale factor (S = 10^4)
			//int64_t y0 = (((int64_t)((y * 2000LL) / height)) - 1000LL);     // Fixed-point scale factor (S = 10^3)
    		int64_t xi = 0; int64_t yi = 0;
    		int iteration = 0;
    		while((iteration <  max_iterations) && ((((xi * xi)/ SCALE) + ((yi * yi)/ SCALE)) <= 4000LL)){
    			int temp  = ((xi * xi)/ SCALE) - ((yi * yi)/ SCALE);
    			yi = ((2LL*(xi * yi)/ SCALE) + y0);
    			xi = temp + x0;

    			iteration++;
    		}
			while((iteration <  max_iterations) && ((((xi * xi)/ SCALE) + ((yi * yi)/ SCALE)) <= 4000LL)){
    			int temp  = ((xi * xi)/ SCALE) - ((yi * yi)/ SCALE);
    			yi = ((2LL*(xi * yi)/ SCALE) + y0);
    			xi = temp + x0;

    			iteration++;
    		}
			while((iteration <  max_iterations) && ((((xi * xi)/ SCALE) + ((yi * yi)/ SCALE)) <= 4000LL)){
    			int temp  = ((xi * xi)/ SCALE) - ((yi * yi)/ SCALE);
    			yi = ((2LL*(xi * yi)/ SCALE) + y0);
    			xi = temp + x0;

    			iteration++;
    		}
    		mandelbrot_sum += iteration;
    	}
    }
    return mandelbrot_sum;

}

uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations){
    uint64_t mandelbrot_sum = 0;
    //TODO: Complete the function implementation
    for(int y = 0 ; y < height ; y++){
    	for(int x = 0 ; x < width ; x++ ){
    		//double x0  = ((((double)(x / width)) * 3.5 ) - 2.5);
    		//double y0  = ((((double)(y / width)) * 2.0 ) - 1.0);
    		double x0 = ((double)x / (double)width) * 3.5 - 2.5;
    		double y0 = ((double)y / (double)height) * 2.0 - 1.0;
    		double xi = 0.0; double yi = 0.0;
    		double iteration =  0;
    		while((iteration < max_iterations) && (((xi * xi) + (yi * yi)) <= 4.0)){
    			double temp = ((xi * xi) - (yi * yi));
    			yi  = (2.0 * (xi * yi) + y0);
    			xi = (temp + x0);
    			iteration++;
    		}
    		mandelbrot_sum += iteration;
    	}
    }
    return mandelbrot_sum;
}
uint64_t calculate_mandelbrot_float(int width, int height, int max_iterations){
    uint64_t mandelbrot_sum = 0;
    //TODO: Complete the function implementation
    //everything  is in (double precision)
   /*we are going through the complex plane multiple times
   any coordinate left after that iteration falls under the mandelbrot set*/
    for(int y = 0 ; y < height ; y++){
    	for(int x = 0 ; x < width ; x++ ){
    		float x0  = ((((float)((float)x / (float)width)) * 3.5 ) - 2.5);// we are keeping the set in between [-2.5 ,1.0]
    		float y0  = ((((float)((float)y / (float)width)) * 2.0 ) - 1.0);//we are keeping or taking the cooridantes in between [-1,1]
    		float xi = 0.0; float yi = 0.0;
    		float iteration =  0;//keeping track of the number of times we have done iterations
    		while((iteration < max_iterations) && (((xi * xi) + (yi * yi)) <= 4.0)){
    			float temp = ((xi * xi) - (yi * yi));//This is the imaginary representation of Z^2
    			yi  = (2.0 * (xi * yi) + y0);//this is the real z^2
    			xi = (temp + x0);//this is the real representation of z^2 +c
    			iteration++;
    		}
    		mandelbrot_sum += iteration;
    	}
    }
    return mandelbrot_sum;
}
/ Power measurement function
void measure_and_store_power(int test_case_id, int iter_index) {
    if (measurement_count >= MAX_MEASUREMENTS) {
        return; // Array full
    }

    // Method 1: Using external ADC reading
    // Example with INA219 or similar current sensor
    float supply_voltage = 3.3f; // Assuming 3.3V supply
    float current_mA = read_current_sensor(); // Your implementation here
    float power_mW = supply_voltage * current_mA;

    // Method 2: Software estimation based on computational load
    uint32_t active_cycles = execution_time * (SystemCoreClock / 1000);
    float estimated_current_mA = 15.0f + (active_cycles / 10000000.0f) * 35.0f; // STM32F4 typical range
    float estimated_power_mW = supply_voltage * estimated_current_mA;

    // Store measurements in simple arrays
    power_measurements_mW[measurement_count] = estimated_power_mW;
    current_measurements_mA[measurement_count] = estimated_current_mA;
    voltage_measurements_mV[measurement_count] = (uint32_t)(supply_voltage * 1000);
    measurement_timestamps[measurement_count] = HAL_GetTick();

    measurement_count++;

    // Also store in structured format
    store_power_measurement(test_case_id, maxIter[iter_index], width, height,
                          execution_time, estimated_power_mW, estimated_current_mA,
                          checksum, active_cycles);
}

// Function to read current from external sensor (placeholder)
float read_current_sensor(void) {

    return 25.0f; // Example current in mA
}

// Function to get stored measurements (for debugger access)
void get_measurement_summary(void) {

}

// Reset measurement arrays for new test run
void reset_measurement_arrays(void) {
    measurement_count = 0;
    power_data_count = 0;
    for (int i = 0; i < MAX_MEASUREMENTS; i++) {
        power_measurements_mW[i] = 0.0f;
        current_measurements_mA[i] = 0.0f;
        voltage_measurements_mV[i] = 0;
        measurement_timestamps[i] = 0;
    }
}

// Function to store structured power data
void store_power_measurement(int test_id, int max_iter, int w, int h,
                           uint32_t exec_time, float power, float current,
                           uint64_t cs, uint32_t cycles) {
    if (power_data_count >= MAX_MEASUREMENTS) return;

    power_data[power_data_count].test_case_id = test_id;
    power_data[power_data_count].max_iterations = max_iter;
    power_data[power_data_count].image_width = w;
    power_data[power_data_count].image_height = h;
    power_data[power_data_count].execution_time_ms = exec_time;
    power_data[power_data_count].power_consumption_mW = power;
    power_data[power_data_count].current_consumption_mA = current;
    power_data[power_data_count].supply_voltage_V = 3.3f;
    power_data[power_data_count].checksum = cs;
    power_data[power_data_count].cpu_cycles = cycles;
    power_data[power_data_count].pixels_per_second = (float)(w * h * 1000) / exec_time;
    power_data[power_data_count].timestamp_ms = HAL_GetTick();

    power_data_count++;
}

// Function to access stored data for debugger inspection
void access_power_data_arrays(void) {

}

// Software-based power estimation
float estimate_power_consumption(uint32_t cpu_cycles, int max_iter) {
    // Base current consumption (idle)
    float base_current_mA = 15.0f; // STM32F4 typical idle current

    // Dynamic current based on computational load
    float load_factor = (float)max_iter / 1000.0f;
    float dynamic_current_mA = load_factor * 40.0f; // Max additional current

    float total_current_mA = base_current_mA + dynamic_current_mA;
    float power_mW = 3.3f * total_current_mA; // Assuming 3.3V supply

    return power_mW;
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
#ifdef USE_FULL_ASSERT
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

