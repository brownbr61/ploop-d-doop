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
ADC_HandleTypeDef hadc;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

struct LEDs {
  uint8_t red;
  uint8_t orange;
  uint8_t green;
  uint8_t blue;
  void (*set)(struct LEDs*);
} leds;

struct UART_INT {
  void (*transmit)(uint16_t);
} uart;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);

void setLEDs(struct LEDs* this) {
  GPIOC->ODR &= ((this->green << 9) | (this->orange << 8) | (this->blue << 7) | (this->red << 6));
  GPIOC->ODR |= ((this->green << 9) | (this->orange << 8) | (this->blue << 7) | (this->red << 6));
}

void initLEDs(struct LEDs* this) {
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  GPIOC->MODER &= ~(0xFF000);
  GPIOC->MODER |= 0x55000;
  this->red = 0;
  this->orange = 0;
  this->green = 0;
  this->blue = 0;
  this->set = &setLEDs;
}

void transmitValue(uint8_t val) {
  // Wait until our tx reg is ready
  volatile int wait = 1;
  while (wait)
    if (USART3->ISR & 0x80)
      wait = 0;
  // Write to tx reg
  USART3->TDR = val;
}

void transmit2bytes(uint16_t val) {
  transmitValue((uint8_t)(val >> 8));
  transmitValue((uint8_t)val);
}

void transmitChar(char c) {
    // Wait until our tx reg is ready
    volatile int wait = 1;
    while (wait)
        if (USART3->ISR & 0x80)
            wait = 0;
    // Write to tx reg
    USART3->TDR = c;
}

void transmitString(char* s) {
    while (*s != 0) {
        transmitChar(*s++);
    }
}

void initUart(struct UART_INT* this) {
  // Set PC10 and PC11 to alt fx mode
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  GPIOC->MODER &= ~(0xF00000);    // Clear
  GPIOC->MODER |= 0xA00000;   		 // 1010

  // Set PC10 to AF1 for USART3_TX/RX
  GPIOC->AFR[1] &= ~(0xFF00);   	 // Clear
  GPIOC->AFR[1] |= 0x1100;   		 // Set 11 and 10 to 000

  // Enable system clock for USART3
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

  // Set Baud rate
  uint64_t t_baud = 9600;
  uint64_t f_clk = SystemCoreClock;
  uint64_t brr = f_clk/t_baud;
  USART3->BRR &= ~(0xFFFFFFFF);   	 // Clear
  USART3->BRR |= brr;

  // Enable tx
  USART3->CR1 |= (1 << 3);

  // Enable rx
  USART3->CR1 |= (1 << 2);

  // Enable rx not-empty interrupt
  USART3->CR1 |= (1 << 5);

  // Enable USART
  USART3->CR1 |= (1 << 0);

  // Enable USART3 interrupt on NVIC
  NVIC_EnableIRQ(USART3_4_IRQn);
  NVIC_SetPriority(USART3_4_IRQn, 1);
  this->transmit = &transmit2bytes;
}


#include <stdint.h>
#include <stdbool.h>
#define NUM_MOTORS 2
#define NUM_SENSORS 3
#define Kp 1            // Proportional gain
#define Ki 1            // Integral gain
#define REDLED (6)
#define BLUELED (7)
#define ORANGELED (8)
#define GREENLED (9)


void stop(void);
void move(uint8_t direction);
void moveForward(void);
void moveRight(void);
void moveBackward(void);
void moveLeft(void);
void hokeyPokey(void);
void initSensors(void);

/* A struct representing the state of a single motor */
struct Motor {
    uint8_t id;                         // Which motor this object represents (1-4)
    int direction;                      // Direction of movement (<0 = reverse; >0 = forward)
    int16_t target_ticks;               // Aka desired speed target
    uint16_t num_ticks;                 // Aka motor speed
    uint16_t error;                     // Speed error signal
    uint16_t error_integral;            // Integrated error signal
    uint16_t error_distance;            // Distance error signal
    void (*spin)(struct Motor*, uint16_t, int);
    void (*stop)(struct Motor*);
    void (*setCycle)(struct Motor*, uint8_t);
    void (*correctError)(struct Motor*, uint64_t);

    // Pointers to the pins that control this motor (convenience variables)
    GPIO_TypeDef *pwmGpio;
    uint8_t pwm_in_pin;
    uint8_t pwm_alt_fxn_code;
    TIM_TypeDef *pwmTimer;

    GPIO_TypeDef *dirGpio;
    uint8_t dir_pin_A;
    uint8_t dir_pin_B;
};

// Order: left, right
struct Motor motors[NUM_MOTORS];
uint64_t encoderCounts[] = { 0, 0};

void initPWM(void);

// Initializes entire class and structure necessary for motion
void initMotion(void);

// Initializes all four motor structs
void initMotors(void);

// Initializes direction pins for all four motors
void initDirection(void);

// Sets up all PWM pins and direction signals to drive the H-Bridges
void initPWMs(void);

// Sets up GPIO inputs for encoder signals + sets up timer for speed check interrupts
void initEncoders(void);

// PI control code is called within a timer interrupt
void PI_update(struct Motor*, uint64_t);

// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycle(struct Motor*, uint8_t);

// Go at given speed
void spinMotor(struct Motor*, uint16_t, int);

// Don't go
void stopMotor(struct Motor*);


struct Filter {
  uint64_t fOut;
  uint8_t shft;
  uint64_t (*filter)(struct Filter*, uint16_t fIn);
};

void initFilter(struct Filter* this, 
                uint8_t shift, 
                uint64_t (*filterFunction)(struct Filter*, uint16_t), 
                uint16_t dIn);

uint64_t geometric(struct Filter*, uint16_t fIn);

struct Sensor {
  uint32_t pin;
  uint16_t (*read)(struct Sensor*);
};
struct Sensor sensors[NUM_SENSORS];

void initSensor(struct Sensor*, uint32_t);
uint16_t readSensor(struct Sensor*);

struct SensorData {
  struct Sensor sensor;
  struct Filter filter;
  uint64_t mean; // may need to create a comparator logic class
  uint8_t var;
  uint64_t (*get)(struct SensorData*);
  void (*setMean)(struct SensorData*);
  void (*setVar)(struct SensorData*);
  bool (*diverges)(struct SensorData*);
};

// todo: once we are done testing, can use initSensors instead of initSensorData
void initSensorData(struct SensorData*);
uint64_t getSensorData(struct SensorData*);
void setMean(struct SensorData*);
void setVar(struct SensorData*);
bool diverges(struct SensorData*);

/* USER CODE BEGIN PFP */

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
  initLEDs(&leds);
  initUart(&uart);
  initMotion();
  leds.red = 1;
  leds.set(&leds);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
  // transmitString("\033[2J\033[H");
  // transmitString("I am here!\r\n");

  /* USER CODE END 2 */
  // todo: comment this in after testing
  //initSensors();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  stop();
  moveForward();
  leds.blue = !leds.blue;
  leds.set(&leds);
  HAL_Delay(1000);
  stop();
  HAL_Delay(1000);
  while (1)
  {
    leds.blue = !leds.blue;
    leds.set(&leds);
    HAL_Delay(500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

  // TODO: comment this back in after testing
    // if (DRAG_MODE) {
    //     moveForward();
    // } else {
    //     moveForward();
    //     hokeyPokey();
    // }
}

/**
  * @brief This function handles EXTI line 2 and 3 interrupts.
  */
// void EXTI2_3_IRQHandler(void)
// {
//   /* USER CODE BEGIN EXTI2_3_IRQn 0 */
//   leds.green = !leds.green;
//   leds.set(&leds);

//   /* USER CODE END EXTI2_3_IRQn 0 */
//   HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
//   HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
//   /* USER CODE BEGIN EXTI2_3_IRQn 1 */

//   /* USER CODE END EXTI2_3_IRQn 1 */
// }

void calibrate_ADC_manual() {    
  // Ensure ADEN = 0
  if ((ADC1->CR & ADC_CR_ADEN) != 0) {
    // Clear ADEN
    ADC1->CR |= ADC_CR_ADDIS;
  }
  while ((ADC1->CR & ADC_CR_ADEN) != 0) { }
  // Clear DMAEN
  ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;
  
  // Launch calibration
  ADC1->CR |= ADC_CR_ADCAL;
  
  // Wait until ADCAL = 0
  while ((ADC1->CR & ADC_CR_ADCAL) != 0) {
  }
}

void initSensor(struct Sensor* this, uint32_t pinNum) {
  this->pin = pinNum;
  this->read = &readSensor;
   	 
  // Configure PC0 to analog mode
  GPIOC->MODER &= ~(0x3);   		 // Clear
  GPIOC->MODER |= 0x3;
  
  // Enable ADC1 in RCC
  RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

  /*** Configure ADC ***/
  // Set 8-bit resolution
  ADC1->CFGR1 |= (1 << 4);
    
  // Turn off hardware triggers
  ADC1->CFGR1 &= ~((1 << 10) | (1 << 11));
    
  // Turn on continuous conversion mode
  ADC1->CFGR1 |= (1 << 13);

  // short analogPin = (1 << 10) | (1 << 13) | (1 << 14);
  short analogPin = (1 << 10);

  ADC1->CHSELR &= ~(analogPin);
  ADC1->CHSELR |=  (analogPin);

  ADC1->CFGR1 &= ~((1 << 10) | (1 << 11));

  calibrate_ADC_manual();

  ADC1->CHSELR &= ~(analogPin);
  ADC1->CHSELR |= 1 << 10;
    
  // Ensure ADSTP, ADSTART, ADDIS are = 0
  ADC1->CR &= ~(1 << 4); //STP
  ADC1->CR &= ~(1 << 2); //START
  ADC1->CR &= ~(1 << 1); //DIS

  ADC1->CR |= 0x1;

  ADC1->CR |= 1 << 2;
}

uint16_t readSensor(struct Sensor* this) {
  ADC1->CHSELR |= 1 << this->pin;
  // for(volatile int i = 0; i < 2400; i++) {}
  uint16_t data = ADC1->DR;
  ADC1->CHSELR &= ~(1 << this->pin);
  return data;
}


void initSensorData(struct SensorData* this) {
  initSensor(&this->sensor, 0);
  initFilter(&this->filter, 8, &geometric, this->sensor.read(&this->sensor));
  this->get = &getSensorData;
  this->setMean = &setMean;
  this->setVar = &setVar;
  this->diverges = &diverges;
  this->setMean(this);
  this->mean = this->filter.fOut;
  this->setVar(this);
}

uint64_t getSensorData(struct SensorData* this) {
  return this->filter.filter(&this->filter,this->sensor.read(&this->sensor));
}

void setMean(struct SensorData* this) {
  for(uint16_t i = 0; i < (1 <<this->filter.shft); i++)
    this->filter.filter(&this->filter,this->sensor.read(&this->sensor));

  this->mean = this->filter.fOut << this->filter.shft;
  for(uint16_t i = 0; i < (1 << (this->filter.shft)); i++)
    this->mean += this->filter.filter(&this->filter,this->sensor.read(&this->sensor)) 
                - (this->mean >> this->filter.shft);
  this->mean = this->mean >> (this->filter.shft);
}

void setVar(struct SensorData* this) {
  for(uint16_t i = 0; i < (1 << this->filter.shft); i++)
    this->var += 
        this->filter.filter(&this->filter,this->sensor.read(&this->sensor))
        - this->mean;
  this->var = (this->var >> this->filter.shft);
}

bool diverges(struct SensorData* this) {
  return (this->filter.fOut - this->mean) > 1000;
}

uint64_t geometric(struct Filter* this, uint16_t fIn) {
  this->fOut += fIn - (this->fOut >> this->shft);
  return this->fOut;
}

void initFilter(struct Filter* this, 
                uint8_t shift, 
                uint64_t (*filterFunction)(struct Filter*, uint16_t), 
                uint16_t dIn) {
  this->shft = shift;
  this->fOut = 193 << shift;
  this->filter = filterFunction;
}

void fillDirPins(int *allPins);
// Sets up the entire motor drive system
void initMotion() {
    initMotors();
    initPWM();
    initEncoders();
}

/* Initializes all motors in the forward direction in a stopped state */
void initMotors() {
    int pwm_in_pins[] = {0, 1};
    int mtr_A_dir_pins[] = {4, 5};
    int mtr_B_dir_pins[] = {6, 7};

    for (int i = 0; i < NUM_MOTORS; i++) {
        struct Motor *motor = &(motors[i]);
        motor->id = i + 1; // Range is 1-2
        motor->direction = 1;
        motor->target_ticks = 100;
        motor->num_ticks = 0;
        motor->error = 0;
        motor->error_integral = 0;

        motor->setCycle = &pwm_setDutyCycle;
        motor->correctError = &PI_update;
        motor->spin = &spinMotor;
        motor->stop = &stopMotor;

        motor->pwmGpio = GPIOA;
        motor->pwm_in_pin = pwm_in_pins[i];

        motor->dirGpio = GPIOB;
        switch (i) {
            case 0:
                motor->dir_pin_A = mtr_A_dir_pins[0];
                motor->dir_pin_B = mtr_A_dir_pins[1];
                break;
            case 1:
                motor->dir_pin_A = mtr_B_dir_pins[0];
                motor->dir_pin_B = mtr_B_dir_pins[1];
                break;
        }
    }
}

// Sets up the PWM and direction signals to drive the H-Bridge
void initPWM() {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

    // Set up pin PA0-1 for H-bridge PWM output (TIMER 2 CH1-2)
    GPIOA->MODER |= (1 << 1);
    GPIOA->MODER &= ~(1 << 0);
    GPIOA->MODER |= (1 << 3);
    GPIOA->MODER &= ~(1 << 2);

    // Set PA0-1 to AF1
    GPIOA->AFR[0] &= 0xFFFFFF00; // clear PA4 bits,
    GPIOA->AFR[0] |= (1 << 1);
    GPIOA->AFR[0] |= (1 << 5);

    // Set up GPIO output pins for motor direction control
    int allDirPins[4] = {4, 5, 6, 7};
    GPIOB->MODER &= (0xFF << 4);    // Clear
    for (int i = 0; i < (NUM_MOTORS * 2); i++) {
        int pinIdx = allDirPins[i] * 2;
        GPIOB->MODER &= ~(11 << pinIdx);  // Clear
        GPIOB->MODER |= (1 << pinIdx);    // Assign
    }

    // For each motor, initialize one direction pin to high, the other low
    for (int i = 0; i < (NUM_MOTORS*2); i++) {
        int pinIdx = allDirPins[i];
        // Set pins in even indices high, odd indices low
        if (i%2 == 0) {
            GPIOB->ODR |= (1 << pinIdx);
        } else {
            GPIOB->ODR &= ~(1 << pinIdx);
        }
    }

    // Set up PWM timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->CR1 = 0;                          // Clears; defaults to edge-aligned upcounting
    TIM2->CCMR1 = 0;                        // (prevents having to manually clear bits)
    TIM2->CCER = 0;

    // Set output-compare CH1-4 to PWM1 mode and enable CCR1 preload buffer
    TIM2->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1); //| TIM_CCMR1_OC1PE); // Enable channel 1
    TIM2->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1); //| TIM_CCMR1_OC2PE); // Enable channel 2

    TIM2->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);        // Enable capture-compare channel 1-2
    TIM2->PSC = 1;                         // Run timer on 24Mhz
    TIM2->ARR = 2400;                      // PWM at 20kHz

    TIM2->CCR1 = 2000;                        // Start PWMs at 0% duty cycle
    TIM2->CCR2 = 2000;

    TIM2->CR1 |= TIM_CR1_CEN;              // Enable timer

    // DEBUGGING
    //TIM2->EGR |= 1; // todo: forces register update
}

/* Sets up four GPIO pins for inputs and enable interrupts on rising and falling edge of encoder wave.
 * Sets up timer for speed calculation and PI updates. */
void initEncoders() {
    // Enable EXTI in NVIC
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    // GPIOB2-3
    // Clear to input mode
    GPIOB->MODER &= ~(1 << 5);
    GPIOB->MODER &= ~(1 << 4);
    GPIOB->MODER &= ~(1 << 7);
    GPIOB->MODER &= ~(1 << 6);

    // Clear to low speed
    GPIOB->OSPEEDR &= ~(1 << 5);
    GPIOB->OSPEEDR &= ~(1 << 4);
    GPIOB->OSPEEDR &= ~(1 << 7);
    GPIOB->OSPEEDR &= ~(1 << 6);

    // Set up pull-up resistor
    GPIOB->PUPDR &= ~(1 << 5);
    GPIOB->PUPDR &= ~(1 << 4);
    GPIOB->PUPDR |= (1 << 4);
    GPIOB->PUPDR &= ~(1 << 5);
    GPIOB->PUPDR &= ~(1 << 4);
    GPIOB->PUPDR |= (1 << 4);

    // Turn on interrupts
    EXTI->IMR &= ~(1 << 2);
    EXTI->IMR |= (1 << 2);
    EXTI->IMR &= ~(1 << 3);
    EXTI->IMR |= (1 << 3);

    // Turn on rising trigger
    EXTI->RTSR &= ~(1 << 2);
    EXTI->RTSR |= (1 << 2);
    EXTI->RTSR &= ~(1 << 3);
    EXTI->RTSR |= (1 << 3);

    // Turn on falling trigger
    EXTI->FTSR &= ~(1 << 2);
    EXTI->FTSR |= (1 << 2);
    EXTI->FTSR &= ~(1 << 3);
    EXTI->FTSR |= (1 << 3);

    // Enable SYSCFG peripheral (this is on APB2 bus)
    RCC->APB2RSTR |= RCC_APB2RSTR_SYSCFGRST;
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    // uint32_t tmp = 0;
    // while(tmp != 0) {
    //     RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    //     tmp = RCC->APB2ENR & RCC_APB2ENR_SYSCFGEN;
    // } 
    // todo: is this correct or do we need to do more to enable here?

    // Configure multiplexer to route PA2-3 to EXTI1
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PB | SYSCFG_EXTICR1_EXTI3_PB;

    // Enable interrupts
    NVIC_EnableIRQ(EXTI2_3_IRQn);
    NVIC_SetPriority(EXTI2_3_IRQn, 1);

    // Configure a second timer (TIM6) to fire an ISR on update event
    // Used to periodically check and update speed variable
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    // Select PSC and ARR values that give an appropriate interrupt rate
    TIM6->PSC = 11;
    TIM6->ARR = 30000;

    TIM6->DIER |= TIM_DIER_UIE;             // Enable update event interrupt
    TIM6->CR1 |= TIM_CR1_CEN;               // Enable Timer

    NVIC_EnableIRQ(TIM6_DAC_IRQn);          // Enable interrupt in NVIC
    NVIC_SetPriority(TIM6_DAC_IRQn, 2);
}

// Encoder interrupt to calculate motor speed, also manages PI controller
void TIM6_DAC_IRQHandler(void) {
    /* Calculate the motor speed in raw encoder counts
     * Note the motor speed is signed! Motor can be run in reverse.
     * Speed is measured by how far the counter moved from center point
     */
    // transmitString("TIM6 is here\r\n");

    uint64_t tickSum = 0;
    for (int i = 0; i < NUM_MOTORS; i++) {
        struct Motor *motor = &motors[i];
        motor->num_ticks = encoderCounts[i];    // Assign count value to appropriate struct
        tickSum += encoderCounts[i];
        encoderCounts[i] = 0;                   // Reset
    }
    uint64_t avgTicks = tickSum / NUM_MOTORS;
    for (int i = 0; i < NUM_MOTORS; i++) {
        struct Motor *motor = &motors[i];
        motor->correctError(motor, avgTicks);
    }
    TIM6->SR &= ~TIM_SR_UIF;        // Acknowledge the interrupt
//    leds.green = 0;
//    leds.set(&leds);
}

/* The handler fired for each tick interrupt.
 * Increments the appropriate count variable according to which encoder fired the event. */
void EXTI2_3_IRQHandler(void) {
    // transmitString("EXTI2_3_IRQHandler is here\r\n");
    
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);

    // NOTE: there is no way to make this pin agnostic, have to manually update consts below if pins are changed
    int ENC_PINS[] = {2, 3};
    int COUNT_IDX_OFFSET = 7;

    // Check which bit is pending (represents which encoder fired tick interrupt)
    for (int i = 0; i < NUM_MOTORS; i++) {
        int pinIdx = ENC_PINS[i];

        uint16_t pinIsPending = EXTI->PR & (1 << pinIdx);
        if (pinIsPending) {
            // Get corresponding count index and increment
            int countIdx = pinIdx - COUNT_IDX_OFFSET;
            encoderCounts[countIdx] += 1;

            // Clear pending interrupt flag
            EXTI->PR &= ~(1 << pinIdx);
        }
    }

//    leds.green = 0;
//    leds.set(&leds);
}

// Calculates error for single motor and resets PWM signal
void PI_update(struct Motor *this, uint64_t avgDist) {

    // Calculate error signal
    this->error = (this->target_ticks - this->num_ticks);

    // Calculate integral error
    this->error_integral += this->error;

    // Clamp the value of the integral to a positive range
    this->error_integral = (this->error_integral > 3200) ? 3200 : this->error_integral;
    this->error_integral = (this->error_integral > 0) ? 0 : this->error_integral;

    // Calculate mean distance traveled by both wheels (in units of ticks)
    this->error_distance = avgDist - this->error_distance;

    // Calculate proportional error & add to output
    int16_t output = Ki * this->error_integral + Kp * this->error;

    // Divide output to get value into proper range (0-100)
    output = output >> 5;

    // Clamp the value for PWM input range
    output = (output > 100) ? 100 : (output < 0) ? 0 : output;

    this->setCycle(this, output);
}

// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycle(struct Motor *this, uint8_t duty) {
    uint32_t TEST_VAL = 1000;
    if(duty <= 100) {

//        // DEBUGGING
//        leds.orange = 1;
//        leds.set(&leds);

        // uint32_t autoReload = this->pwmTimer->ARR;
        uint8_t motorIdx = this->id;
        switch (motorIdx) {
            case 1:
                // todo: change this back after testing
                //this->pwmTimer->CCR1 = ((uint32_t)duty * autoReload)/100;  // Use linear transform to produce CCR1 value
                TIM2->CCR1 = TEST_VAL;
                break;
            case 2:
                //this->pwmTimer->CCR2 = ((uint32_t)duty * autoReload)/100;  // Use linear transform to produce CCR1 value
                TIM2->CCR2 = TEST_VAL;
                break;
        }

//        // DEBUGGING
//        leds.orange = 0;
//        leds.set(&leds);
    }
}

/* Sets the target RPM and direction for this motor */
// todo: we are getting through this
void spinMotor(struct Motor *this, uint16_t targetRpm, int dir) {
  leds.orange = 1;
  leds.set(&leds);
    if (targetRpm > 0) {
        this->target_ticks = targetRpm;
        uint8_t pinIdxA = this->dir_pin_A;
        uint8_t pinIdxB = this->dir_pin_B;

        if (dir > 0) {
            // Pin A to high, pin B to low
            this->dirGpio->ODR |= (1 << pinIdxA);
            this->dirGpio->ODR &= ~(1 << pinIdxB);
        } else if (dir < 0) {
            this->dirGpio->ODR |= (1 << pinIdxB);
            this->dirGpio->ODR &= ~(1 << pinIdxA);
        }
    }
}

/* Mode used for obstacle course */
void hokeyPokey() {
    int THRESHOLD_DIST = 10;

    struct Sensor *left_sensor = &(sensors[0]);
    struct Sensor *center_sensor = &(sensors[1]);
    struct Sensor *right_sensor = &(sensors[2]);

    // Poll values in sensors
    while(1) {
        uint16_t left_dist = left_sensor->read(left_sensor);
        uint16_t center_dist = center_sensor->read(center_sensor);
        uint16_t right_dist = right_sensor->read(right_sensor);

        // Stop if any of our sensors have a close value
        if ((left_dist > THRESHOLD_DIST) || (center_dist > THRESHOLD_DIST) || (right_dist > THRESHOLD_DIST)) {
            stop();
        }

        int leftDetect = left_dist - THRESHOLD_DIST > 0;
        int centerDetect = center_dist = THRESHOLD_DIST > 0;
        int rightDetect = right_dist - THRESHOLD_DIST > 0;

        // todo: fill in
        if (leftDetect && centerDetect && rightDetect) {
            
        } else if (leftDetect && centerDetect) {

        } else if (rightDetect && centerDetect) {

        } else if (leftDetect && rightDetect) {

        } else if (leftDetect) {

        } else if (centerDetect) {

        } else if (rightDetect) {

        } else {
            moveForward();
        }
        // todo: add delay here
    }
}

/* Stops the motor by setting the target RPM to zero. Does not affect direction. */
void stopMotor(struct Motor *this) {
    this->target_ticks = 0;
}

// Convenience methods
void stop() {
    move(0);
}

void moveForward() {
    move(1);
}

void moveRight() {
    move(2);
}

void moveBackward() {
    move(3);
}

void moveLeft() {
    move(4);
}

// Actual movement logic
void move(uint8_t direction) {
    const int FORWARD = 1;
    const int REVERSE = -1;
    const int TARGET_RPM = 100; // todo: placeholder

    struct Motor *left = &motors[0];
    struct Motor *right = &motors[1];

    leds.green = 1;
    leds.set(&leds);

    switch (direction) {
        case 0: // Stop
            motors[0].stop(&(motors[0]));
            motors[1].stop(&(motors[1]));
            break;
        case 1: // Forward
            left->spin(left, TARGET_RPM, FORWARD);
            right->spin(right, TARGET_RPM, FORWARD);
            break;
        case 2: // Right
            left->spin(left, TARGET_RPM, FORWARD);
            right->spin(right, TARGET_RPM, REVERSE);
            break;
        case 3: // Backward
            left->spin(left, TARGET_RPM, REVERSE);
            right->spin(right, TARGET_RPM, REVERSE);
            break;
        case 4: // Left
            left->spin(left, TARGET_RPM, REVERSE);
            right->spin(right, TARGET_RPM, FORWARD);
            break;
    }

    leds.orange = 1;
    leds.set(&leds);
}

/* Initializes three IR sensors */
void initSensors() {
    uint32_t sensorPins[3] = {0, 1, 2};
    for (int i = 0; i < NUM_SENSORS; i++) {
        struct Sensor *sensor = &(sensors[i]);
        initSensor(sensor, sensorPins[i]);
    }
    // todo: may need to init filter here too
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

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
