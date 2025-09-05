#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_i2c.h"
#include "stm32f0xx_ll_bus.h"



// PCA9685 Register Addresses
#define PCA9685_MODE1       0x00
#define PCA9685_PRESCALE    0xFE
#define PCA9685_LED0_ON_L   0x06
#define PCA9685_LED0_ON_H   0x07
#define PCA9685_LED0_OFF_L  0x08
#define PCA9685_LED0_OFF_H  0x09

// PCA9685 Mode bits
#define MODE1_RESTART       0x80
#define MODE1_SLEEP         0x10
#define MODE1_AI            0x20

// Configuration
#define PCA9685_I2C_ADDR    0x40        // 7-bit address
#define SERVO_FREQ          50          // 50Hz for standard servos
#define OSC_FREQ            25000000    // Internal oscillator frequency

// Function prototypes
void SystemClock_Config(void);
void I2C1_Init(void);
void PCA9685_Init(void);
void PCA9685_SetServoAngle(uint8_t servo, uint16_t angle_deg);
void PCA9685_SetAllServos(uint16_t angle_deg);
void PCA9685_SetMultipleServos(uint16_t angles[6]);
void PCA9685_SetPWM(uint8_t servo, uint16_t on, uint16_t off);
void PCA9685_WriteReg(uint8_t reg, uint8_t value);
uint8_t PCA9685_ReadReg(uint8_t reg);

int main(void)
 {
    SystemClock_Config();
     LL_Init1msTick(48000000);
    I2C1_Init();
    
    // Initialize PCA9685
    PCA9685_Init();
    
    
    while (1)
    {
        
    //    while(1){
    //    for (float angle = 100; angle <= 180; angle +=10) {
    //    PCA9685_SetServoAngle(0, angle);
    //    LL_mDelay(100); // Wait 500ms between steps - using wait_ms() for your library
    //    // wait_ms(500); // Alternative using your library's wait_ms function
    //  }
    
    //for (float angle = 180; angle >= 100; angle -= 10) {
    //    PCA9685_SetServoAngle(0, angle);
    //    LL_mDelay(100);
    //}
    //LL_mDelay(5000);
    //}
        //PCA9685_SetServoAngle(0, 150);  // Move servo to 90°
        LL_mDelay(300000);  // Stagger the movement
        // Example 1: Control all 6 servos to same position
        PCA9685_SetAllServos(0);    // All to 0°
        LL_mDelay(1000);
        
        
        // Example 2: Control each servo individually
        for(uint8_t servo = 0; servo < 6; servo++)
        {
            PCA9685_SetServoAngle(servo, 90);  // Move servo to 90°
            LL_mDelay(300);  // Stagger the movement
        }
        LL_mDelay(1000);
        
        // Example 3: Set all servos to different positions at once
        uint16_t positions[6] = {0, 30, 60, 90, 120, 180};  // Different angles
        PCA9685_SetMultipleServos(positions);
        LL_mDelay(2000);
        
        // Example 4: Wave pattern
        for(uint16_t angle = 0; angle <= 180; angle += 10)
        {
            for(uint8_t servo = 0; servo < 6; servo++)
            {
                uint16_t servo_angle = angle + (servo * 15);  // Offset each servo
                if(servo_angle > 180) servo_angle = 180;
                PCA9685_SetServoAngle(servo, servo_angle);
            }
            LL_mDelay(100);
        }
        
        LL_mDelay(2000);
    }
}

void PCA9685_Init(void)
{
    // Reset the PCA9685
    PCA9685_WriteReg(PCA9685_MODE1, MODE1_RESTART);
    LL_mDelay(10);
    
    // Set PWM frequency to 50Hz for servo control
    // Calculate prescale: prescale = (OSC_FREQ / (4096 * freq)) - 1
    uint8_t prescale = (uint8_t)((OSC_FREQ / (4096.0 * SERVO_FREQ)) + 0.5) - 1;
    
    // Put PCA9685 to sleep to set prescale
    uint8_t oldmode = PCA9685_ReadReg(PCA9685_MODE1);
    uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;
    PCA9685_WriteReg(PCA9685_MODE1, newmode);
    
    // Set prescale
    PCA9685_WriteReg(PCA9685_PRESCALE, prescale);
    
    // Wake up and enable auto-increment
    PCA9685_WriteReg(PCA9685_MODE1, oldmode);
    LL_mDelay(5);
    PCA9685_WriteReg(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);
}

void PCA9685_SetServoAngle(uint8_t servo, uint16_t angle_deg)
{
    // MG996R extended range: 1ms to 2.5ms for 0° to 180°
    // At 50Hz: 20ms period = 4096 counts
    // 1ms = 204.8 counts ≈ 205
    // 2.5ms = 512 counts
    uint16_t pulse = 102 + ((angle_deg * 410) / 180);  // 0.5ms to 2.5ms range
    
    PCA9685_SetPWM(servo, 0, pulse);
}

void PCA9685_SetPWM(uint8_t servo, uint16_t on, uint16_t off)
{
    uint8_t reg = PCA9685_LED0_ON_L + 4 * servo;
    
    PCA9685_WriteReg(reg,     on & 0xFF);        // ON_L
    PCA9685_WriteReg(reg + 1, on >> 8);          // ON_H
    PCA9685_WriteReg(reg + 2, off & 0xFF);       // OFF_L
    PCA9685_WriteReg(reg + 3, off >> 8);         // OFF_H
}

void PCA9685_WriteReg(uint8_t reg, uint8_t value)
{
    // Wait until I2C is ready
    while(LL_I2C_IsActiveFlag_BUSY(I2C1));
    
    // Start I2C transaction
    LL_I2C_HandleTransfer(I2C1, PCA9685_I2C_ADDR << 1, LL_I2C_ADDRSLAVE_7BIT, 2, 
                         LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
    
    // Send register address
    while(!LL_I2C_IsActiveFlag_TXIS(I2C1));
    LL_I2C_TransmitData8(I2C1, reg);
    
    // Send data
    while(!LL_I2C_IsActiveFlag_TXIS(I2C1));
    LL_I2C_TransmitData8(I2C1, value);
    
    // Wait for transfer complete
    while(!LL_I2C_IsActiveFlag_STOP(I2C1));
    LL_I2C_ClearFlag_STOP(I2C1);
}

uint8_t PCA9685_ReadReg(uint8_t reg)
{
    uint8_t data;
    
    // Wait until I2C is ready
    while(LL_I2C_IsActiveFlag_BUSY(I2C1));
    
    // Send register address
    LL_I2C_HandleTransfer(I2C1, PCA9685_I2C_ADDR << 1, LL_I2C_ADDRSLAVE_7BIT, 1, 
                         LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);
    
    while(!LL_I2C_IsActiveFlag_TXIS(I2C1));
    LL_I2C_TransmitData8(I2C1, reg);
    
    while(!LL_I2C_IsActiveFlag_TC(I2C1));
    
    // Read data
    LL_I2C_HandleTransfer(I2C1, PCA9685_I2C_ADDR << 1, LL_I2C_ADDRSLAVE_7BIT, 1, 
                         LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
    
    while(!LL_I2C_IsActiveFlag_RXNE(I2C1));
    data = LL_I2C_ReceiveData8(I2C1);
    
    while(!LL_I2C_IsActiveFlag_STOP(I2C1));
    LL_I2C_ClearFlag_STOP(I2C1);
    
    return data;
}

void SystemClock_Config(void)
{
    // Configure HSI as system clock source (8MHz)
    LL_RCC_HSI_Enable();
    while(!LL_RCC_HSI_IsReady());
    
    // Alternative: Direct register configuration if LL functions don't work
    // Configure PLL: HSI/2 * 12 = 48MHz
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL);
    RCC->CFGR |= (RCC_CFGR_PLLSRC_HSI_DIV2 | RCC_CFGR_PLLMUL12);
    
    LL_RCC_PLL_Enable();
    while(!LL_RCC_PLL_IsReady());
    
    // Set flash latency
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
    
    // Configure system clock
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);
    
    // Configure AHB and APB prescalers
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);   // 48MHz
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);    // 48MHz
    
    // Update system core clock variable
    LL_SetSystemCoreClock(48000000);
}

void I2C1_Init(void)
{
    // Enable GPIO and I2C clocks
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
    
    // Configure GPIO pins for I2C1
    // PB6 -> I2C1_SCL, PB7 -> I2C1_SDA
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // Configure I2C1
    LL_I2C_InitTypeDef I2C_InitStruct = {0};
    I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
    I2C_InitStruct.Timing = 0x2000090E;  // 100kHz timing for 48MHz PCLK
    I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
    I2C_InitStruct.DigitalFilter = 0;
    I2C_InitStruct.OwnAddress1 = 0;
    I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
    I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
    LL_I2C_Init(I2C1, &I2C_InitStruct);
    
    // Enable I2C1
    LL_I2C_Enable(I2C1);
}

void PCA9685_SetAllServos(uint16_t angle_deg)
{
    // Set all 6 servos to the same angle
    for(uint8_t servo = 0; servo < 6; servo++)
    {
        PCA9685_SetServoAngle(servo, angle_deg);
    }
}

void PCA9685_SetMultipleServos(uint16_t angles[6])
{
    // Set each servo to a different angle
    for(uint8_t servo = 0; servo < 6; servo++)
    {
        PCA9685_SetServoAngle(servo, angles[servo]);
    }
}