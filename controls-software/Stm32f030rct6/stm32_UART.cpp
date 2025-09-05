/* STM32F030RCT6 UART Communication Code for YOLO Object Detection Data */

#include "stm32f0xx.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Function prototypes
void SystemClock_Config(void);
void UART_Init(void);
void UART_SendString(const char* str);
void UART_SendChar(char c);
char UART_ReceiveChar(void);
void UART_ReceiveString(char* buffer, uint16_t max_len);
void GPIO_Init(void);
void ParseCoordinates(const char* data, int* center_x, int* center_y, int* width, int* height);

// Global variables
char rx_buffer[100];

int main(void)
{
    // Initialize system
    SystemClock_Config();
    GPIO_Init();
    UART_Init();
    
    // Send startup message
    UART_SendString("STM32F030RCT6 Ready - Waiting for YOLO data!\r\n");
    
    int center_x, center_y, width, height;
    char display_buffer[100];
    
    while (1)
    {
        // Wait for data reception
        UART_ReceiveString(rx_buffer, sizeof(rx_buffer));
        
        // Parse the received coordinates (format: center_x,center_y,width,height)
        ParseCoordinates(rx_buffer, &center_x, &center_y, &width, &height);
        
        // Display in the requested format: C(center coordinates), (x distance, y distance)
        sprintf(display_buffer, "C(%d,%d), (%d,%d)\r\n", center_x, center_y, width, height);
        UART_SendString(display_buffer);
        
        // Clear buffer
        memset(rx_buffer, 0, sizeof(rx_buffer));
    }
}

void ParseCoordinates(const char* data, int* center_x, int* center_y, int* width, int* height)
{
    // Initialize values to 0 in case parsing fails
    *center_x = 0;
    *center_y = 0;
    *width = 0;
    *height = 0;
    
    // Create a copy of the data for parsing
    char temp_data[100];
    strncpy(temp_data, data, sizeof(temp_data) - 1);
    temp_data[sizeof(temp_data) - 1] = '\0';
    
    // Parse comma-separated values
    char* token = strtok(temp_data, ",");
    if (token != NULL) {
        *center_x = atoi(token);
        
        token = strtok(NULL, ",");
        if (token != NULL) {
            *center_y = atoi(token);
            
            token = strtok(NULL, ",");
            if (token != NULL) {
                *width = atoi(token);
                
                token = strtok(NULL, ",");
                if (token != NULL) {
                    *height = atoi(token);
                }
            }
        }
    }
}

void SystemClock_Config(void)
{
    // Enable HSI (8MHz internal oscillator)
    RCC->CR |= RCC_CR_HSION;
    while(!(RCC->CR & RCC_CR_HSIRDY));
    
    // Set HSI as system clock
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_HSI;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);
}

void GPIO_Init(void)
{
    // Enable GPIOA clock
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    
    // Configure PA9 (TX) and PA10 (RX) for UART1
    // PA9 - Alternate function, push-pull
    GPIOA->MODER &= ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10);
    GPIOA->MODER |= (GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1);
    
    // Set alternate function to AF1 (UART1) for PA9 and PA10
    GPIOA->AFR[1] &= ~(GPIO_AFRH_AFRH1 | GPIO_AFRH_AFRH2);
    GPIOA->AFR[1] |= (0x01 << GPIO_AFRH_AFRH1_Pos) | (0x01 << GPIO_AFRH_AFRH2_Pos);
    
    // Set output speed to high
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR9 | GPIO_OSPEEDER_OSPEEDR10);
}

void UART_Init(void)
{
    // Enable UART1 clock
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    
    // Configure UART1
    // Disable UART1 first
    USART1->CR1 &= ~USART_CR1_UE;
    
    // Set baud rate to 9600 (assuming 8MHz system clock)
    // USARTDIV = 8000000 / 9600 = 833.33 ≈ 833
    USART1->BRR = 833;
    
    // Configure frame: 8 data bits, no parity, 1 stop bit
    USART1->CR1 &= ~(USART_CR1_M | USART_CR1_PCE);
    USART1->CR2 &= ~USART_CR2_STOP;
    
    // Enable transmitter and receiver
    USART1->CR1 |= (USART_CR1_TE | USART_CR1_RE);
    
    // Enable UART1
    USART1->CR1 |= USART_CR1_UE;
}

void UART_SendChar(char c)
{
    // Wait until transmit data register is empty
    while(!(USART1->ISR & USART_ISR_TXE));
    
    // Send character
    USART1->TDR = c;
    
    // Wait until transmission is complete
    while(!(USART1->ISR & USART_ISR_TC));
}

void UART_SendString(const char* str)
{
    while(*str)
    {
        UART_SendChar(*str++);
    }
}

char UART_ReceiveChar(void)
{
    // Wait until data is received
    while(!(USART1->ISR & USART_ISR_RXNE));
    
    // Return received character
    return (char)USART1->RDR;
}

void UART_ReceiveString(char* buffer, uint16_t max_len)
{
    uint16_t i = 0;
    char c;
    
    do
    {
        c = UART_ReceiveChar();
        
        if(c == '\r')
        {
            break;
        }
        else if(c == '\n')
        {
            break;
        }
        else if(c == '\b' || c == 0x7F) // Backspace or DEL
        {
            if(i > 0)
            {
                i--;
            }
        }
        else if(i < max_len - 1)
        {
            buffer[i++] = c;
        }
        
    } while(i < max_len - 1);
    
    buffer[i] = '\0'; // Null terminate
}

// Error handler
void Error_Handler(void)
{
    while(1)
    {
        // Stay here in case of error
    }
}