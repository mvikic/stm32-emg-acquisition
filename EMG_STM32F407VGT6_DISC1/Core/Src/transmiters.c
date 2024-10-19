#include "transmiters.h"
#include "config.h"
#include <math.h>

// Function to convert integer to string and send via UART
void send_number_with_newline(UART_HandleTypeDef *huart, uint16_t num)
{
    char buffer[10]; // Buffer to hold the number as string with newline
    int i = 0;

    // Convert the number to string
    if (num == 0)
    {
        buffer[i++] = '0';
    }
    else
    {
        // Process each digit (start from the least significant digit)
        int temp = num;
        while (temp > 0)
        {
            buffer[i++] = (temp % 10) + '0';
            temp /= 10;
        }

        // Reverse the buffer to correct the digit order
        for (int j = 0; j < i / 2; j++)
        {
            char t = buffer[j];
            buffer[j] = buffer[i - j - 1];
            buffer[i - j - 1] = t;
        }
    }

    // Add newline characters
    buffer[i++] = '\r';
    buffer[i++] = '\n';

    // Send the number with newline
    HAL_UART_Transmit(huart, (uint8_t*)buffer, i, TIMEOUT);
}

void send_float_with_newline(UART_HandleTypeDef *huart, float num)
{
    char buffer[50]; // Buffer to hold the number as string with newline
    int i = 0;
    
    // Handle negative numbers
    if (num < 0)
    {
        buffer[i++] = '-';
        num = -num;
    }

    // Convert integer part
    uint32_t int_part = (uint32_t)num;
    float frac_part = num - (float)int_part;

    if (int_part == 0)
    {
        buffer[i++] = '0';
    }
    else
    {
        // Process each digit
        char int_buffer[20];
        int int_len = 0;
        while (int_part > 0)
        {
            int_buffer[int_len++] = (int_part % 10) + '0';
            int_part /= 10;
        }

        // Reverse the integer part string
        for (int j = int_len - 1; j >= 0; j--)
        {
            buffer[i++] = int_buffer[j];
        }
    }

    // Add decimal point
    buffer[i++] = '.';

    // Convert fractional part
    frac_part *= FLOAT_PRECISION; // Adjust precision here
    uint32_t frac_int = (uint32_t)round(frac_part);

    // Process each digit of fractional part
    char frac_buffer[20];
    int frac_len = 0;
    if (frac_int == 0)
    {
        buffer[i++] = '0';
    }
    else
    {
        while (frac_int > 0)
        {
            frac_buffer[frac_len++] = (frac_int % 10) + '0';
            frac_int /= 10;
        }

        // Reverse the fractional part string
        for (int j = frac_len - 1; j >= 0; j--)
        {
            buffer[i++] = frac_buffer[j];
        }
    }

    // Add newline characters
    buffer[i++] = '\r';
    buffer[i++] = '\n';

    // Send the number with newline
    HAL_UART_Transmit(huart, (uint8_t*)buffer, i, TIMEOUT);
}