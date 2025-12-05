#include "platform.h"
#include "stm32h7xx_hal.h"

extern I2C_HandleTypeDef hi2c1;  // Match your I2C

uint8_t VL53L8CX_RdByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value)
{
    uint8_t status = 0;
    if(HAL_I2C_Mem_Read(&hi2c1, p_platform->address, RegisterAdress,
                        I2C_MEMADD_SIZE_16BIT, p_value, 1, 5000) != HAL_OK)
    {
        status = 255;
    }
    return status;
}

uint8_t VL53L8CX_WrByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value)
{
    uint8_t status = 0;
    if(HAL_I2C_Mem_Write(&hi2c1, p_platform->address, RegisterAdress,
                         I2C_MEMADD_SIZE_16BIT, &value, 1, 5000) != HAL_OK)
    {
        status = 255;
    }
    return status;
}

uint8_t VL53L8CX_WrMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress,
                         uint8_t *p_values, uint32_t size)
{
    uint8_t status = 0;
    uint32_t timeout = 10000 + (size * 1);

    if(HAL_I2C_Mem_Write(&hi2c1, p_platform->address, RegisterAdress,
                         I2C_MEMADD_SIZE_16BIT, p_values, size, timeout) != HAL_OK)
    {
        status = 255;
    }
    return status;
}

uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress,
                         uint8_t *p_values, uint32_t size)
{
    uint8_t status = 0;
    uint32_t timeout = 10000 + (size * 1);

    if(HAL_I2C_Mem_Read(&hi2c1, p_platform->address, RegisterAdress,
                        I2C_MEMADD_SIZE_16BIT, p_values, size, timeout) != HAL_OK)
    {
        status = 255;
    }
    return status;
}

uint8_t VL53L8CX_Reset_Sensor(VL53L8CX_Platform *p_platform)
{
    return 0;
}

void VL53L8CX_SwapBuffer(uint8_t *buffer, uint16_t size)
{
    uint32_t i, tmp;
    for(i = 0; i < size; i = i + 4)
    {
        tmp = (buffer[i]<<24)|(buffer[i+1]<<16)|(buffer[i+2]<<8)|(buffer[i+3]);
        memcpy(&(buffer[i]), &tmp, 4);
    }
}

uint8_t VL53L8CX_WaitMs(VL53L8CX_Platform *p_platform, uint32_t TimeMs)
{
    HAL_Delay(TimeMs);
    return 0;
}
