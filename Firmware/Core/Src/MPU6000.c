#include "MPU6000.h"

void spiReadBytes(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *pData, uint16_t size) // ***
{
    reg = reg | 0x80;

    HAL_SPI_Transmit(hspi, &reg, 1, MSG_TIMEOUT);
    HAL_SPI_Receive(hspi, pData, size, MSG_TIMEOUT);

}

void spiWriteOneByte(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t data) // ***
{
    reg = reg & 0x7F;

    HAL_SPI_Transmit(hspi, &reg, 1, MSG_TIMEOUT);
    HAL_SPI_Transmit(hspi, &data, 1, MSG_TIMEOUT);
}

void spiReadOneByte(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t* pData) // ***
{
    reg = reg | 0x80;

    HAL_SPI_Transmit(hspi, &reg, 1, MSG_TIMEOUT);
    HAL_SPI_Receive(hspi, pData, 1, MSG_TIMEOUT);
}



uint8_t MPU6000_init(SPI_HandleTypeDef *MPU6000_SPI) {

	uint8_t whoAmI;
	uint8_t addr[2] = {(MPU6000_WHOAMI | 0x80), 0};
	uint8_t addr_null = 0;
	__HAL_SPI_ENABLE(MPU6000_SPI);

	HAL_SPI_TransmitReceive(MPU6000_SPI, addr, &whoAmI, 2, 10);

    spiWriteOneByte(MPU6000_SPI, MPU6000_PWR_MGMT_1, BIT_H_RESET);  // reset device configuration
    HAL_Delay(150);

    spiWriteOneByte(MPU6000_SPI, 0x68, 0x07); //MPU6000_SIGNAL_PATH_RESET
    HAL_Delay(150);

    spiWriteOneByte(MPU6000_SPI, MPU6000_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
    HAL_Delay(1);

    spiReadOneByte(MPU6000_SPI, MPU6000_WHOAMI, &whoAmI);
    HAL_Delay(1);

    spiWriteOneByte(MPU6000_SPI, MPU6000_USER_CTRL, BIT_I2C_IF_DIS);    // disable I2C interface
    HAL_Delay(1);

    spiWriteOneByte(MPU6000_SPI, MPU6000_PWR_MGMT_2, 0x00);
    HAL_Delay(1);

    spiWriteOneByte(MPU6000_SPI, MPU6000_SMPLRT_DIV, 0x00);
    HAL_Delay(1);

    spiWriteOneByte(MPU6000_SPI, MPU6000_GYRO_CONFIG, BITS_FS_2000DPS);
    HAL_Delay(1);

    spiWriteOneByte(MPU6000_SPI, MPU6000_ACCEL_CONFIG, BITS_FS_16G);;
    HAL_Delay(1);

    spiWriteOneByte(MPU6000_SPI, MPU6000_INT_PIN_CFG, 0x10);
    HAL_Delay(1);

	uint8_t rawData[14];
	spiReadBytes(MPU6000_SPI, MPU6000_ACCEL_XOUT_H, rawData, 14);

	return HAL_OK;
}
