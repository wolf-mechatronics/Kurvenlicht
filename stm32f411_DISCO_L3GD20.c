/*
 * stm32f411_DISCO_L3GD20.c
 *
 *  Created on: 26.08.2019
 *      Author: Laeuterer
 */

#include "stm32f4xx_hal.h"
#include "stm32f411_DISCO_L3GD20.h"


/*
 * SPI_HandleTypeDef muss über CUBE MX configuriert werden, auf dem DISCO board ist der L3GD20 fest an SPI1 gebunden.
 * Der Chip Selekt ist auf dem DISCO board ebenfalls fest verdrahtet und an Port E, pin 3 Gebunden
 */

__weak SPI_HandleTypeDef hspi1;								// wird durch CUBE MX in der main erzeugt ist zur Laufzeit bekannt hier nur um compilierungsfehler zu vermeiden



int32_t Summe_x, Summe_y, Summe_z;
int16_t turnX, turnY, turnZ;
int16_t buf_x[4]={-0, 0, 0, 0},buf_y[4]={-0, 0, 0, 0}, buf_z[4]={-0, 0, 0, 0};
uint8_t spiTxBuf[2], spiRxBuf[6],X,Y,Z,count=0;


typedef enum
{                                               // DEFAULT    TYPE
  L3GD20_REGISTER_WHO_AM_I            = 0x0F,   // 11010100   r
  L3GD20_REGISTER_CTRL_REG1           = 0x20,   // 00000111   rw
  L3GD20_REGISTER_CTRL_REG2           = 0x21,   // 00000000   rw
  L3GD20_REGISTER_CTRL_REG3           = 0x22,   // 00000000   rw
  L3GD20_REGISTER_CTRL_REG4           = 0x23,   // 00000000   rw
  L3GD20_REGISTER_CTRL_REG5           = 0x24,   // 00000000   rw
  L3GD20_REGISTER_REFERENCE           = 0x25,   // 00000000   rw
  L3GD20_REGISTER_OUT_TEMP            = 0x26,   //            r
  L3GD20_REGISTER_STATUS_REG          = 0x27,   //            r
  L3GD20_REGISTER_OUT_X_L             = 0x28,   //            r
  L3GD20_REGISTER_OUT_X_H             = 0x29,   //            r
  L3GD20_REGISTER_OUT_Y_L             = 0x2A,   //            r
  L3GD20_REGISTER_OUT_Y_H             = 0x2B,   //            r
  L3GD20_REGISTER_OUT_Z_L             = 0x2C,   //            r
  L3GD20_REGISTER_OUT_Z_H             = 0x2D,   //            r
  L3GD20_REGISTER_FIFO_CTRL_REG       = 0x2E,   // 00000000   rw
  L3GD20_REGISTER_FIFO_SRC_REG        = 0x2F,   //            r
  L3GD20_REGISTER_INT1_CFG            = 0x30,   // 00000000   rw
  L3GD20_REGISTER_INT1_SRC            = 0x31,   //            r
  L3GD20_REGISTER_TSH_XH              = 0x32,   // 00000000   rw
  L3GD20_REGISTER_TSH_XL              = 0x33,   // 00000000   rw
  L3GD20_REGISTER_TSH_YH              = 0x34,   // 00000000   rw
  L3GD20_REGISTER_TSH_YL              = 0x35,   // 00000000   rw
  L3GD20_REGISTER_TSH_ZH              = 0x36,   // 00000000   rw
  L3GD20_REGISTER_TSH_ZL              = 0x37,   // 00000000   rw
  L3GD20_REGISTER_INT1_DURATION       = 0x38    // 00000000   rw
} l3gd20Registers_t;

typedef enum
{
	//Der Chip Selekt ist auf dem DISCO board e fest verdrahtet und an Port E, pin 3 gebunden

//	L3GD20_ChipSelect_Port					= GPIOE,
	L3GD20_ChipSelect_Pin					= GPIO_PIN_3,


}l3gd20pin_t;

//Registerbeschreibung aus der Adafruit Ardoino Bibliothek übernommen

void init_L3GD20(void)
{
	/* Set CTRL_REG1 (0x20)
	   ====================================================================
	   BIT  Symbol    Description                                   Default
	   ---  ------    --------------------------------------------- -------
	   7-6  DR1/0     Output data rate                                   00
	   5-4  BW1/0     Bandwidth selection                                00
	     3  PD        0 = Power-down mode, 1 = normal/sleep mode          0
	     2  ZEN       Z-axis enable (0 = disabled, 1 = enabled)           1
	     1  YEN       Y-axis enable (0 = disabled, 1 = enabled)           1
	     0  XEN       X-axis enable (0 = disabled, 1 = enabled)           1 */

	  /* Switch to normal mode and enable all three channels */

  //Bring Slave Select low
HAL_GPIO_WritePin(GPIOE,L3GD20_ChipSelect_Pin,GPIO_PIN_RESET);
//Transmit register+data
spiTxBuf[0]=L3GD20_REGISTER_CTRL_REG1;											//Control Register 1
spiTxBuf[1]=0x4F;
HAL_SPI_Transmit(&hspi1, spiTxBuf, 2,50);
//Bring Slave Select High
HAL_GPIO_WritePin(GPIOE,L3GD20_ChipSelect_Pin,GPIO_PIN_SET);

HAL_Delay(10);

/* Set CTRL_REG2 (0x21)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
   5-4  HPM1/0    High-pass filter mode selection                    00
   3-0  HPCF3..0  High-pass filter cutoff frequency selection      0000 */


//Bring Slave Select low
HAL_GPIO_WritePin(GPIOE,L3GD20_ChipSelect_Pin,GPIO_PIN_RESET);
//Transmit register+data
spiTxBuf[0]=L3GD20_REGISTER_CTRL_REG2;											//Control Register 2
spiTxBuf[1]=0x27;
HAL_SPI_Transmit(&hspi1, spiTxBuf, 2,50);
//Bring Slave Select High
HAL_GPIO_WritePin(GPIOE,L3GD20_ChipSelect_Pin,GPIO_PIN_SET);

HAL_Delay(10);

/* Set CTRL_REG3 (0x22)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     7  I1_Int1   Interrupt enable on INT1 (0=disable,1=enable)       0
     6  I1_Boot   Boot status on INT1 (0=disable,1=enable)            0
     5  H-Lactive Interrupt active config on INT1 (0=high,1=low)      0
     4  PP_OD     Push-Pull/Open-Drain (0=PP, 1=OD)                   0
     3  I2_DRDY   Data ready on DRDY/INT2 (0=disable,1=enable)        0
     2  I2_WTM    FIFO wtrmrk int on DRDY/INT2 (0=dsbl,1=enbl)        0
     1  I2_ORun   FIFO overrun int on DRDY/INT2 (0=dsbl,1=enbl)       0
     0  I2_Empty  FIFI empty int on DRDY/INT2 (0=dsbl,1=enbl)         0 */

//Bring Slave Select low
HAL_GPIO_WritePin(GPIOE,L3GD20_ChipSelect_Pin,GPIO_PIN_RESET);
//Transmit register+data
spiTxBuf[0]=L3GD20_REGISTER_CTRL_REG3;											//Control Register 3
spiTxBuf[1]=0x8;																//Data ready Interrupt aktivieren, so wird pollen von messwerten verhindert
HAL_SPI_Transmit(&hspi1, spiTxBuf, 2,50);
//Bring Slave Select High
HAL_GPIO_WritePin(GPIOE,L3GD20_ChipSelect_Pin,GPIO_PIN_SET);

/* Set CTRL_REG4 (0x23)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     7  BDU       Block Data Update (0=continuous, 1=LSB/MSB)         0
     6  BLE       Big/Little-Endian (0=Data LSB, 1=Data MSB)          0
   5-4  FS1/0     Full scale selection                               00
                                  00 = 250 dps
                                  01 = 500 dps
                                  10 = 2000 dps
                                  11 = 2000 dps
     0  SIM       SPI Mode (0=4-wire, 1=3-wire)                       0 */


//Bring Slave Select low
HAL_GPIO_WritePin(GPIOE,L3GD20_ChipSelect_Pin,GPIO_PIN_RESET);
//Transmit register+data
spiTxBuf[0]=L3GD20_REGISTER_CTRL_REG4;											//Control Register 4
spiTxBuf[1]=0x00;
HAL_SPI_Transmit(&hspi1, spiTxBuf, 2,50);
//Bring Slave Select High
HAL_GPIO_WritePin(GPIOE,L3GD20_ChipSelect_Pin,GPIO_PIN_SET);

/* Set CTRL_REG5 (0x24)
    ====================================================================
    BIT  Symbol    Description                                   Default
    ---  ------    --------------------------------------------- -------
      7  BOOT      Reboot memory content (0=normal, 1=reboot)          0
      6  FIFO_EN   FIFO enable (0=FIFO disable, 1=enable)              0
      4  HPen      High-pass filter enable (0=disable,1=enable)        0
    3-2  INT1_SEL  INT1 Selection config                              00
    1-0  OUT_SEL   Out selection config                               00 */

//Bring Slave Select low
HAL_GPIO_WritePin(GPIOE,L3GD20_ChipSelect_Pin,GPIO_PIN_RESET);
//Transmit register+data
spiTxBuf[0]=L3GD20_REGISTER_CTRL_REG5;											//Control Register 5
spiTxBuf[1]=0x10;																//Hochpassfilter aktiviert gegen driften der Messwerte
HAL_SPI_Transmit(&hspi1, spiTxBuf, 2,50);
//Bring Slave Select High
HAL_GPIO_WritePin(GPIOE,L3GD20_ChipSelect_Pin,GPIO_PIN_SET);

HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12,GPIO_PIN_SET);
return;
}

void Messwerte_L3GD20(int16_t *roll, int16_t *pitch, int16_t *yaw)
{

	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,GPIO_PIN_RESET);
	  //Transmit register+data

	  spiTxBuf[0]=0x28|0x80|0x40;

	  HAL_SPI_Transmit(&hspi1, spiTxBuf, 1,10);

	  HAL_SPI_Receive(&hspi1, spiRxBuf, 6,10);

	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,GPIO_PIN_SET);

	  //Die Register für die Werte von X high, X low bis z low liegen direkt hintereinander
	  //einmaliger aufruf liest drei mal zwei register aus

	turnX=spiRxBuf[1]<<8;	//Bitshift um die high werte einzusortieren
	turnX+=spiRxBuf[0];

	turnY=spiRxBuf[3]<<8;	//Verteilen der Werte aus dem recivebuffer
	turnY+=spiRxBuf[2];

	turnZ=spiRxBuf[4]<<8;
	turnZ+=spiRxBuf[5];

							//Zur stabilisierung der Messwerte wird ein weiterer software Tiefpass eingesetzt
	buf_x[count]=turnX;

	Summe_x=buf_x[0];
	Summe_x+=buf_x[1];
	Summe_x+=buf_x[2];
	Summe_x+=buf_x[3];

	*roll=Summe_x/4;	//Normierung der Darstellung

	buf_y[count]=turnY;

	Summe_y=buf_y[0];
	Summe_y+=buf_y[1];
	Summe_y+=buf_y[2];
	Summe_y+=buf_y[3];

	*pitch=Summe_y/4;

	buf_z[count]=turnZ;

	Summe_z=buf_z[0];
	Summe_z+=buf_z[1];
	Summe_z+=buf_z[2];
	Summe_z+=buf_z[3];

	*yaw=Summe_z/4;


		if (count>2)	//Die gemeinsame Zählvariable wird gemanaged, eine Abfrage auf gleichheit dauert länger als die Größenbestimmung
	{
		count=0;
	}
	else
	{
		count++;
	}

	return;
}
