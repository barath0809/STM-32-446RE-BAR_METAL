#include"stm32f10x.h"
#include "stdio.h"

uint8_t rfid_id[4];



int main(void)
{
	 
		//SysClockConfig();
		systick_init_ms(16000000);
		rc522_init();
		lcd_init();
		setCursor(0,0);
		lcd_send_string("RFID RC522 with");
		setCursor(7,1);
		lcd_send_string("STM32F4");
		setCursor(0,2);
		lcd_send_string("EmbeddedExperIO");
		delay(2000);
		lcd_clear();
		while(1)
				{
				if(rc522_checkCard(rfid_id))
							{

							lcd_clear();
							char data[20];
							setCursor(0,0);
							lcd_send_string("RFID code is");
							setCursor(0,1);
							sprintf(data,"0x%x 0x%x 0x%x 0x%x",rfid_id[0],rfid_id[1],rfid_id[2],rfid_id[3]);
							lcd_send_string(data);
							delay(1000);
							}
				delay(100);
				}

}


void SPI_Init(void)
{
	#define AF5 0x05
	RCC->AHB1ENR |=RCC_AHB1ENR_GPIOAEN; //enable clock forn gpio a
	RCC->APB2ENR |=RCC_APB2ENR_SPI1EN; //enable clock for spi1
	GPIOA->MODER |=GPIOA_MODER_MODE5_1|GPIOA_MODER_MODE6_1|GPIOA_MODER_MODE7_1;
	GPIOA->MODER &=~(GPIOA_MODER_MODE5_0|GPIO_MODER_MODE6_0|GPIO_MODER_MODE7_0);
	GPIOA->OSPEEDR |=GPIOA_OSPEEDER_OSPEEDR5|GPIO_OSPEEDER_OSPEEDR6|GPIO_OSPEEDER_OSPEEDR7;

	GPIOA->AFR[0]|=(AF5<<20)|(AF5<<24)|(AF5<<28);
	SPI1->CR2=0;
	SPI1->CR1=SPI_CR1_SSM|SPI_CR1_MSTR|SPI_CR1_BR_2|SPI_CR1_SSI|SPI_CR1_SPE;
}
int8_t SPI_Transmit(uint8_t *data, uint32_t size)
{


	uint32_t i		=0;
	uint8_t  temp	=0;
	uint32_t start=millis();
	temp =SPI1->DR;
	temp=SPI1->SR;
	while(i<size)
		{
		while(!((SPI1->SR)&SPI_SR_TXE)){if(millis()-start>1000){
			printf("TXE timed out\r\n");
			return -1;}} // wait to transmision buffer to be emplty
		SPI1->DR= data[i];
		while(!(SPI1->SR&SPI_SR_BSY)){if(millis()-start>1000){printf("BSY timed out\r\n");return -1;}}
		i++;
		}
while(!((SPI1->SR)&SPI_SR_TXE)){if(millis()-start>1000){printf("TXE2 time dout\r\n");return -1;}}
while((SPI1->SR)&SPI_SR_BSY){if(millis()-start>1000){printf("BSY2 timed out\r\n"); return -1;}}
temp =SPI1->DR;
temp=SPI1->SR;
return 0;
}
int8_t SPI_Receive(uint8_t *data, uint32_t size)
{
while(size)
		{
	uint32_t start=millis();
		SPI1->DR=0;
		while(!(SPI1->SR&SPI_SR_RXNE)){if(millis()-start>200){return -1;}}
		*data++=(SPI1->DR);
			size--;
		}
return 0;
}

void rc522_init(void)
{
	/*
	 * STM32 ->RFID
	 * SPI  -> SPI
	 * PA8  ->RST
	 * PB0  ->CS
	 * */
  SPI_Init();
  GPIOA->MODER |=GPIO_MODER_MODE8_0;
  GPIOA->MODER &=~GPIO_MODER_MODE8_1;

  RCC->AHB1ENR |=RCC_AHB1ENR_GPIOBEN;

  GPIOB->MODER |=GPIO_MODER_MODE0_0;
  GPIOB->MODER &=~GPIO_MODER_MODE0_1;
  GPIOA->BSRR=GPIO_BSRR_BR8;
  for(volatile int i=0;i<100000;i++);
  GPIOA->BSRR=GPIO_BSRR_BS8;
  for(volatile int i=0;i<100000;i++);
  rc522_reset();

  rc522_regWrite8(MFRC522_REG_T_MODE, 0x80);
  rc522_regWrite8(MFRC522_REG_T_PRESCALER, 0xA9);
  rc522_regWrite8(MFRC522_REG_T_RELOAD_L, 0xE8);
  rc522_regWrite8(MFRC522_REG_T_RELOAD_H, 0x03);


  rc522_regWrite8(MFRC522_REG_TX_AUTO, 0x40);
  rc522_regWrite8(MFRC522_REG_MODE, 0x3D);

  rc522_antennaON();   //Open the antenna
}

uint8_t rc522_regRead8(uint8_t reg)
{
  spi_cs_rfid_write(0);
  reg = ((reg << 1) & 0x7E) | 0x80;
  SPI_Transmit(&reg, 1);
  uint8_t dataRd=0;
  SPI_Receive(&dataRd, 1);
  spi_cs_rfid_write(1);
  return dataRd;
}

/**
 * @brief write register
 */
void rc522_regWrite8(uint8_t reg, uint8_t data8)
{
  spi_cs_rfid_write(0);
  uint8_t txData[2] = {0x7E&(reg << 1), data8};
  SPI_Transmit(txData, 2);
  spi_cs_rfid_write(1);
}
bool rc522_toCard(
    uint8_t command,
    uint8_t* sendData,
    uint8_t sendLen,
    uint8_t* backData,
    uint16_t* backLen);

bool rc522_request(uint8_t reqMode, uint8_t *tagType);

bool rc522_antiColl(uint8_t* serNum);

void spi_cs_rfid_write(bool state)
{
	if(state)
	  {
	    GPIOB->ODR |= (1UL << 0);
	  }
	  else
	  {
	    GPIOB->ODR &= ~(1UL << 0);
	  }
}

/**
 * @brief set bit
 */
void rc522_setBit(uint8_t reg, uint8_t mask)
{
  rc522_regWrite8(reg, rc522_regRead8(reg)|mask);
}

/**
 * @brief clear bit
 */
void rc522_clearBit(uint8_t reg, uint8_t mask)
{
  rc522_regWrite8(reg, rc522_regRead8(reg)&(~mask));
}

/**
 * @brief reset function
 */
void rc522_reset(void)
{
  rc522_regWrite8(0x01, 0x0F);
}

/**
 * @brief Antenna ON
 */
void rc522_antennaON(void)
{
  uint8_t temp;

  temp = rc522_regRead8(MFRC522_REG_TX_CONTROL);
  if (!(temp & 0x03)) {
    rc522_setBit(MFRC522_REG_TX_CONTROL, 0x03);
  }
}

/**
 * @brief Check card
 */
bool rc522_checkCard(uint8_t *id)
{
  bool status=false;
  //Find cards, return card type
    status = rc522_request(PICC_REQIDL, id);
    if (status == true) {
      //Card detected
      //Anti-collision, return card serial number 4 bytes
      status = rc522_antiColl(id);
    }
    rc522_halt();      //Command card into hibernation

    return status;
}

/**
 * @brief Request function
 */
bool rc522_request(uint8_t reqMode, uint8_t *tagType)
{
  bool status=false;
  uint16_t backBits;
  rc522_regWrite8(MFRC522_REG_BIT_FRAMING, 0x07);
  tagType[0] = reqMode;
  status = rc522_toCard(PCD_TRANSCEIVE, tagType, 1, tagType, &backBits);
  if ((status != true) || (backBits != 0x10)) {
    status = false;
  }
  return status;
}

/**
 * @brief to Card
 */
bool rc522_toCard(
    uint8_t command,
    uint8_t* sendData,
    uint8_t sendLen,
    uint8_t* backData,
    uint16_t* backLen)
{
  bool status = false;
  uint8_t irqEn = 0x00;
  uint8_t waitIRq = 0x00;
  uint8_t lastBits;
  uint8_t n;
  uint16_t i;

  switch (command) {
    case PCD_AUTHENT: {
      irqEn = 0x12;
      waitIRq = 0x10;
      break;
    }
    case PCD_TRANSCEIVE: {
      irqEn = 0x77;
      waitIRq = 0x30;
      break;
    }
    default:
      break;
  }

  rc522_regWrite8(MFRC522_REG_COMM_IE_N, irqEn | 0x80);
  rc522_clearBit(MFRC522_REG_COMM_IRQ, 0x80);
  rc522_setBit(MFRC522_REG_FIFO_LEVEL, 0x80);

  rc522_regWrite8(MFRC522_REG_COMMAND, PCD_IDLE);

  //Writing data to the FIFO
  for (i = 0; i < sendLen; i++) {
    rc522_regWrite8(MFRC522_REG_FIFO_DATA, sendData[i]);
  }

  //Execute the command
  rc522_regWrite8(MFRC522_REG_COMMAND, command);
  if (command == PCD_TRANSCEIVE) {
    rc522_setBit(MFRC522_REG_BIT_FRAMING, 0x80);   //StartSend=1,transmission of data starts
  }

  //Waiting to receive data to complete
  i = 100;  //i according to the clock frequency adjustment, the operator M1 card maximum waiting time 25ms???
  do {
    //CommIrqReg[7..0]
    //Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
    n = rc522_regRead8(MFRC522_REG_COMM_IRQ);
    i--;
  } while ((i!=0) && !(n&0x01) && !(n&waitIRq));

  rc522_clearBit(MFRC522_REG_BIT_FRAMING, 0x80);     //StartSend=0

  if (i != 0)  {
    if (!(rc522_regRead8(MFRC522_REG_ERROR) & 0x1B)) {
      status = true;
      if (n & irqEn & 0x01) {
        status = false;
      }

      if (command == PCD_TRANSCEIVE) {
        n = rc522_regRead8(MFRC522_REG_FIFO_LEVEL);
        uint8_t l = n;
        lastBits = rc522_regRead8(MFRC522_REG_CONTROL) & 0x07;
        if (lastBits) {
          *backLen = (n - 1) * 8 + lastBits;
        } else {
          *backLen = n * 8;
        }

        if (n == 0) {
          n = 1;
        }
        if (n > MFRC522_MAX_LEN) {
          n = MFRC522_MAX_LEN;
        }

        //Reading the received data in FIFO
        for (i = 0; i < n; i++) {
          uint8_t d = rc522_regRead8(MFRC522_REG_FIFO_DATA);
          if (l == 4)
            printf("%02x ", d);
          backData[i] = d;
        }
        if (l==4)
          printf("\r\n");
        return status;
      }
    } else {
      printf("error\r\n");
      status = false;
    }
  }

  return status;
}

bool rc522_antiColl(uint8_t* serNum)
{
  bool status;
  uint8_t i;
  uint8_t serNumCheck = 0;
  uint16_t unLen;
  //for (i = 0; i < 4; i++)
//    printf("Anticoll In %d: 0x%02x\r\n", i, serNum[i]);


  rc522_regWrite8(MFRC522_REG_BIT_FRAMING, 0x00);    //TxLastBists = BitFramingReg[2..0]

  serNum[0] = PICC_ANTICOLL;
  serNum[1] = 0x20;
  status = rc522_toCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

  //for (i = 0; i < 4; i++)
//      printf("Anticoll ToCard %d: 0x%02x\r\n", i, serNum[i]);

  if (status == true) {
    //Check card serial number
    for (i = 0; i < 4; i++) {
      serNumCheck ^= serNum[i];
    }
    if (serNumCheck != serNum[i]) {
      status = false;
    }
  }
  return status;
}

void rc522_halt(void)
{
  uint16_t unLen;
  uint8_t buff[4];

  buff[0] = PICC_HALT;
  buff[1] = 0;
  rc522_calculateCRC(buff, 2, &buff[2]);

  rc522_toCard(PCD_TRANSCEIVE, buff, 4, buff, &unLen);
}

void rc522_calculateCRC(uint8_t*  pIndata, uint8_t len, uint8_t* pOutData)
{
  uint8_t i, n;

  rc522_clearBit(MFRC522_REG_DIV_IRQ, 0x04);     //CRCIrq = 0
  rc522_setBit(MFRC522_REG_FIFO_LEVEL, 0x80);      //Clear the FIFO pointer
  //Write_MFRC522(CommandReg, PCD_IDLE);

  //Writing data to the FIFO
  for (i = 0; i < len; i++) {
    rc522_regWrite8(MFRC522_REG_FIFO_DATA, *(pIndata+i));
  }
  rc522_regWrite8(MFRC522_REG_COMMAND, PCD_CALCCRC);

  //Wait CRC calculation is complete
  i = 0xFF;
  do {
    n = rc522_regRead8(MFRC522_REG_DIV_IRQ);
    i--;
  } while ((i!=0) && !(n&0x04));      //CRCIrq = 1

  //Read CRC calculation result
  pOutData[0] = rc522_regRead8(MFRC522_REG_CRC_RESULT_L);
  pOutData[1] = rc522_regRead8(MFRC522_REG_CRC_RESULT_M);
}

/**
 * @brief compare IDs
 */
bool rc522_compareIds(uint8_t *idCurrent, uint8_t *idReference)
{
  uint8_t i;
  for(i=0; i<4;i++)
  {
    if(idCurrent[i] != idReference[i])
    {
      return false;
    }
  }
  return true;
}
