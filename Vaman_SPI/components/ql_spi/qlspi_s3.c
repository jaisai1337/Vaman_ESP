#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <esp_log.h>
#include <esp_err.h>

#include "tlc_reg.h"

//moved here since QL_Trace.h has same definition */
#if 0
#define QL_STATUS_BASE 0	// QL Status starting number
typedef enum {
				QL_STATUS_OK = QL_STATUS_BASE,		/*0*/
				QL_STATUS_ERROR,					/*1 Error*/
				QL_STATUS_ERROR_BAD_PARAMETER,		/*2 Bad parameter passed*/
				QL_STATUS_ERROR_TIMEOUT,			/*3 Timeout occured*/
				QL_STATUS_ERROR_DEVICE_BUSY			/*4 Device is currently busy*/
			 } QL_Status;
#endif
#include "qlspi_s3.h"

static const char *TAG = "[qlspi_s3]";

/* tlc_reg.c */

int32_t tlc_reg_read( uint32_t addr, uint8_t *readBuf, uint32_t length)
{
	unsigned char cmd = 0;
	cmd = CREATE_CMD(QLULPSH_CMD_READ, addr);

	return shub_spi_read( cmd, readBuf, length);
}


int32_t tlc_reg_write(uint32_t addr, uint8_t *writeBuf, uint32_t length)
{
	unsigned char cmd = 0;

	cmd = CREATE_CMD(QLULPSH_CMD_WRITE, addr);

	return shub_spi_write(cmd, writeBuf, length);
}

/* shub_ahb.c */

int32_t shub_readDMAStatus(uint8_t* buf)
{
	return tlc_reg_read(SPITLC_DMA_STATUS, buf, 1);
}

int32_t read_AHB_status(uint8_t *buf)
{
	return tlc_reg_read(SPITLC_AHB_STATUS, buf, 1);
}


int32_t shub_ahb_write( uint32_t addr, uint8_t *buf, uint32_t sz)
{
    uint8_t lByte = 0;
    int32_t err=SPI_STATUS_OK;
    int counter = 0;
    uint8_t *addrPtr;
    int rty_count = 2;

    do
    {
        err = read_AHB_status(&lByte);
        if(err!=SPI_STATUS_OK)
        {
            return SPI_STATUS_ERR;
        }

        if(counter++ >= MAX_WAIT)
            break;

    } while((lByte & 0x01) != 0);
    // printf("shub_ahb_write after while\n");
    // printf("lByte :%d\n",lByte);

    if (counter++ >= MAX_WAIT)
        return SPI_STATUS_ERR;

RETRY:
    lByte = 0x3;

    err = tlc_reg_write( SPITLC_AHB_ACCESS_CTL, &lByte, ONE_BYTE);

    if(err < 0){

        if(rty_count-- > 0)
            goto RETRY;
        return err;
    }

    //make sure last 2 bits are 0x3
    addr |= 0x3;
    addrPtr = (uint8_t *)(&addr);

    err = tlc_reg_write( SPITLC_MEM_ADDR_BYTE_0, addrPtr , 4);

    if(err < 0){
        if(rty_count-- > 0)
            goto RETRY;
        return err;
    }

    err = tlc_reg_write( SPITLC_MEM_DATA_BYTE_0, buf, sz);

    if(err < 0){
        if(rty_count-- > 0)
            goto RETRY;
        return err;
    }

#if 0 //check AHB status is cleared. Not necessary, since we check at the beginning anyway.

    lByte = 0;
    counter = 0;
    do
    {
        err = read_AHB_status(&lByte);
        if(err!=SPI_STATUS_OK)
        {
            return SPI_STATUS_ERR;
        }

        if(counter++ >= MAX_WAIT)
            break;

    } while((lByte & 0x01) != 0);

    if (counter++ >= MAX_WAIT)
        return SPI_STATUS_ERR;

#endif    
    
    
    return err;
}


int32_t shub_ahb_read(  unsigned int addr, uint8_t *readBuf, uint32_t size)
{
    int32_t err=SPI_STATUS_OK;
    uint8_t dmaStatus=0;
    uint8_t *addrPtr;
    uint32_t dmaReadCount=0, counter=0;

    if (size > MAX_BUF_SIZE || size == 0)
    {
    	return SPI_STATUS_ERR;
    }

    //make sure last 2 bits are 0x3. But will be ignored since using DMA.
    addr |= 0x3; 
#if 0// this not optimum. Write DMA address and burst size together    
    addrPtr = (uint8_t *)(&addr);

    //Send 4 bytes of address to DMA
    err = tlc_reg_write(SPITLC_DMA_ADD0, addrPtr, 4);
    if(err < 0)
         return err;

    /*DMA Read size is in Bytes- 4 : eg to read 8 bytes we need to write 4*/
	dmaReadCount = size - 4;

    //Write size of the bytes to read using DMA
	addrPtr = (uint8_t *)(&dmaReadCount);

    //Send "size of data" DMA need to read
	err = tlc_reg_write( SPITLC_DMA_BRUST_SIZE0, addrPtr, 2);
#else
    uint8_t dma_data[4+2];
    dmaReadCount = size - 4;
    dma_data[0] = (uint8_t)(addr & 0xFF);
    dma_data[1] = (uint8_t)((addr >> 8) & 0xFF);
    dma_data[2] = (uint8_t)((addr >> 16) & 0xFF);
    dma_data[3] = (uint8_t)((addr >> 24) & 0xFF);
    dma_data[4] = (uint8_t)(dmaReadCount & 0xFF);
    dma_data[5] = (uint8_t)((dmaReadCount >> 8) & 0xFF);
    addrPtr = dma_data;

    //Send "address and size of data" DMA need to read
    err = tlc_reg_write( SPITLC_DMA_ADD0, addrPtr, 4+2);
#endif
    if(err < 0)
        return err;

    do
    {
        err=shub_readDMAStatus(&dmaStatus);

        if(err!=SPI_STATUS_OK)
        {
            return err;
        }

        if(counter >= MAX_WAIT)
            break;
    } while(((dmaStatus & 0x01) == 1) && (counter++ < MAX_WAIT)); //Wait untill data is ready

    if (counter >= MAX_WAIT)
    {
        err = SPI_STATUS_ERR;
    }

    // read DMA data.
    if(err == SPI_STATUS_OK)
    {
        err = tlc_reg_read(SPITLC_DMA_READ_DATA, readBuf, size);
    }
    return err;

}


/* ql_spi_interrupts.c */

int32_t ql_spi_read_s3_mem(uint32_t addr, uint8_t *data, uint32_t len)
{
    int32_t ret = SPI_STATUS_ERR;
	ret = shub_ahb_read(addr, data, len);
/*    
    while(ret != SPI_STATUS_OK)
    {
        printf("==");
    }
*/        
    return ret;
}

int32_t ql_spi_write_s3_mem(uint32_t addr, uint8_t *data, uint32_t len)
{
    int32_t ret = SPI_STATUS_ERR;
    uint8_t dummy = 0x02;

    if( shub_ahb_write(addr, data, len) == SPI_STATUS_OK)
    {
        if( tlc_reg_write(SPITLC_SCRATCH_BYTE, &dummy, ONE_BYTE) == SPI_STATUS_OK)
            ret = SPI_STATUS_OK;
    }
/*
    while(ret != SPI_STATUS_OK)
    {
        printf("--");
    }
*/
	return ret;
}


/* qlspi_interrupts.c */

uint32_t gClearIntCnt = 0;
uint32_t gClearIntCnt2 = 0;
void clear_intr_sts_s3(void)
{
    uint32_t reg_clr_val = 0;
    uint32_t reg_read_val =1;
    gClearIntCnt++;
    while(reg_read_val)
    {
      ql_spi_write_s3_mem(SW_INTR_2_REG, (uint8_t *)&reg_clr_val, 4);
      ql_spi_read_s3_mem(SW_INTR_2_REG, (uint8_t *)&reg_read_val, 4);
      gClearIntCnt2++;
    }
}
uint32_t gDisableIntCnt = 0;
void dis_intr_from_s3(void)
{
    uint32_t reg_clr_val = 0;
    gDisableIntCnt++;
    ql_spi_write_s3_mem(SW_INTR_2_EN_AP_REG, (uint8_t *)&reg_clr_val, 4);
		ql_spi_read_s3_mem(SW_INTR_2_EN_AP_REG, (uint8_t *)&reg_clr_val, 4);
}

void dis_intr_to_s3(void)
{
    uint32_t reg_clr_val = 0;
    uint32_t reg_read_val = 1;

    while(reg_read_val)
    {
      ql_spi_write_s3_mem(SW_INTR_1_EN_M4, (uint8_t *)&reg_clr_val, 4);
      ql_spi_read_s3_mem(SW_INTR_1_EN_M4, (uint8_t *)&reg_read_val, 4); 
    }
}
void en_intr_to_s3(void)
{
    uint32_t reg_clr_val = 1;
    uint32_t reg_read_val = 0;

    while(!reg_read_val)
    {
    ql_spi_write_s3_mem(SW_INTR_1_EN_M4, (uint8_t *)&reg_clr_val, 4);
      ql_spi_read_s3_mem(SW_INTR_1_EN_M4, (uint8_t *)&reg_read_val, 4); 
    }
}

uint32_t gEnableIntCnt = 0;
uint32_t gEnableIntCnt2 = 0;
void en_intr_from_s3(void)
{
    uint32_t reg_enable_val = 1;
    uint32_t reg_read_val =0;
    gEnableIntCnt++;
    while(!reg_read_val)
    {
      ql_spi_write_s3_mem(SW_INTR_2_EN_AP_REG, (uint8_t *)&reg_enable_val, 4);
	  ql_spi_read_s3_mem(SW_INTR_2_EN_AP_REG, (uint8_t *)&reg_read_val, 4);
      gEnableIntCnt2++;
    }
}

/* shub_spi.c */
#define MAX_SPI_TX_SIZE (1*1024) //(512) 
#define MAX_SPI_RX_SIZE (3*1024) 
uint8_t gSpiTxBuf[MAX_SPI_TX_SIZE] = {0};
uint8_t gSpiRxBuf[MAX_SPI_RX_SIZE] = {0};

#if 1 //spi read using ESP32 drivers
extern int32_t esp32_tx_spi_data(uint8_t *data, uint32_t len);
extern int32_t esp32_txrx_spi_data(uint8_t *cmd, uint32_t cmd_len, uint8_t *data, uint32_t data_len);
extern int32_t esp32_rx_spi_data(uint8_t *cmd, uint8_t *data, uint32_t data_len);

int32_t shub_spi_write( uint8_t cmd, uint8_t *data, uint32_t len)
{
    uint32_t i = 0;
    int32_t err = 0;
    gSpiTxBuf[0] = cmd; //first value is the 7bit write register number 
    
    //check if it fits in the transmit buffer
    if(len > (MAX_SPI_TX_SIZE - 1))
        return -1;
    
    // printf("TEST :spi write\n");
    //copy the data into the tx buffer after the register address
    for(i=0; i<len; i++)
        gSpiTxBuf[1+i] = data[i];
        
    if(esp32_tx_spi_data(gSpiTxBuf, len+1) != ESP_OK)
    {
        printf("Error while writing to S3 memory \r\n");
        ESP_LOGE(TAG, "Error while writing to S3 memory \r\n");
        err = -1;
    }
    return err;
}


int32_t shub_spi_read( uint8_t reg_addr, uint8_t *data, uint32_t len)
{
    int32_t err = 0;
    //int32_t i = 0;

    memset(gSpiRxBuf, 0, len + 3);
    //2 dummy bytes. 1 + 2 = 3 len
    if( reg_addr == SPITLC_DEVICE_ID_BYTE )	//Spl case where read of device id cmd is a write, so send 0xFF.
        gSpiTxBuf[0] = 0xFF;
    else
        gSpiTxBuf[0] = reg_addr;
    gSpiTxBuf[1] = 0;
    gSpiTxBuf[2] = 0;
    gSpiTxBuf[3] = 0;
    //check if it fits in the transmit buffer
    if(len > (MAX_SPI_RX_SIZE - 3))
        return -2;
/*    if(len == 1)    {
      err = esp32_rx_spi_data(gSpiTxBuf, gSpiRxBuf, 1 + 3);
    } else {
      err = esp32_rx_spi_data(gSpiTxBuf, gSpiRxBuf, len + 3);
    }
*/
    if(esp32_rx_spi_data(gSpiTxBuf, gSpiRxBuf, len + 3) != ESP_OK)
    //if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error while Reading S3 memory \r\n");
        err = -1;
    }
    if(len == 1) {
        if( reg_addr == SPITLC_DEVICE_ID_BYTE )
            *data = gSpiRxBuf[2]; //ID byte reading is different
        else
            *data = gSpiRxBuf[3];
        
    } else {
        memcpy(data, &gSpiRxBuf[3], len);

//    printf("[QL_SPI]: Rx pkt = 0x%02X, 0x%02X, 0x%02X, 0x%02X ",gSpiRxBuf[0],gSpiRxBuf[1],gSpiRxBuf[2],gSpiRxBuf[3]);
//    printf("0x%02X, 0x%02X, 0x%02X, 0x%02X \n",gSpiRxBuf[4],gSpiRxBuf[5],gSpiRxBuf[6],gSpiRxBuf[7]);
//    printf("0x%02X, 0x%02X, 0x%02X, 0x%02X \n",gSpiRxBuf[8],gSpiRxBuf[9],gSpiRxBuf[10],gSpiRxBuf[1]);

    }
    return err;
}
#endif
#if 0 //spI read using S3 drivers
extern SPI_HandleTypeDef spi_m_handle;

int32_t shub_spi_write( uint8_t cmd, uint8_t *data, uint32_t len)
{
  uint32_t i = 0;
	int32_t err = 0;
	gSpiTxBuf[0] = cmd;
	for(i=0; i<len; i++)
		gSpiTxBuf[1+i] = data[i];
	//SPIXfer(gSpiTxBuf, gSpiRxBuf, len + 1);
    spi_m_handle.Init.ucCmdType = CMD_NoResponse;
    if(HAL_SPI_Transmit(&spi_m_handle,gSpiTxBuf, len+1,NULL) != HAL_OK)
	{
		ESP_LOGE(TAG, "Error while writing to S3 memory \r\n");
	}
	return err;
}


int32_t shub_spi_read( uint8_t reg_addr, uint8_t *data, uint32_t len)
{
	int32_t err = 0;
	int32_t i = 0;

	//2 dummy bytes. 1 + 2 = 3 len
	if( reg_addr == SPITLC_DEVICE_ID_BYTE )	//Spl case where read of device id cmd is a write, so send 0xFF.
		gSpiTxBuf[0] = 0xFF;
	else
		gSpiTxBuf[0] = reg_addr;
	//SPIXfer(gSpiTxBuf, gSpiRxBuf, len + 3);
    
	// SJ : Fix : DMA reads only in multiple of 4.
	// When read for 1 byte is called to check DMA status,
	// it gets converted to 4 bytes and first 3 bytes of actual data get lost.
	// TODO : NonDMA read not working for lenght more than 8 bytes
	if(len < 4 )
    {
        spi_m_handle.Init.ucCmdType = CMD_WithResponse;
    }
    else 
    {
        spi_m_handle.Init.ucCmdType = READ_CMD;
    }
    
    if(HAL_SPI_TransmitReceive(&spi_m_handle,gSpiTxBuf, 3, data, len, NULL) != HAL_OK)
    {
        ESP_LOGE(TAG, ("Error while Reading S3 memory \r\n");
    }

	/*
	for(i=0; i<len; i++ )
		data[i] = gSpiRxBuf[3+i];
	*/
	return err;
}
#endif

/* qlspi.c */

//#define NULL 0
uint32_t gVoiceEventCnt = 0;
uint8_t gS3AckPendingFlg = 0;
static QLSPI_Isr_Handler gsQLSpiIsrHandler = (QLSPI_Isr_Handler) NULL;
QL_Status QLSPI_Init (struct QLSPI_Platform *spi_plat)
{
	int32_t ql_status = QL_STATUS_OK;



	return (QL_Status)ql_status;
}



/* This routine should perform read on SPI Bus. Reads data from S3 Memory using SPI Bus
 */
QL_Status QLSPI_Read_S3_Mem (uint32_t addr, uint8_t *data, uint32_t len)
{
	if ((len == 0) || (len > QL_SPI_READ_MAX_LEN))
	{
		return QL_STATUS_ERROR_BAD_PARAMETER;
	}

	if (data == NULL)
	{
		return QL_STATUS_ERROR_BAD_PARAMETER;
	}

	return ((QL_Status)(ql_spi_read_s3_mem(addr, data, len)));
}

/* This routine should perform write on SPI Bus. Write data to S3 Memory using SPI Bus
 */
QL_Status QLSPI_Write_S3_Mem (uint32_t addr, uint8_t *data, uint32_t len)
{
	if ((len == 0) || (len > QL_SPI_WRITE_MAX_LEN))
	{
		return QL_STATUS_ERROR_BAD_PARAMETER;
	}

	if (data == NULL)
	{
		return QL_STATUS_ERROR_BAD_PARAMETER;
	}

	return ((QL_Status)(ql_spi_write_s3_mem(addr, data, len)));
}

/* This routine registers the user defined ISR to QLSPI. This routine need to be called
 * during initialization to register the ISR handler. This handler will be called when
 * interrupt is generated from EOSS3 M4
 */
QL_Status QLSPI_Register_Isr (QLSPI_Isr_Handler handler)
{
    gsQLSpiIsrHandler = handler;
	return QL_STATUS_OK;
}
/*
 * This routine clears the interrupt to EOSS3 M4
 */
QL_Status QLSPI_Clear_Intr(void)
{
	uint32_t trigger_int = 0x0;

    return ((QL_Status)(ql_spi_write_s3_mem( SW_INTR_1_REG, (uint8_t *)(&trigger_int), 4)));
}
/*
 * This routine generate an interrupt to EOSS3 M4
 */
QL_Status QLSPI_Trigger_Intr(void)
{
	uint32_t trigger_int = 0x1;

    //clear it first always
    //QLSPI_Clear_Intr();
    
    //enable the interrupt first
    //en_intr_to_s3();
    dis_intr_to_s3();
    uint32_t reg_read_val =0;
    QL_Status ret = QL_STATUS_OK;
    while(!reg_read_val)
    {
      ret = ql_spi_write_s3_mem( SW_INTR_1_REG, (uint8_t *)(&trigger_int), 4);  
      ret = ql_spi_read_s3_mem(SW_INTR_1_REG, (uint8_t *)&reg_read_val, 4);
    }
    en_intr_to_s3();
    return ret;
    //return ((QL_Status)(ql_spi_write_s3_mem( SW_INTR_1_REG, (uint8_t *)(&trigger_int), 4)));
}
QL_Status QLSPI_Trigger_Intr_2 (void)
{
    uint32_t trigger_int[3] = { 1,0,1 }; //write to 2 registers at once
    uint32_t reg_read_val[3] = {0 };
    QL_Status ret = QL_STATUS_OK;

    ret = ql_spi_write_s3_mem( SW_INTR_1_REG, (uint8_t *)(&trigger_int[0]), 4*3);  
#if 0
    ret = ql_spi_read_s3_mem(SW_INTR_1_REG, (uint8_t *)&reg_read_val[0], 4*3);
    if((reg_read_val[0] == 0) ||  (reg_read_val[2] == 0)) {
        printf("===QLSPI Write ERROR %d, %d ===\n", reg_read_val[0], reg_read_val[2]);
        ret = -1;
    }
#endif
    return ret;
}
//
//Interrupt should already have enabled on Device
//So, just set the interrupt and read back
//
QL_Status QLSPI_Trigger_Intr_3 (void)
{
    uint32_t trigger_int = 0x1;
    uint32_t reg_read_val =0;
    QL_Status ret = QL_STATUS_OK;

    ret = ql_spi_write_s3_mem( SW_INTR_1_REG, (uint8_t *)(&trigger_int), 4);  
    ret |= ql_spi_read_s3_mem(SW_INTR_1_REG, (uint8_t *)&reg_read_val, 4);
    return ret;
}

#if 0
void processS3GPIOInterrupt(void)
{
		static uint8_t btn0_nstate = 0;  // Store new (current) state of button 0
		static uint8_t btn0_ostate = 0;  // Store old (previous) state of button 0
	
		btn0_nstate = nrf_gpio_pin_read(SPI_INTR_PIN);

		if( btn0_nstate != btn0_ostate )
		{
				if( btn0_nstate == 1 )
				{
					nrf_gpio_pin_set(LED_1);
					dis_intr_from_s3();
					if( gsQLSpiIsrHandler )
						gsQLSpiIsrHandler(NULL);
					if( gS3AckPendingFlg == 1 )
						gS3AckPendingFlg = 0;
					clear_intr_sts_s3();
					//nrf_delay_ms(1);
					btn0_nstate = nrf_gpio_pin_read(0);
					if( btn0_nstate == 0 )
						nrf_gpio_pin_clear(LED_1);
					  en_intr_from_s3();
					//nrf_delay_ms(1);
				}
				else
					nrf_gpio_pin_clear(LED_1);
				//btn0_ostate = btn0_nstate;
		}
		btn0_ostate = btn0_nstate;
}
#endif
