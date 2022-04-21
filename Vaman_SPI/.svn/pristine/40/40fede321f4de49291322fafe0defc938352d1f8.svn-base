/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "qlspi_s3.h"
#include "h2d_protocol.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "ble_sec_gatts_demo.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

#define GATTS_TABLE_TAG "SEC_GATTS_DEMO"
#define HEART_PROFILE_NUM                         1
#define HEART_PROFILE_APP_IDX                     0
#define ESP_HEART_RATE_APP_ID                     0x55
#define EXAMPLE_DEVICE_NAME                       "Anil"
#define HEART_RATE_SVC_INST_ID                    0

#define GATTS_DEMO_CHAR_VAL_LEN_MAX               0x40

#define ADV_CONFIG_FLAG                           (1 << 0)
#define SCAN_RSP_CONFIG_FLAG                      (1 << 1)

static const char *TAG = "[app_main]";

extern void esp32_init_ql_spi(void);
extern int h2d_protocol_init(void);
extern signed portBASE_TYPE StartRtosTaskHost( void);
void zero (void);
#include "chilkat_ai_app7_bdm.c"
#define M4_FILE_RAW_DATA ChilkatrawData
#define S3_SENSOR_STORE (0x20041F04)

// #include "pygmy_example_gpio_01.c"
// #define M4_FILE_RAW_DATA rawData
// #define S3_SENSOR_STORE (0x20041F04)

#include "QuickAI_Bluekat_FFE.c"
#define FFE_FILE_RAW_DATA FFErawData

typedef struct
{
  uint32_t magic;
  uint32_t img_nr;
  uint32_t img_type;
  uint32_t start_address;
  uint32_t size;
  uint32_t paddedsize;
  uint8_t  name[16];
  uint8_t  future[24];
}flash_img_entry;


#define PMU_BASE                                (0x40004400)
#define PMU_MISC_POR_0_ADDR                     (PMU_BASE + 0x00)
#define PMU_MISC_POR_2_ADDR                     (PMU_BASE + 0x08)
#define PMU_MISC_POR_3_ADDR                     (PMU_BASE + 0x0C)
#define QLSPI_CHUNK_SIZE                        (128)


#define EOSS3_SPI_MOSI        18 //13 //15 //17 //16 //18
#define EOSS3_SPI_CLK         5
#define EOSS3_SPI_MISO        19
#define EOSS3_SPI_SS_N        27
#define EOSS3_SYS_RST_NEW     26

uint8_t texty[240];

uint32_t faddress = 0;
uint8_t tester = 0;

static uint8_t adv_config_done = 0;

static uint16_t heart_rate_handle_table[HRS_IDX_NB];

static uint8_t test_manufacturer[12]={'O', 'P', 'T', 'I', 'M', 'U', 'S', 'L', 'O', 'G', 'I', 'C'};

static uint8_t sec_service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    //0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x18, 0x0D, 0x00, 0x00,
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e,

};

// static uint8_t service_uuid[16] = {
//     /* LSB <--------------------------------------------------------------------------------> MSB */
//     //first uuid, 16bit, [12],[13] is the value     "6E400001B5A3F393E0A9E50E24DCCA9E"
//     0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e,
// };
static uint8_t rx_char_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value     "6E400001B5A3F393E0A9E50E24DCCA9E"
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e,
};
static uint8_t tx_char_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value     "6E400001B5A3F393E0A9E50E24DCCA9E"
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e,
};


// config adv data
static esp_ble_adv_data_t heart_rate_adv_config = {
    .set_scan_rsp = false,
    .include_txpower = true,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(sec_service_uuid),
    .p_service_uuid = sec_service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// config scan response data
static esp_ble_adv_data_t heart_rate_scan_rsp_config = {
    .set_scan_rsp = true,
    .include_name = true,
    .manufacturer_len = sizeof(test_manufacturer),
    .p_manufacturer_data = test_manufacturer,
};

static esp_ble_adv_params_t heart_rate_adv_params = {
    .adv_int_min        = 0x100,
    .adv_int_max        = 0x100,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_RANDOM,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

void init_ql_components(void)
{
  gpio_install_isr_service(0); //ESP_INTR_FLAG_DEFAULT);

  //enable SPI Master
  esp32_init_ql_spi();
  printf("%s :spi master initialized\n", TAG);

  // init h2d protocol
  // h2d_protocol_init();
  printf("%s :h2d task created\n", TAG);

  //create ql host task
  // StartRtosTaskHost();
  printf("%s :host task created\n", TAG);
  return;
}

int qlfwloader_ffe_setup_clk_power(void) {

    // setup clocks and power domains for FFE fw download:
    volatile uint32_t value = 0x0;
    int counter = 0;
    volatile int ql_status = 0;

    // FFE_PWR_MODE_CFG
    value = 0x0;
    ql_status = QLSPI_Write_S3_Mem(0x40004494, (uint8_t*)&value, 4);

    // FFE_PD_SRC_MASK_N
    value = 0x0;
    ql_status = QLSPI_Write_S3_Mem(0x40004498, (uint8_t*)&value, 4);

    // FFE_WU_SRC_MASK_N
    value = 0x0;
    ql_status = QLSPI_Write_S3_Mem(0x4000449C, (uint8_t*)&value, 4);

    // FFE_FB_PF_SW_WU
    value = 0x01;
    ql_status = QLSPI_Write_S3_Mem(0x40004610, (uint8_t*)&value, 4);

    // FFE_STATUS
    value = 0x0;
    counter = 0;
    while(1) {
        counter++;
        ql_status = QLSPI_Read_S3_Mem(0x40004490, (uint8_t*)&value, 4);
        if(value & 0x1) {
            // ok
            break;
        }
        if(counter >= 100) {
            // error!
            return -1;
        }
    }

    // CLK_CTRL_C_0
    value = 0x204;
    ql_status = QLSPI_Write_S3_Mem(0x40004010, (uint8_t*)&value, 4);

    // CLK_SWITCH_FOR_C0
    value = 0x0;
    ql_status = QLSPI_Write_S3_Mem(0x40004134, (uint8_t*)&value, 4);

    // C01_CLK_GATE
    value = 0x29F;
    ql_status = QLSPI_Write_S3_Mem(0x40004040, (uint8_t*)&value, 4);

    // C08_X4_CLK_GATE
    value = 0x01;
    ql_status = QLSPI_Write_S3_Mem(0x40004048, (uint8_t*)&value, 4);

    // C08_X1_CLK_GATE
    value = 0x0F;
    ql_status = QLSPI_Write_S3_Mem(0x4000404C, (uint8_t*)&value, 4);

    // FFE_SW_RESET
    value = 0x03;
    ql_status = QLSPI_Write_S3_Mem(0x40004084, (uint8_t*)&value, 4);

    // FFE_SW_RESET
    value = 0x0;
    ql_status = QLSPI_Write_S3_Mem(0x40004084, (uint8_t*)&value, 4);
    return ql_status;
}

// #if (USE_nRF_SPI_FLASH == nRF_NO_FLASH_USE_RAM)

/*volatile*/ int qlfwloader2_ffe_load(int flashimgnr) {

    // ffe_fw has a special header which describes the parts in the fw.
    // ffe has 5 part or "sections" : CFG, DM, SM0, SM1, CM.
    // CFG is a legacy section no longer used, however all 5 section sizes are part of the header.
    // the header has the following structure:
    // <ignored_bytes>$<ignored_bytes>\n<4B CFG size><4B DM size><4B SM0 size><4B SM1 size><4B CM size>
    // followed by this, the actual fw content starts, in the same order, with specified bytes in each section

    // ensure that the FFE clocks and power domains have been set correctly at this point.
    // call the qlfwloader_ffe_setup_clk_power()

    volatile int ql_status = 0;

    //uint8_t *ffe_fw;
    volatile uint32_t ffe_fw_size;
    
    volatile uint32_t ffe_fw_section_cfg_size = 0; // dinosaur, extinct now.
    volatile uint32_t ffe_fw_section_dm_size = 0;
    volatile uint32_t ffe_fw_section_sm0_size = 0;
    volatile uint32_t ffe_fw_section_sm1_size = 0;
    volatile uint32_t ffe_fw_section_cm_size = 0;

    volatile uint32_t FFE_CFG_ADDR = 0x2007FE80; // dinosaur, extinct now.
    volatile uint32_t FFE_DM_ADDR = 0x40040000;
    volatile uint32_t FFE_SM0_ADDR  = 0x40044000;
    volatile uint32_t FFE_SM1_ADDR  = 0x40048000;
    volatile uint32_t FFE_CM_ADDR  = 0x40050000;
    flash_img_entry immg2;

    volatile uint32_t curr_ffe_fw_ptr = 0;
    volatile uint32_t read_out_data_ptr = 0;

    ql_status = qlfwloader_ffe_setup_clk_power();

    if (ql_status)
      return ql_status;

    memset(&immg2,0,sizeof(immg2));
    memset(texty,0,sizeof(texty));
    //spif_test(0x10,256+flashimgnr*64,sizeof(flash_img_entry),texty);
    //memcpy((uint8_t *)&immg2,&texty[3],sizeof(flash_img_entry));
    immg2.size = sizeof(FFE_FILE_RAW_DATA);
    immg2.start_address = 0;
    memset(texty,0,sizeof(texty));
    
   /* if (immg2.magic != 0x43474C4F)
      return -1;

    if (strncmp(immg2.name,"FFE",sizeof("FFE")) != 0)
      return -2;*/

    //spif_test(0x10,(immg2.start_address),128,(uint8_t *)texty);
    memcpy(&texty[3],FFE_FILE_RAW_DATA,QLSPI_CHUNK_SIZE);
    read_out_data_ptr = ql_status+QLSPI_CHUNK_SIZE;
    ffe_fw_size = ql_status+immg2.size;

    // start at the beginning
    for (curr_ffe_fw_ptr = 0;curr_ffe_fw_ptr<QLSPI_CHUNK_SIZE;curr_ffe_fw_ptr++)
    {
      // seek to the location right after the '\n'
        if (texty[curr_ffe_fw_ptr+3] == 0x0A)
          break;
    }

    // if (curr_ffe_fw_ptr > read_out_data_ptr)
    //     return -3;

    curr_ffe_fw_ptr++;
    if ((read_out_data_ptr-curr_ffe_fw_ptr) < 20)
    {
        return -4;
    }

    // <4B CFG size>
    ffe_fw_section_cfg_size = *(uint32_t*)(&texty[curr_ffe_fw_ptr+3]);
    curr_ffe_fw_ptr += 4;

    // <4B DM size>
    ffe_fw_section_dm_size = *(uint32_t*)(&texty[curr_ffe_fw_ptr+3]);
    curr_ffe_fw_ptr += 4;

    // <4B SM0 size>
    ffe_fw_section_sm0_size = *(uint32_t*)(&texty[curr_ffe_fw_ptr+3]);
    curr_ffe_fw_ptr += 4;

    // <4B SM1 size>
    ffe_fw_section_sm1_size = *(uint32_t*)(&texty[curr_ffe_fw_ptr+3]);
    curr_ffe_fw_ptr += 4;

    // <4B CM size>
    ffe_fw_section_cm_size = *(uint32_t*)(&texty[curr_ffe_fw_ptr+3]);
    curr_ffe_fw_ptr += 4;
    read_out_data_ptr = curr_ffe_fw_ptr;
    int write_offset = 0;

    // write CFG section:
    if(ffe_fw_section_cfg_size > 0) {
        for (write_offset=0;write_offset<ffe_fw_section_cfg_size;write_offset+=4)
        {
            //spif_test(0x10,(immg2.start_address+curr_ffe_fw_ptr),4,(uint8_t *)texty);
            memcpy(&texty[3],&FFE_FILE_RAW_DATA[immg2.start_address+curr_ffe_fw_ptr],4);
            QLSPI_Write_S3_Mem(FFE_CFG_ADDR+write_offset, &texty[3], 4);
            curr_ffe_fw_ptr+=4;
        }
    }

    // write DM section:
    if(ffe_fw_section_dm_size > 0) {
        for (write_offset=0;write_offset<ffe_fw_section_dm_size;write_offset+=4)
        {
            //spif_test(0x10,(immg2.start_address+curr_ffe_fw_ptr),4,(uint8_t *)texty);
            memcpy(&texty[3],&FFE_FILE_RAW_DATA[immg2.start_address+curr_ffe_fw_ptr],4);
            QLSPI_Write_S3_Mem(FFE_DM_ADDR+write_offset, &texty[3], 4);
            curr_ffe_fw_ptr += 4;
        }
    }

    // write SM0 section:
    if(ffe_fw_section_sm0_size > 0) {
        for (write_offset=0;write_offset<ffe_fw_section_sm0_size;write_offset+=4)
        {
            //spif_test(0x10,(immg2.start_address+curr_ffe_fw_ptr),4,(uint8_t *)texty);
            memcpy(&texty[3],&FFE_FILE_RAW_DATA[immg2.start_address+curr_ffe_fw_ptr],4);
            QLSPI_Write_S3_Mem(FFE_SM0_ADDR+write_offset, &texty[3], 4);
            curr_ffe_fw_ptr += 4;
        }
    }

   // write SM1 section:
    if(ffe_fw_section_sm1_size > 0) {
        for (write_offset=0;write_offset<ffe_fw_section_sm1_size;write_offset+=4)
        {
            //spif_test(0x10,(immg2.start_address+curr_ffe_fw_ptr),4,(uint8_t *)texty);
            memcpy(&texty[3],&FFE_FILE_RAW_DATA[immg2.start_address+curr_ffe_fw_ptr],4);
            QLSPI_Write_S3_Mem(FFE_SM1_ADDR+write_offset, &texty[3], 4);
            curr_ffe_fw_ptr += 4;
        }
    }

   // write CM section:
    if(ffe_fw_section_cm_size > 0) {
        for (write_offset=0;write_offset<ffe_fw_section_cm_size;write_offset+=4)
        {
            //spif_test(0x10,(immg2.start_address+curr_ffe_fw_ptr),4,(uint8_t *)texty);
            memcpy(&texty[3],&FFE_FILE_RAW_DATA[immg2.start_address+curr_ffe_fw_ptr],4);
            QLSPI_Write_S3_Mem(FFE_CM_ADDR+write_offset, &texty[3], 4);
            curr_ffe_fw_ptr += 4;
        }
    }

    return ql_status;
}

int boot_eoss3(uint8_t smartphone_mode_en, uint8_t debug_mode_en)
{
    // gpio_set_direction(EOSS3_PWR_CTL, GPIO_MODE_OUTPUT);    

    // gpio_set_direction(EOSS3_HOST_IRQ, GPIO_MODE_OUTPUT); //not specified in esp32     
    gpio_set_direction(EOSS3_SPI_MOSI, GPIO_MODE_OUTPUT);
    gpio_set_direction(EOSS3_SPI_CLK, GPIO_MODE_OUTPUT);
    gpio_set_direction(EOSS3_SPI_MISO, GPIO_MODE_OUTPUT);
    gpio_set_direction(EOSS3_SPI_SS_N, GPIO_MODE_OUTPUT);
    gpio_set_direction(EOSS3_SYS_RST_NEW, GPIO_MODE_OUTPUT);
    
    // enable power to EOSS3
    //nrf_gpio_pin_write(EOSS3_PWR_CTL, 1);

    // Test Code to check if RST goes High and Low
    //nrf_gpio_pin_write(EOSS3_SYS_RST_N,1);
    //nrf_delay_ms(100);

    // Hold EOSS3 in reset
    gpio_set_level(EOSS3_SYS_RST_NEW, 0);

    // configure EOSS3 bootstraps: 
    // IO_19 = 0, IO_20 = 0 (wearable mode for boot from flash)
    // IO_19 = 0, IO_20 = 1 (smartphone mode, wait for host to load code)
    // IO_19 = 1, IO_20 = 1 (fully debuggable)    
    
    gpio_set_level(EOSS3_SPI_MOSI, debug_mode_en);        // EOSS3 IO_19
    gpio_set_level(EOSS3_SPI_SS_N, smartphone_mode_en);   // EOSS3 IO_20
    
    // wait for the GPIOs levels to settle    
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // Release EOSS3 from reset
    gpio_set_level(EOSS3_SYS_RST_NEW, 1);

    // if wearable mode, wait until code starts running on the EOSS3 M4 core, remove this after EOSS3-nRF52832 sync code in place.
    //nrf_delay_ms(4000); // can be lower? depends on BL configuration on the EOSS3.
    // NRF_LOG_INFO("EOSS3 M4 code should be running now");NRF_LOG_FLUSH();
    vTaskDelay(5 / portTICK_PERIOD_MS);
    return 0;
}



 



void s3_gpio_config(int gpio)
{
    if (gpio <=7)
    {
        if (gpio ==0)
        {
            //For GPIO0 Configuration    This doesn't Work
           //  faddress = 259 ; // IOMUX_IO_S
             //printf("write 0x40004C00\n");
             //QLSPI_Write_S3_Mem(0x40004C00,(unsigned char *)&faddress,4);
            // vTaskDelay(50 / portTICK_PERIOD_MS);
            // tester = 0;
             ////printf("Read 0x40004C00 \n");
            // QLSPI_Read_S3_Mem(0x40004C00,(unsigned char *)(&tester),4);
            // vTaskDelay(50 / portTICK_PERIOD_MS);
            // printf("tester: %d\n",tester);

        }
        else if (gpio ==1)
        {
            //For GPIO1 Configuration    This doesn't Work
             //faddress = 259 ; // IOMUX_IO_S
             //printf("write 0x40004C00\n");
             //QLSPI_Write_S3_Mem(0x40004C00,(unsigned char *)&faddress,4);
             //vTaskDelay(50 / portTICK_PERIOD_MS);
            /// tester = 0;
             //printf("Read 0x40004C00 \n");
             //QLSPI_Read_S3_Mem(0x40004C00,(unsigned char *)(&tester),4);
            // vTaskDelay(50 / portTICK_PERIOD_MS);
            // printf("tester: %d\n",tester);

        }
        else if (gpio ==2)
        {
            //For GPIO2 Configuration    This doesn't Work
            //faddress = 259 ; // IOMUX_IO_S
           // printf("write 0x40004C00\n");
           // QLSPI_Write_S3_Mem(0x40004C00,(unsigned char *)&faddress,4);
            //vTaskDelay(50 / portTICK_PERIOD_MS);
           // tester = 0;
            //printf("Read 0x40004C00 \n");
            //QLSPI_Read_S3_Mem(0x40004C00,(unsigned char *)(&tester),4);
           // vTaskDelay(50 / portTICK_PERIOD_MS);
           // printf("tester: %d\n",tester);
        }
        else if (gpio ==3)
        {
            //For GPIO3 Configuration    This doesn't Work
         //    faddress = 259 ; // IOMUX_IO_S
           //  printf("write 0x40004C00\n");
           //  QLSPI_Write_S3_Mem(0x40004C00,(unsigned char *)&faddress,4);
           //  vTaskDelay(50 / portTICK_PERIOD_MS);
            // tester = 0;
           //  printf("Read 0x40004C00 \n");
            // QLSPI_Read_S3_Mem(0x40004C00,(unsigned char *)(&tester),4);
            // vTaskDelay(50 / portTICK_PERIOD_MS);
            // printf("tester: %d\n",tester);
        }
        else if(gpio ==4)
        {
            //For GPIO4 Configuration  
            faddress = 259 ; // IOMUX_IO_S
            printf("write 0x40004C48\n");
            QLSPI_Write_S3_Mem(0x40004C48,(unsigned char *)&faddress,4);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            tester = 0;
            printf("Read 0x40004C48 \n");
            QLSPI_Read_S3_Mem(0x40004C48,(unsigned char *)(&tester),4);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            printf("tester: %d\n",tester);
        }
        else if(gpio ==5) 
        {
            //For GPIO5 Configuration  
            faddress = 259 ; // IOMUX_IO_S
            printf("write 0x40004C54\n");
            QLSPI_Write_S3_Mem(0x40004C54,(unsigned char *)&faddress,4);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            tester = 0;
            printf("Read 0x40004C54 \n");
            QLSPI_Read_S3_Mem(0x40004C54,(unsigned char *)(&tester),4);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            printf("tester: %d\n",tester);
        }
        else if (gpio ==6)
        {
            //For GPIO6 Configuration  
            faddress = 259 ; // IOMUX_IO_S
            printf("write 0x40004C58\n");
            QLSPI_Write_S3_Mem(0x40004C58,(unsigned char *)&faddress,4);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            tester = 0;
            printf("Read 0x40004C58 \n");
            QLSPI_Read_S3_Mem(0x40004C58,(unsigned char *)(&tester),4);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            printf("tester: %d\n",tester);
        }
        
        else if (gpio ==7)
        {
            //For GPIO7 Configuration  
            faddress = 259 ; // IOMUX_IO_S
            printf("write 0x40004C5C\n");
            QLSPI_Write_S3_Mem(0x40004C5C,(unsigned char *)&faddress,4);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            tester = 0;
            printf("Read 0x40004C5C \n");
            QLSPI_Read_S3_Mem(0x40004C5C,(unsigned char *)(&tester),4);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            printf("tester: %d\n",tester);
        }
    }

    faddress = 0 ;      // IOMUX_IO_SEL_REG
    printf("write 0x40004D74\n");
    QLSPI_Write_S3_Mem(0x40004D74,(unsigned char *)&faddress,4);
    vTaskDelay(50 / portTICK_PERIOD_MS);

    tester = 0;
    printf("Read 0x40004D74 \n");
    QLSPI_Read_S3_Mem(0x40004D74,(unsigned char *)(&tester),4);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    printf("tester: %d\n",tester);
}

void zero (void)
{
    //printf("Farword -- GPIO 4 & GPIO 6\n");
    printf("zero");
    faddress = 0; // IOMUX_IO_S
    QLSPI_Write_S3_Mem(0x40005104,(unsigned char *)&faddress,4);
    // vTaskDelay(50 / portTICK_PERIOD_MS);
    // tester = 0;
    // QLSPI_Read_S3_Mem(0x40005104,(unsigned char *)(&tester),4);
    // vTaskDelay(50 / portTICK_PERIOD_MS);
    // printf("tester: %d\n",tester);
}
void first(void)
{
      printf("first");
    //printf("Reverse -- GPIO 5 & GPIO 7\n");
    faddress = (1<<7); // IOMUX_IO_S
    QLSPI_Write_S3_Mem(0x40005104,(unsigned char *)&faddress,4);
    // vTaskDelay(50 / portTICK_PERIOD_MS);
    // tester = 0;
    // QLSPI_Read_S3_Mem(0x40005104,(unsigned char *)(&tester),4);
    // vTaskDelay(50 / portTICK_PERIOD_MS);
    // printf("tester: %d\n",tester);
}

void second(void)
{
   // printf("Left Turn --GPIO 4\n");
   printf("second");
    faddress = (1<<6) ; // IOMUX_IO_S
    QLSPI_Write_S3_Mem(0x40005104,(unsigned char *)&faddress,4);
    // vTaskDelay(50 / portTICK_PERIOD_MS);
    // tester = 0;
    // QLSPI_Read_S3_Mem(0x40005104,(unsigned char *)(&tester),4);
    // vTaskDelay(50 / portTICK_PERIOD_MS);
    // printf("tester: %d\n",tester);
}

void third(void)
{
    //printf("Right Turn --GPIO 5\n");
    printf("third");
    faddress = ((1<<6)|(1<<7)) ; // IOMUX_IO_S
    QLSPI_Write_S3_Mem(0x40005104,(unsigned char *)&faddress,4);
    // vTaskDelay(50 / portTICK_PERIOD_MS);
    // tester = 0;
    // QLSPI_Read_S3_Mem(0x40005104,(unsigned char *)(&tester),4);
    // vTaskDelay(50 / portTICK_PERIOD_MS);
    // printf("tester: %d\n",tester);
}

void four(void)
{
   // printf("Stop\n");
    printf("four");
    faddress = ((1<<5))  ;         // 128-GPIO7 64-RLED, 32-GLED, 16-BLED    // MISC_IO_OUTPUT  ALL RGB LED
    QLSPI_Write_S3_Mem(0x40005104,(unsigned char *)&faddress,4);
    // vTaskDelay(50 / portTICK_PERIOD_MS);
    // tester = 0;
    // QLSPI_Read_S3_Mem(0x40005104,(unsigned char *)(&tester),4);
    // vTaskDelay(50 / portTICK_PERIOD_MS);
    // printf("tester: %d\n",tester);
}

void five(void){
   printf("five");
  faddress = ((1<<5)|(1<<7))  ;         // 128-GPIO7 64-RLED, 32-GLED, 16-BLED    // MISC_IO_OUTPUT  ALL RGB LED
    QLSPI_Write_S3_Mem(0x40005104,(unsigned char *)&faddress,4);
}

void six(void){
  printf("six");
  faddress = ((1<<5)|(1<<6))  ;         // 128-GPIO7 64-RLED, 32-GLED, 16-BLED    // MISC_IO_OUTPUT  ALL RGB LED
    QLSPI_Write_S3_Mem(0x40005104,(unsigned char *)&faddress,4);
}

void seven(void){
   printf("seven");
  faddress = ((1<<5)|(1<<6)|(1<<7))  ;         // 128-GPIO7 64-RLED, 32-GLED, 16-BLED    // MISC_IO_OUTPUT  ALL RGB LED
    QLSPI_Write_S3_Mem(0x40005104,(unsigned char *)&faddress,4);
}

void eight(void){
  printf("eight");
  faddress = (1<<4) ;         // 128-GPIO7 64-RLED, 32-GLED, 16-BLED    // MISC_IO_OUTPUT  ALL RGB LED
    QLSPI_Write_S3_Mem(0x40005104,(unsigned char *)&faddress,4);
}
void nine(void){
  printf("nine");
  faddress = ((1<<4)|(1<<7))  ;         // 128-GPIO7 64-RLED, 32-GLED, 16-BLED    // MISC_IO_OUTPUT  ALL RGB LED
    QLSPI_Write_S3_Mem(0x40005104,(unsigned char *)&faddress,4);
}



static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst heart_rate_profile_tab[HEART_PROFILE_NUM] = {
    [HEART_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,      /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },

};

/*
 *  Heart Rate PROFILE ATTRIBUTES
 ****************************************************************************************
 */

/// Heart Rate Sensor Service
// static const uint16_t heart_rate_svc = ESP_GATT_UUID_HEART_RATE_SVC;

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
/*static const */uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
// static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE;
// static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE|ESP_GATT_CHAR_PROP_BIT_READ;

/// Heart Rate Sensor Service - Heart Rate Measurement Characteristic, notify 
/*static const*/ uint16_t heart_rate_meas_uuid = ESP_GATT_HEART_RATE_MEAS;
static const uint8_t heart_measurement_ccc[2] ={ 0x00, 0x00};


/// Heart Rate Sensor Service -Body Sensor Location characteristic, read
// static const uint16_t body_sensor_location_uuid = ESP_GATT_BODY_SENSOR_LOCATION;
// static const uint8_t body_sensor_loc_val[1] = {0x00};


/// Heart Rate Sensor Service - Heart Rate Control Point characteristic, write&read
static const uint16_t heart_rate_ctrl_point = ESP_GATT_HEART_RATE_CNTL_POINT;
static const uint8_t heart_ctrl_point[1] = {0x00};

/// Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t heart_rate_gatt_db[HRS_IDX_NB] =
{
    // Heart Rate Service Declaration
    [HRS_IDX_SVC]                    =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(sec_service_uuid), (uint8_t *)&sec_service_uuid}},
    //   {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
    //   sizeof(uint16_t), sizeof(heart_rate_svc), (uint8_t *)&heart_rate_svc}},

    // Heart Rate Measurement Characteristic Declaration
    [HRS_IDX_HR_MEAS_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_notify}},
    // {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    //   CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE,(uint8_t *)&rx_char_uuid}},

    // Heart Rate Measurement Characteristic Value
    [HRS_IDX_HR_MEAS_VAL]             =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&tx_char_uuid, ESP_GATT_PERM_READ,
      HRPS_HT_MEAS_MAX_LEN,0, NULL}},
    //   {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&heart_rate_meas_uuid, ESP_GATT_PERM_READ,
    //   HRPS_HT_MEAS_MAX_LEN,0, NULL}},

    // Heart Rate Measurement Characteristic - Client Characteristic Configuration Descriptor
    [HRS_IDX_HR_MEAS_NTF_CFG]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
      sizeof(uint16_t),sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},

    // Body Sensor Location Characteristic Declaration
    [HRS_IDX_BOBY_SENSOR_LOC_CHAR]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},

    // // Body Sensor Location Characteristic Value
    // [HRS_IDX_BOBY_SENSOR_LOC_VAL]   =
    // {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&body_sensor_location_uuid, ESP_GATT_PERM_READ_ENCRYPTED,
    //   sizeof(uint8_t), sizeof(body_sensor_loc_val), (uint8_t *)body_sensor_loc_val}},

    // Heart Rate Control Point Characteristic Declaration
    [HRS_IDX_HR_CTNL_PT_CHAR]          =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&rx_char_uuid, ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},

    // Heart Rate Control Point Characteristic Value
    [HRS_IDX_HR_CTNL_PT_VAL]             =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&heart_rate_ctrl_point, ESP_GATT_PERM_WRITE /*ESP_GATT_PERM_WRITE_ENCRYPTED|ESP_GATT_PERM_READ_ENCRYPTED*/,
     sizeof(uint8_t), sizeof(heart_ctrl_point), (uint8_t *)heart_ctrl_point}},
};
static char *esp_key_type_to_str(esp_ble_key_type_t key_type)
{
   char *key_str = NULL;
   switch(key_type) {
    case ESP_LE_KEY_NONE:
        key_str = "ESP_LE_KEY_NONE";
        break;
    case ESP_LE_KEY_PENC:
        key_str = "ESP_LE_KEY_PENC";
        break;
    case ESP_LE_KEY_PID:
        key_str = "ESP_LE_KEY_PID";
        break;
    case ESP_LE_KEY_PCSRK:
        key_str = "ESP_LE_KEY_PCSRK";
        break;
    case ESP_LE_KEY_PLK:
        key_str = "ESP_LE_KEY_PLK";
        break;
    case ESP_LE_KEY_LLK:
        key_str = "ESP_LE_KEY_LLK";
        break;
    case ESP_LE_KEY_LENC:
        key_str = "ESP_LE_KEY_LENC";
        break;
    case ESP_LE_KEY_LID:
        key_str = "ESP_LE_KEY_LID";
        break;
    case ESP_LE_KEY_LCSRK:
        key_str = "ESP_LE_KEY_LCSRK";
        break;
    default:
        key_str = "INVALID BLE KEY TYPE";
        break;

   }

   return key_str;
}

static char *esp_auth_req_to_str(esp_ble_auth_req_t auth_req)
{
   char *auth_str = NULL;
   switch(auth_req) {
    case ESP_LE_AUTH_NO_BOND:
        auth_str = "ESP_LE_AUTH_NO_BOND";
        break;
    case ESP_LE_AUTH_BOND:
        auth_str = "ESP_LE_AUTH_BOND";
        break;
    case ESP_LE_AUTH_REQ_MITM:
        auth_str = "ESP_LE_AUTH_REQ_MITM";
        break;
    case ESP_LE_AUTH_REQ_BOND_MITM:
        auth_str = "ESP_LE_AUTH_REQ_BOND_MITM";
        break;
    case ESP_LE_AUTH_REQ_SC_ONLY:
        auth_str = "ESP_LE_AUTH_REQ_SC_ONLY";
        break;
    case ESP_LE_AUTH_REQ_SC_BOND:
        auth_str = "ESP_LE_AUTH_REQ_SC_BOND";
        break;
    case ESP_LE_AUTH_REQ_SC_MITM:
        auth_str = "ESP_LE_AUTH_REQ_SC_MITM";
        break;
    case ESP_LE_AUTH_REQ_SC_MITM_BOND:
        auth_str = "ESP_LE_AUTH_REQ_SC_MITM_BOND";
        break;
    default:
        auth_str = "INVALID BLE AUTH REQ";
        break;
   }

   return auth_str;
}

static void show_bonded_devices(void)
{
    int dev_num = esp_ble_get_bond_device_num();

    esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    ESP_LOGI(GATTS_TABLE_TAG, "Bonded devices number : %d\n", dev_num);

    ESP_LOGI(GATTS_TABLE_TAG, "Bonded devices list : %d\n", dev_num);
    for (int i = 0; i < dev_num; i++) {
        esp_log_buffer_hex(GATTS_TABLE_TAG, (void *)dev_list[i].bd_addr, sizeof(esp_bd_addr_t));
    }

    free(dev_list);
}

static void __attribute__((unused)) remove_all_bonded_devices(void)
{
    int dev_num = esp_ble_get_bond_device_num();

    esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    for (int i = 0; i < dev_num; i++) {
        esp_ble_remove_bond_device(dev_list[i].bd_addr);
    }

    free(dev_list);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    ESP_LOGV(GATTS_TABLE_TAG, "GAP_EVT, event %d\n", event);

    switch (event) {
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&heart_rate_adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&heart_rate_adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed, error status = %x", param->adv_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTS_TABLE_TAG, "advertising start success");
        break;
    case ESP_GAP_BLE_PASSKEY_REQ_EVT:                          /* passkey request event */
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_PASSKEY_REQ_EVT");
        /* Call the following function to input the passkey which is displayed on the remote device */
        //esp_ble_passkey_reply(heart_rate_profile_tab[HEART_PROFILE_APP_IDX].remote_bda, true, 0x00);anil*/
       break;
    case ESP_GAP_BLE_OOB_REQ_EVT: {
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_OOB_REQ_EVT");
        uint8_t tk[16] = {1}; //If you paired with OOB, both devices need to use the same tk
        esp_ble_oob_req_reply(param->ble_security.ble_req.bd_addr, tk, sizeof(tk));
        break;
    }
    case ESP_GAP_BLE_LOCAL_IR_EVT:                              /* BLE local IR event */
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_LOCAL_IR_EVT");
        break;
    case ESP_GAP_BLE_LOCAL_ER_EVT:                              /* BLE local ER event */
       ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_LOCAL_ER_EVT");
        break;
    case ESP_GAP_BLE_NC_REQ_EVT:  
        /* The app will receive this evt when the IO has DisplayYesNO capability and the peer device IO also has DisplayYesNo capability.
        show the passkey number to the user to confirm it with the number displayed by peer device. */
       esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_NC_REQ_EVT, the passkey Notify number:%d", param->ble_security.key_notif.passkey);
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT: 
        /* send the positive(true) security response to the peer device to accept the security request.
        If not accept the security request, should send the security response with negative(false) accept value*/
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:  ///the app will receive this evt when the IO  has Output capability and the peer device IO has Input capability.
        ///show the passkey number to the user to input it in the peer device.
        ESP_LOGI(GATTS_TABLE_TAG, "The passkey Notify number:%06d", param->ble_security.key_notif.passkey);
        break;
    case ESP_GAP_BLE_KEY_EVT:
        //shows the ble key info share with peer device to the user.
        ESP_LOGI(GATTS_TABLE_TAG, "key type = %s", esp_key_type_to_str(param->ble_security.ble_key.key_type));
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT: {
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTS_TABLE_TAG, "remote BD_ADDR: %08x%04x",\
                (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(GATTS_TABLE_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(GATTS_TABLE_TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        if(!param->ble_security.auth_cmpl.success) {
            ESP_LOGI(GATTS_TABLE_TAG, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
        } else {
            ESP_LOGI(GATTS_TABLE_TAG, "auth mode = %s",esp_auth_req_to_str(param->ble_security.auth_cmpl.auth_mode));
        }
        show_bonded_devices();
        break;
    }
    case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT: {
        ESP_LOGD(GATTS_TABLE_TAG, "ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT status = %d", param->remove_bond_dev_cmpl.status);
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_REMOVE_BOND_DEV");
        ESP_LOGI(GATTS_TABLE_TAG, "-----ESP_GAP_BLE_REMOVE_BOND_DEV----");
        esp_log_buffer_hex(GATTS_TABLE_TAG, (void *)param->remove_bond_dev_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTS_TABLE_TAG, "------------------------------------");
        break;
    }
    case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
        if (param->local_privacy_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTS_TABLE_TAG, "config local privacy failed, error status = %x", param->local_privacy_cmpl.status);
            break;
        }

        esp_err_t ret = esp_ble_gap_config_adv_data(&heart_rate_adv_config);
        if (ret){
            ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
        }else{
            adv_config_done |= ADV_CONFIG_FLAG;
        }

        ret = esp_ble_gap_config_adv_data(&heart_rate_scan_rsp_config);
        if (ret){
            ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
        }else{
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
        }

        break;
    default:
        break;
    }
} 

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGV(GATTS_TABLE_TAG, "event = %x\n",event);
    switch (event) {
        case ESP_GATTS_REG_EVT:
            esp_ble_gap_set_device_name(EXAMPLE_DEVICE_NAME);
            //generate a resolvable random address
            esp_ble_gap_config_local_privacy(true);
            esp_ble_gatts_create_attr_tab(heart_rate_gatt_db, gatts_if,
                                      HRS_IDX_NB, HEART_RATE_SVC_INST_ID);
            break;
        case ESP_GATTS_READ_EVT: 
            ESP_LOGI(GATTS_TABLE_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = 4;
            rsp.attr_value.value[0] = 0xde;
            rsp.attr_value.value[1] = 0xed;
            rsp.attr_value.value[2] = 0xbe;
            rsp.attr_value.value[3] = 0xef;
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                        ESP_GATT_OK, &rsp);
            break;
        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT, write value:");
            esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
            printf("len : %d %d %d\n",param->write.len ,param->write.value[5],param->write.value[6]);

            if (param->write.value[5] == 0x20)
            { 
               
                 six();
               // printf("Square\n");
                
            }
            else if (param->write.value[5] == 0x08)
            {
                eight();
                //printf("Circle\n");
            }
            else if (param->write.value[5] == 0x04)
            {
                seven();
                //printf("Triangle\n");
            }
            else if (param->write.value[5] == 0x10)
            {   
               
                nine();
                //printf("Cross\n");
            }
            else if (param->write.value[5] == 0x02)
            {
               
                four();
               // printf("Select\n");
            }
            else if (param->write.value[5] == 0x01)
            {
               
                five();
                //printf("start\n");
            }
            else if (param->write.value[6] == 0x04)
            {
              
                  first();
                //printf("Left\n");
            }
            else if (param->write.value[6] == 0x08)
            {
                
                 third();
                //printf("Right\n");
            }
            else if (param->write.value[6] == 0x01)
            {
               zero();
                
                
                //printf("UP\n");
            }
            else if (param->write.value[6] == 0x02)
            {
               
                second();
                //printf("Down\n"); 
            }
            break;

        case ESP_GATTS_EXEC_WRITE_EVT:
            break;
        case ESP_GATTS_MTU_EVT:
            break;
        case ESP_GATTS_CONF_EVT:
            break;
        case ESP_GATTS_UNREG_EVT:
            break;
        case ESP_GATTS_DELETE_EVT:
            break;
        case ESP_GATTS_START_EVT:
            break;
        case ESP_GATTS_STOP_EVT:
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT");
            /* start security connect with peer device when receive the connect event sent by the master */
            esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
            /* start advertising again when missing the connect */
            esp_ble_gap_start_advertising(&heart_rate_adv_params);
            break;
        case ESP_GATTS_OPEN_EVT:
            break;
        case ESP_GATTS_CANCEL_OPEN_EVT:
            break;
        case ESP_GATTS_CLOSE_EVT:
            break;
        case ESP_GATTS_LISTEN_EVT:
            break;
        case ESP_GATTS_CONGEST_EVT:
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
            ESP_LOGI(GATTS_TABLE_TAG, "The number handle = %x",param->add_attr_tab.num_handle);
            if (param->create.status == ESP_GATT_OK){
                if(param->add_attr_tab.num_handle == HRS_IDX_NB) {
                    memcpy(heart_rate_handle_table, param->add_attr_tab.handles,
                    sizeof(heart_rate_handle_table));
                    esp_ble_gatts_start_service(heart_rate_handle_table[HRS_IDX_SVC]);
                }else{
                    ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)",
                         param->add_attr_tab.num_handle, HRS_IDX_NB);
                }
            }else{
                ESP_LOGE(GATTS_TABLE_TAG, " Create attribute table failed, error code = %x", param->create.status);
            }
        break;
    }

        default:
           break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            heart_rate_profile_tab[HEART_PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d\n",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < HEART_PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == heart_rate_profile_tab[idx].gatts_if) {
                if (heart_rate_profile_tab[idx].gatts_cb) {
                    heart_rate_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}


void app_main(void)
{
   
    uint32_t faddress = 0;
	esp_err_t ret;

    
	
	 // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
	
	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth", __func__);
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(ESP_HEART_RATE_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;     //bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           //set the IO capability to No output No input
    uint8_t key_size = 16;      //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    //set static passkey
    uint32_t passkey = 123456;
    uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
    uint8_t oob_support = ESP_BLE_OOB_DISABLE;
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));
    /* If your BLE device acts as a Slave, the init_key means you hope which types of key of the master should distribute to you,
    and the response key means which key you can distribute to the master;
    If your BLE device acts as a master, the response key means you hope which types of key of the slave should distribute to you,
    and the init key means which key you can distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    /* Just show how to clear all the bonded devices
     * Delay 30s, clear all the bonded devices
     *
     * vTaskDelay(30000 / portTICK_PERIOD_MS);
     * remove_all_bonded_devices();
     */
	
	ESP_LOGE(GATTS_TABLE_TAG, "Ble Initialization Done\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ", CONFIG_IDF_TARGET, chip_info.cores,
    (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "", (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);
    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024), (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

    // wait for the GPIOs levels to settle    
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    boot_eoss3(1, 0);
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    init_ql_components();

    uint8_t tester = 0;
    faddress= 0;
    // printf("write at PMU_MISC_POR_0_ADDR\n");
    QLSPI_Write_S3_Mem(PMU_MISC_POR_0_ADDR,(unsigned char *)&faddress,4);
    vTaskDelay(50 / portTICK_PERIOD_MS);

    printf("Read at 0x40005484 \n");
    QLSPI_Read_S3_Mem(0x40005484,(unsigned char *)(&tester),4);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    printf("tester: %d\n",tester);

    // Initilise GPIOs
    
    s3_gpio_config(4);
    s3_gpio_config(5);
    s3_gpio_config(6);
    s3_gpio_config(7);
   


  
    faddress = 1;
    QLSPI_Write_S3_Mem(PMU_MISC_POR_0_ADDR,(unsigned char *)&faddress,4);
    vTaskDelay(6000 / portTICK_PERIOD_MS);

   
}
