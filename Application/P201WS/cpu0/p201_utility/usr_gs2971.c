#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hal.h"
#include "debuglog.h"
#include "hal_hdmi_rx.h"
#include "sys_event.h"
#include "test_hal_spi.h"
#include "hal_gpio.h"
#include "usr_gs2971.h"

static STRU_HDMI_RX_STATUS s_st_hdmiRxStatus;

void SDI_Spi_Init(void)
{
    STRU_HAL_SPI_INIT st_spiInitInfo;

    HAL_GPIO_SetMode(HAL_GPIO_NUM60, HAL_GPIO_PIN_MODE0);
    HAL_GPIO_SetMode(HAL_GPIO_NUM64, HAL_GPIO_PIN_MODE0);
    HAL_GPIO_SetMode(HAL_GPIO_NUM68, HAL_GPIO_PIN_MODE0);
    HAL_GPIO_SetMode(HAL_GPIO_NUM72, HAL_GPIO_PIN_MODE0);

    st_spiInitInfo.u16_halSpiBaudr = 1;
    st_spiInitInfo.e_halSpiPolarity = HAL_SPI_POLARITY_HIGH;
    st_spiInitInfo.e_halSpiPhase = HAL_SPI_PHASE_2EDGE;

    int ret = HAL_SPI_MasterInit(HAL_SPI_COMPONENT_2, &st_spiInitInfo);

    DLOG_Info("SDI_Spi_Init return is: %d\n", ret);
}

int GS2971_SPi_read_register(unsigned char u8_reg)
{
    unsigned char u8_txData[128];
    unsigned char u8_rxData[128];
    int reg_value = 0;

    memset(u8_txData, 0x00, sizeof(u8_txData));
    memset(u8_rxData, 0x00, sizeof(u8_rxData));

    u8_txData[0] = 0x80;
    u8_txData[1] = u8_reg;

    int ret = HAL_SPI_MasterWriteRead(HAL_SPI_COMPONENT_2, u8_txData, 2, u8_rxData, 2, 20);
    if (ret == HAL_OK) {
        reg_value = (((int)u8_rxData[0] & 0x0FF) << 8) + u8_rxData[1];
        DLOG_Info("reg: %02x, return data is: %02x \n", u8_reg, reg_value);

        return reg_value;
    } else {
        DLOG_Info("Error to read register !");
    }

    return -1;
}

void SDI_RxVideoNotifyH264Endcoder(void *pu8_rxBuf)
{
    SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_H264_INPUT_FORMAT_CHANGE, pu8_rxBuf);
}

HAL_HDMI_RxHandle SDIRxVideoHandle = SDI_RxVideoNotifyH264Endcoder;

static void HDSDI_RX_CheckFormatStatus()
{

}

static void HDSDI_RX_IdleCallback(void *paramPtr)
{
    int width = 0, height = 0, fps = 0;
    int lines_per_frame_value = 0;
    int words_per_line_value;
    int words_per_active_line;
    int active_lines_per_frame;

    if (s_st_hdmiRxStatus.st_videoFormat.u16_width == 0)
    {
        STRU_SysEvent_H264InputFormatChangeParameter p;
        p.index = 0;
        p.vic = 34;
        p.width = 1280;
        p.hight = 720;
        p.framerate = 60;

        lines_per_frame_value = GS2971_SPi_read_register((unsigned char)0x21) & 0x7FF;
        words_per_line_value = GS2971_SPi_read_register((unsigned char)0x20) & 0x3FFF;

        words_per_active_line = GS2971_SPi_read_register((unsigned char)0x1F) & 0x3FFF;
        active_lines_per_frame = GS2971_SPi_read_register((unsigned char)0x22) & 0x7FF;

        DLOG_Info("lines_per_frame_value: %d, words_per_line_value: %d, words_per_active_line: %d, active_lines_per_frame: %d\n",
                    lines_per_frame_value , words_per_line_value, words_per_active_line, active_lines_per_frame);

	if (749 <= lines_per_frame_value && lines_per_frame_value <= 750) {
	    p.width = 1280;
            p.hight = 720;

            if (words_per_line_value >= 3960) {
                p.framerate = 25;
            } else if (words_per_line_value >= 3300) {
                p.framerate = 30;
            } else if (words_per_line_value > 1650) {
                p.framerate = 50;
            } else {
                p.framerate = 60;
            }
	}

        if (1124 <= lines_per_frame_value && lines_per_frame_value <= 1125) {
            p.width = 1920;
            p.hight = 1080;

            if (words_per_line_value >= 2200 + 550) {
                p.framerate = 24; 
            } else if (words_per_line_value >= 2200 + 440) {
                p.framerate = 25;   
            } else if (words_per_line_value >= 2200) {
                p.framerate = 30;
            } else {
                p.framerate = 60;
            }
	}
 
        DLOG_Info("width is %d, height is %d, framerate: %d \n",  p.width, p.hight, p.framerate);

        if ((p.width == 1920) && (p.hight == 1080) && (p.framerate >= 50)) {
            p.width = 1280;
            p.hight = 720;
//          p.framerate = 60;
        }
	
        p.e_h264InputSrc = ENCODER_INPUT_SRC_HDMI_0;
        HDMI_RxVideoNotifyH264Endcoder((void *)&p);

        s_st_hdmiRxStatus.st_videoFormat.u16_width = p.width;
        s_st_hdmiRxStatus.st_videoFormat.u16_hight = p.hight;
        s_st_hdmiRxStatus.st_videoFormat.u8_framerate = p.framerate;
        s_st_hdmiRxStatus.st_videoFormat.u8_vic = p.vic;
    }
}

void HAL_SDI_RX_Init()
{
    HAL_GPIO_OutPut(HAL_GPIO_NUM63);
    HAL_GPIO_SetPin(HAL_GPIO_NUM63, HAL_GPIO_PIN_SET);
    HAL_Delay(15);
    HAL_GPIO_SetPin(HAL_GPIO_NUM63, HAL_GPIO_PIN_RESET);
    HAL_Delay(15);
    HAL_GPIO_SetPin(HAL_GPIO_NUM63, HAL_GPIO_PIN_SET);

    HAL_Delay(30);

    SDI_Spi_Init();

    s_st_hdmiRxStatus.u8_devEnable = 1;

    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_VIDEO_EVENT, SDIRxVideoHandle);

    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_IDLE, HDSDI_RX_IdleCallback);
}

