#include <stdint.h>
#include "sram_ground.h"
#include "debuglog.h"
#include "usbd_def.h"
#include "usbd_hid.h"
#include "usbh_mtp.h"
#include "reg_rw.h"
#include "systicks.h"
#include "cpu_info.h"
#include "usbh_def.h"
#include "usbh_cdc.h"
#include "sys_event.h"
#include "hal_ret_type.h"
#include "hal_sram_ground.h"

volatile uint32_t               sramReady0;
volatile uint32_t               sramReady1;

volatile uint32_t               sram0ReceivedDataSize = 0;
volatile uint32_t               sram1ReceivedDataSize = 0;

extern USBD_HandleTypeDef       USBD_Device[USBD_PORT_NUM];
extern USBH_HandleTypeDef		hUSBHost[USBH_PORT_NUM];
volatile uint8_t                g_u8DataPathReverse = 0;
volatile uint32_t               g_TickRecord[2];
STRU_CHANNEL_PORT_CONFIG        g_stChannelPortConfig[SRAM_CHANNEL_NUM];
#ifdef ARCAST
uint8_t                         g_mp3DecodeBuff[SRAM_MP3_DECODE_BUFF_SIZE];
uint32_t                        g_mp3DecodeBuffRdPos = 0;
uint32_t                        g_mp3DecodeBuffWrPos = 0;
#endif

static uint8_t ground_LockStatus = 0; //baseband ground Lockstatus [0]: vt Lock  [1]: vt id match

extern volatile uint8_t g_mtp_enable;


void SRAM_PackForMtp(uint8_t *header, uint32_t dataLen, uint8_t channel)
{
    uint32_t                i;
    uint16_t                checksum = 0;
    uint8_t                 temp = 0;

    for (i = 0; i < SRAM_DATA_RESERVED_LEN; i++)
    {
        header[i] = 0;
    }

    header[0] = 0xFF;
    header[1] = 0xA5;
    header[2] = 0x5A;
    header[3] = 0xFF;

    header[11] = channel & 0x01;// channel 0 or channel 1
    header[12] = (uint8_t)dataLen;
    header[13] = (uint8_t)(dataLen >> 8);

    for (i = 0; i < (SRAM_DATA_RESERVED_LEN - 2); i++)
    {
        checksum += header[i];
    }

    header[14] = (uint8_t)checksum;
    header[15] = (uint8_t)(checksum >> 8);

    for (i = 0; i < SRAM_DATA_RESERVED_LEN; i+= 4)
    {
        temp        = header[i];
        header[i]   = header[i+3];
        header[i+3] = temp;

        temp        = header[i+1];
        header[i+1] = header[i+2];
        header[i+2] = temp;
    }
}

void SRAM_Ready0_USB_HOST_Video_Config(uint8_t *buff, uint32_t dataLen)
{
    buff = (uint8_t *)SRAM_BUFF_0_ADDRESS;
	if ((ground_LockStatus != 0x03) || (g_mtp_enable != 1))
	{
		SRAM_Ready0Confirm();
		return;
	}

	set_mtp_video_enable(1);

	SRAM_PackForMtp(buff, dataLen, 0);
	USBH_MTP_SendVideoData(buff, dataLen + SRAM_DATA_RESERVED_LEN, 0);

	g_TickRecord[0] = SysTicks_GetTickCount();

	sramReady0 = 1;
}

void SRAM_Ready0_USB_DEVICE_Video_Config(uint8_t *buff, uint32_t dataLen, uint8_t usb_port_id)
{
	uint8_t u8_usbPortId;
    USBD_HandleTypeDef *pdev;
    uint8_t u8_endPoint;

	buff = (uint8_t *)(SRAM_BUFF_0_ADDRESS + SRAM_DATA_RESERVED_LEN);
	if (g_stChannelPortConfig[0].u8_rtspEnable)
	{
		SRAM_InsertRTSPBuffer(dataLen, buff);
		SRAM_Ready0Confirm();
		return;
	}

#ifdef ARCAST
	u8_usbPortId = g_stChannelPortConfig[0].u8_usbPort;
	pdev = &USBD_Device[u8_usbPortId];
#else
	for (u8_usbPortId = 0; u8_usbPortId < USBD_PORT_NUM; ++u8_usbPortId)
	{
        if (usb_port_id >= HAL_USB_PORT_0 && usb_port_id < HAL_USB_PORT_NUM)
            u8_usbPortId = usb_port_id;

		pdev = &USBD_Device[u8_usbPortId];
		if (pdev->dev_state == USBD_STATE_CONFIGURED)
		{
			//DLOG_Error("dev_state: %d, u8_usbPortId: %d", pdev->dev_state, u8_usbPortId);
			g_stChannelPortConfig[0].u8_usbPort = u8_usbPortId;
			break;
		}
	}

	if (u8_usbPortId >= USBD_PORT_NUM)
	{
		SRAM_Ready0Confirm();
		return;
	}
#endif

	if (ground_LockStatus != 0x03)
	{
		SRAM_Ready0Confirm();
		return;
	}

	set_mtp_video_enable(0);

	u8_endPoint = g_stChannelPortConfig[0].u8_usbEp;

	if (USBD_OK != USBD_HID_SendReport(pdev, buff, dataLen, u8_endPoint))
	{
		DLOG_Info("HID0 Send Error!\n");
		SRAM_Ready0Confirm();
	}
	else
	{
		if (pdev->u8_connState != 2)
		{
			SRAM_Ready0Confirm();
		}
		else
		{
			g_TickRecord[0] = SysTicks_GetTickCount();
			sramReady0 = 1;
		}
	}
}

void SRAM_Ready0IRQHandler(uint32_t u32_vectorNum)
{
    uint8_t					*buff;
    uint32_t                dataLen;
    uint8_t                 u8_usbPortId;
    USBD_HandleTypeDef		*pdev;
	USBH_HandleTypeDef		*phost;
    uint8_t                 u8_endPoint;
    uint8_t                 u8_usb_port_priority = HAL_USB_PORT_NUM;

    buff                    = (uint8_t *)SRAM_BUFF_0_ADDRESS;

    dataLen                 = SRAM_DATA_VALID_LEN_0;
    dataLen                 = (dataLen << 2);

	ENUM_HAL_USB_DR_MODE usb0_mode = HAL_USB_Get_USB_DR_Mode(HAL_USB_PORT_0);
	ENUM_HAL_USB_DR_MODE usb1_mode = HAL_USB_Get_USB_DR_Mode(HAL_USB_PORT_1);
	uint8_t usb0_connected = 0;
	uint8_t usb1_connected = 0;

    sram0ReceivedDataSize += dataLen;

	if (usb0_mode == HAL_USB_DR_MODE_HOST)
	{
		phost = &hUSBHost[HAL_USB_PORT_0];
		usb0_connected = phost->device.is_connected;		//HPRT
	}
    else
	{
		pdev = &USBD_Device[HAL_USB_PORT_0];
		usb0_connected = (pdev->dev_state == USBD_STATE_CONFIGURED);
	}

	
    if (usb1_mode == HAL_USB_DR_MODE_HOST)
    {
        phost = &hUSBHost[HAL_USB_PORT_1];
        usb1_connected = phost->device.is_connected;        //HPRT
    }
    else
    {
        pdev = &USBD_Device[HAL_USB_PORT_1];
        usb1_connected = (pdev->dev_state == USBD_STATE_CONFIGURED);
    }

	if (usb0_connected && usb1_connected)
		u8_usb_port_priority = HAL_USB_Get_Video_Priority();
    else if (usb0_connected)
        u8_usb_port_priority = HAL_USB_PORT_0;
    else if (usb1_connected)
        u8_usb_port_priority = HAL_USB_PORT_1;
    else
        u8_usb_port_priority = HAL_USB_PORT_NUM;

	switch (u8_usb_port_priority)
	{
		case HAL_USB_PORT_0:
			if (usb0_mode == HAL_USB_DR_MODE_HOST)
				SRAM_Ready0_USB_HOST_Video_Config(buff, dataLen);
			else
				SRAM_Ready0_USB_DEVICE_Video_Config(buff, dataLen, HAL_USB_PORT_0);
			break;
		case HAL_USB_PORT_1:
			if (usb1_mode == HAL_USB_DR_MODE_HOST)
				SRAM_Ready0_USB_HOST_Video_Config(buff, dataLen);
			else
				SRAM_Ready0_USB_DEVICE_Video_Config(buff, dataLen, HAL_USB_PORT_1);
			break;
		case HAL_USB_PORT_NUM:
		default:
			if (g_mtp_enable == 1)
				SRAM_Ready0_USB_HOST_Video_Config(buff, dataLen);
			else
				SRAM_Ready0_USB_DEVICE_Video_Config(buff, dataLen, HAL_USB_PORT_NUM);
			break;
	}
}


void SRAM_Ready1_USB_HOST_Video_Config(uint8_t *buff, uint32_t dataLen)
{
    buff = (uint8_t *)SRAM_BUFF_1_ADDRESS;

    if ((ground_LockStatus != 0x03) || (g_mtp_enable != 1))
    {
        SRAM_Ready1Confirm();
        return;
    }

	set_mtp_video_enable(1);

    SRAM_PackForMtp(buff, dataLen, 1);

	USBH_MTP_SendVideoData(buff, dataLen + SRAM_DATA_RESERVED_LEN, 1);

    g_TickRecord[1] = SysTicks_GetTickCount();

    sramReady1 = 1;
}

void SRAM_Ready1_USB_DEVICE_Video_Config(uint8_t *buff, uint32_t dataLen, uint8_t usb_port_id)
{
    uint8_t u8_usbPortId;
    USBD_HandleTypeDef *pdev;
    uint8_t u8_endPoint;

    buff = (uint8_t *)(SRAM_BUFF_1_ADDRESS + SRAM_DATA_RESERVED_LEN);
    if (g_stChannelPortConfig[1].u8_rtspEnable)
    {
        SRAM_InsertRTSPBuffer(dataLen, buff);
        SRAM_Ready1Confirm();
        return;
    }

#ifdef ARCAST
    u8_usbPortId		= g_stChannelPortConfig[1].u8_usbPort;
    pdev				= &USBD_Device[u8_usbPortId];
#else
/*
	if (usb_port_id >= HAL_USB_PORT_0 && usb_port_id < HAL_USB_PORT_NUM)
	{
		u8_usbPortId = usb_port_id;
		pdev = &USBD_Device[u8_usbPortId];
		{
            //DLOG_Error("dev_state: %d, u8_usbPortId: %d", pdev->dev_state, u8_usbPortId);
            g_stChannelPortConfig[1].u8_usbPort = u8_usbPortId;
            break;
        }
	}
**/

	for (u8_usbPortId = 0; u8_usbPortId < USBD_PORT_NUM; ++u8_usbPortId)
    {
		if (usb_port_id >= HAL_USB_PORT_0 && usb_port_id < HAL_USB_PORT_NUM)
			u8_usbPortId = usb_port_id;

        pdev = &USBD_Device[u8_usbPortId];
        if (pdev->dev_state == USBD_STATE_CONFIGURED)
        {
            //DLOG_Error("dev_state: %d, u8_usbPortId: %d", pdev->dev_state, u8_usbPortId);
            g_stChannelPortConfig[1].u8_usbPort = u8_usbPortId;
            break;
        }
    }

    if (u8_usbPortId >= USBD_PORT_NUM)
    {
        SRAM_Ready1Confirm();
        return;
    }
#endif

	if (ground_LockStatus != 0x03)
    {
        SRAM_Ready1Confirm();
        return;
    }

	set_mtp_video_enable(0);

    u8_endPoint = g_stChannelPortConfig[1].u8_usbEp;

    if (USBD_OK != USBD_HID_SendReport(pdev, buff, dataLen, u8_endPoint))
    {
        DLOG_Info("HID1 Send Error!\n");
        SRAM_Ready1Confirm();
    }
    else
    {
        if (pdev->u8_connState != 2)
        {
            SRAM_Ready1Confirm();
        }
        else
        {
            g_TickRecord[1] = SysTicks_GetTickCount();
            sramReady1 = 1;
        }
    }
}

void SRAM_Ready1IRQHandler(uint32_t u32_vectorNum)
{
    uint8_t					*buff;
    uint32_t                dataLen;
    uint8_t                 u8_usbPortId;
    USBD_HandleTypeDef		*pdev;
	USBH_HandleTypeDef		*phost;
    uint8_t                 u8_endPoint;
	uint8_t                 u8_usb_port_priority = HAL_USB_PORT_NUM;

    buff                    = (uint8_t *)SRAM_BUFF_1_ADDRESS;

    dataLen                 = SRAM_DATA_VALID_LEN_1;
    dataLen                 = (dataLen << 2);

    sram1ReceivedDataSize += dataLen;

	ENUM_HAL_USB_DR_MODE usb0_mode = HAL_USB_Get_USB_DR_Mode(HAL_USB_PORT_0);
    ENUM_HAL_USB_DR_MODE usb1_mode = HAL_USB_Get_USB_DR_Mode(HAL_USB_PORT_1);
    uint8_t usb0_connected = 0;
    uint8_t usb1_connected = 0;

    if (usb0_mode == HAL_USB_DR_MODE_HOST)
    {
        phost = &hUSBHost[HAL_USB_PORT_0];
        usb0_connected = phost->device.is_connected;        //HPRT
    }
    else
    {
        pdev = &USBD_Device[HAL_USB_PORT_0];
        usb0_connected = (pdev->dev_state == USBD_STATE_CONFIGURED);
    }


    if (usb1_mode == HAL_USB_DR_MODE_HOST)
    {
        phost = &hUSBHost[HAL_USB_PORT_1];
        usb1_connected = phost->device.is_connected;        //HPRT
    }
    else
    {
        pdev = &USBD_Device[HAL_USB_PORT_1];
        usb1_connected = (pdev->dev_state == USBD_STATE_CONFIGURED);
    }

    if (usb0_connected && usb1_connected)
        u8_usb_port_priority = HAL_USB_Get_Video_Priority();
    else if (usb0_connected)
        u8_usb_port_priority = HAL_USB_PORT_0;
    else if (usb1_connected)
        u8_usb_port_priority = HAL_USB_PORT_1;
    else
        u8_usb_port_priority = HAL_USB_PORT_NUM;

	switch (u8_usb_port_priority)
    {
        case HAL_USB_PORT_0:
            if (usb0_mode == HAL_USB_DR_MODE_HOST)
                SRAM_Ready1_USB_HOST_Video_Config(buff, dataLen);
            else
                SRAM_Ready1_USB_DEVICE_Video_Config(buff, dataLen, HAL_USB_PORT_0);
            break;
        case HAL_USB_PORT_1:
            if (usb1_mode == HAL_USB_DR_MODE_HOST)
                SRAM_Ready1_USB_HOST_Video_Config(buff, dataLen);
            else
                SRAM_Ready1_USB_DEVICE_Video_Config(buff, dataLen, HAL_USB_PORT_1);
            break;
        case HAL_USB_PORT_NUM:
        default:
            if (g_mtp_enable == 1)
                SRAM_Ready1_USB_HOST_Video_Config(buff, dataLen);
            else
                SRAM_Ready1_USB_DEVICE_Video_Config(buff, dataLen, HAL_USB_PORT_NUM);
            break;
    }
}


void SRAM_Ready0Confirm(void)
{
    sramReady0 = 0;

    /* confirm to Baseband that the SRAM data has been processed, ready to receive new data */
    Reg_Write32(DMA_READY_0, 1);
}


void SRAM_Ready1Confirm(void)
{
    sramReady1 = 0;

    /* confirm to Baseband that the SRAM data has been processed, ready to receive new data */
    Reg_Write32(DMA_READY_1, 1);
}


static void SRAM_GROUND_BBLockStatusHanlder(void *p)
{
    STRU_SysEvent_DEV_BB_STATUS *pstatus = (STRU_SysEvent_DEV_BB_STATUS *)p;
    if (BB_LOCK_STATUS == pstatus->pid)
    {
        ground_LockStatus = pstatus->lockstatus;
        DLOG_Info("Lock:%x", ground_LockStatus);
    }
}

void SRAM_GROUND_ReceiveVideoConfig(void)
{
    //uint8_t      temp;

    /* Base Band bypass data, directly write to SRAM */
    #if 0
    temp    = BB_SPI_ReadByte(PAGE1, 0x8d);
    temp   |= 0x40;
    BB_SPI_WriteByte(PAGE1, 0x8d, temp);

    BB_SPIWriteByte(PAGE2, 0x56, 0x06);
    #endif
    /* Threshold of usb_fifo in BaseBand */

    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_BB_EVENT, SRAM_GROUND_BBLockStatusHanlder);

    /* Set the start address of sram for bb bypass channel 0*/
    Reg_Write32(SRAM_WR_ADDR_OFFSET_0, SRAM_BB_BYPASS_OFFSET_0);

    /* Set the max num of SRAM_READY interrupt trigger signal */
    Reg_Write32(SRAM_WR_MAX_LEN_0, SRAM_DMA_READY_LEN);

    /* Set the start address of sram for bb bypass channel 1*/
    Reg_Write32(SRAM_WR_ADDR_OFFSET_1, SRAM_BB_BYPASS_OFFSET_1);

    /* Set the max num of SRAM_READY interrupt trigger signal */
    Reg_Write32(SRAM_WR_MAX_LEN_1, SRAM_DMA_READY_LEN);

}


void SRAM_CheckTimeout(void)
{
    if (sramReady0)
    {
        if ((SysTicks_GetDiff(g_TickRecord[0], SysTicks_GetTickCount())) >= SRAM_TIMEOUT_THRESHOLD)
        {
            DLOG_Error("channel 0 timeout:%u", SysTicks_GetTickCount());

            SRAM_Ready0Confirm();
        }
    }

    if (sramReady1)
    {
        if ((SysTicks_GetDiff(g_TickRecord[1], SysTicks_GetTickCount())) >= SRAM_TIMEOUT_THRESHOLD)
        {
            DLOG_Error("channel 1 timeout:%u", SysTicks_GetTickCount());

            SRAM_Ready1Confirm();
        }
    }
}

void SRAM_GetReceivedDataSize(uint32_t* p_sram0Size, uint32_t* p_sram1Size)
{
    if (p_sram0Size)
    {
        *p_sram0Size = sram0ReceivedDataSize;
    }

    if (p_sram1Size)
    {
        *p_sram1Size = sram1ReceivedDataSize;
    }
}

#ifdef ARCAST

uint32_t SRAM_GetMp3BufferLength(void)
{
    if (g_mp3DecodeBuffWrPos >= g_mp3DecodeBuffRdPos)
    {
        return g_mp3DecodeBuffWrPos - g_mp3DecodeBuffRdPos;
    }
    else
    {
        return ((SRAM_MP3_DECODE_BUFF_SIZE + g_mp3DecodeBuffWrPos) - g_mp3DecodeBuffRdPos);
    }
}


void SRAM_InsertMp3Buffer(uint32_t dataLen, uint8_t *data)
{
    uint32_t         src;
    uint32_t         dest;
    uint32_t         dataLenTemp;

    src     = (uint32_t)data;
    dest    = (uint32_t)(g_mp3DecodeBuff) + g_mp3DecodeBuffWrPos;

    if ((SRAM_MP3_DECODE_BUFF_SIZE - SRAM_GetMp3BufferLength()) < dataLen)
    {
        return;
    }

    if ((g_mp3DecodeBuffWrPos + dataLen) <= SRAM_MP3_DECODE_BUFF_SIZE)
    {
        memcpy((void *)dest, (void *)src, dataLen);

        g_mp3DecodeBuffWrPos    += dataLen;

        if (g_mp3DecodeBuffWrPos >= SRAM_MP3_DECODE_BUFF_SIZE)
        {
            g_mp3DecodeBuffWrPos -= SRAM_MP3_DECODE_BUFF_SIZE;
        }

    }
    else
    {
        dataLenTemp     = (SRAM_MP3_DECODE_BUFF_SIZE - g_mp3DecodeBuffWrPos);

        memcpy((void *)dest, (void *)src, dataLenTemp);

        src             = (uint32_t)data + dataLenTemp;
        dest            = (uint32_t)g_mp3DecodeBuff;
        dataLenTemp     = dataLen - dataLenTemp;

        memcpy((void *)dest, (void *)src, dataLenTemp);

        g_mp3DecodeBuffWrPos    = dataLenTemp;


        if (g_mp3DecodeBuffWrPos >= SRAM_MP3_DECODE_BUFF_SIZE)
        {
            g_mp3DecodeBuffWrPos -= SRAM_MP3_DECODE_BUFF_SIZE;
        }
    }

    return;
}


uint32_t SRAM_GetMp3Data(uint32_t dataLen, uint8_t *dataBuff)
{
    uint32_t        i;
    uint32_t        read_size = 0;

    //DLOG_Info("read  pos: %d", g_mp3DecodeBuffRdPos);
    /* ensure 4 bytes align */
    dataLen   -= (dataLen & 0x3);

    for (i = 0; i < dataLen; i+=4)
    {

        dataBuff[i]       = g_mp3DecodeBuff[g_mp3DecodeBuffRdPos+3];
        dataBuff[i+1]     = g_mp3DecodeBuff[g_mp3DecodeBuffRdPos+2];
        dataBuff[i+2]     = g_mp3DecodeBuff[g_mp3DecodeBuffRdPos+1];
        dataBuff[i+3]     = g_mp3DecodeBuff[g_mp3DecodeBuffRdPos];

        g_mp3DecodeBuffRdPos+=4;
		read_size+=4;
        if (g_mp3DecodeBuffRdPos >= SRAM_MP3_DECODE_BUFF_SIZE)
        {
            g_mp3DecodeBuffRdPos = 0;
        }
    }
    return read_size;
}


#endif

__weak HAL_RET_T RTSPBufferCallBak(uint32_t dataLen, uint8_t *data)
{
    return 0;
}

uint8_t GetRTSPBufferIndex(uint8_t index)
{
    return (index & (CDC_SEND_BUFFER_MAX_SIZE - 1));
}

static unsigned int     header[2];
static unsigned char    header_index = 0;
static unsigned char    recving_data = 0;
static unsigned char    baseband_shift = 0;
static unsigned int     save_data = 0;
volatile uint8_t        g_RTSP_Restart;

void SetRTSPRestart()
{
    g_RTSP_Restart = 1;
}

void SetRTSPInit()
{
    int i = 0;
    g_netcardSendBufferWrIndex = 0;
    header_index = 0;
    recving_data = 0;
    baseband_shift = 0;
    save_data = 0;
    header[0] = 0;
    header[1] = 0;

    for (i = 0; i < CDC_SEND_BUFFER_MAX_SIZE; i++)
    {
        g_netcardSendBuffer[i].data_valid = 0;
    }
}

void SRAM_InsertRTSPBuffer(uint32_t dataLen, uint8_t *data)
{
    unsigned int            i = 0;
    unsigned int            j = 0;
    unsigned int            copy_len = 0;

    unsigned int            special_word = 0;
    unsigned int            next_word = 0;
    unsigned short          packet_len = 0;
    unsigned short          zero_padding = 0;

    unsigned int           *data_buf = (unsigned int *)data;
    unsigned int            data_len = (dataLen >> 2);


    if (g_netcardSendBufferWrIndex - g_netcardSendBufferRdIndex > CDC_SEND_BUFFER_MAX_SIZE - 1)
    {
        DLOG_Error("buffer is full: wr: %d, rd: %d", g_netcardSendBufferWrIndex, g_netcardSendBufferRdIndex);
        if(g_netcardSendBufferWrIndex - g_netcardSendBufferRdIndex > CDC_SEND_BUFFER_MAX_SIZE)
        {
            SetRTSPInit();
            g_netcardSendBufferRdIndex = 0;
        }
        return;
    }
    DLOG_Info("wr: %d, rd: %d", g_netcardSendBufferWrIndex, g_netcardSendBufferRdIndex);

    while (i < data_len)
    {
        if(1 == g_RTSP_Restart)
        {
            SetRTSPInit();
            g_RTSP_Restart = 0;
            return;
        }

        if (recving_data == 0)
        {
            if (0 == header_index)
            {
                special_word = save_data;
                next_word    = data_buf[i];

                while (i < (data_len - 1))
                {
                    if (special_word == 0xFF5AA5FF)
                    {
                        baseband_shift = 0;
                        break;
                    }
                    else if ((special_word & 0xFFFFFF) == 0xFF5AA5)
                    {
                        special_word  = (special_word << 8);
                        special_word |= (next_word >> 24);
    
                        if (special_word == 0xFF5AA5FF)
                        {
                            baseband_shift = 8;
                            break;
                        }
                    }
                    else if ((special_word & 0xFFFF) == 0xFF5A)
                    {
                        special_word  = (special_word << 16);
                        special_word |= (next_word >> 16);
    
                        if (special_word == 0xFF5AA5FF)
                        {
                            baseband_shift = 16;
                            break;
                        }
                    }
                    else if ((special_word & 0xFF) == 0xFF)
                    {
                        special_word  = (special_word << 24);
                        special_word |= (next_word >> 8);
    
                        if (special_word == 0xFF5AA5FF)
                        {
                            baseband_shift = 24;
                            break;
                        }
                    }

                    special_word = data_buf[i++];
                    next_word    = data_buf[i];
                }

                save_data = next_word;
                ++i;

                if (i >= (data_len - 1))
                {
                    if (baseband_shift)
                    {
                        special_word  = (special_word << baseband_shift);
                        special_word |= (next_word >> (32 - baseband_shift));
                    }
                }

                if (special_word == 0xFF5AA5FF)
                {
                    header[0] = special_word;

                    header_index = 1;
                }
            }
            else if (1 == header_index)
            {
                if (baseband_shift)
                {
                    header[1] = ((save_data << baseband_shift) | (data_buf[i] >> (32 - baseband_shift)));
                }
                else
                {
                    header[1] = save_data;
                }

                zero_padding    = (unsigned short)header[1];
                packet_len      = (unsigned short)(header[1] >> 16);

                header_index = 0;

                if (baseband_shift)
                {
                    save_data = data_buf[i++];
                }

                if (packet_len < 1600)
                {
                    uint8_t tmpindex = GetRTSPBufferIndex(g_netcardSendBufferWrIndex);
                    recving_data = 1;

                    g_netcardSendBuffer[tmpindex].data_size       = packet_len;
                    g_netcardSendBuffer[tmpindex].zero_padding    = zero_padding;
                    g_netcardSendBuffer[tmpindex].wr_pos          = 0;
                    g_netcardSendBuffer[tmpindex].data_valid      = 0;
                }
                else
                {
                    printf("rtsp header not found: 0x%08x, 0x%08x \r\n", special_word, packet_len);
                }
            }
        }
        else
        {
            uint8_t tmpindex = GetRTSPBufferIndex(g_netcardSendBufferWrIndex);
            if (baseband_shift)
            {
                g_netcardSendBuffer[tmpindex].data[g_netcardSendBuffer[tmpindex].wr_pos]
                                    = ((save_data << baseband_shift)|(data_buf[i] >> (32 - baseband_shift)));
            }
            else
            {
                g_netcardSendBuffer[tmpindex].data[g_netcardSendBuffer[tmpindex].wr_pos]
                                    = data_buf[i];

                i++;
            }

            g_netcardSendBuffer[tmpindex].wr_pos++;

            while ((g_netcardSendBuffer[tmpindex].wr_pos < (g_netcardSendBuffer[tmpindex].data_size >> 2))&&
                   (i < (data_len - 1)))
            {
                if (baseband_shift)
                {
                    g_netcardSendBuffer[tmpindex].data[g_netcardSendBuffer[tmpindex].wr_pos]
                                    = ((data_buf[i] << baseband_shift)|(data_buf[i+1] >> (32 - baseband_shift)));
                }
                else
                {
                    g_netcardSendBuffer[tmpindex].data[g_netcardSendBuffer[tmpindex].wr_pos]
                                    = data_buf[i];
                }

                g_netcardSendBuffer[tmpindex].wr_pos++;

                i++;
            }

            if (baseband_shift)
            {
                save_data = data_buf[i++];
            }

            if (g_netcardSendBuffer[tmpindex].wr_pos < (g_netcardSendBuffer[tmpindex].data_size >> 2))
            {
                if (baseband_shift == 0)
                {
                    g_netcardSendBuffer[tmpindex].data[g_netcardSendBuffer[tmpindex].wr_pos++]
                                                    = data_buf[i++];
                }
            }

            if (g_netcardSendBuffer[tmpindex].wr_pos >= (g_netcardSendBuffer[tmpindex].data_size >> 2))
            {
#if 1
                if(HAL_OK == RTSPBufferCallBak(g_netcardSendBuffer[tmpindex].data_size, (uint8_t *)(g_netcardSendBuffer[tmpindex].data)))
                {
                    g_netcardSendBuffer[tmpindex].data_size = g_netcardSendBuffer[tmpindex].data_size - g_netcardSendBuffer[tmpindex].zero_padding;
                    g_netcardSendBuffer[tmpindex].total_size = (g_netcardSendBuffer[tmpindex].data_size + 8);
                    g_netcardSendBuffer[tmpindex].data_size = g_netcardSendBuffer[tmpindex].data_size | 0xC0000000;

                    unsigned char   old_index = tmpindex;

                    if (g_setIPCameraMACaddr == 0)
                    {
                        uint8_t     tmp = 0xff;
                        uint8_t     *tmp_buf = (uint8_t *)&g_netcardSendBuffer[old_index].data[0];

                        for(j = 0; j < 6; j++)
                        {
                            tmp &= tmp_buf[j];
                        }

                        if(tmp == 0xff)
                        {
                            memcpy(g_IPCameraMACaddr, tmp_buf + 6, 6);
                            DLOG_Warning("get Mac %x %x %x %x %x %x ", g_IPCameraMACaddr[0], g_IPCameraMACaddr[1], g_IPCameraMACaddr[2], g_IPCameraMACaddr[3], g_IPCameraMACaddr[4], g_IPCameraMACaddr[5]);

                            g_setIPCameraMACaddr = 1;
                        }
                    }

                    g_netcardSendBuffer[old_index].data_valid = 1;

                    g_netcardSendBufferWrIndex++;
                }

                recving_data = 0;
#else
                g_netcardSendBuffer[tmpindex].data_size = g_netcardSendBuffer[tmpindex].data_size - g_netcardSendBuffer[tmpindex].zero_padding;
                g_netcardSendBuffer[tmpindex].total_size = (g_netcardSendBuffer[tmpindex].data_size + 8);
                g_netcardSendBuffer[tmpindex].data_size = g_netcardSendBuffer[tmpindex].data_size | 0xC0000000;

                recving_data = 0;

                unsigned char   old_index = tmpindex;

                if (g_setIPCameraMACaddr == 0)
                {
                    uint8_t     tmp = 0xff;
                    uint8_t     *tmp_buf = (uint8_t *)&g_netcardSendBuffer[old_index].data[0];

                    for(j = 0; j < 6; j++)
                    {
                        tmp &= tmp_buf[j];
                    }

                    if(tmp == 0xff)
                    {
                        memcpy(g_IPCameraMACaddr, tmp_buf + 6, 6);
                        DLOG_Warning("get Mac %x %x %x %x %x %x ", g_IPCameraMACaddr[0], g_IPCameraMACaddr[1], g_IPCameraMACaddr[2], g_IPCameraMACaddr[3], g_IPCameraMACaddr[4], g_IPCameraMACaddr[5]);

                        g_setIPCameraMACaddr = 1;
                    }
                }

                if(HAL_OK == RTSPBufferCallBak(g_netcardSendBuffer[old_index].total_size - 8, (uint8_t *)(g_netcardSendBuffer[old_index].data)))
                {
                    g_netcardSendBuffer[old_index].data_valid = 1;

                    g_netcardSendBufferWrIndex++;
                }

#endif
            }
        }
    }

    return;
}


