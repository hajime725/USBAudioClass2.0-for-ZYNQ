/******************************************************************************
* Copyright (C) 2020 Xilinx, Inc.  All rights reserved.
* SPDX-License-Identifier: MIT
 ******************************************************************************/

/*****************************************************************************/
/**
 *
 * @file xusbps_audio_example.c
 *
 * This file contains the implementation of chapter 9 specific code for
 * the example.
 *
 * <pre>
 * MODIFICATION HISTORY:
 *
 * Ver   Who	Date     Changes
 * ----- ---- -------- -------------------------------------------------------
 * 1.0   pm	20/02/20 First release
 *
 * </pre>
 *
 *****************************************************************************/

/***************************** Include Files ********************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "xparameters.h"
#include "xgpio.h"

#include "xusbps_ch9_audio.h"
#include "xusbps_class_audio.h"

#include "xaudioformatter.h"
#include "xscutimer.h"
#include "xi2stx.h"
#include "xil_printf.h"

#include "xscugic.h"
#include "xil_exception.h"
#include "xpseudo_asm.h"
#include "xreg_cortexa9.h"
#include "xil_cache.h"

#include "xclk_wiz.h"
#include "xil_types.h"
#include "xstatus.h"
#include "sleep.h"


#ifdef XUSBPS_MICROPHONE
#include "xusbps_audiodata.h"
#endif

/************************** Constant Definitions ****************************/
#define DIV_ROUND_UP(n, d)	(((n) + (d) - 1) / (d))

/*
 * The following constants are to be modified to get different size of memory.
 */
#define RAMDISKSECTORS  	0x10//0x400		//1KB
#define RAMBLOCKS		4096

//#define MEMORY_SIZE (64 * 1024)
#define MEMORY_SIZE (64 * 1024)

#ifdef __ICCARM__
#pragma data_alignment = 32
u8 Buffer[MEMORY_SIZE];
#pragma data_alignment = 4
#else
u8 Buffer[MEMORY_SIZE] ALIGNMENT_CACHELINE;
#endif

#ifdef XUSBPS_UAC1

/*
 * Default is 8000Hz
 * Change this value to set different sampling rate.
 * 		u8 AudioFreq [MAX_AudioFreq][3] ={
 * 			{ 0x40, 0x1F, 0x00 }, // sample frequency 8000
 * 			{ 0x44, 0xAC, 0x00 }, // sample frequency 44100
 * 			{ 0x80, 0xBB, 0x00 }, // sample frequency 48000
 * 			{ 0x00, 0x77, 0x01,}, // sample frequency 96000
 *		};
 */
#define CUR_AUDIOFREQ		0x01

#else	/*	XUSPBS_UAC2 */

/*
 * Default is 44100Hz
 * Change this value to set different sampling rate.
 * 		u8 AudioFreq [MAX_AudioFreq][3] ={
 * 			{ 0x40, 0x1F, 0x00 }, // sample frequency 8000
 * 			{ 0x44, 0xAC, 0x00 }, // sample frequency 44100
 * 			{ 0x80, 0xBB, 0x00 }, // sample frequency 48000
 * 			{ 0x00, 0x77, 0x01,}, // sample frequency 96000
 *		};
 */
#define CUR_AUDIOFREQ		0x01//0x01

#endif

#define USB_DEVICE_ID		XPAR_XUSBPS_0_DEVICE_ID
#define INTC_DEVICE_ID		XPAR_SCUGIC_SINGLE_DEVICE_ID
#define	USB_INTR_ID		XPAR_XUSBPS_0_INTR


/***************XAUDIO_FORMATTER関連 ******************************/
/************************** Constant Definitions ******************************/

#define XAUDIO_FORMATTER_SW_VER "v1.00"

#define I2S_TX_DEVICE_ID	XPAR_XI2STX_0_DEVICE_ID
#define I2S_TX_INTERRUPT_ID	XPAR_FABRIC_I2STX_0_VEC_ID
#define I2S_TX_FS		48 /* kHz */
#define I2S_TX_MCLK		128*48//48*64//128*48*3//(128*3*48/*I2S_TX_FS*/)//(384*I2S_TX_FS)
#define I2S_TX_TIME_OUT 500000
#define AF_DEVICE_ID	XPAR_XAUDIOFORMATTER_0_DEVICE_ID
#define AF_MM2S_INTERRUPT_ID XPAR_FABRIC_AUDIO_FORMATTER_0_IRQ_MM2S_INTR
#define AF_FS		I2S_TX_FS//44//48 /* kHz */
#define AF_MCLK		I2S_TX_MCLK//(384 * AF_FS)	//384
#define AF_S2MM_TIMEOUT 0x80000000
/************************** Variable Definitions ******************************/
XAudioFormatter AFInstance;
XI2s_Tx I2sTxInstance;		/* Instance of the I2s Transmitter device */
//XScuGic Intc;
//#define Intc InterruptController
u32 I2sTxIntrReceived=0;
u32 S2MMAFIntrReceived=0;
u32 MM2SAFIntrReceived=0;

#define AUDIO_STREAM_ADDR 0x1A000000		//開始アドレス
#define AUDIO_STREAM_CHANNEL 2				//チャネル数
#define INDEXMAX 512
//256
u32 fifobuf[INDEXMAX] __attribute__ ((aligned(32)));
//u32 fifobuf[128] __attribute__ ((aligned(32)));
int bufready = 0;		//バッファが十分たまっているか
int bufstartflg = 0;		//バッファ開始

int feedbackadjust = 0;

// {開始アドレス		,ch数,分解能bit,全チャネル分のbyte数(4byte*ch数),fifoサンプル数}
XAudioFormatterHwParams af_hw_params;// = {fifobuf, AUDIO_STREAM_CHANNEL, BIT_DEPTH_24, 4 * AUDIO_STREAM_CHANNEL, sizeof(fifobuf)/sizeof(fifobuf[0])/AUDIO_STREAM_CHANNEL};//デバイスから見るとオフセットがかかっている。
/********************************** Timer ************************************/
#define TIMER_DEVICE_ID		XPAR_XSCUTIMER_0_DEVICE_ID
#define TIMER_LOAD_VALUE	0x000FFFFF

/********************************** CLK wizard ************************************/
/*
* The following constants map to the names of the hardware instances.
* They are only defined here such that a user can easily change all the
* needed device IDs in one place.
*/
#define XCLK_WIZARD_DEVICE_ID		XPAR_CLK_WIZ_1_DEVICE_ID//XPAR_TX_SUBSYSTEM_VID_CLK_RST_HIER_CLK_WIZARD_1_DEVICE_ID
/*
* The following constants are part of clock dynamic reconfiguration
* They are only defined here such that a user can easily change
* needed parameters
*/

#define CLK_LOCK			1

/*
 * Output frequency in MHz. User need to change this value to
 * generate grater/lesser interrupt as per input frequency
 */

/**************************** Type Definitions *******************************/

/***************** Macros (Inline Functions) Definitions *********************/

/************************** Function Prototypes ******************************/

int ScuTimerPolledExample(u16 DeviceId);

/************************** Variable Definitions *****************************/


XScuTimer Timer;		/* Cortex A9 SCU Private Timer Instance */

/************************** GPIO関連 ******************************/

#define LED 0x3f   /* Assumes bit 0 of GPIO is connected to an LED  */
#define LED_OUT 0x07
#define LED_IN  0x38
/*
 * The following constants map to the XPAR parameters created in the
 * xparameters.h file. They are defined here such that a user can easily
 * change all the needed parameters in one place.
 */
#define GPIO_EXAMPLE_DEVICE_ID  XPAR_GPIO_0_DEVICE_ID

/*
 * The following constant is used to wait after an LED is turned on to make
 * sure that it is visible to the human eye.  This constant might need to be
 * tuned for faster or slower processor speeds.
 */
#define LED_DELAY     10000000

/*
 * The following constant is used to determine which channel of the GPIO is
 * used for the LED if there are 2 channels supported.
 */
#define LED_CHANNEL 0x01

/**************************** Type Definitions *******************************/


/***************** Macros (Inline Functions) Definitions *********************/

#ifdef PRE_2_00A_APPLICATION

/*
 * The following macros are provided to allow an application to compile that
 * uses an older version of the driver (pre 2.00a) which did not have a channel
 * parameter. Note that the channel parameter is fixed as channel 1.
 */
#define XGpio_SetDataDirection(InstancePtr, DirectionMask) \
        XGpio_SetDataDirection(InstancePtr, LED_CHANNEL, DirectionMask)

#define XGpio_DiscreteRead(InstancePtr) \
        XGpio_DiscreteRead(InstancePtr, LED_CHANNEL)

#define XGpio_DiscreteWrite(InstancePtr, Mask) \
        XGpio_DiscreteWrite(InstancePtr, LED_CHANNEL, Mask)

#define XGpio_DiscreteSet(InstancePtr, Mask) \
        XGpio_DiscreteSet(InstancePtr, LED_CHANNEL, Mask)

#endif

/************************** Function Prototypes ******************************/


/************************** Variable Definitions *****************************/

/*
 * The following are declared globally so they are zeroed and so they are
 * easily accessible from a debugger
 */

XGpio Gpio; /* The Instance of the GPIO Driver */

/************************** Function Prototypes ******************************/
static void XUsbPs_IsoInHandler(void *CallBackRef, u32 RequestedBytes,
		u32 BytesTxed );
static void XUsbPs_IsoOutHandler(void *CallBackRef, u32 RequestedBytes,
		u32 BytesTxed );
//static void XUsbPs_IsoOutFeedbackHandler(void *CallBackRef, u32 RequestedBytes,
//		u32 BytesTxed);
static void XUsbPs_IsoInFeedbackHandler(void *CallBackRef, u32 RequestedBytes,
		u32 BytesTxed);

static void XUsbPs_AudioTransferSize(void);
static void XUsbPs_Ep0EventHandler(void *CallBackRef, u8 EpNum, u8 EventType, void *Data);
s32 XUsbPs_SetupInterruptSystem(XUsbPs *InstancePtr, u16 IntcDeviceID,
		XScuGic *IntcInstancePtr);
s32 XUsbPs_CfgInit(struct Usb_DevData *InstancePtr, Usb_Config *ConfigPtr,
		u32 BaseAddress);

u32 InitializeAudioFormatter(XAudioFormatter *AFInstancePtr);
u32 InitializeI2sTx(XI2s_Tx *I2sTxInstancePtr);
int setupAudioI2S(void);
void WriteDMAfifo();
void Usb2I2sReader(u8 *retbuf ,u32 RequestedBytes,u32 *RetBytes);

/************************** CLK wizard *****************************/
u32 ClkWiz_Example(XClk_Wiz *IntcInstancePtr, u32 DeviceId, int Frequency);
u32 XClk_WaitForLock(XClk_Wiz_Config *CfgPtr_Dynamic);

/************************** Variable Definitions *****************************/
struct Usb_DevData UsbInstance;

Usb_Config *UsbConfigPtr;
XUsbPs PrivateData;

/*
 * Interrupt controller instance
 */
XScuGic InterruptController;
XUsbPs_DeviceConfig DeviceConfig;

//CLK wizard
XClk_Wiz ClkWiz_Dynamic; /* The instance of the ClkWiz_Dynamic */

/*
 * A ram array
 */
u32 RamDisk[RAMDISKSECTORS * RAMBLOCKS] __attribute__ ((aligned(4)));
u8 *WrRamDiskPtr = (u8 *) &(RamDisk[0]);

u8 BufferPtrTemp[1024];
u8 BufferDummy[1024*16]={0};		//バッファリング中のダミーデータ

u32 Index = 0;
u32 Index_eof = 0;				//EOF
u32 Index_read = 0;
u8	Index_refresh = 0;
u8 FirstPktFrame = 1;
u8 FirstPktFrame_In = 1;
u8 MM2SIntFlg = 0;					//I2SのFIFO割り込みフラグ

u32 Framesize = 0, Interval = 0, PacketSize = 0,
    PacketResidue = 0, Residue = 0, Residue_In = 0;

#ifdef XUSBPS_MICROPHONE
static u32 FileSize = sizeof(Hello_wav);
#else
static u32 FileSize = sizeof(RamDisk);				//buffer size
#endif

/* Supported AUDIO sampling frequencies */
u8 AudioFreq [MAX_AUDIO_FREQ][3] ={
	{ 0x40, 0x1F, 0x00 },	/* sample frequency 8000  */
	{ 0x44, 0xAC, 0x00 },	/* sample frequency 44100 */
	{ 0x80, 0xBB, 0x00 },	/* sample frequency 48000 */
	{ 0x00, 0x77, 0x01,},	/* sample frequency 96000 */
};

extern u16 wRequestLength;
extern u16 wRequestObject;
extern u8 bMuteState;
extern u32 dFrequency;

/****************************************************************************/
/**
 * This function is the main function of the USB audio example.
 *
 * @param	None
 *
 * @return
 *		- XST_SUCCESS if successful,
 *		- XST_FAILURE if unsuccessful.
 *
 * @note	None.
 *
 *
 *****************************************************************************/
int main(void)
{
	const u8 NumEndpoints = 2;
	u8 *MemPtr = NULL;
	s32 Status;
	int gpio_Status;
	static int pdFrequency = 0;
	// {開始アドレス		,ch数,分解能bit,全チャネル分のbyte数(4byte*ch数),fifoサンプル数}
	//XAudioFormatterHwParams af_hw_params = {fifobuf, AUDIO_STREAM_CHANNEL, BIT_DEPTH_24, 4 * AUDIO_STREAM_CHANNEL, sizeof(fifobuf)/sizeof(fifobuf[0])/AUDIO_STREAM_CHANNEL};//デバイスから見るとオフセットがかかっている。
	af_hw_params.buf_addr = fifobuf;
	af_hw_params.active_ch = AUDIO_STREAM_CHANNEL;
	af_hw_params.bits_per_sample = BIT_DEPTH_24;
	af_hw_params.periods = 4 * AUDIO_STREAM_CHANNEL;
	af_hw_params.bytes_per_period = sizeof(fifobuf)/sizeof(fifobuf[0])/AUDIO_STREAM_CHANNEL;
//	Xil_ICacheInvalidate();
//	Xil_ICacheEnable();
//
//	/* Initialize DCache */
//	Xil_DCacheInvalidate();
//	Xil_DCacheEnable();



	gpio_Status = XGpio_Initialize(&Gpio, GPIO_EXAMPLE_DEVICE_ID);
		if (gpio_Status != XST_SUCCESS) {
			xil_printf("Gpio Initialization Failed\r\n");
			return XST_FAILURE;
		}

		/* Set the direction for all signals as inputs except the LED output */
		XGpio_SetDataDirection(&Gpio, LED_CHANNEL, ~LED);

	xil_printf("Xilinx Audio Start...2020\r\n");


	UsbConfigPtr = XUsbPs_LookupConfig(USB_DEVICE_ID);			//起動時のデバイスIDは0
	if (NULL == UsbConfigPtr) {
		return XST_FAILURE;
	}

	Status = XUsbPs_CfgInit(&UsbInstance, UsbConfigPtr,
			UsbConfigPtr->BaseAddress);
	if (XST_SUCCESS != Status) {
		return XST_FAILURE;
	}

	/*
	 * Assign the ep configuration to USB driver
	 * USBドライバにエンドポイントコンフィグレーションを割り当て
	 */

		//コントロール転送
//	DeviceConfig.EpCfg[0].Out.Type = XUSBPS_EP_TYPE_CONTROL;
//	DeviceConfig.EpCfg[0].Out.NumBufs = 2;
//	DeviceConfig.EpCfg[0].Out.BufSize = 64;
//	DeviceConfig.EpCfg[0].Out.MaxPacketSize = 64;
//	DeviceConfig.EpCfg[0].In.Type = XUSBPS_EP_TYPE_CONTROL;
//	DeviceConfig.EpCfg[0].In.NumBufs = 2;
//	DeviceConfig.EpCfg[0].In.MaxPacketSize = 64;

	DeviceConfig.EpCfg[0].Out.Type = XUSBPS_EP_TYPE_CONTROL;
	DeviceConfig.EpCfg[0].Out.NumBufs = 2;
	DeviceConfig.EpCfg[0].Out.BufSize = 64;
	DeviceConfig.EpCfg[0].Out.MaxPacketSize =  64;
	DeviceConfig.EpCfg[0].In.Type = XUSBPS_EP_TYPE_CONTROL;
	DeviceConfig.EpCfg[0].In.NumBufs = 2;
//	DeviceConfig.EpCfg[0].In.BufSize = 64;
	DeviceConfig.EpCfg[0].In.MaxPacketSize =  64;
		//OUT PC->ZYNQ
	DeviceConfig.EpCfg[1].Out.Type = XUSBPS_EP_TYPE_ISOCHRONOUS;
	DeviceConfig.EpCfg[1].Out.NumBufs = 16;
	DeviceConfig.EpCfg[1].Out.BufSize = 1024;
	DeviceConfig.EpCfg[1].Out.MaxPacketSize = 1024;
	DeviceConfig.EpCfg[1].In.Type = XUSBPS_EP_TYPE_ISOCHRONOUS;		//
	DeviceConfig.EpCfg[1].In.NumBufs = 2;//16;
//	DeviceConfig.EpCfg[1].In.BufSize = 1024;
	DeviceConfig.EpCfg[1].In.MaxPacketSize = 4;//1024;					//0-1024
		//IN ZYNQ->PC
//	DeviceConfig.EpCfg[2].Out.Type = XUSBPS_EP_TYPE_ISOCHRONOUS;	//FEEDBACK_ENDPOINT
//	DeviceConfig.EpCfg[2].Out.NumBufs = 4;
//	DeviceConfig.EpCfg[2].Out.BufSize = 64;
//	DeviceConfig.EpCfg[2].Out.MaxPacketSize = 16;
//	DeviceConfig.EpCfg[2].In.Type = XUSBPS_EP_TYPE_ISOCHRONOUS;
//	DeviceConfig.EpCfg[2].In.NumBufs = 16;
//	DeviceConfig.EpCfg[2].In.MaxPacketSize = 1024;

	DeviceConfig.NumEndpoints = NumEndpoints;		//2（エンドポイント終端+1）

	MemPtr = (u8 *) &Buffer[0];										//Bufferを0で埋めて初期化
	memset(MemPtr, 0, MEMORY_SIZE);
	Xil_DCacheFlushRange((unsigned int) MemPtr, MEMORY_SIZE);

	/* Finish the configuration of the DeviceConfig structure and configure
	 * the DEVICE side of the controller.
	 * DeviceConfig構造体の構成を完了し、USBコントローラーをDEVICEとして構成する。
	 */
	DeviceConfig.DMAMemPhys = (u32) MemPtr;

	Status = XUsbPs_ConfigureDevice(UsbInstance.PrivateData, &DeviceConfig);

	if (XST_SUCCESS != Status) {
		return XST_FAILURE;
	}

	/*
	 * Hook up chapter9 handler
	 * Chapter9ハンドラーを接続します
	 */
	Status = XUsbPs_EpSetHandler(UsbInstance.PrivateData, 0,
			XUSBPS_EP_DIRECTION_OUT,
			XUsbPs_Ep0EventHandler,
			UsbInstance.PrivateData);

	/*
	 * set endpoint handlers
	 * XUsbPsu_IsoInHandler -  to be called when data is sent
	 * XUsbPsu_IsoOutHandler -  to be called when data is received
	 * エンドポイントハンドラをセットする。
	 * XUsbPsu_IsoInHandler -  データ送信時に呼ばれる
	 * XUsbPsu_IsoOutHandler -  データ受信時に呼ばれる
	 */
//	XUsbPs_EpSetIsoHandler(UsbInstance.PrivateData, ISO_EP,		//feedback end point
//			XUSBPS_EP_DIRECTION_IN,
//			XUsbPs_IsoInFeedbackHandler);

	XUsbPs_EpSetIsoHandler(UsbInstance.PrivateData, ISO_EP,		//データ受信時に呼ばれる
			XUSBPS_EP_DIRECTION_OUT,
			XUsbPs_IsoOutHandler);
#ifdef USB_IN_ENABLE
	XUsbPs_EpSetIsoHandler(UsbInstance.PrivateData, ISO_EP2,	//データ送信時に呼ばれる
				XUSBPS_EP_DIRECTION_IN,
				XUsbPs_IsoInHandler);
#else

	XUsbPs_EpSetIsoHandler(UsbInstance.PrivateData, ISO_EP2,		//feedback end point
				XUSBPS_EP_DIRECTION_IN,
				XUsbPs_IsoInFeedbackHandler);
#endif

	/*
	 * Setup interrupts
	 * 割り込み初期化
	 */
	Status = XUsbPs_SetupInterruptSystem((XUsbPs *)UsbInstance.PrivateData,
			INTC_DEVICE_ID,
			&InterruptController);
	if (Status != XST_SUCCESS) {
		xil_printf("FAILURE::XUsbPs_SetupInterruptSyste\r\n");
		return XST_FAILURE;
	}


	XUsbPs_AudioTransferSize();

	/*
	 * Start the controller so that Host can see our device
	 * ホストがデバイスを認識できるようにコントローラーを起動します
	 */
	XUsbPs_Start((XUsbPs *)UsbInstance.PrivateData);
	//XUsbPs_IsoInFeedbackHandler((XUsbPs *)UsbInstance.PrivateData, 4,4);	//nack対策
	setupAudioI2S();




	while(1) {
		if(pdFrequency != dFrequency){
				////////////////////////////////CLKtest///////////////////////////////////////////
				//xil_printf("------------------------------------------\n\r");
				//xil_printf("CLK_WIZARD example\n\r");
				//xil_printf("-------------------------------------------\n\r\n\r");

				Status = ClkWiz_Example(&ClkWiz_Dynamic, XCLK_WIZARD_DEVICE_ID, dFrequency);
				if (Status != XST_SUCCESS) {
					xil_printf("CLK_WIZARD Monitor interrupt example Failed");
					return XST_FAILURE;
				}

				xil_printf("Success CLK/\n\r");
				////////////////////////////////////////////////////////////////////////////////////
				pdFrequency = dFrequency;
		}
		/*
		 * Rest is taken care by interrupts
		 * 残りは割り込みによって処理されます
		 */
		//ScuTimerPolledExample(TIMER_DEVICE_ID);
		//ScuTimerPolledExample(TIMER_DEVICE_ID);
		//ScuTimerPolledExample(TIMER_DEVICE_ID);
		//xil_printf("MM2S Status Register :%x\r\n",XAudioFormatter_ReadReg(XPAR_AUDIO_FORMATTER_0_BASEADDR,0x114));
		//if(MM2SIntFlg == 1){			//割り込みがあった時だけ送る
			MM2SIntFlg = 0;
			WriteDMAfifo();
		//}
	}

	return XST_SUCCESS;
}

/****************************************************************************/
/**
 * This function calculates Data to be sent at every Interval
 * この関数は、間隔ごとに送信されるデータを計算します
 *
 * @param	None
 *
 * @return	None
 *
 * @note	None.
 *
 *
 *****************************************************************************/
static void XUsbPs_AudioTransferSize(void)

{
	u32 Rate = 0, AudioFreqTemp = 0, MaxPacketSize = 0;

	/*
	 * Audio sampling frequency which filled in TYPE One Format
	 * descriptors
	 * TYPEOneのディスクリプタに入力されたオーディオサンプリング周波数
	 */

			//44100(周波数)
	AudioFreqTemp = dFrequency;//(u32)((u8)AudioFreq[CUR_AUDIOFREQ][0] |
//			(u8)AudioFreq[CUR_AUDIOFREQ][1] << 8 |
//			(u8)AudioFreq[CUR_AUDIOFREQ][2] << 16);

	/*
	 * Audio transmission Bytes required to send in one sec
	 * (Sampling Freq * Number of Channel * Audio frame size)
	 * オーディオデータの１秒当たりのbyte数
	 */

	//AUDIO_CHANNEL_NUM
	int selectedaudiochannels = 2;									// TODO:alt設定に追従させる。
	Framesize = selectedaudiochannels * AUDIO_FRAME_SIZE;				// 2ch*2byte=4
	Rate = AudioFreqTemp * Framesize;								// 44100*4=176,400
	Interval = INTERVAL_PER_SECOND / (1 << (AUDIO_INTERVAL - 1));	// 8000/(1<<3)=8000/8=1000

	/*
	 * Audio data transfer size to be transfered at every interval
	 * 間隔ごとに転送される音声データ転送サイズ
	 */
					// 2*2*繰り上げ(AudioFreqTemp/Interval)=2*2*繰り上げ(44100/1000)=2*2*45=180
	MaxPacketSize = selectedaudiochannels * AUDIO_FRAME_SIZE *
		DIV_ROUND_UP(AudioFreqTemp, INTERVAL_PER_SECOND /
				(1 << (AUDIO_INTERVAL - 1)));
					// 176400/1000=176
	PacketSize = ((Rate / Interval) < MaxPacketSize) ?
		(Rate / Interval) : MaxPacketSize;

	if (PacketSize < MaxPacketSize)
		PacketResidue = Rate % Interval;//パケット余り（byte）
	else
		PacketResidue = 0;
}

/****************************************************************************/
/**
 * This function is ISO IN Endpoint handler/Callback function, called by driver
 * when data is sent to host.
 * この関数はISOINエンドポイントハンドラー/コールバック関数であり、
 * データがホストに送信されるときにドライバーによって呼び出されます。
 *
 * @param	CallBackRef is pointer to Usb_DevData instance.
 * @param	RequestedBytes is number of bytes requested to send.
 * @param	BytesTxed is actual number of bytes sent to Host.
 *
 * @return	None
 *
 * @note	None.
 *
 *****************************************************************************/


static void XUsbPs_IsoInHandler(void *CallBackRef, u32 RequestedBytes,
		u32 BytesTxed)
{
	struct Usb_DevData *InstancePtr = CallBackRef;
	u32 Size;
//	static int bufready = 0;		//バッファが十分たまっているか
	//static int brink = 0;
	Size = PacketSize;
	Residue_In += PacketResidue;

	if ((Residue_In / Interval) >= Framesize) {
		Size += Framesize;
		Residue_In -= Framesize * Interval;
	}

//	if ((Index_read + Size) > Index_eof/*FileSize*/) {
//		/* Buffer is completed, retransmitting the same file data */
//		if((Index_refresh == 0) && (bufready == 1)){		//バッファが空になったとき
//			Index = 0;
//			//memset(&WrRamDiskPtr[0],(u8)0,Index_eof);
//			//Index = 0;
//			Index_eof = 0;
//			//xil_printf("RSTFIFO  ");
//			bufready = 0;
//		}
//		Index_read = 0;
//		Index_refresh = 0;
//
//	}
//
//	if(bufready == 0){
//		Index_read = 0;
//		if(Index_eof > (Size << 4))bufready = 1;		//バッファに読み出し量の8倍以上たまったら再生開始
//		xil_printf("FIFOEMPTY\r\n");
//	}

#ifdef XUSBPS_MICROPHONE
	if (XUsbPs_EpBufferSend((XUsbPs *)InstancePtr->PrivateData, ISO_EP,
				&Hello_wav[Index],
				Size) == XST_SUCCESS) {
#else	/* XUSPBS_UAC2 */

	if (XUsbPs_EpBufferSend((XUsbPs *)InstancePtr->PrivateData, ISO_EP2,
			/*(1 == bufready)? */&WrRamDiskPtr[0]/* : BufferDummy*/,		//バッファ量が十分でなければ0出力とする
				Size) == XST_SUCCESS) {
#endif
//		if(brink = ~brink){
//			/* LED点灯 */
//			XGpio_DiscreteWrite(&Gpio, LED_CHANNEL, LED_IN);
//		}else{
//		/* LED消灯 */
//			XGpio_DiscreteClear(&Gpio, LED_CHANNEL, LED_IN);
//		}
		//if(1 == bufready) Index_read += Size;								//バッファ準備中はカウントしない
		if (FirstPktFrame_In) {
			Size = PacketSize;
			Residue_In += PacketResidue;

			if ((Residue_In / Interval) >= Framesize) {
				Size += Framesize;
				Residue_In -= Framesize * Interval;
			}

//			if ((Index_read + Size) > Index_eof)
//				Index_read = 0;
//			else
//				Index_read += Size;

			FirstPktFrame_In = 0;
		}
	}

}




/****************************************************************************/
/**
 * This function is ISO OUT Endpoint handler/Callback function, called by driver
 * when data is received from host.
 *
 * @param	CallBackRef is pointer to Usb_DevData instance.
 * @param	RequestedBytes is number of bytes requested to send.
 * @param	BytesTxed is actual number of bytes sent to Host.
 * BytesTxedは、ホストに送信された実際のバイト数です。
 *
 * @return	None
  *　この関数は1.000ｋHzで呼ばれる
  *　PC -> ZYNQ
 * @note	None.
 *
 *****************************************************************************/
static void XUsbPs_IsoOutHandler(void *CallBackRef, u32 RequestedBytes,
		u32 BytesTxed)
{
	struct Usb_DevData *InstancePtr = CallBackRef;
	u32 Size;
	static int brink = 0;
	int xst_state= -1;
//	static int cnt = 0;
	//Index = 0;
	if (FirstPktFrame)XUsbPs_AudioTransferSize();
	Size = PacketSize;
	Residue += PacketResidue;		//パケットのあまり分を積算

	if ((Residue / Interval) >= Framesize) {		//積算パケット余りが1フレーム分以上の時
		Size += Framesize;							//受信要求サイズにその分足す
		Residue -= Framesize * Interval;			//足した分は引く
	}

	if (FirstPktFrame) {
		FirstPktFrame = 0;
		bufstartflg = 1;
		xil_printf("FirstPktFrame\r\n");
	} else {
		if ((Index + BytesTxed) > FileSize) {
			/* Buffer is full, overwriting the data */
			Index_eof = Index;			//data終端を記憶
			Index = 0;
			Index_refresh = 1;
		}
		else if ((Index + BytesTxed) > Index_eof) {

					Index_eof = Index + BytesTxed;			//data終端を記憶
				}
		else if ((Index + BytesTxed) > Index_read) {
					/* Buffer is full, overwriting the data */
			//if(Index < Index_read) xil_printf("FIFOFULL\r\n");
					Index_eof = Index + BytesTxed;			//data終端を記憶
				}
		/* LED点灯 */
		//XGpio_DiscreteWrite(&Gpio, LED_CHANNEL, 0x01);//LD0
		if(brink = ~brink){
					/* LED点灯 */
					//XGpio_DiscreteWrite(&Gpio, LED_CHANNEL, 0x01);
				}else{
				/* LED消灯 */
					//XGpio_DiscreteClear(&Gpio, LED_CHANNEL, 0x01);
				}
		/* Copy received to RAM array */
		memcpy(&WrRamDiskPtr[Index], BufferPtrTemp, BytesTxed);
		Index += BytesTxed;
		/* LED消灯 */
		//XGpio_DiscreteClear(&Gpio, LED_CHANNEL, 0x01);

	}
	/* LED点灯 */
	//XGpio_DiscreteWrite(&Gpio, LED_CHANNEL, 0x02);
	//次のバッファを準備
	Size = 1024;
	xst_state=XUsbPs_EpDataBufferReceive((XUsbPs *)InstancePtr->PrivateData, ISO_EP,
			BufferPtrTemp, Size);
//	if(cnt++>1000){
//		cnt= 0;
//		xil_printf("Txed:%d\r\n",BytesTxed);
//	}
//	Timeout = XUSBPS_TIMEOUT_COUNTER;
//	do {
//		Status = XUsbPs_EpBufferReceive(InstancePtr,
//			ISO_EP, &BufferPtr, &BufferLen, &Handle);
//	} while((Status != XST_SUCCESS) && --Timeout);
	if(BytesTxed == 0){
		//XGpio_DiscreteWrite(&Gpio, LED_CHANNEL, 0x02);
//		xil_printf("IsoOutHandler :0x");
//
//		while(--Size)xil_printf("%01x",BufferPtrTemp[Size]);
//		xil_printf("\r\n");

	}
	//else XGpio_DiscreteClear(&Gpio, LED_CHANNEL, 0x02);
	//XGpio_DiscreteClear(&Gpio, LED_CHANNEL, 0x02);
}

/****************************************************************************/
/**
 * This function is ISO IN Feedback Endpoint handler/Callback function, called by driver
 * when data is sent to host.(OUTのfeedback)
 *
 * @param	CallBackRef is pointer to Usb_DevData instance.
 * @param	RequestedBytes is number of bytes requested to send.
 * @param	BytesTxed is actual number of bytes sent to Host.
 *
 * @return	None
 *
 * @note	None.
 *
 *****************************************************************************/
static void XUsbPs_IsoInFeedbackHandler(void *CallBackRef, u32 RequestedBytes,
		u32 BytesTxed)
{
	struct Usb_DevData *InstancePtr = CallBackRef;
	static int ledst = 1;
	feedbackadjust = -200;
	u32 feedback = (dFrequency + feedbackadjust) * 125 * 8 * 0x10000;//0x00058338;//44.1kHz usbHS

	if(ledst){
	XGpio_DiscreteWrite(&Gpio, LED_CHANNEL, 0x02);
	ledst = 0;
	}else{
	XGpio_DiscreteClear(&Gpio, LED_CHANNEL, 0x02);
	ledst = 1;
	//xil_printf("F\n\r");
	}
	if (XUsbPs_EpBufferSend((XUsbPs *)InstancePtr->PrivateData, ISO_EP, (u8*)&feedback,	4) == XST_SUCCESS) {	}
}


/****************************************************************************/
/**
 * This function is ISO OUT Feedback Endpoint handler/Callback function, called by driver
 * when data is sent to host.(INのfeedback)
 *
 * @param	CallBackRef is pointer to Usb_DevData instance.
 * @param	RequestedBytes is number of bytes requested to send.
 * @param	BytesTxed is actual number of bytes sent to Host.
 *
 * @return	None
 *
 * @note	None.
 *
 *****************************************************************************/
//static void XUsbPs_IsoOutFeedbackHandler(void *CallBackRef, u32 RequestedBytes,
//		u32 BytesTxed)
//{
//	struct Usb_DevData *InstancePtr = CallBackRef;
//	u32 feedback = 0x00058338;//44.1kHz usbHS
//
//	if (XUsbPs_EpBufferSend((XUsbPs *)InstancePtr->PrivateData, ISO_EP2, (u8*)&feedback, 4) == XST_SUCCESS) {	}
//}

/*****************************************************************************/
/**
 * This function is registered to handle callbacks for endpoint 0 (Control).
 * この関数は、エンドポイント0（コントロール）のコールバックを処理するために登録されています。
 *
 * It is called from an interrupt context such that the amount of processing
 * performed should be minimized.
 * 実行される処理の量を最小限に抑えるように、割り込みコンテキストから呼び出されます。
 *
 *
 * @param	CallBackRef is the reference passed in when the function
 *		was registered.
 * @param	EpNum is the Number of the endpoint on which the event occurred.
 * @param	EventType is type of the event that occurred.
 *
 * @return	None.
 *
 ******************************************************************************/
static void XUsbPs_Ep0EventHandler(void *CallBackRef, u8 EpNum, u8 EventType, void *Data) {
	XUsbPs *InstancePtr;
	int Status;
	XUsbPs_SetupData SetupData;
	u8 *BufferPtr;
	u32 BufferLen;
	u32 Handle;
//	static u8 Reply[XUSBPS_REQ_REPLY_LEN] ALIGNMENT_CACHELINE;

	Xil_AssertVoid(NULL != CallBackRef);

	InstancePtr = (XUsbPs *) CallBackRef;




	switch (EventType) {

	/* Handle the Setup Packets received on Endpoint 0. */
	/* エンドポイント0で受信した初期化パケットを処理します。 */
	case XUSBPS_EP_EVENT_SETUP_DATA_RECEIVED:
		Status = XUsbPs_EpGetSetupData(InstancePtr, EpNum, &SetupData);
		if (XST_SUCCESS == Status) {
			/* Handle the setup packet. */
			(int) XUsbPs_Ch9HandleSetupPacket((XUsbPs *)InstancePtr,
					&SetupData);
		}
		break;

		/* We get data RX events for 0 length packets on endpoint 0.
		 * We receive and immediately release them again here, but
		 * there's no action to be taken.
		 */
	case XUSBPS_EP_EVENT_DATA_RX:
		xil_printf("XUSBPS_EP_EVENT_DATA_RX\r\n");
		/* Get the data buffer. */
//		if(wRequestLength > 0){			//受信リクエストがあったとき
//			XUsbPs_EpDataBufferReceive(
//			(XUsbPs *)InstancePtr,
//			0,
//			Reply, wRequestLength);
//			if(wRequestObject == 1){
//				bMuteState=Reply[0];
//			}
//			wRequestObject = 0;
//			xil_printf("dat0=%d\r\n",Reply[0]);
//			xil_printf("dat1=%d\r\n",Reply[1]);
//			xil_printf("Len=%d\r\n",wRequestLength);
//			xil_printf("Handle=%d\r\n",Handle);
//			wRequestLength=0;
//		}
		Status = XUsbPs_EpBufferReceive(InstancePtr, EpNum, &BufferPtr,
				&BufferLen, &Handle);

//		xil_printf("dat0=%d\r\n",*BufferPtr);
//		xil_printf("dat1=%d\r\n",*(BufferPtr+1));
//		xil_printf("Len=%d\r\n",BufferLen);
//		xil_printf("Handle=%d\r\n",Handle);
		if (XST_SUCCESS == Status) {
			/* Return the buffer. */
			XUsbPs_EpBufferRelease(Handle);
		}
		break;

	default:
		/* Unhandled event. Ignore. */
		break;
	}
}

/****************************************************************************/
/**
 * This function setups the interrupt system such that interrupts can occur.
 * This function is application specific since the actual system may or may not
 * have an interrupt controller.  The USB controller could be
 * directly connected to aprocessor without an interrupt controller.
 * The user should modify this function to fit the application.
 *
 * @param	InstancePtr is a pointer to the XUsbPs instance.
 * @param	IntcDeviceID is the unique ID of the interrupt controller
 * @param	IntcInstacePtr is a pointer to the interrupt controller
 *			instance.
 *
 * @return	XST_SUCCESS if successful, otherwise XST_FAILURE.
 *
 * @note		None.
 *
 *****************************************************************************/
s32 XUsbPs_SetupInterruptSystem(XUsbPs *InstancePtr, u16 IntcDeviceID,
		XScuGic *IntcInstancePtr)
{
	s32 Status;
	XScuGic_Config *IntcConfig;

	/*
	 * Initialize the interrupt controller driver so that it is ready to
	 * use.
	 */
	IntcConfig = XScuGic_LookupConfig(IntcDeviceID);
	if (IntcConfig == NULL) {
		return XST_FAILURE;
	}
	Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
			IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	Xil_ExceptionInit();
	/*
	 * Connect the interrupt controller interrupt handler to the hardware
	 * interrupt handling logic in the processor.
	 */
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,
			(Xil_ExceptionHandler)XScuGic_InterruptHandler,
			IntcInstancePtr);
	/*
	 * Connect the device driver handler that will be called when an
	 * interrupt for the device occurs, the handler defined above performs
	 * the specific interrupt processing for the device.
	 */
	Status = XScuGic_Connect(IntcInstancePtr, USB_INTR_ID,
			(Xil_ExceptionHandler)XUsbPs_IntrHandler,
			(void *)InstancePtr);
	if (Status != XST_SUCCESS) {
		return Status;
	}
	/*
	 * Enable the interrupt for the device.
	 */
	XScuGic_Enable(IntcInstancePtr, USB_INTR_ID);

	/*
	 * Enable interrupts in the Processor.
	 */
	Xil_ExceptionEnableMask(XIL_EXCEPTION_IRQ);

	/* Enable the interrupts. */
	XUsbPs_IntrEnable(InstancePtr, XUSBPS_IXR_UR_MASK | XUSBPS_IXR_UI_MASK);


	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* This function initializes a XUsbPs instance/driver.
*
* The initialization entails:
* - Initialize all members of the XUsbPs structure.
*
* @param	InstancePtr is a pointer to XUsbPs instance of the controller.
* @param	ConfigPtr is a pointer to a XUsbPs_Config configuration
*		structure. This structure will contain the requested
*		configuration for the device. Typically, this is a local
*		structure and the content of which will be copied into the
*		configuration structure within XUsbPs.
* @param	BaseAddress is the base address of the device.
*
* @return
*		- XST_SUCCESS no errors occurred.
*		- XST_FAILURE an error occurred during initialization.
*
* @note
*
******************************************************************************/

s32 XUsbPs_CfgInit(struct Usb_DevData *InstancePtr, Usb_Config *ConfigPtr,
		u32 BaseAddress)
{
	PrivateData.AppData = InstancePtr;
	InstancePtr->PrivateData = (void *)&PrivateData;

	return XUsbPs_CfgInitialize((XUsbPs *)InstancePtr->PrivateData,
			ConfigPtr, BaseAddress);
}

int setupAudioI2S(void)
{
	u32 Status;
	//u32 RxStatus;
	u32 TxStatus;
	u32 IntrCount = 0;
//	static u32 transfer_max = 0;
//	static u32 transfer_count = 0;
	//static u32 readIndex=0;
//	unsigned int i=0;//0x0FFFFFFF;
	unsigned int cnttime=0;
//	u32 statusbuffer;
//		unsigned int *pfifo = fifobuf;

	//af_hw_params.buf_addr =  (u32) &(fifobuf[0]);

	XI2s_Tx *I2sTxInstancePtr = &I2sTxInstance;
	XAudioFormatter *AFInstancePtr = &AFInstance;

	//for(i=0;i<128;i++)fifobuf[i]=0;					//バッファ初期化
	memset(fifobuf, 0, sizeof(fifobuf));

	Xil_DCacheFlushRange((u32)fifobuf, sizeof(fifobuf));		//(startアドレス書き込み,バイト数)
	xil_printf("\nsyokika\n");
	xil_printf("\r\n-----------------------------------------------\r\n");
	xil_printf(" Xilinx Audio Formatter Example Design %s\r\n",
		XAUDIO_FORMATTER_SW_VER);
	xil_printf("	(c) 2018 by Xilinx Inc.\r\n");

	/* Initialize ICache */
//	Xil_ICacheInvalidate();
//	Xil_ICacheEnable();
//
//	/* Initialize DCache */
//	Xil_DCacheInvalidate();
//	Xil_DCacheEnable();

	/* Initialize IRQ */
//	Status = SetupInterruptSystem();
//	if (Status == XST_FAILURE) {
//		xil_printf("IRQ init failed.\n\r\r");
//		return XST_FAILURE;
//	}
	Xil_ExceptionEnable();

	Status = InitializeAudioFormatter(AFInstancePtr);
	if (Status == XST_FAILURE) {
		xil_printf("\r\nAudio Formatter Init failed\r\n");
		return XST_FAILURE;
	}else xil_printf("\r\nAudio Formatter Init Success\r\n");

	Status = InitializeI2sTx(I2sTxInstancePtr);
	if (Status == XST_FAILURE) {
		xil_printf("\r\nI2STx Init failed\r\n");
		return XST_FAILURE;
	}else xil_printf("\r\nI2STx Init Success\r\n");

	IntrCount = 0;
	TxStatus =  XST_FAILURE;

	while (IntrCount < 10/*I2S_TX_TIME_OUT*/) {


		if (I2sTxIntrReceived == 1 && MM2SAFIntrReceived == 1) {
			TxStatus =  XST_SUCCESS;
			break;
		}
		IntrCount++;
	}
	if (TxStatus == XST_SUCCESS)
		xil_printf("\r\nAudio Formatter Test successfull\r\n");
	else{
		xil_printf("\r\nAudio Formatter Test Failed2\r\n");
		xil_printf("I2sTxIntr:%d,MM2SAFIntr:%d\r\n",I2sTxIntrReceived,MM2SAFIntrReceived);
	}

	AFInstancePtr->ChannelId = XAudioFormatter_MM2S;
	//XAudioFormatterDMAStart(AFInstancePtr);
	//XAudioFormatterDMAStop(AFInstancePtr);
	//for(i=0;i<128;i++) fifobuf[i]=i;
	//Xil_DCacheFlushRange((u32)fifobuf, 128*4);//cache書き込み(startアドレス,vyte数)

			xil_printf("\nsyokika2\r\n");
			xil_printf("%x::%x\r\n",fifobuf,af_hw_params.buf_addr);
	AFInstancePtr->ChannelId = XAudioFormatter_MM2S;
	//XAudioFormatterDMAStop(AFInstancePtr);
	//XAudioFormatter_WriteReg(XPAR_AUDIO_FORMATTER_0_BASEADDR,XAUD_FORMATTER_BUFF_ADDR_LSB + 0x100,(u32) (fifobuf));
//	xil_printf("ADDR:%x %x\r\n",XAudioFormatter_ReadReg(XPAR_AUDIO_FORMATTER_0_BASEADDR,
//			XAUD_FORMATTER_BUFF_ADDR_MSB + 0x100),
//		XAudioFormatter_ReadReg(XPAR_AUDIO_FORMATTER_0_BASEADDR,
//			XAUD_FORMATTER_BUFF_ADDR_LSB + 0x100));
	xil_printf("MM2S Control Register:%x\r\n",XAudioFormatter_ReadReg(XPAR_AUDIO_FORMATTER_0_BASEADDR,0x110));
	xil_printf("MM2S Status Register :%x\r\n",XAudioFormatter_ReadReg(XPAR_AUDIO_FORMATTER_0_BASEADDR,0x114));
	xil_printf("MM2S Transfer Count Register :%x\r\n",XAudioFormatter_ReadReg(XPAR_AUDIO_FORMATTER_0_BASEADDR,0x128));

	//XAudioFormatterSetHwParams(AFInstancePtr, &af_hw_params);
	//XAudioFormatterDMAStart(AFInstancePtr);
//	transfer_max = 0;
	/*
	while(0){

		//Xil_DCacheInvalidateRange(0,4);//cache読み込み(startアドレス,vyte数)
		AFInstancePtr->ChannelId = XAudioFormatter_MM2S;
		transfer_count = XAudioFormatterGetDMATransferCount(AFInstancePtr);
//		if(transfer_count > transfer_max){
//			transfer_max = transfer_count;
//			xil_printf("countmax=%d\r\n",transfer_max);
//		}
		//ScuTimerPolledExample(TIMER_DEVICE_ID);
		//for(i=0;i<128;i++) fifobuf[i]= i;//(i-64)*(((cnttime)%2)?2:1);
		//Xil_DCacheFlushRange((u32)fifobuf, 128*4);//cache書き込み(startアドレス,vyte数)
		//xil_printf("cnt:%d\r\n",cnttime);
		//xil_printf("MM2S Status Register :%x\r\n",XAudioFormatter_ReadReg(XPAR_AUDIO_FORMATTER_0_BASEADDR,0x114));
		//xil_printf("MM2S Transfer Count Register :%x\r\n",XAudioFormatter_ReadReg(XPAR_AUDIO_FORMATTER_0_BASEADDR,0x128));

	//	while(XAudioFormatter_ReadReg(XPAR_AUDIO_FORMATTER_0_BASEADDR,0x114)){
//			Xil_DCacheInvalidateRange(XPAR_AUDIO_FORMATTER_0_BASEADDR|0x114,4);
//			statusbuffer = XAudioFormatter_ReadReg(XPAR_AUDIO_FORMATTER_0_BASEADDR,0x114);
//			XAudioFormatter_WriteReg(XPAR_AUDIO_FORMATTER_0_BASEADDR,0x114,statusbuffer);		//ステータスレジスタリセット
//			Xil_DCacheFlushRange(XPAR_AUDIO_FORMATTER_0_BASEADDR|0x114,4);
			//Xil_DCacheInvalidateRange(XPAR_AUDIO_FORMATTER_0_BASEADDR|0x114,4);
		//}
		//xil_printf("MM2S Status Register2 :%x\r\n",XAudioFormatter_ReadReg(XPAR_AUDIO_FORMATTER_0_BASEADDR,0x114));
		//xil_printf("ICCIAR :%x\r\n",*((u32 *)0xF8F0010C));
		cnttime++;
	}
	*/
	//while(1);
	return Status;
}

int ScuTimerPolledExample(u16 DeviceId)
{
	int Status;
	//volatile u32 CntValue1 = 0;
	volatile u32 CntValue2 = 0;
	XScuTimer_Config *ConfigPtr;
	XScuTimer *TimerInstancePtr = &Timer;

	/*
	 * Initialize the Scu Private Timer so that it is ready to use.
	 */
	ConfigPtr = XScuTimer_LookupConfig(DeviceId);

	/*
	 * This is where the virtual address would be used, this example
	 * uses physical address.
	 */
	Status = XScuTimer_CfgInitialize(TimerInstancePtr, ConfigPtr,
				 ConfigPtr->BaseAddr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	XScuTimer_SetPrescaler(TimerInstancePtr, 255);
	/*
	 * Load the timer counter register.
	 */
	XScuTimer_LoadTimer(TimerInstancePtr, TIMER_LOAD_VALUE);

	/*
	 * Get a snapshot of the timer counter value before it's started
	 * to compare against later.
	 */
	//CntValue1 = XScuTimer_GetCounterValue(TimerInstancePtr);

	/*
	 * Start the Scu Private Timer device.
	 */
	XScuTimer_Start(TimerInstancePtr);

	/*
	 * Read the value of the timer counter and wait for it to change,
	 * since it's decrementing it should change, if the hardware is not
	 * working for some reason, this loop could be infinite such that the
	 * function does not return.
	 */
	while (1) {
		CntValue2 = XScuTimer_GetCounterValue(TimerInstancePtr);
		if ( (CntValue2==0)) {
			XScuTimer_Stop(TimerInstancePtr);
			break;
		}
		//if(CntValue2 != CntValue1)xil_printf("TIMER:%d\r\n",CntValue2);
	}
	return XST_SUCCESS;
}

/*****************************************************************************/
/**
 * This function is the handler which performs processing for the I2s
 * Transmitter.
 * It is called from an interrupt context when the I2s Transmitter receives a
 * AES Block Complete Interrupt.
 *
 * This handler provides an example of how to handle I2s Transmitter interrupts
 * but is application specific.
 *
 * @param	CallBackRef is a pointer to the callback function
 *
 * @return	None.
 *
 * @note	None.
 *
******************************************************************************/
void I2sTxAesBlockCmplIntrHandler(void *CallBackRef)
{
	XI2s_Tx *InstancePtr = (XI2s_Tx *)CallBackRef;
	/* Set the interrupt received flag. */
	I2sTxIntrReceived = 1;
	xil_printf("INT:I2sTxAesCmpl\r\n");
	XI2s_Tx_IntrDisable(&I2sTxInstance, XI2S_TX_INTR_AES_BLKCMPLT_MASK);
}

/*****************************************************************************/
/**
 * This function is the handler which performs processing for the I2s
 * Transmitter.
 * It is called from an interrupt context when the I2s Transmitter receives a
 * AES Block Error Interrupt.
 *
 * This handler provides an example of how to handle I2s Transmitter interrupts
 * but is application specific.
 *
 * @param	CallBackRef is a pointer to the callback function
 *
 * @return	None.
 *
 * @note	None.
 *
******************************************************************************/
void I2sTxAesBlockErrIntrHandler(void *CallBackRef)
{
	XI2s_Tx *InstancePtr = (XI2s_Tx *)CallBackRef;
	/* Set the interrupt received flag. */
	InstancePtr = InstancePtr;
	I2sTxIntrReceived = 1;
	xil_printf("INT:I2sTxAesErr\r\n");
}

/*****************************************************************************/
/**
 * This function is the handler which performs processing for the I2s
 * Transmitter.
 * It is called from an interrupt context when the I2s Transmitter receives a
 * AES Channel Status Updated Interrupt.
 *
 * This handler provides an example of how to handle I2s Transmitter interrupts
 * but is application specific.
 *
 * @param	CallBackRef is a pointer to the callback function
 *
 * @return	None.
 *
 * @note	None.
 *
******************************************************************************/
void I2sTxAesGetChStsHandler(void *CallBackRef)
{
	XI2s_Tx *InstancePtr = (XI2s_Tx *)CallBackRef;
	/* Set the interrupt received flag. */
	I2sTxIntrReceived = 1;
	xil_printf("INT:I2sTxAesGet\r\n");
}

/*****************************************************************************/
/**
 * This function is the handler which performs processing for the I2s
 * Transmitter.
 * It is called from an interrupt context when the I2s Transmitter receives a
 * Underflow Interrupt.
 *
 * This handler provides an example of how to handle I2s Transmitter interrupts
 * but is application specific.
 *
 * @param	CallBackRef is a pointer to the callback function
 *
 * @return	None.
 *
 * @note	None.
 *
******************************************************************************/
void I2sTxUnderflowIntrHandler(void *CallBackRef)
{
	XI2s_Tx *InstancePtr = (XI2s_Tx *)CallBackRef;
	/* Set the interrupt received flag. */
	I2sTxIntrReceived = 1;
	xil_printf("INT:I2sTxAesUF\r\n");
}



/*****************************************************************************/
/**
 *
 * This function is called from the interrupt handler of audio formatter core.
 * After the first MM2S interrupt is received the interrupt_flag is set here.
 *
 * @return
 *
 * @note	This function assumes a Microblaze or ARM system and no
 *	operating system is used.
 *
*******************************************************************************/
void *XMM2SAFCallback(void *data)
{
	XAudioFormatter *AFInstancePtr = (XAudioFormatter *)data;
	/* clear interrupt flag */
	static int cntint = 0;
	MM2SAFIntrReceived = 1;
	AFInstancePtr->ChannelId = XAudioFormatter_MM2S;
	//xil_printf("MM2SINTSAF,MM2S STAT :%x\r\n",XAudioFormatter_ReadReg(XPAR_AUDIO_FORMATTER_0_BASEADDR,0x114));
	//XAudioFormatter_WriteReg(XPAR_AUDIO_FORMATTER_0_BASEADDR,0x114,0xC0000000);		//ステータスレジスタリセット
	//Xil_DCacheFlushRange(XPAR_AUDIO_FORMATTER_0_BASEADDR|0x114,4);
	//XAudioFormatterDMAStop(&AFInstance);
	if(cntint++ > 2000){
		//xil_printf("int\r\n");
		cntint = 0;
	}
	//xil_printf("INT0x%x\r\n",XAudioFormatterGetDMATransferCount(AFInstancePtr));
	//WriteDMAfifo();
	MM2SIntFlg = 1;
	//return;
}

/*****************************************************************************/
/**
 * This function does the lookup and intialization of the I2S transmitter.
 *
 * @param	I2sTxInstancePtr is a pointer to the I2sTx driver instance
 *
 * @return	XST_SUCCESS if the call is successful, otherwise XST_FAILURE.
 *
 * @note	None.
 *
******************************************************************************/
u32 InitializeI2sTx(XI2s_Tx *I2sTxInstancePtr)
{
	/*
	 * Lookup and Initialize the I2S transmitter so that it's ready to use.
	 */
	u32 Status;
	XI2stx_Config *I2STxConfig;
	I2STxConfig = XI2s_Tx_LookupConfig(I2S_TX_DEVICE_ID);
	if (I2STxConfig == NULL) {
		return XST_FAILURE;
	}

	Status = XI2s_Tx_CfgInitialize(I2sTxInstancePtr, I2STxConfig,
		I2STxConfig->BaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
//	Status = XScuGic_Connect(&InterruptController,	I2S_TX_INTERRUPT_ID,
//		(XInterruptHandler)XI2s_Tx_IntrHandler,
//		(void *)I2sTxInstancePtr);
//	if (Status == XST_SUCCESS) {
//		XScuGic_Enable(&InterruptController, I2S_TX_INTERRUPT_ID);
//	} else {
//		xil_printf("ERR:: Unable to register i2s tx interrupt handler");
//		return XST_FAILURE;
//	}
	//XScuGic_InterruptMaptoCpu(&InterruptController, 0x00, I2S_TX_INTERRUPT_ID);//20200907Add
//	XI2s_Tx_SetHandler(I2sTxInstancePtr, XI2S_TX_HANDLER_AES_BLKCMPLT,
//			&I2sTxAesBlockCmplIntrHandler,
//			(void *)I2sTxInstancePtr);
//	XI2s_Tx_SetHandler(I2sTxInstancePtr, XI2S_TX_HANDLER_AES_BLKSYNCERR,
//			&I2sTxAesBlockErrIntrHandler,
//			(void *)I2sTxInstancePtr);
//	XI2s_Tx_SetHandler(I2sTxInstancePtr, XI2S_TX_HANDLER_AES_CHSTSUPD,
//			&I2sTxAesGetChStsHandler,
//			(void *)I2sTxInstancePtr);
//	XI2s_Tx_SetHandler(I2sTxInstancePtr, XI2S_TX_HANDLER_AUD_UNDRFLW,
//			&I2sTxUnderflowIntrHandler,
//			(void *)I2sTxInstancePtr);
	XI2s_Tx_SetSclkOutDiv(I2sTxInstancePtr, I2S_TX_MCLK, I2S_TX_FS);
	XI2s_Tx_SetChMux(I2sTxInstancePtr, 0, XI2S_TX_CHMUX_AXIS_01);
//	XI2s_Tx_IntrEnable(I2sTxInstancePtr, XI2S_TX_GINTR_EN_MASK);
//	XI2s_Tx_IntrEnable(I2sTxInstancePtr, XI2S_TX_INTR_AES_BLKCMPLT_MASK);
//
//	XI2s_Tx_IntrEnable(I2sTxInstancePtr, XI2S_TX_INTR_AUDUNDRFLW_MASK);
	XI2s_Tx_Enable(I2sTxInstancePtr, 0x1);
	return XST_SUCCESS;
}



/*****************************************************************************/
/**
 * This function does the lookup and intialization of the audio formatter.
 *
 * @param	AFInstancePtr is a pointer to audio formatter driver instance
 *
 * @return	XST_SUCCESS if the call is successful, otherwise XST_FAILURE.
 *
 * @note	None.
 *
******************************************************************************/
u32 InitializeAudioFormatter(XAudioFormatter *AFInstancePtr)
{
	u32 Status;
	//u32 offset;

//	XAudioFormatter_Config *AFConfig;

	/*
	 * Lookup and Initialize the audio formatter so that it's ready to use.
	 */
	Status = XAudioFormatter_Initialize(&AFInstance, AF_DEVICE_ID);
	if (Status != XST_SUCCESS)
		return XST_FAILURE;

	//ScuTimerPolledExample(TIMER_DEVICE_ID);
	if (AFInstancePtr->mm2s_presence == 1) {
		xil_printf("AFInstancePtr->mm2s_presence == 1\r\n");
		Status = XScuGic_Connect(&InterruptController, AF_MM2S_INTERRUPT_ID,
			(XInterruptHandler)XAudioFormatterMM2SIntrHandler,
			(void *)AFInstancePtr);
		if (Status == XST_SUCCESS) {
			XScuGic_Enable(&InterruptController, AF_MM2S_INTERRUPT_ID);
		} else {
			xil_printf("Failed to register AF interrupt handler");
			return XST_FAILURE;
		}
		XScuGic_InterruptMaptoCpu(&InterruptController, 0x00, AF_MM2S_INTERRUPT_ID);//20200907Add

		AFInstancePtr->ChannelId = XAudioFormatter_MM2S;
		XAudioFormatter_SetMM2SCallback(AFInstancePtr,
			XAudioFormatter_IOC_Handler, XMM2SAFCallback,
			(void *)AFInstancePtr);

		XAudioFormatter_InterruptEnable(AFInstancePtr,
			XAUD_CTRL_IOC_IRQ_MASK);
		XAudioFormatterSetFsMultiplier(AFInstancePtr, AF_MCLK, AF_FS);
		af_hw_params.buf_addr = (&fifobuf);
		XAudioFormatterSetHwParams(AFInstancePtr, &af_hw_params);

		//XAudioFormatter_WriteReg(XPAR_AUDIO_FORMATTER_0_BASEADDR,
		//				0x120 ,(u32 *) (fifobuf));
		//Xil_DCacheFlushRange(XPAR_AUDIO_FORMATTER_0_BASEADDR|0x120,4);

		//XAudioFormatterDMAStop(AFInstancePtr);
		//XAudioFormatter_WriteReg(XPAR_AUDIO_FORMATTER_0_BASEADDR,0x114,0xC0000000);		//ステータスレジスタリセット
		//Xil_DCacheFlushRange(XPAR_AUDIO_FORMATTER_0_BASEADDR|0x114,4);

		//XAudioFormatterDMAStart(AFInstancePtr);
		//ScuTimerPolledExample(TIMER_DEVICE_ID);
		xil_printf("DMAstart(580)\r\n");
		//ScuTimerPolledExample(TIMER_DEVICE_ID);
		XAudioFormatterDMAStart(AFInstancePtr);
	}
	return Status;
}

void WriteDMAfifosimple(){
	XAudioFormatter *AFInstancePtr = &AFInstance;
	//#define INDEXMAX (sizeof(fifobuf)/sizeof(fifobuf[0]))
	static int pwrite_index=0;
	static int pread_index=0;
	static int pread_zero = 1;
	static u32 count = 0;
	u16 frmbuf[INDEXMAX * 2]={0};
	int write_index = 0;
	int write_index_buf = 0;
	int read_index = 0;
	u32 loc_index_eof=0;
	u8 Hbyte;
	u8 Lbyte;

	AFInstancePtr->ChannelId = XAudioFormatter_MM2S;
	read_index = XAudioFormatterGetDMATransferCount(AFInstancePtr) >> 2;//4bytes
	if(pread_index > read_index)pread_zero++;		//０を超えた時
	if(pread_zero > 1)pread_zero = 1;
	if(pwrite_index >= INDEXMAX)pwrite_index = INDEXMAX;		//範囲外参照防止

	if(pread_zero > 0){								//前回の一周分を読み出し済みの時

		Usb2I2sReader((u8*)frmbuf ,(INDEXMAX - pwrite_index)*2,&loc_index_eof);	//USB音声データ読み出し
		//xil_printf("eof:%d,",loc_index_eof);
		loc_index_eof >>= 1;
		loc_index_eof += pwrite_index;
		for(write_index = pwrite_index;write_index < loc_index_eof;write_index++){
			fifobuf[write_index] = ((u32)frmbuf[write_index - pwrite_index])<<8;

		}
		//ここでwrite_indexがINDEXMAXより大きい値をとることがある。たぶん+1とか
		if(write_index >= (INDEXMAX)){						//端まで来たらアドレスリセット
				write_index = 0;
				//if(pread_zero > 0){
					pread_zero=0;//--;							//writeが0を超えた
				//}
				//pwrite_index = write_index;
				count++;
			}

	}
	if(pwrite_index < INDEXMAX){
		Usb2I2sReader((u8*)frmbuf ,(read_index - pwrite_index)*2,&loc_index_eof);	//USB音声データ読み出し
		//xil_printf("eof:%d,",loc_index_eof);
		loc_index_eof >>= 1;
		loc_index_eof += pwrite_index;

		//xil_printf("eof:%d\r\n,",loc_index_eof);	//なぜかコメントアウトすると動かなくなる
		for(write_index = pwrite_index;write_index < loc_index_eof;write_index++){
			fifobuf[write_index] = ((u32)frmbuf[write_index - pwrite_index])<<8;
		}
		//ここでwrite_indexがINDEXMAXより大きい値をとることがある。たぶん+1とか
	}
	Xil_DCacheFlushRange((u32)fifobuf, INDEXMAX*4);//cache書き込み(startアドレス,byte数)


	pread_index = read_index;
	pwrite_index = write_index;
	//xil_printf("index:%d\r\n",write_index);
}

void WriteDMAfifo(){
	XAudioFormatter *AFInstancePtr = &AFInstance;
	//#define INDEXMAX 128
	static int pwrite_index=0;
	static int pread_index=0;
	static int pread_zero = 1;
	static u32 count = 0;
	static int index_remaining = 0;
	static int index_remaining2 = 0;
	static int pindex_remaining = 0;
	static u32 tstcnt =0;
	static int pbufready = 0;
	int deltareadindex=0;
	u16 frmbuf[INDEXMAX * 2]={0};
	int write_index = 0;
//	int write_index_buf = 0;
	int read_index = 0;
	u32 loc_index_eof=0;
//	u8 Hbyte;
//	u8 Lbyte;

	AFInstancePtr->ChannelId = XAudioFormatter_MM2S;
	read_index = XAudioFormatterGetDMATransferCount(AFInstancePtr) >> 2;//4bytes
	if(pread_index > read_index){
		deltareadindex = read_index + INDEXMAX - pread_index;
	}
	else{
		deltareadindex = read_index - pread_index;
	}
	index_remaining -= deltareadindex;	//バッファ残量追加


	if(pread_index > read_index)pread_zero++;		//０を超えた時
	if(pread_zero > 1)pread_zero = 1;
	if(pwrite_index > INDEXMAX)pwrite_index = INDEXMAX;		//範囲外参照防止

	if(pread_zero > 0){								//前回の一周分を読み出し済みの時

		Usb2I2sReader((u8*)frmbuf ,(INDEXMAX - pwrite_index)*2,&loc_index_eof);	//USB音声データ読み出し
		index_remaining2 = (INDEXMAX - pwrite_index)*2;
		//xil_printf("eof:%d,",loc_index_eof);
		loc_index_eof >>= 1;
		if(loc_index_eof > 0)index_remaining = 0;
		index_remaining += loc_index_eof;	//バッファ残量追加
		loc_index_eof += pwrite_index;
		for(write_index = pwrite_index;write_index < loc_index_eof;write_index++){
			fifobuf[write_index] = ((u32)frmbuf[write_index - pwrite_index])<<8;

		}


		//ここでwrite_indexがINDEXMAXより大きい値をとることがある。たぶん+1とか

		if(write_index >= INDEXMAX){						//端まで来たらアドレスリセット
			write_index = 0;
			if(pread_zero > 0)pread_zero = 0;//--;							//writeが0を超えた
			pwrite_index = write_index;
			count++;
		}/*else if((write_index > 1) && ((write_index + 1) < INDEXMAX)){
			fifobuf[write_index] = ((int)fifobuf[write_index - 2])/2;
			fifobuf[write_index + 1] = ((int)fifobuf[write_index - 1])/2;
		}*/
	}else {
		Usb2I2sReader((u8*)frmbuf ,(read_index - pwrite_index)*2,&loc_index_eof);	//USB音声データ読み出し
		//index_remaining2 = (read_index - pwrite_index)*2;
		//xil_printf("eof:%d,",loc_index_eof);
		loc_index_eof >>= 1;
		if(loc_index_eof > 0)index_remaining = 0;
		index_remaining += loc_index_eof;	//バッファ残量追加
		loc_index_eof += pwrite_index;

		//xil_printf("eof:%d\r\n,",loc_index_eof);	//なぜかコメントアウトすると動かなくなる
		for(write_index = pwrite_index;write_index < loc_index_eof;write_index++){
			fifobuf[write_index] = ((u32)frmbuf[write_index - pwrite_index])<<8;
		}

		/*if((write_index > 1) && ((write_index + 1) < INDEXMAX)&& ((write_index + 1) < read_index)){
			fifobuf[write_index] = ((int)fifobuf[write_index - 2])/2;
			fifobuf[write_index + 1] = ((int)fifobuf[write_index - 1])/2;
		}*/
		//ここでwrite_indexがINDEXMAXより大きい値をとることがある。たぶん+1とか
	}



	//if(index_remaining > 1145)index_remaining = 1145;
		//if(index_remaining < -1024)index_remaining = -1024;
		if(bufready){//index_remaining > -1024){
					/* LED点灯 */
					XGpio_DiscreteClear(&Gpio, LED_CHANNEL, LED_IN);
					if(pbufready == 0||bufstartflg){
						//xil_printf("DMAstart\r\n");
						XAudioFormatterDMAStart(AFInstancePtr);
						bufstartflg = 0;

					}

				}else{
				/* LED消灯 */
					XGpio_DiscreteWrite(&Gpio, LED_CHANNEL, LED_IN);
					memset(fifobuf,0,INDEXMAX*4);
					/*
					pwrite_index=0;
					pread_index=0;
					pread_zero = 1;
					read_index = 0;
					write_index = 0;
					*/
					if(pbufready == 1){
						XAudioFormatterDMAStop(AFInstancePtr);
						//xil_printf("DMAstop\r\n");
					}

										//write_index = read_index;
				}

	Xil_DCacheFlushRange((u32)fifobuf, INDEXMAX*4);//cache書き込み(startアドレス,byte数)
	pread_index = read_index;
	pwrite_index = write_index;
	pbufready = bufready;

	//feedbackadjust = pindex_remaining/32;
	if(tstcnt++ > 4000 ||((pindex_remaining < 128) && (index_remaining2 > 128))){
		xil_printf("ind_rem:%d\r\n",index_remaining2);
		tstcnt = 0;
	}
	 pindex_remaining = index_remaining2;
	if(index_remaining < 0)index_remaining = 0;
	usleep(2000000/dFrequency);
}


/********************************************************/
/* u32 *retbuf				バッファ開始アドレス				*/
/* u32 RequestedBytes		要求バイト数					*/
/* u32 RetBytes				応答バイト数(要求バイト数以内)		*/
/********************************************************/
void Usb2I2sReader(u8 *retbuf ,u32 RequestedBytes,
	u32 *RetBytes)
{
	u32 Size;

	Size = RequestedBytes;
//	static int cnt = 0;
	static u32 pSize = 0;
	//Residue += PacketResidue;		//レート計算用?
	//xil_printf("req=%d\r\n",Size);

	if (Index_eof <= Index_read) {
		/* Buffer is completed, retransmitting the same file data */
		if((Index_refresh == 0) && (bufready == 1)){		//バッファが空になったとき
			//Index = 0;
			//memset(&WrRamDiskPtr[0],(u8)0,Index_eof);
			Index = 0;
			Index_eof = 0;
			//xil_printf("RSTFIFO\r\n");
			bufready = 0;
		}
		Index_read = 0;
		Index_refresh = 0;
	}


	if(bufready == 0){
		Index_read = 0;
		Size = 0;
		//if(Index_eof > (8<<3))bufready = 1;		//バッファに読み出し量の8倍以上たまったら再生開始
		if(Index_eof > (((dFrequency + 1000) * 8 / 1000) * 2))bufready = 1;		//バッファに読み出し量の8倍以上たまったら再生開始(短くしすぎると再生できなくなる。)
		//xil_printf("FIFOEMPTY\r\n");
	}

	if(bufready == 1){

		if ((Index_read + Size) > (Index_eof)/*FileSize*/) {//0819-プつぷつ関係なし
			Size = Index_eof - Index_read;						//要求サイズが大きすぎるとき

			//xil_printf("Size=%d - %d\r\n",Index_eof,Index_read);
		}

		if(Size < 0){	//16bitなのでINDEXMAX*2。32bitならINDEXMAX
			//xil_printf("0x%x,0x%x[B]\r\n",Index_read,Size);
			Size = 0;
		}
		if(Size > INDEXMAX*2){	//16bitなのでINDEXMAX*2。32bitならINDEXMAX
				//xil_printf("0x%x,0x%x[B]\r\n",Index_read,Size);
				Size = INDEXMAX*2;
			}
		/*if(Size > 0)*/memcpy(retbuf,(1 == bufready)? &WrRamDiskPtr[Index_read] : BufferDummy,Size);		//バッファ量が十分でなければ0出力とする
	}

	if(1 == bufready) {
		Index_read += Size;								//バッファ準備中はカウントしない
		if(pSize < Size){
			pSize = Size;
			//xil_printf("ready:%d\r\n",Size);
//			cnt = 0;
		}
	}

	*RetBytes = Size;
}


/*****************************************************************************/
/**
*
* This is the XClk_WaitForLock function, it will wait for lock to settle change
* frequency value
*
* @param	CfgPtr_Dynamic provides pointer to clock wizard dynamic config
*
* @return
*		- XST_SUCCESS if the lock occurs.
*		- XST_FAILURE if timeout.
*
* @note		None
*
******************************************************************************/
u32 XClk_WaitForLock(XClk_Wiz_Config *CfgPtr_Dynamic)
{
	u32 Count = 0;

	while(!(*(u32 *)(CfgPtr_Dynamic->BaseAddr + 0x04) & CLK_LOCK)) {
		if(Count == 10000) {
			return XST_FAILURE;
			break;
		}
		usleep(100);
		Count++;
        }
	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* This function is the main entry point for the versal example using the
* XClk_Wiz driver. This function will set up the system with interrupts
* handlers.
*
* @param	DeviceId is the unique device ID of the CLK_WIZ
*		Subsystem core.
*
* @return
*		- XST_FAILURE if the system setup failed.
*		- XST_SUCCESS if successful.
*
******************************************************************************/
typedef union {
	struct {
		u8 DIVCLK_DIVIDE  : 8;			//0x200
		u8 CLKFBOUT_MULT  : 8; 			//0x201	Integer part of multiplier value i.e. For 8.125, this value is 8 = 0x8.
		u16 CLKFBOUT_FRAC_MULT : 10;	//0x202(0-875)Fractional part of multiplier value i.e. For 8.125, this value is 125 = 0x7D.
	} val;
	u32 Register;
}STClock_Config0;

typedef union {
	struct {
		u8 CLKOUT0_DIVIDE  		: 8;	//0x208 Integer part of multiplier value i.e. For 8.125, this value is 8 = 0x8.
		u16 CLKOUT0_FRAC_DIVIDE : 10; 	//0x201	(0-875)Fractional part of multiplier value i.e. For 8.125, this value is 125 = 0x7D.
	} val;
	u32 Register;
}STClock_Config2;

u32 ClkWiz_Example(XClk_Wiz *IntcInstancePtr, u32 DeviceId, int Frequency)
{

//	int ch = 2;//ch
//	int bit = 32;//bit
//	int sclkfreq = 100;	//MHz
//	double freq_outreq = 44.1;//kHz
//	double freq_request = (freq_outreq / 1000.0) * ch * bit ;	//MHz
//	int DIVCLK;//1-106
//	double CLKFBOUT_MUL = 2;//2.000-64.000(0.125刻み)
//	double CLKOUT0_DIV = 1.000;//1.000-128.000(0.125刻み)
//	double realCLKout = 100 * CLKFBOUT_MUL / (DIVCLK * CLKOUT0_DIV);	//MHz

	XClk_Wiz_Config *CfgPtr_Dynamic;
	u32 Status = XST_FAILURE;
	STClock_Config0 Clock_Config0;
	STClock_Config2 Clock_Config2;



	/*
	 * Get the CLK_WIZ Dynamic reconfiguration driver instance
	 */
	CfgPtr_Dynamic = XClk_Wiz_LookupConfig(DeviceId);
	if (!CfgPtr_Dynamic) {
		return XST_FAILURE;
	}

	/*
	 * Initialize the CLK_WIZ Dynamic reconfiguration driver
	 */
	Status = XClk_Wiz_CfgInitialize(&ClkWiz_Dynamic, CfgPtr_Dynamic,
		 CfgPtr_Dynamic->BaseAddr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* Calling Clock wizard dynamic reconfig */

	//*(u32 *)(CfgPtr_Dynamic->BaseAddr + 0x3F0) = 0;
	//XClk_Wiz_SetRate(&ClkWiz_Dynamic, 50);		//周波数SET[MHz]

	//*(u32 *)(CfgPtr_Dynamic->BaseAddr + 0x000) = 0x0000000A;	//Software Reset
	//98.29787MHz
	Clock_Config0.val.DIVCLK_DIVIDE = 5;//1-106
	Clock_Config0.val.CLKFBOUT_MULT = 57;//2-64
	Clock_Config0.val.CLKFBOUT_FRAC_MULT = 750;//125の倍数
	Clock_Config2.val.CLKOUT0_DIVIDE = 11;//1-128
	Clock_Config2.val.CLKOUT0_FRAC_DIVIDE = 750;//0,125,250,375,500,625,750,875

	switch(Frequency){
	case 44100:
		//44.1kHz(5.6448MHz)
			Clock_Config0.val.DIVCLK_DIVIDE = 2;//1-106
			Clock_Config0.val.CLKFBOUT_MULT = 29;//2-64
			Clock_Config0.val.CLKFBOUT_FRAC_MULT = 125;//625;//125の倍数
			Clock_Config2.val.CLKOUT0_DIVIDE = 123;//1-128
			Clock_Config2.val.CLKOUT0_FRAC_DIVIDE = 875;//250;//0,125,250,375,500,625,750,875
		break;
	case 48000:
		//48kHz(6.144MHz)
			Clock_Config0.val.DIVCLK_DIVIDE = 1;//7;//1-106
			Clock_Config0.val.CLKFBOUT_MULT = 16;//61;//2-64
			Clock_Config0.val.CLKFBOUT_FRAC_MULT = 0;//125;//125の倍数
			Clock_Config2.val.CLKOUT0_DIVIDE = 125;//1-128
			Clock_Config2.val.CLKOUT0_FRAC_DIVIDE = 0;//375;//0,125,250,375,500,625,750,875
		break;
	case 96000:
		//96kHz(12.288MHz)
			Clock_Config0.val.DIVCLK_DIVIDE = 1;//6;//1-106
			Clock_Config0.val.CLKFBOUT_MULT = 20;//61;//50;//2-64
			Clock_Config0.val.CLKFBOUT_FRAC_MULT = 0;//250;//875;//125の倍数
			Clock_Config2.val.CLKOUT0_DIVIDE = 78;//23;//1-128
			Clock_Config2.val.CLKOUT0_FRAC_DIVIDE = 125;//0;//0,125,250,375,500,625,750,875
		break;
	case 192000:
		//192kHz(24.576MHz)=Fs*128
			Clock_Config0.val.DIVCLK_DIVIDE = 1;//1-106
			Clock_Config0.val.CLKFBOUT_MULT = 24;//2-64
			Clock_Config0.val.CLKFBOUT_FRAC_MULT = 0;//875;//125の倍数
			Clock_Config2.val.CLKOUT0_DIVIDE = 46;//11;//1-128
			Clock_Config2.val.CLKOUT0_FRAC_DIVIDE = 875;//500;//0,125,250,375,500,625,750,875
		break;
	default:
		//48kHz(6.144MHz)
			Clock_Config0.val.DIVCLK_DIVIDE = 21;//1-106
			Clock_Config0.val.CLKFBOUT_MULT = 61;//2-64
			Clock_Config0.val.CLKFBOUT_FRAC_MULT = 125;//125の倍数
			Clock_Config2.val.CLKOUT0_DIVIDE = 47;//1-128
			Clock_Config2.val.CLKOUT0_FRAC_DIVIDE = 500;//0,125,250,375,500,625,750,875
		break;
	}

	/*
	switch(Frequency){
	case 44100:
		//44.1kHz(5.6448MHz)
			Clock_Config0.val.DIVCLK_DIVIDE = 9;//1-106
			Clock_Config0.val.CLKFBOUT_MULT = 63;//2-64
			Clock_Config0.val.CLKFBOUT_FRAC_MULT = 250;//125の倍数
			Clock_Config2.val.CLKOUT0_DIVIDE = 124;//1-128
			Clock_Config2.val.CLKOUT0_FRAC_DIVIDE = 500;//0,125,250,375,500,625,750,875
		break;
	case 48000:
		//48kHz(6.144MHz)
			Clock_Config0.val.DIVCLK_DIVIDE = 6;//1-106
			Clock_Config0.val.CLKFBOUT_MULT = 44;//2-64
			Clock_Config0.val.CLKFBOUT_FRAC_MULT = 375;//125の倍数
			Clock_Config2.val.CLKOUT0_DIVIDE = 120;//1-128
			Clock_Config2.val.CLKOUT0_FRAC_DIVIDE = 375;//0,125,250,375,500,625,750,875
		break;
	case 96000:
		//96kHz(12.288MHz)
			Clock_Config0.val.DIVCLK_DIVIDE = 5;//1-106
			Clock_Config0.val.CLKFBOUT_MULT = 48;//2-64
			Clock_Config0.val.CLKFBOUT_FRAC_MULT = 0;//125の倍数
			Clock_Config2.val.CLKOUT0_DIVIDE = 78;//1-128
			Clock_Config2.val.CLKOUT0_FRAC_DIVIDE = 125;//0,125,250,375,500,625,750,875
		break;
	case 192000:
		//162kHz(24.576MHz)
			Clock_Config0.val.DIVCLK_DIVIDE = 5;//1-106
			Clock_Config0.val.CLKFBOUT_MULT = 33;//34;//2-64
			Clock_Config0.val.CLKFBOUT_FRAC_MULT = 500;//250;//125の倍数
			Clock_Config2.val.CLKOUT0_DIVIDE = 27;//27;//1-128
			Clock_Config2.val.CLKOUT0_FRAC_DIVIDE = 0;//875;//0,125,250,375,500,625,750,875
		break;
	default:
		//48kHz(6.144MHz)
			Clock_Config0.val.DIVCLK_DIVIDE = 6;//1-106
			Clock_Config0.val.CLKFBOUT_MULT = 44;//2-64
			Clock_Config0.val.CLKFBOUT_FRAC_MULT = 375;//125の倍数
			Clock_Config2.val.CLKOUT0_DIVIDE = 120;//1-128
			Clock_Config2.val.CLKOUT0_FRAC_DIVIDE = 375;//0,125,250,375,500,625,750,875
		break;
	}
	*/

	//44.1kHz(16.9344MHz)


	*(u32 *)(CfgPtr_Dynamic->BaseAddr + 0x25C) = 0x1;		//設定値を内部レジスタロード
	Xil_DCacheFlushRange(CfgPtr_Dynamic->BaseAddr + 0x25C, 4);//cache書き込み(startアドレス,byte数)


	*(u32 *)(CfgPtr_Dynamic->BaseAddr + 0x200) = Clock_Config0.Register;
	Xil_DCacheFlushRange(CfgPtr_Dynamic->BaseAddr + 0x200, 4);//cache書き込み(startアドレス,byte数)
	*(u32 *)(CfgPtr_Dynamic->BaseAddr + 0x204) = Clock_Config0.Register;
	Xil_DCacheFlushRange(CfgPtr_Dynamic->BaseAddr + 0x204, 4);//cache書き込み(startアドレス,byte数)
	*(u32 *)(CfgPtr_Dynamic->BaseAddr + 0x208) = Clock_Config2.Register;
	Xil_DCacheFlushRange(CfgPtr_Dynamic->BaseAddr + 0x208, 4);//cache書き込み(startアドレス,byte数)
	Status = XClk_WaitForLock(CfgPtr_Dynamic);
	if (Status != XST_SUCCESS) {
			xil_printf("\n ERROR: Clock is not locked : 0x%x \t Expected "\
			": 0x1\n\r", *(u32 *)(CfgPtr_Dynamic->BaseAddr + 0x04) & CLK_LOCK);
		}

	*(u32 *)(CfgPtr_Dynamic->BaseAddr + 0x25C) =0x03;		//設定値を内部レジスタにセット（ここで初めて周波数が変わる）
	Xil_DCacheFlushRange(CfgPtr_Dynamic->BaseAddr + 0x25C, 4);//cache書き込み(startアドレス,byte数)
	Status = XClk_WaitForLock(CfgPtr_Dynamic);
	//xil_printf("CLKADDR:0x%x\n",CfgPtr_Dynamic->BaseAddr);

	if (Status != XST_SUCCESS) {
		xil_printf("\n ERROR: Clock is not locked : 0x%x \t Expected "\
		": 0x1\n\r", *(u32 *)(CfgPtr_Dynamic->BaseAddr + 0x04) & CLK_LOCK);
	}

	return Status;
}
