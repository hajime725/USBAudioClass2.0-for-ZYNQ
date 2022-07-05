/******************************************************************************
* Copyright (C) 2020 Xilinx, Inc.  All rights reserved.
* SPDX-License-Identifier: MIT
 ******************************************************************************/

/*****************************************************************************/
/**
 *
 * @file xusbps_class_audio.c
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
/***************************** Include Files *********************************/
#include "xusbps_class_audio.h"
#include "xil_cache.h"
/************************** Constant Definitions *****************************/

/***************** Macros (Inline Functions) Definitions *********************/
//#define CH9_DEBUG
//#define FREQ441HZ
/**************************** Type Definitions *******************************/
#ifdef CH9_DEBUG
#include <stdio.h>
#define printf xil_printf
#endif

/************************** Function Prototypes ******************************/

/************************** Variable Definitions *****************************/
u8 bMuteState = 0x00;
s16 wVolume = 0;		// 1/256dB刻み
u32 dFrequency = 192000;	//周波数
#define FREQ_LISTSIZE  4
u32 dFrequencyList[FREQ_LISTSIZE] = { //対応周波数リスト
		 44100,
		 48000,
		 96000,
		192000
};

u16 wRequestLength=0;
u16 wRequestObject=0;
/****************************************************************************/
/**
 * This function is called by Chapter9 handler when class request is received
 * from Host.
 *
 * @param	InstancePtr is pointer to Usb_DevData instance.
 * @param	SetupData is the setup packet received from Host.
 *
 * @note	None.
 *
 *****************************************************************************/
void XUsbPs_ClassReq(XUsbPs *InstancePtr, XUsbPs_SetupData *SetupData)
{
	s32 Status;
	u8 Error = 0;
	u32 ReplyLen;
	int xst_state=-1;
	u32 Timeout = 1000;
	u32	Reg;
	u8	EpNum = 0;
	u32     Handle;
	u8      *BufferPtr;
	u32     BufferLen = 0;
	u32 dFrequencyBuf = 0;
	int cnt = 0;
	int bufFlg = 0;

	static u8 Reply[XUSBPS_REQ_REPLY_LEN] ALIGNMENT_CACHELINE;
#ifndef XUSBPS_UAC1
	u8 UnitId = SetupData->wIndex >> 8;
#endif

	/* Check that the requested reply length is not bigger than our reply
	 * buffer. This should never happen...
	 */
	if (SetupData->wLength > XUSBPS_REQ_REPLY_LEN) {
		return;
	}

#ifdef CH9_DEBUG
	if((SetupData->bmRequestType & 0x80) == 0x80){
		xil_printf("\r\nREAD: C:bmRequestType 0x%x\r\n", SetupData->bmRequestType);
	}else{
		xil_printf("\r\nWRITE:C:bmRequestType 0x%x\r\n", SetupData->bmRequestType);
	}
	xil_printf("C:bRequest 0x%x\r\n", SetupData->bRequest);
	xil_printf("C:wValue 0x%x\r\n", SetupData->wValue);
	xil_printf("C:wIndex 0x%x\r\n", SetupData->wIndex);
	xil_printf("C:wLength 0x%x\r\n", SetupData->wLength);
#endif

	//////////////////リクエスト受信処理//////////////////
	if (((SetupData->bmRequestType & 0x80) == 0x00) && (SetupData->wLength > 0)) {
		/* Re-Prime the endpoint to receive Setup DATA */
		XUsbPs_EpPrime(InstancePtr, 0, XUSBPS_EP_DIRECTION_OUT);

		/* Check whether EP prime is successful or not */
		Timeout = XUSBPS_TIMEOUT_COUNTER;
		do {
		Reg = XUsbPs_ReadReg(InstancePtr->Config.BaseAddress,
						XUSBPS_EPPRIME_OFFSET);
		} while(((Reg & (1 << EpNum)) == 1) && --Timeout);

		if (!Timeout) {
			xst_state = XST_FAILURE;
			//return;// XST_FAILURE;
		}

		/* Get the Setup DATA, don't wait for the interrupt */
		Timeout = XUSBPS_TIMEOUT_COUNTER;
		do {
			Status = XUsbPs_EpBufferReceive(InstancePtr,
				EpNum, &BufferPtr, &BufferLen, &Handle);
		} while((Status != XST_SUCCESS) && --Timeout);

		if (!Timeout) {
			xst_state = XST_FAILURE;
			//return;// XST_FAILURE;
		}

		Xil_DCacheInvalidateRange((unsigned int)BufferPtr,
							BufferLen);
#ifdef CH9_DEBUG
		xil_printf("REQUEST-RX:: 0x%x,0x%x,0x%x,0x%x--Len:%d\r\n", BufferPtr[0],BufferPtr[1],BufferPtr[2],BufferPtr[3],BufferLen);
#endif
	}
	//////////////////リクエスト受信処理END//////////////////

	switch (SetupData->bRequest) {
#ifdef XUSBPS_UAC1
	case UAC1_SET_CUR:
		ReplyLen = SetupData->wLength;
		XUsbPs_EpDataBufferReceive((XUsbPs *)InstancePtr, 0, Reply,
						ReplyLen);

		XUsbPs_EpBufferSend((XUsbPs *)InstancePtr, 0, NULL, 0);

		break;
	case UAC1_GET_CUR:
		ReplyLen = SetupData->wLength;
		Reply[0] = (u8)0x40;
		Reply[1] = (u8)0x1F;
		Reply[2] = (u8)0x00;

		Status = XUsbPs_EpBufferSend((XUsbPs *)InstancePtr, 0,
				Reply, ReplyLen);
		if (XST_SUCCESS != Status) {
			/* Failure case needs to be handled */
			for (;;);
		}

		break;
	case UAC1_GET_MIN:
		ReplyLen = SetupData->wLength;
		Reply[0] = (u8)0x40;
		Reply[1] = (u8)0x1F;
		Reply[2] = (u8)0x00;

		Status = XUsbPs_EpBufferSend((XUsbPs *)InstancePtr, 0,
				Reply, ReplyLen);
		if (XST_SUCCESS != Status) {
			/* Failure case needs to be handled */
			for (;;);
		}

		break;
	case UAC1_GET_MAX:
		ReplyLen = SetupData->wLength;
		Reply[0] = (u8)0x00;
		Reply[1] = (u8)0x77;
		Reply[2] = (u8)0x01;

		Status = XUsbPs_EpBufferSend((XUsbPs *)InstancePtr, 0,
				Reply, ReplyLen);
		if (XST_SUCCESS != Status) {
			/* Failure case needs to be handled */
			for (;;);
		}

		break;
	case UAC1_GET_RES:
		ReplyLen = SetupData->wLength;
		Reply[0] = (u8)0x30;
		Reply[1] = (u8)0x00;

		Status = XUsbPs_EpBufferSend((XUsbPs *)InstancePtr, 0,
				Reply, ReplyLen);
		if (XST_SUCCESS != Status) {
			/* Failure case needs to be handled */
			for (;;);
		}

		break;
#else	/*	XUSPBS_UAC2 */

	case UAC2_CS_CUR:
#ifdef CH9_DEBUG
	xil_printf("bRequest:UAC2_CS_CUR,UnitId=%d \r\n",UnitId);
#endif
	switch(UnitId) {
		case USB_CLK_SRC_ID:			//5
#ifdef CH9_DEBUG
			xil_printf("USB_CLK_SRC_ID \r\n");
#endif
			switch(SetupData->wValue >> 8) {//wValue=0:マスタチャンネル,1:1ch,2:2ch
				case UAC2_CS_CONTROL_SAM_FREQ:
	#ifdef CH9_DEBUG
						xil_printf("UAC2_CS_CONTROL_SAM_FREQ\r\n");
	#endif
					if ((SetupData->bmRequestType &
						XUSBPS_ENDPOINT_DIR_MASK) == 0) {
						/* Set Request */
						ReplyLen = SetupData->wLength;
//						XUsbPs_EpDataBufferReceive(
//								(XUsbPs *)InstancePtr,
//								0,
//								Reply, ReplyLen);


						if(BufferLen > 0)memcpy(Reply,BufferPtr,BufferLen);
#ifdef CH9_DEBUG
						xil_printf("REQUEST::UAC2_CS_CONTROL_SAM_FREQ: 0x%x,0x%x,0x%x,0x%x--Rep:%d,Buf:%d\r\n", Reply[0],Reply[1],Reply[2],Reply[3],ReplyLen,BufferLen);
#endif
						dFrequencyBuf = (BufferPtr[3] << 24) | (BufferPtr[2] << 16) | (BufferPtr[1] << 8) | (BufferPtr[0]);

						if(BufferLen == 4){
							bufFlg = 0;
							for(cnt = 0;cnt < FREQ_LISTSIZE; cnt++){					// bufFlg==1:リストに存在する周波数設定
								if(dFrequencyList[cnt] == dFrequencyBuf)bufFlg = 1;
							}
							if(1 == bufFlg) dFrequency = dFrequencyBuf;		// サンプリング周波数をセット

						}
#ifdef CH9_DEBUG
						xil_printf("buf=%dHz,Freq=%dHz",dFrequencyBuf,dFrequency);
#endif
						ReplyLen = BufferLen;

						XUsbPs_EpBufferSend(
								(XUsbPs *)InstancePtr,
								0,
								NULL, 0);
					} else {
						/* Get Request */
						ReplyLen = SetupData->wLength > 4 ? 4 :
							SetupData->wLength;

	#ifdef  FREQ441HZ
						Reply[0] = (u8)0x44;
						Reply[1] = (u8)0xAC;
						Reply[2] = (u8)0x00;
						Reply[3] = (u8)0x00;
	#else
						Reply[0] = (u8)(dFrequency & 0xff);			//windowsは、サンプリングの設定に成功したかどうかはここで見ている。
						Reply[1] = (u8)((dFrequency >> 8) & 0xff);
						Reply[2] = (u8)((dFrequency >> 16) & 0xff);
						Reply[3] = (u8)((dFrequency >> 24) & 0xff);
//						Reply[0] = (u8)0x00;//96kHz
//						Reply[1] = (u8)0x77;
//						Reply[2] = (u8)0x01;
						//0x00, 0x77, 0x01
	#endif

	#ifdef CH9_DEBUG
						xil_printf("UAC2_CS_CONTROL_SAM_FREQ: 0x%x,0x%x,0x%x,0x%x\r\n", Reply[0],Reply[1],Reply[2],Reply[3]);
	#endif

						Status = XUsbPs_EpBufferSend(
								(XUsbPs *)InstancePtr,
								0,
								Reply, ReplyLen);
						if (XST_SUCCESS != Status) {
						/* Failure case needs to be handled */
							for (;;);
						}
					}

					break;
				case UAC2_CS_CONTROL_CLOCK_VALID:
#ifdef CH9_DEBUG
					xil_printf("UAC2_CS_CONTROL_CLOCK_VALID\r\n");
#endif
					ReplyLen = SetupData->wLength > 4 ? 4 :
						SetupData->wLength;
					/* Internal clock always valid */
					Reply[0] = (u8)0x01;
	#ifdef CH9_DEBUG
					xil_printf("UAC2_CS_CONTROL_CLOCK_VALID: 0x%x\r\n", Reply[0]);
	#endif

					Status = XUsbPs_EpBufferSend(
							(XUsbPs *)InstancePtr,
							0,
							Reply, ReplyLen);
					if (XST_SUCCESS != Status) {
						/* Failure case needs to be handled */
						for (;;);
					}

					break;
				default:
				/* Unknown Control Selector for Clock Unit */
#ifdef CH9_DEBUG
				xil_printf("Error\r\n");
#endif
				Error = 1;
				break;
			}

			break;
		case USB_CLK_SEL_ID:			//6
#ifdef CH9_DEBUG
			xil_printf("USB_CLK_SEL_ID \r\n");
#endif
			if ((SetupData->bmRequestType &
					XUSBPS_ENDPOINT_DIR_MASK) == 0) {
				/* Set Request */
				ReplyLen = SetupData->wLength;

//				XUsbPs_EpDataBufferReceive(
//						(XUsbPs *)InstancePtr, 0,
//						Reply, ReplyLen);
			if(BufferLen > 0)memcpy(Reply,BufferPtr,BufferLen);
#ifdef CH9_DEBUG
							xil_printf("REQUEST::0x%x,0x%x,0x%x,0x%x--Rep:%d,Buf:%d\r\n", Reply[0],Reply[1],Reply[2],Reply[3],ReplyLen,BufferLen);
#endif
							ReplyLen = BufferLen;

				XUsbPs_EpBufferSend((XUsbPs *)InstancePtr, 0,
						NULL, 0);
			} else {
				/* Get Request */
				ReplyLen = SetupData->wLength > 4 ? 4 :
					SetupData->wLength;
				Reply[0] = (u8)0x01;

				Status = XUsbPs_EpBufferSend(
						(XUsbPs *)InstancePtr,
						0,
						Reply, ReplyLen);
				if (XST_SUCCESS != Status) {
					/* Failure case needs to be handled */
					for (;;);
				}
			}

			break;
		case OUT_FETR_UNT_ID:		//7
		case IN_FETR_UNT_ID:		//8
#ifdef CH9_DEBUG
			if(UnitId==OUT_FETR_UNT_ID)xil_printf("OUT_FETR_UNT_ID \r\n");
			else xil_printf("IN_FETR_UNT_ID \r\n");
#endif
		switch(SetupData->wValue >> 8) {
			case UAC2_FU_VOLUME_CONTROL:
#ifdef CH9_DEBUG
					xil_printf("UAC2_FU_VOLUME_CONTROL \r\n");
#endif
				/* Feature not available */
				if ((SetupData->bmRequestType &
					XUSBPS_ENDPOINT_DIR_MASK) == 0) {
					/* Set Request */
					ReplyLen = SetupData->wLength;
					wRequestLength = ReplyLen;
//					XUsbPs_EpDataBufferReceive(
//							(XUsbPs *)InstancePtr,
//							0,
//							Reply, ReplyLen);
					if(BufferLen > 0)memcpy(Reply,BufferPtr,BufferLen);
#ifdef CH9_DEBUG
					xil_printf("REQUEST::0x%x,0x%x,0x%x,0x%x--Rep:%d,Buf:%d\r\n", Reply[0],Reply[1],Reply[2],Reply[3],ReplyLen,BufferLen);
#endif
					ReplyLen = BufferLen;

					XUsbPs_EpBufferSend(
							(XUsbPs *)InstancePtr,
							0,
							NULL, 0);
					wRequestObject=2;
					wVolume = ((((int)Reply[1])&0xff)<<8) |(((int)Reply[0])&0xff);
#ifdef CH9_DEBUG
					xil_printf("wVolume=%d \r\n",wVolume);
#endif
				} else {
					/* Get Request */

					ReplyLen = SetupData->wLength > 4 ? 4 :
						SetupData->wLength;
					Reply[1] = (wVolume >> 8) & 0xff;
					Reply[0] = wVolume & 0xff;
#ifdef CH9_DEBUG
					xil_printf("wVolume=%d \r\n",wVolume);
#endif

					Status = XUsbPs_EpBufferSend(
							(XUsbPs *)InstancePtr,
							0,
							Reply, ReplyLen);
					if (XST_SUCCESS != Status) {
					/* Failure case needs to be handled */
						for (;;);
					}
				}

				break;
			case UAC2_FU_MUTE_CONTROL:
#ifdef CH9_DEBUG
					xil_printf("UAC2_FU_MUTE_CONTROL \r\n");
#endif
				/* Feature not available */
				if ((SetupData->bmRequestType &
					XUSBPS_ENDPOINT_DIR_MASK) == 0) {//セットリクエスト
					/* Set Request */
					ReplyLen = SetupData->wLength;
					wRequestLength = ReplyLen;
					wRequestObject=1;
//					xst_state = XUsbPs_EpDataBufferReceive(
//							(XUsbPs *)InstancePtr,
//							0,
//							Reply, ReplyLen);
					if(BufferLen > 0)memcpy(Reply,BufferPtr,BufferLen);
#ifdef CH9_DEBUG
					xil_printf("REQUEST:: 0x%x,0x%x,0x%x,0x%x--Rep:%d,Buf:%d\r\n", Reply[0],Reply[1],Reply[2],Reply[3],ReplyLen,BufferLen);
#endif
					ReplyLen = BufferLen;
					bMuteState = Reply[0]?0x01:0x00;//0x01:ミュート,0x00:ミュート解除
					XUsbPs_EpBufferSend(
							(XUsbPs *)InstancePtr,
							0,
							NULL, 0);

#ifdef CH9_DEBUG
					xil_printf("%d,len=%d",Reply[0],ReplyLen);
					xil_printf(bMuteState?"mute\r\n":"unmute\r\n");
#endif
				} else {
					/* Get Request */
					ReplyLen = SetupData->wLength > 4 ? 4 :
						SetupData->wLength;
					Reply[0] = bMuteState;
#ifdef CH9_DEBUG
					xil_printf(bMuteState?"mute\r\n":"unmute\r\n");
#endif
					Status = XUsbPs_EpBufferSend(
							(XUsbPs *)InstancePtr,
							0,
							Reply, ReplyLen);
					if (XST_SUCCESS != Status) {
					/* Failure case needs to be handled */
						xil_printf("Failure\r\n");
						//for (;;);
					}
				}

				break;
			default:
#ifdef CH9_DEBUG
					xil_printf("ERROR \r\n");
#endif
				/* Unknown Control Selector for Feature Unit */
				Error = 1;
				break;
		}

		break;
	default:
		/* Unknown unit ID */
#ifdef CH9_DEBUG
			xil_printf("Unknown unit ID \r\n");
#endif
		Error = 1;
			break;
	}

	break;
	case UAC2_CS_RANGE:
#ifdef CH9_DEBUG
	xil_printf("bRequest:UAC2_CS_RANGE\r\n");
#endif
		switch(UnitId) {
		case USB_CLK_SRC_ID:
#ifdef CH9_DEBUG
			xil_printf("USB_CLK_SRC_ID\r\n");
#endif
			switch(SetupData->wValue >> 8) {
			case UAC2_CS_CONTROL_SAM_FREQ:
				//windowsは個々の中から使えるサンプルレートを探す。
				//44.1kHzを必ず含む必要がある。
				ReplyLen = 0;
				Reply[ReplyLen++] = (u8)0x04;//0x01;	//wNumSubRanges_L
				Reply[ReplyLen++] = (u8)0x00;	//wNumSubRanges_H
										//44.1kHz
				Reply[ReplyLen++] = (u8)0x44;	//dMIN(1) 0x FF 00 00 00
				Reply[ReplyLen++] = (u8)0xAC;	//dMIN(1) 0x 00 FF 00 00
				Reply[ReplyLen++] = (u8)0x00;	//dMIN(1) 0x 00 00 FF 00
				Reply[ReplyLen++] = (u8)0x00;	//dMIN(1) 0x 00 00 00 FF

				Reply[ReplyLen++] = (u8)0x44;	//dMAX(1) 0x FF 00 00 00
				Reply[ReplyLen++] = (u8)0xAC;	//dMAX(1) 0x 00 FF 00 00
				Reply[ReplyLen++] = (u8)0x00;	//dMAX(1) 0x 00 00 FF 00
				Reply[ReplyLen++] = (u8)0x00;	//dMAX(1) 0x 00 00 00 FF

				Reply[ReplyLen++] = (u8)0x00;	//dRES(1) 0x FF 00 00 00
				Reply[ReplyLen++] = (u8)0x00;	//dRES(1) 0x 00 FF 00 00
				Reply[ReplyLen++] = (u8)0x00;	//dRES(1) 0x 00 00 FF 00
				Reply[ReplyLen++] = (u8)0x00;	//dRES(1) 0x 00 00 00 FF
										//48kHz
				Reply[ReplyLen++] = (u8)0x80;	//dMIN(2) 0x FF 00 00 00
				Reply[ReplyLen++] = (u8)0xBB;	//dMIN(2) 0x 00 FF 00 00
				Reply[ReplyLen++] = (u8)0x00;	//dMIN(2) 0x 00 00 FF 00
				Reply[ReplyLen++] = (u8)0x00;	//dMIN(2) 0x 00 00 00 FF

				Reply[ReplyLen++] = (u8)0x80;	//dMAX(2) 0x FF 00 00 00
				Reply[ReplyLen++] = (u8)0xBB;	//dMAX(2) 0x 00 FF 00 00
				Reply[ReplyLen++] = (u8)0x00;	//dMAX(2) 0x 00 00 FF 00
				Reply[ReplyLen++] = (u8)0x00;	//dMAX(2) 0x 00 00 00 FF

				Reply[ReplyLen++] = (u8)0x00;	//dRES(2) 0x FF 00 00 00
				Reply[ReplyLen++] = (u8)0x00;	//dRES(2) 0x 00 FF 00 00
				Reply[ReplyLen++] = (u8)0x00;	//dRES(2) 0x 00 00 FF 00
				Reply[ReplyLen++] = (u8)0x00;	//dRES(2) 0x 00 00 00 FF
								//96kHz
				Reply[ReplyLen++] = (u8)0x00;	//dMIN(2) 0x FF 00 00 00
				Reply[ReplyLen++] = (u8)0x77;	//dMIN(2) 0x 00 FF 00 00
				Reply[ReplyLen++] = (u8)0x01;	//dMIN(2) 0x 00 00 FF 00
				Reply[ReplyLen++] = (u8)0x00;	//dMIN(2) 0x 00 00 00 FF

				Reply[ReplyLen++] = (u8)0x00;	//dMAX(2) 0x FF 00 00 00
				Reply[ReplyLen++] = (u8)0x77;	//dMAX(2) 0x 00 FF 00 00
				Reply[ReplyLen++] = (u8)0x01;	//dMAX(2) 0x 00 00 FF 00
				Reply[ReplyLen++] = (u8)0x00;	//dMAX(2) 0x 00 00 00 FF

				Reply[ReplyLen++] = (u8)0x00;	//dRES(2) 0x FF 00 00 00
				Reply[ReplyLen++] = (u8)0x00;	//dRES(2) 0x 00 FF 00 00
				Reply[ReplyLen++] = (u8)0x00;	//dRES(2) 0x 00 00 FF 00
				Reply[ReplyLen++] = (u8)0x00;	//dRES(2) 0x 00 00 00 FF
								//192kHz
				Reply[ReplyLen++] = (u8)0x00;	//dMIN(2) 0x FF 00 00 00
				Reply[ReplyLen++] = (u8)0xEE;	//dMIN(2) 0x 00 FF 00 00
				Reply[ReplyLen++] = (u8)0x02;	//dMIN(2) 0x 00 00 FF 00
				Reply[ReplyLen++] = (u8)0x00;	//dMIN(2) 0x 00 00 00 FF

				Reply[ReplyLen++] = (u8)0x00;	//dMAX(2) 0x FF 00 00 00
				Reply[ReplyLen++] = (u8)0xEE;	//dMAX(2) 0x 00 FF 00 00
				Reply[ReplyLen++] = (u8)0x02;	//dMAX(2) 0x 00 00 FF 00
				Reply[ReplyLen++] = (u8)0x00;	//dMAX(2) 0x 00 00 00 FF

				Reply[ReplyLen++] = (u8)0x00;	//dRES(2) 0x FF 00 00 00
				Reply[ReplyLen++] = (u8)0x00;	//dRES(2) 0x 00 FF 00 00
				Reply[ReplyLen++] = (u8)0x00;	//dRES(2) 0x 00 00 FF 00
				Reply[ReplyLen++] = (u8)0x00;	//dRES(2) 0x 00 00 00 FF

				ReplyLen = SetupData->wLength > ReplyLen ? ReplyLen ://14 ? 14 :	//2+12*n
									SetupData->wLength;

#ifdef CH9_DEBUG
	xil_printf("UAC2_CS_RANGE:USB_CLK_SRC_ID: 0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,\r\n",
			Reply[0],Reply[1],Reply[2],Reply[3],Reply[4],Reply[5],Reply[6],Reply[7],Reply[8],Reply[9],Reply[10],Reply[11],Reply[12],Reply[13]);
#endif
				Status = XUsbPs_EpBufferSend(
						(XUsbPs *)InstancePtr,
						0,
						Reply, ReplyLen);
				if (XST_SUCCESS != Status) {
					/* Failure case needs to be handled */
					for (;;);
				}

				break;
			default:
				/* Unknown Clock Source Range Request */
				Error = 1;
				break;
			}

			break;
		case OUT_FETR_UNT_ID:
		case IN_FETR_UNT_ID:
#ifdef CH9_DEBUG
			if(UnitId==OUT_FETR_UNT_ID)xil_printf("OUT_FETR_UNT_ID \r\n");
			else xil_printf("IN_FETR_UNT_ID \r\n");
#endif
			switch(SetupData->wValue >> 8) {
				case UAC2_FU_VOLUME_CONTROL:
#ifdef CH9_DEBUG
					xil_printf("UAC2_FU_VOLUME_CONTROL::Len=%d \r\n",SetupData->wLength);//dB
#endif
					/* Feature not available */
					ReplyLen = SetupData->wLength >
						14 ? 14 :
						SetupData->wLength;
					// 1/256dB(0x0001)刻み
					// +127.9961dB(0x7FFF)～-127.9961dB(0x7FFF)の値
					Reply[0] = (u8)0x01;	//wNumSubRangesL
					Reply[1] = (u8)0x00;	//wNumSubRangesH
					Reply[2] = (u8)0x00;	//wMINL(dB)
					Reply[3] = (u8)0x81;	//wMINH(dB)
					Reply[4] = (u8)0x00;	//wMAXL(dB)
					Reply[5] = (u8)0x00;	//wMAXH(dB)
					Reply[6] = (u8)0x00;	//wRESL(dB)
					Reply[7] = (u8)0x01;	//wRESH(dB)


					Status = XUsbPs_EpBufferSend(
							(XUsbPs *)InstancePtr,
							0,
							Reply, ReplyLen);
					if (XST_SUCCESS != Status) {
					/* Failure case needs to be handled */
						for (;;);
					}

					break;
				default:
				/* Unknown Control Selector for Feature Unit */
					Error = 1;
					break;
			}

			break;
		default:
			/* Unknown unit ID */
			Error = 1;
				break;
		}

		break;
#endif  /* end of XUSBPS_UAC2 */

		default:
#ifdef CH9_DEBUG
	xil_printf("bRequest:ERROR\r\n");
#endif
			Error = 1;
			break;
	}

	/* Set the send stall bit if there is an error */
	if (Error) {
#ifdef CH9_DEBUG
		printf("std dev req %d/%d error, stall 0 in out\n",
			SetupData->bRequest, (SetupData->wValue >> 8) & 0xff);
#endif
		XUsbPs_EpStall((XUsbPs *)InstancePtr, 0U,
			XUSBPS_EP_DIRECTION_IN | XUSBPS_EP_DIRECTION_OUT);
	}
}
