/** ============================================================================
 *  @file   helloDSP_config.h
 *
 *  @path   
 *
 *  @desc   Header file for MSGQ and POOL configurations for helloDSP.
 *
 *  @ver    1.10
 *  ============================================================================
 *  Copyright (c) Texas Instruments Incorporated 2002-2009
 *
 *  Use of this software is controlled by the terms and conditions found in the
 *  license agreement under which this software has been supplied or provided.
 *  ============================================================================
 */


#if !defined (helloDSP_CONFIG_)
#define helloDSP_CONFIG_

#if defined (__cplusplus)
extern "C" {
#endif /* defined (__cplusplus) */

/*  ----------------------------------- DSP/BIOS Headers            */
#include "helloDSPcfg.h"
#include <msgq.h>
#include <pool.h>

/*  ----------------------------------- DSP/BIOS LINK Headers       */
#include <dsplink.h>

#if defined (MSGQ_ZCPY_LINK)
#include <zcpy_mqt.h>
#include <sma_pool.h>
#endif /* if defined (MSGQ_ZCPY_LINK) */

/* Name of the MSGQ on the GPP and on the DSP. */
#define GPP_MSGQNAME        "GPPMSGQ1"
#define DSP_MSGQNAME        "DSPMSGQ"

/* ID of the POOL used by helloDSP. */
#define SAMPLE_POOL_ID      0

/* Argument size passed to the control message queue */
#define ARG1_SIZE 			256
#define ARG2_SIZE			128

/* Control message data structure. */
/* Must contain a reserved space for the header */
typedef struct ControlMsgS
{
    MSGQ_MsgHeader header;
    Uint16  command;
    int     arg1;                           // Cycles timer from DSP
    Uint16  arg2[ARG2_SIZE][ARG2_SIZE];     // 128x128, 16-bit-element matrix
} ControlMsgS;

typedef struct ControlMsgL
{
    MSGQ_MsgHeader header;
    Uint16  command;
    int     arg1;                           // Cycles timer from DSP
    Uint32  arg2[ARG2_SIZE/2][ARG2_SIZE];   // 64x128, 32-bit-element matrix
} ControlMsgL;

/* Messaging buffer used by the application.
 * Note: This buffer must be aligned according to the alignment expected
 * by the device/platform. */
#define APP_BUFFER_SIZE DSPLINK_ALIGN (sizeof (ControlMsgS), DSPLINK_BUF_ALIGN)

/* Number of pools configured in the system. */
#define NUM_POOLS          1

/* Number of local message queues */
#define NUM_MSG_QUEUES     1

/* Number of BUF pools in the entire memory pool */
#define NUM_MSG_POOLS      4

/* Number of messages in each BUF pool. */
#define NUM_MSG_IN_POOL0   1
#define NUM_MSG_IN_POOL1   2
#define NUM_MSG_IN_POOL2   2
#define NUM_MSG_IN_POOL3   4


#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */
#endif /* !defined (helloDSP_CONFIG_) */
