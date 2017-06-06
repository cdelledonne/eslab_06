/** ============================================================================
 *  @file   tskMessage.c
 *
 *  @path   
 *
 *  @desc   This is simple TSK based application that uses MSGQ. It receives
 *          and transmits messages from/to the GPP and runs the DSP
 *          application code (located in an external source file)
 *
 *  @ver    1.10
 */


/*  ----------------------------------- DSP/BIOS Headers            */
#include "helloDSPcfg.h"
#include <gbl.h>
#include <sys.h>
#include <sem.h>
#include <msgq.h>
#include <pool.h>

/*  ----------------------------------- DSP/BIOS LINK Headers       */
#include <dsplink.h>
#include <platform.h>
#include <failure.h>

/*  ----------------------------------- Sample Headers              */
#include <helloDSP_config.h>
#include <tskMessage.h>

/*  ----------------------------------- Timer Headers               */
#include "c6x.h"

#ifdef __cplusplus
extern "C" {
#endif


/* FILEID is used by SET_FAILURE_REASON macro. */
#define FILEID  FID_APP_C

/* Place holder for the MSGQ name created on DSP */
Uint8 dspMsgQName[DSP_MAX_STRLEN];

/* Number of iterations message transfers to be done by the application. */
extern Uint16 matrixSize;

#define MAXSIZE         128

/* Matrices to store results from GPP */
Uint16 mat1[MAXSIZE][MAXSIZE];
Uint16 mat2[MAXSIZE][MAXSIZE];


/** ============================================================================
 *  @func   TSKMESSAGE_create
 *
 *  @desc   Create phase function for the TSKMESSAGE application. Initializes
 *          the TSKMESSAGE_TransferInfo structure with the information that will
 *          be used by the other phases of the application.
 *
 *  @modif  None.
 *  ============================================================================
 */
Int TSKMESSAGE_create(TSKMESSAGE_TransferInfo** infoPtr)
{
    Int status = SYS_OK;
    MSGQ_Attrs msgqAttrs = MSGQ_ATTRS;
    TSKMESSAGE_TransferInfo* info = NULL;
    MSGQ_LocateAttrs syncLocateAttrs;

    /* Allocate TSKMESSAGE_TransferInfo structure that will be initialized
     * and passed to other phases of the application */
    *infoPtr = MEM_calloc(DSPLINK_SEGID, sizeof(TSKMESSAGE_TransferInfo), DSPLINK_BUF_ALIGN);
    if (*infoPtr == NULL)
    {
        status = SYS_EALLOC;
        SET_FAILURE_REASON(status);
    }
    else
    {
        info = *infoPtr;
        info->matrixSize = matrixSize;
        info->localMsgq = MSGQ_INVALIDMSGQ;
        info->locatedMsgq = MSGQ_INVALIDMSGQ;
    }

    if (status == SYS_OK)
    {
        /* Set the semaphore to a known state. */
        SEM_new(&(info->notifySemObj), 0);

        /* Fill in the attributes for this message queue. */
        msgqAttrs.notifyHandle = &(info->notifySemObj);
        msgqAttrs.pend = (MSGQ_Pend) SEM_pendBinary;
        msgqAttrs.post = (MSGQ_Post) SEM_postBinary;

        SYS_sprintf((Char *)dspMsgQName, "%s%d", DSP_MSGQNAME, GBL_getProcId());

        /* Creating message queue */
        status = MSGQ_open((String)dspMsgQName, &info->localMsgq, &msgqAttrs);
        if (status != SYS_OK)
        {
            SET_FAILURE_REASON(status);
        }
        else
        {
            /* Set the message queue that will receive any async. errors. */
            MSGQ_setErrorHandler(info->localMsgq, SAMPLE_POOL_ID);

            /* Synchronous locate.                           */
            /* Wait for the initial startup message from GPP */
            status = SYS_ENOTFOUND;
            while ((status == SYS_ENOTFOUND) || (status == SYS_ENODEV))
            {
                syncLocateAttrs.timeout = SYS_FOREVER;
                status = MSGQ_locate(GPP_MSGQNAME, &info->locatedMsgq, &syncLocateAttrs);
                if ((status == SYS_ENOTFOUND) || (status == SYS_ENODEV))
                {
                    TSK_sleep(1000);
                }
                else if(status != SYS_OK)
                {
#if !defined (LOG_COMPONENT)
                    LOG_printf(&trace, "MSGQ_locate (msgqOut) failed. Status = 0x%x\n", status);
#endif
                }
            }
        }
       /* Initialize the sequenceNumber */
        info->sequenceNumber = 0;
    }

    return status;
}


/** ============================================================================
 *  @func   TSKMESSAGE_execute
 *
 *  @desc   Execute phase function for the TSKMESSAGE application. Application
 *          receives a message, verifies the id and executes the DSP processing.
 *
 *  @modif  None.
 *  ============================================================================
 */
Int TSKMESSAGE_execute(TSKMESSAGE_TransferInfo* info)
{
    Int status = SYS_OK;
    /* Two different types of message (see struct typedef for more info) */
    ControlMsgS* msgS;
    ControlMsgL* msgL;
    Uint8 i;
    Uint8 j, k, l;

    /* Pointers to a matrix */
    Uint16 (*matrixPtS)[MAXSIZE];
    Uint32 (*matrixPtL)[MAXSIZE];

    /* Cycle counters */
    int start, stop, total;

    /* Allocate and send the message */
    status = MSGQ_alloc(SAMPLE_POOL_ID, (MSGQ_Msg*) &msgS, APP_BUFFER_SIZE);
    matrixPtS = msgS->arg2;

    if (status == SYS_OK)
    {
        MSGQ_setMsgId((MSGQ_Msg) msgS, info->sequenceNumber);
        MSGQ_setSrcQueue((MSGQ_Msg) msgS, info->localMsgq);
        msgS->command = 0x01;

        status = MSGQ_put(info->locatedMsgq, (MSGQ_Msg) msgS);
        if (status != SYS_OK)
        {
            /* Must free the message */
            MSGQ_free ((MSGQ_Msg) msgS);
            SET_FAILURE_REASON(status);
        }
    }
    else
    {
        SET_FAILURE_REASON(status);
    }

    /* Execute the loop to receive the two matrices and send back the result */
    for (i = 0; ( (i < 2) && (status == SYS_OK) ); i++)
    {
        /* Receive a message from the GPP */
        status = MSGQ_get(info->localMsgq,(MSGQ_Msg*) &msgS, SYS_FOREVER);
        if (status == SYS_OK)
        {
            /* Check if the message is an asynchronous error message */
            if (MSGQ_getMsgId((MSGQ_Msg) msgS) == MSGQ_ASYNCERRORMSGID)
            {
#if !defined (LOG_COMPONENT)
                LOG_printf(&trace, "Transport error Type = %d",((MSGQ_AsyncErrorMsg *) msgS)->errorType);
#endif
                /* Must free the message */
                MSGQ_free((MSGQ_Msg) msgS);
                status = SYS_EBADIO;
                SET_FAILURE_REASON(status);
            }
            /* Check if the message received has the correct sequence number */
            else if (MSGQ_getMsgId ((MSGQ_Msg) msgS) != info->sequenceNumber)
            {
#if !defined (LOG_COMPONENT)
                LOG_printf(&trace, "Out of sequence message!");
#endif
                MSGQ_free((MSGQ_Msg) msgS);
                status = SYS_EBADIO;
                SET_FAILURE_REASON(status);
            }
            else
            {

		/* Include your control flag or processing code here */

                /* Increment the sequenceNumber for next received message */
                info->sequenceNumber++;
                /* Make sure that sequenceNumber stays within the range of iterations */
                if (info->sequenceNumber == MSGQ_INTERNALIDSSTART)
                    info->sequenceNumber = 0;
                MSGQ_setMsgId((MSGQ_Msg) msgS, info->sequenceNumber);
                MSGQ_setSrcQueue((MSGQ_Msg) msgS, info->localMsgq);

                if (i == 0) 
                {
                    /* Store first matrix */
                    for (j=0; j<matrixSize; j++)
                        for (k=0; k<matrixSize; k++)
                            mat1[j][k] = matrixPtS[j][k];
                    msgS->command = 0x02;

                    /* Send the ACK back to the GPP */
                    status = MSGQ_put(info->locatedMsgq,(MSGQ_Msg) msgS);
                    if (status != SYS_OK)
                        SET_FAILURE_REASON(status);
                }
                else
                {
                    /* Store second matrix */
                    for (j=0; j < matrixSize; j++)
                        for (k=0; k < matrixSize; k++)
                            mat2[j][k] = matrixPtS[j][k];
                    msgS->command = 0x02;

                    /* Free the message and allocate a new one */
                    MSGQ_free((MSGQ_Msg) msgS);
                    status = MSGQ_alloc(SAMPLE_POOL_ID, (MSGQ_Msg*) &msgL, APP_BUFFER_SIZE);
                    matrixPtL = msgL->arg2;

                    if (status == SYS_OK) {
                        MSGQ_setMsgId((MSGQ_Msg) msgL, info->sequenceNumber);
                        MSGQ_setSrcQueue((MSGQ_Msg) msgL, info->localMsgq);

                        /* Compute the product and time the computation */
                        TSCL = 0;
                        start = TSCL;
                        for (j = 0; (j < matrixSize) && (j < MAXSIZE/2); j++)
                            for (k = 0; k < matrixSize; k++)
                            {
                                matrixPtL[j][k] = 0;
                                for(l = 0; l < matrixSize; l++)
                                    matrixPtL[j][k] = matrixPtL[j][k] + 
                                                      ((Uint32)mat1[j][l]) * ((Uint32)mat2[l][k]);
                            }
                        stop = TSCL;
                        total += stop - start;
                        start = TSCL;

                        /* Send the first half of the result back to the GPP */
                        status = MSGQ_put(info->locatedMsgq,(MSGQ_Msg) msgL);
                        if (status != SYS_OK)
                            SET_FAILURE_REASON(status);

                        status = MSGQ_get(info->localMsgq,(MSGQ_Msg*) &msgL, SYS_FOREVER);
                        MSGQ_setMsgId((MSGQ_Msg) msgL, info->sequenceNumber);
                        MSGQ_setSrcQueue((MSGQ_Msg) msgL, info->localMsgq);

                        for (j = MAXSIZE/2; j < matrixSize; j++)
                            for (k = 0; k < matrixSize; k++)
                            {
                                matrixPtL[j-MAXSIZE/2][k] = 0;
                                for(l = 0; l < matrixSize; l++)
                                    matrixPtL[j-MAXSIZE/2][k] = matrixPtL[j-MAXSIZE/2][k] + 
                                                                ((Uint32)mat1[j][l]) * ((Uint32)mat2[l][k]);
                            }
                        stop = TSCL;
                        total += stop - start;
                        msgS->arg1 = total;

                        /* Send the second half of the result back to the GPP */
                        status = MSGQ_put(info->locatedMsgq,(MSGQ_Msg) msgL);
                        if (status != SYS_OK)
                            SET_FAILURE_REASON(status);
                    }        
                }

            }
        }
        else
        {
            SET_FAILURE_REASON (status);
        }
    }
    return status;
}


/** ============================================================================
 *  @func   TSKMESSAGE_delete
 *
 *  @desc   Delete phase function for the TSKMESSAGE application. It deallocates
 *          all the resources of allocated during create phase of the
 *          application.
 *
 *  @modif  None.
 *  ============================================================================
 */
Int TSKMESSAGE_delete(TSKMESSAGE_TransferInfo* info)
{
    Int status = SYS_OK;
    Int tmpStatus = SYS_OK;
    Bool freeStatus = FALSE;

    /* Release the located message queue */
    if (info->locatedMsgq != MSGQ_INVALIDMSGQ)
    {
        status = MSGQ_release(info->locatedMsgq);
        if (status != SYS_OK)
        {
            SET_FAILURE_REASON(status);
        }
    }

     /* Reset the error handler before deleting the MSGQ that receives */
     /* the error messages.                                            */
    MSGQ_setErrorHandler(MSGQ_INVALIDMSGQ, POOL_INVALIDID);

    /* Delete the message queue */
    if (info->localMsgq != MSGQ_INVALIDMSGQ)
    {
        tmpStatus = MSGQ_close(info->localMsgq);
        if ((status == SYS_OK) && (tmpStatus != SYS_OK))
        {
            status = tmpStatus;
            SET_FAILURE_REASON(status);
        }
    }

    /* Free the info structure */
    freeStatus = MEM_free(DSPLINK_SEGID, info, sizeof(TSKMESSAGE_TransferInfo));
    if ((status == SYS_OK) && (freeStatus != TRUE))
    {
        status = SYS_EFREE;
        SET_FAILURE_REASON(status);
    }
    return status;
}


#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */
