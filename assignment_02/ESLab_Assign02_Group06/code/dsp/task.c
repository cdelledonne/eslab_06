/* ------------------------------------ DSP/BIOS Headers -------------------- */
#include <std.h>
#include <gbl.h>
#include <log.h>
#include <swi.h>
#include <sys.h>
#include <tsk.h>
#include <pool.h>

/* ------------------------------------ DSP/BIOS LINK Headers --------------- */
#include <failure.h>
#include <dsplink.h>
#include <platform.h>
#include <notify.h>
#include <bcache.h>

/* ------------------------------------ Application Headers ----------------- */
#include <pool_notify_config.h>
#include <task.h>
#include <IQmath.h>
#include <ti/dsplib/dsplib.h>

/* ------------------------------------ Notification codes ------------------ */
#define NOTIF_PDF_MODEL         0x10000000
#define NOTIF_PDF_CANDIDATE     0x20000000
#define NOTIF_BGR_PLANE         0x40000000
#define PLANE_MASK              0x00000003 // Extract BGR plane number from notification


#define NUMBER_OF_PLANES        3
#define MATRIX_WIDTH            58
#define MATRIX_HEIGHT           24
#define BUFFER_SIZE             (MATRIX_WIDTH * MATRIX_HEIGHT * 4)


/* Target model and target candidate Q16 fixed-point matrices */
_iq16 pdf_model[NUMBER_OF_PLANES][16];
_iq16 pdf_candidate[NUMBER_OF_PLANES][16];

/* Flag for Task_execute routine */
Uint8 DSP_needed;

/* Notification payload */
Uint32 notif_payload;

/* Buffer pointers */
Uint32* unsignedBuffer;
float*  floatBuffer;


extern Uint16 MPCSXFER_BufferSize ;

static Void Task_notify (Uint32 eventNo, Ptr arg, Ptr info) ;

Int Task_create (Task_TransferInfo ** infoPtr)
{
    Int status    = SYS_OK ;
    Task_TransferInfo * info = NULL ;

    /* Allocate Task_TransferInfo structure that will be initialized
     * and passed to other phases of the application */
    if (status == SYS_OK) 
	{
        *infoPtr = MEM_calloc (DSPLINK_SEGID,
                               sizeof (Task_TransferInfo),
                               0) ; /* No alignment restriction */
        if (*infoPtr == NULL) 
		{
            status = SYS_EALLOC ;
        }
        else 
		{
            info = *infoPtr ;
        }
    }

    /* Fill up the transfer info structure */
    if (status == SYS_OK) 
	{
        info->dataBuf       = NULL ; /* Set through notification callback. */
        info->bufferSize    = MPCSXFER_BufferSize ;
        SEM_new (&(info->notifySemObj), 0) ;
    }

    /*
     *  Register notification for the event callback to get control and data
     *  buffer pointers from the GPP-side.
     */
    if (status == SYS_OK) 
	{
        status = NOTIFY_register (ID_GPP,
                                  MPCSXFER_IPS_ID,
                                  MPCSXFER_IPS_EVENTNO,
                                  (FnNotifyCbck) Task_notify,
                                  info) ;
        if (status != SYS_OK) 
		{
            return status;
        }
    }

    /*
     *  Send notification to the GPP-side that the application has completed its
     *  setup and is ready for further execution.
     */
    if (status == SYS_OK) 
	{
        status = NOTIFY_notify (ID_GPP,
                                MPCSXFER_IPS_ID,
                                MPCSXFER_IPS_EVENTNO,
                                (Uint32) 0) ; /* No payload to be sent. */
        if (status != SYS_OK) 
		{
            return status;
        }
    }

    /*
     *  Wait for the event callback from the GPP-side to post the semaphore
     *  indicating receipt of the data buffer pointer.
     */
    SEM_pend (&(info->notifySemObj), SYS_FOREVER);

    return status ;
}


Int Task_execute (Task_TransferInfo * info)
{
    Uint8 i, j;

    /**
     * Old version: the pdf matrix elements would have different weight depending
     * on the current pixel position (greater weights towards the centre of
     * the ellipse computed in MeanShift::Epanechnikov_kernel).
     *
     * Current version: maximum weight is assigned to one single pixel in the
     * centre of the rectangle (degenerate case of smaller ellipse).
     *
     * pdf_hiprob_index contains the column indices of the pdf_model (or candidate)
     * elements to be assigned maximum probability.
     */
    Uint8 pdf_hiprob_index[NUMBER_OF_PLANES];

    Uint32 curr_pixel, bin_value, row_start;

    Uint32 plane = 0;

    _iq16 div;

    /* Convert 1 to fixed point. */
    _iq16 pdf_hiprob_value = _IQ16(1);

    DSP_needed = 1;

    /* Initialise pdf matrices with smallest possible fixed-point. */
    for (i=0; i<NUMBER_OF_PLANES; i++) {
        for (j=0; j<16; j++) {
            pdf_model[i][j] = 1;
            pdf_candidate[i][j] = 1;
        }
    }

    /**
     * Main DSP routine
     *
     * Notification payload may contain indices for the pdf matrices or
     * nothing useful in case a BGR plane is received.
     *
     * In both cases the MSB of the payload indicates the content of
     * the buffer, specifically:
     *     0x10000000 -> payload contains indices for the pdf_model matrix
     *     0x20000000 -> payload contains indices for the pdf_candidate matrix
     *     0x40000000 -> buffer contains bgr_plane[0]
     *     0x40000001 -> buffer contains bgr_plane[1]
     *     0x40000002 -> buffer contains bgr_plane[2]
     *     else (0)   -> DSP not needed any longer
     */
    while (DSP_needed) {

        /* Wait for notification from GPP. */
        SEM_pend (&(info->notifySemObj), SYS_FOREVER);

        if (notif_payload & NOTIF_PDF_MODEL) {
            
            /* Target model indices in the payload, update matrix. */

            pdf_hiprob_index[0] = (Uint8)notif_payload;         // take first LSB
            pdf_hiprob_index[1] = (Uint8)(notif_payload >> 8);  // take second LSB
            pdf_hiprob_index[2] = (Uint8)(notif_payload >> 16); // take third LSB

            pdf_model[0][pdf_hiprob_index[0]] += pdf_hiprob_value;
            pdf_model[1][pdf_hiprob_index[1]] += pdf_hiprob_value;
            pdf_model[2][pdf_hiprob_index[2]] += pdf_hiprob_value;
        }

        else if (notif_payload & NOTIF_PDF_CANDIDATE) {
            
            /* Target candidate indices in the payload, update matrix. */

            pdf_candidate[0][pdf_hiprob_index[0]] = 1;
            pdf_candidate[1][pdf_hiprob_index[1]] = 1;
            pdf_candidate[2][pdf_hiprob_index[2]] = 1;

            pdf_hiprob_index[0] = (Uint8)notif_payload;         // take first LSB
            pdf_hiprob_index[1] = (Uint8)(notif_payload >> 8);  // take second LSB
            pdf_hiprob_index[2] = (Uint8)(notif_payload >> 16); // take third LSB

            pdf_candidate[0][pdf_hiprob_index[0]] += pdf_hiprob_value;
            pdf_candidate[1][pdf_hiprob_index[1]] += pdf_hiprob_value;
            pdf_candidate[2][pdf_hiprob_index[2]] += pdf_hiprob_value;
        }

        else if (notif_payload & NOTIF_BGR_PLANE) {

            /* BGR plane received, compute weights. */

            plane = notif_payload & PLANE_MASK;

            /* Invalidate cache. */
            BCACHE_inv((Ptr)unsignedBuffer, BUFFER_SIZE, TRUE);

            /* Compute weights and convert to floating-point before writing. */
            row_start = 0;
            for (i=0; i<MATRIX_HEIGHT; i++) {
                for (j=0; j<MATRIX_WIDTH; j++) {
                    curr_pixel = unsignedBuffer[row_start + j];
                    bin_value = curr_pixel >> 4;
                    div = _IQ16div(pdf_model[plane][bin_value], pdf_candidate[plane][bin_value]);
                    floatBuffer[row_start + j] = _IQ16toF(div);
                }
                row_start += MATRIX_WIDTH;
            }

            /* Write back on the buffer. */
            BCACHE_wb((Ptr)floatBuffer, BUFFER_SIZE, TRUE);

            /* Notify the GPP. */
            NOTIFY_notify(ID_GPP, MPCSXFER_IPS_ID, MPCSXFER_IPS_EVENTNO, (Uint32)notif_payload);
        }

        else {

            /* DSP not needed any longer. */
            DSP_needed = 0;
        }
    }

    return SYS_OK;
}

Int Task_delete (Task_TransferInfo * info)
{
    Int    status     = SYS_OK ;
    /*
     *  Unregister notification for the event callback used to get control and
     *  data buffer pointers from the GPP-side.
     */
    status = NOTIFY_unregister (ID_GPP,
                                MPCSXFER_IPS_ID,
                                MPCSXFER_IPS_EVENTNO,
                                (FnNotifyCbck) Task_notify,
                                info);

    /* Free the info structure */
    MEM_free (DSPLINK_SEGID,
              info,
              sizeof (Task_TransferInfo)) ;
    info = NULL ;

    return status ;
}


static Void Task_notify (Uint32 eventNo, Ptr arg, Ptr info)
{
    static int count = 0;
    Task_TransferInfo * mpcsInfo = (Task_TransferInfo *) arg;

    /* To avoid compiler warning. */
    (Void) eventNo;

    /**
     * First notification received contains the address of the pool buffer.
     * Assign the address to the two pointers.
     *
     * Following notifications contain information to be used in the
     * Task_execute DSP routine.
     */

    if (count == 0) {
        unsignedBuffer = (Uint32*)info; // for reading from the buffer
        floatBuffer = (float*)info;     // for writing on the buffer
    }
    else {
        notif_payload = (Uint32)info;
    }
    count++;

    SEM_post(&(mpcsInfo->notifySemObj));
}
