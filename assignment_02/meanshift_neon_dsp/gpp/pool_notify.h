#if !defined (pool_notify_H)
#define pool_notify_H

#include <semaphore.h>
/* ------------------------------------ DSP/BIOS Link ----------------------- */
#include <dsplink.h>

/* ------------------------------------ DSP/BIOS LINK API ------------------- */
#include <proc.h>
#include <pool.h>
#include <mpcs.h>
#include <notify.h>
#if defined (DA8XXGEM)
#include <loaderdefs.h>
#endif


#if defined (__cplusplus)
extern "C" {
#endif /* defined (__cplusplus) */


/** ============================================================================
 *  @const  ID_PROCESSOR
 *
 *  @desc   The processor id of the processor being used.
 *  ============================================================================
 */
#define ID_PROCESSOR       0


/* ------------------------------------ Notification codes ------------------ */
#define NOTIF_PDF_MODEL         0x10000000
#define NOTIF_PDF_CANDIDATE     0x20000000
#define NOTIF_BGR_PLANE         0x40000000
#define PLANE_MASK              0x00000003


/** ============================================================================
 *  @func   pool_notify_Create
 *
 *  @desc   This function allocates and initializes resources used by
 *          this application.
 *
 *  @arg    dspExecutable
 *              DSP executable name.
 *  @arg    strBufferSize
 *              String representation of buffer size to be used
 *              for data transfer.
 *  @arg    strNumIterations
 *              Number of iterations a data buffer is transferred between
 *              GPP and DSP in string format.
 *  @arg    processorId
 *             Id of the DSP Processor. 
 *
 *  @ret    DSP_SOK
 *              Operation successfully completed.
 *          DSP_EFAIL
 *              Resource allocation failed.
 *
 *  @enter  None
 *
 *  @leave  None
 *
 *  @see    pool_notify_Delete
 *  ============================================================================
 */
NORMAL_API
DSP_STATUS
pool_notify_Create(IN Char8 * dspExecutable, IN Char8 * strBufferSize, IN Uint8 processorId);


/** ============================================================================
 *  @func   pool_notify_Execute
 *
 *  @desc   This function implements the execute phase for this application.
 *
 *  @arg    numIterations
 *              Number of times to send the message to the DSP.
 *  @arg    processorId
 *             Id of the DSP Processor. 
 *
 *  @ret    DSP_SOK
 *              Operation successfully completed.
 *          DSP_EFAIL
 *              MESSAGE execution failed.
 *
 *  @enter  None
 *
 *  @leave  None
 *
 *  @see    pool_notify_Delete , pool_notify_Create
 *  ============================================================================
 */
NORMAL_API
DSP_STATUS
pool_notify_Execute(IN Uint8 processorId);


/** ============================================================================
 *  @func   pool_notify_Delete
 *
 *  @desc   This function releases resources allocated earlier by call to
 *          pool_notify_Create ().
 *          During cleanup, the allocated resources are being freed
 *          unconditionally. Actual applications may require stricter check
 *          against return values for robustness.
 *
 *  @arg    processorId
 *             Id of the DSP Processor. 
 *
 *  @ret    DSP_SOK
 *              Operation successfully completed.
 *          DSP_EFAIL
 *              Resource deallocation failed.
 *
 *  @enter  None
 *
 *  @leave  None
 *
 *  @see    pool_notify_Create
 *  ============================================================================
 */
NORMAL_API
Void
pool_notify_Delete(IN Uint8 processorId);


/** ============================================================================
 *  @func   pool_notify_Main
 *
 *  @desc   The OS independent driver function for the MESSAGE sample
 *          application.
 *
 *  @arg    dspExecutable
 *              Name of the DSP executable file.
 *  @arg    strBufferSize
 *              Buffer size to be used for data-transfer in string format.
 *  @arg    strNumIterations
 *              Number of iterations a data buffer is transferred between
 *              GPP and DSP in string format.
 *  @arg    strProcessorId
 *             ID of the DSP Processor in string format. 
 *
 *  @ret    None
 *
 *  @enter  None
 *
 *  @leave  None
 *
 *  @see    pool_notify_Create, pool_notify_Execute, pool_notify_Delete
 *  ============================================================================
 */
NORMAL_API
Void
pool_notify_Init(IN Char8 * dspExecutable, IN Char8 * strBufferSize);


Uint32* get_pool_buffer_address(void);
Void wait_for_DSP(void);
Void notify_DSP(Uint32 notif_payload);
Void write_buffer(uint32_t size);


#endif /* !defined (pool_notify_H) */


#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */
