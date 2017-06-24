

#ifndef HAL_USE_MASS_STORAGE_USB
#define HAL_USE_MASS_STORAGE_USB 0
#endif

#if HAL_USE_MASS_STORAGE_USB || defined(__DOXYGEN__)
#include "usb_msd.h"

#include "mmc_spi.h"
/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief The list of Serial USB driver
 */
static USBMassStorageDriver *driver_head;

/* Request types */
#define MSD_REQ_RESET   0xFF
#define MSD_GET_MAX_LUN 0xFE

/* CBW/CSW block signatures */
#define MSD_CBW_SIGNATURE 0x43425355
#define MSD_CSW_SIGNATURE 0x53425355

/* Setup packet access macros */
#define MSD_SETUP_WORD(setup, index) (uint16_t)(((uint16_t)setup[index + 1] << 8) | (setup[index] & 0x00FF))
#define MSD_SETUP_VALUE(setup)       MSD_SETUP_WORD(setup, 2)
#define MSD_SETUP_INDEX(setup)       MSD_SETUP_WORD(setup, 4)
#define MSD_SETUP_LENGTH(setup)      MSD_SETUP_WORD(setup, 6)

/* Command statuses */
#define MSD_COMMAND_PASSED      0x00
#define MSD_COMMAND_FAILED      0x01
#define MSD_COMMAND_PHASE_ERROR 0x02

/* SCSI commands */
#define SCSI_CMD_TEST_UNIT_READY              0x00
#define SCSI_CMD_REQUEST_SENSE                0x03
#define SCSI_CMD_FORMAT_UNIT                  0x04
#define SCSI_CMD_INQUIRY                      0x12
#define SCSI_CMD_MODE_SENSE_6                 0x1A
#define SCSI_CMD_START_STOP_UNIT              0x1B
#define SCSI_CMD_SEND_DIAGNOSTIC              0x1D
#define SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL 0x1E
#define SCSI_CMD_READ_FORMAT_CAPACITIES       0x23
#define SCSI_CMD_READ_CAPACITY_10             0x25
#define SCSI_CMD_READ_10                      0x28
#define SCSI_CMD_WRITE_10                     0x2A
#define SCSI_CMD_VERIFY_10                    0x2F

/* SCSI sense keys */
#define SCSI_SENSE_KEY_GOOD                            0x00
#define SCSI_SENSE_KEY_RECOVERED_ERROR                 0x01
#define SCSI_SENSE_KEY_NOT_READY                       0x02
#define SCSI_SENSE_KEY_MEDIUM_ERROR                    0x03
#define SCSI_SENSE_KEY_HARDWARE_ERROR                  0x04
#define SCSI_SENSE_KEY_ILLEGAL_REQUEST                 0x05
#define SCSI_SENSE_KEY_UNIT_ATTENTION                  0x06
#define SCSI_SENSE_KEY_DATA_PROTECT                    0x07
#define SCSI_SENSE_KEY_BLANK_CHECK                     0x08
#define SCSI_SENSE_KEY_VENDOR_SPECIFIC                 0x09
#define SCSI_SENSE_KEY_COPY_ABORTED                    0x0A
#define SCSI_SENSE_KEY_ABORTED_COMMAND                 0x0B
#define SCSI_SENSE_KEY_VOLUME_OVERFLOW                 0x0D
#define SCSI_SENSE_KEY_MISCOMPARE                      0x0E

#define SCSI_ASENSE_NO_ADDITIONAL_INFORMATION          0x00
#define SCSI_ASENSE_WRITE_FAULT                        0x03
#define SCSI_ASENSE_LOGICAL_UNIT_NOT_READY             0x04
#define SCSI_ASENSE_READ_ERROR                         0x11
#define SCSI_ASENSE_INVALID_COMMAND                    0x20
#define SCSI_ASENSE_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE 0x21
#define SCSI_ASENSE_INVALID_FIELD_IN_CDB               0x24
#define SCSI_ASENSE_WRITE_PROTECTED                    0x27
#define SCSI_ASENSE_NOT_READY_TO_READY_CHANGE          0x28
#define SCSI_ASENSE_FORMAT_ERROR                       0x31
#define SCSI_ASENSE_MEDIUM_NOT_PRESENT                 0x3A

#define SCSI_ASENSEQ_NO_QUALIFIER                      0x00
#define SCSI_ASENSEQ_FORMAT_COMMAND_FAILED             0x01
#define SCSI_ASENSEQ_INITIALIZING_COMMAND_REQUIRED     0x02
#define SCSI_ASENSEQ_OPERATION_IN_PROGRESS             0x07

/**
 * @brief Response to a READ_CAPACITY_10 SCSI command
 */
PACK_STRUCT_BEGIN typedef struct {
    uint32_t last_block_addr;
    uint32_t block_size;
} PACK_STRUCT_STRUCT msd_scsi_read_capacity_10_response_t PACK_STRUCT_END;

/**
 * @brief Response to a READ_FORMAT_CAPACITIES SCSI command
 */
PACK_STRUCT_BEGIN typedef struct {
    uint8_t reserved[3];
    uint8_t capacity_list_length;
    uint32_t block_count;
    uint32_t desc_and_block_length;
} PACK_STRUCT_STRUCT msd_scsi_read_format_capacities_response_t PACK_STRUCT_END;

/**
 * @brief   Read-write buffers (TODO: need a way of specifying the size of this)
 */
static uint8_t rw_buf[2][512];

/**
 * @brief Byte-swap a 32 bits unsigned integer
 */
#define swap_uint32(x) ((((x) & 0x000000FF) << 24) \
                      | (((x) & 0x0000FF00) << 8) \
                      | (((x) & 0x00FF0000) >> 8) \
                      | (((x) & 0xFF000000) >> 24))

/**
 * @brief Byte-swap a 16 bits unsigned integer
 */
#define swap_uint16(x) ((((x) & 0x00FF) << 8) \
                      | (((x) & 0xFF00) >> 8))

/**
 * @brief   USB device configured handler.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @iclass
 */
void msdConfigureHookI(USBDriver *usbp)
{
  (void) usbp;
  USBMassStorageDriver *msdp = driver_head;
  while (msdp != NULL)
  {
    msdp->state = MSD_RESET;
    chBSemSignalI(&msdp->bsem);

    msdp = msdp->driver_next;
  }
}

void msdSuspendHookI(USBDriver *usbp)
{
  (void) usbp;
  USBMassStorageDriver *msdp = driver_head;
  while (msdp != NULL)
  {
    msdp->state = MSD_RESET;
    msdp->eject_requested = true;
    chBSemSignalI(&msdp->bsem);

    msdp = msdp->driver_next;
  }

}

/**
 * @brief   Default requests hook.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @return              The hook status.
 * @retval TRUE         Message handled internally.
 * @retval FALSE        Message not handled.
 */
bool_t msdRequestsHook(USBDriver *usbp) {
    /* check that the request is of type Class / Interface */
    if (((usbp->setup[0] & USB_RTYPE_TYPE_MASK) == USB_RTYPE_TYPE_CLASS) &&
        ((usbp->setup[0] & USB_RTYPE_RECIPIENT_MASK) == USB_RTYPE_RECIPIENT_INTERFACE)) {

        USBMassStorageDriver *msdp = driver_head;
        uint8_t index = usbp->setup[4]; // TODO: wIndex is actually uint16_t in LE
        while (msdp->driver_next != NULL) {
            if (msdp->config->control_interface == index) break;
            msdp = msdp->driver_next;
        }

        /* act depending on bRequest = setup[1] */
        switch (usbp->setup[1]) {
        case MSD_REQ_RESET:
            /* check that it is a HOST2DEV request */
            if (((usbp->setup[0] & USB_RTYPE_DIR_MASK) != USB_RTYPE_DIR_HOST2DEV) ||
               (MSD_SETUP_LENGTH(usbp->setup) != 0) ||
               (MSD_SETUP_VALUE(usbp->setup) != 0))
            {
                return FALSE;
            }

            /* reset all endpoints */
            msdp->state = MSD_IDLE;
            /* The device shall NAK the status stage of the device request until
             * the Bulk-Only Mass Storage Reset is complete.
             */
            return TRUE;
        case MSD_GET_MAX_LUN:
            /* check that it is a DEV2HOST request */
            if (((usbp->setup[0] & USB_RTYPE_DIR_MASK) != USB_RTYPE_DIR_DEV2HOST) ||
               (MSD_SETUP_LENGTH(usbp->setup) != 1) ||
               (MSD_SETUP_VALUE(usbp->setup) != 0))
            {
                return FALSE;
            }

            static uint8_t len_buf[1] = {0};
            /* stall to indicate that we don't support LUN */
            usbSetupTransfer(usbp, len_buf, 1, NULL);
            return TRUE;
        default:
            return FALSE;
            break;
        }
    }

    return FALSE;
}

/**
 * @brief Wait until the end-point interrupt handler has been called
 */
static bool_t msd_wait_for_isr(USBMassStorageDriver *msdp) {
    /* sleep until it completes */
    chSysLock();
    chBSemWaitS(&msdp->bsem);
    uint8_t state = msdp->state;
    chSysUnlock();
    return state != MSD_RESET;
}

static void msd_eject(USBMassStorageDriver *msdp) {
    msdp->eject_requested = FALSE;
    msdp->bbdp = NULL;
    chEvtBroadcast(&msdp->evt_ejected);
}

/**
 * @brief Called when data can be read or written on the endpoint -- wakes the thread up
 */
void msdUsbEvent(USBDriver *usbp, usbep_t ep) {
    (void)usbp;
    (void)ep;

    USBMassStorageDriver *msdp = driver_head;
    while (msdp->driver_next != NULL) {
      if (msdp->config->bulk_in_ep == ep) break;
      if (msdp->config->bulk_out_ep == ep) break;
      msdp = msdp->driver_next;
    }

    chSysLockFromIsr();
    chBSemSignalI(&msdp->bsem);
    chSysUnlockFromIsr();
}

/**
 * @brief Starts sending data
 */
static void msd_start_transmit(USBMassStorageDriver *msdp, const uint8_t* buffer, size_t size) {
    usbPrepareTransmit(msdp->config->usbp, msdp->config->bulk_in_ep, buffer, size);
    chSysLock();
    usbStartTransmitI(msdp->config->usbp, msdp->config->bulk_in_ep);
    chSysUnlock();
}

/**
 * @brief Starts receiving data
 */
static void msd_start_receive(USBMassStorageDriver *msdp, uint8_t* buffer, size_t size) {
    usbPrepareReceive(msdp->config->usbp, msdp->config->bulk_out_ep, buffer, size);
    chSysLock();
    usbStartReceiveI(msdp->config->usbp, msdp->config->bulk_out_ep);
    chSysUnlock();
}

/**
 * @brief Changes the SCSI sense information
 */
static inline void msd_scsi_set_sense(USBMassStorageDriver *msdp, uint8_t key, uint8_t acode, uint8_t aqual) {
    msdp->sense.byte[2] = key;
    msdp->sense.byte[12] = acode;
    msdp->sense.byte[13] = aqual;
}

static bool_t msd_is_ready(USBMassStorageDriver *msdp)
{
  BaseBlockDevice* bbdp = msdp->bbdp;
  if (bbdp != NULL &&
      blkIsInserted(bbdp) &&
      blkGetDriverState(bbdp) == BLK_READY) {
      /* device inserted and ready */
      return TRUE;
  } else {
      /* device not present or not ready */
      msd_scsi_set_sense(msdp,
                         SCSI_SENSE_KEY_NOT_READY,
                         SCSI_ASENSE_MEDIUM_NOT_PRESENT,
                         SCSI_ASENSEQ_NO_QUALIFIER);

      if (msdp->bbdp != NULL)
        msd_eject(msdp);
      return FALSE;
  }
}

/**
 * @brief Processes an INQUIRY SCSI command
 */
static bool_t msd_scsi_process_inquiry(USBMassStorageDriver *msdp) {

    msd_cbw_t *cbw = &(msdp->cbw);

    /* check the EVPD bit (Vital Product Data) */
    if (cbw->scsi_cmd_data[1] & 0x01) {

        /* check the Page Code byte to know the type of product data to reply */
        switch (cbw->scsi_cmd_data[2]) {

        /* unit serial number */
        case 0x80: {
            uint8_t response[] = {'0'}; /* TODO */
            msd_start_transmit(msdp, response, sizeof(response));
            return msd_wait_for_isr(msdp);
        }

        /* unhandled */
        default:
            msd_scsi_set_sense(msdp,
                               SCSI_SENSE_KEY_ILLEGAL_REQUEST,
                               SCSI_ASENSE_INVALID_FIELD_IN_CDB,
                               SCSI_ASENSEQ_NO_QUALIFIER);
            return FALSE;
        }
    }
    else
    {
        msd_start_transmit(msdp, (const uint8_t *)&msdp->inquiry, sizeof(msdp->inquiry));
        return msd_wait_for_isr(msdp);
    }
}

/**
 * @brief Processes a REQUEST_SENSE SCSI command
 */
static bool_t msd_scsi_process_request_sense(USBMassStorageDriver *msdp) {
    msd_start_transmit(msdp, (const uint8_t *)&msdp->sense, sizeof(msdp->sense));
    return msd_wait_for_isr(msdp);
}

/**
 * @brief Processes a READ_CAPACITY_10 SCSI command
 */
static bool_t msd_scsi_process_read_capacity_10(USBMassStorageDriver *msdp) {
    if (!msd_is_ready(msdp)) return FALSE;

    static msd_scsi_read_capacity_10_response_t response;

    response.block_size = swap_uint32(msdp->block_dev_info.blk_size);
    response.last_block_addr = swap_uint32(msdp->block_dev_info.blk_num-1);

    msd_start_transmit(msdp, (const uint8_t *)&response, sizeof(response));
    return msd_wait_for_isr(msdp);
}

/**
 * @brief Processes a SEND_DIAGNOSTIC SCSI command
 */
static bool_t msd_scsi_process_send_diagnostic(USBMassStorageDriver *msdp) {

    msd_cbw_t *cbw = &(msdp->cbw);

    if (!(cbw->scsi_cmd_data[1] & (1 << 2))) {
        /* only self-test supported - update SENSE key and fail the command */
        msd_scsi_set_sense(msdp,
                           SCSI_SENSE_KEY_ILLEGAL_REQUEST,
                           SCSI_ASENSE_INVALID_FIELD_IN_CDB,
                           SCSI_ASENSEQ_NO_QUALIFIER);
        return FALSE;
    }

    /* TODO: actually perform the test */
    return TRUE;
}

static bool_t msd_do_write(USBMassStorageDriver *msdp, uint32_t start, uint16_t total)
{
  /* get the first packet */
  msd_start_receive(msdp, rw_buf[(total + 1) % 2], msdp->block_dev_info.blk_size);
  if (!msd_wait_for_isr(msdp)) return FALSE;

  if (mmcStartSequentialWrite((MMCDriver *)msdp->bbdp, start) == CH_FAILED)
    return FALSE;

  /* loop over each block */
  while (total-- > 0)
  {
    if (total > 0) {
      /* there is at least one block of data left to be read over USB */
      /* queue this read before issuing the blocking write */
      msd_start_receive(msdp, rw_buf[(total + 1) % 2], msdp->block_dev_info.blk_size);
    }

    /* now write the block to the block device */
    if (mmcSequentialWrite((MMCDriver *)msdp->bbdp, rw_buf[total % 2]) == CH_FAILED)
        return FALSE;

    if (total > 0) {
      /* now wait for the USB event to complete */
      if (!msd_wait_for_isr(msdp)) return FALSE;
    }
  }

  if (mmcStopSequentialWrite((MMCDriver *)msdp->bbdp) == CH_FAILED)
    return FALSE;

  return TRUE;
}

static bool_t msd_do_read(USBMassStorageDriver *msdp, uint32_t start, uint16_t total)
{
  uint16_t i;
  if (mmcStartSequentialRead((MMCDriver *)msdp->bbdp, start) == CH_FAILED)
    return FALSE;

  /* loop over each block */
  for (i = 0; i < total; i++)
  {
    /* now write the block to the block device */
    if (mmcSequentialRead((MMCDriver *)msdp->bbdp, rw_buf[i % 2]) == CH_FAILED) {
      if (i > 0)
        if (!msd_wait_for_isr(msdp)) return FALSE;
      return FALSE;
    }

    if (i > 0)
      if (!msd_wait_for_isr(msdp)) return FALSE;

    /* transmit the block */
    msd_start_transmit(msdp, rw_buf[i % 2], msdp->block_dev_info.blk_size);
  }

  if (!msd_wait_for_isr(msdp)) return FALSE;

  if (mmcStopSequentialRead((MMCDriver *)msdp->bbdp) == CH_FAILED)
    return FALSE;

  return TRUE;
}

/**
 * @brief Processes a READ_WRITE_10 SCSI command
 */
static bool_t msd_scsi_process_start_read_write_10(USBMassStorageDriver *msdp) {

    msd_cbw_t *cbw = &(msdp->cbw);

    if (!msd_is_ready(msdp)) return FALSE;

    if ((cbw->scsi_cmd_data[0] == SCSI_CMD_WRITE_10) && blkIsWriteProtected(msdp->bbdp)) {
        /* device is write protected and a write has been issued */
        /* block address is invalid, update SENSE key and return command fail */
        msd_scsi_set_sense(msdp,
                           SCSI_SENSE_KEY_DATA_PROTECT,
                           SCSI_ASENSE_WRITE_PROTECTED,
                           SCSI_ASENSEQ_NO_QUALIFIER);
        return FALSE;
    }

    uint32_t rw_block_address = swap_uint32(*(uint32_t *)&cbw->scsi_cmd_data[2]);
    uint16_t total = swap_uint16(*(uint16_t *)&cbw->scsi_cmd_data[7]);

    if (rw_block_address >= msdp->block_dev_info.blk_num) {
        /* block address is invalid, update SENSE key and return command fail */
        msd_scsi_set_sense(msdp,
                           SCSI_SENSE_KEY_ILLEGAL_REQUEST,
                           SCSI_ASENSE_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE,
                           SCSI_ASENSEQ_NO_QUALIFIER);
        return FALSE;
    }

    if (cbw->scsi_cmd_data[0] == SCSI_CMD_WRITE_10) {
        /* process a write command */
      if (!msd_do_write(msdp, rw_block_address, total)) {
        /* write failed */
        msd_scsi_set_sense(msdp,
                           SCSI_SENSE_KEY_MEDIUM_ERROR,
                           SCSI_ASENSE_WRITE_FAULT,
                           SCSI_ASENSEQ_NO_QUALIFIER);
        return FALSE;
      }
    } else {
      if (!msd_do_read(msdp, rw_block_address, total)) {
        /* write failed */
        msd_scsi_set_sense(msdp,
                           SCSI_SENSE_KEY_MEDIUM_ERROR,
                           SCSI_ASENSE_READ_ERROR,
                           SCSI_ASENSEQ_NO_QUALIFIER);
        return FALSE;
      }
    }

    return TRUE;
}

/**
 * @brief Processes a START_STOP_UNIT SCSI command
 */
static bool_t msd_scsi_process_start_stop_unit(USBMassStorageDriver *msdp) {

    if ((msdp->cbw.scsi_cmd_data[4] & 0x03) == 0x02) {
      /* Eject */
      msd_eject(msdp);
    }

    return TRUE;
}

/**
 * @brief Processes a MODE_SENSE_6 SCSI command
 */
static bool_t msd_scsi_process_mode_sense_6(USBMassStorageDriver *msdp) {

    static uint8_t response[4] = {
        0x03, /* number of bytes that follow                    */
        0x00, /* medium type is SBC                             */
        0x00, /* not write protected (TODO handle it correctly) */
        0x00  /* no block descriptor                            */
    };

    msd_start_transmit(msdp, response, sizeof(response));
    return msd_wait_for_isr(msdp);
}


/**
 * @brief Processes a TEST_UNIT_READY SCSI command
 */
static bool_t msd_scsi_process_test_unit_ready(USBMassStorageDriver *msdp)
{
    return msd_is_ready(msdp);
}

/**
 * @brief Waits for a new command block
 */
static void msd_wait_for_command_block(USBMassStorageDriver *msdp) {
    msd_start_receive(msdp, (uint8_t *)&msdp->cbw, sizeof(msdp->cbw));
    msdp->state = MSD_READ_COMMAND_BLOCK;

    msd_wait_for_isr(msdp);
}

/**
 * @brief Reads a newly received command block
 */
static void msd_read_command_block(USBMassStorageDriver *msdp) {

    msd_cbw_t *cbw = &(msdp->cbw);

    /* by default transition back to the idle state */
    msdp->state = MSD_IDLE;

    /* check the command */
    if ((cbw->signature != MSD_CBW_SIGNATURE) ||
        (cbw->lun > 0) ||
        ((cbw->data_len > 0) && (cbw->flags & 0x1F)) ||
        (cbw->scsi_cmd_len == 0) ||
        (cbw->scsi_cmd_len > 16)) {

        /* stall both IN and OUT endpoints */
        chSysLock();
        usbStallReceiveI(msdp->config->usbp, msdp->config->bulk_out_ep);
        usbStallTransmitI(msdp->config->usbp, msdp->config->bulk_in_ep);
        chSysUnlock();

        return;
    }

    bool_t result;

    /* check the command */
    switch (cbw->scsi_cmd_data[0]) {
    case SCSI_CMD_INQUIRY:
        result = msd_scsi_process_inquiry(msdp);
        break;
    case SCSI_CMD_REQUEST_SENSE:
        result = msd_scsi_process_request_sense(msdp);
        break;
    case SCSI_CMD_READ_CAPACITY_10:
        result = msd_scsi_process_read_capacity_10(msdp);
        break;
    case SCSI_CMD_READ_10:
    case SCSI_CMD_WRITE_10:
        if (msdp->config->rw_activity_callback)
            msdp->config->rw_activity_callback(TRUE);
        result = msd_scsi_process_start_read_write_10(msdp);
        if (msdp->config->rw_activity_callback)
            msdp->config->rw_activity_callback(FALSE);
        break;
    case SCSI_CMD_SEND_DIAGNOSTIC:
        result = msd_scsi_process_send_diagnostic(msdp);
        break;
    case SCSI_CMD_MODE_SENSE_6:
        result = msd_scsi_process_mode_sense_6(msdp);
        break;
    case SCSI_CMD_START_STOP_UNIT:
        result = msd_scsi_process_start_stop_unit(msdp);
        break;
    case SCSI_CMD_TEST_UNIT_READY:
        result = msd_scsi_process_test_unit_ready(msdp);
        break;
    case SCSI_CMD_FORMAT_UNIT:
        /* don't handle */
        result = TRUE;
        break;
    case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
        /* don't handle */
        result = TRUE;
        break;
    case SCSI_CMD_VERIFY_10:
        /* don't handle */
        result = TRUE;
        break;
    default:
        msd_scsi_set_sense(msdp,
                           SCSI_SENSE_KEY_ILLEGAL_REQUEST,
                           SCSI_ASENSE_INVALID_COMMAND,
                           SCSI_ASENSEQ_NO_QUALIFIER);

        result = FALSE;
        break;
    }

    if (msdp->state == MSD_RESET)
      return;

    if (result) {
        /* update sense with success status */
        msd_scsi_set_sense(msdp,
                           SCSI_SENSE_KEY_GOOD,
                           SCSI_ASENSE_NO_ADDITIONAL_INFORMATION,
                           SCSI_ASENSEQ_NO_QUALIFIER);

        /* reset data length left */
        cbw->data_len = 0;
    }

    msd_csw_t *csw = &(msdp->csw);

    if (!result && cbw->data_len) {
        /* still bytes left to send, this is too early to send CSW? */
        chSysLock();
        if (cbw->flags & 0x80) // Is data-in?
          // Stall TX-IN
          usbStallTransmitI(msdp->config->usbp, msdp->config->bulk_in_ep);
        else
          // Stall RX-OUT
          usbStallReceiveI(msdp->config->usbp, msdp->config->bulk_out_ep);
        chSysUnlock();
    }

    /* update the command status wrapper and send it to the host */
    csw->status = result ? MSD_COMMAND_PASSED : MSD_COMMAND_FAILED;
    csw->signature = MSD_CSW_SIGNATURE;
    csw->data_residue = cbw->data_len;
    csw->tag = cbw->tag;

    msd_start_transmit(msdp, (const uint8_t *)csw, sizeof(*csw));
    msd_wait_for_isr(msdp);
}

/**
 * @brief Mass storage thread that processes commands
 */
static WORKING_AREA(mass_storage_thread_wa, 1024);
static msg_t mass_storage_thread(void *arg) {

    USBMassStorageDriver *msdp = (USBMassStorageDriver *)arg;

    chRegSetThreadName("USB-MSD");

    while (!chThdShouldTerminate()) {
        /* wait on data depending on the current state */
        switch (msdp->state) {
        case MSD_IDLE:
            if (msdp->eject_requested)
              msd_eject(msdp);
            else
              msd_wait_for_command_block(msdp);
            break;
        case MSD_READ_COMMAND_BLOCK:
            msd_read_command_block(msdp);
            break;
        case MSD_RESET:
            if (msdp->eject_requested)
              msd_eject(msdp);
            msdp->state = MSD_IDLE;
            break;
        }
    }

    return 0;
}

/**
 * @brief Initializse a USB mass storage driver
 */
void msdObjectInit(USBMassStorageDriver *msdp) {
    chDbgCheck(msdp != NULL, "msdInit");
    USBMassStorageDriver *dp = driver_head;
    if (dp == NULL) {
      driver_head = msdp;
    } else {
      while (dp->driver_next != NULL)
        dp = dp->driver_next;
      dp->driver_next = msdp;
    }
    msdp->driver_next = NULL;

    msdp->config = NULL;
    msdp->thread = NULL;
    msdp->state = MSD_IDLE;

    /* initialize the driver events */
    chEvtInit(&msdp->evt_ejected);

    /* initialise the binary semaphore as taken */
    chBSemInit(&msdp->bsem, TRUE);

    /* initialise the sense data structure */
    size_t i;
    for (i = 0; i < sizeof(msdp->sense.byte); i++)
        msdp->sense.byte[i] = 0x00;
    msdp->sense.byte[0] = 0x70; /* response code */
    msdp->sense.byte[7] = 0x0A; /* additional sense length */

    /* initialize the inquiry data structure */
    msdp->inquiry.peripheral = 0x00;           /* direct access block device  */
    msdp->inquiry.removable = 0x80;            /* removable                   */
    msdp->inquiry.version = 0x04;              /* SPC-2                       */
    msdp->inquiry.response_data_format = 0x02; /* response data format        */
    msdp->inquiry.additional_length = 0x20;    /* response has 0x20 + 4 bytes */
    msdp->inquiry.sccstp = 0x00;
    msdp->inquiry.bqueetc = 0x00;
    msdp->inquiry.cmdque = 0x00;
}

/**
 * @brief Starts a USB mass storage driver
 */
void msdStart(USBMassStorageDriver *msdp, const USBMassStorageConfig *config) {
    chDbgCheck(msdp != NULL, "msdStart");
    chDbgCheck(config != NULL, "msdStart");
    chDbgCheck(msdp->thread == NULL, "msdStart");

    /* save the configuration */
    msdp->config = config;

    /* copy the config strings to the inquiry response structure */
    size_t i;
    for (i = 0; i < sizeof(msdp->config->short_vendor_id); ++i)
        msdp->inquiry.vendor_id[i] = config->short_vendor_id[i];
    for (i = 0; i < sizeof(msdp->config->short_product_id); ++i)
        msdp->inquiry.product_id[i] = config->short_product_id[i];
    for (i = 0; i < sizeof(msdp->config->short_product_version); ++i)
        msdp->inquiry.product_rev[i] = config->short_product_version[i];

    /* set the initial state */
    msdp->state = MSD_IDLE;

    /* run the thread */
    msdp->thread = chThdCreateStatic(mass_storage_thread_wa, sizeof(mass_storage_thread_wa), NORMALPRIO, mass_storage_thread, msdp);
}


void msdReady(USBMassStorageDriver *msdp, BaseBlockDevice *bbdp)
{
  if (msdp->bbdp != NULL)
    return;

  /* get block device information */
  blkGetInfo(bbdp, &msdp->block_dev_info);

  msdp->bbdp = bbdp;
}

/**
 * @brief   Notify the host that media is ejected
 */
void msdEject(USBMassStorageDriver *msdp)
{
  msdp->eject_requested = TRUE;
}

/**
 * @brief Stops a USB mass storage driver
 */
void msdStop(USBMassStorageDriver *msdp) {

    chDbgCheck(msdp->thread != NULL, "msdStop");

    /* notify the thread that it's over */
    chThdTerminate(msdp->thread);

    /* wake the thread up and wait until it ends */
    chSysLock();
    msdp->state = MSD_RESET;
    chBSemSignalI(&msdp->bsem);
    chSysUnlock();

    chThdWait(msdp->thread);
    msdp->thread = NULL;
}

#endif // HAL_USE_MASS_STORAGE_USB
