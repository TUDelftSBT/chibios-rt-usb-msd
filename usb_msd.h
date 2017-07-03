/**
 * @brief   USB mass storage driver and functions
 * @file    usb_msd.h
 */

#ifndef _USB_MSD_H_
#define _USB_MSD_H_

#include "ch.h"
#include "hal.h"

#define PACK_STRUCT_FIELD(x) x __attribute__((packed))
#define PACK_STRUCT_STRUCT __attribute__((packed))
#define PACK_STRUCT_BEGIN
#define PACK_STRUCT_END

/**
 * @brief Command Block Wrapper structure
 */
typedef struct {
	uint32_t signature;
	uint32_t tag;
	uint32_t data_len;
	uint8_t flags;
	uint8_t lun;
	uint8_t scsi_cmd_len;
	uint8_t scsi_cmd_data[16];
} PACK_STRUCT_STRUCT msd_cbw_t;

/**
 * @brief Command Status Wrapper structure
 */
typedef struct {
	uint32_t signature;
	uint32_t tag;
	uint32_t data_residue;
	uint8_t status;
} PACK_STRUCT_STRUCT msd_csw_t;

/**
 * @brief Structure holding sense data (status/error information)
 */
typedef struct {
		uint8_t byte[18];
} PACK_STRUCT_STRUCT msd_scsi_sense_response_t PACK_STRUCT_END;

/**
 * @brief structure holding the data to reply to an INQUIRY SCSI command
 */
PACK_STRUCT_BEGIN typedef struct
{
    uint8_t peripheral;
    uint8_t removable;
    uint8_t version;
    uint8_t response_data_format;
    uint8_t additional_length;
    uint8_t sccstp;
    uint8_t bqueetc;
    uint8_t cmdque;
    uint8_t vendor_id[8];
    uint8_t product_id[16];
    uint8_t product_rev[4];
} PACK_STRUCT_STRUCT msd_scsi_inquiry_response_t PACK_STRUCT_END;

/**
 * @brief Possible states for the USB mass storage driver
 */
typedef enum {
    MSD_IDLE = 0x01,
    MSD_READ_COMMAND_BLOCK = 0x10,
    MSD_RESET = 0xff
} msd_state_t;

/**
 * @brief Driver configuration structure
 */
typedef struct {
    /**
    * @brief USB driver to use for communication
    */
    USBDriver *usbp;

    /**
       * @brief   Control interface number
       * @note    Only used in composite USB device.
       *          Default is 0 which is good for single device.
       */
    uint8_t control_interface;

    /**
     * @brief   Bulk out endpoint number
     */
    uint8_t bulk_out_ep;

    /**
     * @brief   Bulk in endpoint number
     */
    uint8_t bulk_in_ep;

    /**
    * @brief Optional callback that will be called whenever there is
    *        read/write activity
    * @note  The callback is called with argument TRUE when activity starts,
    *        and FALSE when activity stops.
    */
    void (*rw_activity_callback)(bool_t);

    /**
    * @brief Short vendor identification
    * @note  ASCII characters only, maximum 8 characters (pad with zeroes).
    */
    uint8_t short_vendor_id[8];

    /**
    * @brief Short product identification
    * @note  ASCII characters only, maximum 16 characters (pad with zeroes).
    */
    uint8_t short_product_id[16];

    /**
    * @brief Short product revision
    * @note  ASCII characters only, maximum 4 characters (pad with zeroes).
    */
    uint8_t short_product_version[4];

} USBMassStorageConfig;

/**
 * @brief   USB mass storage driver structure.
 * @details This structure holds all the states and members of a USB mass
 *          storage driver.
 */

typedef struct USBMassStorageDriver USBMassStorageDriver;

struct USBMassStorageDriver {
    const USBMassStorageConfig* config;
    binary_semaphore_t bsem_received;
	binary_semaphore_t bsem_transmitted;
    thread_t* thread;
    event_source_t evt_ejected;

    /**
    * @brief Block device to use for storage
    */
    BaseBlockDevice *bbdp;
    bool eject_requested;

    BlockDeviceInfo block_dev_info;
    msd_state_t state;
    msd_cbw_t cbw;
    msd_csw_t csw;
    msd_scsi_sense_response_t sense;
    msd_scsi_inquiry_response_t inquiry;
    /* Linked list to the next serial USB driver */
    USBMassStorageDriver *driver_next;
};

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Initializes a USB mass storage driver.
 */
void msdObjectInit(USBMassStorageDriver *msdp);

/**
 * @brief   Starts a USB mass storage driver.
 * @details This function is sufficient to have USB mass storage running, it internally
 *          runs a thread that handles USB requests and transfers.
 */
void msdStart(USBMassStorageDriver *msdp, const USBMassStorageConfig *config);

/**
 * @brief   Stops a USB mass storage driver.
 * @details This function waits for current tasks to be finished, if any, and then
 *          stops the mass storage thread.
 */
void msdStop(USBMassStorageDriver *msdp);

/**
 * @brief   Set the device as ready, notify the host that media is inserted
 * @details The block device must be connected before calling.
 */
void msdReady(USBMassStorageDriver *msdp, BaseBlockDevice *bbdp);

/**
 * @brief   Request to eject the media
 * @details If transfer is in progress, eject will be done gracefully.
 *          The ejected event will be notified.
 */
void msdEject(USBMassStorageDriver *msdp);

/**
 * @brief   USB device configured handler.
 * @details This should be called when USB_EVENT_CONFIGURED occurs
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @iclass
 */
void msdConfigureHookI(USBDriver *usbp);

/**
 * @brief   USB device suspend handler.
 * @details This should be called when USB_EVENT_SUSPEND occurs
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @iclass
 */
void msdSuspendHookI(USBDriver *usbp);

/**
 * @brief   Default requests hook.
 * @details Applications wanting to use the Mass Storage over USB driver can use
 *          this function as requests hook in the USB configuration.
 *          The following requests are emulated:
 *          - MSD_REQ_RESET.
 *          - MSD_GET_MAX_LUN.
 *
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @return              The hook status.
 * @retval TRUE         Message handled internally.
 * @retval FALSE        Message not handled.
 */
bool msdRequestsHook(USBDriver *usbp);

/**
 * @brief   USB endpoint data handler.
 * @details Hook the Bulk-IN and Bulk-OUT endpoint callback to here
 */
void msdUsbEventIn(USBDriver *usbp, usbep_t ep);
void msdUsbEventOut(USBDriver *usbp, usbep_t ep);

static int counter_tmp_OUT;
int GetCounterOut();
static int counter_tmp_IN;
int GetCounterIn();

#ifdef __cplusplus
}
#endif

#endif /* _USB_MSD_H_ */
