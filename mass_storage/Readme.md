Mass Storage Device
===================

This driver implements a USB mass storage device. It requires a Chibios block device (e.g mmc_spi or SDC)

Bugs & TODO:
------------

* Currently the Read/Write function is written and optimized for MMC_SPI but not the generic BaseBlockDevice.
  Achieving 200KiB/s read/write over 20MHz MMC SPI 
* Commenting and formatting.
* 

Example usage:
--------------
```c

USBMassStorageDriver UMSD1;

mmcObjectInit(&MMCD1);
mmcStart(&MMCD1, &mmccfg);
mmcConnect(&MMCD1);

msdObjectInit(&UMSD);
msdStart(&UMSD, &ums_cfg);

msdReady(&UMSD, &MMCD1);
msdEject(&UMSD);

...

static void usb_event(USBDriver *usbp, usbevent_t event) {
  switch (event) {
  case USB_EVENT_CONFIGURED:
    chSysLockFromIsr();

    // Endpoints initializations
    usbInitEndpointI(usbp, MASS_STORAGE_IN_EPADDR, &ms_ep_in_config);
    usbInitEndpointI(usbp, MASS_STORAGE_OUT_EPADDR, &ms_ep_out_config);

    msdConfigureHookI(usbp);
    chSysUnlockFromIsr();
    return;
    
  case USB_EVENT_SUSPEND:
    // USB Disconnect detection
    msdSuspendHookI(usbp); 
    return;
  }
  return;
}
```

Events:
--------------
```c
chEvtRegisterMask(&UMSD1.evt_ejected, &listener_ejected, 2);

while(TRUE) {
  if(chEvtWaitOneTimeout(1, TIME_IMMEDIATE)) {
    /* drive is now connected */
      
    /* wait until the drive is ejected */
    chEvtWaitOne(2);
      
    /* drive is now ejected. do something */
  }

  chThdSleepMilliseconds(1000);
}
```

Reference
---------
* USB Mass Storage Class Bulk-Only Transportation:
  http://www.usb.org/developers/devclass_docs/usbmassbulk_10.pdf

* SCSI Command:
  http://www.seagate.com/staticfiles/support/disc/manuals/scsi/100293068a.pdf