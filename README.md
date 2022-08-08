# TODO

- Handle modifier keys
- Put everything on timers so that it doesn't run more often than necessary
- Why does `lsusb` freeze for so long before returning the successful descriptors?
  Possibly related: what are we supposed to do with DEVICE_QUALIFIER GET_DESCRIPTOR requests?
- Do time-based debouncing (if necessary...)
- Handle suspend/reset properly
- Test on many OS'es
- Get USB bootloading working

# Useful Commands

- To get a printout of HID report descriptors and interrupt responses: `sudo usbhid-dump --entity=all`.