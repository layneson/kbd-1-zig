# TODO

- Why does `lsusb` freeze for so long before returning the successful descriptors?
  Possibly related: what are we supposed to do with DEVICE_QUALIFIER GET_DESCRIPTOR requests?

# Useful Commands

- To get a printout of HID report descriptors and interrupt responses: `sudo usbhid-dump --entity=all`.