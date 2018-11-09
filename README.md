# ESP32 RMT RF

Simple example of reading RF codes from a 433MHz receiver with the ESP32 RMT peripheral.

Currently this only supports protocol 1 from [rc-switch](https://github.com/sui77/rc-switch),
used by the PT2262-type receivers.

On startup the peripheral throws `rmt: RMT[0] ERR rmt: status: 0x13000040`.
Reading around the ESP32 forums, this seems to be a ringbuffer overflow.
It doesn't seem to be a problem as the RMT continues to receive normally.

