# Transceiver Arduino Sketches

These sketches must include files from `include/`, but that's impossible with the Arduino IDE. We hardlink the each file in `include/` manually.

If you add more files to `include/`, do something like this to hardlink them:

```bash
ln include/rov_common.hpp surface_transceiver/rov_common.hpp
ln include/rov_common.hpp float_transceiver/rov_common.hpp
```