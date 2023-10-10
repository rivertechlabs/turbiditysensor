# :cyclone:turbiditysensor
## A note on versions:

**This sensor is prototype 2.0 (or "first version"). The newest prototype 3.0 (or "version Ötz-T") is found in this [repo](https://github.com/rivertechlabs/suspendedsedimentsensor).**

## News!

**This sensor was published in Scientific Reports! Read an [Open-source, low-cost, in-situ turbidity sensor for river network monitoring here](https://www.nature.com/articles/s41598-022-14228-4.epdf?sharing_token=cyUjQXGA580BT96PZ3YG99RgN0jAjWel9jnR3ZoTv0OX8suukScp-om3npRI-9yHRu_IZlkYQE1BVdYvvL0vDxBGDfr8sI7F2l-Z1-pe-oVAVnkSKqac9C03GKGG5Jwydy4lDJZvCDPdijmWYlu6RFAzK-AAFhSjDVT-taU_3Ic%3D).**

-----------------------------------------
## Introduction
This repo is dedicated to the open-source turbidity sensor. More info can be found here:
- https://rivertechlabs.org/
- https://hyd.ifu.ethz.ch/research/riparian/susp-sed-sense.html

This repo contains several items to build the current prototype of the open-source turbidity sensor including:
- The code needed to upload to the ESP32
- CAD files
- Hookup schematics

## Code
Before you begin, you need the ESP-IDF installed on your computer. Please visit the [Espressif website](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/) for more information on installation.

The current code takes 100 frequency measurements and enters deep-sleep for 1min, this can of course be changed. Also, the code isn't beautiful and I apologize but it will get better with time.

## Hookup schematics
<img src="Images/schematic.png" width="700">

## Contact
For questions, please email me at droujko@ifu.baug.ethz.ch or @rivertechjess on Twitter :bird:

-----------------------------------------
## Prototype 1.0
This repo also contains the CAD files for prototype 1.0 of the turbidity sensor (you can use this to 3D print the device in a school project)
