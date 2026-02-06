Arduino build & UF2 instructions for QT Py (SAMD21)

Prerequisites
- arduino-cli installed and on PATH: https://arduino.github.io/arduino-cli/latest/installation/
- Install Adafruit SAMD core: `arduino-cli core update-index && arduino-cli core install adafruit:samd`
- uf2conv.py available on PATH (see https://github.com/adafruit/uf2-samdx1 or https://github.com/Microsoft/uf2)

Build & convert (from repo root or the arduino folder):

    ./build_qtpy_samd21.sh

You can override the FQBN and bootloader offset with environment variables:

    FQBN=adafruit:samd:qtpy_m0 BOOTLOADER_OFFSET=0x2000 ./build_qtpy_samd21.sh

Flash:
1. Double-tap the QT Py to enter the UF2 bootloader (board appears as USB mass storage)
2. Copy the generated `.uf2` file to the board's USB drive

Notes
- If the FQBN is different for your installed package, run `arduino-cli board listall | grep -i qtpy` to find the correct FQBN. The default used in the script is `adafruit:samd:qtpy_m0` which matches the Adafruit board package in most setups.
- If you prefer, you can also flash directly via bossac (requires different flow). This script focuses on generating a `.uf2` you can copy to the device.
