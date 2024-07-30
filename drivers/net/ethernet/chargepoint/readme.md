compile and install DTS overlay:

    sudo dtc -@ -Hepapr -I dts -O dtb -o /boot/firmware/overlays/eth-spi.dtbo eth-spi-overlay.dts

compile loadable module:

    ./make.sh
