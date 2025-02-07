# Simulator for pic32

In this repository you will find a fork of the QEMU simulator for Microchip PIC32 processors.
The following PIC32 microcontrollers and boards are supported:

Machine selector    | Microcontroller and board
--------------------|------------------------------------
pic32mx7-explorer16 | PIC32MX7 on Microchip Explorer-16 board
pic32mx7-max32      | PIC32MX7 on chipKIT Max32 board
pic32mx7-maximite   | PIC32MX7 on Geoff's Maximite board
pic32mz-explorer16  | PIC32MZ on Microchip Explorer-16 board
pic32mz-meb2        | PIC32MZ on Microchip MEB-II board
pic32mz-wifire      | PIC32MZ on chipKIT WiFire board

For the mainstream documentation on QEMU see [README.rst](https://github.com/qemu/qemu/blob/master/README.rst)
or visit [wiki.qemu-project.org](http://wiki.qemu-project.org).

# Building

The build is performed in four steps:

 * Install dependencies
 * Configure QEMU
 * Compile
 * Install

## Install dependencies

On Linux:

    sudo apt install libpixman-1-dev libfdt-dev zlib1g-dev libglib2.0-dev libsdl1.2-dev readline-dev libssl-dev
    curl -fsSL https://pyenv.run | bash
    ~/.pyenv/bin/pyenv install 2.7

On MacOS:

    brew install pyenv
    ~/.pyenv/bin/pyenv install 2.7

## Configure QEMU

    git clone git@github.com:sergev/qemu.git
    cd qemu
    ./configure --target-list=mipsel-softmmu --python=$HOME/.pyenv/versions/2.7.18/bin/python2 \
        --disable-werror --disable-opengl --disable-libnfs

## Compile

    make

## Install

Copy the resulting binary to a directory of your choice, with name `qemu-pic32`.
For example:

    cp mipsel-softmmu/qemu-system-mipsel ~/.local/bin/qemu-pic32

# Examples

 * [Run 'Hello World' demo on Max32 board](https://github.com/sergev/qemu/wiki/Max32-Hello-World)
 * [Run RetroBSD on Max32 board](https://github.com/sergev/qemu/wiki/RetroBSD-Example)
 * [Run LiteBSD on WiFire board](https://github.com/sergev/qemu/wiki/LiteBSD-Example)
 * [Full instruction trace of 'Hello World' demo](https://github.com/sergev/qemu/wiki/Example-of-instruction-trace)
