# Installation

## Ubuntu 12.04

    git clone https://github.com/OpenKinect/libfreenect2.git /tmp/libfreenect2
    
    sudo apt-get install -y build-essential libturbojpeg libtool autoconf libudev-dev cmake mesa-common-dev freeglut3-dev libxrandr-dev doxygen libxi-dev libopencv-dev xorg-dev libglu1-mesa-dev

    cd /tmp/libfreenect2/depends
    sh install_ubuntu.sh

Copy the libusb-1.0 libraries

    sudo cp /tmp/libfreenect2/depends/libusb/lib/lib* /usr/local/lib

    cd ../examples/protonect/
    cmake CMakeLists.txt
    make && sudo make install
