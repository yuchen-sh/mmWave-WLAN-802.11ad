## Introduction:
This is a repository for the development of the 60 GHz mmWave wireless LAN (IEEE 802.11ad/ay) in network simulator ns-3. This repository is based on [Wigig Tools](https://github.com/wigig-tools/wigig-module).

## Features:
In addition to IEEE 802.11ad/ay features as in original Wigig module, this version supports the following new features:

1. Cuboid-based obstacle model (including both furniture-type obstacles and human obstacles).
1. Accurate Line-of-Sight determination function.
1. Different mmWave-specific channel models.
1. Add TGad evaluation examples.
1. Support for multiple access point / reflector configurations (under development and continue to update).



## Building the Project:
The current implementation is based on ns-3.31. Please type the following command when building the project:

    ./waf configure --disable-examples --disable-tests --disable-python --enable-modules='applications','core','internet','point-to-point','wifi','flow-monitor','spectrum'
    ./waf build

Or, to build the project in optimized mode for fast execution type the following command:

    ./waf configure --disable-examples --disable-tests --disable-python --enable-modules='applications','core','internet','point-to-point','wifi','flow-monitor','spectrum' --enable-static -d optimized
    ./waf build
    

## Prerequisites:
Before start using this mmWave WLAN module, please keep the following in mind:

1. Understand WLAN IEEE 802.11 MAC/PHY operations.
1. Get familiar with ns-3 and how to run simulations in ns-3.
1. Understand the existing Wifi Model in ns-3 which implements WLAN IEEE 802.11a/b/g/n/ac/ax.

## Contact info:
yuchen.liu.sn at gmail dot com

## Acknowledgement
1. 1. North Carolina State University, USA
1. IMDEA Networks Institute, Madrid, Spain
1. Georgia Institute of Technology, Atlanta, USA
1. University of Washington, Seattle, USA
