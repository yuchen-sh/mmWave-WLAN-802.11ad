## Introduction:
This is a repository for the development of the WLAN IEEE 802.11ad/ay standards in network simulator ns-3. Both 11ad and 11ay standards support wireless networking in the unlicensed 60 GHz band. Our implementation paves the way to perform high fidelity simulations for large dense wireless networks consisting of devices with heterogeneous capabilities and constraints.

## IEEE 802.11ay Features:
We are happy to share the first pre-release of our IEEE 802.11ay module in network simulator ns-3. We list here some of the new features:

1. IEEE 802.11ay PHY frame structure, new MAC frame formats, and new Information Elements.
1. Advanced beamforming techniques (EDMG BRP PPDU and short SSW frame).
1. Channel bonding up-to four channels.
1. Channel transmit masks for all channel configurations in IEEE 802.11ay.
1. SU-MIMO beamforming training and channel access procedures.
1. MU-MIMO beamforming training procedure.
1. MIMO Q-D channel interface.
1. A comprehensive set of examples for MIMO communication and 11ay throughput validation.
1. Re-based the current implementation to ns-3.31.

## IEEE 802.11ad Features:
1. DMG Channel Access Periods (BTI/A-BFT/ATI/DTI with both CBAP and Service Periods).
1. Beamforming Training (BT) in both BHI and DTI access periods.
1. DMG PLCP Model for 802.11ad frame transmission and reception.
1. Abstract DMG PHY layer implementation for DMG CTRL/SC/OFDM.
1. Beamforming Codebook design for beamforming training and beam steering.
1. Fast Session Transfer (FST) mechanism.
1. Dynamic Allocation of Service Period (Polling).
1. Service Period Allocation.
1. DMG Relay Support (Full Duplex and Half Duplex Modes).
1. Beamformed Link Maintenance for service Period Allocation.
1. Decentralized clustering support.
1. Spatial sharing and interference assessment mechanism. 
1. Quasi-deterministic channel model to simulate real propagation environments.
1. Codebook Generator Application in MATLAB to generate beams codebook.
1. Multi-antenna beamforming training support.
1. Accurate BER vs. SNR lookup tables for all DMG PHY layers.
1. Beam refinement for both transmit and receive beam patterns.
1. Multi-AP support without the need for decentralized clustering.
1. A comprehensive set of examples and tutorials for each feature.

## Complementary WLAN modeling features:
1. Cuboid-based obstacle model (including both furniture-type obstacle and humam obstacle modeling).
1. Accurate Line-of-Sight determination function.
1. Modified channel model for indoor WLAN, where the path loss model is extended to include multipath, shadowing, and penertration loss components.
1. Sparse cluster-based channel model, which characterizes the multipath components (MPCs) arriving in clusters.
1. Support for multiple access point and mobile access point configurations.
1. Add specular reflection path determination function, support for strategical reflector placement. (under development)

## Limitation:
Below is a list of the limitation in the code:
1. SU-MIMO is limited to two devices at the moment in the network.
1. MU-MIMO data communication is not implemented yet.
1. Data communication in SP allocation does not support A-MPDU aggregation and only A-MSDU aggregation. The reason is A-MPDU aggregation requires establishing a Block Acknowledgement agreement between the initiator and the responder, which requires a bi-directional transmission in the SP allocation. However, communication in SP is only allowed in a uni-directional way. To solve the previous problem, Reverse Directional Protocol is required.
1. Not all the features have been tested in complex scenarios. In case, you run into a bug, feel free to report it.

## Prerequisites:
Before start using our 11ad/ay module, please keep the following in mind:

1. Understand WLAN IEEE 802.11 MAC/PHY operations. There are plenty of references on the Internet describing the CSMA/CA protocol and the evolution of the 802.11 protocol.
1. Get familiar with ns-3 and how to run simulations in ns-3. Have a look at the tutorial page of the simulator.
1. Understand the existing Wifi Model in ns-3 which implements WLAN IEEE 802.11a/b/g/n/ac/ax.
1. Finally, do not contact us asking how to use the model or provide you with some documentation on how to use ns-3. We will ignore your email :)

Once you have completed all these steps, you can proceed with building the project.

## Building the Project:
The current implementation is based on ns3-31. To build the project in debug mode, type the following command:

    ./waf configure --disable-examples --disable-tests --disable-python --enable-modules='core','applications','wifi','spectrum','flow-monitor','point-to-point','buildings'
    ./waf build

To build the project in optimized mode for fast execution type the following command:

    ./waf configure --disable-examples --disable-tests --disable-python --enable-modules='applications','core','internet','point-to-point','wifi','flow-monitor','spectrum' --enable-static -d optimized
    ./waf build
    
You may need to ignore the warnings when building the project, and please type the following command:

     CXXFLAGS="-Wno-error" ./waf configure --disable-examples --disable-tests --disable-python --enable-modules='applications','core','internet','point-to-point','wifi','flow-monitor','spectrum' --enable-static -d optimized
     ./waf build

Warning: The previous command will generate large executable files.

## Tutorial Scripts:
The project includes different scripts located in the scratch folder to test the previously listed features and mechanisms. At the beginning of each script, we added a detailed description regarding the tested functionality, network topology, expected output, and usage method.


## Author Information:
The project is developed and maintained by:
1. [Hany Assasa](http://people.networks.imdea.org/~hany_assasa/) (Project leader)
1. [Nina Grosheva](https://networks.imdea.org/team/imdea-networks-team/people/nina-grosheva/)
1. Yuchen Liu (yuchen.liu.sn@gmail.com)

