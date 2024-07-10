# StreetCred Simulation in Real World Traffic

StreeCred simulation is built on [Veins](https://veins.car2x.org/) and [OpenCV2X](http://www.cs.ucc.ie/~bm18/cv2x) to emulate V2X scenarios based on the DSRC IEEE 802.11p and 3GPP CV2X Mode 4 standard through an extension of the SimuLTE framework. It also depends on [OMNeT++](https://omnetpp.org/) and [SUMO](https://sumo.dlr.de/docs/index.html), you can set up the working environment by following our instructions.

## Environment Setup

We highly recommend you to use the Instant Veins Virtual Machine as the easy way to prepare environment for OMNeT++, SUMO and Veins. Otherwise, you can install them on your local machine following [this tutorial](https://veins.car2x.org/tutorial/). The following guideline is based on Instant Veins VM.

Unfortunately, Instant Veins VM doesn't support ARM architecture, so it cannot run on Mac with Apple M1/M2/M3 chips.

1. Download the [Instant Veins VM image](https://veins.car2x.org/download/instant-veins-5.2-i1.ova). Import it to your preferred virtualization software like Oracle VM VirtualBox and VMware Workstation Player.
2. Generate SSH keys on VM for GitHub repo cloning.
3. Go to the "src/omnetpp" directory under "Home" directory. Run `source setenv && configure WITH_OSG=no WITH_OSGEARTH=no && make` in terminal. This command rebuild OMNeT++ to avoid its compatibility problem with OpenCV2X.
4. Go to the "workspace.omnetpp" directory under "Home" directory, which is a default workspace of OMNeT++ in Instant Veins VM.
5. Clone the repo [simulte](https://github.com/zqzqz/simulte) and [veins](https://github.com/zqzqz/veins). Switch to branch "streetcred" in both repo. Download [inet-3.6.6](https://github.com/inet-framework/inet/archive/refs/tags/v3.6.6.zip) in this directory.
6. Start OMNeT++ and SUMO server launcher(veins_launchd) via clicking their icon in "Activity" toolbar.
7. Import simulte, veins and inet-3.6.6 to OMNeT++ workspace according to the tutorial [here (Page 2)](https://www.cs.ucc.ie/cv2x/media/OpenCV2X_Documentation.pdf).

## Running StreetCred Simulation

### DSRC Simulation

1. Go to "example/veins" directory under "workspace.omnetpp/veins" directory.
2. Run `bash run_all data` command under terminal to simulate all real world maps, StreetCred schemes and RSU server CPU core numbers. Results are collected under the "data" directory.
3. You can adjust simulation case in "run_all.sh" by changing the `maps`, `schemes` and `numCpuCores` lists.

### CV2X Simulation

1. Go to "simulation/streetcred" directory under "workspace.omnetpp/simulte" directory.
2. Run `bash run_all data` command under terminal to simulate a set of real world maps, StreetCred schemes and RSU server CPU core numbers. Results are collected under the "data" directory.
3. You can adjust simulation case in "run_all.sh" by changing the `settings` lists, where each element should follow the format `map scheme numCpuCore`.
