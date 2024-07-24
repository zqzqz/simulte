// 
//                           SimuLTE
// 
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself, 
// and cannot be removed from it.
//
package lte.simulations.streetcred;

import lte.world.radio.LteChannelControl;
import lte.corenetwork.binder.LteBinder;
import lte.corenetwork.deployer.LteDeployer;
import lte.corenetwork.nodes.cars.CarNonIp;

import org.car2x.veins.subprojects.veins_inet.VeinsInetManager;
import org.car2x.veins.visualizer.roads.RoadsCanvasVisualizer;

network Intersection
{
    parameters:
        double playgroundSizeX @unit(m); // x size of the area the nodes are in (in meters)
        double playgroundSizeY @unit(m); // y size of the area the nodes are in (in meters)
        double playgroundSizeZ @unit(m); // z size of the area the nodes are in (in meters)
		//# default (1000.1000)
		//# beijing (2000,2000)
		//# paris (1100,1300)
        @display("bgb=2000,2000");

    submodules:

        //# Veins manager module
        veinsManager: VeinsInetManager {
            @display("p=50,200;is=s");
        }
        roadsCanvasVisualizer: RoadsCanvasVisualizer {
            @display("p=300,0");
        }
        
        //# LTE modules
        channelControl: LteChannelControl {
            @display("p=50,25;is=s");
        }
        binder: LteBinder {
            @display("p=50,140;is=s");
        }
        deployer: LteDeployer {
            @display("p=50,259;is=s");
        }
        
        //# Network instances
		rsu[1]: CarNonIp {
            @display("p=225,225;i=veins/sign/yellowdiamond;is=vs");
        }
}

