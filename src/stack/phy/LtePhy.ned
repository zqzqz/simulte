// 
//                           SimuLTE
// 
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself, 
// and cannot be removed from it.
// 


package lte.stack.phy;

// 
// Interface for the LtePhy module of PHY Layer
//
moduleinterface LtePhy {
    parameters:
        @display("i=block/wrxtx");
        bool usePropagationDelay;
        xml channelModel;
        
    gates:
        input upperGateIn;
        output upperGateOut;
        input radioIn;
}

// 
// Base LtePhy module of PHY Layer
//
simple LtePhyBase like LtePhy {
    parameters:
        @display("i=block/wrxtx");
        bool usePropagationDelay = default(true);                     // Should transmission delay be simulated?
        xml channelModel;
        double ueTxPower =default(26);
        double eNodeBTxPower =default(46);
        double relayTxPower =default(20);
        double microTxPower =default(30);
        string txDirection = default("OMNI");
        double txAngle = default(0);       
        
        bool dynamicCellAssociation = default(false);
        
        // switch for handover messages handling on UEs
        bool enableHandover = default(false);
        double handoverLatency @unit(s) = default(0.05s);
               
    gates:
        input upperGateIn;       // from upper layer
        output upperGateOut;    // to upper layer
        input radioIn;           // gate to receive message via sendDirect
}

// 
// User Equipment LtePhy module of PHY Layer
//
simple LtePhyUe extends LtePhyBase {
     parameters:
         @class("LtePhyUe");
         
         @signal[servingCell];
         @statistic[servingCell](title="ID of the serving eNodeB for the UE"; unit=""; source="servingCell"; record=vector);
        
         //# CQI statistics
         @signal[averageCqiDl];
         @statistic[averageCqiDl](title="Average Cqi reported in DL"; unit="cqi"; source="averageCqiDl"; record=mean,vector);
         @signal[averageCqiUl];
         @statistic[averageCqiUl](title="Average Cqi reported in UL"; unit="cqi"; source="averageCqiUl"; record=mean,vector);
}

// 
// D2D-capable User Equipment LtePhy module of PHY Layer
//
simple LtePhyUeD2D extends LtePhyUe {
     parameters:
         @class("LtePhyUeD2D");
         double d2dTxPower =default(26);
         bool d2dMulticastCaptureEffect = default(true);
         string d2dMulticastCaptureEffectFactor = default("RSRP");  // or distance
         
         //# D2D CQI statistic
         @signal[averageCqiD2D];
         @statistic[averageCqiD2D](title="Average Cqi reported in D2D"; unit="cqi"; source="averageCqiD2D"; record=mean,vector);
}

simple LtePhyUeMode4D2D extends LtePhyUeD2D {
	parameters:   
	    @class("LtePhyUeMode4D2D");
	    
	    bool adjacencyPSCCHPSSCH = default(true);
	    int pStep = default(100); 
        int numSubchannels = default(10);
	    int subchannelSize = default(5);
	    int selectionWindowStartingSubframe = default(1);
	    
	    @signal[cbr];
 		@statistic[cbr](title="Channel Busy Ratio"; source="cbr"; record=mean,vector);
 		@signal[scisReceived];
		@statistic[scisReceived](title="Number of received SCIs"; source="scisReceived"; record=sum,vector);
		@signal[scisDecoded];
		@statistic[scisDecoded](title="Number of SCIs successfully decoded"; source="scisDecoded"; record=sum,vector);
		@signal[scisNotDecoded];
        @statistic[scisNotDecoded](title="Number of SCIs successfully decoded"; source="scisNotDecoded"; record=sum,vector);
		@signal[scisSent];
		@statistic[scisSent](title="Number of SCIs sent"; source="scisSent"; record=sum,vector);
		@signal[tbsSent];
		@statistic[tbsSent](title="Number of TBs sent"; source="tbsSent"; record=sum,vector);
		@signal[tbsReceived];
		@statistic[tbsReceived](title="Number of TBs received"; source="tbsReceived"; record=sum,vector);
		@signal[tbsDecoded];
		@statistic[tbsDecoded](title="Number of TBs Decoded"; source="tbsDecoded"; record=sum,vector);
		@signal[tbsFailedDueToNoSCI];
		@statistic[tbsFailedDueToNoSCI](title="Number of Tbs not decoded due to no SCI"; source="tbsFailedDueToNoSCI"; record=sum,vector);
		@signal[tbFailedButSCIReceived];
		@statistic[tbFailedButSCIReceived](title="Number of failed TBs with a decoded SCI"; source="tbFailedButSCIReceived"; record=sum,vector);
		@signal[tbAndSCINotReceived];
		@statistic[tbAndSCINotReceived](title="Number of failed TBs with a failed SCI"; source="tbAndSCINotReceived"; record=sum,vector);
		@signal[threshold];
		@statistic[threshold](title="Increase in RSRP needed to find csr"; source="threshold"; record=mean,vector);
}

// 
// eNodeB LtePhy module of PHY Layer
//
simple LtePhyEnb extends LtePhyBase {
    @class("LtePhyEnb");
    xml feedbackComputation;
}

// 
// eNodeB LtePhy module of PHY Layer with support for D2D
//
simple LtePhyEnbD2D extends LtePhyEnb {
    parameters:
        @class("LtePhyEnbD2D");
        bool enableD2DCqiReporting = default(false);
}

// 
// Relay LtePhy module of PHY Layer
//
simple LtePhyRelay extends LtePhyBase {
    @class("LtePhyRelay");
}


