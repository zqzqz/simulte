//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef STACK_MAC_LAYER_LTEMACUEMODE4D2D_H_
#define STACK_MAC_LAYER_LTEMACUEMODE4D2D_H_

#include "stack/mac/layer/LteMacUeRealisticD2D.h"

//class LteMode4SchedulingGrant;

class LteMacUeMode4D2D: public LteMacUeRealisticD2D {

protected:

   // RAC Handling variables
   bool racD2DMulticastRequested_;
   // Multicast D2D BSR handling
   bool bsrD2DMulticastTriggered_;

   // if true, use the preconfigured TX params for transmission, else use that signaled by the eNB
   bool usePreconfiguredTxParams_;
   UserTxParams* preconfiguredTxParams_;
   UserTxParams* getPreconfiguredTxParams();  // build and return new user tx params

    /**
     * Generate a scheduling grant
     */
    virtual void macGenerateSchedulingGrant();


    /**
     * Handles the SPS candidate resources message from the PHY layer.
     */
    virtual void macHandleSps(cPacket* pkt);

    /**
     * Reads MAC parameters for ue and performs initialization.
     */
    virtual void initialize(int stage);

    /**
     * Analyze gate of incoming packet
     * and call proper handler
     */
    virtual void handleMessage(cMessage *msg);

    /**
     * Main loop
     */
    virtual void handleSelfMessage();

    virtual void macHandleGrant(cPacket* pkt);

    /*
     * Checks RAC status
     */
    // We aren't going to do this as it's primarily used for asking the eNodeb for access
    // which we don't do.
//    virtual void checkRAC();
    /*
     * Receives and handles RAC responses
     */

    void macHandleD2DModeSwitch(cPacket* pkt);

    // I shouldn't need the makeBsr function as it is something that mode4 doesn't require.
    virtual LteMacPdu* makeBsr(int size);

    /**
     * macPduMake() creates MAC PDUs (one for each CID)
     * by extracting SDUs from Real Mac Buffers according
     * to the Schedule List.
     * It sends them to H-ARQ (at the moment lower layer)
     *
     * On UE it also adds a BSR control element to the MAC PDU
     * containing the size of its buffer (for that CID)
     */
    virtual void macPduMake();

public:
    LteMacUeMode4D2D();
    virtual ~LteMacUeMode4D2D();

    virtual bool isD2DCapable()
    {
        return true;
    }

    virtual void triggerBsr(MacCid cid)
    {
        if (connDesc_[cid].getDirection() == D2D_MULTI)
            bsrD2DMulticastTriggered_ = true;
        else
            bsrTriggered_ = true;
    }
    virtual void doHandover(MacNodeId targetEnb);
};

#endif /* STACK_MAC_LAYER_LTEMACUEMODE4D2D_H_ */
