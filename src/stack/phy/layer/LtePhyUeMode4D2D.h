//
//                           SimuLTE
//
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself,
// and cannot be removed from it.
//

#ifndef _LTE_AIRPHYUED2D_H_
#define _LTE_AIRPHYUED2D_H_

#include "stack/phy/layer/LtePhyUe.h"
#include "stack/phy/packet/SidelinkControlInformation_m.h"
#include "stack/mac/packet/LteMode4SchedulingGrant.h"
#include "stack/mac/allocator/LteAllocationModule.h"
#include "stack/phy/layer/Subchannel.h"

class LtePhyUeMode4D2D : public LtePhyUe
{
  protected:

    // D2D Tx Power
    double d2dTxPower_;

    bool adjacencyPSCCHPSSCH_;
    int pStep_;

    std::vector<LteAirFrame*> tbFrames_; // airframes received in the current TTI. Only one will be decoded
    cMessage* d2dDecodingTimer_; // timer for triggering decoding at the end of the TTI. Started when the first airframe is received

    std::vector<std::vector> tbRsrpVectors_;
    std::vector<std::vector> tbRssiVectors_;

    int numSubchannels_;
    int subchannelSize_ ;
    std::list<std::list<Subchannel>> sensingWindow_;
    LteMode4SchedulingGrant* sciGrant_;
    std::vector<std::vector> sciRsrpVectors_;
    std::vector<std::vector> sciRssiVectors_;
    std::vector<LteAirFrame*> sciFrames_;
    std::vector<cPacket*> decodedScis_;

    LteAllocationModule* allocator_;

    void storeAirFrame(LteAirFrame* newFrame);
    LteAirFrame* extractAirFrame();
    void decodeAirFrame(LteAirFrame* frame, UserControlInfo* lteInfo);
    // ---------------------------------------------------------------- //

    virtual void initialize(int stage);
    virtual void finish();
    virtual void handleAirFrame(cMessage* msg);
    virtual void handleUpperMessage(cMessage* msg);
    virtual void handleSelfMessage(cMessage *msg);

    // Helper function which prepares a frame for sending
    virtual LteAirFrame* prepareAirFrame(cPacket* msg, UserControlInfo* lteInfo);

    // Generate an SCI message corresponding to a Grant
    virtual SidelinkControlInformation* reateSCIMessage(cPacket* message);

    // Compute Candidate Single Subframe Resources which the MAC layer can use for transmission
    virtual void computeCSRs(SchedulingGrant* grant);

    virtual void triggerHandover();
    virtual void doHandover();

  public:
    LtePhyUeMode4D2D();
    virtual ~LtePhyUeMode4D2D();

    virtual void sendFeedback(LteFeedbackDoubleVector fbDl, LteFeedbackDoubleVector fbUl, FeedbackRequest req);
    virtual double getTxPwr(Direction dir = UNKNOWN_DIRECTION)
    {
        if (dir == D2D)
            return d2dTxPower_;
        return txPower_;
    }
};

#endif  /* _LTE_AIRPHYUED2D_H_ */
