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
#include "stack/mac/packet/LteSchedulingGrant.h"
#include "stack/mac/allocator/LteAllocationModule.h"
#include "stack/phy/layer/Subchannel.h"

class LtePhyUeMode4D2D : public LtePhyUe
{
  protected:

    // D2D Tx Power
    double d2dTxPower_;

    bool adjacencyPSCCHPSSCH_;
    int pStep_;
    int numSubchannels_;
    int subchannelSize_ ;
    int selectionWindowStartingSubframe_;

    bool transmitting_;

    std::vector<int> ThresPSSCHRSRPvector_;

    std::vector<LteAirFrame*> tbFrames_; // airframes received in the current TTI. Only one will be decoded
    cMessage* d2dDecodingTimer_; // timer for triggering decoding at the end of the TTI. Started when the first airframe is received

    std::vector<std::vector<double>> tbRsrpVectors_;
    std::vector<std::vector<double>> tbRssiVectors_;

    std::vector<std::vector<Subchannel*>> sensingWindow_;
    LteMode4SchedulingGrant* sciGrant_;
    std::vector<std::vector<double>> sciRsrpVectors_;
    std::vector<std::vector<double>> sciRssiVectors_;
    std::vector<LteAirFrame*> sciFrames_;
    std::vector<cPacket*> decodedScis_;
    std::vector<int> cbrHistory_;

    double currentCBR_;
    int cbrIndex_;

    LteAllocationModule* allocator_;

    void storeAirFrame(LteAirFrame* newFrame);
    LteAirFrame* extractAirFrame();
    void decodeAirFrame(LteAirFrame* frame, UserControlInfo* lteInfo, std::vector<double> &rsrpVector, std::vector<double> &rssiVector);
    // ---------------------------------------------------------------- //

    virtual void initialize(int stage);
    virtual void finish();
    virtual void handleAirFrame(cMessage* msg);
    virtual void handleUpperMessage(cMessage* msg);
    virtual void handleSelfMessage(cMessage *msg);

    // Helper function which prepares a frame for sending
    virtual LteAirFrame* prepareAirFrame(cMessage* msg, UserControlInfo* lteInfo);

    // Generate an SCI message corresponding to a Grant
    virtual SidelinkControlInformation* createSCIMessage();

    // Compute Candidate Single Subframe Resources which the MAC layer can use for transmission
    virtual void computeCSRs(LteMode4SchedulingGrant* &grant);

    virtual void createSubframe(simtime_t subframeTime);

    virtual void checkSensed(std::vector<std::vector<Subchannel*>> &selectionWindow, LteMode4SchedulingGrant* &grant);

    virtual void checkRSRP(std::vector<std::vector<Subchannel*>> &selectionWindow, LteMode4SchedulingGrant* &grant, int thresholdIncreaseFactor);

    virtual std::vector<std::vector<Subchannel*>> getPossibleCSRs(std::vector<std::vector<Subchannel*>> &selectionWindow, LteMode4SchedulingGrant* &grant);

    virtual std::vector<std::vector<Subchannel*>> selectBestRSSIs(std::vector<std::vector<Subchannel*>> &possibleCSRs, LteMode4SchedulingGrant* &grant, int totalPossibleCSRs);

    virtual std::tuple<int,int> decodeRivValue(SidelinkControlInformation* sci, UserControlInfo* sciInfo);

    virtual void updateCBR();

  public:
    LtePhyUeMode4D2D();
    virtual ~LtePhyUeMode4D2D();

    virtual double getTxPwr(Direction dir = UNKNOWN_DIRECTION)
    {
        if (dir == D2D)
            return d2dTxPower_;
        return txPower_;
    }
};

#endif  /* _LTE_AIRPHYUED2D_H_ */
