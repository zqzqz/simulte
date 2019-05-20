//
//                           SimuLTE
//
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself,
// and cannot be removed from it.
//
// This file is an extension of SimuLTE
// Author: Brian McCarthy
// email : b.mccarthy@cs.ucc.ie


#include <math.h>
#include <assert.h>
#include <algorithm>
#include <iterator>
#include <vector>
#include "stack/phy/layer/LtePhyVUeMode4.h"
#include "stack/phy/packet/LteFeedbackPkt.h"
#include "stack/d2dModeSelection/D2DModeSelectionBase.h"
#include "stack/phy/packet/SpsCandidateResources.h"
#include "stack/phy/packet/cbr_m.h"

Define_Module(LtePhyVUeMode4);

LtePhyVUeMode4::LtePhyVUeMode4()
{
    handoverStarter_ = NULL;
    handoverTrigger_ = NULL;
}

LtePhyVUeMode4::~LtePhyVUeMode4()
{
}

void LtePhyVUeMode4::initialize(int stage)
{
    if (stage != inet::INITSTAGE_NETWORK_LAYER_2)
        LtePhyUeD2D::initialize(stage);

    if (stage == inet::INITSTAGE_LOCAL)
    {
        adjacencyPSCCHPSSCH_ = par("adjacencyPSCCHPSSCH");
        pStep_ = par("pStep");
        selectionWindowStartingSubframe_ = par("selectionWindowStartingSubframe");
        numSubchannels_ = par("numSubchannels");
        subchannelSize_ = par("subchannelSize");
        d2dDecodingTimer_ = NULL;
        transmitting_ = false;
        currentCBR_= 0;
        cbrIndex_= -1; // Start at -1 simply to ensure that on first call to create subframe we start at index 0
        cbrHistory_.reserve(100);

        d2dTxPower_ = par("d2dTxPower");
        if (d2dTxPower_ <= 0){
            d2dTxPower_ = txPower_;
        }

        // The threshold has a size of 64, and allowable values of 0 - 66
        // Deciding on this for now as it makes the most sense (low priority for both then more likely to take it)
        // High priority for both then less likely to take it.
        for (int i = 1; i < 66; i++)
        {
            ThresPSSCHRSRPvector_.push_back(i);
        }

        cbr                    = registerSignal("cbr");
        scisReceived           = registerSignal("scisReceived");
        scisDecoded            = registerSignal("scisDecoded");
        scisNotDecoded         = registerSignal("scisNotDecoded");
        scisSent               = registerSignal("scisSent");
        tbsSent                = registerSignal("tbsSent");
        tbsReceived            = registerSignal("tbsReceived");
        tbsDecoded             = registerSignal("tbsDecoded");
        tbsFailedDueToNoSCI    = registerSignal("tbsFailedDueToNoSCI");
        tbFailedButSCIReceived = registerSignal("tbFailedButSCIReceived");
        tbAndSCINotReceived    = registerSignal("tbAndSCINotReceived");
        threshold              = registerSignal("threshold");
        txRxDistanceTB         = registerSignal("txRxDistanceTB");
        txRxDistanceSCI        = registerSignal("txRxDistanceSCI");
        sciFailedHalfDuplex    = registerSignal("sciFailedHalfDuplex");
        tbFailedHalfDuplex     = registerSignal("tbFailedHalfDuplex");

        scisReceived_ = 0;
        scisDecoded_ = 0;
        scisNotDecoded_ = 0;
        sciFailedHalfDuplex_ = 0;
        tbsReceived_ = 0;
        tbsDecoded_ = 0;
        tbsFailedDueToNoSCI_ = 0;
        tbFailedButSCIReceived_ = 0;
        tbAndSCINotReceived_ = 0;
        tbFailedHalfDuplex_ = 0;

        sensingWindowFront_ = 0; // Will ensure when we first update the sensing window we don't skip over the first element
    }
    else if (stage == INITSTAGE_NETWORK_LAYER_2)
    {
        // Need to start initialising the sensingWindow
        deployer_ = getDeployer();
        int index = intuniform(0, binder_->phyPisaData.maxChannel() - 1);
        deployer_->lambdaInit(nodeId_, index);
        deployer_->channelUpdate(nodeId_, intuniform(1, binder_->phyPisaData.maxChannel2()));

//        LteMacBase* mac = binder_->getMacFromMacNodeId(nodeId_);
        allocator_ = new LteAllocationModule(mac_, D2D);
        allocator_->initAndReset(deployer_->getNumRbUl(), deployer_->getNumBands());

        initialiseSensingWindow();
    }
}

void LtePhyVUeMode4::handleSelfMessage(cMessage *msg)
{
    if (msg->isName("d2dDecodingTimer"))
    {
        while (!sciFrames_.empty()){
            // Get received SCI and it's corresponding RsrpVector
            LteAirFrame* frame = sciFrames_.back();
            std::vector<double> rsrpVector = sciRsrpVectors_.back();
            std::vector<double> rssiVector = sciRssiVectors_.back();

            // Remove it from the vector
            sciFrames_.pop_back();
            sciRsrpVectors_.pop_back();
            sciRssiVectors_.pop_back();

            UserControlInfo* lteInfo = check_and_cast<UserControlInfo*>(frame->removeControlInfo());

            // decode the selected frame
            decodeAirFrame(frame, lteInfo, rsrpVector, rssiVector);

            emit(scisReceived, scisReceived_);
            emit(scisDecoded, scisDecoded_);
            emit(scisNotDecoded, scisNotDecoded_);
            emit(sciFailedHalfDuplex, sciFailedHalfDuplex_);

            scisReceived_ = 0;
            scisDecoded_ = 0;
            scisNotDecoded_ = 0;
            sciFailedHalfDuplex_ = 0;

            currentCBR_ = currentCBR_/numSubchannels_;
            cbrHistory_[cbrIndex_]=currentCBR_;
            currentCBR_=0;
            updateCBR();
        }
        while (!tbFrames_.empty())
        {
            LteAirFrame* frame = tbFrames_.back();
            std::vector<double> rsrpVector = tbRsrpVectors_.back();
            std::vector<double> rssiVector = tbRssiVectors_.back();

            tbFrames_.pop_back();
            tbRsrpVectors_.pop_back();
            tbRssiVectors_.pop_back();

            UserControlInfo* lteInfo = check_and_cast<UserControlInfo*>(frame->removeControlInfo());

            // decode the selected frame
            decodeAirFrame(frame, lteInfo, rsrpVector, rssiVector);

            emit(tbsReceived, tbsReceived_);
            emit(tbsDecoded, tbsDecoded_);
            emit(tbsFailedDueToNoSCI, tbsFailedDueToNoSCI_);
            emit(tbFailedButSCIReceived, tbFailedButSCIReceived_);
            emit(tbFailedHalfDuplex, tbFailedHalfDuplex_);

            tbsReceived_ = 0;
            tbsDecoded_ = 0;
            tbsFailedDueToNoSCI_ = 0;
            tbFailedButSCIReceived_ = 0;
            tbFailedHalfDuplex_ = 0;
        }
        std::vector<cPacket*>::iterator it;
        for(it=decodedScis_.begin();it!=decodedScis_.end();it++)
        {
            delete(*it);
        }
        decodedScis_.clear();
        delete msg;
        d2dDecodingTimer_ = NULL;
    }
    else if (msg->isName("updateSubframe"))
    {
        transmitting_ = false;
        cbrIndex_++;
        if (cbrIndex_ == 100)
            cbrIndex_ = 0;
        cbrHistory_[cbrIndex_] = currentCBR_;
        updateSubframe();
        delete msg;
    }
    else
        LtePhyUe::handleSelfMessage(msg);
}

// TODO: ***reorganize*** method
void LtePhyVUeMode4::handleAirFrame(cMessage* msg)
{
    UserControlInfo* lteInfo = check_and_cast<UserControlInfo*>(msg->removeControlInfo());

    connectedNodeId_ = masterId_;
    LteAirFrame* frame = check_and_cast<LteAirFrame*>(msg);
    EV << "LtePhyVUeMode4: received new LteAirFrame with ID " << frame->getId() << " from channel" << endl;
    //Update coordinates of this user
    if (lteInfo->getFrameType() == HANDOVERPKT)
    {
        // check if handover is already in process
        if (handoverTrigger_ != NULL && handoverTrigger_->isScheduled())
        {
            delete lteInfo;
            delete frame;
            return;
        }

        handoverHandler(frame, lteInfo);
        return;
    }

    // HACK: if this is a multicast connection, change the destId of the airframe so that upper layers can handle it
    // All packets in mode 4 are multicast
    lteInfo->setDestId(nodeId_);

    // send H-ARQ feedback up
    if (lteInfo->getFrameType() == HARQPKT || lteInfo->getFrameType() == GRANTPKT || lteInfo->getFrameType() == RACPKT || lteInfo->getFrameType() == D2DMODESWITCHPKT)
    {
        handleControlMsg(frame, lteInfo);
        return;
    }

    // this is a DATA packet

    // if not already started, auto-send a message to signal the presence of data to be decoded
    if (d2dDecodingTimer_ == NULL)
    {
        d2dDecodingTimer_ = new cMessage("d2dDecodingTimer");
        d2dDecodingTimer_->setSchedulingPriority(10);          // last thing to be performed in this TTI
        scheduleAt(NOW, d2dDecodingTimer_);
    }

    // store frame, together with related control info
    frame->setControlInfo(lteInfo);

    // Capture the Airframe for decoding later
    storeAirFrame(frame);
}

void LtePhyVUeMode4::handleUpperMessage(cMessage* msg)
{

    UserControlInfo* lteInfo = check_and_cast<UserControlInfo*>(msg->removeControlInfo());

    LteAirFrame* frame;

    if (lteInfo->getFrameType() == GRANTPKT)
    {
        // Generate CSRs or save the grant for use when generating SCI information
        LteMode4SchedulingGrant* grant = check_and_cast<LteMode4SchedulingGrant*>(msg);
        if (grant->getTotalGrantedBlocks() == 0){
            // Generate a vector of CSRs and send it to the MAC layer
            computeCSRs(grant);
            delete lteInfo;
            delete grant;
        }
        else
        {
            sciGrant_ = grant;
            lteInfo->setUserTxParams(sciGrant_->getUserTxParams()->dup());
            lteInfo->setGrantedBlocks(sciGrant_->getGrantedBlocks());
            lteInfo->setDirection(D2D_MULTI);
            availableRBs_ = sendSciMessage(msg, lteInfo);
            for (int i=0; i<numSubchannels_; i++)
            {
                // Mark all the subchannels as not sensed
                sensingWindow_[sensingWindowFront_][i]->setSensed(false);
            }
        }
        return;
    }
    else if (lteInfo->getFrameType() == HARQPKT)
    {
        frame = new LteAirFrame("harqFeedback-grant");
    }

    // if this is a multicast/broadcast connection, send the frame to all neighbors in the hearing range
    // otherwise, send unicast to the destination

    EV << "LtePhyVUeMode4::handleUpperMessage - " << nodeTypeToA(nodeType_) << " with id " << nodeId_
           << " sending message to the air channel. Dest=" << lteInfo->getDestId() << endl;

    // Mark that we are in the process of transmitting a packet therefore when we go to decode messages we can mark as failure due to half duplex
    transmitting_=true;

    lteInfo->setGrantedBlocks(availableRBs_);

    frame = prepareAirFrame(msg, lteInfo);

    emit(tbsSent, 1);

    if (lteInfo->getDirection() == D2D_MULTI)
        sendBroadcast(frame);
    else
        sendUnicast(frame);
}

RbMap LtePhyVUeMode4::sendSciMessage(cMessage* msg, UserControlInfo* lteInfo)
{
    // Store the RBs used for transmission. For interference computation
    RbMap rbMap = lteInfo->getGrantedBlocks();
    UsedRBs info;
    info.time_ = NOW;
    info.rbMap_ = rbMap;

    usedRbs_.push_back(info);

    std::vector<UsedRBs>::iterator it = usedRbs_.begin();
    while (it != usedRbs_.end())  // purge old allocations
    {
        if (it->time_ < NOW - 0.002)
            usedRbs_.erase(it++);
        else
            ++it;
    }
    lastActive_ = NOW;

    UserControlInfo* SCIInfo = lteInfo->dup();
    LteAirFrame* frame = NULL;

    RbMap sciRbs;
    if (adjacencyPSCCHPSSCH_)
    {
        // Setup so SCI gets 2 RBs from the grantedBlocks.
        RbMap::iterator it;
        std::map<Band, unsigned int>::iterator jt;
        //for each Remote unit used to transmit the packet
        int allocatedBlocks = 0;
        for (it = rbMap.begin(); it != rbMap.end(); ++it)
        {
            if (allocatedBlocks == 2)
            {
                break;
            }
            //for each logical band used to transmit the packet
            for (jt = it->second.begin(); jt != it->second.end(); ++jt)
            {
                Band band = jt->first;

                if (allocatedBlocks == 2)
                {
                    // Have all the blocks allocated to the SCI so can move on.
                    break;
                }

                if (jt->second == 0) // this Rb is not allocated
                    continue;
                else
                {
                    // RB is allocated to grant, now give it to the SCI.
                    if (jt->second == 1){
                        jt->second = 0;
                        sciRbs[it->first][band] = 1;
                        ++allocatedBlocks;
                    }
                }
            }
        }
    }
    else
    {
        // Setup so SCI gets 2 RBs from the grantedBlocks.
        RbMap::iterator it;
        std::map<Band, unsigned int>::iterator jt;
        int allocatedRbs = 0;
        //for each Remote unit used to transmit the packet
        for (it = rbMap.begin(); it != rbMap.end(); ++it) {
            if (allocatedRbs == 2) {
                break;
            }
            //for each logical band used to transmit the packet
            for (jt = it->second.begin(); jt != it->second.end(); ++jt) {
                if (allocatedRbs == 2) {
                    // Have all the blocks allocated to the SCI so can move on.
                    break;
                }
                // sciRbs[remote][band] = assigned Rb
                sciRbs[it->first][jt->first] = 1;
                ++allocatedRbs;
            }
        }
    }

    SCIInfo->setFrameType(SCIPKT);
    SCIInfo->setGrantedBlocks(sciRbs);

    /*
     * Need to prepare the airframe were sending
     * Ensure that it will fit into it's grant
     * if not don't send anything except a break reservation message
     */
    EV << NOW << " LtePhyVUeMode4::handleUpperMessage - message from stack" << endl;

    // create LteAirFrame and encapsulate the received packet
    SidelinkControlInformation* SCI = createSCIMessage();
    LteAirFrame* sciFrame = prepareAirFrame(SCI, SCIInfo);

    emit(scisSent, 1);
    sendBroadcast(sciFrame);

    delete sciGrant_;
    delete lteInfo;

    return (rbMap);
}

void LtePhyVUeMode4::computeCSRs(LteMode4SchedulingGrant* &grant) {
    EV << NOW << " LtePhyVUeMode4::computeCSRs - going through sensing window to compute CSRS..." << endl;
    // Determine the total number of possible CSRs
    if (grant->getMaximumLatency() > 100) {
        grant->setMaximumLatency(100);
    }

    EV << NOW
       << " LtePhyVUeMode4::computeCSRs - eliminating CSRS which were not sensed in sensing window and those above the threshold ..."
       << endl;
    int pRsvpTx = grant->getPeriod();
    unsigned int grantLength = grant->getNumSubchannels();
    int cResel = grant->getResourceReselectionCounter();
    int maxLatency = grant->getMaximumLatency();
    std::vector<double> allowedRRIs = grant->getPossibleRRIs();

    // Start and end of Selection Window.
    int minSelectionIndex = (10 * pStep_) + selectionWindowStartingSubframe_;
    int maxSelectionIndex = (10 * pStep_) + maxLatency;

    int totalPossibleCSRs = ((maxSelectionIndex - minSelectionIndex) * numSubchannels_) / grantLength;

    // If we don't have RRIs greater than 100ms then any subframe which is older than 100ms is not relevant for this part
    // of the selection process, thus we can skip a majority of the sensing window saving time.
    int maxRRI = *std::max_element(allowedRRIs.begin(), allowedRRIs.end());
    int fallBack = pStep_ * maxRRI;

    int minSubCh = (10 * pStep_) - fallBack;

    // Create a set of all the possible CSRs
    // Each SubchannelIndex being the starting index of a CSR.
    // Subframe -> {SubchannelIndex, SubchannelIndex}
    std::unordered_map<int, std::set<int>> possibleCSRs;
    for (int i = minSelectionIndex; i < maxSelectionIndex; i++) {
        std::set<int> subframe;
        for (int j = 0; j < (numSubchannels_ - grantLength); j += grantLength) {
            subframe.insert(j);
        }
        possibleCSRs[i] = subframe;
    }

    // subframes disallowed
    std::vector<int> notSensedSubframes;

    // subchannels disallowed
    std::map < int, std::unordered_map < int, std::vector < int >> > aboveThresholdDisallowedIndices;

    int disallowedCSRs = 0;

    // Number of time the threshold needs to be increased by 3dB to allow for 20% of CSRs to be selected
    int minThresholdIncreasesRequired = 0;

    int z = minSubCh;
    while (z < sensingWindow_.size()) {
        // The use of z is to correspond with the notation in the standard see 3GPP TS 36.213 14.1.1.6

        int pRsvpTxPrime = pStep_ * pRsvpTx / 100;
        int Q = 1;

        // Check if frame is sensed or not.

        int translatedZ = translateIndex(z);

        if (!sensingWindow_[translatedZ][0]->getSensed()) {
            /**
             *  Not sensed calculation
             *
             *  y + j * P'rsvpTx = z + Pstep * k * q
             *
             *  y = subframe of possible CSR
             *  j = {0, 1, ... Cresel-1}
             *  Pstep is the lenght of frames we have e.g. 100ms long frames
             *  PrsvpTx is the resource reservation interval of transmission e.g. 100
             *  P'rsvpTx = Pstep * PrsvpTx / 100
             *
             *  z = sensing window subframe index
             *  k is all possible RRIs {0.2, 0.5, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10}
             *  q = {1, 2 ... Q}
             *  n' is the current subframe index i.e. 1000.
             *  Q = 1/k if k < 1 AND n' - z <= Pstep * k.
             *
             *  Translated calc (to get all possible disallowed indices)
             *
             *  y = (z + Pstep * k * q) - (j * P'rsvpTx)
             */

            std::vector<double>::iterator k;
            for (k = allowedRRIs.begin(); k != allowedRRIs.end(); k++) {
                // This applies to all allowed RRIs as well.
                // 10 * pStep_ = n'

                if ((*k) < 1 && 10 * pStep_ - z < pStep_ * (*k)) {
                    Q = 1 / *k;
                }

                for (int q = 1; q <= Q; q++) {

                    for (int j = 1; j < cResel; j++) {
                        int disallowedSubframe = (z + pStep_ * (*k) * q) - (j * pRsvpTxPrime);
                        // Only mark as disallowed if it corresponds with a frame in the selection window
                        if (disallowedSubframe >= minSelectionIndex && disallowedSubframe <= maxSelectionIndex) {
                            notSensedSubframes.push_back(disallowedSubframe);
                            disallowedCSRs += numSubchannels_ / grantLength;
                        }
                    }
                }
            }
        } else {
            int j = 0;
            while (j < numSubchannels_) {
                /**
                 *  RSRP based calculation FIX THIS
                 *
                 *  y + j * P'rsvpRx = tSLm + q * Pstep * PrsvpRx
                 *
                 *  y = subframe of possible CSR
                 *  j = {0, 1, ... Cresel-1} (will use c in the below code for ease of coding)
                 *  Pstep is the length of frames we have e.g. 100ms long frames
                 *  PrsvRx is the resource reservation interval of the received SCI e.g. 100
                 *  P'rsvpTx = Pstep * PrsvpTx / 100
                 *
                 *  tSLm = sensing window subframe index (equivalent of z in the above calc, will use z here for ease of coding)
                 *  q = {1, 2 ... Q}
                 *  n' is the current subframe index i.e. 1000.
                 *  Q = 1/PrsvpTx if PrsvpTx < 1 AND n' - z <= Pstep * PrsvpTx.
                 *
                 *  Translated calc (to get all possible disallowed indices)
                 *
                 *  y = (z + q * pStep_ * PrsvpRx) - (j * P'rsvpTx);
                 */

                // It's possible that a grant might span multiple subchannels, if this is the case then check all for
                // An SCI and record the information for each independently for the later calculation
                std::vector<double> averageRSRPs;
                std::vector<int> priorities;
                std::vector < SidelinkControlInformation * > scis;

                // If an SCI reserves subchannels spanning a selection then use this to avoid double counting it.
                // i.e. SCI reserves subchannels 2 & 3, if we check 1 & 2 and it is above the threshold then we count it
                // as disallowed, but when we move to check subchannel 3 the same will happen and we will count it again
                // this is incorrect. Instead move to the end of the grant to avoid this i.e. never check 3.
                bool overReachingGrant = false;

                bool subchannelReserved = false;

                if (j + grantLength > numSubchannels_) {
                    // We cannot fill this grant in this subframe and should move to the next one
                    break;
                }

                int k = j;
                while (k < j + grantLength) {
                    if (sensingWindow_[translatedZ][k]->getReserved()) {
                        subchannelReserved = true;

                        // Get the SCI and all the necessary information
                        SidelinkControlInformation *receivedSCI = check_and_cast<SidelinkControlInformation *>(
                                sensingWindow_[translatedZ][k]->getSCIMessage());
                        UserControlInfo *sciInfo = check_and_cast<UserControlInfo *>(receivedSCI->getControlInfo());
                        priorities.push_back(receivedSCI->getPriority());
                        scis.push_back(receivedSCI);

                        std::tuple<int, int> indexAndLength = decodeRivValue(receivedSCI, sciInfo);
                        int lengthInSubchannels = std::get<1>(indexAndLength);

                        int totalRSRP = 0;
                        for (int l = k; l < k + lengthInSubchannels; l++) {
                            totalRSRP += sensingWindow_[translatedZ][l]->getAverageRSRP();
                        }
                        averageRSRPs.push_back(totalRSRP / lengthInSubchannels);

                        k += lengthInSubchannels;
                        if (k > j + grantLength) {
                            overReachingGrant = true;
                        }
                    } else {
                        k++;
                    }
                }

                if (subchannelReserved) {
                    subchannelReserved = false;

                    int highestThreshold;
                    unsigned int pRsvpRx;

                    // Get the priorities of both messages
                    int messagePriority = grant->getSpsPriority();
                    for (int l = 0; l < averageRSRPs.size(); l++) {
                        double averageRSRP = averageRSRPs[l];
                        int receivedPriority = priorities[l];
                        SidelinkControlInformation *receivedSCI = scis[l];

                        // Get the threshold for the corresponding priorities
                        int index = messagePriority * 8 + receivedPriority + 1;
                        int threshold = ThresPSSCHRSRPvector_[index];
                        int thresholdDbm = (-128 * (threshold - 1) * 2);

                        if (averageRSRP > thresholdDbm) {
                            // Must determine the number of increases required to make this a CSR.
                            int thresholdIncreaseFactor = 1;
                            while (averageRSRP > thresholdDbm) {
                                thresholdDbm = thresholdDbm + (3 * thresholdIncreaseFactor);
                                ++thresholdIncreaseFactor;
                            }
                            if (!highestThreshold || thresholdIncreaseFactor > highestThreshold) {
                                highestThreshold = thresholdIncreaseFactor;
                                pRsvpRx = receivedSCI->getResourceReservationInterval();
                            }
                        }
                    }

                    if (highestThreshold) {
                        // This series of subchannels is to be excluded
                        int Q = 1;
                        if (pRsvpRx < 1 && z <= (pStep_ * 10) - pStep_ * pRsvpRx) {
                            Q = 1 / pRsvpRx;
                        }
                    }

                    for (int q = 1; q <= Q; q++) {
                        // j replaced with c in this case as would disrupt above use of j
                        for (int c = 0; c < cResel; c++) {
                            // Based on above calc comment
                            int disallowedIndex = (z + q * pStep_ * pRsvpRx) - (c * pRsvpTxPrime);

                            // Only mark as disallowed if it corresponds with a frame in the selection window
                            if (disallowedIndex >= minSelectionIndex && disallowedIndex <= maxSelectionIndex) {
                                aboveThresholdDisallowedIndices[highestThreshold][disallowedIndex].push_back(j);
                                ++disallowedCSRs;
                            }
                        }
                    }
                }
                if (overReachingGrant) {
                    j = k;
                } else {
                    j += grantLength;
                }
            }
        }
        z++;
    }


    // If too many CSRs are reserved need to reclaim some
    if (disallowedCSRs > totalPossibleCSRs * .8)
    {
        std::map<int, std::unordered_map<int, std::vector<int>>>::const_iterator it;
        for (it = aboveThresholdDisallowedIndices.begin(); it != aboveThresholdDisallowedIndices.end(); it++) {
            int disallowedAtThisIncrease = 0;
            for (int j = minSelectionIndex; j < maxSelectionIndex; j++) {

                std::unordered_map<int, std::vector<int>>::const_iterator got = it->second.find(j);

                if (got != it->second.end()) {
                    disallowedAtThisIncrease += got->second.size();
                }
            }
            // Remove CSRs counted at this increase.
            disallowedCSRs -= disallowedAtThisIncrease;

            // If we go below the 80% disallowed CSRs then mark it, these will have to be added back into possibleCSRs
            if (disallowedCSRs < totalPossibleCSRs * .8) {
                // Found the minimum increases to have enough CSRs.
                minThresholdIncreasesRequired = it->first;
                break;
            }
        }
    }

    // Need to remove all the not sensed subframes first
    for (int i=0; i<notSensedSubframes.size(); i++)
    {
        int subframeIndex = notSensedSubframes[i];
        // Simply erase this element as an option.
        possibleCSRs.erase(subframeIndex);
    }

    // Now need to go through all the threshold breaking CSRs and remove them
    std::map<int, std::unordered_map<int, std::vector<int>>>::const_iterator it;
    for (it = aboveThresholdDisallowedIndices.begin(); it != aboveThresholdDisallowedIndices.end(); it++) {

        // Ignore those that we have to keep due to increased thresholds
        if (it->first > minThresholdIncreasesRequired)
        {
            // Go through each subframe in this threshold
            std::unordered_map<int, std::vector<int>>::const_iterator jt;
            for (jt=it->second.begin(); jt!=it->second.end(); jt++) {

                // Go through each subchannel in this threshold
                std::vector<int>::const_iterator kt;
                for (kt=jt->second.begin(); kt!=jt->second.end(); kt++){
                    // Erase the subchannel
                    possibleCSRs[jt->first].erase(*kt);

                    if (possibleCSRs[jt->first].size() == 0){
                        // If the subframe is now empty then erase it also.
                        possibleCSRs.erase(jt->first);
                    }
                }
            }
        }
    }

    /*
     * Using RSSI pick subchannels with lowest RSSI (Across time) pick 20% lowest.
     * report this to MAC layer.
     */
    std::vector<std::tuple<int, int, int>> optimalCSRs;

    optimalCSRs = selectBestRSSIs(possibleCSRs, grant, totalPossibleCSRs);

    // Send the packet up to the MAC layer where it will choose the CSR and the retransmission if that is specified
    // Need to generate the message that is to be sent to the upper layers.
    SpsCandidateResources* candidateResourcesMessage = new SpsCandidateResources("CSRs");
    candidateResourcesMessage->setCSRs(optimalCSRs);
    send(candidateResourcesMessage, upperGateOut_);

    // Send self message to trigger another subframes creation and insertion. Need one for every TTI
    cMessage* deleteSelectionWindow = new cMessage("deleteSelectionWindow");
    deleteSelectionWindow->setSchedulingPriority(0);        // Generate the subframe at start of next TTI
    scheduleAt(NOW + TTI, deleteSelectionWindow);
}

std::vector<std::tuple<int, int, int>> LtePhyVUeMode4::selectBestRSSIs(std::unordered_map<int, std::set<int>> possibleCSRs, LteMode4SchedulingGrant* &grant, int totalPossibleCSRs)
{
    EV << NOW << " LtePhyVUeMode4::selectBestRSSIs - Selecting best CSRs from possible CSRs..." << endl;
    int decrease = pStep_;
    if (grant->getPeriod() < 100)
    {
        // Same as pPrimeRsvpTx from other parts of the function
        decrease = (pStep_ * grant->getPeriod())/100;
    }

    int maxLatency = grant->getMaximumLatency();

    // Start and end of Selection Window.
    int minSelectionIndex = (10 * pStep_) + selectionWindowStartingSubframe_;
    int maxSelectionIndex = (10 * pStep_) + maxLatency;

    unsigned int grantLength = grant->getNumSubchannels();

    // This will be avgRSSI -> (subframeIndex, subchannelIndex)
    std::vector<std::tuple<int, int, int>> orderedCSRs;
    std::unordered_map<int, std::set<int>>::iterator it;

    for (it=possibleCSRs.begin(); it!=possibleCSRs.end(); it++)
    {
        int subframe = it->first;
        int sensingSubframeIndex = subframe;
        std::set<int>::iterator jt;
        for (jt=it->second.begin(); jt!=it->second.end(); jt++)
        {
            int initialSubchannelIndex = *jt;
            int finalSubchannelIndex = *jt + grantLength;

            while (sensingSubframeIndex > (10 * pStep_)){
                // decrease the subframe index until we are within the sensing window.
                sensingSubframeIndex -= decrease;
            }

            int totalRSSI = 0;
            int numSubchannels = 0;
            while (sensingSubframeIndex > 0)
            {
                int translatedSubframeIndex = translateIndex(sensingSubframeIndex);
                for (int subchannelCounter = initialSubchannelIndex; subchannelCounter <= finalSubchannelIndex; subchannelCounter++)
                {
                    if (sensingWindow_[translatedSubframeIndex][subchannelCounter]->getSensed())
                    {
                        totalRSSI += sensingWindow_[translatedSubframeIndex][subchannelCounter]->getAverageRSSI();
                        ++numSubchannels;
                    }
                    else
                    {
                        break;
                    }
                }
                sensingSubframeIndex -= decrease;
            }
            int averageRSSI = 0;
            if (numSubchannels != 0)
            {
                // Can be the case when the sensing window is not full that we don't find the historic CSRs
                averageRSSI = totalRSSI / numSubchannels;
                orderedCSRs.push_back(std::make_tuple(averageRSSI, subframe, initialSubchannelIndex));
            }
        }
    }
    int minSize = std::round(totalPossibleCSRs * .2);
    orderedCSRs.resize(minSize);

    return orderedCSRs;
}

SidelinkControlInformation* LtePhyVUeMode4::createSCIMessage()
{
    EV << NOW << " LtePhyVUeMode4::createSCIMessage - Start creating SCI..." << endl;

    SidelinkControlInformation* sci = new SidelinkControlInformation("SCI Message");

    /*
     * Priority (based on upper layer)
     * 0-7
     * Mapping unknown, so possibly just take the priority max and min and map to 0-7
     * This needs to be integrated from the application layer.
     * Will take this from the scheduling grant.
     */
    sci->setPriority(sciGrant_->getSpsPriority());

    /* Resource Interval
     *
     * 0 -> 16
     * 0 = not reserved
     * 1 = 100ms (1) RRI [Default]
     * 2 = 200ms (2) RRI
     * ...
     * 10 = 1000ms (10) RRI
     * 11 = 50ms (0.5) RRI
     * 12 = 20ms (0.2) RRI
     * 13 - 15 = Reserved
     *
     */
    if (sciGrant_->getExpiration() != 0)
    {
        sci->setResourceReservationInterval(sciGrant_->getPeriod()/100);
    }
    else
    {
        sci->setResourceReservationInterval(0);
    }
    /* frequency Resource Location
     * Based on another parameter RIV
     * but size is
     * Log2(Nsubchannel(Nsubchannel+1)/2) (rounded up)
     * 0 - 8 bits
     * 0 - 256 different values
     *
     * Based on TS36.213 14.1.1.4C
     *     if SubchannelLength -1 < numSubchannels/2
     *         RIV = numSubchannels(SubchannelLength-1) + subchannelIndex
     *     else
     *         RIV = numSubchannels(numSubchannels-SubchannelLength+1) + (numSubchannels-1-subchannelIndex)
     */
    unsigned int riv;
    //
    if (sciGrant_->getNumSubchannels() -1 <= (numSubchannels_/2))
    {
        // RIV calculation for less than half+1
        riv = ((numSubchannels_ * (sciGrant_->getNumSubchannels() - 1)) + sciGrant_->getStartingSubchannel());
    }
    else
    {
        // RIV calculation for less than half size
        riv = ((numSubchannels_ * (numSubchannels_ - sciGrant_->getNumSubchannels() + 1)) + (numSubchannels_ - 1 - sciGrant_->getStartingSubchannel()));
    }

    sci->setFrequencyResourceLocation(riv);

    /* TimeGapRetrans
     * 1 - 15
     * ms from init to retrans
     */
    sci->setTimeGapRetrans(sciGrant_->getTimeGapTransRetrans());


    /* mcs
     * 5 bits
     * 26 combos
     * Technically the MAC layer determines the MCS that it wants the message sent with and as such it will be in the packet
     */
    sci->setMcs(sciGrant_->getMcs());

    /* retransmissionIndex
     * 0 / 1
     * if 0 retrans is in future/this is the first transmission
     * if 1 retrans is in the past/this is the retrans
     */
    if (sciGrant_->getRetransmission())
    {
        sci->setRetransmissionIndex(1);
    }
    else
    {
        sci->setRetransmissionIndex(0);
    }

    /* Filler up to 32 bits
     * Can just set the length to 32 bits and ignore the issue of adding filler
     */
    sci->setBitLength(32);

    return sci;
}

LteAirFrame* LtePhyVUeMode4::prepareAirFrame(cMessage* msg, UserControlInfo* lteInfo){
    // Helper function to prepare airframe for sending.
    LteAirFrame* frame = new LteAirFrame("airframe");

    frame->encapsulate(check_and_cast<cPacket*>(msg));
    frame->setSchedulingPriority(airFramePriority_);
    frame->setDuration(TTI);

    lteInfo->setCoord(getRadioPosition());

    lteInfo->setTxPower(txPower_);
    lteInfo->setD2dTxPower(d2dTxPower_);
    frame->setControlInfo(lteInfo);

    return frame;
}

void LtePhyVUeMode4::storeAirFrame(LteAirFrame* newFrame)
{
    // implements the capture effect
    // store the frame received from the nearest transmitter
    UserControlInfo* newInfo = check_and_cast<UserControlInfo*>(newFrame->getControlInfo());
    Coord myCoord = getCoord();

    std::vector<double> rsrpVector = channelModel_->getRSRP_D2D(newFrame, newInfo, nodeId_, myCoord);
    // Seems we don't really actually need the enbId, I have set it to 0 as it is referenced but never used for calc
    std::vector<double> rssiVector = channelModel_->getRSSI(newFrame, newInfo, nodeId_, myCoord, 0, rsrpVector);

    // Need to be able to figure out which subchannel is associated to the Rbs in this case
    if (newInfo->getFrameType() == SCIPKT){
        sciFrames_.push_back(newFrame);
        sciRsrpVectors_.push_back(rsrpVector);
        sciRssiVectors_.push_back(rssiVector);
    }
    else{
        tbFrames_.push_back(newFrame);
        tbRsrpVectors_.push_back(rsrpVector);
        tbRssiVectors_.push_back(rssiVector);
    }
}

void LtePhyVUeMode4::decodeAirFrame(LteAirFrame* frame, UserControlInfo* lteInfo, std::vector<double> &rsrpVector, std::vector<double> &rssiVector)
{
    EV << NOW << " LtePhyVUeMode4::decodeAirFrame - Start decoding..." << endl;

    // apply decider to received packet
    bool result = false;

    RemoteSet r = lteInfo->getUserTxParams()->readAntennaSet();
    if (r.size() > 1)
    {
        // DAS
        for (RemoteSet::iterator it = r.begin(); it != r.end(); it++)
        {
            EV << "LtePhyVUeMode4::decodeAirFrame: Receiving Packet from antenna " << (*it) << "\n";

            /*
             * On UE set the sender position
             * and tx power to the sender das antenna
             */

            // cc->updateHostPosition(myHostRef,das_->getAntennaCoord(*it));
            // Set position of sender
            // Move m;
            // m.setStart(das_->getAntennaCoord(*it));
            RemoteUnitPhyData data;
            data.txPower=lteInfo->getTxPower();
            data.m=getRadioPosition();
            frame->addRemoteUnitPhyDataVector(data);
        }
        // apply analog models For DAS
        result=channelModel_->errorDas(frame,lteInfo);
    }

    cPacket* pkt = frame->decapsulate();

    if(lteInfo->getFrameType() == SCIPKT)
    {
        double pkt_dist = getCoord().distance(lteInfo->getCoord());
        emit(txRxDistanceSCI, pkt_dist);

        if (!transmitting_)
        {
            result = channelModel_->error_Mode4_D2D(frame, lteInfo, rsrpVector, 0);

            scisReceived_ += 1;

            if (result) {
                SidelinkControlInformation *sci = check_and_cast<SidelinkControlInformation *>(pkt);
                std::tuple<int, int> indexAndLength = decodeRivValue(sci, lteInfo);
                int subchannelIndex = std::get<0>(indexAndLength);
                int lengthInSubchannels = std::get<1>(indexAndLength);

                currentCBR_ += lengthInSubchannels;

                std::vector<Subchannel *>::iterator kt;
                std::vector < Subchannel * > currentSubframe = sensingWindow_.back();
                for (kt = currentSubframe.begin() + subchannelIndex;
                     kt != currentSubframe.begin() + subchannelIndex + lengthInSubchannels; kt++) {
                    // Record the SCI in the subchannel.
                    (*kt)->setSCI(sci->dup());
                }
                lteInfo->setDeciderResult(true);
                pkt->setControlInfo(lteInfo);
                decodedScis_.push_back(pkt);
                scisDecoded_ += 1;
            }
            else
            {
                scisNotDecoded_ += 1;
                delete lteInfo;
                delete pkt;
            }

        }
        else
        {
            sciFailedHalfDuplex_ += 1;
            delete lteInfo;
            delete pkt;
        }
        delete frame;
    }
    else
    {
        double pkt_dist = getCoord().distance(lteInfo->getCoord());
        emit(txRxDistanceTB, pkt_dist);

        if(!transmitting_){

            tbsReceived_ += 1;

            // Have a TB want to make sure we have the SCI for it.
            bool foundCorrespondingSci = false;
            SidelinkControlInformation *correspondingSCI;
            UserControlInfo *sciInfo;
            std::vector<cPacket *>::iterator it;
            for (it = decodedScis_.begin(); it != decodedScis_.end(); it++) {
                sciInfo = check_and_cast<UserControlInfo *>((*it)->removeControlInfo());
                // if the SCI and TB have same source then we have the right SCI
                if (sciInfo->getSourceId() == lteInfo->getSourceId()) {
                    //Successfully received the SCI
                    foundCorrespondingSci = true;

                    correspondingSCI = check_and_cast<SidelinkControlInformation *>(*it);

                    //RELAY and NORMAL
                    if (lteInfo->getDirection() == D2D_MULTI)
                        result = channelModel_->error_Mode4_D2D(frame, lteInfo, rsrpVector, correspondingSCI->getMcs());
                    else
                        result = channelModel_->error(frame, lteInfo);

                    // Remove the SCI
                    decodedScis_.erase(it);
                    break;
                } else {
                    (*it)->setControlInfo(sciInfo);
                }
            }
            if (!foundCorrespondingSci) {
                tbsFailedDueToNoSCI_ += 1;
            } else if (!result) {
                tbFailedButSCIReceived_ += 1;
            } else {
                tbsDecoded_ += 1;
                // Now need to find the associated Subchannels, record the RSRP and RSSI for the message and go from there.
                // Need to again do the RIV steps
                std::tuple<int, int> indexAndLength = decodeRivValue(correspondingSCI, sciInfo);
                int subchannelIndex = std::get<0>(indexAndLength);
                int lengthInSubchannels = std::get<1>(indexAndLength);

                std::vector<Subchannel *>::iterator kt;
                std::vector <Subchannel *> currentSubframe = sensingWindow_[sensingWindowFront_];
                for (kt = currentSubframe.begin() + subchannelIndex;
                     kt != currentSubframe.begin() + subchannelIndex + lengthInSubchannels; kt++) {
                    std::vector<Band>::iterator lt;
                    std::vector <Band> allocatedBands = (*kt)->getOccupiedBands();
                    for (lt = allocatedBands.begin(); lt != allocatedBands.end(); lt++) {
                        // Record RSRP and RSSI for this band
                        (*kt)->addRsrpValue(rsrpVector[(*lt)], (*lt));
                        (*kt)->addRssiValue(rssiVector[(*lt)], (*lt));
                    }
                }
                // Need to delete the message now
                delete correspondingSCI;
                delete sciInfo;
            }
        }
        else{
            tbFailedHalfDuplex_ += 1;
        }

        delete frame;

        // send decapsulated message along with result control info to upperGateOut_
        lteInfo->setDeciderResult(result);
        pkt->setControlInfo(lteInfo);
        send(pkt, upperGateOut_);

        if (getEnvir()->isGUI())
            updateDisplayString();
    }

    // update statistics
    if (result)
    {
        numAirFrameReceived_++;
    }
    else
    {
        numAirFrameNotReceived_++;
    }

    EV << "Handled LteAirframe with ID " << frame->getId() << " with result "
       << (result ? "RECEIVED" : "NOT RECEIVED") << endl;
}

std::tuple<int,int> LtePhyVUeMode4::decodeRivValue(SidelinkControlInformation* sci, UserControlInfo* sciInfo)
{
    EV << NOW << " LtePhyVUeMode4::decodeRivValue - Decoding RIV value of SCI allows correct placement in sensing window..." << endl;
    //UserControlInfo* lteInfo = check_and_cast<UserControlInfo*>(pkt->removeControlInfo());
    RbMap rbMap = sciInfo->getGrantedBlocks();
    RbMap::iterator it;
    std::map<Band, unsigned int>::iterator jt;
    Band startingBand;
    bool bandNotFound = true;

    it = rbMap.begin();

    while (it != rbMap.end() && bandNotFound )
    {
        for (jt = it->second.begin(); jt != it->second.end(); ++jt)
       {
           Band band = jt->first;
           if (jt->second == 1) // this Rb is not allocated
           {
               startingBand = band;
               bandNotFound= false;
               break;
           }
       }
    }

    // Get RIV first as this is common
    unsigned int RIV = sci->getFrequencyResourceLocation();

    // Get the subchannel Index (allows us later to calculate the length of the message
    int subchannelIndex;
    if (adjacencyPSCCHPSSCH_)
    {
        // If adjacent: Subchannel Index = band//subchannelSize
        subchannelIndex = startingBand/subchannelSize_;
    }
    else
    {
        // If non-adjacent: Subchannel Index = band/2
        subchannelIndex = startingBand/2;
    }

    // Based on TS36.213 14.1.1.4C
    // if SubchannelLength -1 < numSubchannels/2
    // RIV = numSubchannels(SubchannelLength-1) + subchannelIndex
    // else
    // RIV = numSubchannels(numSubchannels-SubchannelLength+1) + (numSubchannels-1-subchannelIndex)
    // RIV limit
    // log2(numSubchannels(numSubchannels+1)/2)
    double subchannelLOverHalf = (numSubchannels_ + 2 + (( -1 - subchannelIndex - RIV )/numSubchannels_));
    double subchannelLUnderHalf = (RIV + numSubchannels_ - subchannelIndex)/numSubchannels_;
    int lengthInSubchannels;

    // First the number has to be whole in both cases, it's length + subchannelIndex must be less than the number of subchannels
    if (floor(subchannelLOverHalf) == subchannelLOverHalf && subchannelLOverHalf <= numSubchannels_ && subchannelLOverHalf + subchannelIndex <= numSubchannels_)
    {
        lengthInSubchannels = subchannelLOverHalf;
    }
    // Same as above but also the length must be less than half + 1
    else if (floor(subchannelLUnderHalf) == subchannelLUnderHalf && subchannelLUnderHalf + subchannelIndex <= numSubchannels_ && subchannelLUnderHalf <= numSubchannels_ /2 + 1)
    {
        lengthInSubchannels = subchannelLUnderHalf;
    }
    return std::make_tuple(subchannelIndex, lengthInSubchannels);
}

void LtePhyVUeMode4::updateCBR()
{
    double cbr = 0;
    for (int i=0; i < cbrHistory_.size();i++)
    {
        cbr += cbrHistory_[i];
    }

    cbr = std::round(cbr);

    emit(cbr, cbr);

    Cbr* cbrPkt = new Cbr("CBR");
    cbrPkt->setCbr(cbr);
    send(cbrPkt, upperGateOut_);
}

void LtePhyVUeMode4::updateSubframe()
{
    EV << NOW << " LtePhyVUeMode4::updateSubframe - updating subframe in the sensingWindow..." << endl;

    // Increment the pointer to the next element in the sensingWindow
    if (sensingWindowFront_ < (10*pStep_)-1) {
        ++sensingWindowFront_;
    }
    else{
        // Front has gone over the end of the sensing window reset it.
        sensingWindowFront_ = 0;
    }


    // First find the subframe that we want to look at i.e. the front one I imagine
    // If the front isn't occupied then skip on
    // If it is occupied, pop it off, update it and push it back
    // All good then.

    std::vector<Subchannel*> subframe = sensingWindow_[sensingWindowFront_];

    if (subframe.at(0)->getSubframeTime() <= NOW - SimTime(10*pStep_, SIMTIME_MS))
    {
        std::vector<Subchannel*>::iterator it;
        for (it=subframe.begin(); it!=subframe.end(); it++)
        {
            (*it)->reset(NOW);
        }
    }

    cMessage* updateSubframe = new cMessage("updateSubframe");
    updateSubframe->setSchedulingPriority(0);        // Generate the subframe at start of next TTI
    scheduleAt(NOW + TTI, updateSubframe);
}

void LtePhyVUeMode4::initialiseSensingWindow()
{
    EV << NOW << " LtePhyVUeMode4::initialiseSensingWindow - creating subframes to be added to sensingWindow..." << endl;

    Band band = 0;

    if (!adjacencyPSCCHPSSCH_)
    {
        // This assumes the bands only every have 1 Rb (which is fine as that appears to be the case)
        band = numSubchannels_*2;
    }

    simtime_t subframeTime = NOW;

    // Reserve the full size of the sensing window (might improve efficiency).
    sensingWindow_.reserve(10*pStep_);

    while(sensingWindow_.size() < 10*pStep_)
    {
        std::vector<Subchannel*> subframe;
        subframe.reserve(numSubchannels_);
        for (int i = 0; i < numSubchannels_; i++) {
            Subchannel *currentSubchannel = new Subchannel(subchannelSize_, subframeTime);
            // Need to determine the RSRP and RSSI that corresponds to background noise
            // Best off implementing this in the channel model as a method.

            std::vector <Band> occupiedBands;

            int overallCapacity = 0;
            // Ensure the subchannel is allocated the correct number of RBs
            while (overallCapacity < subchannelSize_ && band < getBinder()->getNumBands()) {
                // This acts like there are multiple RBs per band which is not allowed.
                overallCapacity += allocator_->getAllocatedBlocks(MAIN_PLANE, MACRO, band);
                occupiedBands.push_back(band);
                ++band;
            }
            currentSubchannel->setOccupiedBands(occupiedBands);
            subframe.push_back(currentSubchannel);
        }
        sensingWindow_.push_back(subframe);
        subframeTime += TTI;

    }
    // Send self message to trigger another subframes creation and insertion. Need one for every TTI
    cMessage* updateSubframe = new cMessage("updateSubframe");
    updateSubframe->setSchedulingPriority(0);        // Generate the subframe at start of next TTI
    scheduleAt(NOW + TTI, updateSubframe);
}

int LtePhyVUeMode4::translateIndex(int index) {
    int transIndex;
    if (index > sensingWindowFront_) {
        transIndex = ((10*pStep_) - index) + sensingWindowFront_;
    }
    else{
        transIndex = sensingWindowFront_ - index;
    }
    return transIndex;
}

void LtePhyVUeMode4::finish()
{
    if (getSimulation()->getSimulationStage() != CTX_FINISH)
    {
        // do this only at deletion of the module during the simulation
        //LtePhyUe::finish();
        LteAmc *amc = getAmcModule(masterId_);
        if (amc != NULL)
        {
            amc->detachUser(nodeId_, UL);
            amc->detachUser(nodeId_, DL);
            amc->detachUser(nodeId_, D2D);
        }

        // binder call
        binder_->unregisterNextHop(masterId_, nodeId_);

        // deployer call
        deployer_->detachUser(nodeId_);
    }

    std::vector<std::vector<Subchannel *>>::iterator it;
    for (it=sensingWindow_.begin();it!=sensingWindow_.end();it++)
    {
        std::vector<Subchannel *>::iterator jt;
        for (jt=it->begin();jt!=it->end();jt++)
        {
            delete (*jt);
        }
    }
    sensingWindow_.clear();

    delete allocator_;
}
