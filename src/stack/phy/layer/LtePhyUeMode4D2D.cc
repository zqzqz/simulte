//
//                           SimuLTE
//
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself,
// and cannot be removed from it.
//

#include <assert.h>
#include "stack/phy/layer/LtePhyUeMode4D2D.h"
#include "stack/phy/packet/LteFeedbackPkt.h"
#include "stack/d2dModeSelection/D2DModeSelectionBase.h"
#include "stack/mac/packet/LteMode4SchedulingGrant.h"

Define_Module(LtePhyUeMode4D2D);

LtePhyUeMode4D2D::LtePhyUeMode4D2D()
{
    handoverStarter_ = NULL;
    handoverTrigger_ = NULL;
}

LtePhyUeMode4D2D::~LtePhyUeMode4D2D()
{
}

void LtePhyUeMode4D2D::initialize(int stage)
{
    LtePhyUe::initialize(stage);
    if (stage == 0)
    {
        averageCqiD2D_ = registerSignal("averageCqiD2D");
        d2dTxPower_ = par("d2dTxPower");
        adjacencyPSCCHPSSCH_ = par("adjacencyPSCCHPSSCH");
        pStep_ = par("pStep");
        numSubchannels_ = par("numSubchannels");
        subchannelSize_ = par("subchannelSize");
        d2dMulticastEnableCaptureEffect_ = par("d2dMulticastCaptureEffect");
        d2dDecodingTimer_ = NULL;
        allocator_ = new LteAllocationModule(this,"D2D");
    }
}

void LtePhyUeMode4D2D::handleSelfMessage(cMessage *msg)
{
    if (msg->isName("d2dDecodingTimer"))
    {
        while (!sciFrames_.empty()){
            // Get received SCI and it's corresponding RsrpVector
            LteAirFrame* frame = sciFrames_.back();
            std::vector<vector> rsrpVector = sciRsrpVectors_.back();
            std::vector<vector> rssiVector = sciRssiVectors_.back();

            // Remove it from the vector
            sciFrames_.pop_back();
            sciRsrpVectors_.pop_back();
            sciRssiVectors_.pop_back();

            UserControlInfo* lteInfo = check_and_cast<UserControlInfo*>(frame->removeControlInfo());

            // decode the selected frame
            decodeAirFrame(frame, lteInfo, rsrpVector, rssiVector);
        }
        while (!d2dReceivedFrames_.empty())
        {
            LteAirFrame* frame = d2dReceivedFrames_.back();
            std::vector<vector> rsrpVector = tbRsrpVectors_.back();
            std::vector<vector> rssiVector = tbRssiVectors_.back();

            d2dReceivedFrames_.pop_back();
            tbRsrpVectors_.pop_back();
            tbRssiVectors_.pop_back();

            UserControlInfo* lteInfo = check_and_cast<UserControlInfo*>(frame->removeControlInfo());

            // decode the selected frame
            decodeAirFrame(frame, lteInfo, rsrpVector, rssiVector);
        }
        delete msg;
        d2dDecodingTimer_ = NULL;
    }
    else if (msg->isName("createSubframe"))
    {
        createSubframe();
    }
    else
        LtePhyUe::handleSelfMessage(msg);
}

// TODO: ***reorganize*** method
void LtePhyUeMode4D2D::handleAirFrame(cMessage* msg)
{
    UserControlInfo* lteInfo = check_and_cast<UserControlInfo*>(msg->removeControlInfo());

    if (useBattery_)
    {
        //TODO BatteryAccess::drawCurrent(rxAmount_, 0);
    }
    connectedNodeId_ = masterId_;
    LteAirFrame* frame = check_and_cast<LteAirFrame*>(msg);
    EV << "LtePhyUeMode4D2D: received new LteAirFrame with ID " << frame->getId() << " from channel" << endl;
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

    // Check if the frame is for us ( MacNodeId matches or - if this is a multicast communication - enrolled in multicast group)
    if (lteInfo->getDestId() != nodeId_ && !(binder_->isInMulticastGroup(nodeId_, lteInfo->getMulticastGroupId())))
    {
        EV << "ERROR: Frame is not for us. Delete it." << endl;
        EV << "Packet Type: " << phyFrameTypeToA((LtePhyFrameType)lteInfo->getFrameType()) << endl;
        EV << "Frame MacNodeId: " << lteInfo->getDestId() << endl;
        EV << "Local MacNodeId: " << nodeId_ << endl;
        delete lteInfo;
        delete frame;
        return;
    }

    if (binder_->isInMulticastGroup(nodeId_,lteInfo->getMulticastGroupId()))
    {
        // HACK: if this is a multicast connection, change the destId of the airframe so that upper layers can handle it
        lteInfo->setDestId(nodeId_);
    }

    // send H-ARQ feedback up
    if (lteInfo->getFrameType() == HARQPKT || lteInfo->getFrameType() == GRANTPKT || lteInfo->getFrameType() == RACPKT || lteInfo->getFrameType() == D2DMODESWITCHPKT)
    {
        handleControlMsg(frame, lteInfo);
        return;
    }

    // Deal with and SCI message
    if (lteInfo->getFrameType == SCIPKT)
    {
        // Store the SCI with the appropriate Subchannels (if over multiple store in multiple)
        // When we decode the actual message then we check the associated Subchannels to see if an SCI was successfully received.
        frame->setControlInfo(lteInfo);
        storeAirframe(frame);
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

void LtePhyUeMode4D2D::triggerHandover()
{
    // stop active D2D flows (go back to Infrastructure mode)
    // currently, DM is possible only for UEs served by the same cell

    // trigger D2D mode switch
    cModule* enb = getSimulation()->getModule(binder_->getOmnetId(masterId_));
    D2DModeSelectionBase *d2dModeSelection = check_and_cast<D2DModeSelectionBase*>(enb->getSubmodule("lteNic")->getSubmodule("d2dModeSelection"));
    d2dModeSelection->doModeSwitchAtHandover(nodeId_, false);

    LtePhyUe::triggerHandover();
}

void LtePhyUeMode4D2D::doHandover()
{
    // amc calls
    LteAmc *oldAmc = getAmcModule(masterId_);
    LteAmc *newAmc = getAmcModule(candidateMasterId_);
    assert(newAmc != NULL);
    oldAmc->detachUser(nodeId_, D2D);
    newAmc->attachUser(nodeId_, D2D);

    LtePhyUe::doHandover();

    // call mode selection module to check if DM connections are possible
    cModule* enb = getSimulation()->getModule(binder_->getOmnetId(masterId_));
    D2DModeSelectionBase *d2dModeSelection = check_and_cast<D2DModeSelectionBase*>(enb->getSubmodule("lteNic")->getSubmodule("d2dModeSelection"));
    d2dModeSelection->doModeSwitchAtHandover(nodeId_, true);
}

void LtePhyUeMode4D2D::handleUpperMessage(cMessage* msg)
{
//    if (useBattery_) {
//    TODO     BatteryAccess::drawCurrent(txAmount_, 1);
//    }

    UserControlInfo* lteInfo = check_and_cast<UserControlInfo*>(msg->removeControlInfo());

    if (lteInfo->getFrameType() == GRANTPKT)
    {
        // Generate CSRs or save the grant for use when generating SCI information
        LteMode4SchedulingGrant grant = check_and_cast<LteMode4SchedulingGrant*>(msg);
        if (grant->getTotalGrantedBlocks() == 0){
            // Generate a list of CSRs and send it to the MAC layer
            computeCSRs(grant);
        }
        else
        {
            sciGrant_ = grant;
        }
    }

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

    if (lteInfo->getFrameType() == DATAPKT && lteInfo->getUserTxParams() != NULL)
    {
        double cqi = lteInfo->getUserTxParams()->readCqiVector()[lteInfo->getCw()];
        if (lteInfo->getDirection() == UL)
            emit(averageCqiUl_, cqi);
        else if (lteInfo->getDirection() == D2D)
            emit(averageCqiD2D_, cqi);

        if (adjacencyPSCCHPSSCH_)
            {
                // Setup so SCI gets 2 RBs from the grantedBlocks.
                RbMap sciRbs;

                // get the average RSRP on the RBs allocated for the transmission
                RbMap::iterator it;
                std::map<Band, unsigned int>::iterator jt;
                //for each Remote unit used to transmit the packet
                for (it = rbMap.begin(); it != rbMap.end(); ++it)
                {
                    int allocatedBlocks = 0;
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
                            // Do the allocation of resources.
                            // Might be the case that this is only ever set to 1 as in assigned
                            // Certain that his is the case
                            // TODO: Change so only the case of if it == 1
                            if (jt->second > 2)
                            {
                                jt->second -= 2;
                                sciRbs[it->first][band] = 2;
                                allocatedBlocks += 2;
                            }
                            else if (jt->second == 1){
                                jt->second -= 1;
                                sciRbs[it->first][band] = 1;
                                allocatedBlocks += 1;
                            }
                        }
                    }
                }
                UserControlInfo* SCIInfo = lteInfo;
                SCIInfo->setFrameType(SCIPKT);
                SCIInfo->setGrantedBlocks(sciRbs);
            }
            else
            {
                // setup so SCI gets 2 RBs from PSCCH allocated RBs
                // TODO: In initialise setup certain Rbs for PSCCH and allow them to be allocated in this section

            }
    }

    EV << NOW << " LtePhyUeMode4D2D::handleUpperMessage - message from stack" << endl;
    LteAirFrame* frame = NULL;

    if (lteInfo->getFrameType() == HARQPKT)
    {
        frame = new LteAirFrame("harqFeedback-grant");
    }
    else
    {
        // create LteAirFrame and encapsulate the received packet
        SidelinkControlInformation SCI = createSCIMessage(msg);
        sciFrame = prepareAirFrame(SCI, SCIInfo);

        // TODO: Signal for Sending SCI
        sendBroadcast(sciFrame);

        frame = prepareAirFrame(msg, lteInfo);
    }

    // if this is a multicast/broadcast connection, send the frame to all neighbors in the hearing range
    // otherwise, send unicast to the destination

    EV << "LtePhyUeMode4D2D::handleUpperMessage - " << nodeTypeToA(nodeType_) << " with id " << nodeId_
           << " sending message to the air channel. Dest=" << lteInfo->getDestId() << endl;
    if (lteInfo->getDirection() == D2D_MULTI)
        sendBroadcast(frame);
    else
        sendUnicast(frame);
}

void LtePhyUeMode4D2D::computeCSRs(LteMode4SchedulingGrant* grant)
{

}

LteAirFrame* LtePhyUeMode4D2D::prepareAirFrame(cPacket* msg, UserControlInfo* lteInfo){
    // Helper function to prepare airframe for sending.
    frame = new LteAirframe("airframe");

    frame->encapsulate(check_and_cast<cPacket*>(msg));
    frame->setSchedulingPriority(airFramePriority_);
    frame->setDuration(TTI);

    lteInfo->setCoord(getRadioPosition());

    lteInfo->setTxPower(txPower_);
    lteInfo->setD2dTxPower(d2dTxPower_);
    frame->setControlInfo(lteInfo);

    return frame;
}

SidelinkControlInformation* LtePhyUeMode4D2D::createSCIMessage(cPacket* message)
{
    // So what is specified, a bit anyway

    /*
     * Priority (based on upper layer)
     * 0-7
     * Mapping unknown, so possibly just take the priority max and min and map to 0-7
     *
     * Resource Interval
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
     * frequency Resource Location
     * Based on another parameter RIV
     * but size is
     * Log2(Nsubchannel(Nsubchannel+1)/2) (rounded up)
     * 0 - 8 bits
     * 0 - 256 different values
     *
     * TimeGapRetrans
     * 1 - 15
     * ms from init to retrans
     *
     * mcs
     * 5 bits
     * 26 combos
     *
     * retransmissionIndex
     * 0 / 1
     * if 0 can still have retrans based on TimeGap
     * if 1 will have retrans 2 times maybe???
     *
     * Filler up to 32 bits
     *
     * Have the Grant associated with the SCI and as such can fill this correctly.
     */
}



void LtePhyUeMode4D2D::storeAirFrame(LteAirFrame* newFrame)
{
    // implements the capture effect
    // store the frame received from the nearest transmitter
    UserControlInfo* newInfo = check_and_cast<UserControlInfo*>(newFrame->getControlInfo());
    Coord myCoord = getCoord();
    double rsrpMean = 0.0;
    std::vector<double> rsrpVector;
    std::vector<double> rssiVector;

    double sum = 0.0;
    unsigned int allocatedRbs = 0;
    rsrpVector = channelModel_->getRSRP_D2D(newFrame, newInfo, nodeId_, myCoord);
    // TODO: Seems we don't really actually need the enbId, I have set it to 0 as it is referenced but never used for calc
    rssiVector = channelModel_->getSINR_D2D(newFrame, newInfo, nodeId_, myCoord, 0, rsrpVector);

    // TODO: Update the subchannel associated with this transmission to include the average RSRP for the sub channel
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

void LtePhyUeMode4D2D::decodeAirFrame(LteAirFrame* frame, UserControlInfo* lteInfo, std::vector* rsrpVector, std::vector* rssiVector)
{
    EV << NOW << " LtePhyUeMode4D2D::decodeAirFrame - Start decoding..." << endl;

    // apply decider to received packet
    bool result = true;

    RemoteSet r = lteInfo->getUserTxParams()->readAntennaSet();
    if (r.size() > 1)
    {
        // DAS
        for (RemoteSet::iterator it = r.begin(); it != r.end(); it++)
        {
            EV << "LtePhyUeMode4D2D::decodeAirFrame: Receiving Packet from antenna " << (*it) << "\n";

            /*
             * On UE set the sender position
             * and tx power to the sender das antenna
             */

//            cc->updateHostPosition(myHostRef,das_->getAntennaCoord(*it));
            // Set position of sender
//            Move m;
//            m.setStart(das_->getAntennaCoord(*it));
            RemoteUnitPhyData data;
            data.txPower=lteInfo->getTxPower();
            data.m=getRadioPosition();
            frame->addRemoteUnitPhyDataVector(data);
        }
        // apply analog models For DAS
        result=channelModel_->errorDas(frame,lteInfo);
    }
    else
    {
        //RELAY and NORMAL
        if (lteInfo->getDirection() == D2D_MULTI)
            result = channelModel_->error_D2D(frame,lteInfo,rsrpVector_);
        else
            result = channelModel_->error(frame,lteInfo);
    }

    // update statistics
    if (result)
        numAirFrameReceived_++;
    else
        numAirFrameNotReceived_++;

    EV << "Handled LteAirframe with ID " << frame->getId() << " with result "
       << ( result ? "RECEIVED" : "NOT RECEIVED" ) << endl;

    cPacket* pkt = frame->decapsulate();

    // attach the decider result to the packet as control info
    lteInfo->setDeciderResult(result);
    pkt->setControlInfo(lteInfo);

    // Want to record RSSI + RSRP for all the blocks allocated to this TB/SCI
    RbMap rbMap = lteInfo->getGrantedBlocks();
    RbMap::iterator it;
    std::map<Band, unsigned int>::iterator jt;
    //for each Remote unit used to transmit the packet

    std::list<Subchannel> currentSubframe = sensingWindow_.front();

    for (it = rbmap.begin(); it != rbmap.end(); ++it)
    {
        //for each logical band used to transmit the packet
        for (jt = it->second.begin(); jt != it->second.end(); ++jt)
        {
            Band band = jt->first;
            if (jt->second == 0) // this Rb is not allocated
                continue;
            // This band is in fact allocated
            std::list<Subchannel>::iterator kt;
            for (kt = currentSubframe.begin(); kt != currentSubframe.end(); ++kt)
            {
                // Check that the current band is in the current subchannel kt
                // if it is
                // Record the SCI
                // Add this bands RSRP + RSSI value for this band
                // Move to the next band until all bands fully done.
                if (std::find(kt->getOccupiedBands().begin(), kt->getOccupiedBands().end(), band) != kt->getOccupiedBands().end())
                {
                    if (lteInfo->getFrameType() == SCIPKT && result)
                    {
                        // If we successfully received the SCI then record it.
                        // TODO: This is not all that elegant, there is definitely a better solution to this.
                        kt->setSCI(pkt);
                    }
                    kt->addRsrpValue(rsrpVector[band], band);
                    kt->addRssiValue(rssiVector[band], band);
                }
            }
        }
    }

    // If the packet is an SCI message we need to associate it with it's subchannels
    // This currently works by assuming adjacency (you pick the subframe corresponding to
    // the one used by this SCI. This won't work in non-adjacent configurations. Instead you
    // must say which RB did the SCI take this corresponds to the startingSubchannelIndex and
    // go from there. Seems I really need to rethink this approach.

    // TODO: Base this approach on RIV
    // RIV = NsubCH(LsubCH-1)+nstartSubCH if Lsubch-1 < NsubCh/2
    // RIV = NsubCh(NsubCH-LsubCH+1)+(NsubCH-1-NstartSubCH) if LsubCH-1 > NsubCH/2
    // NsubCh = numSubchannels_
    // LsubCH = ??
    // NstartSubCh = starting subchannel for the message (
    // if Adjacent: Current subchannel
    // if non-adjacent corresponding subchannel i.e. RB 0 nStartSubChx = 0 RB 3 nStartSubCh = 1 RB 5 nStartSubCh = 2
    if (lteInfo->getFrameType() == SCIPKT)
    {
        RbMap rbmap = newInfo->getGrantedBlocks();

        RbMap::iterator it;
        std::map<Band, unsigned int>::iterator jt;
        //for each Remote unit used to transmit the packet

        std::list<Subchannel> currentSubframe = sensingWindow_.front();

        for (it = rbmap.begin(); it != rbmap.end(); ++it)
        {
            //for each logical band used to transmit the packet
            for (jt = it->second.begin(); jt != it->second.end(); ++jt)
            {
                Band band = jt->first;
                if (jt->second == 0) // this Rb is not allocated
                    continue;
                // This band is in fact allocated
                std::list<Subchannel>::iterator kt;
                for (kt = currentSubframe.begin(); kt != currentSubframe.end(); ++kt)
                {
                    // Check that the current band is in the current subchannel kt
                    // if it is
                    // Record the SCI
                    // Add this bands RSRP + RSSI value for this band
                    // Move to the next band until all bands fully done.
                    if (std::find(kt->getOccupiedBands().begin(), kt->getOccupiedBands().end(), band) != kt->getOccupiedBands().end())
                    {
                        if (result)
                        {
                            // If we successfully received the SCI then record it.

                            // TODO: This is not all that elegant, there is definitely a better solution to this.
                            kt->setSCI(pkt);
                        }
                        kt->addRsrpValue(rsrpVector[band], band);
                        kt->addRssiValue(rssiVector[band], band);
                    }
                }
            }
        }
        // We do not want to send SCI messages to the upper layer.
        return;
    }
    else
    {
        std::vector<cPacket>::iterator it;
        for(it=decodedScis_.begin(); it!=decodedScis_.end(); it++)
        {
            foundCorrespondingSci = false;
            successfullyReceivedSci = false;
            UserControlInfo *sciInfo = check_and_cast<UserControlInfo *>(it->getControlInfo());
            SidelinkControlInformation *sci = check_and_cast<SidelinkControlInformation*>(it);
            if (sciInfo->getSourceId() == lteInfo->getSourceId())
            {
                // SCI has same source as current TB (Therefore this SCI is for this TB)
                foundCorrespondingSci = true;
                if (sciInfo->getDeciderResult())
                {
                    // Successfully received the SCI
                    successfullyReceivedSci = true;

                    // Remove the SCI
                    decodedScis_.erase(it);

                    // No need to keep searching for new SCIs
                    break;
                }
            }
            // TODO: Signal to say we never found an SCI for this TB.
        }
        if(lteInfo->getDeciderResult())
        {
            // TODO: Signal for this (maybe received TB and result is x
            lteInfo->setDeciderResult(successfullyReceivedSci);
        }
        else
        {
            // TODO: Signal to say we got a TB but failed to read it and our result of the SCI is x
        }
    }

    // send decapsulated message along with result control info to upperGateOut_
    send(pkt, upperGateOut_);

    if (getEnvir()->isGUI())
        updateDisplayString();
}

void LtePhyUeMode4D2D::createSubframe()
{
    // TODO: Figure out where the deployer exists
    // We need it to get the number of Rbs and it must exists somewhere
    allocator_->initAndReset(getDeployer(MacNodeId)->getNumRbUl(), getBinder()->getNumBands());
    std::list<Subchannel> subframe;
    Band band = 0;
    for (int i = 0; i < numSubchannels_; i++)
    {
        Subchannel currentSubchannel = new Subchannel(subchannelSize_);
        // Need to determine the RSRP and RSSI that corresponds to background noise
        // Best off implementing this in the channel model as a method.

        std:vector<Band> occupiedBands;

        int overallCapacity = 0;
        // Ensure the subchannel is allocated the correct number of RBs
        while(overallCapacity < subchannelSize_ && band < getBinder()->getNumBands()){
            overallCapacity += allocator_->getAllocatedBlocks(MAIN_PLANE, MACRO, band);
            occupiedBands.push(band);
            ++band;
        }
        currentSubchannel->setOccupiedBands(occupiedBands);
        subframe.push(currentSubchannel);
    }
    if (sensingWindow_.size() < 10*pStep_)
    {
        sensingWindow_.push_front(subframe);
    }
    else
    {
        sensingWindow_.pop_back();
        sensingWindow_.push_front(subframe);
    }

    // Send self message to trigger another subframes creation and insertion. Need one for every TTI
    createSubframe = new cMessage("createSubframe");
    createSubframe->setSchedulingPriority(0);        // Generate the subframe at start of next TTI
    scheduleAt(NOW + TTI, createSubframe);
}


void LtePhyUeMode4D2D::sendFeedback(LteFeedbackDoubleVector fbDl, LteFeedbackDoubleVector fbUl, FeedbackRequest req)
{
    Enter_Method("SendFeedback");
    EV << "LtePhyUeMode4D2D: feedback from Feedback Generator" << endl;

    //Create a feedback packet
    LteFeedbackPkt* fbPkt = new LteFeedbackPkt();
    //Set the feedback
    fbPkt->setLteFeedbackDoubleVectorDl(fbDl);
    fbPkt->setLteFeedbackDoubleVectorDl(fbUl);
    fbPkt->setSourceNodeId(nodeId_);
    UserControlInfo* uinfo = new UserControlInfo();
    uinfo->setSourceId(nodeId_);
    uinfo->setDestId(masterId_);
    uinfo->setFrameType(FEEDBACKPKT);
    uinfo->setIsCorruptible(false);
    // create LteAirFrame and encapsulate a feedback packet
    LteAirFrame* frame = new LteAirFrame("feedback_pkt");
    frame->encapsulate(check_and_cast<cPacket*>(fbPkt));
    uinfo->feedbackReq = req;
    uinfo->setDirection(UL);
    simtime_t signalLength = TTI;
    uinfo->setTxPower(txPower_);
    uinfo->setD2dTxPower(d2dTxPower_);
    // initialize frame fields

    frame->setSchedulingPriority(airFramePriority_);
    frame->setDuration(signalLength);

    uinfo->setCoord(getRadioPosition());

    frame->setControlInfo(uinfo);
    //TODO access speed data Update channel index
//    if (coherenceTime(move.getSpeed())<(NOW-lastFeedback_)){
//        deployer_->channelIncrease(nodeId_);
//        deployer_->lambdaIncrease(nodeId_,1);
//    }
    lastFeedback_ = NOW;
    EV << "LtePhy: " << nodeTypeToA(nodeType_) << " with id "
       << nodeId_ << " sending feedback to the air channel" << endl;
    sendUnicast(frame);
}

void LtePhyUeMode4D2D::finish()
{
    if (getSimulation()->getSimulationStage() != CTX_FINISH)
    {
        // do this only at deletion of the module during the simulation

        // amc calls
        LteAmc *amc = getAmcModule(masterId_);
        if (amc != NULL)
            amc->detachUser(nodeId_, D2D);

        LtePhyUe::finish();
    }
}
