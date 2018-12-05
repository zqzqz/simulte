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

/**
 * LteMacUeMode4D2D is a new model which implements the functionality of LTE Mode 4 as per 3GPP release 14
 * Author: Brian McCarthy
 * Email: b.mccarthy@cs.ucc.ie
 */

#include "stack/mac/buffer/harq/LteHarqBufferRx.h"
#include "stack/mac/buffer/LteMacQueue.h"
#include "stack/mac/buffer/harq_d2d/LteHarqBufferRxD2DMirror.h"
#include "stack/mac/packet/LteMode4SchedulingGrant.h"
#include "stack/mac/layer/LteMacUeMode4D2D.h"
#include "stack/phy/packet/SpsCandidateResources.h"
#include "stack/mac/scheduler/LteSchedulerUeUl.h"
#include <random>

Define_Module(LteMacUeMode4D2D);

LteMacUeMode4D2D::LteMacUeMode4D2D() :
    LteMacUeRealisticD2D()
{
    std::mt19937 generator(rand_dev());
}

LteMacUeMode4D2D::~LteMacUeMode4D2D()
{
}

void LteMacUeMode4D2D::initialize(int stage)
{
    LteMacUeRealisticD2D::initialize(stage);
}

//Function for create a BSR for the eNB, this is something I do not need and need to strip from the rest of this implementation
//LteMacPdu* LteMacUeMode4D2D::makeBsr(int size){
//
//    UserControlInfo* uinfo = new UserControlInfo();
//    uinfo->setSourceId(getMacNodeId());
//    uinfo->setDestId(getMacCellId());
//    uinfo->setDirection(UL);
//    uinfo->setUserTxParams(schedulingGrant_->getUserTxParams()->dup());
//    LteMacPdu* macPkt = new LteMacPdu("LteMacPdu");
//    macPkt->setHeaderLength(MAC_HEADER);
//    macPkt->setControlInfo(uinfo);
//    macPkt->setTimestamp(NOW);
//    MacBsr* bsr = new MacBsr();
//    bsr->setTimestamp(simTime().dbl());
//    bsr->setSize(size);
//    macPkt->pushCe(bsr);
//    bsrTriggered_ = false;
//    EV << "LteMacUeRealisticD2D::makeBsr() - BSR with size " << size << "created" << endl;
//    return macPkt;
//}

void LteMacUeMode4D2D::macPduMake()
{
    int64 size = 0;

    macPduList_.clear();

    /*
     * The following section generates a BSR which is in essence a request to the eNodeB for access to the medium
     * This obviously is something we don't need here and needs to be removed completely
     */

//    bool bsrAlreadyMade = false;
//    // UE is in D2D-mode but it received an UL grant (for BSR)
//    if ((bsrTriggered_ || bsrD2DMulticastTriggered_) && schedulingGrant_->getDirection() == UL && scheduleList_->empty())
//    {
//        // Compute BSR size taking into account only DM flows
//        int sizeBsr = 0;
//        LteMacBufferMap::const_iterator itbsr;
//        for (itbsr = macBuffers_.begin(); itbsr != macBuffers_.end(); itbsr++)
//        {
//            MacCid cid = itbsr->first;
//            Direction connDir = (Direction)connDesc_[cid].getDirection();
//
//            // if the bsr was triggered by D2D (D2D_MULTI), only account for D2D (D2D_MULTI) connections
//            if (bsrTriggered_ && connDir != D2D)
//                continue;
//            if (bsrD2DMulticastTriggered_ && connDir != D2D_MULTI)
//                continue;
//
//            sizeBsr += itbsr->second->getQueueOccupancy();
//
//            // take into account the RLC header size
//            if (sizeBsr > 0)
//            {
//                if (connDesc_[cid].getRlcType() == UM)
//                    sizeBsr += RLC_HEADER_UM;
//                else if (connDesc_[cid].getRlcType() == AM)
//                    sizeBsr += RLC_HEADER_AM;
//            }
//        }
//
//        if (sizeBsr > 0)
//        {
//            // Call the appropriate function for make a BSR for a D2D communication
//            LteMacPdu* macPktBsr = makeBsr(sizeBsr);
//            UserControlInfo* info = check_and_cast<UserControlInfo*>(macPktBsr->getControlInfo());
//            if (bsrD2DMulticastTriggered_)
//            {
//                info->setLcid(D2D_MULTI_SHORT_BSR);
//                bsrD2DMulticastTriggered_ = false;
//            }
//            else
//                info->setLcid(D2D_SHORT_BSR);
//
//            // Add the created BSR to the PDU List
//            if( macPktBsr != NULL )
//            {
//               macPduList_[ std::pair<MacNodeId, Codeword>( getMacCellId(), 0) ] = macPktBsr;
//               bsrAlreadyMade = true;
//               EV << "LteMacUeRealisticD2D::macPduMake - BSR D2D created with size " << sizeBsr << "created" << endl;
//            }
//        }
//        else
//        {
//            bsrD2DMulticastTriggered_ = false;
//            bsrTriggered_ = false;
//        }
//    }

    // In a D2D communication if BSR was created above this part isn't executed
    // Build a MAC PDU for each scheduled user on each codeword
    LteMacScheduleList::const_iterator it;
    for (it = scheduleList_->begin(); it != scheduleList_->end(); it++)
    {
        LteMacPdu* macPkt;
        cPacket* pkt;

        MacCid destCid = it->first.first;
        Codeword cw = it->first.second;

        // get the direction (UL/D2D/D2D_MULTI) and the corresponding destination ID
        FlowControlInfo* lteInfo = &(connDesc_.at(destCid));
        MacNodeId destId = lteInfo->getDestId();
        Direction dir = (Direction)lteInfo->getDirection();

        std::pair<MacNodeId, Codeword> pktId = std::pair<MacNodeId, Codeword>(destId, cw);
        unsigned int sduPerCid = it->second;

        MacPduList::iterator pit = macPduList_.find(pktId);

        if (sduPerCid == 0)
        {
            continue;
        }

        // No packets for this user on this codeword
        if (pit == macPduList_.end())
        {
            // Always goes here because of the macPduList_.clear() at the beginning
            // Build the Control Element of the MAC PDU
            UserControlInfo* uinfo = new UserControlInfo();
            uinfo->setSourceId(getMacNodeId());
            uinfo->setDestId(destId);
            uinfo->setLcid(MacCidToLcid(destCid));
            uinfo->setDirection(dir);
            uinfo->setLcid(MacCidToLcid(SHORT_BSR));
            if (usePreconfiguredTxParams_)
                uinfo->setUserTxParams(preconfiguredTxParams_->dup());
            else
                uinfo->setUserTxParams(schedulingGrant_->getUserTxParams()->dup());
            // Create a PDU
            macPkt = new LteMacPdu("LteMacPdu");
            macPkt->setHeaderLength(MAC_HEADER);
            macPkt->setControlInfo(uinfo);
            macPkt->setTimestamp(NOW);
            macPduList_[pktId] = macPkt;
        }
        else
        {
            // Never goes here because of the macPduList_.clear() at the beginning
            macPkt = pit->second;
        }

        while (sduPerCid > 0)
        {
            // Add SDU to PDU
            // Find Mac Pkt
            if (mbuf_.find(destCid) == mbuf_.end())
                throw cRuntimeError("Unable to find mac buffer for cid %d", destCid);

            if (mbuf_[destCid]->empty())
                throw cRuntimeError("Empty buffer for cid %d, while expected SDUs were %d", destCid, sduPerCid);

            pkt = mbuf_[destCid]->popFront();

            // multicast support
            // this trick gets the group ID from the MAC SDU and sets it in the MAC PDU
            int32 groupId = check_and_cast<LteControlInfo*>(pkt->getControlInfo())->getMulticastGroupId();
            if (groupId >= 0) // for unicast, group id is -1
                check_and_cast<LteControlInfo*>(macPkt->getControlInfo())->setMulticastGroupId(groupId);

            drop(pkt);

            macPkt->pushSdu(pkt);
            sduPerCid--;
        }

        // consider virtual buffers to compute BSR size
        size += macBuffers_[destCid]->getQueueOccupancy();

        if (size > 0)
        {
            // take into account the RLC header size
            if (connDesc_[destCid].getRlcType() == UM)
                size += RLC_HEADER_UM;
            else if (connDesc_[destCid].getRlcType() == AM)
                size += RLC_HEADER_AM;
        }
    }

    // Put MAC PDUs in H-ARQ buffers
    MacPduList::iterator pit;
    for (pit = macPduList_.begin(); pit != macPduList_.end(); pit++)
    {
        MacNodeId destId = pit->first.first;
        Codeword cw = pit->first.second;
        // Check if the HarqTx buffer already exists for the destId
        // Get a reference for the destId TXBuffer
        LteHarqBufferTx* txBuf;
        HarqTxBuffers::iterator hit = harqTxBuffers_.find(destId);
        if ( hit != harqTxBuffers_.end() )
        {
            // The tx buffer already exists
            txBuf = hit->second;
        }
        else
        {
            // The tx buffer does not exist yet for this mac node id, create one
            LteHarqBufferTx* hb;
            // FIXME: hb is never deleted
            UserControlInfo* info = check_and_cast<UserControlInfo*>(pit->second->getControlInfo());
            if (info->getDirection() == UL)
                hb = new LteHarqBufferTx((unsigned int) ENB_TX_HARQ_PROCESSES, this, (LteMacBase*) getMacByMacNodeId(destId));
            else // D2D or D2D_MULTI
                hb = new LteHarqBufferTxD2D((unsigned int) ENB_TX_HARQ_PROCESSES, this, (LteMacBase*) getMacByMacNodeId(destId));
            harqTxBuffers_[destId] = hb;
            txBuf = hb;
        }

        // search for an empty unit within current harq process
        UnitList txList = txBuf->getEmptyUnits(currentHarq_);
        EV << "LteMacUeRealisticD2D::macPduMake - [Used Acid=" << (unsigned int)txList.first << "] , [curr=" << (unsigned int)currentHarq_ << "]" << endl;

        //Get a reference of the LteMacPdu from pit pointer (extract Pdu from the MAP)
        LteMacPdu* macPkt = pit->second;

        // Attach BSR to PDU if RAC is won and wasn't already made
        // Again no need for BSR type operations
//        if ((bsrTriggered_ || bsrD2DMulticastTriggered_) && !bsrAlreadyMade )
//        {
//            MacBsr* bsr = new MacBsr();
//            bsr->setTimestamp(simTime().dbl());
//            bsr->setSize(size);
//            macPkt->pushCe(bsr);
//            bsrTriggered_ = false;
//            bsrD2DMulticastTriggered_ = false;
//            EV << "LteMacUeRealisticD2D::macPduMake - BSR created with size " << size << endl;
//        }

        EV << "LteMacUeRealisticD2D: pduMaker created PDU: " << macPkt->info() << endl;

        // TODO: harq test
        // pdu transmission here (if any)
        // txAcid has HARQ_NONE for non-fillable codeword, acid otherwise
        if (txList.second.empty())
        {
            EV << "LteMacUeRealisticD2D() : no available process for this MAC pdu in TxHarqBuffer" << endl;
            delete macPkt;
        }
        else
        {
            //Insert PDU in the Harq Tx Buffer
            //txList.first is the acid
            txBuf->insertPdu(txList.first,cw, macPkt);
        }
    }
}

void LteMacUeMode4D2D::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage())
    {
        LteMacUeRealisticD2D::handleMessage(msg);
        return;
    }

    cPacket* pkt = check_and_cast<cPacket *>(msg);
    cGate* incoming = pkt->getArrivalGate();

    if (incoming == down_[IN])
    {
        UserControlInfo *userInfo = check_and_cast<UserControlInfo *>(pkt->getControlInfo());
        if (userInfo->getFrameType() == SPSCANDIDATESPKT)
        {
            EV << "LteMacUeMode4D2D::handleMessage - Received packet " << pkt->getName() <<
            " from port " << pkt->getArrivalGate()->getName() << endl;

            // message from PHY_to_MAC gate (from lower layer)
            emit(receivedPacketFromLowerLayer, pkt);

            // call handler
            macHandleSps(pkt);

            return;
        }
    }

    LteMacUeRealisticD2D::handleMessage(msg);
}



void LteMacUeMode4D2D::handleSelfMessage()
{
    /**
     * Pure copy and paste job so far, want to see if I can get away with not having to re-implement the whole thing
     * only issue is that I'll have to make this somewhat tidier down the road as it is a bit silly doing it this way.
     */
    EV << "----- UE MAIN LOOP -----" << endl;

    // extract pdus from all harqrxbuffers and pass them to unmaker
    HarqRxBuffers::iterator hit = harqRxBuffers_.begin();
    HarqRxBuffers::iterator het = harqRxBuffers_.end();
    LteMacPdu *pdu = NULL;
    std::list<LteMacPdu*> pduList;

    for (; hit != het; ++hit)
    {
        pduList=hit->second->extractCorrectPdus();
        while (! pduList.empty())
        {
            pdu=pduList.front();
            pduList.pop_front();
            macPduUnmake(pdu);
        }
    }

    // I don't have an eNB so this isn't necessary.

    // For each D2D communication, the status of the HARQRxBuffer must be known to the eNB
    // For each HARQ-RX Buffer corresponding to a D2D communication, store "mirror" buffer at the eNB
//    HarqRxBuffers::iterator buffIt = harqRxBuffers_.begin();
//    for (; buffIt != harqRxBuffers_.end(); ++buffIt)
//    {
//        MacNodeId senderId = buffIt->first;
//        LteHarqBufferRx* buff = buffIt->second;
//
//        // skip the H-ARQ buffer corresponding to DL transmissions
//        if (senderId == cellId_)
//            continue;
//
//        // skip the H-ARQ buffers corresponding to D2D_MULTI transmissions
//        if (buff->isMulticast())
//            continue;
//
//        //The constructor "extracts" all the useful information from the harqRxBuffer and put them in a LteHarqBufferRxD2DMirror object
//        //That object resides in enB. Because this operation is done after the enb main loop the enb is 1 TTI backward respect to the Receiver
//        //This means that enb will check the buffer for retransmission 3 TTI before
//        // DANGER BELOW!
//        // I don't have an enb_ so I suppose we can comment this bit of danger out for the moment
//        LteHarqBufferRxD2DMirror* mirbuff = new LteHarqBufferRxD2DMirror(buff, (unsigned char)this->par("maxHarqRtx"), senderId);
//        enb_->storeRxHarqBufferMirror(nodeId_, mirbuff);
//    }

    EV << NOW << "LteMacUeMode4D2D::handleSelfMessage " << nodeId_ << " - HARQ process " << (unsigned int)currentHarq_ << endl;
    // updating current HARQ process for next TTI

    //unsigned char currentHarq = currentHarq_;

    // no grant available - if user has backlogged data, it will trigger scheduling request
    // no harq counter is updated since no transmission is sent.

    bool generateNewSchedulingGrant = false;

    if (schedulingGrant_ == NULL)
    {
        EV << NOW << " LteMacUeMode4D2D::handleSelfMessage " << nodeId_ << " NO configured grant" << endl;

        // scheduling grant needs to be generated at end of process
        generateNewSchedulingGrant = true;
        // TODO ensure all operations done  before return ( i.e. move H-ARQ rx purge before this point)
    }

    // Currently set to <= imagining the case that we end up half an ms off from each other,
    // not sure if this is a likely situation but to be safe will leave it <= for now.l
    else if (schedulingGrant_->getPeriodic() && schedulingGrant_ -> getPeriod() <= NOW)
    {
        // Periodic checks
        if(schedulingGrant_->getExpiration() == 1)
        {
            // Periodic grant is expired
            std::uniform_real_distribution<float> floatdist(0, 1);
            float randomReReserve = floatdist(generator);
            if (randomReReserve < probResourceKeep)
            {
                generateNewSchedulingGrant = true;
            }
            else
            {
                std::uniform_int_distribution<int> range(5, 15);
                int expiration = range(generator);
                schedulingGrant_ -> setExpiration(expiration);
            }
        }
    }

    bool requestSdu = false;
    if (schedulingGrant_!=NULL && schedulingGrant_ -> getPeriod() <= NOW ) // if a grant is configured
    {
        if(!firstTx)
        {
            EV << "\t currentHarq_ counter initialized " << endl;
            firstTx=true;
            // the eNb will receive the first pdu in 2 TTI, thus initializing acid to 0
//            currentHarq_ = harqRxBuffers_.begin()->second->getProcesses() - 2;
            currentHarq_ = UE_TX_HARQ_PROCESSES - 2;
        }
        EV << "\t " << schedulingGrant_ << endl;

        EV << NOW << " LteMacUeMode4D2D::handleSelfMessage " << nodeId_ << " entered scheduling" << endl;

        bool retx = false;

        HarqTxBuffers::iterator it2;
        LteHarqBufferTx * currHarq;
        for(it2 = harqTxBuffers_.begin(); it2 != harqTxBuffers_.end(); it2++)
        {
            EV << "\t Looking for retx in acid " << (unsigned int)currentHarq_ << endl;
            currHarq = it2->second;

            // check if the current process has unit ready for retx
            bool ready = currHarq->getProcess(currentHarq_)->hasReadyUnits();
            CwList cwListRetx = currHarq->getProcess(currentHarq_)->readyUnitsIds();

            EV << "\t [process=" << (unsigned int)currentHarq_ << "] , [retx=" << ((retx)?"true":"false")
               << "] , [n=" << cwListRetx.size() << "]" << endl;

            // if a retransmission is needed
            if(ready)
            {
                UnitList signal;
                signal.first=currentHarq_;
                signal.second = cwListRetx;
                currHarq->markSelected(signal,schedulingGrant_->getUserTxParams()->getLayers().size());
                retx = true;
            }
        }
        // if no retx is needed, proceed with normal scheduling
        if(!retx)
        {
            scheduleList_ = lcgScheduler_->schedule();
            if (scheduleList_->empty())
            {
                // no connection scheduled, but probably can make a PDU (not sure but we can see)
                macPduMake();
            }
            else
            {
                requestSdu = macSduRequest(); // return a bool
            }

        }

        // Message that triggers flushing of Tx H-ARQ buffers for all users
        // This way, flushing is performed after the (possible) reception of new MAC PDUs
        cMessage* flushHarqMsg = new cMessage("flushHarqMsg");
        flushHarqMsg->setSchedulingPriority(1);        // after other messages
        scheduleAt(NOW, flushHarqMsg);

        // Ensure that the period of the schedulingGrant is one resourceReserationInterval away
        // Also make sure RRI is on the same scale at TTI
        schedulingGrant_->setPeriod(NOW+ (resourceReservationInterval*TTI));
    }
    if (schedulingGrant_ == NULL || generateNewSchedulingGrant)
    {
        generateNewSchedulingGrant();
    }

    //============================ DEBUG ==========================
    HarqTxBuffers::iterator it;

    EV << "\n htxbuf.size " << harqTxBuffers_.size() << endl;

    int cntOuter = 0;
    int cntInner = 0;
    for(it = harqTxBuffers_.begin(); it != harqTxBuffers_.end(); it++)
    {
        LteHarqBufferTx* currHarq = it->second;
        BufferStatus harqStatus = currHarq->getBufferStatus();
        BufferStatus::iterator jt = harqStatus.begin(), jet= harqStatus.end();

        EV << "\t cicloOuter " << cntOuter << " - bufferStatus.size=" << harqStatus.size() << endl;
        for(; jt != jet; ++jt)
        {
            EV << "\t\t cicloInner " << cntInner << " - jt->size=" << jt->size()
               << " - statusCw(0/1)=" << jt->at(0).second << "/" << jt->at(1).second << endl;
        }
    }
    //======================== END DEBUG ==========================

    unsigned int purged =0;
    // purge from corrupted PDUs all Rx H-HARQ buffers
    for (hit= harqRxBuffers_.begin(); hit != het; ++hit)
    {
        // purge corrupted PDUs only if this buffer is for a DL transmission. Otherwise, if you
        // purge PDUs for D2D communication, also "mirror" buffers will be purged
        if (hit->first == cellId_)
            purged += hit->second->purgeCorruptedPdus();
    }
    EV << NOW << " LteMacUeMode4D2D::handleSelfMessage Purged " << purged << " PDUS" << endl;

    if (requestSdu == false)
    {
        // update current harq process id
        currentHarq_ = (currentHarq_+1) % harqProcesses_;
    }

    EV << "--- END UE MAIN LOOP ---" << endl;
}

void LteMacUeMode4D2D::macHandleSps(cPacket* pkt)
{
    // This is where we add the subchannels to the actual scheduling grant, so a few things
    /**
     * 1. Need to ensure in the self message part that if at any point we have a scheduling grant without assigned subchannels, we have to wait
     * 2. Need to pick at random from the SPS list of CSRs
     * 3. Assign the CR
     * 4. return
     */
    SpsCandidateResources* candidatesPacket = check_and_cast<SpsCandidateResources *>(pkt);
    std::vector<RbMap> possibleCSRs = candidatesPacket -> getPossibleCSRs();
    std::vector<simtime_t> CSRStartTimes = candidatesPacket -> getCSRStartTimes();

    // Select random element from vector
    std::uniform_int_distribution<int> distr(0, possibleCSRs.size());
    int index = distr(generator);

    RbMap selectedCr = possibleCSRs.at(index);
    simtime_t selectedStartTime = CSRStartTimes.at(index);

    LteMode4SchedulingGrant* mode4Grant = check_and_cast<LteMode4SchedulingGrant *>(schedulingGrant_);

    mode4Grant -> setStartTime(selectedStartTime);
    mode4Grant -> setPeriodic(true);
    mode4Grant -> setGrantedBlocks(selectedCr);

    // Based on restrictResourceReservation interval But will be between 1 and 15
    // Again technically this needs to reconfigurable as well. But all of that needs to come in through ini and such.
    std::uniform_int_distribution<int> range(5, 15);
    int resourceReselectionCounter = range(generator);
    int period = 100; // Determined by RistrictResourceCounter param (but always 100) though needs to still be configurable

    mode4Grant -> setExpiration(resourceReselectionCounter);
    mode4Grant -> setPeriod(period);

    // Trigger a self message at the correct simtime_t (i.e. startTime for a message to be sent)

}

void LteMacUeMode4D2D::handleUpperMessage(cPacket* pkt)
{
    FlowControlInfo* lteInfo = check_and_cast<FlowControlInfo*>(pkt->getControlInfo());
    MacCid cid = idToMacCid(lteInfo->getDestId(), lteInfo->getLcid());

    // bufferize packet
    bufferizePacket(pkt);

    if (strcmp(pkt->getName(), "lteRlcFragment") == 0)
    {
        // new MAC SDU has been received
        if (pkt->getByteLength() == 0)
            delete pkt;

        // creates pdus from schedule list and puts them in harq buffers
        macPduMake();

        EV << NOW << " LteMacUeRealistic::handleUpperMessage - incrementing counter for HARQ processes " << (unsigned int)currentHarq_ << " --> " << (currentHarq_+1)%harqProcesses_ << endl;
        currentHarq_ = (currentHarq_+1)%harqProcesses_;
    }
    else
    {
        delete pkt;
    }
}

void LteMacUeMode4D2D::macGenerateSchedulingGrant()
{
    /**
     * 1. Packet priority
     * 2. Resource reservation interval
     * 3. Maximum latency
     * 4. Number of subchannels
     * 6. Send message to PHY layer looking for CSRs
     */

    LteMode4SchedulingGrant* mode4Grant = new LteMode4SchedulingGrant("LteMode4Grant");

    mode4Grant -> setSpsPriority(messagePriority);
    mode4Grant -> setPeriod(resourceReservationInterval);
    mode4Grant -> setMaximumLatency(maximumLatency);

    /**
     * Need to pick the number of subchannels for this reservation
     */
    // Select random element from vector
    std::uniform_int_distribution<int> distr(minSubchannel, maxSubchannel);
    int numSubchannels = distr(generator);

    mode4Grant -> setNumberSubchannels(numSubchannels);

    sendLowerMessage(mode4Grant);

    schedulingGrant_ = mode4Grant;

}


