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
#include "stack/mac/layer/LteMacUeMode4D2D.h"
#include "stack/mac/scheduler/LteSchedulerUeUl.h"
#include "stack/phy/packet/SpsCandidateResources.h"
#include "stack/phy/packet/cbr_m.h"
#include "stack/phy/layer/Subchannel.h"
#include "stack/mac/amc/AmcPilotD2D.h"
#include "common/LteCommon.h"
#include "stack/phy/layer/LtePhyBase.h"
#include "inet/networklayer/common/InterfaceEntry.h"
#include "inet/common/ModuleAccess.h"
#include "inet/networklayer/ipv4/IPv4InterfaceData.h"
#include "stack/mac/amc/LteMcs.h"
#include <random>

Define_Module(LteMacUeMode4D2D);

LteMacUeMode4D2D::LteMacUeMode4D2D() :
    LteMacUeRealisticD2D()
{
}

LteMacUeMode4D2D::~LteMacUeMode4D2D()
{
}

void LteMacUeMode4D2D::initialize(int stage)
{
    if (stage !=INITSTAGE_NETWORK_LAYER_3)
    LteMacUeRealisticD2D::initialize(stage);

    if (stage == inet::INITSTAGE_LOCAL)
    {
        generator_.seed(rand_device_());
        parseUeTxConfig(par("txConfig").xmlValue());
        parseCbrTxConfig(par("txConfig").xmlValue());
        parseRriConfig(par("txConfig").xmlValue());
        resourceReservationInterval_ = validResourceReservationIntervals_.at(0);
        maximumLatency_ = par("maximumLatency");
        subchannelSize_ = par("subchannelSize");
        numSubchannels_ = par("numSubchannels");
        probResourceKeep_ = par("probResourceKeep");
        usePreconfiguredTxParams_ = par("usePreconfiguredTxParams");
        maximumLatency_ = par("maximumLatency");
        resourceReservationInterval_ = par("resourceReservationInterval");
        reselectAfter_ = par("reselectAfter");
        useCBR_ = par("useCBR");
        maximumCapacity_ = 0;
        cbr_=0;
        currentCw_=0;
        missedTransmissions_=0;
    }
    else if (stage == INITSTAGE_NETWORK_LAYER_3)
    {
        // TODO: When deploying a UE add the deployer here, make it so deployer can exist on the UE as well.
        deployer_ = getDeployer();
        numAntennas_ = getNumAntennas();
        mcsScaleD2D_ = deployer_->getMcsScaleUl();
        d2dMcsTable_.rescale(mcsScaleD2D_);

        if (usePreconfiguredTxParams_)
        {
            preconfiguredTxParams_ = getPreconfiguredTxParams();
        }

        // LTE UE Section
        nodeId_ = getAncestorPar("macNodeId");

        /* Insert UeInfo in the Binder */
        UeInfo* info = new UeInfo();
        info->id = nodeId_;            // local mac ID
        info->cellId = cellId_;        // cell ID
        info->init = false;            // flag for phy initialization
        info->ue = this->getParentModule()->getParentModule();  // reference to the UE module

        // Get the Physical Channel reference of the node
        info->phy = check_and_cast<LtePhyBase*>(info->ue->getSubmodule("lteNic")->getSubmodule("phy"));

        binder_->addUeInfo(info);

        // find interface entry and use its address
        IInterfaceTable *interfaceTable = getModuleFromPar<IInterfaceTable>(par("interfaceTableModule"), this);
        // TODO: how do we find the LTE interface?
        InterfaceEntry * interfaceEntry = interfaceTable->getInterfaceByName("wlan");

        IPv4InterfaceData* ipv4if = interfaceEntry->ipv4Data();
        if(ipv4if == NULL)
            throw new cRuntimeError("no IPv4 interface data - cannot bind node %i", nodeId_);
        binder_->setMacNodeId(ipv4if->getIPAddress(), nodeId_);
    }
}

void LteMacUeMode4D2D::parseUeTxConfig(cXMLElement* xmlConfig)
{
    if (xmlConfig == 0)
    throw cRuntimeError("No sidelink configuration file specified");

    // Get channel Model field which contains parameters fields
    cXMLElementList ueTxConfig = xmlConfig->getElementsByTagName("userEquipment-txParameters");

    if (ueTxConfig.empty())
        throw cRuntimeError("No userEquipment-txParameters configuration found in configuration file");

    if (ueTxConfig.size() > 1)
        throw cRuntimeError("More than one userEquipment-txParameters configuration found in configuration file.");

    cXMLElement* ueTxConfigData = ueTxConfig.front();

    ParameterMap params;
    getParametersFromXML(ueTxConfigData, params);

    //get lambda max threshold
    ParameterMap::iterator it = params.find("minMCS-PSSCH");
    if (it != params.end())
    {
        minMCSPSSCH_ = it->second;
    }
    else
        minMCSPSSCH_ = par("minMCSPSSCH");
    it = params.find("maxMCS-PSSCH");
    if (it != params.end())
    {
        maxMCSPSSCH_ = it->second;
    }
    else
        maxMCSPSSCH_ = par("minMCSPSSCH");
    it = params.find("minSubchannel-NumberPSSCH");
    if (it != params.end())
    {
        minSubchannelNumberPSSCH_ = it->second;
    }
    else
        minSubchannelNumberPSSCH_ = par("minSubchannelNumberPSSCH");
    it = params.find("maxSubchannel-NumberPSSCH");
    if (it != params.end())
    {
        maxSubchannelNumberPSSCH_ = it->second;
    }
    else
        maxSubchannelNumberPSSCH_ = par("maxSubchannelNumberPSSCH");
    it = params.find("allowedRetxNumberPSSCH");
    if (it != params.end())
    {
        allowedRetxNumberPSSCH_ = it->second;
    }
    else
        allowedRetxNumberPSSCH_ = par("allowedRetxNumberPSSCH");
}

void LteMacUeMode4D2D::parseCbrTxConfig(cXMLElement* xmlConfig)
{
    if (xmlConfig == 0)
    throw cRuntimeError("No cbr configuration specified");

    // Get channel Model field which contains parameters fields
    cXMLElementList cbrTxConfig = xmlConfig->getElementsByTagName("Sl-CBR-CommonTxConfigList");

    if (cbrTxConfig.empty())
        throw cRuntimeError("No Sl-CBR-CommonTxConfigList found in configuration file");

    cXMLElement* cbrTxConfigData = cbrTxConfig.front();

    ParameterMap params;
    getParametersFromXML(cbrTxConfigData, params);

    //get lambda max threshold
    ParameterMap::iterator it = params.find("default-cbr-ConfigIndex");
    if (it != params.end())
    {
        defaultCbrIndex_ = it->second;
    }

    cXMLElementList cbrLevelConfigs = xmlConfig->getElementsByTagName("cbr-Levels-Config");

    if (cbrLevelConfigs.empty())
        throw cRuntimeError("No cbr-Levels-Config found in configuration file");

    cXMLElementList::iterator xmlIt;
    for(xmlIt = cbrLevelConfigs.begin(); xmlIt != cbrLevelConfigs.end(); xmlIt++)
    {
        std::map<std::string, int> cbrLevelsMap;
        ParameterMap cbrLevelsParams;
        getParametersFromXML((*xmlIt), cbrLevelsParams);
        it = cbrLevelsParams.find("cbr-lower");
        if (it != cbrLevelsParams.end())
        {
            cbrLevelsMap.insert(pair<string, int>("cbr-lower",  it->second));
        }
        it = cbrLevelsParams.find("cbr-upper");
        if (it != cbrLevelsParams.end())
        {
            cbrLevelsMap.insert(pair<string, int>("cbr-upper",  it->second));
        }
        it = cbrLevelsParams.find("cbr-lower");
        if (it != cbrLevelsParams.end())
        {
            cbrLevelsMap.insert(pair<string, int>("cbr-PSSCH-TxConfig-Index",  it->second));
        }
    }

    cXMLElementList cbrTxConfigs = xmlConfig->getElementsByTagName("cbr-PSSCH-TxConfig");

    if (cbrTxConfigs.empty())
        throw cRuntimeError("No CBR-TxConfig found in configuration file");

    cXMLElementList cbrTxParams = xmlConfig->getElementsByTagName("txParameters");

    for(xmlIt = cbrTxParams.begin(); xmlIt != cbrTxParams.end(); xmlIt++)
    {
        std::map<std::string, int> cbrMap;
        ParameterMap cbrParams;
        getParametersFromXML((*xmlIt), cbrParams);
        it = cbrParams.find("minMCS-PSSCH");
        if (it != cbrParams.end())
        {
            cbrMap.insert(pair<string, int>("minMCSPSSCH",  it->second));
        }
        else
            cbrMap.insert(pair<string, int>("minMCSPSSCH",  par("minMCSPSSCH")));
        it = cbrParams.find("maxMCS-PSSCH");
        if (it != cbrParams.end())
        {
            cbrMap.insert(pair<string, int>("maxMCSPSSCH",  it->second));
        }
        else
            cbrMap.insert(pair<string, int>("maxMCSPSSCH",  par("maxMCSPSSCH")));
        it = cbrParams.find("minSubchannel-NumberPSSCH");
        if (it != cbrParams.end())
        {
            cbrMap.insert(pair<string, int>("minSubchannelNumberPSSCH",  it->second));
        }
        else
            cbrMap.insert(pair<string, int>("minSubchannelNumberPSSCH",  par("minSubchannelNumberPSSCH")));
        it = cbrParams.find("maxSubchannel-NumberPSSCH");
        if (it != cbrParams.end())
        {
            cbrMap.insert(pair<string, int>("maxSubchannelNumberPSSCH",  it->second));
        }
        else
            cbrMap.insert(pair<string, int>("maxSubchannelNumberPSSCH",  par("maxSubchannelNumberPSSCH")));
        it = cbrParams.find("allowedRetxNumberPSSCH");
        if (it != cbrParams.end())
        {
            cbrMap.insert(pair<string, int>("allowedRetxNumberPSSCH",  it->second));
        }
        else
            cbrMap.insert(pair<string, int>("allowedRetxNumberPSSCH",  par("allowedRetxNumberPSSCH")));
        it = cbrParams.find("cr-Limit");
        if (it != cbrParams.end())
        {
            cbrMap.insert(pair<string, int>("cr-Limit",  it->second));
        }
        cbrPSSCHTxConfigList_.push_back(cbrMap);
    }
}

void LteMacUeMode4D2D::parseRriConfig(cXMLElement* xmlConfig)
{
    if (xmlConfig == 0)
    throw cRuntimeError("No cbr configuration specified");

    // Get channel Model field which contains parameters fields
    cXMLElementList rriConfig = xmlConfig->getElementsByTagName("RestrictResourceReservationPeriodList");

    if (rriConfig.empty())
        throw cRuntimeError("No RestrictResourceReservationPeriodList found in configuration file");

    cXMLElementList rriConfigs = xmlConfig->getElementsByTagName("RestrictResourceReservationPeriod");

    if (rriConfigs.empty())
        throw cRuntimeError("No RestrictResourceReservationPeriods found in configuration file");

    cXMLElementList::iterator xmlIt;
    for(xmlIt = rriConfigs.begin(); xmlIt != rriConfigs.end(); xmlIt++)
    {
        ParameterMap rriParams;
        getParametersFromXML((*xmlIt), rriParams);
        ParameterMap::iterator it = rriParams.find("rri");
        if (it != rriParams.end())
        {
            validResourceReservationIntervals_.push_back(it->second);
        }
    }
}

int LteMacUeMode4D2D::getNumAntennas()
{
    /* Get number of antennas: +1 is for MACRO */
    return deployer_->getNumRus() + 1;
}

void LteMacUeMode4D2D::macPduMake()
{
    int64 size = 0;

    macPduList_.clear();

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

            // First translate MCS to CQI
            LteMode4SchedulingGrant* mode4Grant = check_and_cast<LteMode4SchedulingGrant*>(schedulingGrant_);

            if (usePreconfiguredTxParams_)
            {
                UserTxParams* userTxParams = preconfiguredTxParams_;
                uinfo->setUserTxParams(userTxParams->dup());
                mode4Grant->setUserTxParams(userTxParams->dup());
            }
            else
                uinfo->setUserTxParams(mode4Grant->getUserTxParams()->dup());

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

        EV << "LteMacUeRealisticD2D: pduMaker created PDU: " << macPkt->info() << endl;

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

UserTxParams* LteMacUeMode4D2D::getPreconfiguredTxParams()
{
    UserTxParams* txParams = new UserTxParams();

    // default parameters for D2D
    txParams->isSet() = true;
    txParams->writeTxMode(SINGLE_ANTENNA_PORT0);
    Rank ri = 1;                                              // rank for TxD is one
    txParams->writeRank(ri);
    txParams->writePmi(intuniform(1, pow(ri, (double) 2)));   // taken from LteFeedbackComputationRealistic::computeFeedback

    BandSet b;
    for (Band i = 0; i < deployer_->getNumBands(); ++i) b.insert(i);
    txParams->writeBands(b);

    RemoteSet antennas;
    antennas.insert(MACRO);
    txParams->writeAntennas(antennas);

    return txParams;
}

void LteMacUeMode4D2D::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage())
    {
        LteMacUeRealisticD2D::handleMessage(msg);
        return;
    }
    if (strcmp(msg->getName(), "GRANTBREAK") == 0)
    {
        // TODO: Signal that grant is broken due to lack of capacity.

        delete schedulingGrant_;

        schedulingGrant_ = NULL;

        // Need to regenerate our grant
        macGenerateSchedulingGrant();

        return;
    }

    cPacket* pkt = check_and_cast<cPacket *>(msg);
    cGate* incoming = pkt->getArrivalGate();

    if (incoming == down_[IN])
    {
        if (strcmp(pkt->getName(), "CSRs") == 0)
        {
            EV << "LteMacUeMode4D2D::handleMessage - Received packet " << pkt->getName() <<
            " from port " << pkt->getArrivalGate()->getName() << endl;

            // message from PHY_to_MAC gate (from lower layer)
            emit(receivedPacketFromLowerLayer, pkt);

            // call handler
            macHandleSps(pkt);

            return;
        }
        if (strcmp(pkt->getName(), "CBR") == 0)
        {
            Cbr* cbrPkt = check_and_cast<Cbr*>(pkt);
            cbr_ = cbrPkt->getCbr();
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

    EV << NOW << "LteMacUeMode4D2D::handleSelfMessage " << nodeId_ << " - HARQ process " << (unsigned int)currentHarq_ << endl;
    // updating current HARQ process for next TTI

    //unsigned char currentHarq = currentHarq_;

    // no grant available - if user has backlogged data, it will trigger scheduling request
    // no harq counter is updated since no transmission is sent.

    bool generateNewSchedulingGrant = false;

    LteMode4SchedulingGrant* mode4Grant = dynamic_cast<LteMode4SchedulingGrant*>(schedulingGrant_);

    if (mode4Grant == NULL)
    {
        EV << NOW << " LteMacUeMode4D2D::handleSelfMessage " << nodeId_ << " NO configured grant" << endl;

        // scheduling grant needs to be generated at end of process
        generateNewSchedulingGrant = true;
    }
    else if (mode4Grant->getPeriodic() && mode4Grant->getStartTime() <= NOW)
    {
        // Periodic checks
        if(--expirationCounter_ == mode4Grant->getPeriod())
        {
            // Gotten to the point of the final tranmission must determine if we reselect or not.
            std::uniform_real_distribution<float> floatdist(0, 1);
            float randomReReserve = floatdist(generator_);
            if (randomReReserve > probResourceKeep_)
            {
                std::uniform_int_distribution<int> range(5, 15);
                int expiration = range(generator_);
                mode4Grant -> setExpiration(expiration);
                mode4Grant -> setFirstTransmission(true);
                expirationCounter_ = expiration * mode4Grant->getPeriod();
            }
            else
            {
                mode4Grant->setExpiration(0);
            }
        }
        // Periodic checks
        else if(expirationCounter_ == 0)
        {
            // Periodic grant is expired
            // This is the last message to send
            generateNewSchedulingGrant = true;
        }
        if (--periodCounter_>0 && !mode4Grant->getFirstTransmission())
        {
            return;
        }
        else if (expirationCounter_ != 0)
        {
            // resetting grant period
            periodCounter_=mode4Grant->getPeriod();
            // this is periodic grant TTI - continue with frame sending
        }
        if (mode4Grant->getFirstTransmission())
        {
            mode4Grant->setFirstTransmission(false);
        }
    }
    bool requestSdu = false;
    if (mode4Grant!=NULL && mode4Grant->getStartTime() <= NOW) // if a grant is configured
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

        bool trans = false;

        HarqTxBuffers::iterator it2;
        LteHarqBufferTx * currHarq;
        for(it2 = harqTxBuffers_.begin(); it2 != harqTxBuffers_.end(); it2++)
        {
            EV << "\t Looking for retx in acid " << (unsigned int)currentHarq_ << endl;
            currHarq = it2->second;

            // check if the current process has unit ready for retx
            UnitList pduId = it2->second->firstAvailable();

            // Always 0
//            unsigned char currentAcid = harqStatus_.at(nodeId);
//            // search for an empty unit within current harq process
//            UnitList txList = txBuf->getEmptyUnits(currentHarq_);
//            EV << "LteMacUeRealisticD2D::macPduMake - [Used Acid=" << (unsigned int)txList.first << "] , [curr=" << (unsigned int)currentHarq_ << "]" << endl;
            // TODO: Fix issue with 0 length pduLength
            // Possibly due to there being no ack or nack and so a message is not removed when sent. Might need some sort of harq buffer for mode4 most likely.
            std::list<Codeword>::iterator cw;
            for (cw=pduId.second.begin(); cw!=pduId.second.end(); cw++)
            {
                int pduLength = currHarq->pduLength((unsigned char)currentHarq_, *cw);
                if (pduLength < maximumCapacity_)
                {
                    trans = true;

                    int cbrIndex = defaultCbrIndex_;
                    if (useCBR_)
                    {
                        std::vector<std::map<string, int>>::iterator it;
                        for (it = cbrLevels_.begin(); it!=cbrLevels_.end(); it++)
                        {
                            if (cbr_<(*it).at("cbr-upper"))
                            {
                                cbrIndex = (*it).at("cbr-PSSCH-TxConfig-Index");
                                break;
                            }
                        }
                    }

                    int minMCS;
                    int maxMCS;
                    if (maxMCSPSSCH_ < cbrPSSCHTxConfigList_.at(cbrIndex).at("minMCSPSSCH") || cbrPSSCHTxConfigList_.at(cbrIndex).at("maxMCSPSSCH") < minMCSPSSCH_)
                    {
                        // No overlap therefore I will use the cbr values (this is left to the UE).
                        minMCS = cbrPSSCHTxConfigList_.at(cbrIndex).at("minMCSPSSCH");
                        maxMCS = cbrPSSCHTxConfigList_.at(cbrIndex).at("maxMCSPSSCH");
                    }
                    else
                    {
                        minMCS = max(minMCSPSSCH_, cbrPSSCHTxConfigList_.at(cbrIndex).at("minMCSPSSCH"));
                        maxMCS = min(maxMCSPSSCH_, cbrPSSCHTxConfigList_.at(cbrIndex).at("maxMCSPSSCH"));
                    }
                    bool foundValidMCS = false;
                    int totalGrantedBlocks = mode4Grant->getTotalGrantedBlocks();
                    for (int mcs=minMCS; mcs < maxMCS; mcs++)
                    {
                        int mcsCapacity = 0;

                        LteMod mod = _QPSK;
                        if (maxMCSPSSCH_ > 9 && maxMCSPSSCH_ < 17)
                        {
                            mod = _16QAM;
                        }
                        else if (maxMCSPSSCH_ > 16 && maxMCSPSSCH_ < 29 )
                        {
                            mod = _64QAM;
                        }

                        unsigned int i = (mod == _QPSK ? 0 : (mod == _16QAM ? 9 : (mod == _64QAM ? 15 : 0)));

                        const unsigned int* tbsVect = itbs2tbs(mod, SINGLE_ANTENNA_PORT0, 1, maxMCSPSSCH_ - i);
                        mcsCapacity = tbsVect[totalGrantedBlocks-1];

                        if (mcsCapacity > pduLength)
                        {
                            foundValidMCS = true;
                            mode4Grant->setMcs(mcs);
                            mode4Grant->setGrantedCwBytes(*cw, mcsCapacity);

                            LteMode4SchedulingGrant* phyGrant = mode4Grant->dup();

                            UserControlInfo* uinfo = new UserControlInfo();
                            uinfo->setSourceId(getMacNodeId());
                            uinfo->setDestId(getMacNodeId());
                            uinfo->setFrameType(GRANTPKT);

                            phyGrant->setControlInfo(uinfo);

                            sendLowerPackets(phyGrant);

//                            currHarq->markSelected(pduId,mode4Grant->getUserTxParams()->getLayers().size());

                            // Message that triggers flushing of Tx H-ARQ buffers for all users
                            // This way, flushing is performed after the (possible) reception of new MAC PDUs
                            cMessage* flushHarqMsg = new cMessage("flushHarqMsg");
                            flushHarqMsg->setSchedulingPriority(1);        // after other messages
                            scheduleAt(NOW, flushHarqMsg);

                            missedTransmissions_ = 0;

                            break;
                        }
                    }
                    if (!foundValidMCS)
                    {
                        // Never found an MCS to satisfy the requirements of the message must regenerate grant
                        delete schedulingGrant_;
                        schedulingGrant_ = NULL;
                        generateNewSchedulingGrant = true;
                    }
                    if (!mode4Grant->getUserTxParams())
                    {
                        //allow breakpoint
                        mode4Grant->setUserTxParams(preconfiguredTxParams_);
                    }
                }
            }
//            if (noPdu)
//            {
//                // Break the grant, need to emit that the grant is broken
//                delete schedulingGrant_;
//                schedulingGrant_ = NULL;
//                generateNewSchedulingGrant = true;
//            }
        }
        // if no retx is needed, proceed with normal scheduling
        if(!trans)
        {
            ++missedTransmissions_;
            if (missedTransmissions_ >= reselectAfter_)
            {
                delete schedulingGrant_;
                schedulingGrant_ = NULL;
                generateNewSchedulingGrant=true;
                missedTransmissions_ = 0;
            }
            scheduleList_ = lcgScheduler_->schedule();
            bool sent = macSduRequest();

            if (!scheduleList_->empty())
            {
                // no connection scheduled
                macPduMake();
            }
            requestSdu = sent; // return a bool
        }
    }
    if (mode4Grant == NULL || generateNewSchedulingGrant)
    {
        macGenerateSchedulingGrant();
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

    if (!requestSdu)
    {
        // update current harq process id
        currentHarq_ = (currentHarq_+1) % harqProcesses_;
    }

    EV << "--- END UE MAIN LOOP ---" << endl;
}

void LteMacUeMode4D2D::macHandleSps(cPacket* pkt)
{
    /**   This is where we add the subchannels to the actual scheduling grant, so a few things
     * 1. Need to ensure in the self message part that if at any point we have a scheduling grant without assigned subchannels, we have to wait
     * 2. Need to pick at random from the SPS list of CSRs
     * 3. Assign the CR
     * 4. return
     */
    SpsCandidateResources* candidatesPacket = check_and_cast<SpsCandidateResources *>(pkt);
    std::vector<std::vector<Subchannel*>> CSRs = candidatesPacket->getCSRs();

    // Select random element from vector
    std::uniform_int_distribution<int> distr(0, CSRs.size()-1);
    int index = distr(generator_);

    std::vector<Subchannel*> selectedCR = CSRs[index];
    // Gives us the time at which we will send the subframe.
    simtime_t selectedStartTime = NOW + TTI * selectedCR[0]->getSubframeIndex();

    std::vector<Subchannel*>::iterator it;
    std::vector<Band> grantedBands;
    for (it=selectedCR.begin(); it!=selectedCR.end();it++)
    {
        std::vector<Band> subchannelBands = (*it)->getOccupiedBands();
        grantedBands.insert(grantedBands.end(), subchannelBands.begin(), subchannelBands.end());
    }

    RbMap grantedBlocks;
    std::vector<Band>::iterator jt;
    int totalGrantedBlocks = 0;
    for (jt=grantedBands.begin(); jt!=grantedBands.end(); jt++)
    {
        // For each band assign block on the MACRO antenna
        // TODO: think this over at some point.
        grantedBlocks[MACRO][*jt] = 1;
        ++totalGrantedBlocks;
    }

    LteMode4SchedulingGrant* mode4Grant = check_and_cast<LteMode4SchedulingGrant*>(schedulingGrant_);

    mode4Grant->setStartTime(selectedStartTime);
    mode4Grant->setPeriodic(true);
    mode4Grant->setGrantedBlocks(grantedBlocks);
    mode4Grant->setTotalGrantedBlocks(totalGrantedBlocks);
    mode4Grant->setDirection(D2D_MULTI);
    mode4Grant->setCodewords(2);

    mode4Grant->setMcs(maxMCSPSSCH_);

    LteMod mod = _QPSK;
    if (maxMCSPSSCH_ > 9 && maxMCSPSSCH_ < 17)
    {
        mod = _16QAM;
    }
    else if (maxMCSPSSCH_ > 16 && maxMCSPSSCH_ < 29 )
    {
        mod = _64QAM;
    }

    unsigned int i = (mod == _QPSK ? 0 : (mod == _16QAM ? 9 : (mod == _64QAM ? 15 : 0)));

    const unsigned int* tbsVect = itbs2tbs(mod, SINGLE_ANTENNA_PORT0, 1, maxMCSPSSCH_ - i);
    maximumCapacity_ = tbsVect[totalGrantedBlocks-1];
    mode4Grant->setGrantedCwBytes(currentCw_, maximumCapacity_);
    // Simply flips the codeword.
    currentCw_ = MAX_CODEWORDS - currentCw_;


    // Based on restrictResourceReservation interval But will be between 1 and 15
    // Again technically this needs to reconfigurable as well. But all of that needs to come in through ini and such.
    std::uniform_int_distribution<int> range(5, 15);
    int resourceReselectionCounter = range(generator_);

    mode4Grant -> setExpiration(resourceReselectionCounter);

    periodCounter_=mode4Grant->getPeriod();
    expirationCounter_=mode4Grant->getExpiration() * periodCounter_;

    // TODO: Setup for HARQ retransmission, if it can't be satisfied then selection must occur again.
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

    // Priority is the most difficult part to figure out, for the moment I will assign it as a fixed value
    // TODO: Message Priority
    mode4Grant -> setSpsPriority(4);
    mode4Grant -> setPeriod(resourceReservationInterval_ * 100);

    // TODO: Maximum Latency is also an "Application Layer" specified parameter
    mode4Grant -> setMaximumLatency(maximumLatency_);

    int cbrIndex = defaultCbrIndex_;
    if (useCBR_)
    {
        std::vector<std::map<string, int>>::iterator it;
        for (it = cbrLevels_.begin(); it!=cbrLevels_.end(); it++)
        {
            if (cbr_<(*it).at("cbr-upper"))
            {
                cbrIndex = (*it).at("cbr-PSSCH-TxConfig-Index");
                break;
            }
        }
    }

    allowedRetxNumberPSSCH_ = min(cbrPSSCHTxConfigList_.at(cbrIndex).at("allowedRetxNumberPSSCH"), allowedRetxNumberPSSCH_);

    /**
     * Need to pick the number of subchannels for this reservation
     */

    int minSubchannelNumberPSSCH;
    int maxSubchannelNumberPSSCH;
    if (maxMCSPSSCH_ < cbrPSSCHTxConfigList_.at(cbrIndex).at("minMCSPSSCH") || cbrPSSCHTxConfigList_.at(cbrIndex).at("maxMCSPSSCH") < minMCSPSSCH_)
    {
        // No overlap therefore I will use the cbr values (this is left to the UE, the opposite approach is also entirely valid).
        minSubchannelNumberPSSCH = cbrPSSCHTxConfigList_.at(cbrIndex).at("minSubchannel-NumberPSSCH");
        maxSubchannelNumberPSSCH = cbrPSSCHTxConfigList_.at(cbrIndex).at("maxSubchannel-NumberPSSCH");
    }
    else
    {
        minSubchannelNumberPSSCH = max(minSubchannelNumberPSSCH_, cbrPSSCHTxConfigList_.at(cbrIndex).at("minSubchannelNumberPSSCH"));
        maxSubchannelNumberPSSCH = min(maxSubchannelNumberPSSCH_, cbrPSSCHTxConfigList_.at(cbrIndex).at("maxSubchannelNumberPSSCH"));
    }
    // Selecting the number of subchannel at random as there is no explanation as to the logic behind selecting the resources in the range unlike when selecting MCS.
    std::uniform_int_distribution<> distr(minSubchannelNumberPSSCH, maxSubchannelNumberPSSCH);
    int numSubchannels = distr(generator_);

    mode4Grant -> setNumberSubchannels(numSubchannels);

    LteMode4SchedulingGrant* phyGrant = mode4Grant->dup();

    UserControlInfo* uinfo = new UserControlInfo();
    uinfo->setSourceId(getMacNodeId());
    uinfo->setDestId(getMacNodeId());
    uinfo->setFrameType(GRANTPKT);

    phyGrant->setControlInfo(uinfo);

    sendLowerPackets(phyGrant);

    schedulingGrant_ = mode4Grant;
}

void LteMacUeMode4D2D::flushHarqBuffers()
{
    // send the selected units to lower layers
    // First make sure packets are sent down
    // HARQ retrans needs to be taken into account
    // Maintain unit list maybe and that causes retrans?
    // But purge them once all messages sent.

    HarqTxBuffers::iterator it2;
    for(it2 = harqTxBuffers_.begin(); it2 != harqTxBuffers_.end(); it2++)
    {
        it2->second->sendSelectedDown();
    }

    // deleting non-periodic grant
    if (schedulingGrant_ != NULL && !schedulingGrant_->getPeriodic())
    {
        delete schedulingGrant_;
        schedulingGrant_=NULL;
    }
}


