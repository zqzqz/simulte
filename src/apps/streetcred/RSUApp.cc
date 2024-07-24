//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without EV_WARNen the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "RSUApp.h"
#include "common.h"
#include "common/LteControlInfo.h"
#include "message/CoinRequest_m.h"
#include "message/CoinAssignment_m.h"
#include "message/CoinDeposit_m.h"
#include "message/CoinDepositSignatureRequest_m.h"
#include "message/CoinDepositSignatureResponse_m.h"
#include "message/CoinSubmission_m.h"

Define_Module(RSUApp);

void RSUApp::initialize(int stage)
{
    BaseApp::initialize(stage);
    if (stage==inet::INITSTAGE_LOCAL){
        // Register the node with the binder
        // Issue primarily is how do we set the link layer address

        // Get the binder
        binder_ = getBinder();

        // Get our UE
        cModule *ue = getParentModule();

        //Register with the binder
        nodeId_ = binder_->registerNode(ue, UE, 0);

        // Register the nodeId_ with the binder.
        binder_->setMacNodeId(nodeId_, nodeId_);
    } else if (stage==inet::INITSTAGE_APPLICATION_LAYER) {
        EV_WARN << "[RSU] address " << nodeId_ << endl;
    }
}

void RSUApp::handleLowerMessage(cMessage* msg)
{
    double currentTime = simTime().dbl();

    if (CoinRequest* req = dynamic_cast<CoinRequest*>(msg)) {
        int vid = req->getVid();
        EV_WARN << "[RSU]: I received a message of CoinRequest from " << vid << endl;

        CoinAssignment* packet = new CoinAssignment();
        packet->setVid(vid);
        packet->setByteLength(COIN_ASSIGNMENT_BYTE_SIZE);
        std::pair<double,double> latency = cpuModel.getLatency(currentTime, COIN_ASSIGNMENT_LATENCY_MEAN, COIN_ASSIGNMENT_LATENCY_STDDEV);
        auto lteControlInfo = new FlowControlInfoNonIp();
        lteControlInfo->setSrcAddr(nodeId_);
        lteControlInfo->setDstAddr(vid);
        lteControlInfo->setDirection(D2D);
        lteControlInfo->setPriority(priority);
        lteControlInfo->setDuration(duration);
        lteControlInfo->setCreationTime(simTime());
        packet->setControlInfo(lteControlInfo);
        sendDelayedDown(packet,latency.first+latency.second);

        coinAssignmentStages[vid] = CoinAssignmentStage::SENT;

        EV_WARN << "[RSU]: I sent a message of CoinAssignment to " << vid << ". Queue time " << latency.first
                << " Computation time " << latency.second << endl;
    }
    else if (CoinDeposit* req = dynamic_cast<CoinDeposit*>(msg)) {
        int vid = req->getVid();
        EV_WARN << "[RSU] I received a message of CoinDeposit from " << vid << endl;

        std::pair<double,double> latency = cpuModel.getLatency(currentTime, COIN_DEPOSIT_SIGNATURE_REQUEST_LATENCY_MEAN, COIN_DEPOSIT_SIGNATURE_REQUEST_LATENCY_STDDEV);
        CoinDepositSignatureRequest* packet = new CoinDepositSignatureRequest();
        packet->setByteLength(COIN_DEPOSIT_SIGNATURE_REQUEST_BYTE_SIZE);
        packet->setVid(vid);
        auto lteControlInfo = new FlowControlInfoNonIp();
        lteControlInfo->setSrcAddr(nodeId_);
        lteControlInfo->setDstAddr(vid);
        lteControlInfo->setDirection(D2D);
        lteControlInfo->setPriority(priority);
        lteControlInfo->setDuration(duration);
        lteControlInfo->setCreationTime(simTime());
        packet->setControlInfo(lteControlInfo);
        sendDelayedDown(packet,latency.first+latency.second);

        coinDepositStages[vid] = CoinDepositStage::SIGNATURE_REQUESTED;

        EV_WARN << "[RSU]: I sent a message of CoinDepositSignatureRequest to " << vid << ". Queue time " << latency.first
                << " Computation time " << latency.second << endl;
    }
    else if (CoinDepositSignatureResponse* req = dynamic_cast<CoinDepositSignatureResponse*>(msg)) {
        int vid = req->getVid();
        EV_WARN << "[RSU] I received a message of CoinDepositSignatureResponse from " << vid << endl;

        if (coinDepositStages.find(vid) != coinDepositStages.end() && coinDepositStages[vid] == CoinDepositStage::SIGNATURE_REQUESTED) {
            // TODO: The communication to central database.
            coinDepositStages[vid] = CoinDepositStage::SUBMITTED;
            EV_WARN << "[Vehicle " << vid << "]: Coin deposit succeed." << endl;
        }
    }
}

RSUApp::~RSUApp() {
    // TODO Auto-generated destructor stub
    binder_->unregisterNode(nodeId_);
}
