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

#include "RSUPerceptionApp.h"
#include "common.h"
#include "common/LteControlInfo.h"
#include "message/CoinRequest_m.h"
#include "message/CoinAssignment_m.h"
#include "message/CoinDeposit_m.h"
#include "message/CoinDepositSignatureRequest_m.h"
#include "message/CoinDepositSignatureResponse_m.h"
#include "message/CoinSubmission_m.h"

Define_Module(RSUPerceptionApp);

// global set to record which cars are assigned coins and have deposited coins
using std::unordered_set;
unordered_set<int> carCoinAssignedSet;

void RSUPerceptionApp::handleLowerMessage(cMessage* msg) {
    double currentTime = simTime().dbl();

    if (CoinRequest* req = dynamic_cast<CoinRequest*>(msg)) {
        int vid = req->getVid();
        EV_WARN << "[RSU]: I received a message of CoinRequest from " << vid << endl;

        std::pair<double,double> latency = cpuModel.getLatency(currentTime, COIN_ASSIGNMENT_LATENCY_MEAN, COIN_ASSIGNMENT_LATENCY_STDDEV);
        CoinAssignment* packet = new CoinAssignment();
        packet->setByteLength(COIN_ASSIGNMENT_BYTE_SIZE);
        packet->setVid(vid);
        auto lteControlInfo = new FlowControlInfoNonIp();
        lteControlInfo->setSrcAddr(nodeId_);
        lteControlInfo->setDstAddr(vid);
        lteControlInfo->setDirection(D2D);
        packet->setControlInfo(lteControlInfo);
        sendDelayedDown(packet,latency.first+latency.second);

        coinAssignmentStages[vid] = CoinAssignmentStage::SENT;
        carCoinAssignedSet.insert(vid);

        EV_WARN << "[RSU]: I sent a message of CoinAssignment to " << vid << ". Queue time " << latency.first
                << " Computation time " << latency.second << endl;
    }
}

RSUPerceptionApp::~RSUPerceptionApp(){
    binder_->unregisterNode(nodeId_);
}
