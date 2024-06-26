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

#include "RSUAuctionApp.h"
#include "common.h"
#include "common/LteControlInfo.h"
#include "message/CoinRequest_m.h"
#include "message/CoinAssignment_m.h"
#include "message/CoinDeposit_m.h"
#include "message/CoinDepositSignatureRequest_m.h"
#include "message/CoinDepositSignatureResponse_m.h"
#include "message/CoinSubmission_m.h"

Define_Module(RSUAuctionApp);

// global set to record which cars are assigned coins and have deposited coins
using std::unordered_set;
unordered_set<int> carCoinAssignedSet;
unordered_set<int> carCoinDepositedSet;

void RSUAuctionApp::handleLowerMessage(cMessage* msg)
{
    double currentTime = simTime().dbl();

    if (CoinRequest* req = dynamic_cast<CoinRequest*>(msg)) {
        int vid = req->getVid();
        EV_WARN << "[RSU]: I received a message of CoinRequest from " << vid << endl;

        CoinAssignment* packet = new CoinAssignment();
        packet->setVid(vid);
        // RSU (randomly) chooses auction winners and misbehaving vehicles. They do not receive reputation coins.
        // We simply set a small packet size indicating that it is a simple notification package without coins.
        // Here we set the probability to about 30% because about 30% of lanes are in green light for a typical intersection.
        if (((double) rand() / RAND_MAX) < 0.3) {
            packet->setByteLength(60);
        } else {
            packet->setByteLength(COIN_ASSIGNMENT_BYTE_SIZE);
        }
        std::pair<double,double> latency = cpuModel.getLatency(currentTime, COIN_ASSIGNMENT_LATENCY_MEAN, COIN_ASSIGNMENT_LATENCY_STDDEV);
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
        packet->setControlInfo(lteControlInfo);
        sendDelayedDown(packet,latency.first+latency.second);

        coinDepositStages[vid] = CoinDepositStage::SIGNATURE_REQUESTED;
        carCoinDepositedSet.insert(vid);

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
RSUAuctionApp::~RSUAuctionApp(){
    binder_->unregisterNode(nodeId_);
}
