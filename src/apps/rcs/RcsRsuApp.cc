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

#include "RcsRsuApp.h"
#include "common/LteControlInfo.h"
#include "CoinRequest_m.h"
#include "CoinAssignment_m.h"
#include "CoinDeposit_m.h"
#include "CoinDepositSignatureRequest_m.h"
#include "CoinDepositSignatureResponse_m.h"
#include "CoinSubmission_m.h"
#include "package.h"

#include <iostream>

Define_Module(RcsRsuApp);

RcsRsuApp::~RcsRsuApp() {
    // TODO Auto-generated destructor stub
}

void RcsRsuApp::initialize(int stage)
{
    Mode4BaseApp::initialize(stage);
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
        cpuModel.init(1);
//        EV << "[RSU] address " << nodeId_ << endl;
        std::cout << "[RSU] address " << nodeId_ << std::endl;

        // testing for package segmentation
//        CoinAssignment* packet = new CoinAssignment();
//        packet->setByteLength(COIN_ASSIGNMENT_BYTE_SIZE);
//        auto lteControlInfo = new FlowControlInfoNonIp();
//        lteControlInfo->setSrcAddr(nodeId_);
//        lteControlInfo->setDstAddr(nodeId_);
//        lteControlInfo->setDirection(D2D);
//        packet->setControlInfo(lteControlInfo);
//        sendDelayedDown(packet, 0);
//        std::cout << "[RSU] sending dummy package..." << std::endl;
    }
}

void RcsRsuApp::handleSelfMessage(cMessage *msg)
{
    Mode4BaseApp::handleSelfMessage(msg);
}

void RcsRsuApp::handleLowerMessage(cMessage* msg)
{
    Mode4BaseApp::handleLowerMessage(msg);
    double currentTime = simTime().dbl();
    // receive num package
    if (CoinRequest* req = dynamic_cast<CoinRequest*>(msg)) {
//        EV << "[RSU] I received a message of CoinRequest from " << req->getVid() << " at " << currentTime << endl;
        std::cout << "[RSU] I received a message of CoinRequest from " << req->getVid() << " at " << currentTime << std::endl;
        int vid = req->getVid();
        if (coinDepositStages.find(vid) == coinDepositStages.end()) {
            CoinAssignment* packet = new CoinAssignment();
            packet->setByteLength(COIN_ASSIGNMENT_BYTE_SIZE);
            packet->setVid(vid);
            auto lteControlInfo = new FlowControlInfoNonIp();
            lteControlInfo->setSrcAddr(nodeId_);
            lteControlInfo->setDstAddr(vid);
            lteControlInfo->setDirection(D2D);
            packet->setControlInfo(lteControlInfo);
            // package list for loop

            std::pair<double,double> latency = cpuModel.getLatency(currentTime, COIN_ASSIGNMENT_LATENCY_MEAN, COIN_ASSIGNMENT_LATENCY_STDDEV);
            double totalLatency = latency.first + latency.second;
            sendDelayedDown(packet, totalLatency);
//            sendDelayedDown(packet, cpuModel.getLatency(simTime().dbl(), COIN_ASSIGNMENT_LATENCY_MEAN, COIN_ASSIGNMENT_LATENCY_STDDEV));
            coinAssignmentStages[vid] = CoinAssignmentStage::SENT;

//            EV << "[RSU] When processing the CoinRequest message, queuing time = "
//                    << latency.first << " and computation time = " << latency.second << endl;
//            EV << "[RSU] I sent a message of CoinAssignment to " << req->getVid() << " at " << currentTime + totalLatency << endl;
            std::cout << "[RSU] When processing the CoinRequest message from " << req->getVid() << ", total time = "
                    << totalLatency << ", queuing time = "
                    << latency.first << " and computation time = " << latency.second << std::endl;
            std::cout << "[RSU] I sent a message of CoinAssignment to " << req->getVid() << " at " << currentTime + totalLatency << endl;
        }
    } else if (CoinDeposit* req = dynamic_cast<CoinDeposit*>(msg)) {
//        EV << "[RSU] I received a message of CoinDeposit from " << req->getVid() << " at " << simTime().dbl() << std::endl;
        std::cout << "[RSU] I received a message of CoinDeposit from " << req->getVid() << " at " << simTime().dbl() << std::endl;
        int vid = req->getVid();
        if (coinDepositStages.find(vid) == coinDepositStages.end()) {
            CoinDepositSignatureRequest* packet = new CoinDepositSignatureRequest();
            packet->setByteLength(COIN_DEPOSIT_SIGNATURE_REQUEST_BYTE_SIZE);
            packet->setVid(vid);
            auto lteControlInfo = new FlowControlInfoNonIp();
            lteControlInfo->setSrcAddr(nodeId_);
            lteControlInfo->setDstAddr(vid);
            lteControlInfo->setDirection(D2D);
            packet->setControlInfo(lteControlInfo);

            std::pair<double,double> latency = cpuModel.getLatency(currentTime, COIN_DEPOSIT_SIGNATURE_REQUEST_LATENCY_MEAN, COIN_DEPOSIT_SIGNATURE_REQUEST_LATENCY_STDDEV);
            double totalLatency = latency.first + latency.second;
            sendDelayedDown(packet, totalLatency);
//            sendDelayedDown(packet, cpuModel.getLatency(simTime().dbl(), COIN_DEPOSIT_SIGNATURE_REQUEST_LATENCY_MEAN, COIN_DEPOSIT_SIGNATURE_REQUEST_LATENCY_STDDEV));
            coinDepositStages[vid] = CoinDepositStage::SIGNATURE_REQUESTED;

//            EV << "[RSU] When processing the CoinDeposit message, queuing time = "
//                    << latency.first << " and computation time = " << latency.second << endl;
//            EV << "[RSU] I sent a message of CoinDepositSignatureRequest to " << req->getVid() << " at " << currentTime + totalLatency << endl;
            std::cout << "[RSU] When processing the CoinDeposit message from " << req->getVid() << ", total time = "
                    << totalLatency << ", queuing time = "
                    << latency.first << " and computation time = " << latency.second << std::endl;
            std::cout << "[RSU] I sent a message of CoinDepositSignatureRequest to " << req->getVid() << " at " << currentTime + totalLatency << std::endl;
        }
    } else if (CoinDepositSignatureResponse* req = dynamic_cast<CoinDepositSignatureResponse*>(msg)) {
//        EV << "[RSU] I received a message of CoinDepositSignatureResponse from " << req->getVid() << " at " << currentTime << endl;
        std::cout << "[RSU] I received a message of CoinDepositSignatureResponse from " << req->getVid() << " at " << currentTime << std::endl;
        int vid = req->getVid();
        if (coinDepositStages.find(vid) != coinDepositStages.end() && coinDepositStages[vid] == CoinDepositStage::SIGNATURE_REQUESTED) {
            // TODO: The communication to central database.
            coinDepositStages[vid] = CoinDepositStage::SUBMITTED;
        }
    }
}

