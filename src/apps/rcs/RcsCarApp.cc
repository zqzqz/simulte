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

#include "RcsCarApp.h"
#include "veins/base/utils/FindModule.h"
#include "common/LteControlInfo.h"
#include "veins_inet/VeinsInetMobility.h"
#include "CoinRequest_m.h"
#include "CoinAssignment_m.h"
#include "CoinDeposit_m.h"
#include "CoinDepositSignatureRequest_m.h"
#include "CoinDepositSignatureResponse_m.h"
#include "CoinSubmission_m.h"

#include <iostream>

Define_Module(RcsCarApp);

RcsCarApp::~RcsCarApp() {
    // TODO Auto-generated destructor stub
}

void RcsCarApp::initialize(int stage)
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
        findHost()->subscribe(veins::VeinsInetMobility::mobilityStateChangedSignal, this);
    } else if (stage==inet::INITSTAGE_APPLICATION_LAYER) {
        cpuModel.init(1);
        coinAssignmentStage = CoinAssignmentStage::INIT;
        coinDepositStage = CoinDepositStage::INIT;
//        EV << "[Vehicle " << nodeId_ << "]: Initialized" << endl;
        std::cout << "[Vehicle " << nodeId_ << "]: Initialized at time " << simTime().dbl() << std::endl;
    }
}

void RcsCarApp::handleSelfMessage(cMessage *msg)
{
    Mode4BaseApp::handleSelfMessage(msg);
}

void RcsCarApp::handleLowerMessage(cMessage* msg)
{
    Mode4BaseApp::handleLowerMessage(msg);
    double currentTime = simTime().dbl();
    if (CoinAssignment* req = dynamic_cast<CoinAssignment*>(msg)) {
        int vid = req->getVid();
        if (vid == nodeId_) {
//            EV << "[Vehicle " << nodeId_ << "]: I received a message of CoinAssignment at " << currentTime << endl;
//            EV << "[Vehicle " << nodeId_ << "]: Time waiting for RSU response = " << currentTime - CoinRequestTime << endl;
            std::cout << "[Vehicle " << nodeId_ << "]: I received a message of CoinAssignment at time " << currentTime << std::endl;
            std::cout << "[Vehicle " << nodeId_ << "]: Time waiting for RSU response = " << currentTime - CoinRequestTime << std::endl;
            coinAssignmentStage = CoinAssignmentStage::FINISHED;
        }
    } else if (CoinDepositSignatureRequest* req = dynamic_cast<CoinDepositSignatureRequest*>(msg)) {
        int vid = req->getVid();
        if (vid == nodeId_) {
//            EV << "[Vehicle " << nodeId_ << "]: I received a message of CoinDepositSignatureRequest at time " << currentTime << endl;
//            EV << "[Vehicle " << nodeId_ << "]: Time waiting for RSU response = " << currentTime - CoinDepositTime << endl;
            std::cout << "[Vehicle " << nodeId_ << "]: I received a message of CoinDepositSignatureRequest at time " << currentTime << std::endl;
            std::cout << "[Vehicle " << nodeId_ << "]: Time waiting for RSU response = " << currentTime - CoinDepositTime << std::endl;

            CoinDepositSignatureResponse* packet = new CoinDepositSignatureResponse();
            packet->setByteLength(COIN_DEPOSIT_SIGNATURE_RESPONSE_BYTE_SIZE);
            packet->setVid(nodeId_);
            auto lteControlInfo = new FlowControlInfoNonIp();
            lteControlInfo->setSrcAddr(nodeId_);
            lteControlInfo->setDstAddr(RSU_ADDR);
            lteControlInfo->setDirection(D2D);
            packet->setControlInfo(lteControlInfo);

            std::pair<double,double> latency = cpuModel.getLatency(currentTime, COIN_DEPOSIT_SIGNATURE_RESPONSE_LATENCY_MEAN, COIN_DEPOSIT_SIGNATURE_RESPONSE_LATENCY_STDDEV);
            double totalLatency = latency.first + latency.second;
            sendDelayedDown(packet,totalLatency);
//            sendDelayedDown(packet,cpuModel.getLatency(simTime().dbl(), COIN_DEPOSIT_SIGNATURE_RESPONSE_LATENCY_MEAN, COIN_DEPOSIT_SIGNATURE_RESPONSE_LATENCY_STDDEV));

            coinDepositStage = CoinDepositStage::SIGNATURE_SENT;
//            EV << "[Vehicle " << nodeId_ << "] When processing the CoinDepositSignatureRequest message, queuing time = "
//                    << latency.first << " and computation time = " << latency.second << endl;
//            EV << "[Vehicle " << nodeId_ << "]: I sent a message of CoinDepositSignatureResponse at " << currentTime + totalLatency << endl;
            std::cout << "[Vehicle " << nodeId_ << "] When processing the CoinDepositSignatureRequest message, total time = "
                    << totalLatency << ", queuing time = "
                    << latency.first << " and computation time = " << latency.second << std::endl;
            std::cout << "[Vehicle " << nodeId_ << "]: I sent a message of CoinDepositSignatureResponse at " << currentTime + totalLatency << std::endl;
        }
    }
}

void RcsCarApp::handlePositionUpdate(cObject* obj)
{
    veins::VeinsInetMobility* const mobility = check_and_cast<veins::VeinsInetMobility*>(obj);
    inet::Coord curPosition = mobility->getCurrentPosition();
    double distanceToRSU = sqrt(pow(curPosition.x - RSU_POSITION_X, 2) + pow(curPosition.y - RSU_POSITION_Y, 2));
    double currentTime = simTime().dbl();

    if (distanceToRSU < 100) {
        if (coinAssignmentStage == CoinAssignmentStage::INIT) {
            CoinRequest* packet = new CoinRequest();
            packet->setByteLength(COIN_REQUEST_BYTE_SIZE);
            packet->setVid(nodeId_);
            auto lteControlInfo = new FlowControlInfoNonIp();
            lteControlInfo->setSrcAddr(nodeId_);
            lteControlInfo->setDstAddr(RSU_ADDR);
            lteControlInfo->setDirection(D2D);
            packet->setControlInfo(lteControlInfo);

            std::pair<double,double> latency = cpuModel.getLatency(currentTime, COIN_REQUEST_LATENCY_MEAN, COIN_REQUEST_LATENCY_STDDEV);
            double totalLatency = latency.first + latency.second;
            sendDelayedDown(packet,totalLatency);
//            sendDelayedDown(packet,cpuModel.getLatency(simTime().dbl(), COIN_REQUEST_LATENCY_MEAN, COIN_REQUEST_LATENCY_STDDEV));

            CoinRequestTime = currentTime + totalLatency;
            coinAssignmentStage = CoinAssignmentStage::REQUESTED;
//            EV << "[Vehicle " << nodeId_ << "] When preparing the CoinRequest message, queuing time = "
//                                            << latency.first << " and computation time = " << latency.second << endl;
//            EV << "[Vehicle " << nodeId_ << "]: I sent a message of CoinRequest at " << CoinRequestTime << endl;
            std::cout << "[Vehicle " << nodeId_ << "] When preparing the CoinRequest message, total time = "
                    << totalLatency << ", queuing time = "
                    << latency.first << " and computation time = " << latency.second << std::endl;
            std::cout << "[Vehicle " << nodeId_ << "]: I sent a message of CoinRequest at " << CoinRequestTime << std::endl;
        }
        if (coinDepositStage == CoinDepositStage::INIT) {
            CoinDeposit* packet = new CoinDeposit();
            packet->setByteLength(COIN_DEPOSIT_BYTE_SIZE);
            packet->setVid(nodeId_);
            auto lteControlInfo = new FlowControlInfoNonIp();
            lteControlInfo->setSrcAddr(nodeId_);
            lteControlInfo->setDstAddr(RSU_ADDR);
            lteControlInfo->setDirection(D2D);
            packet->setControlInfo(lteControlInfo);

            std::pair<double,double> latency = cpuModel.getLatency(currentTime, COIN_DEPOSIT_LATENCY_MEAN, COIN_DEPOSIT_LATENCY_STDDEV);
            double totalLatency = latency.first + latency.second;
            sendDelayedDown(packet,totalLatency);
//            sendDelayedDown(packet,cpuModel.getLatency(simTime().dbl(), COIN_DEPOSIT_LATENCY_MEAN, COIN_DEPOSIT_LATENCY_STDDEV));

            CoinDepositTime = currentTime + totalLatency;
            coinDepositStage = CoinDepositStage::REQUESTED;

//            EV << "[Vehicle " << nodeId_ << "] When preparing the CoinDeposit message, queuing time = "
//                                << latency.first << " and computation time = " << latency.second << endl;
//            EV << "[Vehicle " << nodeId_ << "]: I sent a message of CoinDeposit at " << CoinDepositTime << endl;
            std::cout << "[Vehicle " << nodeId_ << "] When preparing the CoinDeposit message, total time = "
                    << totalLatency << ", queuing time = "
                    << latency.first << " and computation time = " << latency.second << std::endl;
            std::cout << "[Vehicle " << nodeId_ << "]: I sent a message of CoinDeposit at " << CoinDepositTime << std::endl;
        }
    } else {
        if (coinAssignmentStage != CoinAssignmentStage::INIT && coinAssignmentStage != CoinAssignmentStage::FINISHED) {
            coinAssignmentStage = CoinAssignmentStage::FAILED;
        }
        if (coinDepositStage != CoinDepositStage::INIT && coinDepositStage != CoinDepositStage::SIGNATURE_SENT) {
            coinDepositStage = CoinDepositStage::FAILED;
        }
    }
}

void RcsCarApp::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details)
{
    Enter_Method_Silent();
    if (signalID == veins::VeinsInetMobility::mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
}
