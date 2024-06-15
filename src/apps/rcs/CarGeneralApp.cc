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

#include "CarGeneralApp.h"
#include "common.h"
#include "veins/base/utils/FindModule.h"
#include "common/LteControlInfo.h"
#include "veins_inet/VeinsInetMobility.h"

#include "message/CoinRequest_m.h"
#include "message/CoinAssignment_m.h"
#include "message/CoinDeposit_m.h"
#include "message/CoinDepositSignatureRequest_m.h"
#include "message/CoinDepositSignatureResponse_m.h"
#include "message/CoinSubmission_m.h"

Define_Module(CarGeneralApp);

void CarGeneralApp::initialize(int stage) {
    RcsCarApp::initialize(stage);
    lastDistanceToRSU = 10000;
}


void CarGeneralApp::handleLowerMessage(cMessage* msg) {
    double currentTime = simTime().dbl();
    if (CoinAssignment* req = dynamic_cast<CoinAssignment*>(msg)) {
        int vid = req->getVid();
        if (vid == nodeId_) {
            EV_WARN << "[Vehicle " << nodeId_ << "]: I received a message of CoinAssignment" << endl;
            EV_WARN << "[Vehicle " << nodeId_ << "]: Time waiting for RSU response = " << currentTime - CoinRequestTime << endl;
            coinAssignmentStage = CoinAssignmentStage::FINISHED;
            EV_WARN << "[Vehicle " << nodeId_ << "]: Coin assignment succeed." << endl;
        }
    } else if (CoinDepositSignatureRequest* req = dynamic_cast<CoinDepositSignatureRequest*>(msg)) {
        int vid = req->getVid();
        if (vid == nodeId_) {
            EV_WARN << "[Vehicle " << nodeId_ << "]: I received a message of CoinDepositSignatureRequest" << endl;
            EV_WARN << "[Vehicle " << nodeId_ << "]: Time waiting for RSU response = " << currentTime - CoinDepositTime << endl;

            std::pair<double,double> latency = cpuModel.getLatency(currentTime, COIN_DEPOSIT_SIGNATURE_RESPONSE_LATENCY_MEAN, COIN_DEPOSIT_SIGNATURE_RESPONSE_LATENCY_STDDEV);
            CoinDepositSignatureResponse* packet = new CoinDepositSignatureResponse();
            packet->setByteLength(COIN_DEPOSIT_SIGNATURE_RESPONSE_BYTE_SIZE);
            packet->setVid(nodeId_);
            auto lteControlInfo = new FlowControlInfoNonIp();
            lteControlInfo->setSrcAddr(nodeId_);
            lteControlInfo->setDstAddr(RSU_ADDR);
            lteControlInfo->setDirection(D2D);
            packet->setControlInfo(lteControlInfo);
            sendDelayedDown(packet,latency.first+latency.second);

            coinDepositStage = CoinDepositStage::SIGNATURE_SENT;
            EV_WARN << "[Vehicle " << nodeId_ << "]: I sent a message of CoinDepositSignatureResponse. Queue time " << latency.first
                    << " Computation time " << latency.second << endl;
        }
    }
}

void CarGeneralApp::handlePositionUpdate(cObject* obj) {
    veins::VeinsInetMobility* const mobility = check_and_cast<veins::VeinsInetMobility*>(obj);
    inet::Coord curPosition = mobility->getCurrentPosition();
    double distanceToRSU = sqrt(pow(curPosition.x - RSU_POSITION_X, 2) + pow(curPosition.y - RSU_POSITION_Y, 2));
    double currentTime = simTime().dbl();

    if (coinAssignmentStage != CoinAssignmentStage::INIT && coinAssignmentStage != CoinAssignmentStage::FINISHED && coinAssignmentStage != CoinAssignmentStage::FAILED) {
        if (currentTime > coinAssignmentLastTry + 1) {
            coinAssignmentStage = CoinAssignmentStage::INIT;
        }
    }
    if (coinDepositStage != CoinDepositStage::INIT && coinDepositStage != CoinDepositStage::SIGNATURE_SENT && coinDepositStage != CoinDepositStage::FAILED) {
        if (currentTime > coinDepositLastTry + 1) {
            coinDepositStage = CoinDepositStage::INIT;
        }
    }

    // When leaving the intersection, trigger coin assignment.
    if (distanceToRSU > lastDistanceToRSU) {
        if (coinAssignmentStage == CoinAssignmentStage::INIT) {
            coinAssignmentLastTry = currentTime;
            std::pair<double,double> latency = cpuModel.getLatency(currentTime, COIN_REQUEST_LATENCY_MEAN, COIN_REQUEST_LATENCY_STDDEV);

            CoinRequest* packet = new CoinRequest();
            packet->setByteLength(COIN_REQUEST_BYTE_SIZE);
            packet->setVid(nodeId_);
            auto lteControlInfo = new FlowControlInfoNonIp();
            lteControlInfo->setSrcAddr(nodeId_);
            lteControlInfo->setDstAddr(RSU_ADDR);
            lteControlInfo->setDirection(D2D);
            packet->setControlInfo(lteControlInfo);
            sendDelayedDown(packet,latency.first+latency.second);

            CoinRequestTime = currentTime + latency.first+latency.second;
            coinAssignmentStage = CoinAssignmentStage::REQUESTED;
            EV_WARN << "[Vehicle " << nodeId_ << "]: I sent a message of CoinRequest. Queue time " << latency.first
                    << " Computation time " << latency.second << endl;
        }
    }

    // When approaching the intersection, trigger coin deposit.
    if (distanceToRSU < 300 && distanceToRSU < lastDistanceToRSU) {
        if (coinDepositStage == CoinDepositStage::INIT) {
            coinDepositLastTry = currentTime;
            std::pair<double,double> latency = cpuModel.getLatency(currentTime, COIN_DEPOSIT_LATENCY_MEAN, COIN_DEPOSIT_LATENCY_STDDEV);

            CoinDeposit* packet = new CoinDeposit();
            packet->setByteLength(COIN_DEPOSIT_BYTE_SIZE);
            packet->setVid(nodeId_);
            auto lteControlInfo = new FlowControlInfoNonIp();
            lteControlInfo->setSrcAddr(nodeId_);
            lteControlInfo->setDstAddr(RSU_ADDR);
            lteControlInfo->setDirection(D2D);
            packet->setControlInfo(lteControlInfo);
            sendDelayedDown(packet,latency.first+latency.second);

            CoinDepositTime = currentTime + latency.first+latency.second;
            coinDepositStage = CoinDepositStage::REQUESTED;
            EV_WARN << "[Vehicle " << nodeId_ << "]: I sent a message of CoinDeposit. Queue time " << latency.first
                    << " Computation time " << latency.second << endl;
        }
    }

    if (distanceToRSU > 300) {
        if (coinAssignmentStage != CoinAssignmentStage::INIT && coinAssignmentStage != CoinAssignmentStage::FINISHED && coinAssignmentStage != CoinAssignmentStage::FAILED) {
            coinAssignmentStage = CoinAssignmentStage::FAILED;
            EV_WARN << "[Vehicle " << nodeId_ << "]: Coin assignment failed." << endl;
        }
        if (coinDepositStage != CoinDepositStage::INIT && coinDepositStage != CoinDepositStage::SIGNATURE_SENT && coinDepositStage != CoinDepositStage::FAILED) {
            coinDepositStage = CoinDepositStage::FAILED;
            EV_WARN << "[Vehicle " << nodeId_ << "]: Coin deposit failed." << endl;
        }
    }
    lastDistanceToRSU = distanceToRSU;
}

void CarGeneralApp::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details)
{
    RcsCarApp::receiveSignal(source,signalID,obj,details);
}

CarGeneralApp::~CarGeneralApp(){
    binder_->unregisterNode(nodeId_);
}
