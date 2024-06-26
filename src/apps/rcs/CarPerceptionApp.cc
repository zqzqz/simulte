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

#include "CarPerceptionApp.h"
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

Define_Module(CarPerceptionApp);

// global set to record which cars are assigned coins and have deposited coins
using std::unordered_set;
extern unordered_set<int> carCoinAssignedSet;

void CarPerceptionApp::initialize(int stage) {
    RcsCarApp::initialize(stage);
    lastDistanceToRSU = 10000;
}

void CarPerceptionApp::handleLowerMessage(cMessage* msg) {
    if (CoinAssignment* req = dynamic_cast<CoinAssignment*>(msg)) {
        int vid = req->getVid();
        if (vid == nodeId_ && coinAssignmentStage != CoinAssignmentStage::FINISHED) {
            EV_WARN << "[Vehicle " << nodeId_ << "]: I received a message of CoinAssignment" << endl;
            EV_WARN << "[Vehicle " << nodeId_ << "]: Time waiting for RSU response = " << simTime().dbl() - CoinRequestTime << endl;
            coinAssignmentStage = CoinAssignmentStage::FINISHED;
            EV_WARN << "[Vehicle " << nodeId_ << "]: Coin assignment succeed." << endl;
        }
    }
}

void CarPerceptionApp::handlePositionUpdate(cObject* obj) {
    veins::VeinsInetMobility* const mobility = check_and_cast<veins::VeinsInetMobility*>(obj);
    inet::Coord curPosition = mobility->getCurrentPosition();
    double distanceToRSU = sqrt(pow(curPosition.x - RSU_POSITION_X, 2) + pow(curPosition.y - RSU_POSITION_Y, 2));
    double currentTime = simTime().dbl();

    if (coinAssignmentStage != CoinAssignmentStage::INIT && coinAssignmentStage != CoinAssignmentStage::FINISHED && coinAssignmentStage != CoinAssignmentStage::FAILED) {
        if (simTime().dbl() > coinAssignmentLastTry + 1) {
            coinAssignmentStage= CoinAssignmentStage::INIT;
        }
    }

    // Triggers coin assignment when leaving the intersection.
    if (distanceToRSU > lastDistanceToRSU) {
//        // debug
//        if (coinRequestCount >= 3 && coinAssignmentStage != CoinAssignmentStage::FINISHED){
//        }
        // RSU already send a coin assignment to me
        if (coinAssignmentStage != CoinAssignmentStage::FINISHED && carCoinAssignedSet.find(nodeId_)!=carCoinAssignedSet.end()){
            coinAssignmentStage = CoinAssignmentStage::FINISHED;
            EV_WARN << "[Vehicle " << nodeId_ << "]: I received a message of CoinAssignment" << endl;
            EV_WARN << "[Vehicle " << nodeId_ << "]: Time waiting for RSU response = " << currentTime - CoinRequestTime << endl;
            EV_WARN << "[Vehicle " << nodeId_ << "]: Coin assignment succeed." << endl;
        }
        else if (coinAssignmentStage == CoinAssignmentStage::INIT) {
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
            coinRequestCount++;
            EV_WARN << "[Vehicle " << nodeId_ << "]: I sent a message of CoinRequest. Queue time " << latency.first
                    << " Computation time " << latency.second << endl;
        }
    }
    if (distanceToRSU > 300 && distanceToRSU > lastDistanceToRSU) {
        if (coinAssignmentStage != CoinAssignmentStage::INIT && coinAssignmentStage != CoinAssignmentStage::FINISHED && coinAssignmentStage != CoinAssignmentStage::FAILED) {
            coinAssignmentStage = CoinAssignmentStage::FAILED;
            EV_WARN << "[Vehicle " << nodeId_ << "]: Coin assignment failed." << endl;
        }
    }

    lastDistanceToRSU = distanceToRSU;
}

void CarPerceptionApp::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details)
{
    RcsCarApp::receiveSignal(source,signalID,obj,details);
}

CarPerceptionApp::~CarPerceptionApp(){
    binder_->unregisterNode(nodeId_);
}
