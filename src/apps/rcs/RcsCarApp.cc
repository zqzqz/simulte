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
#include "package.h"

#include <iostream>
#include <map>
#include <vector>

// global recorder of message segCnt
std::map<MacNodeId, MsgSegCnt> VehicleMsgSegCntMap; // msg sent by vehicle to RSU
// extern std::map<MacNodeId, MsgSegCnt> RSUMsgSegCntMap; // msg sent by RSU to vehicle

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
        CoinAssignmentSegCnt = 0;
        CoinDepositSignatureRequestSegCnt = 0;
        EV << "[Vehicle " << nodeId_ << "]: Initialized" << endl;
//        std::cout << "[Vehicle " << nodeId_ << "]: Initialized at time " << simTime().dbl() << std::endl;
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
        // wait until receive all segments

        int vid = req->getVid();
        if (vid == nodeId_) {
            CoinAssignmentSegCnt++;
            if (CoinAssignmentSegCnt >= RSUMsgSegCntMap[vid][COIN_ASSIGNMENT])
            {
                EV << "[Vehicle " << nodeId_ << "]: I received a message of CoinAssignment" << endl;
                EV << "[Vehicle " << nodeId_ << "]: Time waiting for RSU response = " << currentTime - CoinRequestTime << endl;
//                std::cout << "[Vehicle " << nodeId_ << "]: I received a message of CoinAssignment at time " << currentTime << std::endl;
//                std::cout << "[Vehicle " << nodeId_ << "]: Time waiting for RSU response = " << currentTime - CoinRequestTime << std::endl;
                coinAssignmentStage = CoinAssignmentStage::FINISHED;
            }
        }
    } else if (CoinDepositSignatureRequest* req = dynamic_cast<CoinDepositSignatureRequest*>(msg)) {
        int vid = req->getVid();
        if (vid == nodeId_) {
            CoinDepositSignatureRequestSegCnt++;
            if (CoinDepositSignatureRequestSegCnt >= RSUMsgSegCntMap[vid][COIN_DEPOSIT_SIGNATURE_REQUEST])
            {
                EV << "[Vehicle " << nodeId_ << "]: I received a message of CoinDepositSignatureRequest" << endl;
                EV << "[Vehicle " << nodeId_ << "]: Time waiting for RSU response = " << currentTime - CoinDepositTime << endl;
//                std::cout << "[Vehicle " << nodeId_ << "]: I received a message of CoinDepositSignatureRequest at time " << currentTime << std::endl;
//                std::cout << "[Vehicle " << nodeId_ << "]: Time waiting for RSU response = " << currentTime - CoinDepositTime << std::endl;

                uint segCnt = COIN_DEPOSIT_SIGNATURE_RESPONSE_BYTE_SIZE / MACPKG_MAXSIZE + 1;
                std::pair<double,double> latency = cpuModel.getLatency(currentTime, COIN_DEPOSIT_SIGNATURE_RESPONSE_LATENCY_MEAN, COIN_DEPOSIT_SIGNATURE_RESPONSE_LATENCY_STDDEV);
                for (uint i = 0; i < segCnt; i++){
                    CoinDepositSignatureResponse* packet = new CoinDepositSignatureResponse();
                    packet->setByteLength(MACPKG_MAXSIZE);
                    packet->setVid(nodeId_);
                    auto lteControlInfo = new FlowControlInfoNonIp();
                    lteControlInfo->setSrcAddr(nodeId_);
                    lteControlInfo->setDstAddr(RSU_ADDR);
                    lteControlInfo->setDirection(D2D);
                    packet->setControlInfo(lteControlInfo);
                    sendDelayedDown(packet,latency.first+latency.second);
//                    std::cout << "[Vehicle " << nodeId_ << "] send #" << i << " segment of CoinDepositSignatureResponse" << std::endl;
    //            sendDelayedDown(packet,cpuModel.getLatency(simTime().dbl(), COIN_DEPOSIT_SIGNATURE_RESPONSE_LATENCY_MEAN, COIN_DEPOSIT_SIGNATURE_RESPONSE_LATENCY_STDDEV));
                }
                // update VehicleMsgSegCntMap
                // if (VehicleMsgSegCntMap.find(nodeId_)==VehicleMsgSegCntMap.end())
                //     VehicleMsgSegCntMap[nodeId_] = std::vector<MsgSegCnt>();
                // VehicleMsgSegCntMap[nodeId_].push_back(std::make_pair(COIN_DEPOSIT_SIGNATURE_RESPONSE,segCnt));
                VehicleMsgSegCntMap[nodeId_][COIN_DEPOSIT_SIGNATURE_RESPONSE]=segCnt;

                coinDepositStage = CoinDepositStage::SIGNATURE_SENT;
                EV << "[Vehicle " << nodeId_ << "]: I sent a message of CoinDepositSignatureResponse. Queue time " << latency.first
                        << " Computation time " << latency.second << endl;
//                std::cout << "[Vehicle " << nodeId_ << "] When processing the CoinDepositSignatureRequest message, total time = "
//                        << totalLatency << ", queuing time = "
//                        << queueLatency << " and computation time = " << computeLatency << std::endl;
//                std::cout << "[Vehicle " << nodeId_ << "]: I sent a message of CoinDepositSignatureResponse at " << currentTime + totalLatency << std::endl;
            }
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
            uint segCnt = COIN_REQUEST_BYTE_SIZE / MACPKG_MAXSIZE + 1;
            std::pair<double,double> latency = cpuModel.getLatency(currentTime, COIN_REQUEST_LATENCY_MEAN, COIN_REQUEST_LATENCY_STDDEV);
            for (uint i = 0; i < segCnt; i++)
            {
                CoinRequest* packet = new CoinRequest();
                packet->setByteLength(MACPKG_MAXSIZE);
                packet->setVid(nodeId_);
                auto lteControlInfo = new FlowControlInfoNonIp();
                lteControlInfo->setSrcAddr(nodeId_);
                lteControlInfo->setDstAddr(RSU_ADDR);
                lteControlInfo->setDirection(D2D);
                packet->setControlInfo(lteControlInfo);
                sendDelayedDown(packet,latency.first+latency.second);
//                std::cout << "[Vehicle " << nodeId_ << "] send #" << i << " segment of CoinRequest" << std::endl;
//            sendDelayedDown(packet,cpuModel.getLatency(simTime().dbl(), COIN_REQUEST_LATENCY_MEAN, COIN_REQUEST_LATENCY_STDDEV));
            }
            // update VehicleMsgSegCntMap
            // if (VehicleMsgSegCntMap.find(nodeId_)==VehicleMsgSegCntMap.end())
            //     VehicleMsgSegCntMap[nodeId_] = std::vector<MsgSegCnt>();
            // VehicleMsgSegCntMap[nodeId_].push_back(std::make_pair(COIN_REQUEST,segCnt));
            VehicleMsgSegCntMap[nodeId_][COIN_REQUEST]=segCnt;

            CoinRequestTime = currentTime + latency.first+latency.second;
            coinAssignmentStage = CoinAssignmentStage::REQUESTED;
            EV << "[Vehicle " << nodeId_ << "]: I sent a message of CoinRequest. Queue time " << latency.first
                    << " Computation time " << latency.second << endl;
//            std::cout << "[Vehicle " << nodeId_ << "] When preparing the CoinRequest message, total time = "
//                    << totalLatency << ", queuing time = "
//                    << queueLatency << " and computation time = " << computeLatency << std::endl;
//            std::cout << "[Vehicle " << nodeId_ << "]: I sent a message of CoinRequest at " << CoinRequestTime << std::endl;
        }
        if (coinDepositStage == CoinDepositStage::INIT) {
            uint segCnt = COIN_REQUEST_BYTE_SIZE / MACPKG_MAXSIZE + 1;
            std::pair<double,double> latency = cpuModel.getLatency(currentTime, COIN_DEPOSIT_LATENCY_MEAN, COIN_DEPOSIT_LATENCY_STDDEV);
            for (uint i = 0; i < segCnt; i++)
            {
                CoinDeposit* packet = new CoinDeposit();
                packet->setByteLength(MACPKG_MAXSIZE);
                packet->setVid(nodeId_);
                auto lteControlInfo = new FlowControlInfoNonIp();
                lteControlInfo->setSrcAddr(nodeId_);
                lteControlInfo->setDstAddr(RSU_ADDR);
                lteControlInfo->setDirection(D2D);
                packet->setControlInfo(lteControlInfo);
                sendDelayedDown(packet,latency.first+latency.second);
//                std::cout << "[Vehicle " << nodeId_ << "] send #" << i << " segment of CoinDeposit" << std::endl;
//            sendDelayedDown(packet,cpuModel.getLatency(simTime().dbl(), COIN_DEPOSIT_LATENCY_MEAN, COIN_DEPOSIT_LATENCY_STDDEV));
            }
            // update VehicleMsgSegCntMap
            // if (VehicleMsgSegCntMap.find(nodeId_)==VehicleMsgSegCntMap.end())
            //     VehicleMsgSegCntMap[nodeId_] = std::vector<MsgSegCnt>();
            // VehicleMsgSegCntMap[nodeId_].push_back(std::make_pair(COIN_DEPOSIT, segCnt));
            VehicleMsgSegCntMap[nodeId_][COIN_DEPOSIT]=segCnt;
            
            CoinDepositTime = currentTime + latency.first+latency.second;
            coinDepositStage = CoinDepositStage::REQUESTED;

            EV << "[Vehicle " << nodeId_ << "]: I sent a message of CoinDeposit. Queue time " << latency.first
                    << " Computation time " << latency.second << endl;
//            std::cout << "[Vehicle " << nodeId_ << "] When preparing the CoinDeposit message, total time = "
//                    << totalLatency << ", queuing time = "
//                    << queueLatency << " and computation time = " << computeLatency << std::endl;
//            std::cout << "[Vehicle " << nodeId_ << "]: I sent a message of CoinDeposit at " << CoinDepositTime << std::endl;
        }
    } else {
        if (coinAssignmentStage != CoinAssignmentStage::INIT && coinAssignmentStage != CoinAssignmentStage::FINISHED) {
            coinAssignmentStage = CoinAssignmentStage::FAILED;
            EV << "[Vehicle " << nodeId_ << "]: Coin assignment failed." << endl;
        }
        if (coinDepositStage != CoinDepositStage::INIT && coinDepositStage != CoinDepositStage::SIGNATURE_SENT) {
            coinDepositStage = CoinDepositStage::FAILED;
            EV << "[Vehicle " << nodeId_ << "]: Coin deposit failed." << endl;
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
