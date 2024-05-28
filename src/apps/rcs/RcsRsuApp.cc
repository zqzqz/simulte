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
#include <map>

// global recorder of message segCnt
//extern std::unordered_map<MacNodeId, MsgSegCnt> VehicleMsgSegCntMap;// msg sent by vehicle to RSU
std::map<MacNodeId, MsgSegCnt> RSUMsgSegCntMap;// msg sent by RSU to vehicle

Define_Module(RcsRsuApp);

RcsRsuApp::~RcsRsuApp() {
    // TODO Auto-generated destructor stub
    std::cout << "VehicleMsgSegCntMap:" <<std::endl;
    for (const auto& e : VehicleMsgSegCntMap)
    {
        std::cout << "Vehicle #" << e.first << ": ";
        for (const auto& v : e.second)
            std::cout << "msgType " << v.first << " length " << v.second <<" ";
        std::cout << std::endl;
    }
    
    std::cout << "RSUMsgSegCntMap:" <<std::endl;
    for (const auto& e : RSUMsgSegCntMap)
    {
        std::cout << "Vehicle #" << e.first << ": ";
        for (const auto& v : e.second)
            std::cout << "msgType " << v.first << " length " << v.second <<" ";
        std::cout << std::endl;
    }
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
        int vid = req->getVid();
        // wait until receive all segments
        if(recvMsgSegCnt.find(vid)==recvMsgSegCnt.end()) recvMsgSegCnt[vid][COIN_REQUEST]=0;
        recvMsgSegCnt[vid][COIN_REQUEST]++;
        if (recvMsgSegCnt[vid][COIN_REQUEST]>=VehicleMsgSegCntMap[vid][COIN_REQUEST])
        {
//        EV << "[RSU] I received a message of CoinRequest from " << req->getVid() << " at " << currentTime << endl;
            std::cout << "[RSU] I received a message of CoinRequest from " << req->getVid() << " at " << currentTime << std::endl;
            
            if (coinDepositStages.find(vid) == coinDepositStages.end()) {
                uint segCnt = COIN_ASSIGNMENT_BYTE_SIZE / MACPKG_MAXSIZE + 1;
                double totalLatency = 0, queueLatency = 0, computeLatency = 0;
                for (uint i = 0; i < segCnt; i++){
                    CoinAssignment* packet = new CoinAssignment();
                    packet->setByteLength(MACPKG_MAXSIZE);
                    packet->setVid(vid);
                    auto lteControlInfo = new FlowControlInfoNonIp();
                    lteControlInfo->setSrcAddr(nodeId_);
                    lteControlInfo->setDstAddr(vid);
                    lteControlInfo->setDirection(D2D);
                    packet->setControlInfo(lteControlInfo);
                    // package list for loop

                    std::pair<double,double> latency = cpuModel.getLatency(currentTime, COIN_ASSIGNMENT_LATENCY_MEAN, COIN_ASSIGNMENT_LATENCY_STDDEV);
                    queueLatency += latency.first;
                    computeLatency += latency.second;
                    totalLatency += (latency.first+latency.second);
                    sendDelayedDown(packet,totalLatency);
    //            sendDelayedDown(packet, cpuModel.getLatency(simTime().dbl(), COIN_ASSIGNMENT_LATENCY_MEAN, COIN_ASSIGNMENT_LATENCY_STDDEV));
                    std::cout << "[RSU] send #" << i << " segment of CoinAssignment" << std::endl;
                }
                // update RSUMsgSegCntMap
                // if (RSUMsgSegCntMap.find(vid)==RSUMsgSegCntMap.end())
                //     RSUMsgSegCntMap[vid] = std::vector<MsgSegCnt>();
                // RSUMsgSegCntMap[vid].push_back(std::make_pair(COIN_ASSIGNMENT,segCnt));
                RSUMsgSegCntMap[vid][COIN_ASSIGNMENT]=segCnt;

                coinAssignmentStages[vid] = CoinAssignmentStage::SENT;

    //            EV << "[RSU] When processing the CoinRequest message, queuing time = "
    //                    << queueLatency << " and computation time = " << computeLatency << endl;
    //            EV << "[RSU] I sent a message of CoinAssignment to " << req->getVid() << " at " << currentTime + totalLatency << endl;
                std::cout << "[RSU] When processing the CoinRequest message from " << req->getVid() << ", total time = "
                        << totalLatency << ", queuing time = "
                        << queueLatency << " and computation time = " << computeLatency << std::endl;
                std::cout << "[RSU] I sent a message of CoinAssignment to " << req->getVid() << " at " << currentTime + totalLatency << endl;
            }
        }
    } else if (CoinDeposit* req = dynamic_cast<CoinDeposit*>(msg)) {
        int vid = req->getVid();
        // wait until receive all segments
        if(recvMsgSegCnt.find(vid)==recvMsgSegCnt.end()) recvMsgSegCnt[vid][COIN_DEPOSIT]=0;
        recvMsgSegCnt[vid][COIN_DEPOSIT]++;
        if (recvMsgSegCnt[vid][COIN_DEPOSIT]>=VehicleMsgSegCntMap[vid][COIN_DEPOSIT]){
    //        EV << "[RSU] I received a message of CoinDeposit from " << req->getVid() << " at " << simTime().dbl() << std::endl;
            std::cout << "[RSU] I received a message of CoinDeposit from " << req->getVid() << " at " << simTime().dbl() << std::endl;
            
            if (coinDepositStages.find(vid) == coinDepositStages.end()) {
                uint segCnt = COIN_DEPOSIT_SIGNATURE_REQUEST_BYTE_SIZE / MACPKG_MAXSIZE + 1;
                double totalLatency = 0, queueLatency = 0, computeLatency = 0;
                for (uint i = 0; i < segCnt; i++){
                    CoinDepositSignatureRequest* packet = new CoinDepositSignatureRequest();
                    packet->setByteLength(MACPKG_MAXSIZE);
                    packet->setVid(vid);
                    auto lteControlInfo = new FlowControlInfoNonIp();
                    lteControlInfo->setSrcAddr(nodeId_);
                    lteControlInfo->setDstAddr(vid);
                    lteControlInfo->setDirection(D2D);
                    packet->setControlInfo(lteControlInfo);

                    std::pair<double,double> latency = cpuModel.getLatency(currentTime, COIN_DEPOSIT_SIGNATURE_REQUEST_LATENCY_MEAN, COIN_DEPOSIT_SIGNATURE_REQUEST_LATENCY_STDDEV);
                    queueLatency += latency.first;
                    computeLatency += latency.second;
                    totalLatency += (latency.first+latency.second);
                    sendDelayedDown(packet,totalLatency);
    //            sendDelayedDown(packet, cpuModel.getLatency(simTime().dbl(), COIN_DEPOSIT_SIGNATURE_REQUEST_LATENCY_MEAN, COIN_DEPOSIT_SIGNATURE_REQUEST_LATENCY_STDDEV));
                    std::cout << "[RSU] send #" << i << " segment of CoinDepositSignatureRequest" << std::endl;
                }
                // update RSUMsgSegCntMap
                // if (RSUMsgSegCntMap.find(vid)==RSUMsgSegCntMap.end())
                // //     RSUMsgSegCntMap[vid] = std::vector<MsgSegCnt>();
                // RSUMsgSegCntMap[vid].push_back(std::make_pair(COIN_DEPOSIT_SIGNATURE_REQUEST,segCnt));
                RSUMsgSegCntMap[vid][COIN_DEPOSIT_SIGNATURE_REQUEST]=segCnt;

                coinDepositStages[vid] = CoinDepositStage::SIGNATURE_REQUESTED;

    //            EV << "[RSU] When processing the CoinDeposit message, queuing time = "
    //                    << queueLatency << " and computation time = " << computeLatency << endl;
    //            EV << "[RSU] I sent a message of CoinDepositSignatureRequest to " << req->getVid() << " at " << currentTime + totalLatency << endl;
                std::cout << "[RSU] When processing the CoinDeposit message from " << req->getVid() << ", total time = "
                        << totalLatency << ", queuing time = "
                        << queueLatency << " and computation time = " << computeLatency << std::endl;
                std::cout << "[RSU] I sent a message of CoinDepositSignatureRequest to " << req->getVid() << " at " << currentTime + totalLatency << std::endl;
            
            }
        }
    } else if (CoinDepositSignatureResponse* req = dynamic_cast<CoinDepositSignatureResponse*>(msg)) {
        int vid = req->getVid();
        // wait until receive all segments
        if(recvMsgSegCnt.find(vid)==recvMsgSegCnt.end()) recvMsgSegCnt[vid][COIN_DEPOSIT_SIGNATURE_RESPONSE]=0;
        recvMsgSegCnt[vid][COIN_DEPOSIT_SIGNATURE_RESPONSE]++;
        if (recvMsgSegCnt[vid][COIN_DEPOSIT_SIGNATURE_RESPONSE]>=VehicleMsgSegCntMap[vid][COIN_DEPOSIT_SIGNATURE_RESPONSE]){
    //        EV << "[RSU] I received a message of CoinDepositSignatureResponse from " << req->getVid() << " at " << currentTime << endl;
            std::cout << "[RSU] I received a message of CoinDepositSignatureResponse from " << req->getVid() << " at " << currentTime << std::endl;
            
            if (coinDepositStages.find(vid) != coinDepositStages.end() && coinDepositStages[vid] == CoinDepositStage::SIGNATURE_REQUESTED) {
                // TODO: The communication to central database.
                coinDepositStages[vid] = CoinDepositStage::SUBMITTED;
            }
        }
    }
}

