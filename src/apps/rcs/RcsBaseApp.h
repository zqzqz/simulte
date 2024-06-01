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

#ifndef APPS_RCS_RCSBASEAPP_H_
#define APPS_RCS_RCSBASEAPP_H_

#include "apps/mode4App/Mode4BaseApp.h"
#include "common.h"
#include "CpuModel.h"

class RcsBaseApp : public Mode4BaseApp {
public:
    void initialize(int stage) override;
protected:
    CpuModel cpuModel;
    int HEADER_BYTE_SIZE;
    int COIN_REQUEST_BYTE_SIZE;
    int COIN_ASSIGNMENT_BYTE_SIZE;
    int COIN_DEPOSIT_BYTE_SIZE;
    int COIN_DEPOSIT_SIGNATURE_REQUEST_BYTE_SIZE;
    int COIN_DEPOSIT_SIGNATURE_RESPONSE_BYTE_SIZE;
    int COIN_SUBMISSION_BYTE_SIZE;
    double COIN_REQUEST_LATENCY_MEAN;
    double COIN_REQUEST_LATENCY_STDDEV;
    double COIN_ASSIGNMENT_LATENCY_MEAN;
    double COIN_ASSIGNMENT_LATENCY_STDDEV;
    double COIN_DEPOSIT_LATENCY_MEAN;
    double COIN_DEPOSIT_LATENCY_STDDEV;
    double COIN_DEPOSIT_SIGNATURE_REQUEST_LATENCY_MEAN;
    double COIN_DEPOSIT_SIGNATURE_REQUEST_LATENCY_STDDEV;
    double COIN_DEPOSIT_SIGNATURE_RESPONSE_LATENCY_MEAN;
    double COIN_DEPOSIT_SIGNATURE_RESPONSE_LATENCY_STDDEV;
    double COIN_SUBMISSION_LATENCY_MEAN;
    double COIN_SUBMISSION_LATENCY_STDDEV;
protected:
    virtual void handleSelfMessage(cMessage *msg) override;
    virtual void handleLowerMessage(cMessage* msg) override;
    virtual void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) override;
    void handlePositionUpdate(cObject* obj);
};

#endif /* APPS_RCS_RCSBASEAPP_H_ */
