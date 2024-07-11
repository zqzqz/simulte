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

#ifndef APPS_TCPSTREEDCRED_TCPRSUAPP_H_
#define APPS_TCPSTREEDCRED_TCPRSUAPP_H_

#include "apps/streetcred/BaseApp.h"
#include "apps/streetcred/common.h"
#include "apps/streetcred/CpuModel.h"
#include "corenetwork/binder/LteBinder.h"
#include "inet/common/INETDefs.h"
#include "inet/applications/tcpapp/TCPSrvHostApp.h"
#include <map>

class TCPRSUApp : public inet::TCPSrvHostApp {
public:
    void initialize(int stage) override;
    virtual ~TCPRSUApp();
protected:
    std::map<int, CoinAssignmentStage> coinAssignmentStages;
    std::map<int, CoinDepositStage> coinDepositStages;

    MacNodeId nodeId_;
    LteBinder* binder_;

    void handleMessage(cMessage *msg) override;

    /* RSUApp parameters */
    CpuModel cpuModel;
    int COIN_ASSIGNMENT_BYTE_SIZE;
    int COIN_DEPOSIT_SIGNATURE_REQUEST_BYTE_SIZE;
    int COIN_SUBMISSION_BYTE_SIZE;
    double COIN_ASSIGNMENT_LATENCY_MEAN;
    double COIN_ASSIGNMENT_LATENCY_STDDEV;
    double COIN_DEPOSIT_SIGNATURE_REQUEST_LATENCY_MEAN;
    double COIN_DEPOSIT_SIGNATURE_REQUEST_LATENCY_STDDEV;
    double COIN_SUBMISSION_LATENCY_MEAN;
    double COIN_SUBMISSION_LATENCY_STDDEV;
};

#endif /* APPS_TCPSTREEDCRED_TCPRSUAPP_H_ */
