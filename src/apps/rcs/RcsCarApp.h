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

#ifndef APPS_RCS_RCSCARAPP_H_
#define APPS_RCS_RCSCARAPP_H_

#include "apps/mode4App/Mode4BaseApp.h"
#include "corenetwork/binder/LteBinder.h"
#include "common.h"
#include "CpuModel.h"

class RcsCarApp : public Mode4BaseApp {

public:
    ~RcsCarApp() override;
protected:
    CoinAssignmentStage coinAssignmentStage;
    CoinDepositStage coinDepositStage;
    CpuModel cpuModel;

    MacNodeId nodeId_;
    LteBinder* binder_;

    virtual void initialize(int stage) override;
    int numInitStages() const { return inet::NUM_INIT_STAGES; }
    virtual void handleSelfMessage(cMessage *msg) override;
    virtual void handleLowerMessage(cMessage* msg) override;
    virtual void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) override;
    void handlePositionUpdate(cObject* obj);
};

#endif /* APPS_RCS_RCSCARAPP_H_ */
