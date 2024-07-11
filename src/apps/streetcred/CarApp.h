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

#include "BaseApp.h"
#include "corenetwork/binder/LteBinder.h"
#include "common.h"
#include "CpuModel.h"

class CarApp : public BaseApp {

public:
    void initialize(int stage) override;
    ~CarApp() override;
protected:
    CoinAssignmentStage coinAssignmentStage;
    CoinDepositStage coinDepositStage;
    double RSU_POSITION_X;
    double RSU_POSITION_Y;
    int RSU_ADDR;
    double lastDistanceToRSU;

    // timestamp of message traffic
    double coinAssignmentLastTry;
    double coinDepositLastTry;

    MacNodeId nodeId_;
    LteBinder* binder_;

    void handleLowerMessage(cMessage* msg) override;
    void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) override;
    void handlePositionUpdate(cObject* obj);
};

#endif /* APPS_RCS_RCSCARAPP_H_ */
