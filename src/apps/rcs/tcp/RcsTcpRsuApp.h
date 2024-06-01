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

#ifndef APPS_RCS_RCSTCPRSUAPP_H_
#define APPS_RCS_RCSTCPRSUAPP_H_

#include "inet/common/INETDefs.h"
#include "inet/applications/tcpapp/TCPSrvHostApp.h"

class RcsTcpRsuApp : public inet::TCPSrvHostApp {
protected:
    // pass
public:
    RcsTcpRsuApp();
    virtual ~RcsTcpRsuApp();

    virtual void handleMessage(cMessage *msg) override;
};

#endif /* APPS_RCS_RCSTCPRSUAPP_H_ */
