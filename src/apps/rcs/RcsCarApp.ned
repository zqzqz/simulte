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

package lte.apps.rcs;

import lte.apps.rcs.RcsBaseApp;

//
// TODO auto-generated type
//
simple RcsCarApp extends RcsBaseApp
{
    parameters:
        @class(RcsCarApp);
        double RSU_POSITION_X;
        double RSU_POSITION_Y;
        int RSU_ADDR;
        
//        int macNodeId = default(1);
//        int rsuMacNodeId = default(0);
        
        @signal[sentMsg];
        @statistic[sentMsg](title="Messages sent"; unit=""; source="sentMsg"; record=sum,vector);
        @signal[delay];
        @statistic[delay](title="Message Delay"; unit="s"; source="delay"; record=mean,vector);    
        @signal[rcvdMsg]; 
        @statistic[rcvdMsg](title="Messages Received"; unit="s"; source="rcvdMsg"; record=sum,vector);
        @signal[cbr];
        @statistic[cbr](title="Channel Busy Ratio"; unit=""; source="cbr"; record=vector);         
        @display("i=block/source");
}
