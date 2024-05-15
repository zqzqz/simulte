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

import inet.applications.contract.ITCPApp;

//
// TODO auto-generated type
//
simple RcsTcpRsuApp like ITCPApp
{
    parameters:
		string localAddress = default("");
		int localPort = default(1000);
		string serverThreadClass = default("RcsTcpRsuThread");
		string dataTransferMode @enum("bytecount", "object", "bytestream") = default("bytecount");
    gates:
        input tcpIn @labels(TCPCommand/up);
        output tcpOut @labels(TCPCommand/up);
}