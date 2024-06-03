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

#include "RcsBaseApp.h"
#include "common.h"

Define_Module(RcsBaseApp);

void RcsBaseApp::initialize(int stage) {
    Mode4BaseApp::initialize(stage);
    int numCpuCores = par("numCpuCores");
    cpuModel.init(numCpuCores);
    HEADER_BYTE_SIZE = par("HEADER_BYTE_SIZE");
    COIN_REQUEST_BYTE_SIZE = par("COIN_REQUEST_BYTE_SIZE");
    COIN_ASSIGNMENT_BYTE_SIZE = par("COIN_ASSIGNMENT_BYTE_SIZE");
    COIN_DEPOSIT_BYTE_SIZE = par("COIN_DEPOSIT_BYTE_SIZE");
    COIN_DEPOSIT_SIGNATURE_REQUEST_BYTE_SIZE = par("COIN_DEPOSIT_SIGNATURE_REQUEST_BYTE_SIZE");
    COIN_DEPOSIT_SIGNATURE_RESPONSE_BYTE_SIZE = par("COIN_DEPOSIT_SIGNATURE_RESPONSE_BYTE_SIZE");
    COIN_SUBMISSION_BYTE_SIZE = par("COIN_SUBMISSION_BYTE_SIZE");
    COIN_REQUEST_LATENCY_MEAN = par("COIN_REQUEST_LATENCY_MEAN");
    COIN_REQUEST_LATENCY_STDDEV = par("COIN_REQUEST_LATENCY_STDDEV");
    COIN_ASSIGNMENT_LATENCY_MEAN = par("COIN_ASSIGNMENT_LATENCY_MEAN");
    COIN_ASSIGNMENT_LATENCY_STDDEV = par("COIN_ASSIGNMENT_LATENCY_STDDEV");
    COIN_DEPOSIT_LATENCY_MEAN = par("COIN_DEPOSIT_LATENCY_MEAN");
    COIN_DEPOSIT_LATENCY_STDDEV = par("COIN_DEPOSIT_LATENCY_STDDEV");
    COIN_DEPOSIT_SIGNATURE_REQUEST_LATENCY_MEAN = par("COIN_DEPOSIT_SIGNATURE_REQUEST_LATENCY_MEAN");
    COIN_DEPOSIT_SIGNATURE_REQUEST_LATENCY_STDDEV = par("COIN_DEPOSIT_SIGNATURE_REQUEST_LATENCY_STDDEV");
    COIN_DEPOSIT_SIGNATURE_RESPONSE_LATENCY_MEAN = par("COIN_DEPOSIT_SIGNATURE_RESPONSE_LATENCY_MEAN");
    COIN_DEPOSIT_SIGNATURE_RESPONSE_LATENCY_STDDEV = par("COIN_DEPOSIT_SIGNATURE_RESPONSE_LATENCY_STDDEV");
    COIN_SUBMISSION_LATENCY_MEAN = par("COIN_SUBMISSION_LATENCY_MEAN");
    COIN_SUBMISSION_LATENCY_STDDEV = par("COIN_SUBMISSION_LATENCY_STDDEV");
}
//
void RcsBaseApp::handleSelfMessage(cMessage *msg) {}
