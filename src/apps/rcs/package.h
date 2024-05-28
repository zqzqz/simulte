/*
 * package.h
 * simulate MAC layer package segmentation
 */

#ifndef PACKAGE_H_
#define PACKAGE_H_

#define MACPKG_MAXSIZE 207
#include <map>

// global map: Vehicle/RSU, packageType, message #segment
//#define COIN_ASSIGNMENT 0
//#define COIN_DEPOSIT_SIGNATURE_REQUEST 1
//#define COIN_REQUEST 2
//#define COIN_DEPOSIT 3
//#define COIN_DEPOSIT_SIGNATURE_RESPONSE 4
enum MsgType {
    // rsu
    COIN_ASSIGNMENT,
    COIN_DEPOSIT_SIGNATURE_REQUEST,

    // vehicle
    COIN_REQUEST,
    COIN_DEPOSIT,
    COIN_DEPOSIT_SIGNATURE_RESPONSE
};

using MsgSegCnt = std::map<MsgType, uint>;



// sender: segment packages into a vector of package, add sequence no

// receiver: reassemble message when receive all packages

// clean up some entries when finishing communication

#endif
