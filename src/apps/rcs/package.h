/*
 * package.h
 * simulate MAC layer package segmentation
 */

#ifndef PACKAGE_H_
#define PACKAGE_H_

#define MACPKG_MAXSIZE 1500

// global map: Vehicle/RSU, packageType, message #segment

// sender: segment packages into a vector of package, add sequence no

// receiver: reassemble message when receive all packages

// clean up some entries when finishing communication

#endif
