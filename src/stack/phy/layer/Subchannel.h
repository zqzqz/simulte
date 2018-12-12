//
//                           SimuLTE
//
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself,
// and cannot be removed from it.
//

#ifndef _LTE_AIRPHYUED2D_H_
#define _LTE_AIRPHYUED2D_H_

#include "common/LteCommon.h"
#include "stack/phy/packet/SidelinkControlInformation_m.h"

class Subchannel
{
    protected:
        int numRbs;
        RbMap usedRbs;
        bool reserved;
        simtime_t subframeTime;
        SidelinkControlInformation SCI;
        double averageRSRP;
        double averageRSSI;

    public:
        Subchannel(const int subchannelSize)
        {
            numRbs = subchannelSize;
            usedRbs = NULL;
            reserved = false;
            subframeTime = simTime();
            SCI = NULL;
            averageRSRP = 0;
            averageRSSI = 0;
        }


        ~Subchannel()
        {
        }

        Subchannel(const Subchannel& other) :
            Subchannel(other.getName())
        {
            operator=(other);
        }

        Subchannel& operator=(const Subchannel& other)
        {
            numRbs = other.numRbs;
            usedRbs = other.usedRbs;
            reserved = other.reserved;
            subframeTime = other.subframeTime;
            SCI = other.SCI;
            averageRSRP = other.averageRSRP;
            averageRSSI = other.averageRSSI;
            return *this;
        }

        virtual Subchannel *dup() const
        {
            return new Subchannel(*this);
        }

        void setSubframeTime(simtime_t subframeTime)
        {
            this->subframeTime = subframeTime;
        }
        simtime_t getSubframeTime() const
        {
            return subframeTime;
        }
        void setNumRbs(int numRbs)
        {
            this->numRbs = numRbs;
        }
        int getNumRbs() const
        {
            return numRbs;
        }
        void setReserved(bool reserved)
        {
            this->reserved = reserved;
        }
        bool getReserved() const
        {
            return reserved;
        }
        void setSCIMessage(SidelinkControlInformation SCI)
        {
            this->SCI = SCI;
        }
        SidelinkControlInformation getSCIMessage() const
        {
            return SCI;
        }
        void setAverageRSRP(double averageRSRP)
        {
            this->averageRSRP = averageRSRP;
        }
        double getAverageRSRP() const
        {
            return averageRSRP;
        }
        void setAverageRSSI(double averageRSSI)
        {
            this->averageRSSI = averageRSSI;
        }
        double getAverageRSSI() const
        {
            return averageRSSI;
        }

};
