//
//                           SimuLTE
//
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself,
// and cannot be removed from it.
//

#include "common/LteCommon.h"

class Subchannel
{
    protected:
        int numRbs;
        RbMap usedRbs;
        bool reserved;
        simtime_t subframeTime;
        cPacket sci;
        std::vector occupiedBands;
        std::map<Band, double> rsrpValues;
        std::map<Band, double> rssiValues;

    public:
        Subchannel(const int subchannelSize)
        {
            numRbs = subchannelSize;
            reserved = false;
            subframeTime = simTime();
            sci = NULL;
        }


        ~Subchannel()
        {
        }

        Subchannel(const Subchannel& other)
        {
            operator=(other);
        }

        Subchannel& operator=(const Subchannel& other)
        {
            numRbs = other.numRbs;
            usedRbs = other.usedRbs;
            reserved = other.reserved;
            subframeTime = other.subframeTime;
            sci = other.sci;
            rsrpValues = other.rsrpValues;
            rssiValues = other.rssiValues;
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
        void setSCI(cPacket* SCI)
        {
            this->sci = SCI;
        }
        cPacket* getSCIMessage() const
        {
            return sci;
        }
        void setAverageRSRP(double averageRSRP)
        {
            this->averageRSRP = averageRSRP;
        }
        double getAverageRSRP() const
        {
            double sum = 0;
            std::map<Band, double>::iterator it;
            for(it=rsrpValues.begin(); it!=rsrpValues.end(); it++)
            {
                sum += it->second;
            }
            return sum/numRbs;
        }
        double getAverageRSSI() const
        {
            double sum = 0;
            std::map<Band, double>::iterator it;
            for(it=rssiValues.begin(); it!=rssiValues.end(); it++)
            {
                sum += it->second;
            }
            return sum/numRbs;
        }
        void addRsrpValue(double rsrpValue, Band band)
        {
            rsrpValues[band] = rsrpValue;
        }
        void addRssiValue(double rssiValue, Band band)
        {
            rssiValues[band] = rssiValue;
        }
        std::vector getOccupiedBands() const
        {
            return occupiedBands;
        }
        void setOccupiedBands(std::vector occupiedBands)
        {
            this->occupiedBands = occupiedBands;
        }

};
