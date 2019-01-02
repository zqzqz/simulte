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
        bool reserved;
        bool sensed;
        bool possibleCSR;
        simtime_t subframeTime;
        int subframeIndex;
        int subchannelIndex;
        cPacket sci;
        std::vector<Band> occupiedBands;
        std::map<Band, double> rsrpValues;
        std::map<Band, double> rssiValues;

    public:
        Subchannel(const int subchannelSize, simtime_t simulationTime, int subframeIndex=0, int subchannelIndex=0)
        {
            numRbs = subchannelSize;
            reserved = false;
            sensed = true;
            subframeTime = simulationTime;
            sci = NULL;
            this->subframeIndex = subframeIndex;
            this->subchannelIndex = subchannelIndex;
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
        std::vector<Band> getOccupiedBands() const
        {
            return occupiedBands;
        }
        void setOccupiedBands(std::vector<Band> occupiedBands)
        {
            this->occupiedBands = occupiedBands;
        }
        void setSensed(bool sensed)
        {
            this->sensed = sensed;
        }
        bool getSensed() const
        {
            return sensed;
        }
        void setSubframeIndex(int subframeIndex)
        {
            this->subframeIndex = subframeIndex;
        }
        int getSubframeIndex() const
        {
            return subframeIndex;
        }
        void setSubchannelIndex(int subchannelIndex)
        {
            this->subchannelIndex = subchannelIndex;
        }
        int getSubchannelIndex() const
        {
            return subchannelIndex;
        }
};
