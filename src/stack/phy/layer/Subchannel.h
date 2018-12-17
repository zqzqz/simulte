//
//                           SimuLTE
//
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself,
// and cannot be removed from it.
//

#include "common/LteCommon.h"
#include "stack/phy/packet/SidelinkControlInformation_m.h"

class Subchannel
{
    protected:
        int numRbs;
        RbMap usedRbs;
        bool reserved;
        simtime_t subframeTime;
        std::vector<SidelinkControlInformation> SCIs;
        double averageRSRP;
        double averageRSSI;
        std::vector<Band> occupiedBands;

    public:
        Subchannel(const int subchannelSize)
        {
            numRbs = subchannelSize;
            reserved = false;
            subframeTime = simTime();
            SCI = NULL;
            // TODO: I need to find a way of logically setting these? Is it a simple as setting them to the UE Noise level?
            // Or maybe in the channel model we determine a background noise level and use that as a basis.
            averageRSRP = 0;
            averageRSSI = 0;
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
        void addSCI(SidelinkControlInformation SCI)
        {
            SCIs.push_back(SCI);
        }
        std::vector<SidelinkControlInformation> getSCIMessage() const
        {
            return SCIs;
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
        std::vector getOccupiedBands() const
        {
            return occupiedBands;
        }
        void setOccupiedBands(std::vector occupiedBands)
        {
            this->occupiedBands = occupiedBands;
        }

};
