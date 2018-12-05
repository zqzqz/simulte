
#include "stack/mac/packet/LteSchedulingGrant.h"

class LteMode4SchedulingGrant : public LteSchedulingGrant
{
    protected:
        simtime_t startTime;
        unsigned int spsPriority;
        unsigned int numSubchannels;
        //unsigned int resourceReservationInterval;
        unsigned int maximumLatency;

    public:

        LteMode4SchedulingGrant(const char *name = NULL, int kind = 0) :
            LteSchedulingGrant(name, kind)
        {
            numSubchannels = 0;
            spsPriority = 0;
            maximumLatency = 0;
            startTime = simTime();
        }


        ~LteMode4SchedulingGrant()
        {
        }

        LteMode4SchedulingGrant(const LteMode4SchedulingGrant& other) :
            LteSchedulingGrant(other.getName())
        {
            operator=(other);
        }

        LteMode4SchedulingGrant& operator=(const LteMode4SchedulingGrant& other)
        {
            numSubchannels = other.numSubchannels;
            spsPriority = other.spsPriority;
            startTime = other.startTime;
            LteSchedulingGrant::operator=(other);
            return *this;
        }

        virtual LteMode4SchedulingGrant *dup() const
        {
            return new LteMode4SchedulingGrant(*this);
        }

        void setStartTime(simtime_t start)
        {
            startTime = start;
        }
        simtime_t getStartTime() const
        {
            return startTime;
        }
        void setSpsPriority(unsigned int priority)
        {
            spsPriority = priority;
        }
        unsigned int getSpsPriority() const
        {
            return spsPriority;
        }
        void setNumberSubchannels(unsigned int subchannels)
        {
            numSubchannels = subchannels;
        }
        unsigned int getNumSubchannels() const
        {
            return numSubchannels;
        }
//        void setResourceReservationInterval(unsigned int resourceResInterval)
//        {
//            resourceReservationInterval = resourceResInterval;
//        }
//        unsigned int getResourceReservationInterval() const
//        {
//            return resourceReservationInterval;
//        }

        void setMaximumLatency(unsigned int maxLatency)
        {
            maximumLatency = maxLatency;
        }
        unsigned int getMaximumLatency() const
        {
            return maximumLatency;
        }
};
