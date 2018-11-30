
#include "stack/mac/packet/LteSchedulingGrant.h"

class LteMode4SchedulingGrant : public LteSchedulingGrant
{
    protected:
        simtime_t startTime;
        unsigned int messagePriority;
        unsigned int numSubchannels;
        unsigned int resourceReservationInterval;

    public:

        LteMode4SchedulingGrant(const char *name = NULL, int kind = 0) :
            LteSchedulingGrant(name, kind)
        {
            numSubchannels = 0;
            messagePriority = 0;
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
            messagePriority = other.messagePriority;
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
        void setMessagePriority(unsigned int priority)
        {
            messagePriority = priority;
        }
        unsigned int getMessagePriority() const
        {
            return messagePriority;
        }
        void setNumberSubchannels(unsigned int subchannels)
        {
            numSubchannels = subchannels;
        }
        unsigned int getNumSubchannels() const
        {
            return numSubchannels;
        }
        void setResourceReservationInterval(unsigned int resourceResInterval)
        {
            resourceReservationInterval = resourceResInterval;
        }
        unsigned int getResourceReservationInterval() const
        {
            return resourceReservationInterval;
        }
};
