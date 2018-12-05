/*
 * SpsCandidateResources.h
 *
 *  Created on: Oct 26, 2018
 *      Author: Brian McCarthy
 *       Email: b.mccarthy@cs.ucc.ie
 */

//
//                           SimuLTE
//
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself,
// and cannot be removed from it.
//

#include "stack/phy/packet/SpsCandidateResources_m.h"
#include "stack/mac/packet/LteMode4SchedulingGrant.h"
#include "common/LteCommon.h"

class SpsCandidateResources: public SpsCandidateResources_Base
{
  protected:

    std::vector<RbMap> possibleCSRs;
    std::vector<simtime_t> CSRStartTimes;
    LteMode4SchedulingGrant schedulingGrant;

  public:

    SpsCandidateResources(const char *name = NULL, int kind = 0) :
        SpsCandidateResources_Base(name, kind)
    {
    }

    ~SpsCandidateResources()
    {
    }

    SpsCandidateResources(const SpsCandidateResources& other)
    {
        operator=(other);
    }

    SpsCandidateResources& operator=(const SpsCandidateResources& other)
    {
        schedulingGrant = other.schedulingGrant;
        possibleCSRs = other.possibleCSRs;
        CSRStartTime = other.CSRStartTime;
        SpsCandidateResources_Base::operator=(other);
        return *this;
    }

    virtual SpsCandidateResources *dup() const
    {
        return new SpsCandidateResources(*this);
    }

    virtual void setPossibleCSRs(const std::vector<RbMap>& possibleCSRsList )
    {
        possibleCSRs = possibleCSRsList;
    }

    virtual std::vector<RbMap>& getPossibleCSRs()
    {
        return possibleCSRs;
    }

    virtual void setCSRStartTimes(const std::vector<simtime_t>& CSRStartTimesList)
    {
        CSRStartTimes = CSRStartTimesList;
    }

    virtual std::vector<simtime_t>& getCSRStartTimes()
    {
        return CSRStartTime;
    }

};
