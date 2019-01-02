//
//                           SimuLTE
//
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself,
// and cannot be removed from it.
//


//
// TODO generated message class
//
packet LteSchedulingGrant
{
    @customize(true);  // see the generated C++ header for more info
    bool periodic;
       unsigned int period;
       int expiration;
    // blocks granted on all Remotes, all Bands
    unsigned int totalGrantedBlocks;
    // number of codewords
    unsigned int codewords;
    // granted bytes per codeword
    abstract unsigned int grantedCwBytes[];

    //#  in LteSchedulingGrant.h
    //# const UserTxParams* userTxParams;
    //# RbMap grantedBlocks;
    //#
}

packet LteMode4SchedulingGrant extends LteSchedulingGrant
{
    @customize(true);
    
    bool retransmission;
    unsigned int timeGapTransRetrans;
    unsigned int spsPriority;
    unsigned int numSubchannels;
    unsigned int maximumLatency;
    unsigned int startingSubchannel;
    unsigned int mcs;
    unsigned int startingSubchannelRetrans; // It is possible the retransmission has different resources assigned to it.
    unsigned int resourceReselectionCounter;
}