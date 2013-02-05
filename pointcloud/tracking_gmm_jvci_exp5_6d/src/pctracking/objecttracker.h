#ifndef OBJECTTRACKER_H
#define OBJECTTRACKER_H

#include "imft/imft.h"
#include "imft/imft.cpp"
#include "imft/mft.h"
#include "imft/mft.cpp"

template<typename Ptr>
class ObjectTracker : public IMFT<Ptr>
{
public:
    typedef IMFT<Ptr> Parent;
    typedef typename Parent::VecPtr VecPtr;
    typedef typename Parent::V V;
    typedef typename Parent::VecTrack VecTrack;
public:
    ObjectTracker(int nWindow, bool isDebug = 0);
    double gain(Ptr pPtr, Ptr cPtr, double alpha = 0.5);
    void eraseLastFrame();
    void twoFrameMatching(VecPtr frame1, VecPtr frame2);
    VecTrack* getMatchedTracks(){return &m_matchedTracks;};

protected:
    double gain(V pNode, V cNode, double alpha = 0.5);
    void confirmDGraph();

    VecTrack m_matchedTracks;

};

#endif // OBJECTTRACKER_H
