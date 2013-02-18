#ifndef TRACKCONTAINER_H
#define TRACKCONTAINER_H

#include "track.h"
#include <vector>
using namespace std;

#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>
using namespace boost;

template<class Object>
class TrackContainer
{
    typedef Track<Object> TrackT;
    typedef vector< shared_ptr<TrackT> > VecTrackPtr;
public:
    TrackContainer(int _maxFrame = 1000, int _maxID = 1000);
    ~TrackContainer();

public:
    inline int numTracks(){return tracks.size();};
    bool createTrack(Object &object, int initT);
    bool createTrack(Object &object, int initT, int id);
    bool merge(TrackContainer<Object> container);
    bool push_back_track(int id, Object &object, int time);
    void deleteNoUpdatedTracks(int size);
    int newId();
    bool updateObject(int id, int time, Object &object);

    vector<int> deletedTrackIDs;

public:
    VecTrackPtr tracks;
    int currentT;

public:
    int maxFrame;
    int oldCnt;
    int maxID;
    shared_array<int> r;
    shared_array<int> g;
    shared_array<int> b;
};

#include "trackcontainer.hpp"
#endif // TRACKCONTAINER_H
