#ifndef TRACKCONTAINER_H
#define TRACKCONTAINER_H

#include "track.h"
#include <vector>
using namespace std;

template<class Object>
class TrackContainer
{
    typedef Track<Object> TrackT;
    typedef vector<TrackT*> VecTrackPtr;
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

protected:
    int maxFrame;
    int maxID;
    int *r, *g, *b;
};

#include "trackcontainer.hpp"
#endif // TRACKCONTAINER_H
