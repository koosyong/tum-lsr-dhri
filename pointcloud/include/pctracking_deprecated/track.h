#ifndef TRACK_H
#define TRACK_H

template<class Object>
class Track
{
public:
    typedef struct{
        Object object;
        int id;
        int time;
    } Frame;
    typedef struct{
        Frame ptr;
        int frame;
        int nodeId;
    } Node;
    typedef struct{
        int id;
        bool isIn;
        int nFrame;
        int edgeID;
        Frame ptr;
        bool isTrack;
        int nTrack;
    } V;

public:
    Track(int _id);
    void insert(Object &object, int time);
    Frame lastFrame(){return frames.at(frames.size()-1);};
    Frame getFrameFromLast(int n){return frames.at(frames.size()-1-n);};
    void updateObjectAtFrame(int time, Object& object);
    void terminate();

public:
    vector<Frame> frames;
    int id;
};

#include "track.hpp"
#endif // TRACK_H
