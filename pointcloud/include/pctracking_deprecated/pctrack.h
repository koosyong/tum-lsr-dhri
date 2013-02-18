#ifndef PCTRACK_H
#define PCTRACK_H

#include "pcobject.h"

class PCTrack
{    
public:
    typedef struct{
        PCObject object;
        int time;
    } Frame;

public:
    PCTrack(int _id);
    void insert(PCObject *object, int time);
    Frame lastFrame(){return frames.at(frames.size()-1);};

public:
    vector<Frame> frames;
    int id;
};

#endif // PCTRACK_H
