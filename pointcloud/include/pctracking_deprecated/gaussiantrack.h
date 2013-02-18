#ifndef GAUSSIANTRACK_H
#define GAUSSIANTRACK_H

#include "gaussian.h"

class GaussianTrack
{
public:
    typedef struct{
        Gaussian gaussian;
        int time;
    } Frame;

public:
    GaussianTrack(int _id);

public:
    void insert(Gaussian *gaussian, int time);
    Frame lastFrame(){return frames.at(frames.size()-1);};

public:
    vector<Frame> frames;
    int id;
};

#endif // GAUSSIANTRACK_H
