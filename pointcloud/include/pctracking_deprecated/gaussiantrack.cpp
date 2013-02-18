#include "gaussiantrack.h"

GaussianTrack::GaussianTrack(int _id)
    :id(_id)
{
}

void GaussianTrack::insert(Gaussian *gaussian, int time)
{
    Frame frame;
    frame.gaussian = *gaussian;
    frame.time = time;
    frames.push_back(frame);
}
