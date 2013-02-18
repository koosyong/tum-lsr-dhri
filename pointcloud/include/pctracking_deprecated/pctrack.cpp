#include "pctrack.h"

PCTrack::PCTrack(int _id)
    :id(_id)
{
}

void PCTrack::insert(PCObject *object, int time)
{
    Frame frame;
    frame.object = *object;
    frame.time = time;
    frames.push_back(frame);
}
