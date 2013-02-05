#ifndef GAUSSIANTRACKCONTAINER_H
#define GAUSSIANTRACKCONTAINER_H

//#include "gaussiantrack.h"
#include "gaussian.h"
#include "trackcontainer.h"

class GaussianTrackContainer : public TrackContainer<Gaussian>
{
public:
    GaussianTrackContainer(int _maxFrame = 1000);
    ~GaussianTrackContainer();
};

#endif // GAUSSIANTRACKCONTAINER_H
