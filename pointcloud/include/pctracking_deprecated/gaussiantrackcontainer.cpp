#include "gaussiantrackcontainer.h"

GaussianTrackContainer::GaussianTrackContainer(int _maxFrame)
    :TrackContainer<Gaussian>::TrackContainer(_maxFrame)
{
}

GaussianTrackContainer::~GaussianTrackContainer()
{
//    TrackContainer<Gaussian>::~TrackContainer();
//    if(b != NULL) {delete[] b; b = NULL; }
//    if(r != NULL) {delete[] r; r = NULL; }
//    if(g != NULL) {delete[] g; g = NULL; }
}
