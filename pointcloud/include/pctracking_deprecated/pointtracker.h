#ifndef POINTTRACKER_H
#define POINTTRACKER_H

#include "imft/imft.h"
#include "imft/imft.cpp"

typedef enum{
    SEP, MEG
} MODE;
typedef struct{
    pcl::PointXYZ measured;
    pcl::PointXYZ prediction;
} TrackPoint;

template<typename Ptr>
class PointTracker : public IMFT<Ptr>
{
public:    


    typedef IMFT<Ptr> Parent;
    typedef typename Parent::VecPtr VecPtr;

    PointTracker(int nWindow, bool isDebug = 0);

    bool twoGroupMatching(CloudPtr &pGroupA,  pcl::PointXYZ velA, CloudPtr &pGroupB, pcl::PointXYZ velB, CloudPtr &fGroup, Cloud &tGroupA, Cloud &tGroupB); // for group matching
    double twoGroupMatchingScore(CloudPtr &pGroupA, pcl::PointXYZ velA, CloudPtr &pGroupB); // for group matching score
    void setMode(MODE mode){m_mode=mode;}; // 1 : separation, 2 : merging

protected:
    double gain(typename Parent::V pNode, typename Parent::V cNode, double alpha = 0.5);
    void confirmDGraph();
    bool m_isDebug;
    MODE m_mode;
};

#endif // POINTTRACKER_H
