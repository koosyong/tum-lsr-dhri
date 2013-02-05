#ifndef PCTRACKING_H
#define PCTRACKING_H

#include "pcobjectcontainer.h"
#include "pctrackcontainer.h"
#include "imft.h"

#include "pcgmmreg.h"
//#include "gmmfiltering.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/tracking/tracker.h>

#define SQR(X)  ((X)*(X))
#define pi 3.141592


typedef vector<PCObject*> VecObjectPtr;
typedef vector<PCObject> VecObject;
typedef struct{
    VecObjectPtr models;
    PCObject* scene;
    VecObject trasformedModels;
} Scene;

typedef struct{
    int pointID;
    int trackID;
} TrackPoint;

typedef struct{
    vector<TrackPoint> trackPoints;
    int time;
} Frame;



class PCTracking
{
public:
    PCTracking(ros::Publisher* _pub_scene, ros::Publisher* _pub_model, FuncPub _funcPub);
    ~PCTracking();

public:
    void run(CloudPtr pCloud);

private:
    void segmentationGMM(CloudPtr pCloud, PCObjectContainer& objects);
    void pointMatching(PCObjectContainer& predictiveObjects, CloudPtr pCloud, CloudPtr unmatchedPoints, vector<CloudPtr>& observedPointsList);
    void registrationGMMs(PCObjectContainer& oldObjects, CloudPtr pCloud, PCObjectContainer& registeredObjects);
    void trackingGaussians(PCObject* object, CloudPtr observedPoints, PCObjectContainer& updatedObjects);

private:
    ros::Publisher* pub_scene;
    ros::Publisher* pub_model;
    FuncPub funcPub;
//    static double maxDist;

public:
    IMFT<PCObject> *imft;
    PCTrackContainer *tracks;

private:
    double scale;
    double percent;
    int dim;
    double segmentation_tolerance;
    double segmentation_minSize;
    double segmentation_maxSize;
    double maxProbAssociation;
    double thrProbScene;
    int minPoints;

};

#endif // PCTRACKING_H
