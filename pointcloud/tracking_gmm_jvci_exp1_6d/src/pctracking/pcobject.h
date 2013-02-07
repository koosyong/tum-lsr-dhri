#ifndef PCOBJECT_H
#define PCOBJECT_H

#include "vector"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"


#include "opencv2/flann/flann.hpp"
#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>

#include <iostream>

#include <lemon/connectivity.h>
//using namespace lemon;

#include "gaussian.h"
#include "imft.h"
#include "gaussiantrackcontainer.h"

typedef enum{
    SIMPLE_HCKL, SIMPLE_HCL2, SIMPLE_FA
} SIMPLE;


class PCObject
{
public:
    typedef double(*funcWeightGaussian)(Gaussian &g1, Gaussian &g2);
    typedef struct{
        Point u;
        Point v;
        double weight;
    }Edge;
    typedef struct{
        ListGraph::Node u;
        ListGraph::Node v;
        double weight_vel;
        double weight_pos;
    }Edge_spatial;

public:
    PCObject();    
    PCObject(int _id);
    ~PCObject();

public:
    void insert(Point p);
    Point getCentroid();
    void initialGMM(double _scale, double _percent);
//    void initialGMM(double _scale);
    double evalGMM(Point x);
//    double evalClosestGMM(Point x);
    double evalNormedGMM(Point x, double den);
    void simplify(int dim, SIMPLE method, double ratio, int nCluster=0);
//    double L2ofGMMandPoints(double scale);
    void setTransParam(vnl_vector<double> param);
    void mergeTwoGMMs(PCObject* gmm1, PCObject* gmm2);
    void setScale(double _scale){scale = _scale;};

    void gaussianTrackingInit(int _window_short, int _window_long, int _maxID, funcWeightGaussian _weight_gaussian, funcWeightGaussian _weight_gaussian_fast);
    void gaussianTracking();
    void updatePredictiveParameters();
    void calculateVelocity();
    void makeTopology();
    double topology_weight(Gaussian g1, Gaussian g2);
    double topology_posweight_rev(Gaussian g1, Gaussian g2);
    double topology_velweight_rev(Gaussian g1, Gaussian g2);
    int componentGraph(vector<PCObject> &newObjects);
public:
    vector<Point> points;
    vector<Gaussian> gmm;

    int id;
    vnl_vector<double> trans_param;
    bool isParamExist;

    IMFT<Gaussian> *imftGaussians;
    GaussianTrackContainer *gaussianTracks;
    int cnt;
    vector<Gaussian*> frame;

public: // topology
    ListGraph* topology_graph;
    ListGraph::NodeMap<Gaussian> *topology_nodeMap;
    ListGraph::EdgeMap<double> *topology_edgeMap;
    vector<Edge> edges;
    double alpha;
    double th_edge;
//    double avgWeight;
    double *diffWeight;
    double filteredWeight;

public:
    double scale;
    double percent;

private:
    int window_short;
    int window_long;

};

#endif // PCOBJECT_H