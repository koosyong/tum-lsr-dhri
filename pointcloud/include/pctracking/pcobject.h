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
#include <boost/shared_ptr.hpp>
using namespace boost;

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
public:
    PCObject();
    PCObject(int _id);
    ~PCObject();

public:
    void insert(Point p);
    Point getCentroid();
    void initialGMM(double _scale, double _percent);
    void initialGMM(double _scale);
    double evalGMM(Point x);
    double evalClosestGMM(Point x);
    double evalNormedGMM(Point x, double den);
    void simplify(SIMPLE method, double ratio, int nCluster=0);
    double L2ofGMMandPoints(double scale);
    void setTransParam(vnl_vector<double> param);
    void mergeTwoGMMs(PCObject* gmm1, PCObject* gmm2);
    void setScale(double _scale){scale = _scale;};

    void gaussianTrackingInit(int _window_short, int _window_long, int _maxID, funcWeightGaussian _weight_gaussian, funcWeightGaussian _weight_gaussian_fast);
    void gaussianTracking();
    void updatePredictiveParameters();
    void calculateVelocity();
    void makeTopology();
    double topology_weight(Gaussian g1, Gaussian g2);
    int componentGraph(vector<PCObject> &newObjects);
    void updateEdge();
public:
    vector<Point> points;
    vector<Gaussian> gmm;

    int id;
    vnl_vector<double> trans_param;
    bool isParamExist;

    shared_ptr< IMFT<Gaussian> > imftGaussians;
    IMFT<Gaussian>::TrackContainerT gaussianTracks;
    int cnt;
    vector< shared_ptr<Gaussian> > frame;

public: // topology
    shared_ptr<ListGraph> topology_graph;
    shared_ptr< ListGraph::NodeMap<Gaussian> > topology_nodeMap;
    shared_ptr< ListGraph::EdgeMap<double> > topology_edgeMap;
    vector<Edge> edges;
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
