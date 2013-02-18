#ifndef IMFT_H
#define IMFT_H

#include <vector>
using namespace std;

#include <lemon/list_graph.h>
#include <lemon/matching.h>
using namespace lemon;

#include "trackcontainer.h"

//************************************************************* cafeful~!! ***********************
// you have to correct updateTracks() function
// in the case of false hypothesis,
// the tracks having false edges are not deleted in trackcontainer
//************************************************************* cafeful~!! ***********************

template<class Object>
class IMFT
{
public:
    typedef TrackContainer<Object> TrackContainerT;
    typedef Track<Object> TrackT;
    typedef typename TrackT::Frame Ptr;
    typedef vector<Ptr> VecPtr;
    typedef typename TrackT::V V;
    typedef vector<TrackT> VecTrack;
    typedef double(*funcWeight)(Object &o1, Object &o2);
    typedef Object* ObjectPtr;
    typedef vector<Object*> VecObjectPtr;

public:
    IMFT(int _window_short = 10, int _window_long = 20, int _maxID=100, funcWeight _weight = 0, funcWeight _weight_fast = 0);
    ~IMFT();

public:
    void setFrame(vector<Object*> objects, int stamp);
    void confirmDGraph();
    void extension();
    void matching();
    void updateTracks();
    TrackContainerT* extractTracks();
    VecObjectPtr getUnmatchedObjects();
    VecObjectPtr getTerminalNodes();
    VecObjectPtr getTerminalNodesLastFrame();
    VecObjectPtr getUnmatchedTracks();
    void getMaximumMatchedTrack(ObjectPtr object, ObjectPtr &maxTrack, double &wHypothesis, ObjectPtr &objectOrigin, double &wOrigin);
    void getMaximumMatchedObject(ObjectPtr trackUnmatched, ObjectPtr &maxObject, double &wHypothesis, ObjectPtr &trackOrigin, double &wOrigin);
//    void filtering();
    bool deleteLastFrame();

    funcWeight weight, weight_fast;
    TrackContainerT *trackContainer;
    int cnt;    

private:
    void movingWindow();
    void addToDGraph(VecPtr ptrs);
    void twoFrameCorresponding(vector<ListGraph::Node> vecUFrame, vector<ListGraph::Node> vecVFrame);

public:
    int window_short, window_long;
    int maxID;

    bool m_isDebug;
    ListGraph m_g;
    ListGraph::NodeMap<V> *m_gNodeMap;
    ListGraph::EdgeMap<double> *m_gEdgeMap;
    double m_maxWeight;
    vector<int> m_vecOldEdge;
    double m_wsum;
    int m_currentT;
    int m_newTrackID;


};

#include "imft.hpp"
#endif // IMFT_H
