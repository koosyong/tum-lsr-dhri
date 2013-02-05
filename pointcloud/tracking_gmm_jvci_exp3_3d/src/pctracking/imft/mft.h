#ifndef MFT_H
#define MFT_H


#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif

#include <lemon/list_graph.h>
#include <vector>
#include "mftrack.h"
#include "mftrack.cpp"  // for linking template definition


#define SX 0.1
#define SY 0.1
#define SZ 0.1
#define ALPHA 0.05

using namespace lemon;
using namespace std;

template<typename Ptr>
class MFT
{
public:
    MFT(int nWindow, bool isDebug = 0);
    ~MFT();

public:
    typedef std::vector<Ptr> VecPtr;
    typedef std::vector<VecPtr> VecFrame;
    typedef MFTrack<Ptr> Track;
    typedef vector<Track> VecTrack;
    typedef typename Track::V V;

public:
    void setFrame(VecPtr ptrs);
    VecTrack* getTracks(){return &m_tracks;};
    bool deleteTrack(int num);
    double getWeightSum(){return m_wsum;};
    void setDebug(bool isOn){m_isDebug = isOn;};
    std::vector<Ptr> getUnmatchedPtrs();
    void deleteNodeInLastFrame(Ptr ptr);
    void updatePtrsInLastFrame(VecPtr ptrs);

protected:
    virtual double gain(V pNode, V cNode, double alpha = 0.5) = 0;
    virtual void confirmDGraph() = 0;

protected:
    void setCurrentFrame(int n);
    void makeGraph();
    void splitGraph();
    void addToDGraph(VecPtr ptrs);

    void movingWindow();
    void tracking();
    void backtracking();
    void twoFrameCorresponding(vector<ListGraph::Node> vecUFrame, vector<ListGraph::Node> vecVFrame);
    void makeTracks();
    void trackUpdate();
    bool eraseNodeinTrack(int nTrack, int nodeId);
protected:
    ListGraph m_g;
    ListGraph::NodeMap<V> *m_gNodeMap;
    ListGraph::EdgeMap<double> *m_gEdgeMap;
    vector<int> m_vecOldEdge;
//    VecTrack m_tracks;
    VecTrack m_tracks;
    int cnt;
    int m_nWindow;
    bool m_isDebug;
    int m_cntTrack;
    double m_wsum;
};

#endif // MFT_H
