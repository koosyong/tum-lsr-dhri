/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Telerobotics and Control Laboratory, KAIST (http://robot.kaist.ac.kr)
 *  Author : Koosy (http://koosy.blogspot.com, koosyong@gmail.com)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef IMFT_H
#define IMFT_H

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

// good for circle and random
//#define SX 80
//#define SY 80
//#define ALPHA 0.05

#define SX 0.1
#define SY 0.1
#define SZ 0.1
#define ALPHA 0.05

using namespace lemon;
using namespace std;

template<typename Ptr>
class IMFT
{
public:
    IMFT(int nWindow, bool isDebug = 0);
    ~IMFT();

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

#endif // IMFT_H
