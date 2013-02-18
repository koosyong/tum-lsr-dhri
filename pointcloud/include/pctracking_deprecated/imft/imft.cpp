#ifndef IMFT_CPP
#define IMFT_CPP

#include "imft.h"
#include "stdio.h"
#include <lemon/matching.h>
#include <math.h>

template<typename Ptr>
IMFT<Ptr>::IMFT(int nWindow, bool isDebug)
{
    m_isDebug = isDebug;
    m_nWindow = nWindow;
    m_cntTrack = 0;
    cnt = 0;
    makeGraph();
}

template<typename Ptr>
IMFT<Ptr>::~IMFT()
{
    delete m_gNodeMap;
    delete m_gEdgeMap;
}

template<typename Ptr>
void IMFT<Ptr>::makeGraph()
{
    m_gNodeMap = new ListGraph::NodeMap<V>(m_g);
    m_gEdgeMap = new ListGraph::EdgeMap<double>(m_g);
}

template<typename Ptr>
void IMFT<Ptr>::setFrame(VecPtr ptrs)
{
    cnt++;
    if(m_isDebug)
        std::cout << "||CNT : " << cnt << " || # of objects : " << ptrs.size() << " || # of tracks : " << m_tracks.size() << " ||" <<std::endl;
    if(cnt > m_nWindow) movingWindow();
    addToDGraph(ptrs);
    tracking();
    if(cnt == m_nWindow) {
        backtracking();
        makeTracks();
    }
    else if(cnt > m_nWindow){
        trackUpdate();
//        setCurrentFrame(cnt);
    }

    setCurrentFrame(cnt);
    confirmDGraph();
}

template<typename Ptr>
void IMFT<Ptr>::setCurrentFrame(int n)
{
    for(int i=0;i<m_tracks.size();i++){
        m_tracks.at(i).setCurrentFrame(n);
    }
}

template<typename Ptr>
void IMFT<Ptr>::movingWindow()
{
    if(m_isDebug)   printf("moving window\n");
    vector<ListGraph::Node> vecDelNode;
    // erase cnt-m_nWindow+1 frame nodes
    for(ListGraph::NodeIt n(m_g); n != INVALID; ++n){
        if((*m_gNodeMap)[n].nFrame <= cnt-m_nWindow){
            if(!(*m_gNodeMap)[n].isIn && (*m_gNodeMap)[n].edgeID >= 0){
                // make edgeID of connected node -1
                ListGraph::Edge e = m_g.edgeFromId((*m_gNodeMap)[n].edgeID);
                ListGraph::Node v = m_g.v(e);
                (*m_gNodeMap)[v].edgeID = -1;
                if(m_isDebug)   printf("edge initialization (-1) of node # %d\n", m_g.id(v));
                // erase edge connected with the node
                m_g.erase(e);
                vecDelNode.push_back(n);
                vecDelNode.push_back(m_g.nodeFromId(m_g.id(n)-1));
            }
        }
    }
    for(int i=0;i<vecDelNode.size();i++){
        ListGraph::Node n = vecDelNode.at(i);
        if(m_isDebug)   printf("erase old node %d of frame %d\n",m_g.id(n), (*m_gNodeMap)[n].nFrame);
        if(!(*m_gNodeMap)[n].isIn && (*m_gNodeMap)[n].edgeID < 0){
            // delete track
            if(m_isDebug)   printf("delete track\n");
            ListGraph::Node u = m_g.nodeFromId(m_g.id(n)-1);
            if(m_isDebug)   printf("delete track of %d\n", (*m_gNodeMap)[u].nTrack);
            if((*m_gNodeMap)[u].isTrack){
                typename VecTrack::iterator it = m_tracks.begin();
                while(1){
                    if((*it).num() == (*m_gNodeMap)[u].nTrack){
                        if(m_isDebug)   printf("delete track of %d\n", (*m_gNodeMap)[u].nTrack);
                        (*it).clear();
                        m_tracks.erase(it);
                        break;
                    }
                    it++;
                    if(it > m_tracks.end()) break;
                }
            }
        }
        m_g.erase(n);
    }
}

template<typename Ptr>
void IMFT<Ptr>::addToDGraph(VecPtr ptrs)
{
    // save old edges
    m_vecOldEdge.clear();
    for(ListGraph::EdgeIt e(m_g); e != INVALID; ++e){
        m_vecOldEdge.push_back(m_g.id(e));
    }

    // making nodes
    for(int i=0;i<ptrs.size();i++){
        // inNode
        ListGraph::Node xi = m_g.addNode();
        V vi;
        vi.id = m_g.id(xi);
        vi.isIn = 1;
        vi.ptr = ptrs[i];
        vi.nFrame = cnt;
        vi.edgeID = -1;
        vi.isTrack = 0;
        vi.nTrack = 0;
        (*m_gNodeMap)[xi] = vi;

        // outNode
        ListGraph::Node xo = m_g.addNode();
        V vo;
        vo.id = m_g.id(xo);
        vo.isIn = 0;
        vo.ptr = ptrs[i];
        vo.nFrame = cnt;
        vo.edgeID = -1;
        vo.isTrack = 0;
        vo.nTrack = 0;
        (*m_gNodeMap)[xo] = vo;

        // connection to previous nodes
        for(ListGraph::NodeIt n(m_g); n != INVALID; ++n){
            if((*m_gNodeMap)[n].nFrame != vi.nFrame && !(*m_gNodeMap)[n].isIn){
                //                double weight = gain((*m_gNodeMap)[n], vi);
                double weight = gain((*m_gNodeMap)[m_g.nodeFromId(m_g.id(n)-1)], vi);

                //                m_g.nodeFromId(m_g.id((*m_gNodeMap)[n])-1);

                (*m_gEdgeMap)[m_g.addEdge(n,xi)] = weight;

                //                ListGraph::Edge e = m_g.addEdge(n,xi);
                //                (*m_gEdgeMap)[e] = weight;
                //                (*m_gNodeMap)[m_g.u(e)].edgeID = m_g.id(e);
                //                (*m_gNodeMap)[m_g.v(e)].edgeID = m_g.id(e);

            }
        }
    }
}
/*
template<typename Ptr>
double IMFT<Ptr>::gain(V pNode, V cNode)
{
    double weight;
    double dst;
    Track track;
    if(pNode.isTrack && (pNode.nFrame != cNode.nFrame)){
        for(int i=0;i<m_tracks.size();i++){
            if(m_tracks.at(i).num() == pNode.nTrack)
                track = m_tracks.at(i);
        }

        Ptr estPtr = track.estPtr(pNode.nFrame, cNode.nFrame);
        Ptr estVct, obsVct;
        estVct.x = estPtr.x - pNode.ptr.x;
        estVct.y = estPtr.y - pNode.ptr.y;
        estVct.z = estPtr.z - pNode.ptr.z;
        obsVct.x = cNode.ptr.x - pNode.ptr.x;
        obsVct.y = cNode.ptr.y - pNode.ptr.y;
        obsVct.z = cNode.ptr.z - pNode.ptr.z;

        dst = sqrt(pow(estVct.x - obsVct.x, 2) + pow(estVct.y - obsVct.y, 2)  + pow(estVct.z - obsVct.z, 2));
        double dstEst, dstObs;
        dstEst = sqrt(pow(estVct.x, 2) + pow(estVct.y, 2) + pow(estVct.z, 2));
        dstObs = sqrt(pow(obsVct.x, 2) + pow(obsVct.y, 2) + pow(obsVct.z, 2));

        if((dstEst == 0) || (dstObs == 0)){
            weight = 1 - dst/(sqrt(pow(SX,2) + pow(SY,2) + pow(SZ,2)));
        }
        else{
            weight = ALPHA*(1/2+(estVct.x*obsVct.x + estVct.y*obsVct.y  + estVct.z*obsVct.z)/(2*dstEst*dstObs)) + (1-ALPHA)*(1 - dst/(sqrt(pow(SX,2) + pow(SY,2) + pow(SZ,2))));
        }
//        printf("Frame : %d,weight: %f dstEst:%f, dstObs:%f pNodeFrame:%f, cNodeFrame:%f\n",cNode.nFrame, weight, dstEst, dstObs, estVct.x, estVct.y);
    }
    else{
        dst = (sqrt(pow(pNode.ptr.x - cNode.ptr.x, 2) + pow(pNode.ptr.y - cNode.ptr.y, 2) + pow(pNode.ptr.z - cNode.ptr.z, 2)));
        weight = 1 - dst/(sqrt(pow(SX,2) + pow(SY,2) + pow(SZ,2)));
//        printf("Frame : %d, OBS : %f\n",dst);
    }

    return weight;
}
*/



template<typename Ptr>
void IMFT<Ptr>::tracking()
{
    // maximum weighted matching
    MaxWeightedMatching<ListGraph, ListGraph::EdgeMap<double> > mwm(m_g, *m_gEdgeMap);
    mwm.run();
    double wsum = mwm.matchingWeight();
    if(m_isDebug)   printf("Max = %d\n", wsum);
    m_wsum = wsum;

    // make maximum path cover C
    /* for all edges
    // if it is not matched, delete edges, if it was old edge -> save oldEdge
    // if it is matched, liking edge to nodes  : save edge id to the connedted nodes
    // find correction edges : matching edges having a node of old edges
    */
    vector<ListGraph::Edge> vecCorEdge, vecDelOldEdge;
    for(ListGraph::EdgeIt e(m_g); e != INVALID; ++e){
        if(mwm.matching(e)){
            if(m_isDebug)   printf("Edge of Maximum Path Cover : %d\n", m_g.id(e));
            (*m_gNodeMap)[m_g.u(e)].edgeID = m_g.id(e);
            (*m_gNodeMap)[m_g.v(e)].edgeID = m_g.id(e);
            // track extension
            ListGraph::Node nodeT = m_g.nodeFromId(m_g.id(m_g.u(e))-1);
            if((*m_gNodeMap)[m_g.v(e)].nFrame == cnt){
                if((*m_gNodeMap)[nodeT].isTrack){
                    for(int i=0;i<m_tracks.size();i++){
                        if(m_tracks.at(i).num() == (*m_gNodeMap)[nodeT].nTrack){
                            m_tracks.at(i).putNode((*m_gNodeMap)[m_g.v(e)].ptr, m_g.id(m_g.v(e)), (*m_gNodeMap)[m_g.v(e)].nFrame);
                            (*m_gNodeMap)[m_g.v(e)].nTrack = (*m_gNodeMap)[nodeT].nTrack;
                            (*m_gNodeMap)[m_g.v(e)].isTrack = 1;
                            if(m_isDebug)
                                printf("At cnt %d, put node %d of frame %d to the track %d\n", cnt, m_g.id(m_g.v(e)), (*m_gNodeMap)[m_g.v(e)].nFrame, (*m_gNodeMap)[m_g.v(e)].nTrack);
                            break;
                        }
                    }
                }
                else{   // make new track
                    if(cnt > m_nWindow){
                        m_cntTrack ++;
                        Track track;
                        track.setNum(m_cntTrack);
                        track.putNode((*m_gNodeMap)[nodeT].ptr, m_g.id(nodeT), (*m_gNodeMap)[nodeT].nFrame);
                        (*m_gNodeMap)[nodeT].isTrack = 1;
                        (*m_gNodeMap)[nodeT].nTrack = m_cntTrack;
                        if(m_isDebug)   printf("Generate new track # %d of node %d\n", m_cntTrack, m_g.id(nodeT));
                        // add v(e) to the track
                        track.putNode((*m_gNodeMap)[m_g.v(e)].ptr, m_g.id(m_g.v(e)), (*m_gNodeMap)[m_g.v(e)].nFrame);
                        (*m_gNodeMap)[m_g.v(e)].isTrack = 1;
                        (*m_gNodeMap)[m_g.v(e)].nTrack = m_cntTrack;
                        m_tracks.push_back(track);
                    }
                }

            }

        }
        else{
            for(int i=0;i<m_vecOldEdge.size();i++){
                if(m_g.id(e) == m_vecOldEdge.at(i)) {   // edges deleted by correction edge
                    vecDelOldEdge.push_back(e);
                    (*m_gNodeMap)[m_g.v(e)].edgeID = -1;
                    break;
                }
            }
            m_g.erase(e);
        }
    }
    // false hypothesis
    /* for all correction edges
    // find false hypothesis : edges having a directed path from an edge replaced by a correction edge
    //                       : find a previous track having an edge replaced by a correction edge
    //                       : false hypotheses = edges of tracks
    // delete all false hypotheses
    */
    for(int i=0;i<vecDelOldEdge.size();i++){
        ListGraph::Edge delEdge = vecDelOldEdge.at(i);
        if(m_isDebug)
            printf("At cnt %d, edge %d is an old edge deleted by a correction edge\n",cnt, m_g.id(delEdge));
        // delete false hypotheses
        int vId = m_g.id(m_g.v(delEdge));

        if(m_isDebug)
            printf("delete node %d from the track %d\n", vId ,(*m_gNodeMap)[m_g.nodeFromId(vId)].nTrack);
        if(eraseNodeinTrack((*m_gNodeMap)[m_g.nodeFromId(vId)].nTrack, vId)){
            (*m_gNodeMap)[m_g.nodeFromId(vId)].nTrack = 0;
            (*m_gNodeMap)[m_g.nodeFromId(vId)].isTrack = 0;
        }

        ListGraph::Node uNode = m_g.nodeFromId(vId+1);
        int fhId = (*m_gNodeMap)[uNode].edgeID;
        while(fhId != -1){
            if(m_isDebug)
                printf("False Hypothsis %d is deleted\n", fhId);
            ListGraph::Edge fhEdge = m_g.edgeFromId(fhId);
            (*m_gNodeMap)[m_g.u(fhEdge)].edgeID = -2;   // check for a node connected with a deleted edge
            (*m_gNodeMap)[m_g.v(fhEdge)].edgeID = -2;   // check for a node connected with a deleted edge

            vId = m_g.id(m_g.v(fhEdge));
            if(m_isDebug)
                printf("At cnt %d, delete node %d(frame %d) from the track %d\n", cnt, vId, (*m_gNodeMap)[m_g.nodeFromId(vId)].nFrame, (*m_gNodeMap)[m_g.nodeFromId(vId)].nTrack);
            if(eraseNodeinTrack((*m_gNodeMap)[m_g.nodeFromId(vId)].nTrack, vId)){
                (*m_gNodeMap)[m_g.nodeFromId(vId)].nTrack = 0;
                (*m_gNodeMap)[m_g.nodeFromId(vId)].isTrack = 0;
            }
            uNode = m_g.nodeFromId(vId+1);
            fhId = (*m_gNodeMap)[uNode].edgeID;
            m_g.erase(fhEdge);
        }
    }
    // new edges
    /* if cnt < m_nWindow : for i=1 to cnt
       if cnt <= m_nWindow : for i=cnt-m_nWindow+1 to cnt
    // make two frame correspondance Vi ~ Vi+1
    */
    vector<ListGraph::Node> vecUFrame, vecVFrame;
    if(cnt < m_nWindow){
        for(int i=1; i<cnt; i++){
            vecUFrame.clear();
            vecVFrame.clear();
            // 2-frame corresponding
            for(ListGraph::NodeIt n(m_g); n != INVALID; ++n){
                //                if((*m_gNodeMap)[n].nFrame > i+1) break;
                if((*m_gNodeMap)[n].nFrame == i && !(*m_gNodeMap)[n].isIn && (*m_gNodeMap)[n].edgeID == -2)
                    vecUFrame.push_back(n);
                if((*m_gNodeMap)[n].nFrame == i+1 && (*m_gNodeMap)[n].isIn && (*m_gNodeMap)[n].edgeID == -2)
                    vecVFrame.push_back(n);
            }
            if(vecUFrame.size() != 0 && vecVFrame.size() != 0){
                if(m_isDebug)   printf("2-Frame Corresponding of %d and %d\n", i, i+1);
                twoFrameCorresponding(vecUFrame, vecVFrame);
            }
        }
    }
    else{
        for(int i=cnt-m_nWindow+1; i<cnt; i++){
            vecUFrame.clear();
            vecVFrame.clear();
            // 2-frame corresponding
            for(ListGraph::NodeIt n(m_g); n != INVALID; ++n){
                //                if((*m_gNodeMap)[n].nFrame > i+1) break;
                if((*m_gNodeMap)[n].nFrame == i && !(*m_gNodeMap)[n].isIn && (*m_gNodeMap)[n].edgeID == -2)
                    vecUFrame.push_back(n);
                if((*m_gNodeMap)[n].nFrame == i+1 && (*m_gNodeMap)[n].isIn && (*m_gNodeMap)[n].edgeID == -2)
                    vecVFrame.push_back(n);
            }
            if(vecUFrame.size() != 0 && vecVFrame.size() != 0){
                if(m_isDebug)   printf("2-Frame Corresponding of %d and %d\n", i, i+1);
                twoFrameCorresponding(vecUFrame, vecVFrame);
            }
        }
    }
}

template<typename Ptr>
void IMFT<Ptr>::twoFrameCorresponding(vector<ListGraph::Node> vecUFrame, vector<ListGraph::Node> vecVFrame)
{
    if(m_isDebug)   printf("2-Frame Corresponding : # F1 - %d, # F2 - %d\n", vecUFrame.size(), vecVFrame.size());
    // make graph, weight map
    ListGraph g;
    ListGraph::NodeMap<V> gNodeMap(g);
    ListGraph::EdgeMap<double> gEdgeMap(g);

    // make nodes of UFrame : save node id of UFrame
    for(int i=0;i<vecUFrame.size();i++){
        ListGraph::Node n = g.addNode();
        V v;
        v.id = m_g.id(vecUFrame.at(i));
        v.ptr = (*m_gNodeMap)[vecUFrame.at(i)].ptr;
        v.nFrame = 1;
        gNodeMap[n] = v;
    }
    // make nodes of VFrame : save node id of VFrame
    for(int i=0;i<vecVFrame.size();i++){
        ListGraph::Node n = g.addNode();
        V v;
        v.id = m_g.id(vecVFrame.at(i));
        v.ptr = (*m_gNodeMap)[vecVFrame.at(i)].ptr;
        v.nFrame = 2;
        gNodeMap[n] = v;

        // connection
        for(ListGraph::NodeIt pn(g); pn != INVALID; ++pn){
            if(gNodeMap[pn].nFrame != v.nFrame){
                double weight = gain(gNodeMap[pn], v);
                gEdgeMap[g.addEdge(pn,n)] = weight;

                //                ListGraph::Edge e = g.addEdge(pn,n);
                //                gEdgeMap[e] = weight;
                //                gNodeMap[m_g.u(e)].edgeID = g.id(e);
                //                gNodeMap[m_g.v(e)].edgeID = g.id(e);
            }
        }
    }

    // maximum weighted matching
    MaxWeightedMatching<ListGraph, ListGraph::EdgeMap<double> > mwm(g, gEdgeMap);
    mwm.run();
    int wsum = mwm.matchingWeight();
    if(m_isDebug)   printf("2-Frame Max = %d\n", wsum);

    // make edges of original graph using original nodes' ids
    for(ListGraph::EdgeIt e(g); e != INVALID; ++e){
        if(mwm.matching(e)){
            int origUId = gNodeMap[g.u(e)].id;
            int origVId = gNodeMap[g.v(e)].id;
            ListGraph::Node newU, newV;
            newU = m_g.nodeFromId(origUId);
            newV = m_g.nodeFromId(origVId);
            if(m_isDebug)   printf("2-Frame Connection %d, %d nodes\n", origUId, origVId);

            double weight = gain((*m_gNodeMap)[newU], (*m_gNodeMap)[newV]);
            ListGraph::Edge e = m_g.addEdge(newU,newV);
            (*m_gEdgeMap)[e] = weight;
            (*m_gNodeMap)[m_g.u(e)].edgeID = m_g.id(e);
            (*m_gNodeMap)[m_g.v(e)].edgeID = m_g.id(e);

            // if u가 track이 없으면, track 생성하고, v도 track에 집어 넣음.
            // u가 track이 있으면, v를 그 track에 집어 넣음.
            if(cnt > m_nWindow){
                int vId = m_g.id(m_g.u(e))-1;
                if(!(*m_gNodeMap)[m_g.nodeFromId(vId)].isTrack){
                    // generate a track of vId
                    m_cntTrack ++;
                    Track track;
                    track.setNum(m_cntTrack);
                    track.putNode((*m_gNodeMap)[m_g.nodeFromId(vId)].ptr, vId, (*m_gNodeMap)[m_g.nodeFromId(vId)].nFrame);
                    (*m_gNodeMap)[m_g.nodeFromId(vId)].isTrack = 1;
                    (*m_gNodeMap)[m_g.nodeFromId(vId)].nTrack = m_cntTrack;
                    if(m_isDebug)   printf("Generate new track # %d of node %d\n", m_cntTrack, vId);
                    // add v(e) to the track
                    track.putNode((*m_gNodeMap)[m_g.v(e)].ptr, m_g.id(m_g.v(e)), (*m_gNodeMap)[m_g.v(e)].nFrame);
                    (*m_gNodeMap)[m_g.v(e)].isTrack = 1;
                    (*m_gNodeMap)[m_g.v(e)].nTrack = m_cntTrack;
                    m_tracks.push_back(track);
                }
                else{
                    // add v(e) to the track
                    for(int i=0;i<m_tracks.size();i++){
                        if(m_tracks.at(i).num() == (*m_gNodeMap)[m_g.nodeFromId(vId)].nTrack){
                            m_tracks.at(i).putNode((*m_gNodeMap)[m_g.v(e)].ptr, m_g.id(m_g.v(e)), (*m_gNodeMap)[m_g.v(e)].nFrame);
                            (*m_gNodeMap)[m_g.v(e)].nTrack = (*m_gNodeMap)[m_g.nodeFromId(vId)].nTrack;
                            (*m_gNodeMap)[m_g.v(e)].isTrack = 1;
                            if(m_isDebug)   printf("put node %d to the track %d\n", m_g.id(m_g.v(e)), (*m_gNodeMap)[m_g.v(e)].nTrack);
                            break;
                        }
                    }
                }
            }
        }
    }
}

template<typename Ptr>
void IMFT<Ptr>::backtracking()
{
    if(m_isDebug) printf("Backtracking\n");
    // backtracking for frame 1-2 corresponding
    for(ListGraph::NodeIt n(m_g); n != INVALID; ++n){
        if((*m_gNodeMap)[n].nFrame == 1 && !(*m_gNodeMap)[n].isIn && (*m_gNodeMap)[n].edgeID >= 0){
            // erase initial edges
            if(m_isDebug)   printf("erase edge # %d\n", (*m_gNodeMap)[n].edgeID);
            ListGraph::Edge e;
            e = m_g.edgeFromId((*m_gNodeMap)[n].edgeID);
            ListGraph::Node v = m_g.v(e);
            (*m_gNodeMap)[n].edgeID = -1;
            (*m_gNodeMap)[v].edgeID = -1;
            if(m_isDebug)   printf("node initialization %d\n", m_g.id(v));
            m_g.erase(e);

            // make extension edges from initial edges
            // connection to previous nodes
            for(ListGraph::NodeIt np(m_g); np != INVALID; ++np){
                if((*m_gNodeMap)[np].nFrame != 1 && (*m_gNodeMap)[np].isIn && (*m_gNodeMap)[np].edgeID < 0){
                    double weight = gain((*m_gNodeMap)[np], (*m_gNodeMap)[n]);
                    ListGraph::Edge e = m_g.addEdge(n,np);
                    (*m_gEdgeMap)[e] = weight;
                    (*m_gNodeMap)[m_g.u(e)].edgeID = m_g.id(e);
                    (*m_gNodeMap)[m_g.v(e)].edgeID = m_g.id(e);
                }
            }
        }
    }
    confirmDGraph();
    // maximum weighted matching
    MaxWeightedMatching<ListGraph, ListGraph::EdgeMap<double> > mwm(m_g, *m_gEdgeMap);
    mwm.run();
    int wsum = mwm.matchingWeight();
    if(m_isDebug)   printf("Max = %d\n", wsum);

    // make maximum path cover C
    /* for all edges
    // if it is not matched, delete edges, if it was old edge -> save oldEdge
    // if it is matched, liking edge to nodes  : save edge id to the connedted nodes
    // find correction edges : matching edges having a node of old edges
    */

    for(ListGraph::EdgeIt e(m_g); e != INVALID; ++e){
        if(mwm.matching(e)){
            if(m_isDebug)   printf("Edge of Maximum Path Cover : %d\n", m_g.id(e));
            (*m_gNodeMap)[m_g.u(e)].edgeID = m_g.id(e);
            (*m_gNodeMap)[m_g.v(e)].edgeID = m_g.id(e);
        }
        else{
            (*m_gNodeMap)[m_g.v(e)].edgeID = -1;
            m_g.erase(e);
        }
    }
    confirmDGraph();
    // save old edges
    m_vecOldEdge.clear();
    for(ListGraph::EdgeIt e(m_g); e != INVALID; ++e){
        m_vecOldEdge.push_back(m_g.id(e));
    }
}

template<typename Ptr>
void IMFT<Ptr>::makeTracks()
{
    m_tracks.clear();
    for(ListGraph::NodeIt n(m_g); n != INVALID; ++n){
        if(m_g.id(n)%2 == 0 && !(*m_gNodeMap)[n].isTrack){
            // make a new track
            m_cntTrack ++;

            Track track;
            track.setNum(m_cntTrack);

            track.putNode((*m_gNodeMap)[n].ptr, m_g.id(n), (*m_gNodeMap)[n].nFrame);
            (*m_gNodeMap)[n].isTrack = 1;
            (*m_gNodeMap)[n].nTrack = m_cntTrack;

            if(m_isDebug)   printf("start node of track %d: %d\n", cnt, m_g.id(n));
            // find track
            int eId = (*m_gNodeMap)[n].edgeID;
            while(eId >= 0){
                ListGraph::Edge e = m_g.edgeFromId(eId);
                int uId = m_g.id(m_g.u(e));
                ListGraph::Node vNode = m_g.nodeFromId(uId-1);
                if(m_isDebug)   printf("node %d is added\n", uId-1);

                track.putNode((*m_gNodeMap)[vNode].ptr, m_g.id(vNode), (*m_gNodeMap)[vNode].nFrame);
                (*m_gNodeMap)[vNode].isTrack = 1;
                (*m_gNodeMap)[vNode].nTrack = m_cntTrack;

                eId = (*m_gNodeMap)[vNode].edgeID;
            }
            // reverse track;
            Track nTrack;
            for(int i=track.size()-1;i>=0;i--){
                nTrack.setNum(track.num());
                nTrack.putNode(track.ptrAt(i), track.nodeAt(i), track.frameAt(i));
            }
            m_tracks.push_back(nTrack);
        }
    }
    //    for(ListGraph::NodeIt n(m_g); n != INVALID; ++n){
    //        (*m_gNodeMap)[n].isTrack = 0;
    //    }
}


template<typename Ptr>
void IMFT<Ptr>::trackUpdate()
{
    // Edge 만드는 순서
    // extension edge 만들고,
    // correction edge 찾고,
    // edge 지우고.
    // 새로운 edge 생성하고..
    // extension edge 안에, FH 등 포함되어 있음.
    // 최종 edge 는..
    // correction edge : correction 뒤에 붙은 point, FH point 삭제
    //
    // extension : 이전 track들의 terminal 뒤에 붙임

    // extension edges

    // delete correction edges
    // make new edges
}

template<typename Ptr>
bool IMFT<Ptr>::eraseNodeinTrack(int nTrack, int nodeId)
{
    for(int i=0;i<m_tracks.size();i++){
        if(m_tracks.at(i).num() == nTrack){
            if(m_tracks.at(i).eraseNode(nodeId, m_nWindow)){
                if(m_isDebug)   printf("Node %d is deleted from the track %d\n", nodeId, nTrack);
                return 1;
            }
            else{
                if(m_isDebug)   printf("WARNING!! Node %d is failed to be deleted from the track %d\n", nodeId, nTrack);
                return 0;
            }
        }
    }
}

template<typename Ptr>
bool IMFT<Ptr>::deleteTrack(int num)
{
    vector<ListGraph::Node> vecDelNode;
    for(ListGraph::NodeIt n(m_g); n != INVALID; ++n){
        if((*m_gNodeMap)[n].nTrack == num){
            ListGraph::Node node = m_g.nodeFromId(m_g.id(n)+1);
            if(!(*m_gNodeMap)[node].isIn){
                vecDelNode.push_back(node);
                vecDelNode.push_back(m_g.nodeFromId(m_g.id(node)-1));
            }
        }
    }
    if(m_isDebug)   printf("deleting %d of nodes\n", vecDelNode.size());
    for(int i=0;i<vecDelNode.size();i++){
        ListGraph::Node n = vecDelNode.at(i);
        m_g.erase(n);
        if(m_isDebug)   printf("node %d is deleted\n", (*m_gNodeMap)[n].id);
    }
    typename VecTrack::iterator it = m_tracks.begin();
    while(1){
        if((*it).num() == num){
            if(m_isDebug)   printf("delete track of %d\n", num);
            (*it).clear();
            m_tracks.erase(it);
            break;
        }
        it++;
        if(it > m_tracks.end()) break;
    }
}

template<typename Ptr>
std::vector<Ptr> IMFT<Ptr>::getUnmatchedPtrs()
{
    VecPtr unmatchedPtrs;
    if(cnt < m_nWindow) return unmatchedPtrs;
    for(ListGraph::NodeIt n(m_g); n != INVALID; ++n){
        if((*m_gNodeMap)[n].nFrame == cnt && (*m_gNodeMap)[n].isIn){
            int eId = (*m_gNodeMap)[n].edgeID;
            if(eId < 0){
                unmatchedPtrs.push_back((*m_gNodeMap)[n].ptr);
            }
        }
    }
    return unmatchedPtrs;
}

template<typename Ptr>
void IMFT<Ptr>::deleteNodeInLastFrame(Ptr ptr)
{
    vector<ListGraph::Node> vecDelNode;
    for(ListGraph::NodeIt n(m_g); n != INVALID; ++n){
        if((*m_gNodeMap)[n].nFrame == cnt && !(*m_gNodeMap)[n].isIn){
            if((*m_gNodeMap)[n].ptr.id == ptr.id){
                ListGraph::Node node = m_g.nodeFromId(m_g.id(n)-1);
                if((*m_gNodeMap)[node].edgeID >= 0){
                    // make edgeID of connected node -1
                    ListGraph::Edge e = m_g.edgeFromId((*m_gNodeMap)[node].edgeID);
                    ListGraph::Node v = m_g.u(e);
                    (*m_gNodeMap)[v].edgeID = -1;
                    if(m_isDebug)   printf("edge initialization (-1) of node # %d\n", m_g.id(v));
                    // erase edge connected with the node
                    m_g.erase(e);
                }
                if(m_isDebug) std::cout << "Delete node " << m_g.id(n) << " and "<< m_g.id(n)+1<<  std::endl;
                vecDelNode.push_back(n);
                vecDelNode.push_back(m_g.nodeFromId(m_g.id(n)-1));
            }
        }
    }

    if(m_isDebug)   printf("deleting %d of nodes\n", vecDelNode.size());
    for(int i=0;i<vecDelNode.size();i++){
        ListGraph::Node n = vecDelNode.at(i);
        m_g.erase(n);
        if(m_isDebug)   printf("node %d is deleted\n", (*m_gNodeMap)[n].id);
        // delete track if the object has
        if((*m_gNodeMap)[n].nTrack != 0){
            for(int j=0;j<m_tracks.size();j++){
                if(m_tracks.at(j).num() == (*m_gNodeMap)[n].nTrack)
                    m_tracks.at(j).pop_back();
            }
        }
    }

}

template<typename Ptr>
void IMFT<Ptr>::updatePtrsInLastFrame(VecPtr ptrs)
{
    for(ListGraph::NodeIt n(m_g); n != INVALID; ++n){
       if((*m_gNodeMap)[n].nFrame == cnt){
           for(int j=0;j<ptrs.size();j++){
               if((*m_gNodeMap)[n].ptr.id = ptrs.at(j).id)
                   (*m_gNodeMap)[n].ptr = ptrs.at(j);
           }
       }
    }
}

#endif // IMFT_CPP
