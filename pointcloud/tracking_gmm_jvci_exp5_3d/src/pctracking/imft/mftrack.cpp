#ifndef MFTRACK_CPP
#define MFTRACK_CPP

#include "mftrack.h"
#include <math.h>

template<typename Ptr>
MFTrack<Ptr>::MFTrack()
{
    m_cFrame = 0;
}

template<typename Ptr>
MFTrack<Ptr>::~MFTrack()
{
    m_nodes.clear();
}

template<typename Ptr>
int MFTrack<Ptr>::findId(int id)
{

}

template<typename Ptr>
void MFTrack<Ptr>::putNode(Ptr ptr, int nodeId, int frame)
{
    Node n;
    n.ptr = ptr;
    n.nodeId = nodeId;
    n.frame = frame;
    m_nodes.push_back(n);
}

template<typename Ptr>
bool MFTrack<Ptr>::eraseNode(int nodeId, int nWindow)
{
    typename vector<Node>::iterator it = m_nodes.end();
    it--;
    int cnt = 0;
    while(1){
        cnt ++;
        if((*it).nodeId == nodeId){
            m_nodes.erase(it);
            return 1;
        }
        it--;
        if(it < m_nodes.begin() || cnt >= nWindow) {
            return 0;
        }
    }
}

template<typename Ptr>
bool MFTrack<Ptr>::isExtended()
{
//    std::cout <<"track " << num() << " " << m_cFrame << " " << m_nodes.at(m_nodes.size()-1).frame << std::endl;
    if (m_nodes.at(m_nodes.size()-1).frame == m_cFrame)
        return 1;
    else return 0;
}

template<typename Ptr>
void MFTrack<Ptr>::clear()
{
    m_nodes.clear();
}

template<typename Ptr>
void MFTrack<Ptr>::setCloud(CloudPtr cloudPtr)
{
    m_cloud.swap(*cloudPtr);
}

template<typename Ptr>
void MFTrack<Ptr>::setPtr(Ptr ptr)
{
    m_nodes.at(m_nodes.size()-1).ptr = ptr;
}

template<typename Ptr>
void MFTrack<Ptr>::pop_back()
{
    if(isExtended())
        m_nodes.pop_back();
    m_cFrame = m_cFrame-1;

}

/*
template<typename Ptr>
Ptr MFTrack<Ptr>::estPtr(int atFrame, int toFrame)
{
    Ptr estPtr;
    Ptr atPtr, prevPtr;
    for(int i=1;i<m_nodes.size();i++){
        if(m_nodes.at(i).frame == atFrame){
            atPtr = m_nodes.at(i).ptr;
            prevPtr = m_nodes.at(i-1).ptr;
            break;
        }
    }
    estPtr.x = (atPtr.x - prevPtr.x)*sqrt(toFrame-atFrame) + atPtr.x;
    estPtr.y = (atPtr.y - prevPtr.y)*sqrt(toFrame-atFrame) + atPtr.y;
    estPtr.z = (atPtr.y - prevPtr.z)*sqrt(toFrame-atFrame) + atPtr.z;
    return estPtr;
}
*/
#endif // MFTRACK_CPP
