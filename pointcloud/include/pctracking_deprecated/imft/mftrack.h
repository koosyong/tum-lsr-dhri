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
#ifndef MFTRACK_H
#define MFTRACK_H

#include <vector>
#include "ptrheader.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pctheader.h"

using namespace std;

//typedef struct{
//    double x;
//    double y;
//} Ptr;

template<class Ptr>
class MFTrack
{
public:
    MFTrack();
    ~MFTrack();

    typedef vector<Ptr> VecPtr;
    typedef struct{
        Ptr ptr;
        int frame;
        int nodeId;
    } Node;
    typedef struct{
        int id;
        bool isIn;
        int nFrame;
        int edgeID;
        Ptr ptr;
        bool isTrack;
        int nTrack;
    } V;

public:
    int findId(int id); // return -1 if there's no id
    int num() { return m_num;}
    int setNum(int n){ m_num = n;};
    int setCurrentFrame(int cFrame){ m_cFrame = cFrame;};
    int size() { return m_nodes.size();};
    Ptr ptrAt(int i){ return m_nodes.at(i).ptr;};
    Ptr ptrAtBack(int i){ return m_nodes.at(m_nodes.size()-i-1).ptr;};
    Ptr ptrLast(){ return m_nodes.at(m_nodes.size()-1).ptr;};
    int nodeAt(int i){ return m_nodes.at(i).nodeId;};
    int frameAt(int i){ return m_nodes.at(i).frame;};
    bool isExtended();
    void putNode(Ptr ptr, int nodeId, int frame);
    bool eraseNode(int nodeId, int nWindow);
    void clear();
    void pop_back();

    void setCloud(CloudPtr cloudPtr);
    void setPtr(Ptr ptr);
    Cloud& getCloud(){ return m_cloud;};
    CloudPtr getCloudPtr(){ return (CloudPtr)&m_cloud;};
//    Ptr estPtr(int atFrame, int toFrame);

private:
    vector<Node> m_nodes;
    int m_nWindow;
    int m_num;
    int m_cFrame;

    Cloud m_cloud;
};


#endif // MFTRACK_H
