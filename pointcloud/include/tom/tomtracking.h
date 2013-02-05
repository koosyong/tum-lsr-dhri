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
#ifndef TOMTRACKING_H
#define TOMTRACKING_H

#include "tomheader.h"
#include "tom/tomobjecttracker.h"
#include "tom/tomobjecttracker.cpp"
#include "tom/tompointtracker.h"
#include "tom/tompointtracker.cpp"
#include "tom/tompcfilter.h"

class TOMTracking
{
public:
    TOMTracking(TOM_TYPE_WS ws);
    ~TOMTracking();

    typedef TOMObjectTracker<TOM_OBJECT> Tracker;

public:
    TOMObjectTracker<TOM_OBJECT>::VecTrack& tracking(std::vector<TOM_OBJECT> &objects, std::vector<CloudPtr> &clouds, std::vector<int>& tracksID, Cloud& tCloudPointA, Cloud &tCloudPointB);

private:
    int findMaxMatchedCloud(TOM_OBJECT tPtr);

private:
    Tracker *tracker;
    Tracker::VecPtr vecPtr;
    std::vector<TOM_OBJECT> m_objects;
    std::vector<CloudPtr> m_clouds;
    TOMObjectTracker<TOM_OBJECT>::VecTrack *m_tracks;



private:
    TOM_TYPE_WS m_ws;
    bool m_isDebug;
};

#endif // TOMTRACKING_H
