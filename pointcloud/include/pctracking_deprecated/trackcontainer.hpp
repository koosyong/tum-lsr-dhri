#include "trackcontainer.h"
#include <time.h>
#include <stdio.h>

template<class Object>
TrackContainer<Object>::TrackContainer(int _maxFrame, int _maxID)
    :maxFrame(_maxFrame), maxID(_maxID)
{
    oldCnt = 0;
    srand(time(NULL));

    r.reset(new int[maxID]);
    g.reset(new int[maxID]);
    b.reset(new int[maxID]);

    for(int i=0;i<maxID;i++){
        r[i] = rand() % 256;
        g[i] = rand() % 256;
        b[i] = rand() % 256;
    }
}

template<class Object>
TrackContainer<Object>::~TrackContainer()
{
    r.reset();
    g.reset();
    b.reset();

//    for(int i=0;i<tracks.size();i++)
//        tracks.at(i).reset();
//    tracks.clear();

}

template<class Object>
bool TrackContainer<Object>::createTrack(Object &object, int initT)
{
    int id = newId();
    if(id == -1)
        return 0;
    shared_ptr<TrackT> track;
    track.reset(new TrackT(id));
    track->insert(object, initT);
    tracks.push_back(track);
}

template<class Object>
bool TrackContainer<Object>::createTrack(Object &object, int initT, int id)
{
    for(int i=0;i<tracks.size();i++)
        if(tracks.at(i)->id == id)
            return 0;

    shared_ptr<TrackT> track;
    track.reset(new TrackT(id));
    track->insert(object, initT);
    tracks.push_back(track);
}


template<class Object>
bool TrackContainer<Object>::push_back_track(int id, Object &object, int time)
{
    // find track of id
    shared_ptr<TrackT> track;
    for(int i=0;i<tracks.size();i++){
        if(tracks.at(i)->id == id){
            track = tracks.at(i);
            break;
        }
    }
    if(track == 0) return 0;
    track->insert(object, time);
}

template<class Object>
bool TrackContainer<Object>::updateObject(int id, int time, Object &object)
{
    shared_ptr<TrackT> track;
    for(int i=0;i<tracks.size();i++){
        if(tracks.at(i)->id == id){
            track = tracks.at(i);
            break;
        }
    }
    if(track == 0) return 0;
    track->updateObjectAtFrame(time, object);
}

template<class Object>
int TrackContainer<Object>::newId()
{
    if(tracks.size() == maxID) return -1;
    int *cand;
    int nCand = maxID-tracks.size();
    int n = 0;
    cand = new int[nCand];
    for(int i=1;i<maxID;i++){
        bool isExist = 0;
        for(int j=0;j<tracks.size();j++){
            if(tracks.at(j)->id == i){
                isExist = 1;
                break;
            }
        }
        if(!isExist){
            cand[n] = i;
            n++;
        }
    }
    int nRand = cand[rand()%nCand];
    delete[] cand;
    cand = NULL;
    return nRand;

    //    if(tracks.size() == 0)
    //        return 1;

    //    for(int i=1;i<maxID;i++){
    //        for(int j=0;j<tracks.size();j++){
    //            if(tracks.at(j)->id == i)
    //                break;
    //            if(j == tracks.size()-1){
    //                // there's no id like i
    //                return i;
    //            }
    //        }
    //    }

    //    return -1;
}

template<class Object>
bool TrackContainer<Object>::merge(TrackContainer<Object> container)
{

}

template<class Object>
void TrackContainer<Object>::deleteNoUpdatedTracks(int size)
{
    deletedTrackIDs.clear();
    if(tracks.size() == 0) return;
    shared_ptr<TrackT> track;

    //    typename VecTrackPtr::iterator it = tracks.begin();
    for(typename VecTrackPtr::iterator it = tracks.begin(); it!=tracks.end();){
        track = *it;
        //        printf("track id : %d\n", track->id);
        //        printf("track size : %d\n", track->frames.size());
        if(track->lastFrame().time <= currentT-size){
            //            printf("Delete Track : %d\n", track->id);
            deletedTrackIDs.push_back(track->id);
            track->terminate();
            track.reset();
//            delete track;
//            track = NULL;
            it = tracks.erase(it);
        }else{
            ++it;
        }
    }
}
