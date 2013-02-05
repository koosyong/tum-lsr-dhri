#include "pcobjectcontainer.h"

PCObjectContainer::PCObjectContainer()
{

}

PCObjectContainer::PCObjectContainer(CloudPtr _pCloud)
    :pCloud(_pCloud)
{
    num = 0;
    makingObjects();
}

PCObjectContainer::~PCObjectContainer()
{

    for(int i=0;i<objects.size();i++)
        objects.at(i).reset();
//        if(objects.at(i) != NULL)
//            delete objects.at(i);
    objects.clear();
}

void PCObjectContainer::makingObjects() // only for PointT : pcl::PointXYZI
{
    /*
    for(int i=0;i<pCloud->points.size();i++){
        PointT point = pCloud->points[i];
        PCObject::Point object_point;
        object_point.pos[0] = point.x;
        object_point.pos[1] = point.y;
        object_point.pos[2] = point.z;
        object_point.id = i;
        if(i == 0){
            PCObject *object = new PCObject(point.intensity);
            object->insert(object_point);            
            objects.push_back(object);
            num++;
        }
        else{
            bool isExist = 0;
            // find class
            for(int j=0;j<objects.size();j++){
                if(objects.at(j)->id == point.intensity){
                    objects.at(j)->insert(object_point);
                    isExist = 1;
                    break;
                }
            }
            if(!isExist){
                PCObject *object = new PCObject(point.intensity);
                object->insert(object_point);
                objects.push_back(object);
                num++;
            }
        }
    }    
    */
}

void PCObjectContainer::makeNewObject(PCObject& object)
{
    num++;
    shared_ptr<PCObject> newObject;
    newObject.reset(new PCObject(num));
    *newObject = object;
    objects.push_back(newObject);

}

void PCObjectContainer::initGMM(double scale, double percent)
{
    for(int i=0;i<numObjects();i++)
        objects.at(i)->initialGMM(scale, percent);
}

bool PCObjectContainer::deleteObject(int id)
{
    bool isExist = 0;
    // find a object of id
    for(vector< shared_ptr<PCObject> >::iterator it = objects.begin(); it!=objects.end();){
        if((*it)->id == id){
            isExist = 1;
            it->reset();// = NULL;
//            delete (*it);
            objects.erase(it);
            return 1;
        }
        else it++;
    }
    if(!isExist) return 0;
}
