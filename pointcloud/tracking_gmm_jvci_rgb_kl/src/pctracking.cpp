#include "pctracking.h"
#include "pctracking_weight.h"
//#include <Eigen/Dense>

PCTracking::PCTracking(ros::Publisher* _pub_scene, ros::Publisher* _pub_model, FuncPub _funcPub)
{
    pub_scene = _pub_scene;
    pub_model = _pub_model;
    funcPub = _funcPub;
    tracks = new PCTrackContainer(100);
    imft = new IMFT<PCObject>(2, 40, 100, &weight_l2, &weight_l2);

    scale = 0.02;
    percent = 0.1;

    segmentation_tolerance = 0.03;
    segmentation_minSize = 10;
    segmentation_maxSize = 25000000;

    maxProbAssociation = 0.000001;

    thrProbScene = 100000;

}

PCTracking::~PCTracking()
{
    delete tracks;
    delete imft;
}

void PCTracking::run(CloudPtr pCloud)
{
    static int cnt = 0;

    // preprocessing : downsampling
    CloudPtr pCloud_output;
    CloudConstPtr pCloudConst = (CloudConstPtr)pCloud;

    pCloud_output.reset(new Cloud);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(pCloudConst);
    vg.setLeafSize(scale, scale, scale); // down sampling using a leaf size of 'scale'
    vg.filter(*pCloud_output);

    pCloud.reset(new Cloud);
    pCloud = pCloud_output;
    pCloud_output.reset(new Cloud);

    // segmentation at initial stage
    PCObjectContainer objects;
    if(cnt == 0){
        segmentationGMM(pCloud, objects);
    }
    // tracking begins
    else{
        PCObjectContainer oldObjectsUpdated, oldObjectsNonupdated, predictiveObjects, updatedObjects, newObjects;
        // make oldObjects from tracks
        for(int j=0;j<tracks->numTracks();j++){
            PCObject object = tracks->tracks.at(j)->lastFrame().object;
            if(tracks->tracks.at(j)->lastFrame().time == tracks->currentT)
                oldObjectsUpdated.makeNewObject(object);
            else
                oldObjectsNonupdated.makeNewObject(object);

        }

        // registration from tracks to pCloud
        ROS_INFO("# of updated oldObjects : %d", oldObjectsUpdated.objects.size());
        ROS_INFO("# of Nonupdated oldObjects : %d", oldObjectsNonupdated.objects.size());

        registrationGMMs(oldObjectsUpdated, pCloud, predictiveObjects);

      for(int i=0;i<oldObjectsNonupdated.objects.size();i++)
            predictiveObjects.makeNewObject(*oldObjectsNonupdated.objects.at(i));

        // point matching
        CloudPtr unmatchedPoints;
        unmatchedPoints.reset(new Cloud);
        vector<CloudPtr> observedPointsList;
        pointMatching(predictiveObjects, pCloud, unmatchedPoints, observedPointsList);

        // gaussian-level tracking in an object
        int nObjects = predictiveObjects.objects.size();
        for(int i=0;i<nObjects;i++){
            CloudPtr observedPoints = observedPointsList.at(i);
            PCObject* predictiveObject = predictiveObjects.objects.at(i);
            if(observedPoints->points.size() > segmentation_minSize){
                trackingGaussians(predictiveObject, observedPoints, updatedObjects);
            }

            //            PCObject* predictiveObject, CloudPtr observedPoints, vector<PCObject>& updatedObjects


        }
        //        updatedObjects.initGMM(scale, percent);

        //        pointMatching(oldObjects, pCloud, unmatchedPoints, updatedObjects);

        // new objects
        if(unmatchedPoints->points.size() > segmentation_minSize)
            segmentationGMM(unmatchedPoints, newObjects);

        // make objects
        ROS_INFO("# of updatedObjects: %d", updatedObjects.numObjects());
        ROS_INFO("# of newObjects: %d", newObjects.numObjects());
        for(int i=0;i<updatedObjects.numObjects();i++)
            objects.makeNewObject(*updatedObjects.objects.at(i));
        for(int i=0;i<newObjects.numObjects();i++)
            objects.makeNewObject(*newObjects.objects.at(i));

    }
    ROS_INFO("# of objects: %d", objects.numObjects());

    // tracking
    bool isMerged = 0;
    bool isSeparated = 0;

    // IMFT
    imft->setFrame(objects.objects, cnt);
    imft->confirmDGraph();
    imft->matching();
    imft->updateTracks();
//    imft->confirmDGraph();

    // extrack tracks from imft
    tracks = (PCTrackContainer*)imft->extractTracks();
    cnt ++;

}

void PCTracking::segmentationGMM(CloudPtr pCloud, PCObjectContainer& objects)
{
    // Creating the KdTree object for the search method of the extraction
//    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (pCloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (segmentation_tolerance); // 2cm
    ec.setMinClusterSize (segmentation_minSize);
    ec.setMaxClusterSize (segmentation_maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud( pCloud);
    ec.extract (cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        PCObject object;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
            Point point(pCloud->points[*pit]);
            object.insert(point);
        }
        object.initialGMM(scale, percent);
        if(object.gmm.size()>2)
            objects.makeNewObject(object);

    }

    // make initial tracks of gaussians in a new object
    for(int i=0;i<objects.numObjects();i++){
        objects.objects.at(i)->gaussianTrackingInit(2, 2, 1000, &weight_gaussian_predictive, &weight_gaussian_predictive);
        objects.objects.at(i)->gaussianTracking();
    }


}

void PCTracking::pointMatching(PCObjectContainer& predictiveObjects, CloudPtr pCloud, CloudPtr unmatchedPoints, vector<CloudPtr>& observedPointsList)
{
    // tracks in the previous time : tracks.
    int nObjects = predictiveObjects.objects.size();
    for(int i=0;i<nObjects;i++){
        CloudPtr observedPoints;
        observedPoints.reset(new Cloud);
        observedPointsList.push_back(observedPoints);
    }
    double sumSizeGMMs = 0.;
    for(int i=0;i<nObjects;i++){
        sumSizeGMMs += predictiveObjects.objects.at(i)->gmm.size();
    }
    for(int i=0;i<pCloud->points.size();i++){
        Point point(pCloud->points.at(i));

        // check the probability of the point to each object
        int maxTrack = 0;
        double maxProb = 0.;
        for(int j=0;j<nObjects;j++){
            PCObject* object = predictiveObjects.objects.at(j);//tracks->tracks.at(j)->lastFrame().object;
            double prob = object->evalNormedGMM(point, sumSizeGMMs);
            //            double prob = object->evalGMM(point);
            //            double prob = object->evalClosestGMM(point);
            if(prob > maxProb){
                maxProb = prob;
                maxTrack = j;
            }
        }
        // test maxProb
        //        ROS_INFO("maxProb: %f", maxProb);
        if(maxProb > maxProbAssociation){ // matched point
            observedPointsList.at(maxTrack)->push_back(pCloud->points.at(i));
        }
        else{   // unmatched point
            unmatchedPoints->push_back(pCloud->points.at(i));
        }
    }
}

void PCTracking::registrationGMMs(PCObjectContainer& oldObjects, CloudPtr pCloud, PCObjectContainer& registeredObjects)
{
    // make scene GMMs
    int nObjects = oldObjects.objects.size();
    PCObject* scenes = new PCObject[nObjects];
    CloudPtr* sceneClouds = new CloudPtr[nObjects];
    double sumSizeGMMs = 0.;
    for(int i=0;i<nObjects;i++){
        sceneClouds[i].reset(new Cloud);
        sumSizeGMMs += oldObjects.objects.at(i)->gmm.size();
    }
    for(int i=0;i<pCloud->points.size();i++){
        Point point(pCloud->points.at(i));
        for(int j=0;j<nObjects;j++){
            double prob = oldObjects.objects.at(j)->evalGMM(point);
//            cout<<prob<<endl;
            if(prob > thrProbScene){
                scenes[j].insert(point);
                sceneClouds[j]->points.push_back(pCloud->points.at(i));
            }
        }
    }

    // publish empty scene and model point set
    CloudPtr pCloudOut_empty;
    pCloudOut_empty.reset(new Cloud);
    PointT point;
    point.x = 0.;
    point.y = 0.;
    point.z = 0.;
    pCloudOut_empty->points.push_back(point);
    sensor_msgs::PointCloud2 output_empty;
    pcl::toROSMsg(*pCloudOut_empty, output_empty);
    output_empty.header.frame_id="/origin";

    // registration oldObjects to scene in order to produce registeredObjects
    for(int i=0;i<nObjects;i++){
        if(scenes[i].points.size() > segmentation_minSize){
            scenes[i].initialGMM(scale, percent);
            // publish scene point cloud
            sensor_msgs::PointCloud2 output_scene;
            pcl::toROSMsg(*sceneClouds[i], output_scene);
            output_scene.header.frame_id="/origin";
            pub_scene->publish (output_scene);

            PCGMMReg gmmreg;
            gmmreg.init(oldObjects.objects.at(i), &scenes[i], 100, funcPub);
            gmmreg.run();
            PCObject trasformedModel;
            trasformedModel = gmmreg.transformed_model;
            trasformedModel.trans_param = gmmreg.param_rigid;
            trasformedModel.updatePredictiveParameters();

            registeredObjects.makeNewObject(trasformedModel);

            pub_model->publish (output_empty);
            pub_scene->publish (output_empty);
        }
    }
    pCloudOut_empty.reset();
    for(int i=0;i<nObjects;i++){
        sceneClouds[i].reset();
    }
}

void PCTracking::trackingGaussians(PCObject* predictiveObject, CloudPtr observedPoints, PCObjectContainer& updatedObjects)
{
    // without filtering
    PCObject measurement;
    for(int i=0;i<observedPoints->points.size();i++){
        Point point(observedPoints->points.at(i));
        measurement.insert(point);
    }
    measurement.initialGMM(scale, percent);

    // update prediction parameter of each gaussian
    //    predictiveObject->updatePredictiveParameters();

    // new frame
    predictiveObject->gmm = measurement.gmm;
    predictiveObject->points = measurement.points;

    predictiveObject->gaussianTracking();

    // calculate velocity of a gaussian track
    predictiveObject->calculateVelocity();

    // making a topology of gmm
    predictiveObject->makeTopology();
//    predictiveObject->updateEdge();

    // check connectivity
    vector<PCObject> newObjects;
    int num = predictiveObject->componentGraph(newObjects);
    if(num != 0){
        ROS_WARN("Graph has component : %d", num);
        for(int i=0;i<newObjects.size();i++){
            if(newObjects.at(i).gmm.size()>1){
                newObjects.at(i).gaussianTrackingInit(2, 2, 1000, &weight_gaussian_predictive, &weight_gaussian_predictive);
                newObjects.at(i).gaussianTracking();
                newObjects.at(i).makeTopology();
                updatedObjects.makeNewObject(newObjects.at(i));
            }
        }
    }
    else{
        updatedObjects.makeNewObject(*predictiveObject);
    }



    // filtering
    /*
    CloudPtr unmatchedPoints;
    unmatchedPoints.reset(new Cloud);

    GMMFiltering filter(*predictiveObject, observedPoints, 0.001, unmatchedPoints, scale);

    // segment unmatchedPoints
    PCObjectContainer newGaussians;
    if(unmatchedPoints->points.size() > segmentation_minSize){
        segmentationGMM(unmatchedPoints, newGaussians);
        int nGaussians = filter.posterior.gmm.size();
        for(int i=0;i<newGaussians.numObjects();i++){
            PCObject* newObjectPart = newGaussians.objects.at(i);
            nGaussians += newObjectPart->gmm.size();
            for (int j=0;j<newObjectPart->points.size();j++){
                Point point = newObjectPart->points.at(j);
                filter.posterior.points.push_back(point);
            }
        }
        for(int i=0;i<filter.posterior.gmm.size();i++){
            filter.posterior.gmm.at(i).weight = filter.posterior.gmm.at(i).weight * (double)filter.posterior.gmm.size() / (double)nGaussians;
        }
        for(int i=0;i<newGaussians.numObjects();i++){
            PCObject* newObjectPart = newGaussians.objects.at(i);
            for(int j=0;j<newObjectPart->gmm.size();j++){
                newObjectPart->gmm.at(j).weight = newObjectPart->gmm.at(j).weight * (double)newObjectPart->gmm.size() / (double)nGaussians;
//                newObjectPart->gmm.at(j).weight = (double)newObjectPart->points.size() / (double)observedPoints->points.size();
                filter.posterior.gmm.push_back(newObjectPart->gmm.at(j));
            }
        }ec
    }

    // weight assignment
    double weightSum = 0.;
    for(int i=0;i<filter.posterior.gmm.size();i++){
        weightSum += filter.posterior.gmm.at(i).weight;
    }
    cout<<"WeightSum"<<weightSum<<endl;

    // MFT among gaussians
    updatedObjects.makeNewObject(filter.posterior);
    */
}
