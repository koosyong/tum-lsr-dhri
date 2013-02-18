#include "pcobject.h"
#include <math.h>
#include <iostream>
#define pi 3.141592

PCObject::PCObject()
{
//    isParamExist = NULL;
//    imftGaussians = NULL;
    gaussianTracks = IMFT<Gaussian>::TrackContainerT(10);

//    topology_graph = NULL;
//    topology_nodeMap = NULL;
//    topology_edgeMap =  NULL;

    topology_graph.reset(new ListGraph());
    topology_nodeMap.reset(new ListGraph::NodeMap<Gaussian>(*topology_graph));
    topology_edgeMap.reset(new ListGraph::EdgeMap<double>(*topology_graph));
    filteredWeight = 0.;

    diffWeight = new double[10];
    for(int i=0;i<10;i++)
        diffWeight[i] = 0.;
}

PCObject::PCObject(int _id)
    :id(_id)
{
//    isParamExist = NULL;
//    imftGaussians = NULL;
    gaussianTracks = IMFT<Gaussian>::TrackContainerT(10);
//    topology_graph = NULL;
//    topology_nodeMap = NULL;
//    topology_edgeMap = NULL;

    topology_graph.reset(new ListGraph());
    topology_nodeMap.reset(new ListGraph::NodeMap<Gaussian>(*topology_graph));
    topology_edgeMap.reset(new ListGraph::EdgeMap<double>(*topology_graph));
    filteredWeight = 0.;

    diffWeight = new double[10];
    for(int i=0;i<10;i++)
        diffWeight[i] = 0.;
}

PCObject::~PCObject()
{
//    points.clear();
//    gmm.clear();
//    for(int i=0;i<frame.size();i++)
//        frame.at(i).reset();
//    edges.clear();
//    imftGaussians.reset();
//    topology_graph.reset();
//    topology_nodeMap.reset();
//    topology_edgeMap.reset();

}

void PCObject::insert(Point p)
{
    points.push_back(p);
}

void PCObject::setTransParam(vnl_vector<double> param)
{
    isParamExist = 1;
    trans_param = param;
}

Point PCObject::getCentroid()
{
    Point centroid;
    centroid.pos[0] = 0.;
    centroid.pos[1] = 0.;
    centroid.pos[2] = 0.;
    int n = points.size();
    for(int i=0;i<n;i++){
        centroid.pos += points.at(i).pos;
    }
    centroid.pos /= n;
    return centroid;
}

void PCObject::initialGMM(double _scale)
{
    scale = _scale;
    gmm.clear();

    int n = points.size();
    for(int i=0;i<n;i++){
        Gaussian gaussian;
        gaussian.mean = points.at(i).pos;
        gaussian.covariance(0,0) = scale*scale;
        gaussian.covariance(0,1) = 0;
        gaussian.covariance(0,2) = 0;
        gaussian.covariance(1,0) = 0;
        gaussian.covariance(1,1) = scale*scale;
        gaussian.covariance(1,2) = 0;
        gaussian.covariance(2,0) = 0;
        gaussian.covariance(2,1) = 0;
        gaussian.covariance(2,2) = scale*scale;
        gaussian.weight = 1./(double)n;
        //        gaussian.weight = 1;
        gaussian.nPoint = 1;
        gaussian.cov_determinant = gaussian.covariance.determinant();
        gaussian.cov_inverse = gaussian.covariance.inverse();

        gmm.push_back(gaussian);
    }
}

void PCObject::initialGMM(double _scale, double _percent)
{
    scale = _scale;
    percent = _percent;
    gmm.clear();

    int n = points.size();
    cout<<"# of points : "<<n<<endl;
    for(int i=0;i<n;i++){
        Gaussian gaussian;
        gaussian.mean = points.at(i).pos;
        gaussian.covariance(0,0) = scale*scale;
        gaussian.covariance(0,1) = 0;
        gaussian.covariance(0,2) = 0;
        gaussian.covariance(1,0) = 0;
        gaussian.covariance(1,1) = scale*scale;
        gaussian.covariance(1,2) = 0;
        gaussian.covariance(2,0) = 0;
        gaussian.covariance(2,1) = 0;
        gaussian.covariance(2,2) = scale*scale;
        gaussian.weight = 1./(double)n;
        //        gaussian.weight = 1;
        gaussian.nPoint = 1;
        gaussian.cov_determinant = gaussian.covariance.determinant();
        gaussian.cov_inverse = gaussian.covariance.inverse();
        gaussian.initPrediction();
        gmm.push_back(gaussian);
    }

    //    double percent;
    //    percent = 0.5;
    //    //    percent = 0.45;
    //    //    percent = 0.4;
    //    //    percent = 0.35;
    //    percent = 0.3;
    ////        percent = 0.25;
    ////        percent = 0.2;
    ////        percent = 0.15;
    ////        percent = 0.1;
    ////    percent = 0.05;
    //    //    percent = 0.02;
    ////        percent = 0.01;

    if(percent > 0){
        double last = pcl::getTime ();
        simplify(SIMPLE_HCKL, 1./percent);
        //    simplify(SIMPLE_FA, 1./percent);
        double now = pcl::getTime ();
        double time = now-last;
//        cout<<"time for simplification "<<time<<" second"<<endl;
        static double total = 0.;
        total += time;
//        cout<<"TOTAL time for simplification "<<total<<" second"<<endl;
    }
}

void PCObject::simplify(SIMPLE method, double ratio, int nCluster)
{
    // kmeans clustering
    int dimensions = 3;
    int sampleCount = gmm.size();
    int clusterCount;
    if(nCluster == 0)
        clusterCount = sampleCount/ratio;
    else
        clusterCount = nCluster;
    if(clusterCount<1) clusterCount = 1;

    //    clusterCount = 1;
    cv::Mat points(sampleCount, dimensions, CV_32F);
    cv::Mat labels(sampleCount, 1, CV_16S, 0);
    cv::Mat centers(clusterCount, 1, points.type());
    //    cout<<"clusterCount "<<clusterCount<<endl;

    //    if(clusterCount > 2)
    //    {
    // values of 1st half of data set is set to 10
    // change the values of 2nd half of the data set; i.e. set it to 20


    for(int i =0;i<points.rows;i++)
    {
        points.at<float>(i,0) = gmm.at(i).mean[0];
        points.at<float>(i,1) = gmm.at(i).mean[1];
        points.at<float>(i,2) = gmm.at(i).mean[2];
    }
    double last = pcl::getTime ();
    cv::kmeans(points, clusterCount, labels, cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0), 3, cv::KMEANS_PP_CENTERS, centers);
    double now = pcl::getTime ();
    //        cout<<"time for kmeans clustering "<<now-last<<" second"<<endl;

    //    }
    //    else{
    //        cv::Mat l(sampleCount,1,CV_16U,0);
    //        labels = l;
    //    }
    if(method == SIMPLE_HCKL || method == SIMPLE_HCL2){
        //    % our implementation of the method proposed in <Hierarchical clustering of
        //    % a mixture model> By Jacob Goldberger Sam Roweis in NIPS 2005.

        int dim = dimensions;
        int n = gmm.size();
        vector<Gaussian> gmmHC;

        double bear = 0.00001;
        double Err0 = 1e50;

        for(int cycle = 1; cycle<=2; cycle++){
            gmmHC.clear();
            int k = 0;
            // find k
            for(int i=0;i<n;i++){
                if(labels.at<int>(i) > k)
                    k = labels.at<int>(i);
            }
            k = k + 1;
            //            cout<<"cycle"<<cycle<<endl;
            //            cout<<"Label"<<labels<<endl;
            double last = pcl::getTime ();
            // Refit
            for(int i=0;i<k;i++){
                Gaussian gmmK;
                vector<Gaussian> set;
                gmmK.weight = 0.;
                for(int j=0;j<n;j++){
                    if(labels.at<int>(j) == i){
                        set.push_back(gmm.at(j));
                        gmmK.weight += gmm.at(j).weight;
                    }
                }
                int num = set.size();
                gmmK.nPoint = num;
                Eigen::Vector3d t;
                t[0] = t[1] = t[2] = 0.;
                for(int j = 0;j<num;j++){
                    t = t + set.at(j).mean * set.at(j).weight;
                } t = t / gmmK.weight;
                gmmK.mean = t;

                Eigen::Matrix3d cov_set = Eigen::Matrix3d::Zero();
                for(int j = 0;j<num;j++){
                    cov_set = cov_set + set.at(j).weight * (set.at(j).covariance + (gmmK.mean-set.at(j).mean)*(gmmK.mean-set.at(j).mean).transpose());
                }
                gmmK.covariance = cov_set / gmmK.weight;
                gmmK.cov_determinant = gmmK.covariance.determinant();
                gmmK.cov_inverse = gmmK.covariance.inverse();
                gmmK.initPrediction();
                gmmHC.push_back(gmmK);
            }
            double now = pcl::getTime ();
            //            cout<<"time for refit "<<now-last<<" second"<<endl;

            last = pcl::getTime ();
            // Regroup
            double **Dis = new double*[k];
            for(int i=0;i<k;i++)
                Dis[i] = new double[n];

            for(int i = 0;i<k;i++){
                Gaussian gmmi, gmmj;
                gmmi = gmmHC.at(i);
                Eigen::Matrix3d covi = gmmi.covariance;
                Eigen::Matrix3d covi_inv = gmmi.cov_inverse;
                Eigen::Vector3d meani = gmmi.mean;
                double deti = gmmi.cov_determinant;
                for (int j = 0;j<n;j++){
                    gmmj = gmm.at(j);
                    Eigen::Matrix3d covj = gmmj.covariance;
                    Eigen::Vector3d meanj = gmmj.mean;
                    double detj = gmmj.cov_determinant;

                    // KL distance
                    if(method == SIMPLE_HCKL){
                        double a = (meanj-meani).transpose()*covi_inv*(meanj-meani);
                        Dis[i][j] = 1./2.*(log(deti/detj) + (covi_inv*covj).trace() + a-dim);
                    }
                    // L2 distance
                    if(method == SIMPLE_HCL2){
                        double energy1, energy2, energy3;
                        Eigen::Matrix3d invij = (covi+covj).inverse();
                        double a = (meani-meanj).transpose()*invij*(meani-meanj);
                        energy1 = 1./sqrt(pow(2*pi,3)*(covi+covi).determinant());
                        energy2 = 1./sqrt(pow(2*pi,3)*(covi+covj).determinant())*exp(-0.5*a);
                        energy3 = 1./sqrt(pow(2*pi,3)*(covj+covj).determinant());
                        Dis[i][j] = gmmi.weight*gmmj.weight*(energy1-2*energy2+energy3);
                    }
                }
            }
            now = pcl::getTime ();
            //            cout<<"time for regroup "<<now-last<<" second"<<endl;

            double Err1 = 0;
            for(int i=0;i<n;i++){
                // find min
                double min;
                int minK;
                for(int j=0;j<k;j++){
                    if(j == 0) {
                        min = Dis[j][i];
                        minK = j;
                    }
                    else{
                        if(Dis[j][i] < min){
                            min = Dis[j][i];
                            minK = j;
                        }
                    }
                }
                Err1 = Err1 + fabs(gmm.at(i).weight) * min;
                labels.at<int>(i) = minK;
            }
            for(int i=0;i<k;i++)
                delete[] Dis[i];
            delete[] Dis;

            // reordering labels
            int K = k;
            int *siz = new int[K];
            for (int i = 0;i<K;i++){
                siz[i] = 0;
                for(int j=0;j<n;j++){
                    if(labels.at<int>(j) == i)
                        siz[i] ++;
                }
            }
            int *map = new int[K];
            int c = -1;
            for (int i = 0;i<K;i++){
                if(siz[i]>=1){
                    c = c + 1;
                }
                map[i] = c;
            }
            k = c;
            for (int i = 0;i<n;i++){
                labels.at<int>(i) = map[labels.at<int>(i)];
            }
            //            cout<<"cycle"<<cycle<<endl;
            if(fabs(Err1 - Err0)<= bear * fabs(Err0))
                break;
            else
                Err0 = Err1;
            delete[] siz;
            delete[] map;

        }
        gmm = gmmHC;
    }

    else if(method == SIMPLE_FA){
        //        % OurGMM0.m approximates a given mixture model using a simpler one with fewer components.
        //        % Output: a simpler mixture model, with the weight, center, and covariance of each component specified.
        //        % T: component centers (m-by-dim)
        //        % W: component weights (m-by-1)
        //        % B: component bandwidth (m-by-1)
        //        W = zeros(1,K);
        //        T = zeros(K,dim);
        //        B = zeros(1,K);

        //        invH = zeros(1,K);

        int dim = dimensions;
        int n = gmm.size();
        vector<Gaussian> gmmFA;

        double bear = 0.1;

        double C1 = 100;
        double C2 = 50;
        double C3 =50;
        double Err0 = 1e50;
        for (int cycle = 1;cycle<=C1;cycle++){
            //            cout<<"cycle"<<cycle<<endl;
            gmmFA.clear();
            int K = 0;
            // find k
            for(int i=0;i<n;i++){
                if(labels.at<int>(i) > K)
                    K = labels.at<int>(i);
            }
            K = K + 1;

            //            cout<<"Label"<<labels<<endl;
            for ( int i=0;i<K;i++){
                //                cout<<"class "<<i<<endl;
                Gaussian gmmK;
                Eigen::Vector3d mean;
                Eigen::Matrix3d cov;
                double weight;

                vector<Gaussian> set;
                // Initial fit
                weight = 0.;
                for(int j=0;j<n;j++){
                    if(labels.at<int>(j) == i){
                        set.push_back(gmm.at(j));
                        weight += gmm.at(j).weight;
                    }
                }
                int num = set.size();
                Eigen::Vector3d t;
                t[0] = t[1] = t[2] = 0.;
                for(int j = 0;j<num;j++){
                    t = t + set.at(j).mean * set.at(j).weight;
                } t = t / weight;
                mean = t;

                Eigen::Matrix3d cov_set = Eigen::Matrix3d::Zero();
                for(int j = 0;j<num;j++){
                    cov_set = cov_set + set.at(j).weight * (set.at(j).covariance + (mean-set.at(j).mean)*(mean-set.at(j).mean).transpose());
                }
                cov = cov_set / weight;
                Gaussian gmmi;
                gmmi.covariance = cov;
                gmmi.mean = mean;
                gmmi.weight = weight;
                for (int cycle1 = 0;cycle1<C2;cycle1++){
                    //                    cout<<"cycle1 "<<cycle1<<endl;
                    Gaussian gmmj;

                    double Ms_err0 = 1e50;
                    Eigen::Vector3d t;
                    t.setZero();
                    for (int mscycle = 0;mscycle<C3;mscycle++){
                        Eigen::Matrix3d c1;
                        Eigen::Vector3d ct;
                        c1.setZero();
                        ct.setZero();
                        for (int j=0;j<num;j++){
                            gmmj = set.at(j);
                            Eigen::Matrix3d invHiHj = (gmmi.covariance + gmmj.covariance).inverse();
                            double detHiHj = (gmmi.covariance + gmmj.covariance).determinant();
                            Eigen::Vector3d v  = gmmi.mean - gmmj.mean;
                            double rij = v.transpose()*invHiHj*v;
                            double kr = exp(-0.5*rij);
                            double kpr = -0.5*kr;
                            Eigen::Matrix3d c  = kpr * gmmj.weight * invHiHj / sqrt(detHiHj);
                            ct = ct + c * gmmj.mean;
                            c1 = c1 + c;
                        }
                        //                        if (c1[0] == 0 && c1[1] == 0 && c1[2] == 0) break;
                        Eigen::Vector3d t1 = c1.inverse()*ct;
                        double Ms_err1 = (t1 - t).norm();
                        t = t1;
                        if (fabs(Ms_err0 - Ms_err1) <= bear * Ms_err0)
                            break;
                        else
                            Ms_err0 = Ms_err1;
                    }
                    mean = t;

                    cov_set.setZero();
                    Eigen::Matrix3d c1;
                    c1.setZero();
                    for (int j=0;j<num;j++){
                        gmmj = set.at(j);
                        Eigen::Matrix3d invHiHj = (gmmi.covariance + gmmj.covariance).inverse();
                        double detHiHj = (gmmi.covariance + gmmj.covariance).determinant();
                        Eigen::Vector3d v = t - gmmj.mean;
                        double rij = v.transpose()*invHiHj*v;
                        double kr = exp( -0.5*rij);
                        double kpr = -0.5*kr;

                        Eigen::Matrix3d c =  gmmj.weight * invHiHj / sqrt(detHiHj);
                        c1 = c1 + c * kr;

                        cov_set = cov_set + c *(kr*gmmj.covariance - 4*kpr*v*v.transpose()*invHiHj*gmmi.covariance);

                    }
                    cov = c1.inverse()*cov_set;

                    //                    if(mscycle <=1);   break;
                }

                double c = 0;
                for (int j = 0;j<num;j++){
                    double detHiHj = (gmmi.covariance + set.at(j).covariance).determinant();
                    Eigen::Matrix3d invHiHj = (gmmi.covariance + set.at(j).covariance).inverse();
                    Eigen::Vector3d v = gmmi.mean - set.at(j).mean;
                    double rij = v.transpose()*invHiHj*v;
                    double kr = exp( -0.5*rij);
                    c =  c + set.at(j).weight * kr / sqrt(detHiHj);
                }
                gmmK.weight = sqrt((2*cov).determinant()) * c;
                gmmK.covariance = cov;
                gmmK.mean = mean;

                gmmFA.push_back(gmmK);
            }


            // Regroup
            double **Dis = new double*[K];
            for(int i=0;i<K;i++)
                Dis[i] = new double[n];

            double *Z = new double[K];
            for (int i = 0;i<K;i++){
                Z[i] = 0;
                for(int j=0;j<n;j++){
                    if(labels.at<int>(j) == i){
                        Z[i] = Z[i] + gmm.at(j).weight;
                    }
                }
            }
            for(int i = 0;i<K;i++){
                Gaussian gmmi, gmmj;
                gmmi = gmmFA.at(i);
                Eigen::Matrix3d covi = gmmi.covariance;
                Eigen::Vector3d meani = gmmi.mean;
                double deti = (2*covi).determinant();
                for (int j = 0;j<n;j++){
                    gmmj = gmm.at(j);
                    Eigen::Matrix3d covj = gmmj.covariance;
                    Eigen::Vector3d meanj = gmmj.mean;
                    double detj = (2*covj).determinant();
                    double detij = (covi+covj).determinant();

                    // L2 distance from FA
                    Eigen::Matrix3d invij = (covi+covj).inverse();
                    Eigen::Vector3d v = meani - meanj;
                    double rij = v.transpose()*invij*v;
                    double kr = exp( -0.5*rij);
                    Dis[i][j] = 1./sqrt(detj) + pow(gmmi.weight/Z[i],2)/sqrt(deti) - 2*gmmi.weight/Z[i]*kr/sqrt(detij);

                    //                    // L2 distance from Koosy
                    //                    double energy1, energy2, energy3;
                    //                    Eigen::Matrix3d invij = (covi+covj).inverse();
                    //                    double a = (meani-meanj).transpose()*invij*(meani-meanj);
                    //                    energy1 = 1./sqrt(pow(2*pi,3)*(covi+covi).determinant());
                    //                    energy2 = 1./sqrt(pow(2*pi,3)*(covi+covj).determinant())*exp(-0.5*a);
                    //                    energy3 = 1./sqrt(pow(2*pi,3)*(covj+covj).determinant());
                    //                    Dis[i][j] = gmmi.weight*gmmj.weight*(energy1-2*energy2+energy3);

                }
            }

            double Err1 = 0;
            for(int i=0;i<n;i++){
                // find min
                double min;
                int minK;
                for(int j=0;j<K;j++){
                    if(j == 0) {
                        min = Dis[j][i];
                        minK = j;
                    }
                    else{
                        if(Dis[j][i] < min){
                            min = Dis[j][i];
                            minK = j;
                        }
                    }
                }
                Err1 = Err1 + fabs(gmm.at(i).weight) * min;
                labels.at<int>(i) = minK;
            }

            delete[] Z;

            // reordering labels
            int *siz = new int[K];
            for (int i = 0;i<K;i++){
                siz[i] = 0;
                for(int j=0;j<n;j++){
                    if(labels.at<int>(j) == i)
                        siz[i] ++;
                }
            }
            int *map = new int[K];
            int c = -1;
            for (int i = 0;i<K;i++){
                if(siz[i]>=1){
                    c = c + 1;
                }
                map[i] = c;
            }
            K = c;
            for (int i = 0;i<n;i++){
                labels.at<int>(i) = map[labels.at<int>(i)];
            }

            if(fabs(Err1 - Err0)<= bear * fabs(Err0))
                break;
            else
                Err0 = Err1;
            delete[] siz;
            delete[] map;
        }
        gmm = gmmFA;
    }

}

double PCObject::L2ofGMMandPoints(double scale)
{
    // L2 distance

    int n = points.size();
    int m = gmm.size();
    vector<Gaussian> gmmPoint;
    for(int i=0;i<n;i++){
        Gaussian point;
        point.mean = points.at(i).pos;
        point.covariance(0,0) = scale*scale;
        point.covariance(0,1) = 0;
        point.covariance(0,2) = 0;
        point.covariance(1,0) = 0;
        point.covariance(1,1) = scale*scale;
        point.covariance(1,2) = 0;
        point.covariance(2,0) = 0;
        point.covariance(2,1) = 0;
        point.covariance(2,2) = scale*scale;
        point.weight = 1./(double)n;

        gmmPoint.push_back(point);
    }
    double energy1 = 0.;
    for(int i=0;i<n;i++){
        for(int j=0;j<n;j++){
            Eigen::Matrix3d cov = gmmPoint.at(i).covariance + gmmPoint.at(j).covariance;
            Eigen::Vector3d mean = gmmPoint.at(i).mean - gmmPoint.at(j).mean;
            Eigen::Matrix3d invij = cov.inverse();
            double a = mean.transpose()*invij*mean;
            double gauss = 1./sqrt(pow(2*pi,3)*cov.determinant())*exp(-0.5*a);
            energy1 += gmmPoint.at(i).weight*gmmPoint.at(j).weight*gauss;
        }
    }
    double energy2 = 0.;
    for(int i=0;i<n;i++){
        for(int j=0;j<m;j++){
            Eigen::Matrix3d cov = gmmPoint.at(i).covariance + gmm.at(j).covariance;
            Eigen::Vector3d mean = gmmPoint.at(i).mean - gmm.at(j).mean;
            Eigen::Matrix3d invij = cov.inverse();
            double a = mean.transpose()*invij*mean;
            double gauss = 1./sqrt(pow(2*pi,3)*cov.determinant())*exp(-0.5*a);
            energy2 += gmmPoint.at(i).weight*gmm.at(j).weight*gauss;
        }
    }
    double energy3 = 0.;
    for(int i=0;i<m;i++){
        for(int j=0;j<m;j++){
            Eigen::Matrix3d cov = gmm.at(i).covariance + gmm.at(j).covariance;
            Eigen::Vector3d mean = gmm.at(i).mean - gmm.at(j).mean;
            Eigen::Matrix3d invij = cov.inverse();
            double a = mean.transpose()*invij*mean;
            double gauss = 1./sqrt(pow(2*pi,3)*cov.determinant())*exp(-0.5*a);
            energy3 += gmm.at(i).weight*gmm.at(j).weight*gauss;
        }
    }
    return energy1 - 2*energy2 + energy3;
}

double PCObject::evalGMM(Point x)
{
    double eval = 0.;
    for(int i=0;i<gmm.size();i++){
        Eigen::Vector3d mean;
        mean = gmm.at(i).mean;
        double weight = gmm.at(i).weight;
        Eigen::Matrix3d inv = gmm.at(i).cov_inverse;

        double a = (x.pos-mean).transpose()*inv*(x.pos-mean);
        //        eval += weight * 1./sqrt(pow(2*pi,3)*det) * exp(-0.5*a);
        eval += weight * exp(-0.5*a);
    }
    //    return eval * gmm.size();
    return eval;
}


double PCObject::evalNormedGMM(Point x, double den)
{
    double eval = 0.;
    for(int i=0;i<gmm.size();i++){
        Eigen::Vector3d mean;
        mean = gmm.at(i).mean;
        Eigen::Matrix3d cov = gmm.at(i).covariance;
        double weight = gmm.at(i).weight;
        Eigen::Matrix3d inv = gmm.at(i).cov_inverse;

        double a = (x.pos-mean).transpose()*inv*(x.pos-mean);
        double det = gmm.at(i).covariance.determinant();
        //        eval += weight * 1./sqrt(pow(2*pi,3)*det) * exp(-0.5*a);
        eval += weight * 1./sqrt(pow(2*pi,3)*det) * exp(-0.5*a);
    }
    return eval * gmm.size() / den;
}

double PCObject::evalClosestGMM(Point x)
{
    double max = 0.;
    for(int i=0;i<gmm.size();i++){
        double eval = 0.;
        Eigen::Vector3d mean;
        mean = gmm.at(i).mean;
        Eigen::Matrix3d cov = gmm.at(i).covariance;
        Eigen::Matrix3d inv = gmm.at(i).cov_inverse;

        double a = (x.pos-mean).transpose()*inv*(x.pos-mean);
        //        eval += weight * 1./sqrt(pow(2*pi,3)*det) * exp(-0.5*a);
        eval = exp(-0.5*a);
        if(eval > max)
            max = eval;
    }
    return max;
}
void PCObject::mergeTwoGMMs(PCObject* gmm1, PCObject* gmm2)
{
    gmm.clear();
    for(int i=0;i<gmm1->gmm.size();i++){
        Gaussian gaussian = gmm1->gmm.at(i);
        gaussian.weight /= 2.;
        gmm.push_back(gaussian);
    }
    for(int i=0;i<gmm2->gmm.size();i++){
        Gaussian gaussian = gmm2->gmm.at(i);
        gaussian.weight /= 2.;
        gmm.push_back(gaussian);
    }
}

void PCObject::gaussianTrackingInit(int _window_short, int _window_long, int _maxID, funcWeightGaussian _weight_gaussian, funcWeightGaussian _weight_gaussian_fast)
{
    window_short = _window_short;
    window_long = _window_long;

    imftGaussians.reset(new IMFT<Gaussian>(window_short, window_long, _maxID, _weight_gaussian, _weight_gaussian_fast));

    cnt = 0;

    frame.clear();
    for(int i=0;i<gmm.size();i++){        
        shared_ptr<Gaussian> ptr(new Gaussian);
        *ptr = gmm.at(i);
        frame.push_back(ptr);
    }
    imftGaussians->setFrame(frame, cnt);
    imftGaussians->matching();
    //    imftGaussians->confirmDGraph();
    imftGaussians->updateTracks();

    gaussianTracks = *(imftGaussians->extractTracks());

    cnt++;
}

void PCObject::gaussianTracking()
{
    frame.clear();
    for(int i=0;i<gmm.size();i++){
        shared_ptr<Gaussian> ptr(new Gaussian);
        *ptr = gmm.at(i);
        frame.push_back(ptr);
    }
    imftGaussians->setFrame(frame, cnt);
    imftGaussians->matching();
//    imftGaussians->confirmDGraph();
    imftGaussians->updateTracks();

    gaussianTracks = *(imftGaussians->extractTracks());

    // update gmm
    gmm.clear();
    for(int i=0;i<gaussianTracks.numTracks();i++){
        gmm.push_back(gaussianTracks.tracks.at(i)->lastFrame().object);
    }


    cnt++;
}

void PCObject::updatePredictiveParameters()
{
    // update predictive parameters in all gaussians in the graph
    //    imftGaussians->confirmDGraph();
    for(ListGraph::NodeIt n(imftGaussians->m_g); n != INVALID; ++n){
        Gaussian *node = &((*imftGaussians->m_gNodeMap)[n].ptr.object);
        node->updateParam(trans_param);
    }
}

void PCObject::updateEdge()
{
    double sum = 0.;
    double newAvgWeight = 0.;
    int num = edges.size();
    for(int i=0;i<num;i++){
        Edge e = edges.at(i);
        sum += e.weight;
    }
    newAvgWeight = sum / num;

    if(filteredWeight == 0)
        filteredWeight = newAvgWeight;
    else{
        double diff = filteredWeight - newAvgWeight;
        double sumWeight = 0;
        for(int i=1;i<10;i++){
            diffWeight[i] = diffWeight[i-1];
            sumWeight += diffWeight[i];
        }
        diffWeight[0] = diff;
        sumWeight += diff;

        cout<<"DIFF "<<diff<<endl;
        double factor = 2.5;
//        if(newDiffweight > 0 && acc > 0){
            // update each edge to reduce as much as diff and check the connectivity
            for(ListGraph::EdgeIt e(*topology_graph); e != INVALID; ++e){
//                ListGraph::Edge edge = (*topology_edgeMap)[e];
                (*topology_edgeMap)[e] += sumWeight*factor;
                if((*topology_edgeMap)[e] < 0.8){
                    // delete edge
                    topology_graph->erase(e);
                    cout<<"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFDELETEDDDDDDDDDDDDDDDDDDDDd"<<endl;
                }
            }
//        }
            filteredWeight = 0.9 * filteredWeight + 0.1 * newAvgWeight;
    }

//    avgWeight = newAvgWeight;

}

void PCObject::makeTopology()
{
//    if(topology_nodeMap != NULL) {delete topology_nodeMap; topology_nodeMap = NULL;}
//    if(topology_edgeMap != NULL) {delete topology_edgeMap; topology_edgeMap = NULL;}
//    if(topology_graph != NULL) {delete topology_graph; topology_graph = NULL;}

    topology_graph->clear();
    edges.clear();

    // make node
    cout<<"GMM SIZE"<<gmm.size()<<endl;
    for(int i=0;i<gmm.size();i++){
        ListGraph::Node node = topology_graph->addNode();
        (*topology_nodeMap)[node] = gmm.at(i);
    }
    // make edge
    for(ListGraph::NodeIt u(*topology_graph); u != INVALID; ++u){
        ListGraph::NodeIt v = u;
//        ++v;
        for(v; v != INVALID; ++v){
            double weight = topology_weight((*topology_nodeMap)[u], (*topology_nodeMap)[v]);
//            std::cout<<weight<<endl;
            if(weight > 0.9){
                ListGraph::Edge edge = topology_graph->addEdge(u,v);
                (*topology_edgeMap)[edge] = weight;

                Edge e;
                e.u.pos = (*topology_nodeMap)[u].mean;
                e.v.pos = (*topology_nodeMap)[v].mean;
                e.weight = weight;
                edges.push_back(e);
            }
        }
    }
}

double PCObject::topology_weight(Gaussian g1, Gaussian g2)
{
//    double maxDist = 23000;   // for 0.02 scale / 0.2 percent : lower -> weaker
    double maxDist = 40000;   // for 0.02 scale / 0.1 percent : lower -> weaker
//    double maxDist = 150000;   // for 0.01 scale
    double maxVel = 0.02;
    double energy1 = 0.;
            Eigen::Matrix3d cov = g1.covariance + g1.covariance;
            Eigen::Vector3d mean = g1.mean - g1.mean;
            Eigen::Matrix3d invij = cov.inverse();
            double a = mean.transpose()*invij*mean;
            double gauss = 1./sqrt(pow(2*pi,3)*cov.determinant())*exp(-0.5*a);
            energy1 += gauss;

    double energy2 = 0.;

            cov = g1.covariance + g2.covariance;
            mean = g1.mean - g2.mean;
            invij = cov.inverse();
            a = mean.transpose()*invij*mean;
            gauss = 1./sqrt(pow(2*pi,3)*cov.determinant())*exp(-0.5*a);
            energy2 += gauss;

    double energy3 = 0.;
            cov = g2.covariance + g2.covariance;
            mean = g2.mean - g2.mean;
            invij = cov.inverse();
            a = mean.transpose()*invij*mean;
            gauss = 1./sqrt(pow(2*pi,3)*cov.determinant())*exp(-0.5*a);
            energy3 += gauss;
    double posCloseness = (maxDist - (energy1 - 2*energy2 + energy3))/maxDist;
    Eigen::Vector3d velDiff = g1.velocity - g2.velocity;
    double velCloseness =  (maxVel - velDiff.norm())/maxVel;
//    cout<<posCloseness<<endl;
//    return 0.98 * posCloseness + 0.02 * velCloseness;
    return 1. * posCloseness + 0. * velCloseness;
}

void PCObject::calculateVelocity()
{
    for(int i=0;i<gaussianTracks.numTracks();i++){
        Gaussian last = gaussianTracks.tracks.at(i)->lastFrame().object;
        Gaussian before = gaussianTracks.tracks.at(i)->getFrameFromLast(1).object;
        gmm.at(i).velocity[0] = last.mean[0] - before.mean[0];
        gmm.at(i).velocity[1] = last.mean[1] - before.mean[1];
        gmm.at(i).velocity[2] = last.mean[2] - before.mean[2];
    }
}

int PCObject::componentGraph(vector<PCObject> &newObjects)
{
    bool connectivity = connected(*topology_graph);
    // check the number of components
    if(connectivity)
        return 0;
    else{
        ListGraph::NodeMap<int> component_nodeMap(*topology_graph);
//        component_nodeMap = new ListGraph::NodeMap<Gaussian>(*topology_graph);
        int num = connectedComponents(*topology_graph, component_nodeMap);
//        newObjects.resize(num);
        for(int i=0;i<num;i++){
            PCObject object;
            newObjects.push_back(object);
        }

        for(ListGraph::NodeIt u(*topology_graph); u != INVALID; ++u){
            int id = (component_nodeMap)[u];
            newObjects.at(id).gmm.push_back((*topology_nodeMap)[u]);
        }
        // check velocity difference
        vector<double> velNorm;
        for(int i=0;i<num;i++){
            int size = newObjects.at(i).gmm.size();
            Eigen::Vector3d velAvg;
            for(int j=0;j<size;j++){
                Eigen::Vector3d vel = velAvg = velAvg + newObjects.at(i).gmm.at(j).velocity;
                velAvg += vel;
            }
            velAvg = velAvg/size;
            velNorm.push_back(velAvg.norm());
//            std::cout<<"vel diff : "<<velNorm[i]<<std::endl;
        }
        double sumVelNorm = 0.;
        for(int i=0;i<velNorm.size();i++){
            sumVelNorm += velNorm.at(i);
        }
        // entropy
        double entropy = 0.;
        for(int i=0;i<velNorm.size();i++){
            entropy += (velNorm.at(i)/sumVelNorm) * log(velNorm.at(i)/sumVelNorm);
        }
        entropy = -entropy;
//        cout<<"ENTROPY "<<entropy<<endl;
//            if(fabs(velNorm.at(i)-velNorm.at(i-1)) < 50000)
//                return 0;
//        }

        // point matching
        double sumSizeGMMs = 0.;
        for(int i=0;i<num;i++){
            sumSizeGMMs += newObjects.at(i).gmm.size();
        }
        for(int i=0;i<points.size();i++){
            Point point = points.at(i);
            int maxTrack = 0;
            double maxProb = 0.;
            for(int j=0;j<num;j++){
                double prob = newObjects.at(j).evalNormedGMM(point, sumSizeGMMs);
                //            double prob = object->evalGMM(point);
                //            double prob = object->evalClosestGMM(point);
                if(prob > maxProb){
                    maxProb = prob;
                    maxTrack = j;
                }
            }
            newObjects.at(maxTrack).points.push_back(point);
        }
        return num;
    }
}
