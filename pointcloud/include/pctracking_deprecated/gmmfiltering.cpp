#include "gmmfiltering.h"
#define pi 3.141592
#include <algorithm>

GMMFiltering::GMMFiltering()
{

}

struct myclass{
    bool operator() (Gaussian a, Gaussian b){
        return a.weight > b.weight;
    }
} myobject;


GMMFiltering::GMMFiltering(PCObject predictive, PCObject measurement)
{
    // time update
    PCObject filtering;

    // measurement update
    // measurement distribution
    filtering.gmm.clear();
    //    measurement.initialGMM();
    double weight_sum = 0.;
    for(int i=0;i<predictive.gmm.size();i++){
        Gaussian gauss_predictive = predictive.gmm.at(i);
        for(int j=0;j<measurement.gmm.size();j++){
            Gaussian gauss_measurement = measurement.gmm.at(j);
            Gaussian gauss_filtering;
            Eigen::Matrix3d cov_measurement = gauss_measurement.covariance;
            double cov_measurement_determinant = gauss_measurement.cov_determinant;
            Eigen::Matrix3d invcov_measurement = gauss_measurement.cov_inverse;
            Eigen::Matrix3d cov_predictive = gauss_predictive.covariance;
            double cov_predictive_determinant = gauss_predictive.cov_determinant;

            Eigen::Matrix3d invcov_predictive = gauss_predictive.cov_inverse;
            Eigen::Matrix3d invsum = invcov_measurement + invcov_predictive;

            Eigen::Vector3d mean_measurement = gauss_measurement.mean;
            Eigen::Vector3d mean_predictive = gauss_predictive.mean;

            // covariance
            Eigen::Matrix3d cov = invsum.inverse();
            gauss_filtering.covariance = cov;
            gauss_filtering.cov_determinant = gauss_filtering.covariance.determinant();
            gauss_filtering.cov_inverse = gauss_filtering.covariance.inverse();

            // mean
            gauss_filtering.mean = cov*invcov_measurement*mean_measurement + cov*invcov_predictive*mean_predictive;

            // weight
            double weight_cov, weight_normal;
            Eigen::Matrix3d cov_sum = cov_measurement + cov_predictive;
            weight_cov = (sqrt(gauss_filtering.cov_determinant) * sqrt(cov_sum.determinant())) / (sqrt(cov_measurement_determinant) * sqrt(cov_predictive_determinant));
            Eigen::Vector3d mean_dif = mean_measurement-mean_predictive;
            double a = mean_dif.transpose()*cov_sum.inverse()*mean_dif;
            weight_normal = 1./sqrt(pow(2*pi,3)*gauss_filtering.cov_determinant)*exp(-0.5*a);

            gauss_filtering.weight = gauss_measurement.weight * gauss_predictive.weight * weight_cov * weight_normal;
            weight_sum += gauss_filtering.weight;

            filtering.gmm.push_back(gauss_filtering);
        }
    }
    for(int i=0;i<filtering.gmm.size();i++){
        filtering.gmm.at(i).weight /= weight_sum;

    }
    cout<<"*********************************************Filtering end : n = "<<filtering.gmm.size()<<endl;


    // simplification
    double lambda = 0.3;
    int n = (predictive.gmm.size() + measurement.gmm.size())/2;

    cout<<"$$$$$$$$$$$$$$$$$ simplification start with # : "<<filtering.gmm.size()<<" to : "<<n<<endl;

    //    // simplification using weight threshold
    //    sort(filtering.gmm.begin(), filtering.gmm.end(), myobject);
    //    posterior.gmm.clear();
    //    weight_sum = 0.;
    //    for(int i=0;i<n;i++){
    //        posterior.gmm.push_back(filtering.gmm.at(i));
    //        weight_sum += filtering.gmm.at(i).weight;
    //    }
    //    for(int i=0;i<posterior.gmm.size();i++){
    //        posterior.gmm.at(i).weight /= weight_sum;
    //    }

    posterior.gmm = filtering.gmm;

    posterior.simplify(SIMPLE_HCKL, 0, n);
    cout<<"$$$$$$$$$$$$$$$$$ simplification end with # :"<<posterior.gmm.size()<<endl;




    //    posterior = posterior;

    posterior.points = measurement.points;
}

GMMFiltering::GMMFiltering(PCObject prior, PCObject measurement, vnl_vector<double> param)
{
    scale_noise = 0.015;
    scale_measurement = 10.;

    cov_noise(0,0) = scale_noise*scale_noise;
    cov_noise(0,1) = 0;
    cov_noise(0,2) = 0;
    cov_noise(1,0) = 0;
    cov_noise(1,1) = scale_noise*scale_noise;
    cov_noise(1,2) = 0;
    cov_noise(2,0) = 0;
    cov_noise(2,1) = 0;
    cov_noise(2,2) = scale_noise*scale_noise;

    // time update
    PCObject filtering;
    predictive = perform_transform(prior, param);

    // measurement update
    // measurement distribution
    filtering.gmm.clear();
    //    measurement.initialGMM();
    double weight_sum = 0.;
    for(int i=0;i<predictive.gmm.size();i++){
        Gaussian gauss_predictive = predictive.gmm.at(i);
        for(int j=0;j<measurement.gmm.size();j++){
            Gaussian gauss_measurement = measurement.gmm.at(j);
            Gaussian gauss_filtering;
            Eigen::Matrix3d cov_measurement = gauss_measurement.covariance;
            Eigen::Matrix3d invcov_measurement = cov_measurement.inverse();
            Eigen::Matrix3d cov_predictive = gauss_predictive.covariance;
            Eigen::Matrix3d invcov_predictive = cov_predictive.inverse();
            Eigen::Matrix3d invsum = invcov_measurement + invcov_predictive;

            Eigen::Vector3d mean_measurement = gauss_measurement.mean;
            Eigen::Vector3d mean_predictive = gauss_predictive.mean;

            // covariance
            Eigen::Matrix3d cov = invsum.inverse();
            gauss_filtering.covariance = cov;

            // mean
            gauss_filtering.mean = cov*invcov_measurement*mean_measurement + cov*invcov_predictive*mean_predictive;

            // weight
            double weight_cov, weight_normal;
            Eigen::Matrix3d cov_sum = cov_measurement + cov_predictive;
            weight_cov = (sqrt(cov.determinant()) * sqrt(cov_sum.determinant())) / (sqrt(cov_measurement.determinant()) * sqrt(cov_predictive.determinant()));
            Eigen::Vector3d mean_dif = mean_measurement-mean_predictive;
            double a = mean_dif.transpose()*cov_sum.inverse()*mean_dif;
            weight_normal = 1./sqrt(pow(2*pi,3)*cov.determinant())*exp(-0.5*a);

            gauss_filtering.weight = gauss_measurement.weight * gauss_predictive.weight * weight_cov * weight_normal;
            weight_sum += gauss_filtering.weight;


            //            double scale = 0.015;

            filtering.gmm.push_back(gauss_filtering);
        }
    }
    for(int i=0;i<filtering.gmm.size();i++){
        filtering.gmm.at(i).weight /= weight_sum;

    }
    cout<<"*********************************************Filtering end : n = "<<filtering.gmm.size()<<endl;


    // simplification
    double lambda = 0.3;
    int n = (predictive.gmm.size() + measurement.gmm.size())/2;

    cout<<"$$$$$$$$$$$$$$$$$ simplification start with # : "<<filtering.gmm.size()<<" to : "<<n<<endl;

    //    // simplification using weight threshold
    //    sort(filtering.gmm.begin(), filtering.gmm.end(), myobject);
    //    posterior.gmm.clear();
    //    weight_sum = 0.;
    //    for(int i=0;i<n;i++){
    //        posterior.gmm.push_back(filtering.gmm.at(i));
    //        weight_sum += filtering.gmm.at(i).weight;
    //    }
    //    for(int i=0;i<posterior.gmm.size();i++){
    //        posterior.gmm.at(i).weight /= weight_sum;
    //    }

    posterior.gmm = filtering.gmm;

    for(int i=0;i<filtering.gmm.size();i++){
        //        cout<<"before : " <<filtering.gmm.at(i).covariance(0,0)<<endl;
        posterior.gmm.at(i).covariance = filtering.gmm.at(i).covariance + cov_noise;
        //        cout<<"after : " <<posterior.gmm.at(i).covariance(0,0)<<endl;

    }
    posterior.simplify(SIMPLE_FA, 0, n);
    cout<<"$$$$$$$$$$$$$$$$$ simplification end with # :"<<posterior.gmm.size()<<endl;




    //    posterior = posterior;

    posterior.points = measurement.points;
}

GMMFiltering::GMMFiltering(PCObject predictive, CloudPtr measurement, double minEvalProb, CloudPtr& unmatchedPoints, double scale)
{
    posterior.gmm.clear();
    int nGaussians = predictive.gmm.size();
    vector<CloudPtr> pointGroups;
    for(int i=0;i<nGaussians;i++){
        CloudPtr pointGroup;
        pointGroup.reset(new Cloud);
        pointGroups.push_back(pointGroup);
    }
    int nMeasuredPoints = measurement->points.size();

    // grouping measured points into each gaussian and unmatched points
    for(int i=0;i<nMeasuredPoints;i++){
        Point point(measurement->points.at(i));
        // grouping points for each gaussian
        int maxGaussian;
        double maxEval = 0.;
        for(int j=0;j<nGaussians;j++){
            Gaussian gaussian = predictive.gmm.at(j);
            double eval = gaussian.evalPoint(point);
            if(eval > maxEval){
                maxEval = eval;
                maxGaussian = j;
            }
        }
//        if(maxEval > minEvalProb){
            pointGroups.at(maxGaussian)->points.push_back(measurement->points.at(i));
//        }
//        else{   // unmatched point
//            unmatchedPoints->points.push_back(measurement->points.at(i));
//        }
    }

    // if there is measured points in each gaussian -> gaussian filtering
    double weight_sum = 0.;
    for(int i=0;i<nGaussians;i++){
        int nPoints = pointGroups.at(i)->points.size();
        //        cout<<"n points"<<nPoints<<endl;
        if(nPoints > 0){
            Gaussian measure(pointGroups.at(i), scale);
            measure.weight = (double)nPoints / (double)(nMeasuredPoints - unmatchedPoints->points.size());
            //            measure.weight = predictive.gmm.at(i).weight;
            //            cout<<"weight"<<measure.weight<<endl;
            // posterior

            posterior.gmm.push_back(measure);

            for(int j=0;j<nPoints;j++){
                Point point(pointGroups.at(i)->points.at(j));
                posterior.points.push_back(point);
            }

            /*
            Gaussian gauss_measurement(pointGroups.at(i), scale);
            gauss_measurement.weight = (double)nPoints / (double)(nMeasuredPoints - unmatchedPoints->points.size());
            Gaussian gauss_predictive = predictive.gmm.at(i);

            Gaussian gauss_filtering;
            Eigen::Matrix3d cov_measurement = gauss_measurement.covariance;
            double cov_measurement_determinant = gauss_measurement.cov_determinant;
            Eigen::Matrix3d invcov_measurement = gauss_measurement.cov_inverse;
            Eigen::Matrix3d cov_predictive = gauss_predictive.covariance;
            double cov_predictive_determinant = gauss_predictive.cov_determinant;

            Eigen::Matrix3d invcov_predictive = gauss_predictive.cov_inverse;
            Eigen::Matrix3d invsum = invcov_measurement + invcov_predictive;

            Eigen::Vector3d mean_measurement = gauss_measurement.mean;
            Eigen::Vector3d mean_predictive = gauss_predictive.mean;

            // covariance
            Eigen::Matrix3d cov = invsum.inverse();
            gauss_filtering.covariance = cov;
            gauss_filtering.cov_determinant = gauss_filtering.covariance.determinant();
            gauss_filtering.cov_inverse = gauss_filtering.covariance.inverse();

            // mean
            gauss_filtering.mean = cov*invcov_measurement*mean_measurement + cov*invcov_predictive*mean_predictive;

            // weight
            double weight_cov, weight_normal;
            Eigen::Matrix3d cov_sum = cov_measurement + cov_predictive;
            weight_cov = (sqrt(gauss_filtering.cov_determinant) * sqrt(cov_sum.determinant())) / (sqrt(cov_measurement_determinant) * sqrt(cov_predictive_determinant));
            Eigen::Vector3d mean_dif = mean_measurement-mean_predictive;
            double a = mean_dif.transpose()*cov_sum.inverse()*mean_dif;
            weight_normal = 1./sqrt(pow(2*pi,3)*gauss_filtering.cov_determinant)*exp(-0.5*a);

            gauss_filtering.weight = gauss_measurement.weight * gauss_predictive.weight * weight_cov * weight_normal;
            weight_sum += gauss_filtering.weight;







//            // posterior
//            Gaussian post;
//            post.covariance = prior.cov_inverse + measure.cov_inverse;
//            Eigen::Matrix3d inv = post.covariance.inverse();
//            post.covariance = inv;
//            post.mean = post.covariance*prior.cov_inverse*prior.mean + post.covariance*measure.cov_inverse*measure.mean;
//            post.weight = measure.weight*prior.weight *  ;

            posterior.gmm.push_back(gauss_filtering);

//            for(int j=0;j<nPoints;j++){
//                Point point(pointGroups.at(i)->points.at(j));
//                posterior.points.push_back(point);
//            }
*/
        }
    }

//    for(int i=0;i<posterior.gmm.size();i++){
//        posterior.gmm.at(i).weight /=  weight_sum;
//    }
    cout<<"# Posterior GMM"<<posterior.gmm.size()<<endl;
    // because of 'collapsing' of gaussian mixture filtering, we need to regroup samples and refit each gaussian
    // Ref: 'Gaussian Sum Particle Filtering' by J. H. Kotecha, et. in IEEE Trans. on Signal Processing, Vol. 51, No.10, 2003

/*
    // clear
    unmatchedPoints->points.clear();
//    for(int i=0;i<nGaussians;i++){
//        pointGroups.at(i).reset();
//    }
//    pointGroups.clear();
    vector<CloudPtr> newPointGroups;
    nGaussians = posterior.gmm.size();
    for(int i=0;i<nGaussians;i++){
        CloudPtr newPointGroup;
        newPointGroup.reset(new Cloud);
        newPointGroups.push_back(newPointGroup);
    }

    // regrouping
    for(int i=0;i<nMeasuredPoints;i++){
        Point point(measurement->points.at(i));
        // grouping points for each gaussian
        int maxGaussian;
        double maxEval = 0.;
        for(int j=0;j<nGaussians;j++){
            Gaussian gaussian = posterior.gmm.at(j);
            double eval = gaussian.evalPoint(point) / weight_sum;
            if(eval > maxEval){
                maxEval = eval;
                maxGaussian = j;
            }
        }
        if(maxEval > 0.000001){
            newPointGroups.at(maxGaussian)->points.push_back(measurement->points.at(i));
        }
        else{   // unmatched point
            unmatchedPoints->points.push_back(measurement->points.at(i));
        }
    }

    // refit
    posterior.gmm.clear();
    // if there is measured points in each gaussian -> gaussian filtering
    for(int i=0;i<nGaussians;i++){
        int nPoints = newPointGroups.at(i)->points.size();
        //        cout<<"n points"<<nPoints<<endl;
        if(nPoints > 0){
            Gaussian measure(newPointGroups.at(i), scale);
            measure.weight = (double)nPoints / (double)(nMeasuredPoints - unmatchedPoints->points.size());
            //            measure.weight = predictive.gmm.at(i).weight;
            //            cout<<"weight"<<measure.weight<<endl;
            // posterior

            posterior.gmm.push_back(measure);

            for(int j=0;j<nPoints;j++){
                Point point(newPointGroups.at(i)->points.at(j));
                posterior.points.push_back(point);
            }
        }
    }
*/

    cout<<"# Posterior GMM"<<posterior.gmm.size()<<endl;
    cout<<"unmatchedPoints"<<unmatchedPoints->points.size()<<endl;
    // gaussian filtering

    //            Gaussian gaussian;

    //            posterior.insert();

}

PCObject GMMFiltering::perform_transform(PCObject prior, const vnl_vector<double> &x)
{
    int m = prior.gmm.size();
    vnl_matrix<double> translation;
    vnl_matrix<double> rotation;
    vnl_matrix<double> ones;
    ones.set_size(m,1);
    ones.fill(1);

    rotation.set_size(3,3);
    vnl_vector<double> q;
    q.set_size(4);
    for (int i=0;i<4;++i) q[i] = x[i];
    PCObject::quaternion2rotation(q, rotation);
    translation.set_size(1,3);
    translation[0][0] = x[4];
    translation[0][1] = x[5];
    translation[0][2] = x[6];

    PCObject object;
    for(int i=0;i<m;i++){
        Gaussian gauss;
        gauss = prior.gmm.at(i);
        vnl_matrix<double> mean(1,3), transmean(1,3);
        mean.put(0,0, gauss.mean[0]);
        mean.put(0,1, gauss.mean[1]);
        mean.put(0,2, gauss.mean[2]);
        transmean = mean*rotation.transpose() + translation;
        gauss.mean[0] = transmean[0][0];
        gauss.mean[1] = transmean[0][1];
        gauss.mean[2] = transmean[0][2];

        vnl_matrix<double> covariance(3,3), transcovariance(3,3), cov_noise(3,3);
        for(int j=0;j<3;j++){
            for(int k=0;k<3;k++){
                covariance.put(j,k, gauss.covariance(j,k));
                cov_noise.put(j,k,0.);
            }
        }

        cov_noise.put(0,0, scale_noise*scale_noise);
        cov_noise.put(1,1, scale_noise*scale_noise);
        cov_noise.put(2,2, scale_noise*scale_noise);

        transcovariance = rotation*covariance*rotation.transpose() + cov_noise;
        for(int j=0;j<3;j++)
            for(int k=0;k<3;k++)
                gauss.covariance(j,k) = covariance[j][k];
        object.gmm.push_back(gauss);
    }
    return object;
}
