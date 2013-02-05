#include "ckinecttflistener.h"

CKinectTFListener::CKinectTFListener()
{
    listener = new tf::TransformListener(ros::Duration(10));
}

void CKinectTFListener::listen()
{
    tf::StampedTransform tfData;
    /*
    string targetFrame, sourceFrame, strNum;

    humans.clear();
    humanNums = findHumans();
    if(humanNums.size()==0) return;
    // else
//    cout<<"Detected humans:";
//    for(int i=0;i<humanNums.size();i++){
//        cout<<" "<<humanNums.at(i);
//    }
//    cout<<endl;

    for(int i=0;i<humanNums.size();i++){
        Human human;
        human.id = humanNums.at(i);
        for(int j=0;j<jointNames.size();j++){
            sourceFrame = jointNames.at(j);
            targetFrame = "/torso";
            if(humanNums.at(i)!=0){
                string strNum;
                stringstream strm;
                strm << "_" << humanNums.at(i);
                strm >> strNum;
                sourceFrame += strNum;
                targetFrame += strNum;
            }
            tfData = transform(targetFrame, sourceFrame);

            orientationMat(mat, tfData.getRotation().w(), tfData.getRotation().x(), tfData.getRotation().y(), tfData.getRotation().z(), tfData.getOrigin().x(),tfData.getOrigin().y(),tfData.getOrigin().z());
            if(jointNames.at(j) == "/head"){
                human.head.x = mat[0][3];
                human.head.y = mat[1][3];
                human.head.z = mat[2][3];
                eularAngle(human.head.alpha, human.head.beta, human.head.gamma, mat);
                human.head.r_00 = mat[0][0];
                human.head.r_01 = mat[0][1];
                human.head.r_02 = mat[0][2];
                human.head.r_10 = mat[1][0];
                human.head.r_11 = mat[1][1];
                human.head.r_12 = mat[1][2];
                human.head.r_20 = mat[2][0];
                human.head.r_21 = mat[2][1];
                human.head.r_22 = mat[2][2];
            }
            if(jointNames.at(j) == "/neck"){
                human.neck.x = mat[0][3];
                human.neck.y = mat[1][3];
                human.neck.z = mat[2][3];
                eularAngle(human.neck.alpha, human.neck.beta, human.neck.gamma, mat);
                human.neck.r_00 = mat[0][0];
                human.neck.r_01 = mat[0][1];
                human.neck.r_02 = mat[0][2];
                human.neck.r_10 = mat[1][0];
                human.neck.r_11 = mat[1][1];
                human.neck.r_12 = mat[1][2];
                human.neck.r_20 = mat[2][0];
                human.neck.r_21 = mat[2][1];
                human.neck.r_22 = mat[2][2];
            }
            if(jointNames.at(j) == "/torso"){
                human.torso.x = mat[0][3];
                human.torso.y = mat[1][3];
                human.torso.z = mat[2][3];
                eularAngle(human.torso.alpha, human.torso.beta, human.torso.gamma, mat);
                human.torso.r_00 = mat[0][0];
                human.torso.r_01 = mat[0][1];
                human.torso.r_02 = mat[0][2];
                human.torso.r_10 = mat[1][0];
                human.torso.r_11 = mat[1][1];
                human.torso.r_12 = mat[1][2];
                human.torso.r_20 = mat[2][0];
                human.torso.r_21 = mat[2][1];
                human.torso.r_22 = mat[2][2];
            }
            if(jointNames.at(j) == "/left_shoulder"){
                human.left_shoulder.x = mat[0][3];
                human.left_shoulder.y = mat[1][3];
                human.left_shoulder.z = mat[2][3];
                eularAngle(human.left_shoulder.alpha, human.left_shoulder.beta, human.left_shoulder.gamma, mat);
                human.left_shoulder.r_00 = mat[0][0];
                human.left_shoulder.r_01 = mat[0][1];
                human.left_shoulder.r_02 = mat[0][2];
                human.left_shoulder.r_10 = mat[1][0];
                human.left_shoulder.r_11 = mat[1][1];
                human.left_shoulder.r_12 = mat[1][2];
                human.left_shoulder.r_20 = mat[2][0];
                human.left_shoulder.r_21 = mat[2][1];
                human.left_shoulder.r_22 = mat[2][2];
            }
            if(jointNames.at(j) == "/left_elbow"){
                human.left_elbow.x = mat[0][3];
                human.left_elbow.y = mat[1][3];
                human.left_elbow.z = mat[2][3];
                eularAngle(human.left_elbow.alpha, human.left_elbow.beta, human.left_elbow.gamma, mat);
                human.left_elbow.r_00 = mat[0][0];
                human.left_elbow.r_01 = mat[0][1];
                human.left_elbow.r_02 = mat[0][2];
                human.left_elbow.r_10 = mat[1][0];
                human.left_elbow.r_11 = mat[1][1];
                human.left_elbow.r_12 = mat[1][2];
                human.left_elbow.r_20 = mat[2][0];
                human.left_elbow.r_21 = mat[2][1];
                human.left_elbow.r_22 = mat[2][2];
            }
            if(jointNames.at(j) == "/left_hand"){
                human.left_hand.x = mat[0][3];
                human.left_hand.y = mat[1][3];
                human.left_hand.z = mat[2][3];
                eularAngle(human.left_hand.alpha, human.left_hand.beta, human.left_hand.gamma, mat);
                human.left_hand.r_00 = mat[0][0];
                human.left_hand.r_01 = mat[0][1];
                human.left_hand.r_02 = mat[0][2];
                human.left_hand.r_10 = mat[1][0];
                human.left_hand.r_11 = mat[1][1];
                human.left_hand.r_12 = mat[1][2];
                human.left_hand.r_20 = mat[2][0];
                human.left_hand.r_21 = mat[2][1];
                human.left_hand.r_22 = mat[2][2];
            }
            if(jointNames.at(j) == "/right_shoulder"){
                human.right_shoulder.x = mat[0][3];
                human.right_shoulder.y = mat[1][3];
                human.right_shoulder.z = mat[2][3];
                eularAngle(human.right_shoulder.alpha, human.right_shoulder.beta, human.right_shoulder.gamma, mat);
                human.right_shoulder.r_00 = mat[0][0];
                human.right_shoulder.r_01 = mat[0][1];
                human.right_shoulder.r_02 = mat[0][2];
                human.right_shoulder.r_10 = mat[1][0];
                human.right_shoulder.r_11 = mat[1][1];
                human.right_shoulder.r_12 = mat[1][2];
                human.right_shoulder.r_20 = mat[2][0];
                human.right_shoulder.r_21 = mat[2][1];
                human.right_shoulder.r_22 = mat[2][2];
            }
            if(jointNames.at(j) == "/right_elbow"){
                human.right_elbow.x = mat[0][3];
                human.right_elbow.y = mat[1][3];
                human.right_elbow.z = mat[2][3];
                eularAngle(human.right_elbow.alpha, human.right_elbow.beta, human.right_elbow.gamma, mat);
                human.right_elbow.r_00 = mat[0][0];
                human.right_elbow.r_01 = mat[0][1];
                human.right_elbow.r_02 = mat[0][2];
                human.right_elbow.r_10 = mat[1][0];
                human.right_elbow.r_11 = mat[1][1];
                human.right_elbow.r_12 = mat[1][2];
                human.right_elbow.r_20 = mat[2][0];
                human.right_elbow.r_21 = mat[2][1];
                human.right_elbow.r_22 = mat[2][2];
            }
            if(jointNames.at(j) == "/right_hand"){
                human.right_hand.x = mat[0][3];
                human.right_hand.y = mat[1][3];
                human.right_hand.z = mat[2][3];
                eularAngle(human.right_hand.alpha, human.right_hand.beta, human.right_hand.gamma, mat);
                human.right_hand.r_00 = mat[0][0];
                human.right_hand.r_01 = mat[0][1];
                human.right_hand.r_02 = mat[0][2];
                human.right_hand.r_10 = mat[1][0];
                human.right_hand.r_11 = mat[1][1];
                human.right_hand.r_12 = mat[1][2];
                human.right_hand.r_20 = mat[2][0];
                human.right_hand.r_21 = mat[2][1];
                human.right_hand.r_22 = mat[2][2];
            }
            if(jointNames.at(j) == "/left_hip"){
                human.left_hip.x = mat[0][3];
                human.left_hip.y = mat[1][3];
                human.left_hip.z = mat[2][3];
                eularAngle(human.left_hip.alpha, human.left_hip.beta, human.left_hip.gamma, mat);
                human.left_hip.r_00 = mat[0][0];
                human.left_hip.r_01 = mat[0][1];
                human.left_hip.r_02 = mat[0][2];
                human.left_hip.r_10 = mat[1][0];
                human.left_hip.r_11 = mat[1][1];
                human.left_hip.r_12 = mat[1][2];
                human.left_hip.r_20 = mat[2][0];
                human.left_hip.r_21 = mat[2][1];
                human.left_hip.r_22 = mat[2][2];
            }
            if(jointNames.at(j) == "/left_knee"){
                human.left_knee.x = mat[0][3];
                human.left_knee.y = mat[1][3];
                human.left_knee.z = mat[2][3];
                eularAngle(human.left_knee.alpha, human.left_knee.beta, human.left_knee.gamma, mat);
                human.left_knee.r_00 = mat[0][0];
                human.left_knee.r_01 = mat[0][1];
                human.left_knee.r_02 = mat[0][2];
                human.left_knee.r_10 = mat[1][0];
                human.left_knee.r_11 = mat[1][1];
                human.left_knee.r_12 = mat[1][2];
                human.left_knee.r_20 = mat[2][0];
                human.left_knee.r_21 = mat[2][1];
                human.left_knee.r_22 = mat[2][2];
            }
            if(jointNames.at(j) == "/left_foot"){
                human.left_foot.x = mat[0][3];
                human.left_foot.y = mat[1][3];
                human.left_foot.z = mat[2][3];
                eularAngle(human.left_foot.alpha, human.left_foot.beta, human.left_foot.gamma, mat);
                human.left_foot.r_00 = mat[0][0];
                human.left_foot.r_01 = mat[0][1];
                human.left_foot.r_02 = mat[0][2];
                human.left_foot.r_10 = mat[1][0];
                human.left_foot.r_11 = mat[1][1];
                human.left_foot.r_12 = mat[1][2];
                human.left_foot.r_20 = mat[2][0];
                human.left_foot.r_21 = mat[2][1];
                human.left_foot.r_22 = mat[2][2];
            }
            if(jointNames.at(j) == "/right_hip"){
                human.right_hip.x = mat[0][3];
                human.right_hip.y = mat[1][3];
                human.right_hip.z = mat[2][3];
                eularAngle(human.right_hip.alpha, human.right_hip.beta, human.right_hip.gamma, mat);
                human.right_hip.r_00 = mat[0][0];
                human.right_hip.r_01 = mat[0][1];
                human.right_hip.r_02 = mat[0][2];
                human.right_hip.r_10 = mat[1][0];
                human.right_hip.r_11 = mat[1][1];
                human.right_hip.r_12 = mat[1][2];
                human.right_hip.r_20 = mat[2][0];
                human.right_hip.r_21 = mat[2][1];
                human.right_hip.r_22 = mat[2][2];
            }
            if(jointNames.at(j) == "/right_knee"){
                human.right_knee.x = mat[0][3];
                human.right_knee.y = mat[1][3];
                human.right_knee.z = mat[2][3];
                eularAngle(human.right_knee.alpha, human.right_knee.beta, human.right_knee.gamma, mat);
                human.right_knee.r_00 = mat[0][0];
                human.right_knee.r_01 = mat[0][1];
                human.right_knee.r_02 = mat[0][2];
                human.right_knee.r_10 = mat[1][0];
                human.right_knee.r_11 = mat[1][1];
                human.right_knee.r_12 = mat[1][2];
                human.right_knee.r_20 = mat[2][0];
                human.right_knee.r_21 = mat[2][1];
                human.right_knee.r_22 = mat[2][2];
            }
            if(jointNames.at(j) == "/right_foot"){
                human.right_foot.x = mat[0][3];
                human.right_foot.y = mat[1][3];
                human.right_foot.z = mat[2][3];
                eularAngle(human.right_foot.alpha, human.right_foot.beta, human.right_foot.gamma, mat);
                human.right_foot.r_00 = mat[0][0];
                human.right_foot.r_01 = mat[0][1];
                human.right_foot.r_02 = mat[0][2];
                human.right_foot.r_10 = mat[1][0];
                human.right_foot.r_11 = mat[1][1];
                human.right_foot.r_12 = mat[1][2];
                human.right_foot.r_20 = mat[2][0];
                human.right_foot.r_21 = mat[2][1];
                human.right_foot.r_22 = mat[2][2];
            }

        }
        humans.push_back(human);
    }

    */
}
