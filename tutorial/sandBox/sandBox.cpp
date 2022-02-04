#include "tutorial/sandBox/sandBox.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include "Eigen/dense"
#include <functional>

//Project comment
#include <igl/directed_edge_orientations.h>
#include <igl/forward_kinematics.h>
#include <igl/dqs.h>
#include <iostream>
#include <set>
using namespace Eigen;
using namespace igl;
using namespace std;
using namespace opengl;
//end comment Project



SandBox::SandBox()
{
	

}

void SandBox::Init(const std::string &config)
{

    std::string item_name;
    std::ifstream nameFileout;
    doubleVariable = 0;
    nameFileout.open(config);
    if (!nameFileout.is_open())
    {
        std::cout << "Can't open file " << config << std::endl;
    }
    else
    {

        while (nameFileout >> item_name)
        {
            std::cout << "openning " << item_name << std::endl;
            load_mesh_from_file(item_name);
            data().show_overlay_depth = true;
            data().show_texture = true;
            data().point_size = 10;
            data().line_width = 2;
            data().show_overlay = 1;
            data().show_lines = 1;
            data().show_overlay_depth = 2;
            data().set_visible(false, 1);
            data().set_visible(true, 2);
            parents.push_back(-1);
            //data().MyScale(Vector3d(1, 1, 8)); //scaling the snake
            // //Assignmet1

            // //Assignemt 2
            //data().MyTranslate(Eigen::Vector3d(pos, 0, -1), true);
            // pos = pos + 1.8;
            // drawBox(&trees[selected_data_index]->m_box, selected_data_index);

        }
        nameFileout.close();
    }

   add_joints();
   drawJoints();
   initDataStructure(data().V, data().F);
   //MyTranslate(Eigen::Vector3d(0, 0, -14), true);
    //setVelocity(Eigen::Vector3d(-0.009, 0, 0));
    data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));
    //joints[1]->MyRotate(Eigen::Vector3d(0, 0, 1), 0.05);

    //Assignment3
    //setInitialPosition();


	//std::string item_name;
	//std::ifstream nameFileout;
	//doubleVariable = 0;
	//nameFileout.open(config);
	//if (!nameFileout.is_open())
	//{
	//	std::cout << "Can't open file " << config << std::endl;
	//}
	//else
	//{

	//	while (nameFileout >> item_name)
	//	{
	//		std::cout << "openning " << item_name << std::endl;
	//		load_mesh_from_file(item_name);

	//		parents.push_back(-1);
	//		data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
	//		data().show_overlay_depth = false;
	//		data().point_size = 10;
	//		data().line_width = 2;
	//		data().set_visible(false, 1);

	//		//Project comment
	//		parents.push_back(-1);
	//		data().MyScale(Eigen::Vector3d(1, 1, 8));
	//		//end comment Project


	//	}
	//	nameFileout.close();
	//}

	////Project comment
	//add_joints();
	//drawJoints();
	//initDataStructure(data().V, data().F);
	//MyTranslate(Eigen::Vector3d(0, 0, -14), true);
	////setVelocity(Eigen::Vector3d(-0.009, 0, 0));
	////data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));

	////MyTranslate(Eigen::Vector3d(0, 0, -1), true);

	//data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));
	//end comment Project

	//Ass 3
	/*
	load_mesh_from_file("C:/Users/97254/Desktop/run_animation2/Animation3D/tutorial/data/sphere.obj");
	//load_mesh_from_file("C:/Users/roi52/Desktop/ThreeDAnimationCourse/EngineForAnimationCourse/tutorial/data/sphere.obj");

	Eigen::RowVector3d center(5, 0, 0);
	parents.push_back(-1);
	// data().add_points(center, Eigen::RowVector3d(0, 0, 1));
	data().show_overlay_depth = false;
	data().point_size = 10;
	data().line_width = 2;
	data().set_visible(false, 1);
	data().MyTranslateInSystem(data().GetRotation(), center);
	//data().SetCenterOfRotation(center.transpose());

	MyTranslateInSystem(GetRotation(), Eigen::RowVector3d(0, 0, -8));
	data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));
	*/
	//Ass 3
}

SandBox::~SandBox()
{

}


//Project comment
void SandBox::initDataStructure(Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    W.resize(data_list[0].V.rows(), joints.size());
    add_weights();

    //from dual quaternion libgil tuturial 404

    RotationList rest_pose;
    igl::directed_edge_orientations(C, BE, rest_pose);

    poses.resize(4, RotationList(4, Eigen::Quaterniond::Identity()));
    const Quaterniond twist(AngleAxisd(M_PI, Vector3d(1, 0, 0)));
    poses[1][2] = rest_pose[2] * twist * rest_pose[2].conjugate();
    const Quaterniond bend(AngleAxisd(M_PI * 0.7, Vector3d(0, 0, 1)));
    poses[3][2] = rest_pose[2] * bend * rest_pose[2].conjugate();



    //Assingment 2
    /*AABB<MatrixXd, 3>* treeTmp = new AABB<MatrixXd, 3>;
    treeTmp->init(data().V, data().F);
    trees.push_back(treeTmp);
    subTrees.push_back(treeTmp);
    velocities.push_back(Eigen::Vector3d(-0.009, 0, 0));*/

}


void SandBox::Skinning()
{
    if (recompute)
    {
        //find pose interval
        const int begin = (int)floor(anim_t) % poses.size();
        const int end = (int)(floor(anim_t) + 1) % poses.size();
        const double t = anim_t - floor(anim_t);

        // Interpolate pose and identity
        RotationList anim_pose(poses[begin].size());
        for (int e = 0; e < poses[begin].size(); e++)
        {
            anim_pose[e] = poses[begin][e].slerp(t, poses[end][e]);
        }

        RotationList vQ; //rotation of joints
        vector<Vector3d> vT; //translation of joints

        //igl::forward_kinematics(C, BE, P, anim_pose, vQ, vT);

        //our forward kinematics start

        vQ.resize(17);
        vT.resize(17);


        Eigen::Vector3d b;
        vector<Eigen::Vector3d> before;// array to save old  points position of the links( links+1 points) 
        vector<Eigen::Vector3d> new_points;// array to save new  points position of the links( links+1 points) 
        int linknum = data_list.size() - 1;//because we dont need the sphere
        int pointArraySize = linknum + 1;//( links+1 points)
        //Eigen::Vector3d target = getTarget();
        double di = 1.6;// distance between 2 points(next to each other points) pi+1 pi will be the size of cylinder
        double ri, lambda;

        for (int i = 0; i < 17; i++) {//pushing only the point of cylinders 
            before.push_back(JointsPoses.at(i));// i+1 because we tart i from 0, and tip function start from 1 . the index of root cylinder
        }

        // Vector3d p1 = getTipbyindex(1);
        // double dist = distance_2Points(p1, target);
        // double sumOf_di = di * (pointArraySize - 1);//pointArraySize its num of pi's , pointArraySize-1 its num of (pi+1-pi)
        /* if (dist > sumOf_di) {
             cout << "cannot reach" << endl;
             isActive = false;
             return;
         }*/


        for (int i = 0; i < 17; i++) {
            new_points.push_back(JointsPoses.at(i));
        }

        //Eigen::Vector3d p1 = new_points.at(0);
        //Eigen::Vector3d pn = new_points.at(new_points.size() - 1);
        //b = p1;
        //double difA = distance_2Points(pn, target);

        //new_points.at(new_points.size() - 1) = target;// pushing target to the vector
        //forward reaching
        //int pn_mius_1_index = pointArraySize - 2;// because pointArraySize - 2 is last index(pn)
        for (int i = 15; i >= 0; i--) {
            ri = distance_2Points(new_points.at(i + 1), new_points.at(i));
            lambda = di / ri;
            new_points.at(i) = (1 - lambda) * new_points.at(i + 1) + lambda * new_points.at(i);
            Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(new_points[i], JointsPoses[i]);
            quat = quat.slerp(0.95, Eigen::Quaterniond::Identity());
            vQ.at(i) = quat;
            vT.at(i) = new_points[i];

        }

        Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(new_points[16], JointsPoses[16]);
        //quat = quat.slerp(0.95, Eigen::Quaterniond::Identity());
        vQ.at(16) = quat;
        vT.at(16) = new_points[16];

        //joints[i]->RotateInSystem(mat.block<3, 3>(0, 0), quat);
        //igl::dqs(data_list[0].V, W, vQ, vT, UU);
        //data_list[0].set_vertices(UU);
        //igl::dqs(data_list[0].V, W, vQ, vT, U);
       // data_list[0].set_vertices(U);
        //data_list[0].set_edges(CT, BET, sea_green);
        //data_list[0].compute_normals();


        //update the joints to the new one

        for (signed int i = 16; i >= 0; i--) {
            JointsPoses.at(i) = new_points[i];
        }





        //our forward kinemtics ends

        const int dim = C.cols();
        MatrixXd T(BE.rows() * (dim + 1), dim);
        for (int e = 0; e < BE.rows(); e++)
        {
            Affine3d a = Affine3d::Identity();
            a.translate(vT[e]);
            a.rotate(vQ[e]);
            T.block(e * (dim + 1), 0, dim + 1, dim) = a.matrix().transpose().block(0, 0, dim + 1, dim);
        }

        igl::dqs(data_list[0].V, W, vQ, vT, U);
        //U = M * T;

        MatrixXd CT;
        MatrixXi BET;
        CT.resize(2 * BE.rows(), C.cols());
        BET.resize(BE.rows(), 2);
        for (int e = 0; e < BE.rows(); e++)
        {
            BET(e, 0) = 2 * e;
            BET(e, 1) = 2 * e + 1;
            Matrix4d t;
            t << T.block(e * 4, 0, 4, 3).transpose(), 0, 0, 0, 0;
            Affine3d a;
            a.matrix() = t;
            Vector3d c0 = C.row(BE(e, 0));
            Vector3d c1 = C.row(BE(e, 1));
            CT.row(2 * e) = a * c0;
            CT.row(2 * e + 1) = a * c1;
        }
        data_list[0].set_vertices(U);
        data_list[0].set_edges(CT, BET, sea_green);
        data_list[0].compute_normals();

        if (isActive)
        {
            anim_t += anim_t_dir;
        }
        else
        {
            recompute = false;
        }
    }
}
void SandBox::forwardLoop()
{
    vector<Eigen::Vector3d> tmpPoses = JointsPoses;
    for (signed int i = num_of_links - 1; i >= 0; i--) {
        double ri = (tmpPoses[i + 1] - tmpPoses[i]).norm();
        double lambda = 1.6 / ri;
        Vector3d newPos = (((1 - lambda) * tmpPoses[i + 1]) + (lambda * tmpPoses[i])).normalized();
        tmpPoses[i] = newPos;
        Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(tmpPoses[i], JointsPoses[i]);
        Eigen::Matrix3d mat = Eigen::Matrix3d::Identity();
        for (int j = i; j > -1; j++) {
            mat = mat * joints[i]->GetRotation();
        }
        quat = quat.slerp(0.95, Eigen::Quaterniond::Identity());

        joints[i]->RotateInSystem(mat.block<3, 3>(0, 0), quat);
        //update tip position
    }
}
/*
void SandBox::setVelocity(Eigen::Vector3d dir)
{
    if (velocities.size() == 0)
        velocities.push_back(dir);
    else
        velocities[0] = dir;
}
*/

double SandBox::calc_related_distance(int i) {
    double sum = 0;
    double distance;
    for (int j = 0; j < joints.size(); j++) {
        distance = abs(joints[j]->position.z() - data_list[0].V.row(i).z());
        if (distance <= 0.1) {
            sum += pow((1 / distance), 4);
        }
    }
    return sum;

}

double SandBox::sum(int i) {
    double sum = 0;
    for (int j = 0; j < W.row(i).size(); j++) {
        sum += W(i, j);
    }
    return sum;
}

void SandBox::add_weights() {
    //calc from article "automatic skinning weight retargeting 2017"
    double distance;
    for (int i = 0; i < data_list[0].V.rows(); i++) {
        double related_distance = calc_related_distance(i);// calc sum of sigma on  k=0 to n(jointd) (1/distance(i,k)^4)
        for (int j = 0; j < joints.size(); j++) {
            distance = abs(joints[j]->position.z() - data_list[0].V.row(i).z());
            double temp = pow((1 / distance), 4); // (1 / distance(i, j) ^ 4)
            W(i, j) = temp / related_distance;
        }
        W.row(i).normalized();
        //cout << "row " << i << " " << sum(i) << endl;
    }

}

Eigen::Vector3d SandBox::getJoint(int indx) {
    Vector3d joint = joints[1]->GetRotation() * Vector3d(0, 0, 1);
    for (int i = 1; i < indx; i++) {
        Matrix3d R = CalcParentsRot(i);
        joint = joint + R * Vector3d(0, 0, link_Len);
    }

    return joint;
}



void SandBox::add_joints() {
    parents.resize(17);
    P.resize(17);// supposed to be paretns, check if needed, maybe needed for differenet algorithm than parents variable
    C.resize(17, 3);//joint's positions
    Vector3d currPos = Vector3d(0, 0, -0.8);
    int parent = -1;
    for (int i = 0; i < num_of_joints; i++) {
        Joint* curr_joint = new Joint(currPos, i, parent);
        C.row(i) = currPos;
        JointsPoses.push_back(currPos);//posses is rotations of joints for animation
        joints.push_back(curr_joint);
        P[i] = parent;
        parents[i] = parent;
        parent++;
        currPos = currPos + Vector3d(0, 0, 0.1); //0.105);
    }
}
void SandBox::drawJoints() {

    Eigen::MatrixXd V_box(17, 3);
    for (int i = 0; i < 17; i++) {
        V_box.row(i) << joints[i]->position.x(), joints[i]->position.y(), joints[i]->position.z();
    }

    // Edges between joints
    BE.resize(16, 2);
    //Eigen::MatrixXi E_box(15, 2);
    BE <<
        0, 1,
        1, 2,
        2, 3,
        3, 4,
        4, 5,
        5, 6,
        6, 7,
        7, 8,
        8, 9,
        9, 10,
        10, 11,
        11, 12,
        12, 13,
        13, 14,
        14, 15,
        15, 16;

    data_list[0].add_points(V_box, Eigen::RowVector3d(0, 0, 1));

    for (unsigned i = 0; i < BE.rows(); ++i)
        data_list[0].add_edges
        (
            V_box.row(BE(i, 0)),
            V_box.row(BE(i, 1)),
            Eigen::RowVector3d(0, 0, 1)
        );
}

Eigen::Matrix3d SandBox::CalcParentsRot(int indx)
{
    Eigen::Matrix3d prevRot = Eigen::Matrix3d::Identity();

    for (int i = indx; P[i] >= 0; i = P[i])
    {
        prevRot = joints[i]->GetRotation() * prevRot;
    }

    return prevRot;
}

void SandBox::setJointsPositions()
{
    for (int i = 1; i <= num_of_joints; i++) {
        JointsPoses[i - 1] = getJoint(i);
    }
}
void SandBox::setHeadPosition()
{
    JointsPoses[num_of_joints] = (CalcParentsTrans(num_of_joints) * joints[num_of_joints]->MakeTransd() * Vector4d(0, 0, (0.1 / 2), 1)).head(3);
}
//end comment Project

void SandBox::Animate()
{
	if (isActive)
	{

        //project comment
        RotationList vQ; //rotation of joints
        vector<Vector3d> vT; //translation of joints
        //Eigen::MatrixXd UU;

        vQ.resize(17);
        vT.resize(17);


        Eigen::Vector3d b;
        vector<Eigen::Vector3d> before;// array to save old  points position of the links( links+1 points) 
        vector<Eigen::Vector3d> new_points;// array to save new  points position of the links( links+1 points) 
        int linknum = data_list.size() - 1;//because we dont need the sphere
        int pointArraySize = linknum + 1;//( links+1 points)
        //Eigen::Vector3d target = getTarget();
        double di = 1.6;// distance between 2 points(next to each other points) pi+1 pi will be the size of cylinder
        double ri, lambda;

        for (int i = 0; i < 17; i++) {//pushing only the point of cylinders 
            before.push_back(JointsPoses.at(i));// i+1 because we tart i from 0, and tip function start from 1 . the index of root cylinder
        }

       // Vector3d p1 = getTipbyindex(1);
       // double dist = distance_2Points(p1, target);
       // double sumOf_di = di * (pointArraySize - 1);//pointArraySize its num of pi's , pointArraySize-1 its num of (pi+1-pi)
       /* if (dist > sumOf_di) {
            cout << "cannot reach" << endl;
            isActive = false;
            return;
        }*/

        
            for (int i = 0; i < 17; i++) {
                new_points.push_back(JointsPoses.at(i));
            }

            //Eigen::Vector3d p1 = new_points.at(0);
            //Eigen::Vector3d pn = new_points.at(new_points.size() - 1);
            //b = p1;
            //double difA = distance_2Points(pn, target);

            //new_points.at(new_points.size() - 1) = target;// pushing target to the vector
            //forward reaching
            //int pn_mius_1_index = pointArraySize - 2;// because pointArraySize - 2 is last index(pn)
            for (int i = 15; i >= 0; i--) {
                ri = distance_2Points(new_points.at(i + 1), new_points.at(i));
                lambda = di / ri;
                new_points.at(i) = (1 - lambda) * new_points.at(i + 1) + lambda * new_points.at(i);
                Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(new_points[i], JointsPoses[i]);
                quat = quat.slerp(0.95, Eigen::Quaterniond::Identity());
                vQ.at(i) = quat;
                vT.at(i) = new_points[i];

            }

            Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(new_points[16], JointsPoses[16]);
            //quat = quat.slerp(0.95, Eigen::Quaterniond::Identity());
            vQ.at(16) = quat;
            vT.at(16) = new_points[16];

            //joints[i]->RotateInSystem(mat.block<3, 3>(0, 0), quat);
            //igl::dqs(data_list[0].V, W, vQ, vT, UU);
            //data_list[0].set_vertices(UU);
            igl::dqs(data_list[0].V, W, vQ, vT, U);
            data_list[0].set_vertices(U);
            //data_list[0].set_edges(CT, BET, sea_green);
            //data_list[0].compute_normals();


            //update the joints to the new one

            for (signed int i = 16; i >= 0; i--) {
                JointsPoses.at(i) = new_points[i];
            }
            //end project comment
        
        /*
        vector<Eigen::Vector3d> tmpPoses = JointsPoses;
        for (signed int i = num_of_links - 1; i >= 0; i--) {
            double ri = (tmpPoses[i + 1] - tmpPoses[i]).norm();
            double lambda = 1.6 / ri;
            Vector3d newPos = (((1 - lambda) * tmpPoses[i + 1]) + (lambda * tmpPoses[i])).normalized();
            tmpPoses[i] = newPos;
            Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(tmpPoses[i], JointsPoses[i]);
            quat = quat.slerp(0.95, Eigen::Quaterniond::Identity());
            vQ.at(i) = quat;
            vT.at(i) = tmpPoses[i];
        }
            
            //joints[i]->RotateInSystem(mat.block<3, 3>(0, 0), quat);
            igl::dqs(data_list[0].V, W, vQ, vT, UU);
            data_list[0].set_vertices(UU);

            //update the joints to the new one

            for (signed int i = num_of_links - 1; i >= 0; i--) {
                JointsPoses.at(i) = tmpPoses[i];
            }
            */


		//Ass 2 comment
		// using isActive as the key for knowing if we can move object[0] or not, in Assignment 2-Collision
		//my translate with pretranslate true
		//data_list[0].MyTranslate(moveDir,true);
		//data_list[moving_index].MyTranslateInSystem(GetRotation(), data_list[moving_index].moveDir);
		//checkCollision();
		//Ass2 comment end	
		//Ass3 comment
		//IKSimulation();//assignment 3 first implementation algorithm
		//FabrikAlgo();//assignment 3 main algoritm
		//end comment Ass3
        //forwardLoop();
        //Skinning();
	}
}


