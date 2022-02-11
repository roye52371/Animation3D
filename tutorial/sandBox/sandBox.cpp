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
    //start comment Project
    std::string item_name;
    std::ifstream nameFileout;
    doubleVariable = 0;

    right = true;// so when press space, it start move to the right and not screwed
    left = false;
    up = false;
    down = false;
    in = false;
    out = false;


    joints_num = 16;
    skelton.resize(joints_num + 1);
    //parents.resize(joints_num + 1);
    scale = 1;
    //Initialize vT, vQ
    vT.resize(17);
    vQ.resize(17);



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

            Eigen::RowVector3d center(0, 0, -0.8);
            parents.push_back(-1);
            data().show_overlay_depth = false;
            data().point_size = 10;
            data().line_width = 2;
            data().set_visible(false, 1);

            if (selected_data_index == 0)
                V = data().V;
        }
        nameFileout.close();
    }
    MyTranslate(Eigen::Vector3d(0, 0, -1), true);

    //Find points for skelton

    double z = -0.8 * scale;
    for (int i = 0; i < skelton.size(); i++)
    {
        skelton.at(i) = Eigen::Vector3d(0, 0, z);
        z = z + 0.1 * scale;
    }

    
    //Calaulate the weights for each vertex
    calc_all_weights();
    //add_weights();

    data().MyRotate(Eigen::Vector3d(0, 1, 0), 3.14 / 2);

    //Create Joints
    //the first joint that dont have a parent
    //Joints.emplace_back();
    //Joints.at(0).MyTranslate(skelton.at(0), true);
    //parents[1] = -1;
    //the 16 other joint that have parents
    /*for (int i = 1; i < joints_num; i++)
    {
        parents[i + 1] = i;
        Joints.emplace_back();
        Joints.at(i + 1).MyTranslate(skelton.at(i + 1), true);

    }*/


    target_pose = skelton[joints_num];
    //data_list[0].set_colors(Eigen::RowVector3d(165,90,0));//brownie color for the snake;
    U = V;

    //levelk();

    //end comment Project


  

    
}

SandBox::~SandBox()
{

}


//Project comment

//double SandBox::calc_related_distance(int i) {
//    double sum = 0;
//    double distance;
//    for (int j = 0; j < joints.size(); j++) {
//        distance = abs(joints[j]->position.z() - data_list[0].V.row(i).z());
//        if (distance <= 0.1) {
//            sum += pow((1 / distance), 4);
//        }
//    }
//    return sum;
//
//}
//
//double SandBox::sum(int i) {
//    double sum = 0;
//    for (int j = 0; j < W.row(i).size(); j++) {
//        sum += W(i, j);
//    }
//    return sum;
//}
//
//void SandBox::add_weights() {
//    //calc from article "automatic skinning weight retargeting 2017"
//    double distance;
//    for (int i = 0; i < data_list[0].V.rows(); i++) {
//        double related_distance = calc_related_distance(i);// calc sum of sigma on  k=0 to n(jointd) (1/distance(i,k)^4)
//        for (int j = 0; j < joints.size(); j++) {
//            distance = abs(joints[j]->position.z() - data_list[0].V.row(i).z());
//            double temp = pow((1 / distance), 4); // (1 / distance(i, j) ^ 4)
//            W(i, j) = temp / related_distance;
//        }
//        W.row(i).normalized();
//        //cout << "row " << i << " " << sum(i) << endl;
//    }
//
//}
//end comment Project



//------------------------------------------------Final Project-----------------------------------


Eigen::VectorXd SandBox::create_weight_vec(double w1, double w1_ind, double w2, double w2_ind)
{
    Eigen::VectorXd Wi;
    Wi.resize(17);
    double weight1 = w1;
    double index1 = w1_ind;
    double weight2 = w2;
    double index2 = w2_ind;

    for (double i = 0; i < 17; i++)
    {
        if (i == index1)
            Wi[index1] = weight1;
        else {
            if (i == index2) 
                Wi[index2] = weight2;
            else
                Wi[i] = 0;
        }
    }
    return Wi;
}

void  SandBox::calc_all_weights()
{
    int verticeSize = data_list[0].V.rows();
    Eigen::MatrixXd V = data_list[0].V;
    W.resize(verticeSize, 17);

    double z_axe_coord, w_1, w_2, lower_bound, upper_bound;
    //int lower_bound, upper_bound;
    for (int i = 0; i < verticeSize; i++){
        z_axe_coord = V.row(i)[2];
        lower_bound = (floor(z_axe_coord * 10)) / 10;
        upper_bound = (ceil(z_axe_coord * 10)) / 10;
        w_1 = abs(z_axe_coord - upper_bound) * 10;
        w_2 = 1 - w_1;
        W.row(i) = create_weight_vec(w_1, lower_bound * 10 + 8, w_2, upper_bound * 10 + 8);
    }
}

void  SandBox::calc_next_pos()
{
    vT[0] = skelton[0];
    for (int i = 0; i < joints_num; i++) {
        vT[i + 1] = skelton[i + 1];
        vT[i] = vT[i] + ((vT[i + 1] - vT[i]) / 6);
    }
    vT[joints_num] = vT[joints_num] + target_pose;
}

//------------------------------------------------Final Project-----------------------------------


/////////////////
void SandBox::add_weights() {
    //calc from article "automatic skinning weight retargeting 2017"
    double distance;
    int numOfV = data_list.at(0).V.rows();
    Eigen::MatrixXd V = data_list.at(0).V;
    W.resize(numOfV, 17);
   // printf("1\n");

    for (int i = 0; i < data_list[0].V.rows(); i++) {
        double related_distance = calc_related_distance(i);// calc sum of sigma on  k=0 to n(jointd) (1/distance(i,k)^4)
        for (int j = 0; j < skelton.size(); j++) {
            //printf("1.1\n");
            distance = abs(skelton.at(j).z() - data_list[0].V.row(i).z());
            //printf("1.11\n");
            double temp = pow((1 / distance), 4);
            //printf("1.111\n");
            
            W(i, j) = temp / related_distance;
            //printf("1.111111\n");
        }
        W.row(i).normalized();
        //cout << "row " << i << " " << sum(i) << endl;
    }
    //printf("2\n");
}

double SandBox::calc_related_distance(int i) {
    double sum = 0;
    double distance;
    //printf("3\n");
    for (int j = 0; j < skelton.size(); j++) {
        distance = abs(skelton.at(j).z() - data_list[0].V.row(i).z());
        if (distance <= 0.1) {
            sum += pow((1 / distance), 4);
        }
    }
    //printf("4\n");
    return sum;
}


//Project levels  functions
void SandBox::levelk() 
{
    if (score >= targetScore * level) {

        //Todo: need to stop game and show menu that ask if do the same level or pass to next one

        isNextLevel = true;

        isActive = false;
        for (int i = 1; i < data_list.size(); i++)
        {
            data_list[i].clear();// clear all food
        }
        //try to reset snake
        data_list[0].set_vertices(data_list[0].OV);// OV keeping the first vertics we had to the snake

        //Todo: need to stop game and show menu that ask if do the same level or pass to next one
        //after decide what level to be, to update it , and the score, and isActive to be True
        //level++;// need to be update in the menu code after finishing level and decide were to continue
        //score = 0;//need to be update in the menu code after finishing level and decide were to continue
    }
    else {
        generate_target(level);
        move_targets(level);
    }

    /*for (int i = 0; i < level; i++)
    {
         load_mesh_from_file("C:/Users/97254/Desktop/run_animation2/Animation3D/tutorial/data/cube.obj");
        
            
           parents.push_back(-1);
           data_list.back().set_visible(false, 1);
           data().show_overlay_depth = false;
           data().point_size = 10;
           data().line_width = 2;
           data().set_visible(false, 1);
           data().MyTranslate(Eigen::Vector3d(0, 2 * (i+1), 0), true);


          load_mesh_from_file("C:/Users/97254/Desktop/run_animation2/Animation3D/tutorial/data/sphere.obj");
  

          parents.push_back(-1);
          data_list.back().set_visible(false, 1);
          data().show_overlay_depth = false;
          data().point_size = 10;
          data().line_width = 2;
          data().set_visible(false, 1);
          data().MyTranslate(Eigen::Vector3d(0, -2 * (i+1), 0), true);
    }
    initTreesAndDrawForCollision();*/

}
//end comment Project


/////////////////////

void SandBox::Animate()
{
	if (isActive)
	{
       /* for (int i = 1; i < data_list.size(); i++)
        {
            data_list[i].MyTranslate(Eigen::Vector3d(-0.004, 0, 0),true);
        }*/
        
        //Project comment
        if (left) {
            target_pose = Eigen::Vector3d(0, 0, -0.03);
        }
        else if (right) {
            target_pose = Eigen::Vector3d(0, 0, 0.03);
        }
        else if (up) {
            target_pose = Eigen::Vector3d(0, 0.03, 0);
        }
        else if (down) {
            target_pose = Eigen::Vector3d(0, -0.03, 0);
        }
        else if (in) {
            target_pose = Eigen::Vector3d(0.03, 0, 0);
        }
        else if (out) {
            target_pose = Eigen::Vector3d(-0.03, 0, 0);
        }

        else {}


        //Move The Snake
        calc_next_pos();//find current vT values
        igl::dqs(V, W, vQ, vT, U);
        data_list.at(0).set_vertices(U);
        //update skelton
        for (int i = 0; i < skelton.size(); i++)
        {
            skelton[i] = vT[i];
        }
        counter++;
        if (counter == 50) {
            counter = 0;
            data_list[0].tree.init(data_list[0].V, data_list[0].F);
            igl::AABB<Eigen::MatrixXd, 3> tree_first = data_list[0].tree;
            Eigen::AlignedBox<double, 3> box_first = tree_first.m_box;
            checkCollision();

        }

        
        //bonus bouncy targets object
        //generate_target(level);
        //move_targets(level);
        levelk(); // need to prevent check collision to earn more points than target points
        //end bonus bouncy targets object

        //end project comment  
	}
}