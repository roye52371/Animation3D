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

//project comment
#include <Windows.h>
#include <MMSystem.h>
#pragma comment(lib, "winmm.lib")
//end comment project


SandBox::SandBox()
{


}

void SandBox::Init(const std::string& config)
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
    snake_skeleton.resize(joints_num + 1);
    scale = 1;
    //Initialize vT, vQ
    vT.resize(joints_num + 1);
    vQ.resize(joints_num + 1);
    //snake_links.resize(16); //we have 16 links and 17 dots
    origin_snake_skeleton.resize(joints_num + 1);
    origin_vT.resize(joints_num + 1);
    origin_vQ.resize(joints_num + 1);
    snakejointBoxvec.resize(joints_num);//we need 16 box and not 17 cause we have 17 points


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

            //data_list[0].MyTranslate(Eigen::Vector3d(-3, -1, 0), true);
            parents.push_back(-1);
            data().show_overlay_depth = false;
            data().point_size = 10;
            data().line_width = 2;
            data().set_visible(false, 1);

            if (selected_data_index == 0)
                V = data().V;

            //data().set_colors(Eigen::RowVector3d(1, 0.55, 0));
            //data().image_texture("C:/Users/97254/Desktop/run_animation2/Animation3D/tutorial/textures/snake1.png");
            data().image_texture("C:/Users/roi52/Desktop/ThreeDAnimationCourse/EngineForAnimationCourse/tutorial/textures/snake1.png");
            //data().image_texture("C:/Users/97254/Desktop/run_animation2/Animation3D/tutorial/textures/snake.jpg");
            //data().image_texture("C:/Users/roi52/Desktop/ThreeDAnimationCourse/EngineForAnimationCourse/tutorial/textures/snake.jpg");
        }
        nameFileout.close();
    }
    MyTranslate(Eigen::Vector3d(0, 0, -1), true);

    //Find points for skelton
    double z = snake_tail_first_pos * scale;
    snake_link_len = snake_length / joints_num;
    for (int i = 0; i < snake_skeleton.size(); i++) {
        snake_skeleton.at(i) = Eigen::Vector3d(0, 0, z);
        z += snake_link_len * scale;
    }


    //Calaulate the weights for each vertex
    calc_all_weights();
    //add_weights();

    data().MyRotate(Eigen::Vector3d(0, 1, 0), M_PI / 2);//rotating the snake to horizontal poistion


    //snake_links.emplace_back();
    //snake_links.at(0).MyTranslate(snake_skeleton.at(0), true);
    //Joints.at(0).SetCenterOfRotation(Eigen::Vector3d(0, 0, -0.8));
    //parentsJoints[0] = -1;
    //the 16 other joint that have parents
    printf("before changing snake links\n");
    for (int i = 0; i < joints_num; i++)
    {
        //parentsJoints[i + 1] = i;
        snake_links.emplace_back();

        //snake_links.at(i).MyTranslate(Eigen::Vector3d(-3, -1, 0), true);

        Eigen::Vector3d currect_snake_skeleton = Eigen::Vector3d(snake_skeleton.at(i)(2), snake_skeleton.at(i)(1), snake_skeleton.at(i)(0)); //snake_skeleton.at(i);// Eigen::Vector3d(snake_skeleton.at(i)(2), snake_skeleton.at(i)(1), snake_skeleton.at(i)(0));
        snake_links.at(i).MyTranslate(currect_snake_skeleton, true);
        //snake_links.at(i).MyRotate(Eigen::Vector3d(0,0, 1), 3.14 / 2);
        //Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(currect_snake_skeleton, Eigen::Vector3d(-3, -1, 0));//currect_snake_skeleton is new translate and Eigen::Vector3d(-3, -1, 0) still hold the old translate 
        //snake_links.at(i).MyRotate(quat);

        //snake_links.at(i).MyRotate(Eigen::Vector3d(0, 1, 0), 3.14 / 2);//rotating the snake to horizontal poistion

        //snake_links.at(i).SetCenterOfRotation(Eigen::Vector3d(0, 0, -0.8));// check if needed
        //std::cout << parents[i + 1] <<"\n";
    }
    printf("after changing snake links\n");

    target_pose = snake_skeleton[joints_num];
    U = V;

    //keep original values of the snake, original vertices kept in OV variable
    for (int i = 0; i < snake_skeleton.size(); i++) {
        origin_snake_skeleton.at(i) = snake_skeleton.at(i);
        origin_vT.at(i) = vT.at(i);
        origin_vQ.at(i) = vQ.at(i);
    }

    initBoundingBoxofSnakeJoints();
    printf("got to the end of init in sandBox\n");
    //end comment Project  
}

SandBox::~SandBox()
{

}


Eigen::VectorXd SandBox::create_weight_vec(double w1, double w1_ind, double w2, double w2_ind)
{
    Eigen::VectorXd Wi;
    Wi.resize(17);
    double weight1 = w1;
    double index1 = w1_ind;
    double weight2 = w2;
    double index2 = w2_ind;

    for (double i = 0; i < 17; i++) {
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

    for (int i = 0; i < verticeSize; i++) {
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
    vT[0] = snake_skeleton[0];
    for (int i = 0; i < joints_num; i++) {
        vT[i + 1] = snake_skeleton[i + 1];
        vT[i] = vT[i] + ((vT[i + 1] - vT[i]) / DiversityFactor_forVtCalc);
    }
    vT[joints_num] = vT[joints_num] + target_pose;
}

void SandBox::add_weights() {
    //calc from article "automatic skinning weight retargeting 2017"
    double distance;
    int numOfV = data_list.at(0).V.rows();
    Eigen::MatrixXd V = data_list.at(0).V;
    W.resize(numOfV, 17);

    for (int i = 0; i < data_list[0].V.rows(); i++) {
        double related_distance = calc_related_distance(i);// calc sum of sigma on  k=0 to n(jointd) (1/distance(i,k)^4)
        for (int j = 0; j < snake_skeleton.size(); j++) {
            distance = abs(snake_skeleton.at(j).z() - data_list[0].V.row(i).z());
            double temp = pow((1 / distance), 4);
            W(i, j) = temp / related_distance;
        }
        W.row(i).normalized();
    }
}

double SandBox::calc_related_distance(int i) {
    double sum = 0;
    double distance;
    for (int j = 0; j < snake_skeleton.size(); j++) {
        distance = abs(snake_skeleton.at(j).z() - data_list[0].V.row(i).z());
        if (distance <= 0.1)
            sum += pow((1 / distance), 4);
    }
    return sum;
}


//Project levels  functions
void SandBox::levelk()
{
    if (score >= targetScore * level) {
        score = 0;
        isNextLevel = true;
        isActive = false;
        isGameStarted = false;
        reset_game();
        PlaySound(TEXT("C:/Users/97254/Desktop/run_animation2/Animation3D/tutorial/sandBox/levelcomplete.wav"), NULL, SND_NODEFAULT | SND_ASYNC);
        //PlaySound(TEXT("C:/Users/roi52/Desktop/ThreeDAnimationCourse/EngineForAnimationCourse/tutorial/sandBox/levelcomplete.wav"), NULL, SND_NODEFAULT | SND_ASYNC);
    }
    else {
        target_generator(level);
        targets_movement(level);
    }
}
void SandBox::reset_game()
{
    for (int i = 1; i < data_list.size(); i++)
        data_list[i].clear();// clear all food

                             //try to reset snake
    data_list[0].set_vertices(data_list[0].OV);// OV keeping the first vertics we had to the snake

                                               //retrieve original values of the snake, original vertices kept in OV variable
    for (int i = 0; i < snake_skeleton.size(); i++) {
        snake_skeleton.at(i) = origin_snake_skeleton.at(i);
        vT.at(i) = origin_vT.at(i);
        vQ.at(i) = origin_vQ.at(i);
    }
    //reset moving direction
    right = true;
    left = false;
    up = false;
    down = false;
    in = false;
    out = false;
}
void SandBox::initBoundingBoxofSnakeJoints() {
    for (int i = 1; i < joints_num + 1; i++)
    {
        double eps = 0.4;
        Eigen::Vector3d pos = snake_skeleton[i - 1];
        Eigen::Vector3d m = pos + Eigen::Vector3d(-eps, -eps, -eps);
        Eigen::Vector3d M = pos + Eigen::Vector3d(eps, eps, eps);
        Eigen::AlignedBox<double, 3> boxforcurrJoint;
        boxforcurrJoint = Eigen::AlignedBox<double, 3>(m, M);
        snakejointBoxvec[i - 1] = boxforcurrJoint;
        //drawsnakejointBox(snakejointBoxvec[i - 1], 0);


    }
}

void SandBox::drawsnakejointBox(Eigen::AlignedBox<double, 3> box, int color) {
    /*point_size = 10;
    line_width = 2;*/
    Eigen::RowVector3d colorVec;
    if (color == 1) {
        colorVec = Eigen::RowVector3d(255, 255, 255);//white
    }
    else
        colorVec = Eigen::RowVector3d(0, 255, 0);//green
    //parameters in order to minimize run-time
    Eigen::RowVector3d BottomRightCeil = box.corner(box.BottomRightCeil);
    Eigen::RowVector3d BottomRightFloor = box.corner(box.BottomRightFloor);
    Eigen::RowVector3d BottomLeftCeil = box.corner(box.BottomLeftCeil);
    Eigen::RowVector3d BottomLeftFloor = box.corner(box.BottomLeftFloor);
    Eigen::RowVector3d TopRightCeil = box.corner(box.TopRightCeil);
    Eigen::RowVector3d TopRightFloor = box.corner(box.TopRightFloor);
    Eigen::RowVector3d TopLeftCeil = box.corner(box.TopLeftCeil);
    Eigen::RowVector3d TopLeftFloor = box.corner(box.TopLeftFloor);

    //add_edges(n1,n2,col)- draws edge from n1 to n2 in color col
    data_list[0].add_edges(BottomLeftCeil, BottomRightCeil, colorVec);
    data_list[0].add_edges(BottomLeftCeil, BottomLeftFloor, colorVec);
    data_list[0].add_edges(BottomRightCeil, BottomRightFloor, colorVec);
    data_list[0].add_edges(BottomLeftFloor, BottomRightFloor, colorVec);
    data_list[0].add_edges(TopLeftCeil, TopRightCeil, colorVec);
    data_list[0].add_edges(TopRightCeil, TopRightFloor, colorVec);
    data_list[0].add_edges(TopLeftCeil, TopLeftFloor, colorVec);
    data_list[0].add_edges(TopLeftFloor, TopRightFloor, colorVec);
    data_list[0].add_edges(TopLeftCeil, BottomLeftCeil, colorVec);
    data_list[0].add_edges(TopRightFloor, BottomRightFloor, colorVec);
    data_list[0].add_edges(TopRightCeil, BottomRightCeil, colorVec);
    data_list[0].add_edges(TopLeftFloor, BottomLeftFloor, colorVec);
}
//end comment Project

void SandBox::Animate()
{
    if (isActive && !isResume)
    {
        //Project comment
        if (left)
            target_pose = Eigen::Vector3d(0, 0, -snakeVelocity);
        else if (right)
            target_pose = Eigen::Vector3d(0, 0, snakeVelocity);
        else if (up)
            target_pose = Eigen::Vector3d(0, snakeVelocity, 0);
        else if (down)
            target_pose = Eigen::Vector3d(0, -snakeVelocity, 0);
        else if (in)
            target_pose = Eigen::Vector3d(snakeVelocity, 0, 0);
        else if (out)
            target_pose = Eigen::Vector3d(-snakeVelocity, 0, 0);
        else {}

        //Move The Snake
        calc_next_pos();//find current vT values
        igl::dqs(V, W, vQ, vT, U);
        data_list.at(0).set_vertices(U);
        /* printf("print vT[0]\n");
         cout << vT.at(0) << endl;*/
        for (int i = 0; i < snake_links.size(); i++)
        {
            //do translationns
            Eigen::Vector3d currect_vt = Eigen::Vector3d(vT.at(i)(2), vT.at(i)(1), vT.at(i)(0));//vT.at(i);// 
            Eigen::Vector3d currect_snake_skeleton = Eigen::Vector3d(snake_skeleton.at(i)(2), snake_skeleton.at(i)(1), snake_skeleton.at(i)(0));//snake_skeleton.at(i);// Eigen::Vector3d(snake_skeleton.at(i)(2), snake_skeleton.at(i)(1), snake_skeleton.at(i)(0));
            snake_links.at(i).MyTranslate(currect_vt - currect_snake_skeleton, true);
            Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(currect_vt, currect_snake_skeleton);//vT is new tranlate and snake_skeleton still hold the old translate 
            snake_links.at(i).MyRotate(quat);
            //std::cout << parents[i + 1] <<"\n";
        }
        //update skelton
        for (int i = 0; i < snake_skeleton.size(); i++)
            snake_skeleton[i] = vT[i];
        //counter++;
        //if (counter == 50) {
        //    counter = 0;
        //    creating_tree_and_box(0);//0- snake index
        //    checkCollision();
        //}
        //initBoundingBoxofSnakeJoints();
        //printf("Before collision\n");
        checkCollision();

        levelk();
        //end bonus bouncy targets object

        //end project comment  
    }
}