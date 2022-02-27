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


    
    for (int i = 0; i < joints_num; i++)
    {
       
        snake_links.emplace_back();

        Eigen::Vector3d currect_snake_skeleton = Eigen::Vector3d(snake_skeleton.at(i)(2), snake_skeleton.at(i)(1), snake_skeleton.at(i)(0)); //snake_skeleton.at(i);// Eigen::Vector3d(snake_skeleton.at(i)(2), snake_skeleton.at(i)(1), snake_skeleton.at(i)(0));
        snake_links.at(i).MyTranslate(currect_snake_skeleton, true);
        
    }

    target_pose = snake_skeleton[joints_num];
    U = V;

    //keep original values of the snake, original vertices kept in OV variable
    for (int i = 0; i < snake_skeleton.size(); i++) {
        origin_snake_skeleton.at(i) = snake_skeleton.at(i);
        origin_vT.at(i) = vT.at(i);
        origin_vQ.at(i) = vQ.at(i);
    }

    //initialize box for every movable link of the snake
    initBoundingBoxofSnakeJoints();

    //end comment Project  
}

SandBox::~SandBox()
{

}

//calc weights for snake
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
//end calc weights for snake

//calc snake's vT next positions
void  SandBox::calc_next_pos()
{
    vT[0] = snake_skeleton[0];
    for (int i = 0; i < joints_num; i++) {
        vT[i + 1] = snake_skeleton[i + 1];
        vT[i] = vT[i] + ((vT[i + 1] - vT[i]) / DiversityFactor_forVtCalc);
    }
    vT[joints_num] = vT[joints_num] + target_pose;
}

//calc weights for snake from article in moodle
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
//end calc weights for snake from article in moodle

//Project check levels  functions
void SandBox::check_levelk()
{
    if (score >= targetScore * level) {
        old_level = level;
        level++;
        isNextLevel = true;
        isActive = false;
        score = 0;
        timer = 0;

        creation_gap = 3;
       

        reset_game();
        //PlaySound(TEXT("C:/Users/97254/Desktop/run_animation2/Animation3D/tutorial/sandBox/levelcomplete.wav"), NULL, SND_NODEFAULT | SND_ASYNC);
        PlaySound(TEXT("C:/Users/roi52/Desktop/ThreeDAnimationCourse/EngineForAnimationCourse/tutorial/sandBox/levelcomplete.wav"), NULL, SND_NODEFAULT | SND_ASYNC);
    }
    
}

//snake links init boundog box per link
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
       
    }
}


//calc snake movement and skining
void SandBox::SnakeMovementAndSkining() {
    //Project comment
    //double snake_speed = 0.15;

    if (!isGameStarted || isPaused || loose) {
        target_pose = Eigen::Vector3d::Zero();
        return;
    }
    if (left)
        target_pose = Eigen::Vector3d(0, 0, -snakeVelocity);
    else if (right)
        target_pose = Eigen::Vector3d(0, 0, snakeVelocity);
    else if (up)
        target_pose = Eigen::Vector3d(0, snakeVelocity, 0);
    else if (down)
        target_pose = Eigen::Vector3d(0, -snakeVelocity, 0);
    /*else if (in)
        target_pose = Eigen::Vector3d(snakeVelocity, 0, 0);
    else if (out)
        target_pose = Eigen::Vector3d(-snakeVelocity, 0, 0);*/
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
        
    }
    //update skelton
    for (int i = 0; i < snake_skeleton.size(); i++)
        snake_skeleton[i] = vT[i];
}
//end comment Project


void SandBox::Animate()
{
    if (isActive && !isPaused)
    {
        //project comment
        SnakeMovementAndSkining();
        checkCollision();
        Manger_Targets_Creators();
        Manage_Targets_Moves();
        CleanTargets_After_DefinedTime();//clean sometimes part of objects to prenet to many obj rendering on screen
        check_levelk();
        

        //end project comment  
    }
}