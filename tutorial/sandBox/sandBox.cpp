#include "tutorial/sandBox/sandBox.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include "Eigen/dense"
#include <functional>



SandBox::SandBox()
{
	

}

void SandBox::Init(const std::string &config)
{
	//Ass 3
	load_mesh_from_file("C:/Users/97254/Desktop/run_animation2/Animation3D/tutorial/data/sphere.obj");

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
	//Ass 3
}

SandBox::~SandBox()
{

}

void SandBox::Animate()
{
	if (isActive)
	{
		//Ass 2 comment
		// using isActive as the key for knowing if we can move object[0] or not, in Assignment 2-Collision
		//my translate with pretranslate true
		//data_list[0].MyTranslate(moveDir,true);
		data_list[moving_index].MyTranslateInSystem(GetRotation(), data_list[moving_index].moveDir);
		checkCollision();
		//Ass2 comment end	
	}
}


