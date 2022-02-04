#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "igl/aabb.h"
#include "igl/opengl/Joint.h"

using namespace std;

class SandBox : public igl::opengl::glfw::Viewer
{
public:
	SandBox();
	~SandBox();
	void Init(const std::string& config);
	double doubleVariable;
	//Project comment
	void initDataStructure(Eigen::MatrixXd&, Eigen::MatrixXi&);
	//void setVelocity(Eigen::Vector3d dir);


	const int snake_head = 0;
	const int num_of_joints = 17;
	const int num_of_links = 16;
	std::vector<igl::opengl::Joint*> joints;
	vector<Eigen::Vector3d> JointsPoses;

	void setJointsPositions();
	void setHeadPosition();
	//end comment Project

private:
	//Project comment
	// Prepare array-based edge data structures and priority queue
	int link_Len = 1.6;
	Eigen::MatrixXd W;
	typedef std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>RotationList;
	Eigen::RowVector3d sea_green = Eigen::RowVector3d((70.0 / 255), 252.0 / 255., 167 / 255.);
	//maybe delete M and P (??)
	Eigen::MatrixXd C, U, M;
	Eigen::MatrixXi BE;
	Eigen::VectorXi P;
	std::vector<RotationList > poses; // rotations of joints for animation
	double anim_t = 0.0;
	double anim_t_dir = 0.015;
	bool use_dqs = false;
	bool recompute = true;
	void add_joints();
	void drawJoints();
	Eigen::Vector3d getJoint(int indx);
	Eigen::Matrix3d SandBox::CalcParentsRot(int indx);
	double calc_related_distance(int i);
	void add_weights();
	double sum(int i);
	void Skinning();
	void forwardLoop();
	//end Project comment
	
	
	void Animate();
};

