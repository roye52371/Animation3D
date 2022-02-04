#pragma once
#include "igl/opengl/glfw/Viewer.h"
namespace igl
{
	namespace opengl
	{
		class Joint : public Movable
		{
		public:
			Joint(Eigen::Vector3d pos, int _id, int _parent);
			virtual ~Joint();
			int id;
			Eigen::Vector3d position;
			int parent;
			Eigen::MatrixXd V;

		};
	}
}