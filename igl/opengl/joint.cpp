#include "joint.h"

using namespace Eigen;
namespace igl
{
	namespace opengl
	{
		Joint::Joint(Vector3d pos, int _id, int _parent) {
				position = pos;
				id = _id;
				parent = _parent;
				//V = pos;
			}
		Joint::~Joint(){}
			
	}
}

