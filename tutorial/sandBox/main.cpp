#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include "sandBox.h"

#define VIEWPORT_WIDTH 3840
#define VIEWPORT_HEIGHT 2160

int main(int argc, char* argv[])
{
	Display* disp = new Display(VIEWPORT_WIDTH, VIEWPORT_HEIGHT, "Animation3D - Final Project");
	Renderer renderer;
	SandBox viewer;
	igl::opengl::glfw::imgui::ImGuiMenu menu;

	viewer.Init("configuration.txt");
	viewer.data_list[0].tree.init(viewer.data_list[0].V, viewer.data_list[0].F);
	igl::AABB<Eigen::MatrixXd, 3> tree_first = viewer.data_list[0].tree;
	Eigen::AlignedBox<double, 3> box_first = tree_first.m_box;
	Init(*disp, &menu);
	renderer.init(&viewer, 2, &menu);

	disp->SetRenderer(&renderer);
	menu.init_callback(viewer);
	disp->launch_rendering(true);

	delete disp;
}