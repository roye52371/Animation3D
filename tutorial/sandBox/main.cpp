
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include "sandBox.h"

static void drawDotsAndLines(igl::opengl::glfw::Viewer& viewer) {
	Eigen::Matrix4f parents = Eigen::Matrix4f().Identity();
	//int size = 1.6;
	for (int i = 1; i <= 4; i++) {
		viewer.data_list[i].MyTranslate(Eigen::Vector3d(0,0, 1.6*i),true);
		viewer.data_list[i].SetCenterOfRotation(Eigen::Vector3d(0,0, 1.6*i - 0.8));

		viewer.data_list[i].show_overlay_depth = false;
		viewer.data_list[i].show_lines = false;
		//if (i != 4) {
			viewer.data_list[i].add_points(Eigen::RowVector3d(0,0, 0.8), Eigen::RowVector3d(0, 0, 255));
			viewer.data_list[i].add_edges(Eigen::RowVector3d(0, -1.6, 0), Eigen::RowVector3d(0, 1.6, 0), Eigen::RowVector3d(0, 255, 0));
			viewer.data_list[i].add_edges(Eigen::RowVector3d(-1.6, 0, 0), Eigen::RowVector3d(1.6, 0, 0), Eigen::RowVector3d(255, 0, 0));
			viewer.data_list[i].add_edges(Eigen::RowVector3d(0, 0, -1.6), Eigen::RowVector3d(0, 0, 1.6), Eigen::RowVector3d(0, 0, 255));
			viewer.data_list[i].point_size = 10;
			viewer.data_list[i].line_width = 3;
		//}
	}
}



int main(int argc, char *argv[])
{
  Display *disp = new Display(1200, 800, "Wellcome");
  Renderer renderer;

  SandBox viewer;
  
  igl::opengl::glfw::imgui::ImGuiMenu* menu = new igl::opengl::glfw::imgui::ImGuiMenu();
  viewer.Init("configuration.txt");

  //Ass 2 comment
  //viewer.isActive = false;//make it false at the begining, so we cam control when to start the collision simulation
  //viewer.initTreesAndDrawForCollision();
  //end Ass 2 comment

  //Ass3 comment
  viewer.MyTranslate(Eigen::Vector3d(0, 0, -8),true);
  viewer.data_list[0].MyTranslate(Eigen::Vector3d(5, 0, 0),true);
  viewer.data_list[0].show_lines = false;

  drawDotsAndLines(viewer);

  //end comment Ass3
  
  Init(*disp, menu);
  renderer.init(&viewer,2,menu);
  
  disp->SetRenderer(&renderer);
  disp->launch_rendering(true);
  delete menu;
  delete disp;
}
