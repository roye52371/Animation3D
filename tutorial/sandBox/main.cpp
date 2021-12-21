
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include "sandBox.h"

static void drawDotsAndLines(igl::opengl::glfw::Viewer& viewer) {

	for (int i = 1; i <= 4; i++) {

		int savedIndx = viewer.selected_data_index;
		//viewer.load_mesh_from_file("C:/Users/97254/Desktop/run_animation2/Animation3D/tutorial/data/zcylinder.obj");
		viewer.load_mesh_from_file("C:/Users/roi52/Desktop/ThreeDAnimationCourse/EngineForAnimationCourse/tutorial/data/zcylinder.obj");
		if (viewer.data_list.size() > viewer.parents.size())
		{
			viewer.parents.push_back(-1);
			viewer.data_list.back().set_visible(false, 1);
			viewer.data_list.back().set_visible(true, 2);
			//viewer.data_list.back().show_faces = 3;
			viewer.selected_data_index = savedIndx;
			//Ass3
			if(i!=1) // below does not needed because we don't want to connect sphere to zcylinders
			//(we dont want sphere to be father of zcylinder in index 1)
			{ 
				int last_index = viewer.data_list.size() - 1;
				viewer.parents[last_index] = last_index - 1;
			 }
			//end Ass3
		}
	}
}



int main(int argc, char *argv[])
{
  Display *disp = new Display(1200, 800, "Wellcome");
  Renderer renderer;

  SandBox viewer;
  //std::cout << "aaaaaaaaa\n" ;
  igl::opengl::glfw::imgui::ImGuiMenu* menu = new igl::opengl::glfw::imgui::ImGuiMenu();
  viewer.Init("configuration.txt");
  //std::cout << "bbbbbbbbbb\n";
  //Ass 2 comment
  //viewer.isActive = false;//make it false at the begining, so we cam control when to start the collision simulation
  //viewer.initTreesAndDrawForCollision();
  //end Ass 2 comment

  //Ass3 

  for (int i = 0; i < viewer.data_list.size(); i++) {
	  viewer.data_list[i].tree.init(viewer.data_list[i].V, viewer.data_list[i].F);
  }
  //std::cout << "cccccccccccc\n";
  //viewer.MyTranslate(Eigen::Vector3d(0, 0, -8),true);
  //viewer.data_list[0].MyTranslate(Eigen::Vector3d(5, 0, 0),true);
  //viewer.data_list[0].show_lines = false;

  drawDotsAndLines(viewer);
  //std::cout << "ddddddddd\n";
  //end comment Ass3
  
  Init(*disp, menu);
  renderer.init(&viewer,2,menu);
  //std::cout << "tttttttttttttttt\n";
  disp->SetRenderer(&renderer);
  //std::cout << "1wwwwwwwwww\n";

  disp->launch_rendering(true);
  //std::cout << "2wwwwwwwwww\n";
  delete menu;
  delete disp;
}
