
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include "sandBox.h"
#include <imgui/imgui.cpp>
#include <igl/get_seconds.h>
#include <external/glfw/include/GLFW/glfw3.h>
//project comment
#include <Windows.h>
#include <MMSystem.h>
#pragma comment(lib, "winmm.lib")
//end comment project
using namespace std; 


static void drawDotsAndLines(igl::opengl::glfw::Viewer& viewer) {

	for (int i = 1; i <= 4; i++) {

		int savedIndx = viewer.selected_data_index;
		viewer.load_mesh_from_file("C:/Users/97254/Desktop/run_animation2/Animation3D/tutorial/data/zcylinder.obj");
		//viewer.load_mesh_from_file("C:/Users/roi52/Desktop/ThreeDAnimationCourse/EngineForAnimationCourse/tutorial/data/zcylinder.obj");
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

//project

static bool all_button_actions(const char* id, SandBox& viewer) {

	static bool isButtuned = false;//checking if any buttoned is pressed
	bool window_appirance = true;

	if (isButtuned == true) {
		viewer.isActive = true;
		window_appirance = false;
		isButtuned = false;
	}
	else if (viewer.isNextLevel) { 
		if (ImGui::Button("             START OVER             ")) {
			window_appirance = true;
			isButtuned = true;
			viewer.isNextLevel = false;
			viewer.isActive = true;
			viewer.score = 0;
		}
		if (ImGui::Button("             NEXT LEVEL             ")) {
			window_appirance = true;
			isButtuned = true;
			viewer.isNextLevel = false;
			viewer.isActive = true;
			viewer.score = 0;
			viewer.level += 1;
		}
	}
	else if (viewer.isResume){
		if (ImGui::Button("              RESUME              ")) {
			window_appirance = true;
			isButtuned = true;
			viewer.isActive = true;//it ruined the movment
			viewer.isResume = false;
			viewer.isGameStarted = true;
			viewer.isNextLevel = false;
		}
	}
	else
	{
		if (ImGui::Button("             Let's Start             "))
			isButtuned = true;
	}
	return window_appirance;
}



//end project
int main(int argc, char *argv[])
{
  

  Display *disp = new Display(1600, 960, "Final Game by R&D");
  Renderer renderer;
  SandBox viewer;
  igl::opengl::glfw::imgui::ImGuiMenu menu;
  viewer.Init("configuration.txt");

  viewer.data_list[0].tree.init(viewer.data_list[0].V, viewer.data_list[0].F);
  igl::AABB<Eigen::MatrixXd, 3> tree_first = viewer.data_list[0].tree;
  Eigen::AlignedBox<double, 3> box_first = tree_first.m_box;
  disp->SetRenderer(&renderer);
  /*Menu Display:
    1- start game menu
	2- next level menu/staying in current lvl
	3- resume game menu- by using ' ' key
	4- while the game is playing menu with score and lvl*/
  menu.callback_draw_custom_window = [&]()
  {
	  ImGui::CreateContext();
	  // window position + size
	  ImGui::SetNextWindowPos(ImVec2(0.f * menu.menu_scaling(), 0), ImGuiCond_Always);
	  ImGui::SetNextWindowSize(ImVec2(400, 1000), ImGuiCond_Always);
	  static bool showWindow = true;

	  if (showWindow && viewer.level == 1) {
		  viewer.isGameStarted = false;
		  if (!ImGui::Begin(
			  "Start Playin'", &showWindow,
			  ImGuiWindowFlags_NoSavedSettings
		  )) {
			  ImGui::End();
		  }
		  else {
			  ImGui::SetWindowFontScale(1.5f);
			  ImGui::PushItemWidth(-100); 
			  ImGui::Text("\n\n\n\n");
			  ImGui::Text("               Score: %d", viewer.score);
			  ImGui::Text("               Level: %d", viewer.level);
			  ImGui::Text("               ");
			  ImGui::PopItemWidth();
			  showWindow = all_button_actions("Start Playin'", viewer);
			  ImGui::End();
		  }
	  }
	  else if(viewer.isNextLevel)
	  {
		  viewer.isGameStarted = false;
		  if (!ImGui::Begin(
			  "Next Level", &showWindow,
			  ImGuiWindowFlags_NoSavedSettings
		  )) {
			  ImGui::End();
		  }
		  else {
			  ImGui::SetWindowFontScale(1.5f);

			  ImGui::PushItemWidth(-100);
			  ImGui::Text("Press NEXT LVL or START OVER\n\n");
			  ImGui::Text("               Score: %d", viewer.score);
			  ImGui::Text("               Level: %d", viewer.level);
			  ImGui::PopItemWidth();

			  showWindow = all_button_actions("NEXT LVL", viewer);
			  ImGui::End();
		  }
	  }
	  else if (viewer.isResume) {
		  viewer.isGameStarted = false;
		  if (!ImGui::Begin(
			  "Resume When Ready To Play", &showWindow,
			  ImGuiWindowFlags_NoSavedSettings
		  )) {
			  ImGui::End();
		  }
		  else {
			  ImGui::SetWindowFontScale(1.5f);
			  // Expose the same variable directly ...
			  ImGui::PushItemWidth(-100);
			  ImGui::Text("\n\n\n\n");
			  ImGui::Text("               Score: %d", viewer.score);
			  ImGui::Text("               Level: %d", viewer.level);
			  ImGui::Text("");
			  ImGui::PopItemWidth();


			  showWindow = all_button_actions("RESUME", viewer);
			  ImGui::End();
		  }
	  }
	  else {
		  if (!ImGui::Begin(
			  "Give Your Best Shot", &showWindow,
			  ImGuiWindowFlags_NoSavedSettings
		  )) {
			  ImGui::End();
		  }
		  else {
			  
			  ImGui::SetWindowFontScale(1.5f);
			  // Expose the same variable directly ...
			  viewer.isGameStarted = true;
			  ImGui::PushItemWidth(-100);
			  ImGui::Text("\n\n\n\n");
			  ImGui::Text("               Score: %d", viewer.score);
			  ImGui::Text("               Level: %d", viewer.level);

			  ImGui::PopItemWidth();
			  ImGui::End();
		  }
	  }
  };
  
  Init(*disp, &menu);
  renderer.init(&viewer, 3, &menu);
  renderer.selected_core_index = 1;

  disp->launch_rendering(true);
  
  //delete menu;
  delete disp;
}
