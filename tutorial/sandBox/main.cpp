
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include "sandBox.h"
#include <imgui/imgui.cpp>
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

static bool toggleButton(const char* id, SandBox& viewer) {
	static float b = 5.0f; //  test whatever color you need from imgui_demo.cpp e.g.
	static float c = 5.0f; // 
	static int i = 3;
	static bool enable_7m = false;  // default value, the button is disabled 


	bool showWindow = true;
	if (enable_7m == true)
	{
		viewer.isActive = true;
		ImGui::PushID(id);
		ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(245, 60, 40));
		ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor::HSV(245, 60, 40));
		ImGui::PushStyleColor(ImGuiCol_ButtonActive, (ImVec4)ImColor::HSV(245, 70, 60));
		ImGui::Button(id);
		ImGui::PopStyleColor(3);
		ImGui::PopID();
		showWindow = false;
		enable_7m = false;
	}
	else if (viewer.isNextLevel) {//ImGui::Button("NEXT LVL") || 
		if (ImGui::Button("START OVER")) {
			showWindow = true;
			enable_7m = true;
			viewer.isNextLevel = false;
			viewer.score = 0;
		}
		if (ImGui::Button("NEXT LVL")) {
			showWindow = true;
			enable_7m = true;
			viewer.isNextLevel = false;
			viewer.score = 0;
			viewer.level += 1;
		}
	}
	else
	{
		if (ImGui::Button("Let's Start"))
			enable_7m = true;
	}
	
	return showWindow;
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
  viewer.data_list[0].drawBox(box_first, 0);
  disp->SetRenderer(&renderer);

 /* menu.callback_draw_viewer_window = [&]() {
	  ImGui::CreateContext();
	  // Define next window position + size
	  ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), ImGuiCond_FirstUseEver);
	  ImGui::SetNextWindowSize(ImVec2(400, 320), ImGuiCond_FirstUseEver);
	  static bool showWindow = true;
	  if (showWindow) {
		  if (!ImGui::Begin(
			  "You Lost", &showWindow,
			  ImGuiWindowFlags_NoSavedSettings
		  )) {
			  ImGui::End();
		  }
		  else {
			  // Expose the same variable directly ...
			  ImGui::PushItemWidth(-80);
			  ImGui::Text("Your Score is: %d", viewer.score);
			  ImGui::Text("Level Number: %d", viewer.level);
			  ImGui::PopItemWidth();
			  showWindow = toggleButton("Let's Play Again");
			  viewer.score = 0;
			  viewer.level = 1;
			  viewer.start = true;

			  //ImGuiWindow* window = ImGui::FindWindowByName("Let's Play");

			  ImGui::End();
		  }
	  }

  };*/

  menu.callback_draw_custom_window = [&]()
  {
	  ImGui::CreateContext();
	  // Define next window position + size
	  ImGui::SetNextWindowPos(ImVec2(30.f * menu.menu_scaling(), 200), ImGuiCond_FirstUseEver);
	  ImGui::SetNextWindowSize(ImVec2(300, 260), ImGuiCond_FirstUseEver);
	  static bool showWindow = true;
	  if (showWindow) {
		  if (!ImGui::Begin(
			  "Start Playin'", &showWindow,
			  ImGuiWindowFlags_NoSavedSettings
		  )) {
			  ImGui::End();
		  }
		  else {
			  // Expose the same variable directly ...
			 
			  ImGui::PushItemWidth(-80);
			  ImGui::Text("               Score: %d", viewer.score);
			  ImGui::Text("               Level: %d", viewer.level);
			  ImGui::PopItemWidth();


			  showWindow = toggleButton("Start Playin'", viewer);
			  //ImGuiWindow* window = ImGui::FindWindowByName("Let's Play");

			  ImGui::End();
		  }
	  }
	  else if(viewer.isNextLevel)
	  {
		  if (!ImGui::Begin(
			  "Next Level", &showWindow,
			  ImGuiWindowFlags_NoSavedSettings
		  )) {
			  ImGui::End();
		  }
		  else {
			  // Expose the same variable directly ...

			  ImGui::PushItemWidth(-80);
			  ImGui::Text("Press NEXT LVL or START OVER");
			  ImGui::Text("               Score: %d", viewer.score);
			  ImGui::Text("               Level: %d", viewer.level);
			  ImGui::PopItemWidth();

			  showWindow = toggleButton("NEXT LVL", viewer);
			  showWindow = toggleButton("START OVER", viewer);
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
			  // Expose the same variable directly ...

			  ImGui::PushItemWidth(-80);
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
  
  //std::cout << "2wwwwwwwwww\n";
  //delete menu;
  delete disp;
}
