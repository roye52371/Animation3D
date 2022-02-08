#pragma once
#include "igl/opengl/glfw/Display.h"
#include "igl/opengl/glfw/Renderer.h"
#include "sandBox.h"
//#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
//#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
//#include <../imgui/imgui.h>



static void glfw_mouse_press(GLFWwindow* window, int button, int action, int modifier)
{

  /*Renderer* rndr = (Renderer*) glfwGetWindowUserPointer(window);
  igl::opengl::glfw::Viewer* scn = rndr->GetScene();

  if (action == GLFW_PRESS)
  {
	  double x2, y2;
	  glfwGetCursorPos(window, &x2, &y2);
	 

	  double depth, closestZ = 1;
	  int i = 0, savedIndx = scn->selected_data_index, lastIndx= scn->selected_data_index;
	  int prev_picked = scn->current_picked;

	  for (; i < scn->data_list.size(); i++)
	  {
		  scn->selected_data_index = i;
		  depth = rndr->Picking(x2, y2);
		  if (depth < 0 && (closestZ > 0 || closestZ < depth))
		  {
			  scn->current_picked = i;
			  savedIndx = i;
			  closestZ = depth;
			  std::cout << "found " << depth << std::endl;
		  }
	  }
	  scn->selected_data_index = savedIndx;
	  scn->data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));
	  if (lastIndx != savedIndx)
		  scn->data_list[lastIndx].set_colors(Eigen::RowVector3d(255.0 / 255.0, 228.0 / 255.0, 58.0 / 255.0));

	  if (scn->current_picked == prev_picked) {
		  scn->current_picked = -1;
	  }
	  rndr->UpdatePosition(x2, y2);

  }
  else
  {
	  rndr->GetScene()->isPicked = false;

  }*/
	Renderer* rndr = (Renderer*)glfwGetWindowUserPointer(window);
	igl::opengl::glfw::Viewer* scn = rndr->GetScene();

	if (action == GLFW_PRESS)
	{
		double x2, y2;
		glfwGetCursorPos(window, &x2, &y2);


		double depth, closestZ = 1;
		int i = 0, savedIndx = scn->selected_data_index, lastIndx = scn->selected_data_index;

		for (; i < scn->data_list.size(); i++)
		{
			scn->selected_data_index = i;
			depth = rndr->Picking(x2, y2);
			if (depth < 0 && (closestZ > 0 || closestZ < depth))
			{
				savedIndx = i;
				closestZ = depth;
				std::cout << "found " << depth << std::endl;
			}
		}
		scn->selected_data_index = savedIndx;
		scn->data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));
		if (lastIndx != savedIndx)
			scn->data_list[lastIndx].set_colors(Eigen::RowVector3d(255.0 / 255.0, 228.0 / 255.0, 58.0 / 255.0));

		rndr->UpdatePosition(x2, y2);

	}
	else
	{
		rndr->GetScene()->isPicked = false;

	}
}


//static void glfw_char_mods_callback(GLFWwindow* window, unsigned int codepoint, int modifier)
//{
//  __viewer->key_pressed(codepoint, modifier);
//}

 void glfw_mouse_move(GLFWwindow* window, double x, double y)
{
	 Renderer* rndr = (Renderer*)glfwGetWindowUserPointer(window);
	 rndr->UpdatePosition(x, y);
	 if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS)
	 {
		 rndr->MouseProcessing(GLFW_MOUSE_BUTTON_RIGHT);
	 }
	 else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
	 {
		 rndr->MouseProcessing(GLFW_MOUSE_BUTTON_LEFT);
	 }
}

static void glfw_mouse_scroll(GLFWwindow* window, double x, double y)
{
	Renderer* rndr = (Renderer*)glfwGetWindowUserPointer(window);
	//if (rndr->GetScene()->current_picked != -1) {
	//	
	//	//Ass3
	//	if (rndr->GetScene()->current_picked != 0)//not sphere
	//		rndr->GetScene()->data(1).MyTranslateInSystem(rndr->GetScene()->GetRotation(), Eigen::Vector3d(0, 0, -y * 0.03));//arm must not brake
	//	else
	//		rndr->GetScene()->data().MyTranslateInSystem(rndr->GetScene()->GetRotation(), Eigen::Vector3d(0, 0, -y * 0.03));//move sphere

	//	//rndr->GetScene()->data().MyScale(Eigen::Vector3d(1 + y * 0.01,1 + y * 0.01,1+y*0.01));
	//	//end Ass3
	//}
	if (rndr->IsPicked())
		rndr->GetScene()->data().MyScale(Eigen::Vector3d(1 + y * 0.01, 1 + y * 0.01, 1 + y * 0.01));
	else
		rndr->GetScene()->MyTranslate(Eigen::Vector3d(0, 0, -y * 0.03), true);
}

void glfw_window_size(GLFWwindow* window, int width, int height)
{
	Renderer* rndr = (Renderer*)glfwGetWindowUserPointer(window);
	//igl::opengl::glfw::Viewer* scn = rndr->GetScene();

    rndr->post_resize(window,width, height);

}

//static void glfw_drop_callback(GLFWwindow *window,int count,const char **filenames)
//{
//
//}

//static void glfw_error_callback(int error, const char* description)
//{
//	fputs(description, stderr);
//}

static void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int modifier)
{
	Renderer* rndr = (Renderer*) glfwGetWindowUserPointer(window);
	SandBox* scn = (SandBox*)rndr->GetScene();
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);

	else if(action == GLFW_PRESS || action == GLFW_REPEAT)
		switch (key)
		{
		case 'A':
		case 'a':
		{
			rndr->core().is_animating = !rndr->core().is_animating;
			break;
		}
		case 'F':
		case 'f':
		{
			scn->data().set_face_based(!scn->data().face_based);
			break;
		}
		case 'I':
		case 'i':
		{
			scn->data().dirty |= igl::opengl::MeshGL::DIRTY_NORMAL;
			scn->data().invert_normals = !scn->data().invert_normals;
			break;
		}
		case 'L':
		case 'l':
		{
			rndr->core().toggle(scn->data().show_lines);
			break;
		}
		case 'O':
		case 'o':
		{
			rndr->core().orthographic = !rndr->core().orthographic;
			break;
		}
		case 'T':
		case 't':
		{
			//rndr->core().toggle(scn->data().show_faces);
			//Ass3
			//int lastLinkidx = scn->link_num;

			//for (int i = 1; i <= lastLinkidx; i++) {
			//	scn->tip_position = scn->ParentsTrans_mat4d(i) * scn->data(i).MakeTransd() * Eigen::Vector4d(0, 0, 0.8, 1);
			//		/*scn->CalcParentsTrans(i) *
			//		scn->data(i).MakeTransd() *
			//		Eigen::Vector4d(scn->data(i).V.colwise().mean()[0],
			//			scn->data(i).V.colwise().maxCoeff()[1], scn->data(i).V.colwise().mean()[2], 1);*/
			//	std::cout << "tip_" << i << ": ("<< scn->tip_position.transpose()[0] << "," << scn->tip_position.transpose()[1] << "," << scn->tip_position.transpose()[2] << ")" << std::endl;
			//}
			break;

		}
		case '[':
		case ']':
		{
			rndr->ChangeCamera(key);
			break;
		}
		case ';':
			scn->data().show_vertid = !scn->data().show_vertid;
			break;
		case ':':
			scn->data().show_faceid = !scn->data().show_faceid;
			break;
		case 'w':
		case 'W':
			//Ass2 comment
			//moving in
			//rndr->changeMovingDirection('w');
			//end comment Ass 2
			//rndr->TranslateCamera(Eigen::Vector3f(0, 0, 0.03f));
			
			//Project comment
			//W is in movment
			if (scn->up) {
				scn->up = false;
				scn->isActive = !scn->isActive;
			}
			if (scn->down) {
				scn->down = false;
				scn->isActive = !scn->isActive;
			}
			if (scn->left) {
				scn->left = false;
				scn->isActive = !scn->isActive;
			}
			if (scn->right) {
				scn->right = false;
				scn->isActive = !scn->isActive;
			}
			if (scn->out) {
				scn->out = false;
				scn->isActive = !scn->isActive;
			}

			if (!scn->in)
				scn->in = true;
			else
				scn->in = false;
			scn->isActive = !scn->isActive;
			// end Project comment
			break;
		case 's':
		case 'S':
			//Ass2 comment
			//moving out
			//rndr->changeMovingDirection('s');
			//end comment Ass 2
			//rndr->TranslateCamera(Eigen::Vector3f(0, 0, -0.03f));

			//Project comment
			//S is out movment
			if (scn->up) {
				scn->up = false;
				scn->isActive = !scn->isActive;
			}
			if (scn->down) {
				scn->down = false;
				scn->isActive = !scn->isActive;
			}
			if (scn->left) {
				scn->left = false;
				scn->isActive = !scn->isActive;
			}
			if (scn->right) {
				scn->right = false;
				scn->isActive = !scn->isActive;
			}
			if (scn->in) {
				scn->in = false;
				scn->isActive = !scn->isActive;
			}

			if (!scn->out)
				scn->out = true;
			else
				scn->out = false;
			scn->isActive = !scn->isActive;
			
			//end comment Project
			break;
		//case GLFW_KEY_UP:
		//	//Ass2 comment
		//	//moving up
		//	//rndr->changeMovingDirection(GLFW_KEY_UP);
		//	//end comment Ass 2
		//	//rndr->TranslateCamera(Eigen::Vector3f(0, 0.01f,0));
		//	//Ass3
		//	rndr->changeRotateAxis(GLFW_KEY_UP);
		//	//end Ass3
		//	break;
		//case GLFW_KEY_DOWN:
		//	//Ass2 comment
		//	//moving down
		//	//rndr->changeMovingDirection(GLFW_KEY_DOWN);
		//	//end comment Ass 2
		//	//rndr->TranslateCamera(Eigen::Vector3f(0, -0.01f,0));
		//	//Ass3
		//	rndr->changeRotateAxis(GLFW_KEY_DOWN);
		//	//end Ass3
		//	break;
		//case GLFW_KEY_LEFT:
		//	//Ass2 comment , return translate camera comment if not use this code, or use translate camera also
		//	//moving left
		//	//rndr->changeMovingDirection(GLFW_KEY_LEFT);
		//	//end comment Ass 2
		//	//rndr->TranslateCamera(Eigen::Vector3f(-0.01f, 0,0));
		//	//Ass3
		//	rndr->changeRotateAxis(GLFW_KEY_LEFT);
		//	//end Ass3
		//	break;
		//case GLFW_KEY_RIGHT:
		//	//Ass2 comment, return translate camera comment if not use this code, or use translate camera also
		//	//moving right
		//	//rndr->changeMovingDirection(GLFW_KEY_RIGHT);
		//	//end comment Ass 2, return translate camera comment if not use this code, or use translate camera also
		//	//rndr->TranslateCamera(Eigen::Vector3f(0.01f, 0, 0));
		//	//Ass3
		//	rndr->changeRotateAxis(GLFW_KEY_RIGHT);
		//	//end Ass3
		//	break;
		//Project comment
		case GLFW_KEY_UP:
			//rndr->TranslateCamera(Eigen::Vector3f(0, 0.01f,0));
			if (scn->right) {
				scn->right = false;
				scn->isActive = !scn->isActive;
			}
			if (scn->down) {
				scn->down = false;
				scn->isActive = !scn->isActive;
			}
			if (scn->left) {
				scn->left = false;
				scn->isActive = !scn->isActive;
			}
			if (scn->in) {
				scn->in = false;
				scn->isActive = !scn->isActive;
			}
			if (scn->out) {
				scn->out = false;
				scn->isActive = !scn->isActive;
			}

			if (!scn->up)
				scn->up = true;
			else
				scn->up = false;
			scn->isActive = !scn->isActive;

			break;
		case GLFW_KEY_DOWN:
			//rndr->TranslateCamera(Eigen::Vector3f(0, -0.01f,0));
			if (scn->up){
				scn->up = false;
				scn->isActive = !scn->isActive;
			}
			if (scn->left) {
				scn->left = false;
				scn->isActive = !scn->isActive;
			}
			if (scn->right) {
				scn->right = false;
				scn->isActive = !scn->isActive;
			}
			if (scn->in) {
				scn->in = false;
				scn->isActive = !scn->isActive;
			}
			if (scn->out) {
				scn->out = false;
				scn->isActive = !scn->isActive;
			}

			if (!scn->down)
				scn->down = true;
			else
				scn->down = false;
			scn->isActive = !scn->isActive;
			break;
		case GLFW_KEY_LEFT:
			//rndr->TranslateCamera(Eigen::Vector3f(-0.01f, 0,0));
			if (scn->up) {
				scn->up = false;
				scn->isActive = !scn->isActive;
			}
			if (scn->down) {
				scn->down = false;
				scn->isActive = !scn->isActive;
			}
			if (scn->right) {
				scn->right = false;
				scn->isActive = !scn->isActive;
			}
			if (scn->in) {
				scn->in = false;
				scn->isActive = !scn->isActive;
			}
			if (scn->out) {
				scn->out = false;
				scn->isActive = !scn->isActive;
			}
	
			if (!scn->left)
				scn->left = true;
			else
				scn->left = false;
			scn->isActive = !scn->isActive;
			break;
		case GLFW_KEY_RIGHT:
			//rndr->TranslateCamera(Eigen::Vector3f(0.01f, 0, 0));
			if (scn->up) {
				scn->up = false;
				scn->isActive = !scn->isActive;
			}
			if (scn->down) {
				scn->down = false;
				scn->isActive = !scn->isActive;
			}
			if (scn->left) {
				scn->left = false;
				scn->isActive = !scn->isActive;
			}
			if (scn->in) {
				scn->in = false;
				scn->isActive = !scn->isActive;
			}
			if (scn->out) {
				scn->out = false;
				scn->isActive = !scn->isActive;
			}
	
			if (!scn->right)
				scn->right = true;
			else
				scn->right = false;
			scn->isActive = !scn->isActive;
			break;
		//end cpmment Project

		case ' ':
			//Ass1 comment, return translate camera comment if not use this code, or use translate camera also
			//Simplification from ass1
			//scn->meshSimplification(0.05 * scn->data().Q->size());
			//end comment Ass1
			//Ass3
			// toggle ik solver aniimation
			//scn->ikAnimation = !scn->ikAnimation;
			//scn->toggleIKSimulation();
			scn->isActive = !scn->isActive;//it ruined the movment
			break;
			//end Ass3
			//break;
			//Ass 2 comment
			// case to change moving to  true(it is initialized to false, so we can also init to true and change the code down here)
			//and case when collison accured, if we want to move the object again, so we need to make is moving true again,(setMovingButton do it)
		case 'k':
		case 'K':
			// 'k' and 'K' activating or deactivating the object movemant
			//scn->setMovingButton();
			break;
			// end comment Ass 2
			//Ass 2 comment
			//changing moving object between 2 objects
		case 'j':
		case 'J':
			//scn->moving_index = (scn->moving_index + 1) % 2;
			break;
			// end comment Ass 2
		//Ass3
		case'P':
		case 'p': 
		//{
		//	Eigen::Matrix3d mat;
		//	if (scn->current_picked != -1 && scn->current_picked != 0) {// we don't need to print sphere in index 0
		//		int idx = scn->selected_data_index;
		//		Eigen::Matrix3d mat = scn->data_list[idx].GetRotation();
		//		std::cout << "rotation_" << idx << ":\n" << mat << std::endl;
		//	}
		//	else if (scn->current_picked == 0) {
		//		std::cout << "Sphere matrix not supposed to be printed" << std::endl;
		//	}
		//	else {
		//		Eigen::Matrix3d mat = scn->GetRotation();
		//		std::cout << "rotation_scn" << ":\n" << mat << std::endl;
		//	}

		//	break;
		//}
			break;
		case 'D':
		case 'd':
			/*scn->destination_position = Eigen::Vector3d(scn->data_list[0].MakeTransd().col(3)[0], scn->data_list[0].MakeTransd().col(3)[1], scn->data_list[0].MakeTransd().col(3)[2]);
			std::cout << "destination: (" << scn->destination_position << ")" << std::endl;*/
			break;

		case 'H':
		case 'h':
		{
			rndr->core().toggle(scn->data().show_overlay);
			break;
		}
		//end Ass3
		default: 
			Eigen::Vector3f shift;
			float scale;
			rndr->core().get_scale_and_shift_to_fit_mesh(scn->data().V, scn->data().F, scale, shift);
			
			std::cout << "near " << rndr->core().camera_dnear << std::endl;
			std::cout << "far " << rndr->core().camera_dfar << std::endl;
			std::cout << "angle " << rndr->core().camera_view_angle << std::endl;
			std::cout << "base_zoom " << rndr->core().camera_base_zoom << std::endl;
			std::cout << "zoom " << rndr->core().camera_zoom << std::endl;
			std::cout << "shift " << shift << std::endl;
			std::cout << "translate " << rndr->core().camera_translation << std::endl;

			break;//do nothing
		}
}


void Init(Display& display, igl::opengl::glfw::imgui::ImGuiMenu *menu)
{
	display.AddKeyCallBack(glfw_key_callback);
	display.AddMouseCallBacks(glfw_mouse_press, glfw_mouse_scroll, glfw_mouse_move);
	display.AddResizeCallBack(glfw_window_size);
	menu->init(&display);
}



