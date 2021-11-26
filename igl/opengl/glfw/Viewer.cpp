// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "Viewer.h"

//#include <chrono>
#include <thread>

#include <Eigen/LU>


#include <cmath>
#include <cstdio>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <limits>
#include <cassert>

#include <igl/project.h>
//#include <igl/get_seconds.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/adjacency_list.h>
#include <igl/writeOBJ.h>
#include <igl/writeOFF.h>
#include <igl/massmatrix.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/quat_mult.h>
#include <igl/axis_angle_to_quat.h>
#include <igl/trackball.h>
#include <igl/two_axis_valuator_fixed_up.h>
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/unproject.h>
#include <igl/serialize.h>
#include <igl\vertex_triangle_adjacency.h>
//Ass1 comment
#include <igl/edge_flaps.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/circulation.h>
#include <Eigen/Core>
#include <igl/collapse_edge.h>
#include <set>
//end comment Ass1

// Internal global variables used for glfw event handling
//static igl::opengl::glfw::Viewer * __viewer;
static double highdpi = 1;
static double scroll_x = 0;
static double scroll_y = 0;

//Ass1 comment
using namespace std;
using namespace Eigen;
//end comment Ass1

namespace igl
{
namespace opengl
{
namespace glfw
{

  void Viewer::Init(const std::string config)
  {
	  

  }

  IGL_INLINE Viewer::Viewer():
    data_list(1),
    selected_data_index(0),
    next_data_id(1),
	isPicked(false),
	isActive(false)
  {
    data_list.front().id = 0;

  

    // Temporary variables initialization
   // down = false;
  //  hack_never_moved = true;
    scroll_position = 0.0f;

    // Per face
    data().set_face_based(false);

    
#ifndef IGL_VIEWER_VIEWER_QUIET
    const std::string usage(R"(igl::opengl::glfw::Viewer usage:
  [drag]  Rotate scene
  A,a     Toggle animation (tight draw loop)
  F,f     Toggle face based
  I,i     Toggle invert normals
  L,l     Toggle wireframe
  O,o     Toggle orthographic/perspective projection
  T,t     Toggle filled faces
  [,]     Toggle between cameras
  1,2     Toggle between models
  ;       Toggle vertex labels
  :       Toggle face labels)"
);
    std::cout<<usage<<std::endl;
#endif
  }

  IGL_INLINE Viewer::~Viewer()
  {
  }

  IGL_INLINE bool Viewer::load_mesh_from_file(
      const std::string & mesh_file_name_string)
  {

    // Create new data slot and set to selected
    if(!(data().F.rows() == 0  && data().V.rows() == 0))
    {
      append_mesh();
    }
    data().clear();

    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }

    std::string extension = mesh_file_name_string.substr(last_dot+1);

    if (extension == "off" || extension =="OFF")
    {
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;
      if (!igl::readOFF(mesh_file_name_string, V, F))
        return false;
      data().set_mesh(V,F);
    }
    else if (extension == "obj" || extension =="OBJ")
    {
      Eigen::MatrixXd corner_normals;
      Eigen::MatrixXi fNormIndices;

      Eigen::MatrixXd UV_V;
      Eigen::MatrixXi UV_F;
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;

      if (!(
            igl::readOBJ(
              mesh_file_name_string,
              V, UV_V, corner_normals, F, UV_F, fNormIndices)))
      {
        return false;
      }

      data().set_mesh(V,F);
      if (UV_V.rows() > 0)
      {
          data().set_uv(UV_V, UV_F);
      }

    }
    else
    {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }

    data().compute_normals();
    data().uniform_colors(Eigen::Vector3d(51.0/255.0,43.0/255.0,33.3/255.0),
                   Eigen::Vector3d(255.0/255.0,228.0/255.0,58.0/255.0),
                   Eigen::Vector3d(255.0/255.0,235.0/255.0,80.0/255.0));

    // Alec: why?
    if (data().V_uv.rows() == 0)
    {
      data().grid_texture();
    }
    

    //for (unsigned int i = 0; i<plugins.size(); ++i)
    //  if (plugins[i]->post_load())
    //    return true;

    //Ass1 comment, use OV, OF as copy to play with the V and F with out touch the original variables
    //init data structure for mesh of every data object
    data().OV = data().V;
    data().OF = data().F;
    initMeshdata();// reset- init data
    //end comment Ass1

    return true;
  }

  IGL_INLINE bool Viewer::save_mesh_to_file(
      const std::string & mesh_file_name_string)
  {
    // first try to load it with a plugin
    //for (unsigned int i = 0; i<plugins.size(); ++i)
    //  if (plugins[i]->save(mesh_file_name_string))
    //    return true;

    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      // No file type determined
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }
    std::string extension = mesh_file_name_string.substr(last_dot+1);
    if (extension == "off" || extension =="OFF")
    {
      return igl::writeOFF(
        mesh_file_name_string,data().V,data().F);
    }
    else if (extension == "obj" || extension =="OBJ")
    {
      Eigen::MatrixXd corner_normals;
      Eigen::MatrixXi fNormIndices;

      Eigen::MatrixXd UV_V;
      Eigen::MatrixXi UV_F;

      return igl::writeOBJ(mesh_file_name_string,
          data().V,
          data().F,
          corner_normals, fNormIndices, UV_V, UV_F);
    }
    else
    {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }
    return true;
  }
 
  IGL_INLINE bool Viewer::load_scene()
  {
    std::string fname = igl::file_dialog_open();
    if(fname.length() == 0)
      return false;
    return load_scene(fname);
  }

  IGL_INLINE bool Viewer::load_scene(std::string fname)
  {
   // igl::deserialize(core(),"Core",fname.c_str());
    igl::deserialize(data(),"Data",fname.c_str());
    return true;
  }

  IGL_INLINE bool Viewer::save_scene()
  {
    std::string fname = igl::file_dialog_save();
    if (fname.length() == 0)
      return false;
    return save_scene(fname);
  }

  IGL_INLINE bool Viewer::save_scene(std::string fname)
  {
    //igl::serialize(core(),"Core",fname.c_str(),true);
    igl::serialize(data(),"Data",fname.c_str());

    return true;
  }

  IGL_INLINE void Viewer::open_dialog_load_mesh()
  {
    std::string fname = igl::file_dialog_open();

    if (fname.length() == 0)
      return;
    
    this->load_mesh_from_file(fname.c_str());
  }

  IGL_INLINE void Viewer::open_dialog_save_mesh()
  {
    std::string fname = igl::file_dialog_save();

    if(fname.length() == 0)
      return;

    this->save_mesh_to_file(fname.c_str());
  }

  IGL_INLINE ViewerData& Viewer::data(int mesh_id /*= -1*/)
  {
    assert(!data_list.empty() && "data_list should never be empty");
    int index;
    if (mesh_id == -1)
      index = selected_data_index;
    else
      index = mesh_index(mesh_id);

    assert((index >= 0 && index < data_list.size()) &&
      "selected_data_index or mesh_id should be in bounds");
    return data_list[index];
  }

  IGL_INLINE const ViewerData& Viewer::data(int mesh_id /*= -1*/) const
  {
    assert(!data_list.empty() && "data_list should never be empty");
    int index;
    if (mesh_id == -1)
      index = selected_data_index;
    else
      index = mesh_index(mesh_id);

    assert((index >= 0 && index < data_list.size()) &&
      "selected_data_index or mesh_id should be in bounds");
    return data_list[index];
  }

  IGL_INLINE int Viewer::append_mesh(bool visible /*= true*/)
  {
    assert(data_list.size() >= 1);

    data_list.emplace_back();
    selected_data_index = data_list.size()-1;
    data_list.back().id = next_data_id++;
    //if (visible)
    //    for (int i = 0; i < core_list.size(); i++)
    //        data_list.back().set_visible(true, core_list[i].id);
    //else
    //    data_list.back().is_visible = 0;
    return data_list.back().id;
  }

  IGL_INLINE bool Viewer::erase_mesh(const size_t index)
  {
    assert((index >= 0 && index < data_list.size()) && "index should be in bounds");
    assert(data_list.size() >= 1);
    if(data_list.size() == 1)
    {
      // Cannot remove last mesh
      return false;
    }
    data_list[index].meshgl.free();
    data_list.erase(data_list.begin() + index);
    if(selected_data_index >= index && selected_data_index > 0)
    {
      selected_data_index--;
    }

    return true;
  }

  IGL_INLINE size_t Viewer::mesh_index(const int id) const {
    for (size_t i = 0; i < data_list.size(); ++i)
    {
      if (data_list[i].id == id)
        return i;
    }
    return 0;
  }

  Eigen::Matrix4d Viewer::CalcParentsTrans(int indx) 
  {
	  Eigen::Matrix4d prevTrans = Eigen::Matrix4d::Identity();

	  for (int i = indx; parents[i] >= 0; i = parents[i])
	  {
		  //std::cout << "parent matrix:\n" << scn->data_list[scn->parents[i]].MakeTrans() << std::endl;
		  prevTrans = data_list[parents[i]].MakeTransd() * prevTrans;
	  }

	  return prevTrans;
  }



  //Ass1 comment - reset - init data

  void Viewer::calc_obj_quad_error() {
      
      Eigen::MatrixXd V = data().OV;
      Eigen::MatrixXi F = data().OF;
      std::vector<std::vector<int> > VF;
      std::vector<std::vector<int> > VFi;

      igl::vertex_triangle_adjacency(V, F, VF, VFi);//returns all the faces that

      for (int vi = 0; vi < V.rows(); vi++) {
          //going over on all of the ventices of curr mesh
          // find edges with this verticie
          std::vector<int> faces;
          data().Quads[vi] = Eigen::Matrix4d::Zero();//initializing Quads with 0 matrix before giving values

          for (int fj = 0; fj < VF[vi].size(); fj++) {
              Eigen::Vector3d norm = data().F_normals.row(VF[vi][fj]).normalized();
              //VF[vi][fi] take the j face of the i vertex, compute its normal
              double d = V.row(vi) * norm;
              double a = norm[0], b = norm[1], c = norm[2];
              d *= -1;

              Eigen::Matrix4d Kp;
              Kp.row(0) = Eigen::Vector4d(a * a, a * b, a * c, a * d);
              Kp.row(1) = Eigen::Vector4d(a * b, b * b, b * c, b * d);
              Kp.row(2) = Eigen::Vector4d(a * c, b * c, c * c, c * d);
              Kp.row(3) = Eigen::Vector4d(a * d, d * b, d * c, d * d);

              data().Quads[vi] += Kp;
          }
      }
  }
  void Viewer:: calc_cost_and_position(const int e, const Eigen::MatrixXd& V, double& cost, Eigen::Vector3d& p)
  {
      
      //printf("first line in calc_cost_position\n");
      //printf("e: %d\n", e);
      int v1_index = data().E(e,0);
      //printf("2\n");
      int v2_index = data().E(e, 1);
     // printf("4\n");
      Eigen::Matrix4d q12 = data().Quads[v1_index] + data().Quads[v2_index];//Q = Q1 + Q2
      Eigen::Matrix4d qtag = q12;
      qtag.row(3) = Eigen::Vector4d(0, 0, 0, 1);//4th row vector
      //printf("5\n");
      bool invert;
      Eigen::Vector4d::Scalar det;
      double a;
      //printf("5\n");
      qtag.computeInverseAndDetWithCheck(qtag, det, invert, a);//inverse,determinant,invertible_bool,Threshold
      //printf("6\n");
      Eigen::Vector4d p4;//declaring 4d vector for cost
      //location of new vertex
      if (invert) {
          //minimum quadric error
          p4 = qtag * (Eigen::Vector4d(0, 0, 0, 1));
          p[0] = p4[0];
          p[1] = p4[1];
          p[2] = p4[2];
      }
      else {
          //avg
          p = (V.row(v1_index) + V.row(v2_index)) / 2;
          //p4 << p, 1;//putting the p vector in p4, but p4 is 4 vector so put 1 in p4(3)
          p4[0] = p[0];
          p4[1] = p[1];
          p4[2] = p[2];
          p4[3] = 1;
      }
      cost = formula_cost(p4, q12);
  }
  double Viewer::formula_cost(Eigen::Vector4d p4, Eigen::Matrix4d q12) {
      return p4.transpose() * q12 * p4;
  }
  bool Viewer::new_collapse_edge(Eigen::MatrixXd& V, Eigen::MatrixXi& F) {
      if (data().Q->empty())
      {
          // no edges to collapse
          return false;
      }
      //stage a
      std::pair<double, int> p = *(data().Q->begin());//pop the edge to collapse by Q elements order
      if (p.first == std::numeric_limits<double>::infinity())
      {
          // min cost edge is infinite cost
          return false;
      }
      data().Q->erase(data().Q->begin());//erasing the minimum cost edge

      int e = p.second;//edge index

      (*data().Qit)[e] = data().Q->end();//reset\delete the removed edge iterator
      
      std::vector<int> N = circulation(e, true, data().EMAP, data().EF, data().EI);//Return list of faces around the end point of an edge e(in direction of e1 when e = (e1,e2))
      std::vector<int> Nd = circulation(e, false, data().EMAP, data().EF, data().EI);//Return list of faces the around the end point of an edge e (in direction of e2 when e = (e1,e2))
      N.insert(N.begin(), Nd.begin(), Nd.end());//inserting all faces the around the end point of an edge e to one vector N(insert ND to the begin of N)

      int vid1 = data().E.row(e)[0], vid2 = data().E.row(e)[1];//vertex index1

      int e1, e2, f1, f2;
      Vector3d new_v = data().C.row(e);
      //stage b + c + d of 11 is written by the library function collapse_edge
      bool collapsed = igl::collapse_edge(e, data().C.row(e), V, F, data().E, data().EMAP, data().EF, data().EI, e1, e2, f1, f2);//If valid, then parameters are adjusted accordingly(703example)

      if (collapsed)
      {
          //stage e
          // update the quad of the new vertex		
          data().Quads[vid1] = data().Quads[vid1] + data().Quads[vid2];
          data().Quads[vid2] = data().Quads[vid1] + data().Quads[vid2];

          // Erase the two, other collapsed edges
          (*data().Q).erase((*data().Qit)[e1]);
          (*data().Qit)[e1] = (*data().Q).end();
          (*data().Q).erase((*data().Qit)[e2]);
          (*data().Qit)[e2] = (*data().Q).end();
          // update local neighbors
          // loop over original face neighbors
          for (auto n : N)
          {
              if (F(n, 0) != IGL_COLLAPSE_EDGE_NULL ||
                  F(n, 1) != IGL_COLLAPSE_EDGE_NULL ||
                  F(n, 2) != IGL_COLLAPSE_EDGE_NULL)//checking for original edges of face neighbor existance
              {
                  for (int v = 0; v < 3; v++)//updating all neighbores of the deleted edge
                  {
                      // get edge id
                      const int ei = data().EMAP(v* F.rows() + n);
                      // erase old entry
                      (*data().Q).erase((*data().Qit)[ei]);
                      // compute cost and potential placement
                      double cost;
                      Vector3d place;
                      calc_cost_and_position (ei, V, cost, place);
                      // Replace in queue
                      (*data().Qit)[ei] = (*data().Q).insert(std::pair<double, int>(cost, ei)).first;
                      data().C.row(ei) = place;
                  }
              }
          }
          //@TODO: check if print cost and position is good enough
          //stage f
          std::cout << "edge " << e << ", cost = " << p.first << ", new v position (" << new_v[0] << "," << new_v[1] << "," << new_v[2] << ")" << endl;
          
      }
      else
      {
          // reinsert with infinite weight (the provided cost function must **not**
          // have given this un-collapsable edge inf cost already)
          p.first = std::numeric_limits<double>::infinity();
          (*data().Qit)[e] = (*data().Q).insert(p).first;
      }
      //return if something collapsed
      return collapsed;
  }
  
  void Viewer::initMeshdata() {
      
      Eigen::MatrixXi F = data().OF;
      Eigen::MatrixXd V = data().OV;
      Eigen::VectorXi EMAP;
      Eigen::MatrixXi E, EF, EI;
      Eigen::MatrixXd C;
      PriorityQueue* Q = new PriorityQueue;
      std::vector<PriorityQueue::iterator >* Qit = new std::vector<PriorityQueue::iterator >;// keep the iterators of edges of Q
      //so we can interate and find the neighbor edges of the edge e (lets say that we iteratte using edge e's iterator )

      //second oart Ass1
      data().Quads.resize(V.rows());

      //
      data().F = F;
      data().V = V;
      data().E = E;
      data().EF = EF;
      data().EI = EI;
      data().EMAP = EMAP;
      data().Q = Q;
      data().Qit = Qit;
      data().C = C;
      //


      calc_obj_quad_error();
      //printf("after quad_error\n");
      //end cimmwnt secind oart Ass

      edge_flaps(data().F, data().E, data().EMAP, data().EF, data().EI); // filing our data struct object EMAP, EF, EI, E and F(edges and faces)
      data().Qit->resize(data().E.rows());
      //resize to giving the current space we need
      data().C.resize(data().E.rows(), data().V.cols()); //keep the new vertices location, for each edge e collapsed, keep the new vertex cordinates he became to
      //cooridate of vertex (example (0,0,1))
      Eigen::VectorXd costs(data().E.rows());
      data().Q->clear();
      //printf("before for loop\n");

      for (int e = 0; e < data().E.rows(); e++)
      {
          double cost = e;

          Eigen::Vector3d p(0,0,0);

          //the function adjust costs and placements of inital data object, using shortest edge size as cost alg, and  midpoint of collapsed edge as new vertex alg
          // shortest_edge_and_midpoint(e, V, F, E, EMAP, EF, EI, cost, p);//algorithm to calc cost in simplification, and return new vertices
          //of collapse edges

          //second oart Ass1
          calc_cost_and_position(e, data().V, cost, p);//algorithm to calc cost in simplification
          //end cimmwnt secind oart Ass


          data().C.row(e) = p;//keep new vertex created from edge collapsion
          (*data().Qit)[e] = data().Q->insert(std::pair<double, int>(cost, e)).first;//keep e's iterator
      }
      data().set_mesh(data().V, data().F);     
  }

  void Viewer::meshSimplification(double num_iter) {
      // simplifying the mesh
      bool something_collapsed = false;
      // collapse edge
      const int max_iter = std::ceil(num_iter);//max edge collapse num_iter each time as asked
      for (int j = 0; j < max_iter; j++)
      {
          //this collapse edge, takes:
          //priority queue based edge collapse with function handles to adjust costs and placements,
          //and all other data,
          //and collapsed edge accordingly and adjust cost and placements according to the collapse
          //if (!collapse_edge(shortest_edge_and_midpoint, data().V, data().F, data().E, data().EMAP, data().EF, data().EI, *(data().Q), *(data().Qit), data().C))
          if (!new_collapse_edge(data().V,data().F))
          {
              break;
          }
          something_collapsed = true;
      }

      if (something_collapsed)
      {
          //if something collapsed need to update the mesh of our object data to be with less triangles and current vertices and faces
          Eigen::MatrixXd V = data().V;
          Eigen::MatrixXi F = data().F;
          //line asked to be added in Assignment
          data().clear();
          data().set_mesh(V, F);
          data().set_face_based(true);
          data().dirty = 157;
          //end of comment -line asked to be added in Assignment
      }

      //return false;
  }
  //end comment Ass1

} // end namespace
} // end namespace
}
