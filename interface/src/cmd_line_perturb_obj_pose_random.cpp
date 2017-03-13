/*****************************************************************************/
/*  Copyright (c) 2015, Karl Pauwels                                         */
/*  All rights reserved.                                                     */
/*                                                                           */
/*  Redistribution and use in source and binary forms, with or without       */
/*  modification, are permitted provided that the following conditions       */
/*  are met:                                                                 */
/*                                                                           */
/*  1. Redistributions of source code must retain the above copyright        */
/*  notice, this list of conditions and the following disclaimer.            */
/*                                                                           */
/*  2. Redistributions in binary form must reproduce the above copyright     */
/*  notice, this list of conditions and the following disclaimer in the      */
/*  documentation and/or other materials provided with the distribution.     */
/*                                                                           */
/*  3. Neither the name of the copyright holder nor the names of its         */
/*  contributors may be used to endorse or promote products derived from     */
/*  this software without specific prior written permission.                 */
/*                                                                           */
/*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS      */
/*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT        */
/*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR    */
/*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT     */
/*  HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   */
/*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT         */
/*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,    */
/*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY    */
/*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT      */
/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE    */
/*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.     */
/*****************************************************************************/

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <robot.h>
#include <ogre_context.h>
#include <ogre_multi_render_target.h>
#include <urdf/model.h>
#include <device_1d.h>
#include <multiple_rigid_models_ogre.h>
#include <utility_kernels_pose.h>
#include <poses_on_icosahedron.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

/*!
   \brief Return the camera projection matrix
   \param input integer width, height and camera matrix in cv::Mat form
   \pre "Pre-conditions"
   \post "Post-conditions"
   \return "Return of the function"
*/
Ogre::Matrix4 generateProjectionMatrix(int width, int height, cv::Mat camera_matrix) {
  double fx = camera_matrix.at<double>(0, 0);
  double fy = camera_matrix.at<double>(1, 1);
  double cx = camera_matrix.at<double>(0, 2);
  double cy = camera_matrix.at<double>(1, 2);

  float near_plane = .001f;
  float far_plane = 10.0f;
  float zoom_x = 1;
  float zoom_y = 1;

  Ogre::Matrix4 projection_matrix = Ogre::Matrix4::ZERO;
  projection_matrix[0][0] = 2.0 * fx / (double)width * zoom_x;
  projection_matrix[1][1] = 2.0 * fy / (double)height * zoom_y;
  projection_matrix[0][2] = 2.0 * (0.5 - cx / (double)width) * zoom_x;
  projection_matrix[1][2] = 2.0 * (cy / (double)height - 0.5) * zoom_y;
  projection_matrix[2][2] =
      -(far_plane + near_plane) / (far_plane - near_plane);
  projection_matrix[2][3] =
      -2.0 * far_plane * near_plane / (far_plane - near_plane);
  projection_matrix[3][2] = -1;

  return projection_matrix;
}

/*!
   \brief Generate vector of rigid objects
   \param Input object file names and object poses in vector format
   \pre number of files equal number of poses
   \post "Post-conditions"
   \return "Return of the function"
*/

  std::vector<std::unique_ptr<render::RigidObject> > setRigidObjects(
        std::vector<string> obj_file_names,
        std::vector<pose::TranslationRotation3D> object_poses,
        const render::OgreContext& ogre_context) {
  // Container of output
  std::vector<std::unique_ptr<render::RigidObject> >rigid_objects;
  for (int o = 0; o < obj_file_names.size(); o++) {
    int segment_ind = o + 1;
    std::string model_resource = "file://" + obj_file_names.at(o);

    auto rigid_object =
        std::unique_ptr<render::RigidObject>{ new render::RigidObject(
            model_resource, ogre_context.scene_manager_, segment_ind) };
    rigid_object->setVisible(true);
    rigid_object->setPose(object_poses.at(o).ogreTranslation(),
                          object_poses.at(o).ogreRotation());
    rigid_objects.push_back(std::move(rigid_object));
  }
  return rigid_objects;
}

int main(int argc, char **argv) {

  // Render objects. Object and camera poses are expressed

  /*********/
  /* INPUT */
  /*********/

  // read object(s) state
  std::vector<pose::TranslationRotation3D> object_poses;
  std::vector<std::string> obj_file_names
  (1,"/home/seasponge/Workspace/catkin_local_ws/src/simtrack-flow-writer/data/object_models/texturedMug/texturedMug.obj");

  int n_objects = 1;
  {
    for (int i = 0; i < n_objects; i++) {
      double T[3] = {0,0,0.0};
      double R[3] = {0,0,0};
      pose::TranslationRotation3D pose;
      pose.setT(T);
      pose.setR(R);
      //    pose.show();
      object_poses.push_back(pose);
    }
  }

  bool show_robot = false;

  int width = 320;
  int height = 240;

  cv::Mat camera_matrix;
  {
    std::vector<int> size;
    std::vector<double> data {525.0/2.,   0.0, 319.5/2., 0.0,
                                       0.0, 525.0/2., 239.5/2., 0.0,
                                       0.0,   0.0,   1.0, 0.0 };
    camera_matrix = cv::Mat(3, 4, CV_64FC1, data.data()).clone();
  }

  auto projection_matrix = generateProjectionMatrix(width, height, camera_matrix);

  /***********/
  /* PROCESS */
  /***********/

  render::OgreContext ogre_context;
  setRigidObjects(obj_file_names, object_poses, ogre_context);

  // create icosahedron
  UniformGridOnIcosahedron ico(10, 10, 1);

  // get scene at object initial pose
  auto camera_poses = ico.getCameraPoses();

  auto camera_pose = camera_poses[0];
  Ogre::Vector3 camera_position = camera_pose.ogreTranslation();
  Ogre::Quaternion camera_orientation = camera_pose.ogreRotation();

  // convert vision (Z-forward) frame to ogre frame (Z-out)
  camera_orientation =
      camera_orientation *
      Ogre::Quaternion(Ogre::Degree(180), Ogre::Vector3::UNIT_X);

  render::OgreMultiRenderTarget ogre_multi_render_target(
      "scene", width, height, ogre_context.scene_manager_);
  ogre_multi_render_target.updateCamera(camera_position, camera_orientation,
                                        projection_matrix);



  ogre_multi_render_target.render();
  std::vector<cudaArray **> cuda_arrays;
  int n_arrays = 6;
  for (int i = 0; i < n_arrays; i++)
    cuda_arrays.push_back(new cudaArray *);

  std::vector<int> out_size{ height, width };
  std::vector<float> h_out(height * width);

  ogre_multi_render_target.mapCudaArrays(cuda_arrays);

  cv::Mat initial_scene = cv::Mat::zeros(height, width, CV_8UC4);

  cudaMemcpyFromArray(initial_scene.data, *cuda_arrays.at(5), 0, 0,
                      width * height * sizeof(float), cudaMemcpyDeviceToHost);

  ogre_multi_render_target.unmapCudaArrays();


  for (auto camera_pose : camera_poses) {
    Ogre::Vector3 camera_position = camera_pose.ogreTranslation();
    Ogre::Quaternion camera_orientation = camera_pose.ogreRotation();

    // convert vision (Z-forward) frame to ogre frame (Z-out)
    camera_orientation =
        camera_orientation *
        Ogre::Quaternion(Ogre::Degree(180), Ogre::Vector3::UNIT_X);
    ogre_multi_render_target.updateCamera(camera_position, camera_orientation,
                                          projection_matrix);


    // render
    ogre_multi_render_target.render();

    /**********/
    /* OUTPUT */
    /**********/
    std::vector<cudaArray **> cuda_arrays;
    int n_arrays = 6;
    for (int i = 0; i < n_arrays; i++)
      cuda_arrays.push_back(new cudaArray *);

    std::vector<int> out_size{ height, width };
    std::vector<float> h_out(height * width);

    ogre_multi_render_target.mapCudaArrays(cuda_arrays);

    cv::Mat texture = cv::Mat::zeros(height, width, CV_8UC4);

    cudaMemcpyFromArray(texture.data, *cuda_arrays.at(5), 0, 0,
                        width * height * sizeof(float), cudaMemcpyDeviceToHost);

    cv::Mat output;
    cv::hconcat(initial_scene,texture,output);
    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Display window", output );   // Show our image inside it.

    cv::waitKey(0);                       // Wait for a keystroke in the window



    ogre_multi_render_target.unmapCudaArrays();
  }


  return EXIT_SUCCESS;
}
