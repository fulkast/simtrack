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
#include <random>
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
#include <multi_rigid_tracker.h>
#include <utilities.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <json.hpp>

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

  std::vector<std::unique_ptr<render::RigidObject> > getRigidObjects(
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

cv::Mat getScene(int width, int height,
                render::OgreMultiRenderTarget& ogre_multi_render_target ) {

  std::vector<cudaArray **> cuda_arrays;
  int n_arrays = 6;
  for (int i = 0; i < n_arrays; i++)
    cuda_arrays.push_back(new cudaArray *);

  std::vector<int> out_size{ height, width };
  std::vector<float> h_out(height * width);

  ogre_multi_render_target.mapCudaArrays(cuda_arrays);

  cv::Mat scene = cv::Mat::zeros(height, width, CV_8UC4);

  cudaMemcpyFromArray(scene.data, *cuda_arrays.at(4), 0, 0,
                      width * height * sizeof(float), cudaMemcpyDeviceToHost);

  ogre_multi_render_target.unmapCudaArrays();
  return scene;
}




int main(int argc, char **argv) {

    /*********/
    /* INPUT */
    /*********/

    std::string save_dir =
    "/home/seasponge/Workspace/random_trees_with_simtrack/data/json-flow-vector-data/";

    vision::D_OpticalAndARFlow::Parameters flow_parameters;
    pose::D_MultipleRigidPoses::Parameters pose_parameters;

    pose_parameters.w_disp_ = 0.0;
  //  pose_parameters.w_flow_ = 0.0;
    pose_parameters.w_flow_ = 1.0;
    flow_parameters.consistent_ = false;
    pose_parameters.check_reliability_ = false;

    std::vector<std::string> tracker_filenames{
      "/home/seasponge/Workspace/catkin_local_ws/src/simtrack-flow-writer/data/object_models/texturedMug/texturedMug.obj"
    };

    std::vector<pose::TranslationRotation3D> object_poses;

    int n_objects = 1;

    pose::TranslationRotation3D initial_object_pose;
    {
      for (int i = 0; i < n_objects; i++) {
        double T[3] = {0,0,0.0};
        double R[3] = {0,0,0};
        initial_object_pose.setT(T);
        initial_object_pose.setR(R);
        //    pose.show();
        object_poses.push_back(initial_object_pose);
      }
    }
    std::vector<pose::TranslationRotation3D>initial_poses(1,
                                                          initial_object_pose);

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


    /***********/
    /* PROCESS */
    /***********/

    int device_id = 0;
    util::initializeCUDARuntime(device_id);

    std::vector<interface::MultiRigidTracker::ObjectInfo> object_info;
    for (int i = 0; i < n_objects; ++i)
      object_info.push_back(interface::MultiRigidTracker::ObjectInfo(
          "dummy_label", tracker_filenames.at(i)));

    interface::MultiRigidTracker tracker(width, height, camera_matrix,
                                         object_info, flow_parameters,
                                         pose_parameters);
    tracker.setPoses(object_poses);
    double T[3] = {0,0,-.3};
    double R[3] = {0,0,0};
    pose::TranslationRotation3D cam_pose;
    cam_pose.setT(T);
    cam_pose.setR(R);
    tracker.setCameraPose(cam_pose);
    cv::Mat image;
    image =
    cv::imread("/home/seasponge/Workspace/DartExample/video/color/color00000.png", CV_LOAD_IMAGE_GRAYSCALE);
    tracker.updatePoses(image);

    // get initial image
    cv::Mat initial_appearance = tracker.generateOutputImage(
        interface::MultiRigidTracker::OutputImageType::
            model_appearance_blended);

    cv::Mat initial_mask = tracker.getGrayObjectMask();
    cv::normalize(initial_mask, initial_mask, 0, 1, cv::NORM_MINMAX);

    std::vector<uchar> initial_mask_vector_data = tracker.getObjectMaskVector();
    std::vector<float> initial_mask_vector(initial_mask_vector_data.size());

    for (int i = 0; i < initial_mask_vector.size(); i++)
      initial_mask_vector[i] = (isnan(initial_mask_vector_data[i]) ||
                                  initial_mask_vector_data[i] == 0) ? 0.0 : 1.0;

    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-30./180.*3.14,
                                                         30./180.*3.14);


    for (size_t i = 0; i < 200; ++i) {

      double random_x = distribution(generator);
      {
        for (int i = 0; i < n_objects; i++) {
          double T[3] = {0,0,0.0};
          double R[3] = {random_x,0,0};
          pose::TranslationRotation3D pose;
          pose.setT(T);
          pose.setR(R);
          //    pose.show();
          object_poses[i] = pose;
        }
      }

      tracker.setPoses(object_poses);
      tracker.updatePoses(image);

      cv::Mat perturbed_appearance = tracker.generateOutputImage(
          interface::MultiRigidTracker::OutputImageType::
              model_appearance);

      cv::putText(perturbed_appearance, "X:" + std::to_string(random_x),
          cvPoint(30,30),
          cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8,cvScalar(200,200,250), 1, CV_AA);

      cv::Mat perturbed_flow = tracker.generateOutputImage(
          interface::MultiRigidTracker::OutputImageType::
              optical_flow_y);

      cv::multiply(initial_mask, perturbed_flow, perturbed_flow);
      // std::cout << perturbed_appearance << std::endl;
      std::vector<float> masked_flow_vector(initial_mask_vector.size());
      std::vector<float> flow_y_vector = tracker.getFloatFlowYVector();
      std::transform( flow_y_vector.begin()+1, flow_y_vector.end(),
                initial_mask_vector.begin()+1, masked_flow_vector.begin(),  // assumes v1,v2 of same size > 1,
                                          //       v one element smaller
                std::multiplies<float>() );

      float aggregate = std::accumulate(flow_y_vector.begin(), flow_y_vector.end(),
                                      0.0, [](float a, float b) {
                                        return isnan(b) ? a : a += b;
                                      });
      std::cout << "average flow: " << aggregate/flow_y_vector.size() << std::endl;
      nlohmann::json jx;
      string filename = save_dir + "frame" + std::to_string(i) + ".json";
      jx["masked_flow_vector"] = masked_flow_vector;
      jx["perturbed_x"] = random_x;
      std::ofstream ox(filename);
      ox << jx << std::endl;

      cv::Mat output_image;
      cv::hconcat(initial_mask, perturbed_appearance, output_image);
      cv::hconcat(output_image, perturbed_flow, output_image);
      cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
      cv::imshow( "Display window", output_image );   // Show our image inside it.

      cv::waitKey(0);                       // Wait for a keystroke in the window


      // reset to initial object pose
      tracker.setPoses(initial_poses);
      tracker.updatePoses(image);

    }



  return EXIT_SUCCESS;
}
