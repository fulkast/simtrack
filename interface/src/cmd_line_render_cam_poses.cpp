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
#include <sstream>
#include <stdexcept>
#include <hdf5_file.h>
#include <device_2d.h>
#include <utilities.h>
#include <multi_rigid_detector.h>
#include <multi_rigid_tracker.h>
#include <poses_on_icosahedron.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv) {

  /*********/
  /* INPUT */
  /*********/

  interface::MultiRigidDetector::Parameters detector_parameters;
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

  // create icosahedron
  UniformGridOnIcosahedron ico(10, 10, 0.3);
  auto camera_poses = ico.getCameraPoses();

  for (size_t i = 0; i < camera_poses.size(); ++i) {
    cv::Mat image;
    image =
    cv::imread("/home/seasponge/Workspace/DartExample/video/color/color00000.png", CV_LOAD_IMAGE_GRAYSCALE);

    tracker.updatePoses(image);

    cv::Mat output_image = tracker.generateOutputImage(
        interface::MultiRigidTracker::OutputImageType::
            optical_flow_x);
    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Display window", output_image );   // Show our image inside it.

    cv::waitKey(0);                       // Wait for a keystroke in the window

    // std::vector<pose::TranslationRotation3D> camera_poses(1);
    // {
    //   for (int i = 0; i < 1; i++) {
    //     double T[3] = {0,0,-1.0};
    //     double R[3] = {0,0,0};
    //     pose::TranslationRotation3D pose;
    //     pose.setT(T);
    //     pose.setR(R);
    //     //    pose.show();
    //     camera_poses[0] = pose;
    //   }
    // }

    tracker.setCameraPose(camera_poses[i]);

  }

  return EXIT_SUCCESS;
}
