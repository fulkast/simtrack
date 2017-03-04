// icosahedron.cpp: Lanke Fu
// Description: This program generates poses on an icosahedron centered at
// the origin. The spread of the positions is uniform along the longitudes
// and the latitudes. The orientation always points towards the origin.
#include <translation_rotation_3d.h>

#include <iostream>
#include <vector>
#include <algorithm>
using namespace std;

// returns integer from cin. Checks and prompts for appropriate type (int)
int getIntInput() {
  int result;
  while (true) {
    if (!(cin >> result)) {
      std::cout << "please enter an integer!" << std::endl;
      cin.clear();
      cin.ignore(100,'\n');
    } else {
      break;
    }
  }
  return result;
}

// given a high and low value, this generates n_segs uniformly spread out
// segments. It returns a vector of floats which are the beginnings of each
// segment.
// CAUTION: This casts the input type to float
template <typename T, typename S >
vector<float> generateUniformPoints(const T low, const S high, int n_segs) {
  float step = (float(high) - float(low)) / float(n_segs);
  vector<float> result(n_segs);
  float current_point = float(low);
  std::generate(result.begin(), result.end(),[&current_point, step]
                                             {return current_point += step;});
  return result;
}

// prints out elements of an stl container in a line.
// assumes that the contents of the container are cout streamable.
template <typename T>
void printIterableInline(T container) {
  for (auto value : container)
    std::cout << value << " ";
    std::cout << std::endl;
}

// obtain the pose with the camera placed at the coordinates defined by
// radius, latitude and logitude, if the camera were pointing towards the origin
//. This assumes that by default the camera points towards the +z axis
pose::TranslationRotation3D
          getCenterPointingPose(int radius, float latitude, float longitude) {
  // generate position on an origing centered sphere, assume that the north-pole
  // is defined by the latitude value of pi, the south-pole by -pi.
  float x = float(radius);
  float y = x, z = x;
  float latitude_slice_rad = abs(cos(latitude));
  z *= sin(latitude);
  x *= latitude_slice_rad * cos(longitude);
  y *= latitude_slice_rad * sin(longitude);

  // assuming that at the indentity quaternion, the camera points in the z
  // direction, we obtain the quaternion representing the camera being placed
  // at x y z at pointing towards the origin.
  // this is done by finding the quaternion difference between the vector
  // poiting at 0,0,1 and the vector pointing at -x,-y,-z.
  Eigen::Vector3f default_pointing(0, 0, 1);
  Eigen::Vector3f camera_pointing(-x, -y, -z);
  std::cout << "camera pointing ";
  std::cout << x << " "  << y << " " << z << std::endl;
  Eigen::Quaternionf camera_pose_eigen = Eigen::Quaternionf::FromTwoVectors(
                                    default_pointing, camera_pointing);
  // std::cout << "camera pose eigen: ";
  // std::cout << camera_pose_eigen << std::endl;

  // generate pose type from the position and the orientation
  Ogre::Vector3 position(x,y,z);
  Ogre::Quaternion quaternion(camera_pose_eigen.w(), camera_pose_eigen.x(),
                              camera_pose_eigen.y(), camera_pose_eigen.z());
  pose::TranslationRotation3D camera_pose =
                              pose::TranslationRotation3D(position, quaternion);

  return camera_pose;
}

int main(int argc, char const *argv[]) {
  // radius of the icosahedron and the number of points
  // along the azimuth and the altitude
  int radius;
  int n_points_azimuth;
  int n_points_altitude;

  do {
    std::cout << "Enter a radius in mm units (-1 to quit): ";
    radius = getIntInput();
    if (radius == -1) break;
    std::cout << "Enter an integer number of longitudes: ";
    n_points_azimuth = getIntInput();
    std::cout << "Enter an integer number of latitudes: ";
    n_points_altitude = getIntInput();

    // generate coordinate points
    auto longitudes  = generateUniformPoints(0,2*3.14, 10);
    auto latitudes = generateUniformPoints(3.14/2.,-3.14/2., 10);

    for (int i = 0; i < latitudes.size(); i++) {
      float latitude  = 0.0;//latitudes[0];
      float longitude = longitudes[i];
      pose::TranslationRotation3D test_pose = getCenterPointingPose(radius,
                                                  latitude, longitude);
      std::cout << "latitude: " << latitude << " longitude: " << longitude
      << std::endl;
      test_pose.show();
    }


  } while (radius != -1);
  return 0;


}
