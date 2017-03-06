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

// prints out elements of an stl container in a line.
// assumes that the contents of the container are cout streamable.
template <typename T>
void printIterableInline(T container) {
  for (auto value : container)
    std::cout << value << " ";
    std::cout << std::endl;
}


// this class creates a 2-D uniform grid on the SURFACE of and ORIGIN CENTERED
// icosahedron.
// it takes in the number of latitudes and longitudes and the
// radius of the icosahedron. It lists all the 3-D points on the grid.
class UniformGridOnIcosahedron {
public:
  UniformGridOnIcosahedron(int n_lat, int n_lon, float rad = 1)
    : n_lat_(n_lat), n_lon_(n_lon), rad_(rad) {
    generateCoordsOnGrid(n_lat_, n_lon_, rad_);
    generateOriginPointingCamPoses();
  }

void showGridPoints() {
  if (coords_on_grid_.empty()) {
    std::cout << "Grid is empty. Did you properly initialize the grid?" << std::endl;
    return;
  }
  std::cout << "Displaying grid points: x y z" << std::endl;
  for (auto point : coords_on_grid_) {
    std::cout << point.transpose() << std::endl;
  }
}

std::vector<pose::TranslationRotation3D> getCameraPoses() const {
  return camera_poses;
}

protected:

  // create for each coordinate on the grid, a camera pose that points to the
  // origin
  void generateOriginPointingCamPoses() {
    camera_poses.clear();
    camera_poses.reserve(coords_on_grid_.size());
    for (auto coord : coords_on_grid_) {
      camera_poses.push_back(
                      getOriginPointingPose(coord.x(), coord.y(), coord.z()));
    }
  }

  // given the number of latitudes, longtitudes and the radius of the
  // icosahedron, generates the 3D points on the surface of the icosahedron.
  void generateCoordsOnGrid(int n_lat, int n_lon, float rad) {
    coords_on_grid_.clear();
    coords_on_grid_.reserve(n_lat * n_lon);
    auto latitudes = generateUniformPoints(3.1415/2.,-3.1415/2., n_lat);
    auto longitudes  = generateUniformPoints(0,2*3.1415, n_lon);
    for (auto lon : longitudes) {
      for (auto lat : latitudes) {
        coords_on_grid_.push_back(getPointFromLatLongRad(lat,lon,rad));
      }
    }
  }

  // obtain the pose with the camera placed at the coordinates defined by
  // radius, latitude and logitude, if the camera were pointing towards the origin
  //. This assumes that by default the camera points towards the +z axis
  pose::TranslationRotation3D getOriginPointingPose(float x, float y, float z) {
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

  // generate position on an origin centered sphere, assume that the north-pole
  // is defined by the latitude value of pi, the south-pole by -pi.
  Eigen::Vector3f getPointFromLatLongRad(float lat, float lon, float radius) {
    float x = radius;
    float y = x, z = x;
    float latitude_slice_rad = abs(cos(lat));
    z *= sin(lat);
    x *= latitude_slice_rad * cos(lon);
    y *= latitude_slice_rad * sin(lon);
    return Eigen::Vector3f(x,y,z);
  }

  // container of 3D coordinates on the grid
  std::vector<Eigen::Vector3f> coords_on_grid_;
  std::vector<pose::TranslationRotation3D> camera_poses;
private:
  // grid resolution and size variables
  int n_lat_, n_lon_, rad_;
};
//
// int main(int argc, char const *argv[]) {
//   // radius of the icosahedron and the number of points
//   // along the azimuth and the altitude
//   int radius;
//   int n_points_azimuth;
//   int n_points_altitude;
//
//
//   do {
//     std::cout << "Enter a radius in mm units (-1 to quit): ";
//     radius = getIntInput();
//     if (radius == -1) break;
//     std::cout << "Enter an integer number of longitudes: ";
//     n_points_azimuth = getIntInput();
//     std::cout << "Enter an integer number of latitudes: ";
//     n_points_altitude = getIntInput();
//
//     // create icosahedron
//     UniformGridOnIcosahedron ico(n_points_altitude, n_points_azimuth, radius);
//
//     ico.showGridPoints();
//
//     auto camera_poses = ico.getCameraPoses();
//     // generate coordinate points
//     // auto longitudes  = generateUniformPoints(0,2*3.14, 10);
//     // auto latitudes = generateUniformPoints(3.14/2.,-3.14/2., 10);
//
//
//   } while (radius != -1);
//   return 0;
//
//
// }
