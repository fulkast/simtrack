// icosahedron.cpp: Lanke Fu
// Description: This program generates poses on an icosahedron centered at
// the origin. The spread of the positions is uniform along the longitudes
// and the latitudes. The orientation always points towards the origin.
#include <iostream>
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
template <typename T>
vector<float> generateUniformPoints(const &T low, const &T high, int n_segs) {
  float diff = float(high) - float(low);
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


  } while (radius != -1);
  return 0;


}
