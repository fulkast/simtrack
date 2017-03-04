// icosahedron.cpp: Lanke Fu
// Description: This program generates poses on an icosahedron centered at
// the origin. The spread of the positions is uniform along the longitudes
// and the latitudes. The orientation always points towards the origin.
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

template <typename T>
void printIterableInline(T container) {
  for (auto value : container)
    std::cout << value << " ";
    std::cout << std::endl;
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
    auto latitudes  = generateUniformPoints(0,2*3.14, 10);
    auto longitudes = generateUniformPoints(0,3.14, 10);



  } while (radius != -1);
  return 0;


}
