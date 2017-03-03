// icosahedron.cpp: Lanke Fu
// Description: This program generates poses on an icosahedron centered at
// the origin. The spread of the positions is uniform along the longitudes
// and the latitudes. The orientation always points towards the origin.
#include <iostream>
using namespace std;



int main(int argc, char const *argv[]) {
  // radius of the icosahedron and the number of points
  // along the azimuth and the altitude
  int radius;
  int n_points_azimuth;
  int n_points_altitude;

  do {
    cout << "Enter a radius: ";
    if(!(cin >> radius))
    {}
  } while (radius != -1);
  return 0;
}
