#ifndef MAPCELL_H
#define MAPCELL_H
#include <cmath>
#include <cstdlib>
#include <limits>
#include <vector>

class mapCell{
	
public:
	struct carState{
		double x, y, theta, t;
		carState(double x, double y, double theta, double t);
		carState();
	};
	double x, y;//cell center position
	int l, t, s;//lane track station
	double v;
	double h;
	double g;
	double f;
	carState state;//Car state in cell
	mapCell* parent;
	mapCell();
	//Using in InitMAP
	mapCell(carState state, int l, int t, int s);
	double getH();
	int compareF(mapCell* b);
private:
	

};


#endif