////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>
#include <cfloat>
////////////////////////////////////////////////////////////////////////////////

typedef std::complex<double> Point;
typedef std::vector<Point> Polygon;

double inline det(const Point &u, const Point &v) {
	return u.real() * v.imag() - u.imag() * v.real();
}

struct Compare {
	Point p0;
	bool operator ()(const Point &p1, const Point &p2) {
		auto ang1 = std::arg(p1 - p0), ang2 = std::arg(p2 - p0);
		return ang1 < ang2 || (ang1 == ang2 && std::norm(p1 - p0) < std::norm(p2 - p0));
	}
};

bool inline salientAngle(Point &a, Point &b, Point &c) {	
	return det(b - a, c - a) > 0;
}

////////////////////////////////////////////////////////////////////////////////

Polygon convex_hull(std::vector<Point> &points) {
	if (points.size() < 3)
		throw std::runtime_error("points less than 3, no convex hull");
	Compare order;
	double xmin = DBL_MAX, ymin = DBL_MAX;
	for (auto& p : points) {
		if (p.imag() < ymin || (p.imag() == ymin && p.real() < xmin)) {
			ymin = p.imag();
			xmin = p.real();
		}
	}
	//std::cout << "P0: " << xmin << " " << ymin << std::endl;
	order.p0 = Point(xmin, ymin);
	std::sort(points.begin(), points.end(), order);
	Polygon hull;
	for (auto& p : points) {
		while (hull.size() > 1 && !salientAngle(hull[hull.size() - 2], hull.back(), p)) {
			hull.pop_back();
		}
		hull.push_back(p);
	}
	return hull;
}

////////////////////////////////////////////////////////////////////////////////

std::vector<Point> load_xyz(const std::string &filename) {
	std::vector<Point> points;
	std::ifstream in(filename);
	if (!in) {
		throw std::runtime_error("failed to open file" + filename);
	}
	int length, idx = 0;
	in >> length;
	points.resize(length);
	while (!in.eof() && idx < length) {
		double x, y, z;
		in >> x >> y >> z;
		Point temp{x, y};
		points[idx++] = temp;
	}
	return points;
}

void save_obj(const std::string &filename, Polygon &poly) {
	std::ofstream out(filename);
	if (!out.is_open()) {
		throw std::runtime_error("failed to open file " + filename);
	}
	out << std::fixed;
	for (const auto &v : poly) {
		out << "v " << v.real() << ' ' << v.imag() << " 0\n";
	}
	for (size_t i = 0; i < poly.size(); ++i) {
		out << "l " << i+1 << ' ' << 1+(i+1)%poly.size() << "\n";
	}
	out << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[]) {
	if (argc <= 2) {
		std::cerr << "Usage: " << argv[0] << " points.xyz output.obj" << std::endl;
	}
	std::vector<Point> points = load_xyz(argv[1]);
	Polygon hull = convex_hull(points);
	save_obj(argv[2], hull);
	return 0;

	/*
	//test
	std::vector<Point> points = load_xyz("../../../data/points.xyz");
	std::cout << points.size() << std::endl;
	Polygon hull = convex_hull(points);
	std::cout << hull.size() << std::endl;
	save_obj("../../../output.obj", hull);*/
}
