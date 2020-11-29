////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>
#include <cfloat>
#include <sstream>
////////////////////////////////////////////////////////////////////////////////

typedef std::complex<double> Point;
typedef std::vector<Point> Polygon;

double inline det(const Point &u, const Point &v) {
	return u.real() * v.imag() - u.imag() * v.real();
}

// Return true iff [a,b] intersects [c,d], and store the intersection in ans
//a: uncertain point; b: outside; c, d: vertices
bool intersect_segment(const Point &a, const Point &b, const Point &c, const Point &d, Point &ans) {
	double deno = det(a - b, c - d);
	if (deno == 0)
		return false; //parallel
	double t = det(a - c, c - d) / deno, u = -det(a - b, a - c) / deno;
	if (t == 0) {
		//uncertain point is on the edge of the polygon, not inside
		ans = { -1,-1 }; //assign this point as a break flag
		return false;
	}
	if (u < 0 || u > 1 || t < 0 || t > 1) {
		//intersection is outside the segment
		return false; 
	}
	if ((u == 0 && det(b - a, d - a) > 0) || (u == 1 && det(b - a, c - a) > 0)) {
		//get through the vertex, the intersection counts if the second vertex of the side lies below the ray
		return false;
	}
	ans = { a.real() + t * (b.real() - a.real()) ,a.imag() + t * (b.imag() - a.imag()) };
	return true;
}

////////////////////////////////////////////////////////////////////////////////

bool is_inside(const Polygon &poly, const Point &query) {
	// 1. Compute bounding box and set coordinate of a point outside the polygon
	double xmax = DBL_MIN, ymax = DBL_MIN;
	for (auto& p : poly) {
		if (p.real() > xmax) {
			xmax = p.real();
			ymax = p.imag();
		}
	}
	Point outside(xmax + 100, ymax + 100);
	// 2. Cast a ray from the query point to the 'outside' point, count number of intersections
	int cnt = 0;
	for (unsigned i = 0; i < poly.size(); ++i) {
		Point ans(DBL_MAX, DBL_MAX);
		if (intersect_segment(query, outside, poly[i], poly[(i + 1) % poly.size()], ans)) {
			cnt++;
		}
		else {
			if (ans.real() == -1 && ans.imag() == -1)
				return false;
		}
	}
	return (cnt % 2) != 0;
}

////////////////////////////////////////////////////////////////////////////////

std::vector<Point> load_xyz(const std::string &filename) {
	std::ifstream in(filename);
	if (!in) {
		throw std::runtime_error("failed to open file" + filename);
	}
	std::vector<Point> points;
	int length, idx = 0;
	in >> length;
	points.resize(length);
	while (!in.eof() && idx < length) {
		double x, y, z;
		in >> x >> y >> z;
		Point temp{ x, y };
		points[idx++] = temp;
	}
	return points;
}

Polygon load_obj(const std::string &filename) {
	std::ifstream in(filename);
	if (!in) {
		throw std::runtime_error("failed to open file" + filename);
	}
	std::vector<Point> points;
	std::string line;
	while (getline(in, line)) {
		switch (line[0])
		{
		case 'v': {
			std::istringstream iss(line);
			double x, y, z;
			char inst;
			iss >> inst >> x >> y >> z;
			points.emplace_back(x, y);
			break;
		}
		case 'f':
			break;
		case 'l':
			break;
		default:
			break;
		}
	}
	return points;
}

void save_xyz(const std::string &filename, const std::vector<Point> &points) {
	std::ofstream out(filename);
	if (!out.is_open()) {
		throw std::runtime_error("failed to open file " + filename);
	}
	out << std::fixed;
	out << points.size() << "\n";
	for (auto& p : points) {
		out << p.real() << ' ' << p.imag() << " 0\n";
	}
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[]) {
	if (argc <= 3) {
		std::cerr << "Usage: " << argv[0] << " points.xyz poly.obj result.xyz" << std::endl;
	}
	std::vector<Point> points = load_xyz(argv[1]);
	Polygon poly = load_obj(argv[2]);
	std::vector<Point> result;
	for (size_t i = 0; i < points.size(); ++i) {
		if (is_inside(poly, points[i])) {
			result.push_back(points[i]);
		}
	}
	save_xyz(argv[3], result);
	return 0;

	//std::vector<Point> points = load_xyz("../../../data/points.xyz");
	//Polygon poly = load_obj("../../../data/polygon.obj");
	//std::vector<Point> result;
	//for (size_t i = 0; i < points.size(); ++i) {
	//	if (is_inside(poly, points[i])) {
	//		result.push_back(points[i]);
	//	}
	//}
	//save_xyz("../../../result.xyz", result);
}
