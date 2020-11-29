// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

void raytrace_sphere() {
	std::cout << "Simple ray tracer, one sphere with orthographic projection" << std::endl;

	const std::string filename("sphere_orthographic.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// Single light source
	const Vector3d light_position(-1,1,1);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_direction = RowVector3d(0,0,-1);

			// Intersect with the sphere
			// NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis
			Vector2d ray_on_xy(ray_origin(0),ray_origin(1));
			const double sphere_radius = 0.9;

			if (ray_on_xy.norm() < sphere_radius) {
				// The ray hit the sphere, compute the exact intersection point
				Vector3d ray_intersection(ray_on_xy(0),ray_on_xy(1),sqrt(sphere_radius*sphere_radius - ray_on_xy.squaredNorm()));

				// Compute normal at the intersection point
				Vector3d ray_normal = ray_intersection.normalized();

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);

}

bool is_intersect_parallelogram(const Vector3d& ray_origin, const Vector3d& ray_direction, const Vector3d& pgram_origin, const Vector3d& pgram_u, const Vector3d& pgram_v, double& u, double& v, double& t) {
	if (!pgram_u.cross(pgram_v).dot(ray_direction))
		return false;
	Matrix3d A;
	A(0, 0) = pgram_u(0);
	A(1, 0) = pgram_u(1);
	A(2, 0) = pgram_u(2);
	A(0, 1) = pgram_v(0);
	A(1, 1) = pgram_v(1);
	A(2, 1) = pgram_v(2);
	A(0, 2) = -ray_direction(0);
	A(1, 2) = -ray_direction(1);
	A(2, 2) = -ray_direction(2);
	Vector3d b = ray_origin - pgram_origin;
	Vector3d x = A.colPivHouseholderQr().solve(b);
	u = x(0);
	v = x(1);
	t = x(2);
	return t >= 0 && u >= 0 && u <= 1 && v >= 0 && v <= 1;
}

void raytrace_parallelogram() {
	std::cout << "Simple ray tracer, one parallelogram with orthographic projection" << std::endl;

	const std::string filename("plane_orthographic.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
	Vector3d pgram_origin(-0.9, -0.5, -0.1);
	Vector3d pgram_u(1, -0.2, 0.4);
	Vector3d pgram_v(0.8, 1, -0.1);

	// Single light source
	const Vector3d light_position(-1,1,1);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_direction = RowVector3d(0,0,-1);

			// TODO: Check if the ray intersects with the parallelogram
			double u = 0, v = 0, t = 0;
			if (is_intersect_parallelogram(ray_origin, ray_direction, pgram_origin, pgram_u, pgram_v, u, v, t)) {
				// TODO: The ray hit the parallelogram, compute the exact intersection point
				Vector3d ray_intersection = ray_origin + t * ray_direction;

				// TODO: Compute normal at the intersection point
				Vector3d normal = pgram_u.cross(pgram_v);
				Vector3d ray_normal = (ray_origin - ray_intersection).dot(normal) > 0 ? normal : -normal;
				ray_normal.normalize();

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);
}

void raytrace_perspective() {
	std::cout << "Simple ray tracer, one parallelogram with perspective projection" << std::endl;

	const std::string filename("plane_perspective.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1, 1, 0.5);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
	Vector3d pgram_origin(-0.5, -0.5, 0);
	Vector3d pgram_u(1.5, 0, 0.3);
	Vector3d pgram_v(0.4, 1, -0.1);

	// Single light source
	const Vector3d light_position(-1,1,1);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// TODO: Prepare the ray (origin point and direction)
			Vector3d ray_origin(-1, 0.5, 1);
			Vector3d ray_direction = origin + double(i) * x_displacement + double(j) * y_displacement - ray_origin;

			// TODO: Check if the ray intersects with the parallelogram
			double u = 0, v = 0, t = 0;
			if (is_intersect_parallelogram(ray_origin, ray_direction, pgram_origin, pgram_u, pgram_v, u, v, t)) {
				// TODO: The ray hit the parallelogram, compute the exact intersection point
				Vector3d ray_intersection = ray_origin + t * ray_direction;

				// TODO: Compute normal at the intersection point
				Vector3d normal = pgram_u.cross(pgram_v);
				Vector3d ray_normal = (ray_origin - ray_intersection).dot(normal) > 0 ? normal : -normal;
				ray_normal.normalize();

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);
}

bool is_intersect_sphere(const Vector3d& ray_origin, const Vector3d& ray_direction, const Vector3d& center, const double& sphere_radius, double& t) {
	double A = ray_direction.dot(ray_direction);
	double B = 2. * ray_direction.dot(ray_origin - center);
	double C = (ray_origin - center).dot((ray_origin - center)) - sphere_radius * sphere_radius;
	double temp = B * B - 4 * A * C;
	if (temp < 0)
		return false;
	else if (temp == 0) {
		t = -B / (2 * A);
		return t >= 0;
	}
	else {
		t = (-B - sqrt(temp)) / (2 * A);
		if (t >= 0)
			return true;
		t = (-B + sqrt(temp)) / (2 * A);
		return t >= 0;
	}
}

void raytrace_shading(){
	std::cout << "Simple ray tracer, one sphere with different shading" << std::endl;

	const std::string filename("shading.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	//Parameters of the sphere (position of the center + radius)
	Vector3d center(0, 0, 0);
	const double sphere_radius = 0.9;

	// Single light source
	//const Vector3d light_position(0, 0, 10);
	const Vector3d light_position(-1,1,1);
	double ambient = 0.1;
	MatrixXd diffuse = MatrixXd::Zero(800, 800);
	MatrixXd specular = MatrixXd::Zero(800, 800);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin(-1, 1, 3);
			Vector3d ray_direction = origin + double(i) * x_displacement + double(j) * y_displacement - ray_origin;

			double t = 0;
			if (is_intersect_sphere(ray_origin, ray_direction, center, sphere_radius, t)) {
				// The ray hit the sphere, compute the exact intersection point
				Vector3d ray_intersection = ray_origin + ray_direction * t;

				// Compute normal at the intersection point
				Vector3d ray_normal = (ray_intersection - center).normalized();

				// TODO: Add shading parameter here
				Vector3d light = (light_position - ray_intersection).normalized();
				Vector3d viewer = -ray_direction.normalized();
				diffuse(i,j) = light.dot(ray_normal);
				double p = 1000;
				specular(i,j) = pow((light + viewer).normalized().dot(ray_normal), p);

				// Simple diffuse model
				C(i,j) = ambient + diffuse(i,j) + specular(i,j);

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C, C, 0. * C, A, filename);
}

int main() {
	raytrace_sphere();
	raytrace_parallelogram();
	raytrace_perspective();
	raytrace_shading();
	return 0;
}
