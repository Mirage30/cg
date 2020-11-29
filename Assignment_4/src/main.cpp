////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <stack>
#include <cfloat>

// Eigen for matrix operations
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"
#include "utils.h"

// JSON parser library (https://github.com/nlohmann/json)
#include "json.hpp"
using json = nlohmann::json;

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

#define EPSILON 0.0001
#define AABB_TREE
//#define BETTER

////////////////////////////////////////////////////////////////////////////////
// Define types & classes
////////////////////////////////////////////////////////////////////////////////

struct Ray {
	Vector3d origin;
	Vector3d direction;
	Ray() { }
	Ray(Vector3d o, Vector3d d) : origin(o), direction(d) { }
};

struct Light {
	Vector3d position;
	Vector3d intensity;
};

struct Intersection {
	Vector3d position;
	Vector3d normal;
	double ray_param;
};

struct Camera {
	bool is_perspective;
	Vector3d position;
	double field_of_view; // between 0 and PI
	double focal_length;
	double lens_radius; // for depth of field
};

struct Material {
	Vector3d ambient_color;
	Vector3d diffuse_color;
	Vector3d specular_color;
	double specular_exponent; // Also called "shininess"

	Vector3d reflection_color;
	Vector3d refraction_color;
	double refraction_index;
};

struct Object {
	Material material;
	virtual ~Object() = default; // Classes with virtual methods should have a virtual destructor!
	virtual bool intersect(const Ray &ray, Intersection &hit) = 0;
};

// We use smart pointers to hold objects as this is a virtual class
typedef std::shared_ptr<Object> ObjectPtr;

struct Sphere : public Object {
	Vector3d position;
	double radius;

	virtual ~Sphere() = default;
	virtual bool intersect(const Ray &ray, Intersection &hit) override;
};

struct Parallelogram : public Object {
	Vector3d origin;
	Vector3d u;
	Vector3d v;

	virtual ~Parallelogram() = default;
	virtual bool intersect(const Ray &ray, Intersection &hit) override;
};

struct AABBTree {
	struct Node {
		AlignedBox3d bbox;
		int parent; // Index of the parent node (-1 for root)
		int left; // Index of the left child (-1 for a leaf)
		int right; // Index of the right child (-1 for a leaf)
		int triangle; // Index of the node triangle (-1 for internal nodes)
	};

	std::vector<Node> nodes;
	int root = -1;

	AABBTree() = default; // Default empty constructor
	AABBTree(const MatrixXd &V, const MatrixXi &F); // Build a BVH from an existing mesh
private:
	int construct_tree(const MatrixXd& V, const MatrixXi& F, const MatrixXd& centroids, std::vector<int>& triangleSet, const int& parent);
};

struct Mesh : public Object {
	MatrixXd vertices; // n x 3 matrix (n points)
	MatrixXi facets; // m x 3 matrix (m triangles)

	AABBTree bvh;

	Mesh() = default; // Default empty constructor
	Mesh(const std::string &filename);
	virtual ~Mesh() = default;
	virtual bool intersect(const Ray &ray, Intersection &hit) override;
private:
	bool search_BVH(const Ray& ray, Intersection& closest_hit, const int& nodeIdx);
};

struct Scene {
	Vector3d background_color;
	Vector3d ambient_light;

	Camera camera;
	std::vector<Material> materials;
	std::vector<Light> lights;
	std::vector<ObjectPtr> objects;
};

////////////////////////////////////////////////////////////////////////////////

// Read a triangle mesh from an off file
void load_off(const std::string &filename, MatrixXd &V, MatrixXi &F) {
	std::ifstream in(filename);
	std::string token;
	in >> token;
	int nv, nf, ne;
	in >> nv >> nf >> ne;
	V.resize(nv, 3);
	F.resize(nf, 3);
	for (int i = 0; i < nv; ++i) {
		in >> V(i, 0) >> V(i, 1) >> V(i, 2);
	}
	for (int i = 0; i < nf; ++i) {
		int s;
		in >> s >> F(i, 0) >> F(i, 1) >> F(i, 2);
		assert(s == 3);
	}
}

Mesh::Mesh(const std::string &filename) {
	// Load a mesh from a file (assuming this is a .off file), and create a bvh
	load_off(filename, vertices, facets);
	bvh = AABBTree(vertices, facets);
}

////////////////////////////////////////////////////////////////////////////////
// BVH Implementation
////////////////////////////////////////////////////////////////////////////////

// Bounding box of a triangle
AlignedBox3d bbox_triangle(const Vector3d &a, const Vector3d &b, const Vector3d &c) {
	AlignedBox3d box;
	box.extend(a);
	box.extend(b);
	box.extend(c);
	return box;
}

int AABBTree::construct_tree(const MatrixXd& V, const MatrixXi& F, const MatrixXd& centroids, std::vector<int>& triangleSet, const int& parent) {
	if (triangleSet.size() == 1) {
		Node node;
		node.bbox = bbox_triangle(V.row(F(triangleSet[0], 0)), V.row(F(triangleSet[0], 1)), V.row(F(triangleSet[0], 2)));
		node.parent = parent;
		node.left = -1;
		node.right = -1;
		node.triangle = triangleSet[0];
		nodes.push_back(node);
		return nodes.size() - 1;
	}

	// Compute the longest axis
	Vector3d cmax(DBL_MIN, DBL_MIN, DBL_MIN), cmin(DBL_MAX, DBL_MAX, DBL_MAX);
	for (int &i: triangleSet) {
		for (int k = 0; k < centroids.cols(); ++k) {
			cmax(k) = std::max(cmax(k), centroids(i, k));
			cmin(k) = std::min(cmin(k), centroids(i, k));
		}
	}
	int longSpanIdx = -1; // the index of the longest axis
	double spanLength = -1;
	for (int i = 0; i < cmax.rows(); ++i) {
		if (cmax(i) - cmin(i) > spanLength) {
			spanLength = cmax(i) - cmin(i);
			longSpanIdx = i;
		}
	}

	// Construct this internal node
	Node node;
	node.bbox.extend(cmax);
	node.bbox.extend(cmin);
	for (int& i : triangleSet) {
		for (int k = 0; k < F.cols(); ++k) {
			node.bbox.extend(V.row(F(i, k)).transpose());
		}
	}
	node.parent = parent;
	node.triangle = -1;
	int cur = nodes.size(); // record the index of the current node 
	nodes.push_back(node);

	// Sort the centroids along one direction
	std::sort(triangleSet.begin(), triangleSet.end(), [&](const int& a, const int& b) {return centroids(a, longSpanIdx) < centroids(b, longSpanIdx); });
	std::vector<int> part1;
	std::vector<int> part2;
	for (int i = 0; i < triangleSet.size(); ++i) {
		if (i < triangleSet.size() / 2)
			part1.push_back(triangleSet[i]);
		else
			part2.push_back(triangleSet[i]);
	}
	int left = construct_tree(V, F, centroids, part1, cur);
	int right = construct_tree(V, F, centroids, part2, cur);
	nodes[cur].left = left;
	nodes[cur].right = right;
	return cur;
}

AABBTree::AABBTree(const MatrixXd &V, const MatrixXi &F) {
	// Compute the centroids of all the triangles in the input mesh
	MatrixXd centroids(F.rows(), V.cols());
	centroids.setZero();
	for (int i = 0; i < F.rows(); ++i) {
		for (int k = 0; k < F.cols(); ++k) {
			centroids.row(i) += V.row(F(i, k));
		}
		centroids.row(i) /= F.cols();
	}

	// TODO (Assignment 3)

	// Method (1): Top-down approach.
	// Split each set of primitives into 2 sets of roughly equal size,
	// based on sorting the centroids along one direction or another.

	// Method (2): Bottom-up approach.
	// Merge nodes 2 by 2, starting from the leaves of the forest, until only 1 tree is left.
	if (centroids.rows() == 0)
		return;

	std::vector<int> triangleSet(centroids.rows());
	for (int i = 0; i < triangleSet.size(); ++i)
		triangleSet[i] = i;

	root = construct_tree(V, F, centroids, triangleSet, -1);
}

////////////////////////////////////////////////////////////////////////////////

bool Sphere::intersect(const Ray &ray, Intersection &hit) {
	// TODO (Assignment 2)
	double A = ray.direction.dot(ray.direction);
	double B = 2. * ray.direction.dot(ray.origin - position);
	double C = (ray.origin - position).dot(ray.origin - position) - radius * radius;
	double rt = B * B - 4 * A * C;
	if (rt < 0)
		return false;
	else if (rt == 0) {
		hit.ray_param = -B / (2 * A);
		if (hit.ray_param < 0)
			return false;
	}
	else {
		hit.ray_param = (-B - sqrt(rt)) / (2 * A);
		if (hit.ray_param < 0) {
			hit.ray_param = (-B + sqrt(rt)) / (2 * A);
			if (hit.ray_param < 0)
				return false;
		}
	}
	hit.position = ray.origin + hit.ray_param * ray.direction;
	hit.normal = (hit.position - position).normalized();
	return true;
}

bool Parallelogram::intersect(const Ray &ray, Intersection &hit) {
	// TODO (Assignment 2)
	if (!u.cross(v).dot(ray.direction))
		return false; // parallel
	Matrix3d A;
	for (int i = 0; i < 3; ++i) {
		A(i, 0) = u(i);
		A(i, 1) = v(i);
		A(i, 2) = -ray.direction(i);
	}
	Vector3d b = ray.origin - origin;
	Vector3d x = A.colPivHouseholderQr().solve(b);
	if (x(2) <= 0 || x(0) < 0 || x(0) > 1 || x(1) < 0 || x(1) > 1)
		return false;
	hit.ray_param = x(2);
	hit.position = ray.origin + ray.direction * hit.ray_param;
	hit.normal = u.cross(v).dot(ray.direction) < 0 ? u.cross(v) : -u.cross(v);
	hit.normal.normalize();
	return true;
}

// -----------------------------------------------------------------------------

bool intersect_triangle(const Ray &ray, const Vector3d &a, const Vector3d &b, const Vector3d &c, Intersection &hit) {
	// TODO (Assignment 3)
	//
	// Compute whether the ray intersects the given triangle.
	// If you have done the parallelogram case, this should be very similar to it.
	Vector3d ab = (b - a), ac = (c - a);
	if (!ab.cross(ac).dot(ray.direction))
		return false; //parallel
	Matrix3d A;
	for (int i = 0; i < 3; ++i) {
		A(i, 0) = ab(i);
		A(i, 1) = ac(i);
		A(i, 2) = -ray.direction(i);
	}
	Vector3d B = ray.origin - a;
	Vector3d x = A.colPivHouseholderQr().solve(B);
	double u = x(0), v = x(1);
	hit.ray_param = x(2);
	if (hit.ray_param <= 0 || u < 0 || v < 0 || u + v > 1)
		return false;
	hit.position = ray.origin + x(2) * ray.direction;
	hit.normal = ab.cross(ac);
	hit.normal = hit.normal.dot(ray.direction) < 0 ? hit.normal : -hit.normal;
	hit.normal.normalize();
	return true;
}

bool intersect_box(const Ray &ray, const AlignedBox3d &box) {
	// TODO (Assignment 3)
	//
	// Compute whether the ray intersects the given box.
	// There is no need to set the resulting normal and ray parameter, since
	// we are not testing with the real surface here anyway.

	// D is the inverse of ray.direction
	Vector3d D(1. / ray.direction(0), 1. / ray.direction(1), 1. / ray.direction(2));
	// TA, TB record the parameters
	Vector3d TA = (box.min() - ray.origin).cwiseProduct(D);
	Vector3d TB = (box.max() - ray.origin).cwiseProduct(D);
	double tmin = std::max({ std::min(TA(0), TB(0)), std::min(TA(1), TB(1)), std::min(TA(2), TB(2)) });
	double tmax = std::min({ std::max(TA(0), TB(0)), std::max(TA(1), TB(1)), std::max(TA(2), TB(2)) });
	return (tmax >= 0 && tmin <= tmax);
}

bool Mesh::intersect(const Ray &ray, Intersection &closest_hit) {
	// TODO (Assignment 3)
#ifndef AABB_TREE
	// Method (1): Traverse every triangle and return the closest hit.
	Intersection hit;
	closest_hit.ray_param = DBL_MAX;
	bool intersects = false;
	for (int i = 0; i < facets.rows(); ++i) {
		if (intersect_triangle(ray, vertices.row(facets(i, 0)), vertices.row(facets(i, 1)), vertices.row(facets(i, 2)), hit) && hit.ray_param < closest_hit.ray_param) {
			closest_hit = hit;
			intersects = true;
		}
	}
	return intersects;
#else
	// Method (2): Traverse the BVH tree and test the intersection with a
	// triangles at the leaf nodes that intersects the input ray.
	closest_hit.ray_param = DBL_MAX;
	return search_BVH(ray, closest_hit, bvh.root);
#endif
}

bool Mesh::search_BVH(const Ray& ray, Intersection& closest_hit, const int& nodeIdx) {
	if (nodeIdx < 0 || nodeIdx >= bvh.nodes.size() || !intersect_box(ray, bvh.nodes[nodeIdx].bbox))
		return false;
	auto& node = bvh.nodes[nodeIdx];

	//leaf node
	if (node.triangle != -1) {
		Intersection hit;
		if (intersect_triangle(ray, vertices.row(facets(node.triangle, 0)), vertices.row(facets(node.triangle, 1)), vertices.row(facets(node.triangle, 2)), hit)) {
			if (hit.ray_param < closest_hit.ray_param)
				closest_hit = hit;
			return true;
		}
		return false;
	}

	//internal node
	bool left = search_BVH(ray, closest_hit, node.left);
	bool right = search_BVH(ray, closest_hit, node.right);
	return left || right;
}

////////////////////////////////////////////////////////////////////////////////
// Define ray-tracing functions
////////////////////////////////////////////////////////////////////////////////

// Function declaration here (could be put in a header file)
Vector3d ray_color(const Scene &scene, const Ray &ray, const Object &object, const Intersection &hit, int max_bounce);
Object * find_nearest_object(const Scene &scene, const Ray &ray, Intersection &closest_hit);
bool is_light_visible(const Scene& scene, const Intersection& hit, const Light& light);
Vector3d shoot_ray(const Scene &scene, const Ray &ray, int max_bounce);

// -----------------------------------------------------------------------------

#ifdef BETTER
// eta = 1/e = sin(theta2)/sin(theta1) = n2/n1 = n2 (if n1 is vacuum)
bool refract(const Vector3d& ray_direction, const Vector3d& normal, const double& eta, Vector3d& refract) {
	double cost1 = -ray_direction.dot(normal);
	double cost2 = 1. - eta * eta * (1. - cost1 * cost1);
	if (cost2 <= 0)
		return false; //total internal reflection
	refract = (eta * ray_direction + normal * (eta * cost1 - sqrt(cost2))).normalized();
	return true;
}
#endif

Vector3d ray_color(const Scene &scene, const Ray &ray, Object &obj, const Intersection &hit, int max_bounce) {
	// Material for hit object
	const Material &mat = obj.material;

	// Ambient light contribution
	Vector3d ambient_color = obj.material.ambient_color.array() * scene.ambient_light.array();

	// Punctual lights contribution (direct lighting)
	Vector3d lights_color(0, 0, 0);
	for (const Light &light : scene.lights) {
		Vector3d Li = (light.position - hit.position).normalized();
		Vector3d N = hit.normal;

		// TODO (Assignment 2, shadow rays)
#ifdef BETTER
		if (!is_light_visible(scene, hit, light))
			continue;
#endif

		// Diffuse contribution
		Vector3d diffuse = mat.diffuse_color * std::max(Li.dot(N), 0.0);

		// TODO (Assignment 2, specular contribution)
		Vector3d h = (Li - ray.direction).normalized();
		Vector3d specular = mat.specular_color * pow(std::max(h.dot(N), 0.0), mat.specular_exponent);

		// Attenuate lights according to the squared distance to the lights
		Vector3d D = light.position - hit.position;
		lights_color += (diffuse + specular).cwiseProduct(light.intensity) /  D.squaredNorm();
	}

	// TODO (Assignment 2, reflected ray)
	Vector3d reflection_color(0, 0, 0);
#ifdef BETTER
	if (mat.reflection_color != Vector3d(0, 0, 0) && max_bounce > 0) {
		Ray reflect_ray;
		reflect_ray.direction = ray.direction - 2 * (ray.direction.dot(hit.normal)) * hit.normal;
		reflect_ray.origin = hit.position + EPSILON * reflect_ray.direction;
		reflection_color = mat.reflection_color.cwiseProduct(shoot_ray(scene, reflect_ray, max_bounce - 1));
	}
#endif

	// TODO (Assignment 2, refracted ray)
	Vector3d refraction_color(0, 0, 0);
#ifdef BETTER
	Ray refract_ray; // from outside to inside
	if (mat.refraction_color != Vector3d(0, 0, 0)) {
		if (max_bounce <= 0)
			refraction_color = scene.background_color;
		else {
			refract(ray.direction, hit.normal, mat.refraction_index, refract_ray.direction);
			refract_ray.origin = hit.position + EPSILON * refract_ray.direction;
			Intersection h;
			Ray new_ray; // from inside to outside
			if (obj.intersect(refract_ray, h) && refract(refract_ray.direction, h.normal, 1. / mat.refraction_index, new_ray.direction)) {
				new_ray.origin = h.position + EPSILON * new_ray.direction;
				refraction_color = mat.refraction_color.cwiseProduct(shoot_ray(scene, new_ray, max_bounce - 1));
			}
		}
	}
#endif

	// Rendering equation
	Vector3d C = ambient_color + lights_color + reflection_color + refraction_color;

	return C;
}

// -----------------------------------------------------------------------------

Object * find_nearest_object(const Scene &scene, const Ray &ray, Intersection &closest_hit) {
	int closest_index = -1;
	// TODO (Assignment 2, find nearest hit)
	for (unsigned i = 0; i < scene.objects.size(); ++i) {
		Intersection hit;
		if (scene.objects[i]->intersect(ray, hit) && (closest_index == -1 || hit.ray_param < closest_hit.ray_param)) {
			closest_index = i;
			closest_hit = hit;
		}
	}

	if (closest_index < 0) {
		// Return a NULL pointer
		return nullptr;
	} else {
		// Return a pointer to the hit object. Don't forget to set 'closest_hit' accordingly!
		return scene.objects[closest_index].get();
	}
}

bool is_light_visible(const Scene &scene, const Intersection& hit, const Light &light) {
	// TODO (Assignment 2, shadow ray)
	Vector3d Li = (light.position - hit.position).normalized();

	// shadow ray with an offset by a small epsilon value
	Ray shadow_ray(hit.position + Li * EPSILON, Li);

	// parameter of the light position on the ray, to check whether the object is behind the light
	double t = (light.position - shadow_ray.origin)(0) / shadow_ray.direction(0);
	for (const auto& obj : scene.objects) {
		Intersection h;
		if (obj->intersect(shadow_ray, h) && h.ray_param < t) {
			return false;
		}
	}
	return true;
}

Vector3d shoot_ray(const Scene &scene, const Ray &ray, int max_bounce) {
	Intersection hit;
	if (Object * obj = find_nearest_object(scene, ray, hit)) {
		// 'obj' is not null and points to the object of the scene hit by the ray
		return ray_color(scene, ray, *obj, hit, max_bounce);
	} else {
		// 'obj' is null, we must return the background color
		return scene.background_color;
	}
}

////////////////////////////////////////////////////////////////////////////////

void render_scene(const Scene &scene) {
	std::cout << "Simple ray tracer." << std::endl;

	int w = 640;
	int h = 480;
	MatrixXd R = MatrixXd::Zero(w, h);
	MatrixXd G = MatrixXd::Zero(w, h);
	MatrixXd B = MatrixXd::Zero(w, h);
	MatrixXd A = MatrixXd::Zero(w, h); // Store the alpha mask

	// The camera always points in the direction -z
	// The sensor grid is at a distance 'focal_length' from the camera center,
	// and covers an viewing angle given by 'field_of_view'.
	double aspect_ratio = double(w) / double(h);
	double scale_y = tan(scene.camera.field_of_view / 2.) * scene.camera.focal_length;; // TODO: Stretch the pixel grid by the proper amount here
	double scale_x = scale_y * aspect_ratio; //

	// The pixel grid through which we shoot rays is at a distance 'focal_length'
	// from the sensor, and is scaled from the canonical [-1,1] in order
	// to produce the target field of view.
	Vector3d grid_origin(-scale_x, scale_y, -scene.camera.focal_length);
	Vector3d x_displacement(2.0/w*scale_x, 0, 0);
	Vector3d y_displacement(0, -2.0/h*scale_y, 0);

	// generate many ray to implement depth of field
	const double tmp = sqrt(0.5);
	const std::vector<std::pair<double, double>> offset = { {0,1},{0,-1},{1,0},{-1,0},{tmp, tmp},{-tmp, tmp},{tmp, -tmp},{-tmp, -tmp} }; // relative position
	const std::vector<double> radius_o = { scene.camera.lens_radius }; // 9 points
	//const std::vector<double> radius_o = { scene.camera.lens_radius, scene.camera.lens_radius / 2 }; // 17 points
	//const std::vector<double> radius_o = { scene.camera.lens_radius, scene.camera.lens_radius * 2 / 3, scene.camera.lens_radius / 3 }; // 25 points

	std::vector<Vector3d> real_offset = { Vector3d(0,0,0) }; // used to store all the offset
#ifdef BETTER
	for (const auto& r : radius_o) {
		for (const auto& p : offset) {
			real_offset.emplace_back(r * p.first, r * p.second, 0);
		}
	}
#endif

	for (unsigned i = 0; i < w; ++i) {
		std::cout << std::fixed << std::setprecision(2);
		std::cout << "Ray tracing: " << (100.0 * i) / w << "%\r" << std::flush;
		for (unsigned j = 0; j < h; ++j) {
			// TODO (Assignment 2, depth of field)
			Vector3d shift = grid_origin + (i+0.5)*x_displacement + (j+0.5)*y_displacement;
			Vector3d C(0, 0, 0);

			for (const auto& v : real_offset) {
				// Prepare the ray
				Ray ray;

				if (scene.camera.is_perspective) {
					// Perspective camera
					// TODO (Assignment 2, perspective camera)
					ray.origin = scene.camera.position + v;
					ray.direction = (shift - v).normalized();
				}
				else {
					// Orthographic camera
					ray.origin = scene.camera.position + Vector3d(shift[0], shift[1], 0);
					ray.direction = Vector3d(0, 0, -1);
				}

				int max_bounce = 5;
				C += shoot_ray(scene, ray, max_bounce);
			}
			C /= double(real_offset.size());

			R(i, j) = C(0);
			G(i, j) = C(1);
			B(i, j) = C(2);
			A(i, j) = 1;
		}
	}

	std::cout << "Ray tracing: 100%  " << std::endl;

	// Save to png
	const std::string filename("raytrace.png");
	write_matrix_to_png(R, G, B, A, filename);
}

////////////////////////////////////////////////////////////////////////////////

Scene load_scene(const std::string &filename) {
	Scene scene;

	// Load json data from scene file
	json data;
	std::ifstream in(filename);
	in >> data;

	// Helper function to read a Vector3d from a json array
	auto read_vec3 = [] (const json &x) {
		return Vector3d(x[0], x[1], x[2]);
	};

	// Read scene info
	scene.background_color = read_vec3(data["Scene"]["Background"]);
	scene.ambient_light = read_vec3(data["Scene"]["Ambient"]);

	// Read camera info
	scene.camera.is_perspective = data["Camera"]["IsPerspective"];
	scene.camera.position = read_vec3(data["Camera"]["Position"]);
	scene.camera.field_of_view = data["Camera"]["FieldOfView"];
	scene.camera.focal_length = data["Camera"]["FocalLength"];
	scene.camera.lens_radius = data["Camera"]["LensRadius"];

	// Read materials
	for (const auto &entry : data["Materials"]) {
		Material mat;
		mat.ambient_color = read_vec3(entry["Ambient"]);
		mat.diffuse_color = read_vec3(entry["Diffuse"]);
		mat.specular_color = read_vec3(entry["Specular"]);
		mat.reflection_color = read_vec3(entry["Mirror"]);
		mat.refraction_color = read_vec3(entry["Refraction"]);
		mat.refraction_index = entry["RefractionIndex"];
		mat.specular_exponent = entry["Shininess"];
		scene.materials.push_back(mat);
	}

	// Read lights
	for (const auto &entry : data["Lights"]) {
		Light light;
		light.position = read_vec3(entry["Position"]);
		light.intensity = read_vec3(entry["Color"]);
		scene.lights.push_back(light);
	}

	// Read objects
	for (const auto &entry : data["Objects"]) {
		ObjectPtr object;
		if (entry["Type"] == "Sphere") {
			auto sphere = std::make_shared<Sphere>();
			sphere->position = read_vec3(entry["Position"]);
			sphere->radius = entry["Radius"];
			object = sphere;
		} else if (entry["Type"] == "Parallelogram") {
			// TODO
			auto parallelogram = std::make_shared<Parallelogram>();
			parallelogram->origin = read_vec3(entry["Origin"]);
			parallelogram->u = read_vec3(entry["U"]);
			parallelogram->v = read_vec3(entry["V"]);
			object = parallelogram;
		} else if (entry["Type"] == "Mesh") {
			// Load mesh from a file
			std::string filename = std::string(DATA_DIR) + entry["Path"].get<std::string>();
			object = std::make_shared<Mesh>(filename);
		}
		object->material = scene.materials[entry["Material"]];
		scene.objects.push_back(object);
	}

	return scene;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {
	if (argc < 2) {
		std::cerr << "Usage: " << argv[0] << " scene.json" << std::endl;
		return 1;
	}
	Scene scene = load_scene(argv[1]);
	//Scene scene = load_scene(std::string(DATA_DIR) + "scene.json");
	render_scene(scene);
	return 0;


	/*
	// test intersect_box
	Ray ray(Vector3d(0, 0, 3), Vector3d(0, 0.51, -1));
	Vector3d a(1, 1, 1), b(-1, -1, -1);
	AlignedBox3d box;
	box.extend(a);
	box.extend(b);
	std::cout << intersect_box(ray, box) << std::endl;
	return 0;*/

	/*
	// test extent function
	Vector3d a(0, 3, 5), b(-1, 4, 4);
	std::cout << a << std::endl;
	Vector3d c = RowVector3d(1, 2, 3);
	std::cout << c << std::endl;
	Vector3d d = RowVector3d(2, 4, 5);
	AlignedBox3d box;
	box.extend(c.transpose());
	box.extend(d);
	return 0;*/

	/*
	// test AlignedBox3d
	AlignedBox3d box;
	box.extend(a);
	box.extend(b);
	std::cout << a.cwiseProduct(b) << std::endl;
	std::cout << box.max() << std::endl;
	std::cout << box.min() << std::endl;
	std::cout << box.corner(AlignedBox3d::BottomLeftFloor) << std::endl;
	std::cout << box.corner(AlignedBox3d::BottomRightFloor) << std::endl;
	std::cout << box.corner(AlignedBox3d::TopRightCeil) << std::endl;
	return 0;*/

	/*
	// test Mesh
	Mesh mesh(std::string(DATA_DIR) + "cube.off");
	std::cout << mesh.bvh.nodes.size() << std::endl;
	std::cout << mesh.vertices.rows() << std::endl;
	std::cout << mesh.facets.rows() << std::endl;
	return 0;*/	
}
