////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <gif.h>

// Eigen for matrix operations
#include <Eigen/Dense>

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
	virtual void changePosition(Vector3d newPos) = 0;
};

// We use smart pointers to hold objects as this is a virtual class
typedef std::shared_ptr<Object> ObjectPtr;

struct Sphere : public Object {
	Vector3d position;
	double radius;

	virtual ~Sphere() = default;
	virtual bool intersect(const Ray &ray, Intersection &hit) override;
	virtual void changePosition(Vector3d newPos) override;
};

struct Parallelogram : public Object {
	Vector3d origin;
	Vector3d u;
	Vector3d v;

	virtual ~Parallelogram() = default;
	virtual bool intersect(const Ray &ray, Intersection &hit) override;
	virtual void changePosition(Vector3d newPos) override;
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

bool Sphere::intersect(const Ray &ray, Intersection &hit) {
	// TODO:
	// Compute the intersection between the ray and the sphere
	// If the ray hits the sphere, set the result of the intersection in the
	// struct 'hit'
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

void Sphere::changePosition(Vector3d newPos) {
	position = newPos;
}

bool Parallelogram::intersect(const Ray &ray, Intersection &hit) {
	// TODO
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
	if (x(2) < 0 || x(0) < 0 || x(0) > 1 || x(1) < 0 || x(1) > 1)
		return false;
	hit.ray_param = x(2);
	hit.position = ray.origin + ray.direction * hit.ray_param;
	hit.normal = u.cross(v).dot(ray.direction) < 0 ? u.cross(v) : -u.cross(v);
	hit.normal.normalize();
	return true;
}

void Parallelogram::changePosition(Vector3d newPos) {
	origin = newPos;
}

////////////////////////////////////////////////////////////////////////////////
// Define ray-tracing functions
////////////////////////////////////////////////////////////////////////////////

// Function declaration here (could be put in a header file)
Vector3d ray_color(const Scene &scene, const Ray &ray, const Object &object, const Intersection &hit, int max_bounce);
Object * find_nearest_object(const Scene &scene, const Ray &ray, Intersection &closest_hit);
bool is_light_visible(const Scene &scene, const Intersection& hit, const Light &light);
Vector3d shoot_ray(const Scene &scene, const Ray &ray, int max_bounce);

// -----------------------------------------------------------------------------

// eta = 1/e = sin(theta2)/sin(theta1) = n2/n1 = n2 (if n1 is vacuum)
bool refract(const Vector3d& ray_direction, const Vector3d& normal, const double &eta, Vector3d &refract) {
	double cost1 = -ray_direction.dot(normal);
	double cost2 = 1. - eta * eta * (1. - cost1 * cost1);
	if (cost2 <= 0)
		return false; //total internal reflection
	refract = (eta * ray_direction + normal * (eta * cost1 - sqrt(cost2))).normalized();
	return true;
}

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

		// TODO: Shoot a shadow ray to determine if the light should affect the intersection point
		if (!is_light_visible(scene, hit, light))
			continue;

		// Diffuse contribution
		Vector3d diffuse = mat.diffuse_color * std::max(Li.dot(N), 0.0);

		// TODO: Specular contribution
		Vector3d h = (Li - ray.direction).normalized();
		Vector3d specular = mat.specular_color * pow(std::max(h.dot(N), 0.0), mat.specular_exponent);

		// Attenuate lights according to the squared distance to the lights
		Vector3d D = light.position - hit.position;
		lights_color += (diffuse + specular).cwiseProduct(light.intensity) /  D.squaredNorm();
	}

	// TODO: Compute the color of the reflected ray and add its contribution to the current point color.
	Vector3d reflection_color(0, 0, 0);
	if (mat.reflection_color != Vector3d(0, 0, 0) && max_bounce > 0) {
		Ray reflect_ray;
		reflect_ray.direction = ray.direction - 2 * (ray.direction.dot(hit.normal)) * hit.normal;
		reflect_ray.origin = hit.position + EPSILON * reflect_ray.direction;
		reflection_color = mat.reflection_color.cwiseProduct(shoot_ray(scene, reflect_ray, max_bounce - 1));
	}

	// TODO: Compute the color of the refracted ray and add its contribution to the current point color.
	//       Make sure to check for total internal reflection before shooting a new ray.
	Vector3d refraction_color(0, 0, 0);
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

	// Rendering equation
	Vector3d C = ambient_color + lights_color + reflection_color + refraction_color;

	return C;
}

// -----------------------------------------------------------------------------

Object * find_nearest_object(const Scene &scene, const Ray &ray, Intersection &closest_hit) {
	int closest_index = -1;

	// TODO: 
	// Find the object in the scene that intersects the ray first
	// The function must return 'nullptr' if no object is hit, otherwise it must
	// return a pointer to the hit object, and set the parameters of the argument
	// 'hit' to their expected values.
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
	// TODO: Determine if the light is visible here
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

bool write_to_gif = false; // generate gif
Vector3d start_position(1, 0.2, -5);
Vector3d end_position(0, 0, 4);
int frame_number = 10;

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
	double scale_y = tan(scene.camera.field_of_view / 2.) * scene.camera.focal_length; // TODO: Stretch the pixel grid by the proper amount here
	double scale_x = scale_y * aspect_ratio; // TODO

	// The pixel grid through which we shoot rays is at a distance 'focal_length'
	// from the sensor, and is scaled from the canonical [-1,1] in order
	// to produce the target field of view.
	Vector3d grid_origin(-scale_x, scale_y, -scene.camera.focal_length);
	Vector3d x_displacement(2.0/w*scale_x, 0, 0);
	Vector3d y_displacement(0, -2.0/h*scale_y, 0);

	// generate many ray to implement depth of field
	const double tmp = sqrt(0.5);
	const std::vector<std::pair<double, double>> offset = { {0,1},{0,-1},{1,0},{-1,0},{tmp, tmp},{-tmp, tmp},{tmp, -tmp},{-tmp, -tmp} }; // relative position
	//const std::vector<double> radius_o = { scene.camera.lens_radius }; // 9 points
	const std::vector<double> radius_o = { scene.camera.lens_radius, scene.camera.lens_radius / 2 }; // 17 points
	//const std::vector<double> radius_o = { scene.camera.lens_radius, scene.camera.lens_radius * 2 / 3, scene.camera.lens_radius / 3 }; // 25 points
	
	std::vector<Vector3d> real_offset = { Vector3d(0,0,0) }; // used to store all the offset
	for (const auto& r : radius_o) {
		for (const auto& p : offset) {
			real_offset.emplace_back(r * p.first, r * p.second, 0);
		}
	}

	for (unsigned i = 0; i < w; ++i) {
		for (unsigned j = 0; j < h; ++j) {
			// TODO: Implement depth of field
			Vector3d shift = grid_origin + (i+0.5)*x_displacement + (j+0.5)*y_displacement;
			Vector3d C(0, 0, 0);
			for (const auto& v : real_offset) {
				// Prepare the ray
				Ray ray;

				if (scene.camera.is_perspective) {
					// TODO: Perspective camera
					ray.origin = scene.camera.position + v;
					ray.direction = (shift - v).normalized();
				} else {
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

	// Save to png
	const std::string filename("raytrace.png");
	write_matrix_to_png(R, G, B, A, filename);

	//write gif
	if (write_to_gif) {
		const char* fileName = "out.gif";
		std::vector<uint8_t> image;
		int delay = 25; // Milliseconds to wait between frames
		GifWriter g;
		GifBegin(&g, fileName, R.rows(), R.cols(), delay);
		
		scene.objects[3]->changePosition(start_position);
		Vector3d interval = (end_position - start_position) / frame_number;

		for (unsigned i = 0; i <= frame_number; i++)
		{
			// Generate R G B A matrices
			// change object's position
			scene.objects[3]->changePosition(start_position + i * interval);

			for (unsigned i = 0; i < w; ++i) {
				for (unsigned j = 0; j < h; ++j) {
					// TODO: Implement depth of field
					Vector3d shift = grid_origin + (i + 0.5) * x_displacement + (j + 0.5) * y_displacement;
					Vector3d C(0, 0, 0);
					for (const auto& v : real_offset) {
						// Prepare the ray
						Ray ray;

						if (scene.camera.is_perspective) {
							// TODO: Perspective camera
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

			write_matrix_to_uint8(R, G, B, A, image);
			GifWriteFrame(&g, image.data(), R.rows(), R.cols(), delay);
		}
		GifEnd(&g);
	}
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
	//Scene scene = load_scene("../../../data/scene.json"); //test
	render_scene(scene);
	return 0;
}
