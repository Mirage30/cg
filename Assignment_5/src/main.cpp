// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <fstream>

// Utilities for the Assignment
#include "raster.h"
#include <gif.h>

// Eigen for matrix operations
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

using namespace std;
using namespace Eigen;

#define PI 3.14159265358979323846

//#define EX1
#define WIREFRAME
//#define FLAT_SHADING
//#define PER_VERTEX_SHADING
#define ANIMATION
#define PERSPECTIVE

//resolution
const int width = 200;
const int height = 500;

// Read a triangle mesh from an off file
void load_off(const std::string& filename, MatrixXd& V, MatrixXi& F) {
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

Scene build_scene() {
	Scene scene;

	// Read scene info
	scene.background_color << 0.3, 0.3, 0.9;
	scene.ambient_light << 0.2, 0.2, 0.2;

	// Read camera info
#ifdef PERSPECTIVE
	scene.camera.is_perspective = true;
#endif // PERSPECTIVE
	scene.camera.position << 0., 0., 2.5;
	scene.camera.gaze = -scene.camera.position;
	scene.camera.viewup << 0., 1., 0;
	scene.camera.field_of_view = 0.7854;
	scene.camera.focal_length = 2.0;
	scene.camera.lens_radius = 0.08;
	scene.camera.aspect_ratio = 1.0;

	// Read material	
	// Material for bunny body part
	scene.material = 0;
	Material mat1;
	mat1.ambient_color << 0., 0.5, 0.;
	mat1.diffuse_color << 0.5, 0.5, 0.5;
	mat1.specular_color << 0.2, 0.2, 0.2;
	mat1.reflection_color << 0.7, 0.7, 0.7;
	mat1.refraction_color << 0., 0., 0.;
	mat1.refraction_index = 1.0;
	mat1.specular_exponent = 256.0;
	scene.materials.push_back(mat1);

	// Material for wire frame
	Material mat2;
	mat2.ambient_color << 1, 1, 1;
	mat2.diffuse_color << 0.5, 0.5, 0.5;
	mat2.specular_color << 0.2, 0.2, 0.2;
	mat2.reflection_color << 0.7, 0.7, 0.7;
	mat2.refraction_color << 0., 0., 0.;
	mat2.refraction_index = 1.0;
	mat2.specular_exponent = 256.0;
	scene.materials.push_back(mat2);

	// Read light
	Light light;
	light.position << 0.0, 0.0, 1.5;
	light.intensity << 1.0, 1.0, 1.0;
	scene.lights.push_back(light);

	return scene;
}

// Calculate by: sum(triangle_center * triangle_area) / total_area
Vector3d get_barycenter(MatrixXd& V, MatrixXi& F) {
	Vector3d barycenter(0, 0, 0);
	double total_area = 0;
	for (int i = 0; i < F.rows(); ++i) {
		Vector3d v0(V(F(i, 0), 0), V(F(i, 0), 1), V(F(i, 0), 2));
		Vector3d v1(V(F(i, 1), 0), V(F(i, 1), 1), V(F(i, 1), 2));
		Vector3d v2(V(F(i, 2), 0), V(F(i, 2), 1), V(F(i, 2), 2));
		Vector3d u = v1 - v0;
		Vector3d v = v2 - v0;
		double area = u.cross(v).norm();
		barycenter += area * (v0 + v1 + v2) / 3;
		total_area += area;
	}
	return barycenter / total_area;
}

void build_transformation(UniformAttributes& uniform) {
	Camera& cam = uniform.scene.camera;
	Vector3d w = -cam.gaze.normalized();
	Vector3d u = cam.viewup.cross(w).normalized();
	Vector3d v = w.cross(u);

	uniform.M_camera <<
		u[0], v[0], w[0], cam.position[0],
		u[1], v[1], w[1], cam.position[1],
		u[2], v[2], w[2], cam.position[2],
		0, 0, 0, 1;
	uniform.M_camera = uniform.M_camera.inverse().eval();	

	double n = -0.1;
	double f = -10.0;
	double t = cam.is_perspective ? tan(cam.field_of_view / 2) * abs(n) : 1;
	double b = -t;
	double r = t * cam.aspect_ratio;
	double l = -r;

	if (cam.is_perspective) {
		uniform.M_perspective <<
			n, 0, 0, 0,
			0, n, 0, 0,
			0, 0, n + f, -f * n,
			0, 0, 1, 0;
	}

	uniform.M_orthographic <<
		2 / (r - l), 0, 0, -(r + l) / (r - l),
		0, 2 / (t - b), 0, -(t + b) / (t - b),
		0, 0, 2 / (n - f), -(n + f) / (n - f),
		0, 0, 0, 1;

	uniform.transformation = uniform.M_orthographic * uniform.M_perspective * uniform.M_camera;
}

FrameBuffer scale_down_4x(const FrameBuffer& fb)
{
	// The size of the framebuffer must be a multiple of 4
	assert(fb.rows() % 4 == 0);
	assert(fb.cols() % 4 == 0);

	// Allocate the reduced buffer
	FrameBuffer out(fb.rows() / 4, fb.cols() / 4);

	for (unsigned i = 0; i < out.rows(); i++)
	{
		for (unsigned j = 0; j < out.cols(); j++)
		{
			Eigen::Vector4f avg = Eigen::Vector4f::Zero();
			for (unsigned ii = 0; ii < 4; ii++)
				for (unsigned jj = 0; jj < 4; jj++)
					avg += fb(i * 4 + ii, j * 4 + jj).color.cast<float>();
			avg /= 16;
			out(i, j).color = avg.cast<uint8_t>();
		}
	}
	return out;
}

int main() 
{
	//load mesh data
	string filename = string(DATA_DIR) + "bunny2.off";
	//string filename = string(DATA_DIR) + "cube.off";
	//string filename = string(DATA_DIR) + "bunny.off";

	// The Framebuffer storing the image rendered by the rasterizer
	Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> frameBuffer(width, height);

	// Global Constants (empty in this example)
	UniformAttributes uniform;
	uniform.scene = build_scene();

	// Basic rasterization program
	Program program;

	// The vertex shader is the identity
	program.VertexShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		VertexAttributes out(va);
		out.position = uniform.view * uniform.transformation * uniform.move * va.position;
		out.position /= out.position[3];

		Vector4f normal(va.normal[0], va.normal[1], va.normal[2], 0);
		normal = uniform.move.transpose().inverse() * normal;
		out.normal << normal[0], normal[1], normal[2];
		out.normal.normalize();
		return out;
	};

	// The fragment shader uses a fixed color
	program.FragmentShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		// Material for hit object
		const Material& mat = uniform.scene.materials[uniform.scene.material];

		// Ambient light contribution
		Vector3d ambient_color = mat.ambient_color.array() * uniform.scene.ambient_light.array();

		//Vector4f original_position = va.position;
		Vector4f original_position = uniform.transformation.inverse() * va.position;
		original_position /= original_position[3];

		Vector3d hit_position(original_position[0], original_position[1], original_position[2]);
		Vector3d ray_direction;
		if (uniform.scene.camera.is_perspective) {
			ray_direction = (hit_position - uniform.scene.camera.position).normalized();
		}
		else {
			ray_direction = uniform.scene.camera.gaze.normalized();
		}

		// Punctual lights contribution (direct lighting)
		Vector3d lights_color(0, 0, 0);
		for (const Light& light : uniform.scene.lights) {
			Vector3d Li = (light.position - hit_position).normalized();
			Vector3d N = va.normal;

			// Diffuse contribution
			Vector3d diffuse = mat.diffuse_color * std::max(Li.dot(N), 0.0);

			// TODO (Assignment 2, specular contribution)
			Vector3d h = (Li - ray_direction).normalized();
			Vector3d specular = mat.specular_color * pow(std::max(h.dot(N), 0.0), mat.specular_exponent);

			// Attenuate lights according to the squared distance to the lights
			Vector3d D = light.position - hit_position;
			lights_color += (diffuse + specular).cwiseProduct(light.intensity) / D.squaredNorm();
		}

		// Rendering equation
		Vector3d C = ambient_color + lights_color;
		for (int i = 0; i < 3; ++i)
			C[i] = min(C[i], 1.);

		FragmentAttributes out(C[0], C[1], C[2]);
		out.position = original_position;
		return out;
	};

	// The blending shader converts colors between 0 and 1 to uint8
	program.BlendingShader = [](const FragmentAttributes& fa, const FrameBufferAttributes& previous)
	{
		if (fa.position[2] > previous.depth - 0.01) {
			FrameBufferAttributes out(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
			out.depth = fa.position[2];
			return out;
		}
		return previous;
	};

	MatrixXd V; // n x 3 matrix (n points)
	MatrixXi F; // m x 3 matrix (m triangles)	
	load_off(filename, V, F);
	vector<VertexAttributes> vertices_triangles;
	vector<VertexAttributes> vertices_lines;

#ifdef EX1
	for (int i = 0; i < F.rows(); ++i) {
		for (int j = 0; j < F.cols(); ++j) {
			vertices_triangles.push_back(VertexAttributes(V(F(i, j), 0), V(F(i, j), 1), V(F(i, j), 2)));
		}
	}
#endif // EX1

#ifdef WIREFRAME
	for (int i = 0; i < F.rows(); ++i) {
		VertexAttributes v0(V(F(i, 0), 0), V(F(i, 0), 1), V(F(i, 0), 2));
		VertexAttributes v1(V(F(i, 1), 0), V(F(i, 1), 1), V(F(i, 1), 2));
		VertexAttributes v2(V(F(i, 2), 0), V(F(i, 2), 1), V(F(i, 2), 2));
		vertices_lines.push_back(v0);
		vertices_lines.push_back(v1);
		vertices_lines.push_back(v1);
		vertices_lines.push_back(v2);
		vertices_lines.push_back(v2);
		vertices_lines.push_back(v0);
	}
#endif // WIREFRAME

#ifdef FLAT_SHADING
	for (int i = 0; i < F.rows(); ++i) {
		VertexAttributes v0(V(F(i, 0), 0), V(F(i, 0), 1), V(F(i, 0), 2));
		VertexAttributes v1(V(F(i, 1), 0), V(F(i, 1), 1), V(F(i, 1), 2));
		VertexAttributes v2(V(F(i, 2), 0), V(F(i, 2), 1), V(F(i, 2), 2));

		Vector4f u = v1.position - v0.position;
		Vector4f v = v2.position - v0.position;
		Vector3d normal = Vector3d(u[0], u[1], u[2]).cross(Vector3d(v[0], v[1], v[2])).normalized();
		//normal = normal.dot(Vector3d(0, 0, 1)) > 0 ? normal : -normal;

		v0.normal = v1.normal = v2.normal = normal;

		vertices_triangles.push_back(v0);
		vertices_triangles.push_back(v1);
		vertices_triangles.push_back(v2);
	}
	for (int i = 0; i < F.rows(); ++i) {
		VertexAttributes v0(V(F(i, 0), 0), V(F(i, 0), 1), V(F(i, 0), 2));
		VertexAttributes v1(V(F(i, 1), 0), V(F(i, 1), 1), V(F(i, 1), 2));
		VertexAttributes v2(V(F(i, 2), 0), V(F(i, 2), 1), V(F(i, 2), 2));
		vertices_lines.push_back(v0);
		vertices_lines.push_back(v1);
		vertices_lines.push_back(v1);
		vertices_lines.push_back(v2);
		vertices_lines.push_back(v2);
		vertices_lines.push_back(v0);
	}
#endif // FLAT_SHADING

#ifdef PER_VERTEX_SHADING
	vector<VertexAttributes> tempVertices;
	for (int i = 0; i < V.rows(); ++i) {
		tempVertices.push_back(VertexAttributes(V(i, 0), V(i, 1), V(i, 2)));
	}
	for (int i = 0; i < F.rows(); ++i) {
		VertexAttributes& v0 = tempVertices[F(i, 0)];
		VertexAttributes& v1 = tempVertices[F(i, 1)];
		VertexAttributes& v2 = tempVertices[F(i, 2)];

		Vector4f u = v1.position - v0.position;
		Vector4f v = v2.position - v0.position;
		Vector3d normal = Vector3d(u[0], u[1], u[2]).cross(Vector3d(v[0], v[1], v[2])).normalized();
		//normal = normal.dot(Vector3d(0, 0, 1)) > 0 ? normal : -normal;

		v0.normal += normal;
		v1.normal += normal;
		v2.normal += normal;
	}
	for (int i = 0; i < tempVertices.size(); ++i) {
		tempVertices[i].normal.normalize();
	}
	for (int i = 0; i < F.rows(); ++i) {
		vertices_triangles.push_back(tempVertices[F(i, 0)]);
		vertices_triangles.push_back(tempVertices[F(i, 1)]);
		vertices_triangles.push_back(tempVertices[F(i, 2)]);
	}
#endif // Per-Vertex Shading

	build_transformation(uniform);

	float aspect_ratio = float(frameBuffer.cols()) / float(frameBuffer.rows());
	if (aspect_ratio < 1)
		uniform.view(0, 0) = aspect_ratio;
	else
		uniform.view(1, 1) = 1 / aspect_ratio;

	uniform.scene.material = 0;
	rasterize_triangles(program, uniform, vertices_triangles, frameBuffer);
	uniform.scene.material = 1;
	rasterize_lines(program, uniform, vertices_lines, 0.51, frameBuffer);

	vector<uint8_t> image;
	framebuffer_to_uint8(frameBuffer,image);
	stbi_write_png("triangle.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows()*4);
	

#ifdef ANIMATION
	const char* fileName = "triangle.gif";
	int delay = 25;
	const int frameNumber = 20;
	GifWriter g;
	GifBegin(&g, fileName, frameBuffer.rows(), frameBuffer.cols(), delay);

	uniform.barycenter = get_barycenter(V, F);
	float angle = PI * 2.0 / frameNumber;
	float distance = 1.0 / frameNumber;

	for (int i = 0; i < frameNumber; ++i)
	{
		frameBuffer.setConstant(FrameBufferAttributes());

		float d_angle = angle * i;
		uniform.move = Matrix4f::Identity(4, 4);
		Matrix4f temp;
		temp <<
			1, 0, 0, -uniform.barycenter[0],
			0, 1, 0, -uniform.barycenter[1],
			0, 0, 1, -uniform.barycenter[2],
			0, 0, 0, 1;
		uniform.move = temp * uniform.move;
		temp <<
			cos(d_angle), 0, -sin(d_angle), 0,
			0, 1, 0, 0,
			sin(d_angle), 0, cos(d_angle), 0,
			0, 0, 0, 1;
		uniform.move = temp * uniform.move;
		temp <<
			1, 0, 0, uniform.barycenter[0],
			0, 1, 0, uniform.barycenter[1],
			0, 0, 1, uniform.barycenter[2] + distance * i,
			0, 0, 0, 1;
		uniform.move = temp * uniform.move;

		uniform.scene.material = 0;
		rasterize_triangles(program, uniform, vertices_triangles, frameBuffer);
		uniform.scene.material = 1;
		rasterize_lines(program, uniform, vertices_lines, 0.51, frameBuffer);

		framebuffer_to_uint8(frameBuffer, image);
		GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
	}

	GifEnd(&g);	
#endif // ANIMATION
	return 0;
}
