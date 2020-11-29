#pragma once

#include <Eigen/Core>

class VertexAttributes
{
	public:
	VertexAttributes(float x = 0, float y = 0, float z = 0, float w = 1)
	{
		position << x,y,z,w;
		normal << 0, 0, 0;
	}

    // Interpolates the vertex attributes
    static VertexAttributes interpolate(
        const VertexAttributes& a,
        const VertexAttributes& b,
        const VertexAttributes& c,
        const float alpha, 
        const float beta, 
        const float gamma
    ) 
    {
        VertexAttributes r;
		r.position = alpha * a.position + beta * b.position + gamma * c.position;
		r.normal = (alpha * a.normal + beta * b.normal + gamma * c.normal).normalized();
        return r;
    }

	Eigen::Vector4f position;
	Eigen::Vector3d normal;
};

class FragmentAttributes
{
	public:
	FragmentAttributes(float r = 0, float g = 0, float b = 0, float a = 1)
	{
		color << r,g,b,a;
	}

	Eigen::Vector4f color;
	Eigen::Vector4f position;
};

class FrameBufferAttributes
{
	public:
	FrameBufferAttributes(uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, uint8_t a = 255)
	{
		color << r,g,b,a;
		depth = -2; // this value should be between -1 and 1, 2 is further than the visible range
	}

	Eigen::Matrix<uint8_t,4,1> color;
	float depth;
};


struct Light {
	Eigen::Vector3d position;
	Eigen::Vector3d intensity;
};

struct Camera {
	bool is_perspective = false;
	Eigen::Vector3d position;
	Eigen::Vector3d gaze;
	Eigen::Vector3d viewup;
	double field_of_view; // between 0 and PI
	double focal_length;
	double lens_radius; // for depth of field
	double aspect_ratio;
};

struct Material {
	Eigen::Vector3d ambient_color;
	Eigen::Vector3d diffuse_color;
	Eigen::Vector3d specular_color;
	double specular_exponent; // Also called "shininess"

	Eigen::Vector3d reflection_color;
	Eigen::Vector3d refraction_color;
	double refraction_index;
};

struct Scene {
	Eigen::Vector3d background_color;
	Eigen::Vector3d ambient_light;

	Camera camera;
	int material = -1;
	std::vector<Material> materials;
	std::vector<Light> lights;
};


class UniformAttributes
{
public:
	Scene scene;

	Eigen::Vector3d barycenter;
	Eigen::Matrix4f move = Eigen::Matrix4f::Identity(4, 4);

	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity(4, 4);
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity(4, 4);
	Eigen::Matrix4f M_camera = Eigen::Matrix4f::Identity(4, 4);
	Eigen::Matrix4f M_orthographic = Eigen::Matrix4f::Identity(4, 4);
	Eigen::Matrix4f M_perspective = Eigen::Matrix4f::Identity(4, 4);
};