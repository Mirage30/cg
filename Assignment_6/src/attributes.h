#pragma once

#include <Eigen/Core>

class VertexAttributes
{
	public:
	VertexAttributes(float x = 0, float y = 0, float z = 0, float w = 1)
	{
		position << x,y,z,w;
		color << 0,0,0,1;
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
        r.position = alpha*a.position + beta*b.position + gamma*c.position;
		r.color = alpha*a.color + beta*b.color + gamma*c.color;
        return r;
    }

	Eigen::Vector4f position;
	Eigen::Vector4f color;
	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
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
		depth = -1;
	}

	Eigen::Matrix<uint8_t,4,1> color;
	float depth;
};

class Triangle {
public:
	Eigen::Vector4f barycenter;
	std::vector<VertexAttributes> vertices;

	Triangle(VertexAttributes& v1, VertexAttributes& v2, VertexAttributes& v3) {
		vertices.push_back(v1);
		vertices.push_back(v2);
		vertices.push_back(v3);
		barycenter = (v1.position + v2.position + v3.position) / 3;
	}

	bool isPointInTriangle(const Eigen::Vector2f& point) const {
		if (vertices.size() != 3)
			return false;
		Eigen::Vector2f pa = (vertices[0].transformation * vertices[0].position).segment(0, 2) - point;
		Eigen::Vector2f pb = (vertices[1].transformation * vertices[1].position).segment(0, 2) - point;
		Eigen::Vector2f pc = (vertices[2].transformation * vertices[2].position).segment(0, 2) - point;
		float ab_cross = crossProduct(pa, pb);
		float bc_cross = crossProduct(pb, pc);
		float ca_cross = crossProduct(pc, pa);
		if (ab_cross * bc_cross >= 0 && bc_cross * ca_cross >= 0)
			return true;
		return false;
	}

	static float crossProduct(const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
		return a[0] * b[1] - a[1] * b[0];
	}
};

class UniformAttributes
{
public:
	bool moving = false;
	int curTriangleIdx = -1;
	int curVertexIdx = -1;
	Eigen::Vector2f start_position;
	std::vector<Triangle> triangles;
};