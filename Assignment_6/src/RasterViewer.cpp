#include "SDLViewer.h"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <functional>
#include <iostream>

#include "raster.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

#define PI 3.141592653589

#define DEBUG

using std::cout;
using std::endl;
using namespace Eigen;

enum class ModeType {Normal, Insert, Translate, Delete, Color} mode;

Vector4f colors[9];

//reload all triangles from uniform to vertices_lines and vertices_triangles
void reload_vertices(UniformAttributes& uniform, std::vector<VertexAttributes>& vertices_lines, std::vector<VertexAttributes>& vertices_triangles) {
    vertices_lines.clear();
    vertices_triangles.clear();
    for (unsigned i = 0; i < uniform.triangles.size(); ++i) {
        uniform.triangles[i].vertices[0].position[2] = uniform.triangles[i].vertices[1].position[2] = uniform.triangles[i].vertices[2].position[2] = i * 0.01;
        VertexAttributes v1 = uniform.triangles[i].vertices[0];
        VertexAttributes v2 = uniform.triangles[i].vertices[1];
        VertexAttributes v3 = uniform.triangles[i].vertices[2];

        //triangles
        vertices_triangles.push_back(v1);
        vertices_triangles.push_back(v2);
        vertices_triangles.push_back(v3);

        //wireframe of triangles
        v1.position[2] = v2.position[2] = v3.position[2] = v1.position[2] + 0.005; 
        v1.color = v2.color = v3.color = Vector4f(0, 0, 0, 1);
        vertices_lines.push_back(v1);
        vertices_lines.push_back(v2);
        vertices_lines.push_back(v1);
        vertices_lines.push_back(v3);
        vertices_lines.push_back(v2);
        vertices_lines.push_back(v3);
    }
}

void select_trianlge(UniformAttributes& uniform, std::vector<VertexAttributes>& vertices_lines, Vector4f& color) {
#ifdef DEBUG
    if (uniform.curTriangleIdx < 0) {
        cout << "[ERROR] index of array equals to -1." << endl;
        return;
    }
#endif // DEBUG
    for (int i = 0; i < 6; ++i) {
        vertices_lines[uniform.curTriangleIdx * 6 + i].color = color;
    }
}

void release_triangle(UniformAttributes& uniform, std::vector<VertexAttributes>& vertices_lines) {
#ifdef DEBUG
    if (uniform.curTriangleIdx < 0) {
        cout << "[ERROR] index of array equals to -1." << endl;
        return;
    }
#endif // DEBUG
    for (int i = 0; i < 6; ++i) {
        vertices_lines[uniform.curTriangleIdx * 6 + i].color << 0, 0, 0, 1;
    }
}

void start_moving_triangle(UniformAttributes& uniform, std::vector<VertexAttributes>& vertices_triangles, Vector4f& color) {
#ifdef DEBUG
    if (uniform.curTriangleIdx < 0) {
        cout << "[ERROR] index of array equals to -1." << endl;
        return;
    }
#endif // DEBUG
    for (int i = 0; i < 3; ++i) {
        vertices_triangles[uniform.curTriangleIdx * 3 + i].color = color;
    }    
}

void stop_moving_triangle(UniformAttributes& uniform, std::vector<VertexAttributes>& vertices_triangles, std::vector<VertexAttributes>& vertices_lines) {
#ifdef DEBUG
    if (uniform.curTriangleIdx < 0) {
        cout << "[ERROR] index of array equals to -1." << endl;
        return;
    }
#endif // DEBUG
    auto& tri = uniform.triangles[uniform.curTriangleIdx];
    for (int i = 0; i < 3; ++i) {
        vertices_triangles[uniform.curTriangleIdx * 3 + i].color = tri.vertices[i].color;
        tri.vertices[i].transformation = vertices_triangles[uniform.curTriangleIdx * 3 + i].transformation;
    }    
}

void move_triangle(UniformAttributes& uniform, std::vector<VertexAttributes>& vertices_triangles, std::vector<VertexAttributes>& vertices_lines, const float& dx, const float& dy) {
#ifdef DEBUG
    if (uniform.curTriangleIdx < 0) {
        cout << "[ERROR] index of array equals to -1." << endl;
        return;
    }
#endif // DEBUG
    Matrix4f temp;
    temp <<
        1, 0, 0, dx - uniform.start_position[0],
        0, 1, 0, dy - uniform.start_position[1],
        0, 0, 1, 0,
        0, 0, 0, 1;
    auto& mat = temp * uniform.triangles[uniform.curTriangleIdx].vertices[0].transformation;
    for(int i = 0; i < 3; ++i)
        vertices_triangles[uniform.curTriangleIdx * 3 + i].transformation = mat;
    for (int i = 0; i < 6; ++i)
        vertices_lines[uniform.curTriangleIdx * 6 + i].transformation = mat;
}

void quit_insertion(std::vector<VertexAttributes>& vertices_preview_lines) {
    vertices_preview_lines.clear();
}

void quit_translation(UniformAttributes& uniform, std::vector<VertexAttributes>& vertices_lines, std::vector<VertexAttributes>& vertices_triangles) {
    uniform.curTriangleIdx = -1;
    uniform.moving = false;
    reload_vertices(uniform, vertices_lines, vertices_triangles);
}

void quit_color(UniformAttributes& uniform, std::vector<VertexAttributes>& vertices_points) {
    vertices_points.clear();
    uniform.curVertexIdx = -1;
}

//rotate the selected triangle by alpha degree, counter-clockwise
void rotate_triangle(UniformAttributes& uniform, std::vector<VertexAttributes>& vertices_lines, std::vector<VertexAttributes>& vertices_triangles, double alpha) {
    auto& tri = uniform.triangles[uniform.curTriangleIdx];
    Vector4f barycenter = tri.vertices[0].transformation * tri.barycenter;
    Matrix4f trans = Matrix4f::Identity(), temp;
    alpha = alpha * PI / 180;
    temp << 
        1, 0, 0, -barycenter[0],
        0, 1, 0, -barycenter[1],
        0, 0, 1, 0,
        0, 0, 0, 1;
    trans = temp * trans;
    temp <<
        cos(alpha), -sin(alpha), 0, 0,
        sin(alpha), cos(alpha), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    trans = temp * trans;
    temp <<
        1, 0, 0, barycenter[0],
        0, 1, 0, barycenter[1],
        0, 0, 1, 0,
        0, 0, 0, 1;
    trans = temp * trans * tri.vertices[0].transformation;
    for (int i = 0; i < 3; ++i) {
        tri.vertices[i].transformation = trans;
        vertices_triangles[3 * uniform.curTriangleIdx + i].transformation = trans;
    }
    for (int i = 0; i < 6; ++i) {
        vertices_lines[6 * uniform.curTriangleIdx + i].transformation = trans;
    }
}

//scale the selected triangle by s time
void scale_triangle(UniformAttributes& uniform, std::vector<VertexAttributes>& vertices_lines, std::vector<VertexAttributes>& vertices_triangles, double s) {
    auto& tri = uniform.triangles[uniform.curTriangleIdx];
    Vector4f barycenter = tri.vertices[0].transformation * tri.barycenter;
    Matrix4f trans = Matrix4f::Identity(), temp;
    temp <<
        1, 0, 0, -barycenter[0],
        0, 1, 0, -barycenter[1],
        0, 0, 1, 0,
        0, 0, 0, 1;
    trans = temp * trans;
    temp <<
        s, 0, 0, 0,
        0, s, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    trans = temp * trans;
    temp <<
        1, 0, 0, barycenter[0],
        0, 1, 0, barycenter[1],
        0, 0, 1, 0,
        0, 0, 0, 1;
    trans = temp * trans * tri.vertices[0].transformation;
    for (int i = 0; i < 3; ++i) {
        tri.vertices[i].transformation = trans;
        vertices_triangles[3 * uniform.curTriangleIdx + i].transformation = trans;
    }
    for (int i = 0; i < 6; ++i) {
        vertices_lines[6 * uniform.curTriangleIdx + i].transformation = trans;
    }
}

Matrix4f get_scale_matrix(double s) {
    Matrix4f ret;
    ret << 
        s, 0, 0, 0,
        0, s, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    return ret;
}

Matrix4f get_translate_matrix(double dx, double dy) {
    Matrix4f ret;
    ret <<
        1, 0, 0, dx,
        0, 1, 0, dy,
        0, 0, 1, 0,
        0, 0, 0, 1;
    return ret;
}

int main(int argc, char *args[])
{
    colors[0] << 1, 0, 0, 1;
    colors[1] << 0, 1, 0, 1;
    colors[2] << 0, 0, 1, 1;
    colors[3] << 1, 1, 0, 1;
    colors[4] << 1, 0, 1, 1;
    colors[5] << 0, 1, 1, 1;
    colors[6] << 0, 0.5, 0.5, 1;
    colors[7] << 0.5, 0.5, 0, 1;
    colors[8] << 0.5, 0, 0.5, 1;

    int width = 1000;
    int height = 800;
    // The Framebuffer storing the image rendered by the rasterizer
	Eigen::Matrix<FrameBufferAttributes,Eigen::Dynamic,Eigen::Dynamic> frameBuffer(width, height);

	// Global Constants (empty in this example)
	UniformAttributes uniform;

	// Basic rasterization program
	Program program;

	// The vertex shader is the identity
	program.VertexShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
        VertexAttributes out(va);
        out.position = va.transformation * va.position;
        return out;
	};

	// The fragment shader uses a fixed color
	program.FragmentShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		FragmentAttributes out(va.color(0),va.color(1),va.color(2));
        out.position = va.position;
        return out;
	};

	// The blending shader converts colors between 0 and 1 to uint8
	program.BlendingShader = [](const FragmentAttributes& fa, const FrameBufferAttributes& previous)
	{
        if (fa.position[2] > previous.depth) {
		    FrameBufferAttributes out(fa.color[0]*255,fa.color[1]*255,fa.color[2]*255,fa.color[3]*255);
            out.depth = fa.position[2];
            return out;
        }
        return previous;
	};

    std::vector<VertexAttributes> vertices_lines;
    std::vector<VertexAttributes> vertices_triangles;

    std::vector<VertexAttributes> vertices_temp;
    std::vector<VertexAttributes> vertices_preview_lines;
    std::vector<VertexAttributes> vertices_points;    

    // Initialize the viewer and the corresponding callbacks
    SDLViewer viewer;
    viewer.init("Viewer Example", width, height);

    viewer.mouse_move = [&](int x, int y, int xrel, int yrel){
        float dx = (float(x) / float(width) * 2) - 1;
        float dy = (float(height - 1 - y) / float(height) * 2) - 1;

        if (mode == ModeType::Insert) {
            vertices_preview_lines.clear();
            VertexAttributes temp_pos(dx, dy, 1);
            if (vertices_temp.size() == 1) {
                vertices_preview_lines.push_back(vertices_temp[0]);
                vertices_preview_lines.push_back(temp_pos);
            }
            else if(vertices_temp.size() == 2){
                vertices_preview_lines.push_back(vertices_temp[0]);
                vertices_preview_lines.push_back(vertices_temp[1]);
                vertices_preview_lines.push_back(vertices_temp[0]);
                vertices_preview_lines.push_back(temp_pos);
                vertices_preview_lines.push_back(vertices_temp[1]);
                vertices_preview_lines.push_back(temp_pos);
            }
            viewer.redraw_next = true;
        }
        else if (mode == ModeType::Translate) {
            if (uniform.moving) {
                move_triangle(uniform, vertices_triangles, vertices_lines, dx, dy);
                viewer.redraw_next = true;
            }
        }
    };

    viewer.mouse_pressed = [&](int x, int y, bool is_pressed, int button, int clicks) {
        /*vertices[2].position << (float(x)/float(width) * 2) - 1, (float(height-1-y)/float(height) * 2) - 1, 0, 1;
        viewer.redraw_next = true;*/

        float dx = (float(x) / float(width) * 2) - 1;
        float dy = (float(height - 1 - y) / float(height) * 2) - 1;

        if (mode == ModeType::Insert && is_pressed) {
            if (button == 1) {
                vertices_temp.push_back(VertexAttributes(dx, dy, 1));
                if (vertices_temp.size() == 3) {
                    auto& v1 = vertices_temp[0];
                    auto& v2 = vertices_temp[1];
                    auto& v3 = vertices_temp[2];
                    v1.color = v2.color = v3.color = colors[0];
                    uniform.triangles.emplace_back(v1, v2, v3);

                    reload_vertices(uniform, vertices_lines, vertices_triangles);

                    vertices_temp.clear();
                    vertices_preview_lines.clear();
                }
            }
            else {
                vertices_temp.clear();
                vertices_preview_lines.clear();
            }
            viewer.redraw_next = true;
        }
        else if (mode == ModeType::Translate) {
            if (is_pressed) {
                //check all the triangle reversely
                for (int i = uniform.triangles.size() - 1; i >= 0; --i) {
                    auto& tri = uniform.triangles[i];
                    if (tri.isPointInTriangle(Vector2f(dx, dy))) {
                        if (uniform.curTriangleIdx != -1) {
                            release_triangle(uniform, vertices_lines);
                        }
                        uniform.moving = true;
                        uniform.curTriangleIdx = i;
                        uniform.start_position << dx, dy;
                        select_trianlge(uniform, vertices_lines, colors[1]);
                        start_moving_triangle(uniform, vertices_triangles, colors[2]);
                        viewer.redraw_next = true;
                        break;
                    }
                }
                //click on blank will remove the current selected status
                if (!uniform.moving && uniform.curTriangleIdx != -1) {
                    release_triangle(uniform, vertices_lines);
                    uniform.curTriangleIdx = -1;
                    viewer.redraw_next = true;
                }
            }
            else if (uniform.moving) {
                stop_moving_triangle(uniform, vertices_triangles, vertices_lines);
                uniform.moving = false;
                viewer.redraw_next = true;
            }
        }
        else if (mode == ModeType::Delete && is_pressed) {
            for (int i = uniform.triangles.size() - 1; i >= 0; --i) {
                auto& tri = uniform.triangles[i];
                if (tri.isPointInTriangle(Vector2f(dx, dy))) {
                    uniform.triangles.erase(uniform.triangles.begin() + i);
                    reload_vertices(uniform, vertices_lines, vertices_triangles);
                    viewer.redraw_next = true;
                    break;
                }
            }
        }
        else if (mode == ModeType::Color && is_pressed) {
            if (button == 1) {
                float minDis = FLT_MAX;
                Vector2f curPos(dx, dy);
                for (int i = 0; i < vertices_triangles.size(); ++i) {
                    auto& ver = vertices_triangles[i];
                    float dis = ((ver.transformation * ver.position).segment(0, 2) - curPos).squaredNorm();
                    if (dis < minDis) {
                        minDis = dis;
                        uniform.curVertexIdx = i;
                    }
                }
                if (uniform.curVertexIdx != -1) {
                    vertices_points.clear();
                    VertexAttributes v1(vertices_triangles[uniform.curVertexIdx]);
                    v1.color = colors[1];
                    v1.position[2] = 1;
                    vertices_points.push_back(v1);
                    vertices_points.push_back(v1);
                    viewer.redraw_next = true;
                }
            }
            else {
                uniform.curVertexIdx = -1;
                vertices_points.clear();
                viewer.redraw_next = true;
            }
        }
    };

    viewer.mouse_wheel = [&](int dx, int dy, bool is_direction_normal) {
    };

    viewer.key_pressed = [&](char key, bool is_pressed, int modifier, int repeat) {
        //cout << key << " " << is_pressed << " " << modifier << " " << repeat << endl;
        if (!is_pressed) {
            //change mode
            if (key == 'i' || key == 'o' || key == 'p' || key == 'c' || key == 'q') {
                //exit previous mode
                if (mode == ModeType::Insert) {
                    cout << "Exit from Insertion mode." << endl;
                    quit_insertion(vertices_preview_lines);
                }
                else if (mode == ModeType::Translate) {
                    cout << "Exit from Translation mode." << endl;
                    quit_translation(uniform, vertices_lines, vertices_triangles);
                }
                else if (mode == ModeType::Delete) {
                    cout << "Exit from Delete mode." << endl;
                }
                else if (mode == ModeType::Color) {
                    cout << "Exit from Color mode." << endl;
                    quit_color(uniform, vertices_points);
                }

                //enter new mode
                if (key == 'i') {            
                    mode = ModeType::Insert;
                    cout << "Enter Triangle Insertion mode." << endl;    
                    vertices_temp.clear();
                }
                else if (key == 'o') {
                    mode = ModeType::Translate;
                    cout << "Enter Triangle Translation mode." << endl;
                }
                else if (key == 'p') {
                    mode = ModeType::Delete;
                    cout << "Enter Delete mode." << endl;
                }
                else if (key == 'c') {
                    mode = ModeType::Color;
                    cout << "Enter Color mode." << endl;
                }
                else if (key == 'q') {
                    mode = ModeType::Normal;
                    cout << "Normal mode." << endl;
                }
                viewer.redraw_next = true;
            }
            else if (mode == ModeType::Translate && uniform.curTriangleIdx != -1) {
                if (key == 'h') {
                    rotate_triangle(uniform, vertices_lines, vertices_triangles, -10);
                    cout << "Rotated the trianlge by 10 degree clockwise." << endl;
                }
                else if (key == 'j') {
                    rotate_triangle(uniform, vertices_lines, vertices_triangles, 10);
                    cout << "Rotated the trianlge by 10 degree counter-clockwise." << endl;
                }
                else if (key == 'k') {
                    scale_triangle(uniform, vertices_lines, vertices_triangles, 1.25);
                    cout << "Scaled the trianlge up by 25%." << endl;
                }
                else if (key == 'l') {
                    scale_triangle(uniform, vertices_lines, vertices_triangles, 0.75);
                    cout << "Scaled the trianlge down by 25%." << endl;
                }
                viewer.redraw_next = true;
            }
            else if (mode == ModeType::Color && uniform.curVertexIdx != -1) {
                if (key >= '1' && key <= '9') {
                    auto& color = colors[key - '1'];
                    vertices_triangles[uniform.curVertexIdx].color = uniform.triangles[uniform.curVertexIdx / 3].vertices[uniform.curVertexIdx % 3].color = color;
                    viewer.redraw_next = true;
                }
            }         
            else if (key == '=' || key == '-' || key == 'w' || key == 'a' || key == 's' || key == 'd') {
                Matrix4f trans = Matrix4f::Identity();
                if (key == '=') {
                    cout << "Zoom in by 20% in the center of the screen." << endl;
                    trans = get_scale_matrix(1.2);
                }
                else if (key == '-') {
                    cout << "Zoom out by 20% in the center of the screen." << endl;
                    trans = get_scale_matrix(0.8);
                }
                else if (key == 'w') {
                    cout << "Translate the entire sence up by 20% of the window size." << endl;
                    trans = get_translate_matrix(0, -0.4);
                }
                else if (key == 's') {
                    cout << "Translate the entire sence down by 20% of the window size." << endl;
                    trans = get_translate_matrix(0, 0.4);
                }
                else if (key == 'a') {
                    cout << "Translate the entire sence left by 20% of the window size." << endl;
                    trans = get_translate_matrix(0.4, 0);
                }
                else if (key == 'd') {
                    cout << "Translate the entire sence right by 20% of the window size." << endl;
                    trans = get_translate_matrix(-0.4, 0);
                }

                for (auto& tri : uniform.triangles) {
                    tri.vertices[0].transformation = tri.vertices[1].transformation = tri.vertices[2].transformation = trans * tri.vertices[0].transformation;
                }
                reload_vertices(uniform, vertices_lines, vertices_triangles);
                viewer.redraw_next = true;
            }
        }
    };

    viewer.redraw = [&](SDLViewer &viewer) {
        // Clear the framebuffer
        for (unsigned i = 0; i < frameBuffer.rows(); i++) {
            for (unsigned j = 0; j < frameBuffer.cols(); j++) {
                frameBuffer(i,j).color << 255,255,255,255;
                frameBuffer(i, j).depth = -1;
            }
        }

        rasterize_triangles(program, uniform, vertices_triangles, frameBuffer);
        rasterize_lines(program, uniform, vertices_lines, 1.5, frameBuffer);
        if (mode == ModeType::Insert)
            rasterize_lines(program, uniform, vertices_preview_lines, 1.5, frameBuffer);
        if (mode == ModeType::Color)
            rasterize_lines(program, uniform, vertices_points, 4, frameBuffer);

        // Buffer for exchanging data between rasterizer and sdl viewer
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> R(width, height);
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> G(width, height);
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> B(width, height);
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> A(width, height);

        for (unsigned i=0; i<frameBuffer.rows();i++)
        {
            for (unsigned j=0; j<frameBuffer.cols();j++)
            {
                R(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(0);
                G(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(1);
                B(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(2);
                A(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(3);
            }
        }
        viewer.draw_image(R, G, B, A);
    };

    viewer.launch();

    return 0;
}
