// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f v2_1(_v[1].x() - _v[0].x(), _v[1].y() - _v[0].y(), 0);
    Vector3f v3_2(_v[2].x() - _v[1].x(), _v[2].y() - _v[1].y(), 0);
    Vector3f v1_3(_v[0].x() - _v[2].x(), _v[0].y() - _v[2].y(), 0);
    Vector3f p1(x-_v[0].x(), y-_v[0].y(), 0);
    Vector3f p2(x-_v[1].x(), y-_v[1].y(), 0);
    Vector3f p3(x-_v[2].x(), y-_v[2].y(), 0);
    float crs1 = v2_1.cross(p1).z();
    float crs2 = v3_2.cross(p2).z();
    float crs3 = v1_3.cross(p3).z();
    
    if( ( (crs1 >=0) && (crs2 >= 0) && (crs3 >= 0) ) ||  ( (crs1 < 0) && (crs2 < 0) && (crs3 < 0) ) )return true;    
    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        // for(int i=0;i<3;i++){
        //     std::cout << "VertexBefore " << i << " : " <<  v[i].x() << " " << v[i].y() << " " << v[i].z() << " " << v[i].w() << std::endl;
        // }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }
        // for(int i=0;i<3;i++){
        //     std::cout << "Vertex " << i << " : " <<  v[i].x() << " " << v[i].y() << " " << v[i].z() << " " << v[i].w() << std::endl;
        // }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);

    }
    #ifdef USE_SUPERSAMPLING
    for(int j=0;j<height*ssrate;j++){
        for(int i=0;i<width*ssrate;i++){          
            frame_buf[(int)(i/ssrate)+(int)(j/ssrate)*width] += ss_frame_buf[i+j*width*ssrate]/(float)(ssrate*ssrate);
        }
    }  
    #endif
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.

    const Vector3f color = t.getColor();
    auto v = t.toVector4();
    float minX = width+1, minY = height+1, maxX = -1.0, maxY = -1.0;
    for(int i=0;i<3;i++){
        if(t.v[i][0]<minX)
            minX = t.v[i][0];
        if(t.v[i][0]>maxX)
            maxX = t.v[i][0];
        if(t.v[i][1]<minY)
            minY = t.v[i][1];
        if(t.v[i][1]>maxY)
            maxY = t.v[i][1];            
    }
    maxX = std::ceil(maxX);
    maxY = std::ceil(maxY);
    float xptr,yptr;
    

#ifdef USE_SUPERSAMPLING
    float step = 1.0/(float)ssrate;
    xptr = std::floor(minX) + 0.5*step;
    yptr = std::floor(minY) + 0.5*step;
    while(yptr < maxY){
        float nextX = std::ceil(xptr);
        float nextY = std::ceil(yptr);
        if(insideTriangle(xptr, yptr, t.v)){
            auto[alpha, beta, gamma] = computeBarycentric2D(xptr, yptr, t.v);
            float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            z_interpolated *= -w_reciprocal;
            int index = get_ss_index(xptr, yptr); 
            if(z_interpolated < ss_depth_buf[index]){
                ss_depth_buf[index] = z_interpolated;
                ss_frame_buf[index] = color;
            }
        }
        xptr += step;
        
        if(xptr > maxX){
            xptr = std::floor(minX) + 0.5*step;
            yptr += step;
        }
    }
    return;
#endif
    
    xptr = std::floor(minX) + 0.5;
    yptr = std::floor(minY) + 0.5;

    while(yptr < maxY){
        if(insideTriangle(xptr,yptr,t.v)){
            auto[alpha, beta, gamma] = computeBarycentric2D(xptr, yptr, t.v);
            float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            z_interpolated *= -w_reciprocal;
            int index = get_index(xptr, yptr);
            if(z_interpolated < depth_buf[index]){
                depth_buf[index] = z_interpolated;
                set_pixel(Vector3f(int(xptr),int(yptr),0),color);
            }
        }        
        xptr += 1;
        if(xptr > maxX){
            xptr = std::floor(minX) + 0.5;
            yptr += 1;
        }
    }     
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    #ifdef USE_SUPERSAMPLING
        std::fill(ss_frame_buf.begin(), ss_frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    #endif
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    #ifdef USE_SUPERSAMPLING
        std::fill(ss_depth_buf.begin(), ss_depth_buf.end(), std::numeric_limits<float>::infinity());
    #endif
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    #ifdef USE_SUPERSAMPLING
        ssrate = SSRATE;
        ss_frame_buf.resize(w * h * ssrate * ssrate);
        ss_depth_buf.resize(w * h * ssrate * ssrate);
    #endif
}

#ifdef USE_SUPERSAMPLING
inline int rst::rasterizer::get_ss_index(float x, float y)
{
    return (y-0.5/(1.0*ssrate))*ssrate*ssrate*width + (x-0.5/(1.0*ssrate))*ssrate;
}
#endif

int rst::rasterizer::get_index(int x, int y)
{
    return y*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    auto ind = get_index(point.x(),point.y());
    frame_buf[ind] = color;

}

// clang-format on