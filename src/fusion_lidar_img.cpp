/*  Copyright (c) 2023, Beamagine

    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification,
    are permitted provided that the following conditions are met:

        - Redistributions of source code must retain the above copyright notice,
          this list of conditions and the following disclaimer.
        - Redistributions in binary form must reproduce the above copyright notice,
          this list of conditions and the following disclaimer in the documentation and/or
          other materials provided with the distribution.
        - Neither the name of copyright holders nor the names of its contributors may be
          used to endorse or promote products derived from this software without specific
          prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
    MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
    EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
    HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
    TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
    EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/point_cloud_conversion.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <boost/numeric/ublas/matrix.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

typedef struct ColorPoint
{
    cv::Point point;
    cv::Vec3b color;
} ColorPoint;

typedef enum visualizationTypes
{
    WHITE = 0,
    INTENSITY,
    RGB,
    RAINBOW,
    RAINBOW_Z,
} visualizationTypes;

std::map<float, cv::Mat> m_img_buffer;

cv::Mat m_camera_matrix_cv;
cv::Mat m_distortion_coef;
boost::numeric::ublas::matrix<float> m_projection_matrix;
boost::numeric::ublas::matrix<float> m_camera_intrinsic_matrix;
boost::numeric::ublas::matrix<float> m_homogeneous_matrix;

sensor_msgs::msg::PointCloud m_point_cloud;

std::string m_lidar_sensor_name;
std::string m_img_sensor_name;

int m_img_width, m_img_height;
int m_max_buffer_size = 5;

int MAX_COLOR_LEVEL;
int **R;
int **G;
int **B;

void initializeColors()
{
    MAX_COLOR_LEVEL = 7;

    R = new int *[2];
    R[0] = new int[7];
    R[1] = new int[7];

    G = new int *[2];
    G[0] = new int[7];
    G[1] = new int[7];

    B = new int *[2];
    B[0] = new int[7];
    B[1] = new int[7];

    // DISTANCE
    R[0][0] = 0;
    G[0][0] = 176;
    B[0][0] = 176;
    R[0][1] = 100;
    G[0][1] = 189;
    B[0][1] = 85;
    R[0][2] = 200;
    G[0][2] = 211;
    B[0][2] = 1;
    R[0][3] = 220;
    G[0][3] = 160;
    B[0][3] = 0;
    R[0][4] = 242;
    G[0][4] = 134;
    B[0][4] = 0;
    R[0][5] = 231;
    G[0][5] = 70;
    B[0][5] = 20;
    R[0][6] = 226;
    G[0][6] = 3;
    B[0][6] = 47;

    // INTENSITY
    R[1][0] = 125;
    G[1][0] = 0;
    B[1][0] = 255;
    R[1][1] = 0;
    G[1][1] = 125;
    B[1][1] = 255;
    R[1][2] = 0;
    G[1][2] = 255;
    B[1][2] = 255;
    R[1][3] = 125;
    G[1][3] = 255;
    B[1][3] = 255;
    R[1][4] = 255;
    G[1][4] = 255;
    B[1][4] = 255;
    R[1][5] = 255;
    G[1][5] = 125;
    B[1][5] = 255;
    R[1][6] = 0;
    G[1][6] = 0;
    B[1][6] = 125;
}

int getSensorsInFile(std::string file_name)
{
    setlocale(LC_NUMERIC, "C"); // This is necessary so that it reads decimals as '.' and not ','

    // Find the sensors in the file
    std::ifstream file(file_name.c_str(), std::ios::out);

    if (!file.is_open())
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "ERROR: Could not open file " << file_name);
        return 1;
    }

    bool max_sensors = false;
    bool sensor_found = false;

    std::string line = "";

    while (!max_sensors)
    {
        getline(file, line);
        std::string name = "";

        if (line.find("INTRINSICS") != std::string::npos)
        { // contains "INTRINSICS"
            max_sensors = true;
        }
        else
        {
            if (line.find("Sensor ID 1") != std::string::npos)
            {
                std::istringstream f(line);
                getline(f, name, '\t');
                getline(f, name, '\n');
                m_lidar_sensor_name = name;
            }
            else if (line.find("Sensor ID") != std::string::npos)
            {
                std::istringstream f(line);
                getline(f, name, '\t');
                getline(f, name);

                if (name == m_img_sensor_name)
                {
                    sensor_found = true;
                    break;
                }
            }
        }
    }

    file.close();

    if (!sensor_found)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "ERROR: Sensor name " << m_img_sensor_name << " not found in matrix file");
        return 2;
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Getting " << m_lidar_sensor_name << " lidar matrix");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Getting " << m_img_sensor_name << " sensor matrix");

    return 0;
}

int getFusionMatrix(std::string file_name, std::string sensor_name, cv::Mat &camera_matrix_cv, cv::Mat &distortion_coef, boost::numeric::ublas::matrix<float> &camera_projection,
                    boost::numeric::ublas::matrix<float> &camera_intrinsics, boost::numeric::ublas::matrix<float> &lidar_homogeneous)
{
    setlocale(LC_NUMERIC, "C"); // This is necessary so that it reads decimals as '.' and not ','

    std::string intrinsic_sensor_line = "----- " + sensor_name + " INTRINSICS -----";

    std::string fov_specs_line = "----- FOV SPECS -----";

    std::string intrinsics_line = "----- CAMERA INTRINSIC MATRIX [fx 0 0; s fy 0; cx cy 1] -----";

    std::string distortion_line = "----- CAMERA DISTORTION VECTOR [k1 k2 p1 p2 k3] -----";

    std::string fusion_matrix_line = "----- " + m_lidar_sensor_name + " TO " + sensor_name + " HOMOGENEOUS MATRIX -----";

    std::ifstream file(file_name.c_str(), std::ios::out);

    if (!file.is_open())
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "ERROR: Could not open file" << file_name);
        return 1;
    }

    bool inside_sensor_section = false;
    bool inside_homogeneous_section = false;

    std::string line;
    while (getline(file, line))
    {

        std::vector<float> matrix_values;
        matrix_values.clear();

        if (line == intrinsic_sensor_line)
        {
            inside_sensor_section = true;
        }
        if (line == fusion_matrix_line)
        {
            inside_homogeneous_section = true;
        }

        if (inside_sensor_section)
        {
            if (line == fov_specs_line)
            {
                getline(file, line);
                getline(file, line);
                std::string height = "";
                std::istringstream f(line);
                getline(f, height, '\t');
                getline(f, height, '\n');
                m_img_height = stoi(height);

                getline(file, line);
                std::string width = "";
                f.str(line);
                f.clear();
                getline(f, width, '\t');
                getline(f, width, '\n');
                m_img_width = stoi(width);
            }
            if (line == intrinsics_line)
            {
                getline(file, line);
                matrix_values.clear();
                std::string value_str;
                std::istringstream f(line);
                while (getline(f, value_str, '\t'))
                {
                    matrix_values.push_back(std::stof(value_str));
                }
                camera_intrinsics(0, 0) = matrix_values[0];
                camera_intrinsics(0, 1) = matrix_values[1];
                camera_intrinsics(0, 2) = matrix_values[2];

                getline(file, line);
                matrix_values.clear();
                f.str(line);
                f.clear();
                while (getline(f, value_str, '\t'))
                {
                    matrix_values.push_back(std::stof(value_str));
                }
                camera_intrinsics(1, 0) = matrix_values[0];
                camera_intrinsics(1, 1) = matrix_values[1];
                camera_intrinsics(1, 2) = matrix_values[2];

                getline(file, line);
                matrix_values.clear();
                f.str(line);
                f.clear();
                while (getline(f, value_str, '\t'))
                {
                    matrix_values.push_back(std::stof(value_str));
                }
                camera_intrinsics(2, 0) = matrix_values[0];
                camera_intrinsics(2, 1) = matrix_values[1];
                camera_intrinsics(2, 2) = matrix_values[2];

                camera_matrix_cv.row(0).col(0) = camera_intrinsics(0, 0);
                camera_matrix_cv.row(0).col(1) = 0;
                camera_matrix_cv.row(0).col(2) = camera_intrinsics(2, 0);
                camera_matrix_cv.row(1).col(0) = camera_intrinsics(0, 1);
                camera_matrix_cv.row(1).col(1) = camera_intrinsics(1, 1);
                camera_matrix_cv.row(1).col(2) = camera_intrinsics(2, 1);
                camera_matrix_cv.row(2).col(0) = camera_intrinsics(0, 2);
                camera_matrix_cv.row(2).col(1) = camera_intrinsics(1, 2);
                camera_matrix_cv.row(2).col(2) = camera_intrinsics(2, 2);
            }
            if (line == distortion_line)
            {
                getline(file, line);
                matrix_values.clear();
                std::string value_str;
                std::istringstream f(line);
                while (getline(f, value_str, '\t'))
                {
                    matrix_values.push_back(std::stof(value_str));
                }
                distortion_coef.row(0).col(0) = matrix_values[0];
                distortion_coef.row(0).col(1) = matrix_values[1];
                distortion_coef.row(0).col(2) = matrix_values[2];
                distortion_coef.row(0).col(3) = matrix_values[3];
                distortion_coef.row(0).col(4) = matrix_values[4];

                // After reading the distortion vector we finish the sensor section
                inside_sensor_section = false;
            }
        }
        if (inside_homogeneous_section)
        {

            getline(file, line);
            std::string value_str;
            std::istringstream f(line);
            while (getline(f, value_str, '\t'))
            {
                matrix_values.push_back(std::stof(value_str));
            }
            lidar_homogeneous(0, 0) = matrix_values[0];
            lidar_homogeneous(0, 1) = matrix_values[1];
            lidar_homogeneous(0, 2) = matrix_values[2];

            getline(file, line);
            matrix_values.clear();
            f.str(line);
            f.clear();
            while (getline(f, value_str, '\t'))
            {
                matrix_values.push_back(std::stof(value_str));
            }
            lidar_homogeneous(1, 0) = matrix_values[0];
            lidar_homogeneous(1, 1) = matrix_values[1];
            lidar_homogeneous(1, 2) = matrix_values[2];

            getline(file, line);
            matrix_values.clear();
            f.str(line);
            f.clear();
            while (getline(f, value_str, '\t'))
            {
                matrix_values.push_back(std::stof(value_str));
            }
            lidar_homogeneous(2, 0) = matrix_values[0];
            lidar_homogeneous(2, 1) = matrix_values[1];
            lidar_homogeneous(2, 2) = matrix_values[2];

            getline(file, line);
            matrix_values.clear();
            f.str(line);
            f.clear();
            while (getline(f, value_str, '\t'))
            {
                matrix_values.push_back(std::stof(value_str));
            }
            lidar_homogeneous(3, 0) = matrix_values[0];
            lidar_homogeneous(3, 1) = matrix_values[1];
            lidar_homogeneous(3, 2) = matrix_values[2];

            inside_homogeneous_section = false;
        }
    }

    // multiply the matrices when finish reading --> H_LIDAR2CAM * K_CAM
    camera_projection = boost::numeric::ublas::prod(lidar_homogeneous, camera_intrinsics);

    file.close();

    return 0;
}

int readFusionMatrix(std::string file_name)
{
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Reading matrix from " << file_name);

    int error = 0;

    // FUSION MATRIX
    m_camera_matrix_cv = cv::Mat(3, 3, CV_32F);
    m_distortion_coef = cv::Mat(1, 5, CV_32F);
    m_projection_matrix.resize(4, 3);
    m_camera_intrinsic_matrix.resize(3, 3);
    m_homogeneous_matrix.resize(4, 3);

    error = getSensorsInFile(file_name);

    if (error)
        return error;

    error = getFusionMatrix(file_name, m_img_sensor_name, m_camera_matrix_cv, m_distortion_coef, m_projection_matrix, m_camera_intrinsic_matrix, m_homogeneous_matrix);

    return error;
}

void printSensorMatrices(const std::string sensor_name, const cv::Mat camera_matrix_cv, const cv::Mat distortion_coef, const boost::numeric::ublas::matrix<float> camera_projection,
                         const boost::numeric::ublas::matrix<float> camera_intrinsics, const boost::numeric::ublas::matrix<float> homogeneous)
{
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\n---------------------------------------- " << sensor_name << " FUSION MATRICES ----------------------------------------\n");

    std::string spacer = "\t";

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Image size (" << m_img_height << ", " << m_img_width << ')');

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "CV camera matrix:");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t[ " << camera_matrix_cv.at<float>(0, 0) << spacer << camera_matrix_cv.at<float>(0, 1) << spacer << camera_matrix_cv.at<float>(0, 2));
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t  " << camera_matrix_cv.at<float>(1, 0) << spacer << camera_matrix_cv.at<float>(1, 1) << spacer << camera_matrix_cv.at<float>(1, 2));
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t  " << camera_matrix_cv.at<float>(2, 0) << spacer << camera_matrix_cv.at<float>(2, 1) << spacer << camera_matrix_cv.at<float>(2, 2) << " ]\n");

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Distortion coefficients:");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t[ " << distortion_coef.at<float>(0, 0) << spacer << distortion_coef.at<float>(0, 1) << spacer << distortion_coef.at<float>(0, 2) << spacer << distortion_coef.at<float>(0, 3) << spacer << distortion_coef.at<float>(0, 4) << " ]\n");

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Camera projection:");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t[ " << camera_projection(0, 0) << spacer << camera_projection(0, 1) << spacer << camera_projection(0, 2));
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t  " << camera_projection(1, 0) << spacer << camera_projection(1, 1) << spacer << camera_projection(1, 2));
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t  " << camera_projection(2, 0) << spacer << camera_projection(2, 1) << spacer << camera_projection(2, 2));
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t  " << camera_projection(3, 0) << spacer << camera_projection(3, 1) << spacer << camera_projection(3, 2) << " ]\n");

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Camera intrinsics:");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t[ " << camera_intrinsics(0, 0) << spacer << camera_intrinsics(0, 1) << spacer << camera_intrinsics(0, 2));
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t  " << camera_intrinsics(1, 0) << spacer << camera_intrinsics(1, 1) << spacer << camera_intrinsics(1, 2));
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t  " << camera_intrinsics(2, 0) << spacer << camera_intrinsics(2, 1) << spacer << camera_intrinsics(2, 2) << " ]\n");

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Homogeneous:");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t[ " << homogeneous(0, 0) << spacer << homogeneous(0, 1) << spacer << homogeneous(0, 2));
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t  " << homogeneous(1, 0) << spacer << homogeneous(1, 1) << spacer << homogeneous(1, 2));
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t  " << homogeneous(2, 0) << spacer << homogeneous(2, 1) << spacer << homogeneous(2, 2));
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\t  " << homogeneous(3, 0) << spacer << homogeneous(3, 1) << spacer << homogeneous(3, 2) << " ]\n");
}

inline void matrixProduct(const boost::numeric::ublas::vector<float> &lidar_point, const boost::numeric::ublas::matrix<float> &fusion_matrix, boost::numeric::ublas::vector<float> &camera_point)
{
    camera_point(0) = ((lidar_point(0)) ? lidar_point(0) * fusion_matrix(0, 0) : 0) + ((lidar_point(1)) ? lidar_point(1) * fusion_matrix(1, 0) : 0) + ((lidar_point(2)) ? lidar_point(2) * fusion_matrix(2, 0) : 0) + fusion_matrix(3, 0); // lidar_point(3) * fusion_matrix(3,0);
    camera_point(1) = ((lidar_point(0)) ? lidar_point(0) * fusion_matrix(0, 1) : 0) + ((lidar_point(1)) ? lidar_point(1) * fusion_matrix(1, 1) : 0) + ((lidar_point(2)) ? lidar_point(2) * fusion_matrix(2, 1) : 0) + fusion_matrix(3, 1); // lidar_point(3) * fusion_matrix(3,1);
    camera_point(2) = ((lidar_point(0)) ? lidar_point(0) * fusion_matrix(0, 2) : 0) + ((lidar_point(1)) ? lidar_point(1) * fusion_matrix(1, 2) : 0) + ((lidar_point(2)) ? lidar_point(2) * fusion_matrix(2, 2) : 0) + fusion_matrix(3, 2); // lidar_point(3) * fusion_matrix(3,2);
}

inline cv::Point distortImagePoint(const int &x, const int &y, const cv::Mat &dist_vect, const boost::numeric::ublas::matrix<float> &camera_matrix)
{
    // norm
    float x_n = ((float)x - camera_matrix(2, 0)) / camera_matrix(0, 0);
    float y_n = ((float)y - camera_matrix(2, 1)) / camera_matrix(1, 1);

    float rr = x_n * x_n + y_n * y_n;
    float rr_2 = rr * rr;
    float rr_3 = rr_2 * rr;
    float xy_n = x_n * y_n;

    float val = (1 + dist_vect.at<float>(0, 0) * rr + dist_vect.at<float>(0, 1) * rr_2 + dist_vect.at<float>(0, 4) * rr_3);

    float x_dist = x_n * val + 2 * dist_vect.at<float>(0, 2) * xy_n + dist_vect.at<float>(0, 3) * (rr + 2 * x_n * x_n);
    float y_dist = y_n * val + dist_vect.at<float>(0, 2) * (rr + 2 * y_n * y_n) + 2 * dist_vect.at<float>(0, 3) * xy_n;

    // unnorm
    float u = x_dist * camera_matrix(0, 0) + camera_matrix(2, 0);
    float v = y_dist * camera_matrix(1, 1) + camera_matrix(2, 1);

    return cv::Point(u, v);
}

cv::Point getProjectionForPoint(geometry_msgs::msg::Point32 *point, const boost::numeric::ublas::matrix<float> &projection_matrix, const cv::Mat &dist_coef, const boost::numeric::ublas::matrix<float> &camera_intrinsics)
{
    try
    {
        int posx = -1;
        int posy = -1;

        // Get 3D and 2D coordinates for each point
        boost::numeric::ublas::vector<float> lidar_point(4);
        boost::numeric::ublas::vector<float> camera_point(3);

        // Cast to homogeneous vector (ROS coord to Beamagine coord)
        lidar_point(0) = -point->y * 1000.0;
        lidar_point(1) = -point->z * 1000.0;
        lidar_point(2) = point->x * 1000.0;
        lidar_point(3) = 1.0;

        // Project the LIDAR point (3D) onto the camera plane
        // boost::numeric::ublas::noalias(camera_point )= boost::numeric::ublas::prec_prod(lidar_point, projection_matrix);
        matrixProduct(lidar_point, projection_matrix, camera_point); // Same as line above but faster (for this case)

        // Transform 2D plane point imposing Z = 1 condition
        posx = camera_point(0) / camera_point(2);
        posy = camera_point(1) / camera_point(2);

        // Distort projected point to match image distortion
        return distortImagePoint(posx, posy, dist_coef, camera_intrinsics);
    }
    catch (...)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR: getProjectionForPoint() Unhandled error");
        return cv::Point(-1, -1);
    }
}

void computeColor(int value, double *red_value, double *green_value, double *blue_value, visualizationTypes visualization_type)
{
    int op = 0;
    double value_d = (double)value;
    int Lo = 0;
    int Lf = 0;
    int color_type;

    if (visualization_type == visualizationTypes::RAINBOW || visualization_type == visualizationTypes::RAINBOW_Z)
    {
        color_type = 0;
    }
    else if (visualization_type == visualizationTypes::INTENSITY)
    {
        color_type = 1;
    }

    op = value / (255 / MAX_COLOR_LEVEL);

    if (op == 0)
    {
        Lo = MAX_COLOR_LEVEL - 1;
    }
    else
    {
        Lo = op - 1;
    }

    value_d = value_d - (op * 36);

    Lf = op;
    *red_value = R[color_type][Lo] + 0.0277 * (R[color_type][Lf] - R[color_type][Lo]) * value_d;
    *green_value = G[color_type][Lo] + 0.0277 * (G[color_type][Lf] - G[color_type][Lo]) * value_d;
    *blue_value = B[color_type][Lo] + 0.0277 * (B[color_type][Lf] - B[color_type][Lo]) * value_d;
}

int getProjectionList(std::vector<ColorPoint> &projection_list, const boost::numeric::ublas::matrix<float> &projection_matrix, const cv::Mat &dist_coef, const boost::numeric::ublas::matrix<float> &camera_intrinsics,
                      const int pointcloud_color, const int min_color_value = 0, const int max_color_value = 10000)
{
    projection_list.clear();

    double lut_scale;
    if (max_color_value == min_color_value)
    {
        lut_scale = 1.0;
    }
    else
    {
        lut_scale = 255.0 / (max_color_value - min_color_value);
    }

    cv::Point point;
    sensor_msgs::msg::ChannelFloat32 channel;
    channel.name = "";
    switch (pointcloud_color)
    {
    case WHITE:
        for (geometry_msgs::msg::Point32 point_pcd : m_point_cloud.points)
        {
            point = getProjectionForPoint(&point_pcd, projection_matrix, dist_coef, camera_intrinsics);

            if (point.x >= 0 && point.x < m_img_width && point.y >= 0 && point.y < m_img_height)
            {
                ColorPoint cPoint;
                cPoint.point = point;
                cPoint.color[0] = (uint8_t)255; // b
                cPoint.color[1] = (uint8_t)255; // g
                cPoint.color[2] = (uint8_t)255; // r
                projection_list.push_back(cPoint);
            }
        }
        break;

    case INTENSITY:
        for (sensor_msgs::msg::ChannelFloat32 ch : m_point_cloud.channels)
        {
            if (ch.name == "intensity")
                channel = ch;
        }
        if (channel.name == "")
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR: No 'intensity' channel available");
            return 3;
        }

        for (int i = 0; i < (int)m_point_cloud.points.size(); i++)
        {
            point = getProjectionForPoint(&m_point_cloud.points[i], projection_matrix, dist_coef, camera_intrinsics);

            if (point.x >= 0 && point.x < m_img_width && point.y >= 0 && point.y < m_img_height)
            {
                // map intensity to color info with min_color_value and max_color_value
                int color_value = round((channel.values[i] - min_color_value) * lut_scale);
                color_value = color_value % 255;
                double r, g, b;
                computeColor(color_value, &r, &g, &b, visualizationTypes::INTENSITY);

                ColorPoint cPoint;
                cPoint.point = point;
                cPoint.color[0] = (uint8_t)b;
                cPoint.color[1] = (uint8_t)g;
                cPoint.color[2] = (uint8_t)r;
                projection_list.push_back(cPoint);
            }
        }
        break;

    case RGB:
        for (sensor_msgs::msg::ChannelFloat32 ch : m_point_cloud.channels)
        {
            if (ch.name == "rgb")
                channel = ch;
        }
        if (channel.name == "")
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR: No 'rgb' channel available");
            return 3;
        }

        for (int i = 0; i < (int)m_point_cloud.points.size(); i++)
        {
            point = getProjectionForPoint(&m_point_cloud.points[i], projection_matrix, dist_coef, camera_intrinsics);

            if (point.x >= 0 && point.x < m_img_width && point.y >= 0 && point.y < m_img_height)
            {
                ColorPoint cPoint;
                cPoint.point = point;
                cPoint.color[0] = (uint32_t)channel.values[i] & 0xFF;         // b
                cPoint.color[1] = ((uint32_t)channel.values[i] >> 8) & 0xFF;  // g
                cPoint.color[2] = ((uint32_t)channel.values[i] >> 16) & 0xFF; // r
                projection_list.push_back(cPoint);
            }
        }
        break;

    case RAINBOW:
        for (geometry_msgs::msg::Point32 point_pcd : m_point_cloud.points)
        {
            point = getProjectionForPoint(&point_pcd, projection_matrix, dist_coef, camera_intrinsics);

            if (point.x >= 0 && point.x < m_img_width && point.y >= 0 && point.y < m_img_height)
            {
                // map euclidean_distance to color info with min_color_value and max_color_value
                float euclidean_distance = sqrt(((double)point_pcd.x * (double)point_pcd.x) +
                                                ((double)point_pcd.y * (double)point_pcd.y) +
                                                ((double)point_pcd.z * (double)point_pcd.z));

                int color_value = round((euclidean_distance * 1000.0 - min_color_value) * lut_scale);
                color_value = color_value % 255;
                double r, g, b;
                computeColor(color_value, &r, &g, &b, visualizationTypes::RAINBOW);

                ColorPoint cPoint;
                cPoint.point = point;
                cPoint.color[0] = (uint8_t)b;
                cPoint.color[1] = (uint8_t)g;
                cPoint.color[2] = (uint8_t)r;
                projection_list.push_back(cPoint);
            }
        }
        break;

    case RAINBOW_Z:
        for (geometry_msgs::msg::Point32 point_pcd : m_point_cloud.points)
        {
            point = getProjectionForPoint(&point_pcd, projection_matrix, dist_coef, camera_intrinsics);

            if (point.x >= 0 && point.x < m_img_width && point.y >= 0 && point.y < m_img_height)
            {
                // map point_pcd.z to color info with min_color_value and max_color_value (move inside if)
                int color_value = round((point_pcd.x * 1000.0 - min_color_value) * lut_scale); // point_pcd.x es Z en coordenadas Beamagine
                color_value = color_value % 255;
                double r, g, b;
                computeColor(color_value, &r, &g, &b, visualizationTypes::RAINBOW_Z);

                ColorPoint cPoint;
                cPoint.point = point;
                cPoint.color[0] = (uint8_t)b;
                cPoint.color[1] = (uint8_t)g;
                cPoint.color[2] = (uint8_t)r;
                projection_list.push_back(cPoint);
            }
        }
        break;

    default:
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR: Value out of range");
        return 4;
        break;
    }

    return 0;
}

class FusionLidarImg : public rclcpp::Node
{
public:
    FusionLidarImg() : Node("fusion_lidar_img")
    {
        // Declare and get parameters
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        this->declare_parameter("mat_file", "/home/beamagine/Downloads/fusion_matrix.txt", descriptor);
        this->declare_parameter("sensor_name", "RGB", descriptor);
        m_img_sensor_name = this->get_parameter("sensor_name").as_string();
        this->declare_parameter("print_matrix", false, descriptor);

        this->declare_parameter("lidar_topic", "PC2_lidar", descriptor);
        this->declare_parameter("img_topic", "img_rgb", descriptor);
        this->declare_parameter("fusion_topic", "fusion", descriptor);

        rcl_interfaces::msg::IntegerRange range;
        descriptor.read_only = false;
        descriptor.description =
            "pointcloud_color options:\n"
            "\tWHITE = 0\n"
            "\tINTENSITY = 1\n"
            "\tRGB = 2\n"
            "\tRAINBOW = 3\n"
            "\tRAINBOW_Z = 4";
        range.set__from_value(0).set__to_value(4).set__step(1); // TODO: enumerate visualizationTypes
        descriptor.integer_range = {range};
        this->declare_parameter("pointcloud_color", (int)RAINBOW, descriptor);
        descriptor.description = "";
        range.set__from_value(0).set__to_value(200000).set__step(100);
        descriptor.integer_range = {range};
        this->declare_parameter("color_value_min", 0, descriptor);
        this->declare_parameter("color_value_max", 10000, descriptor);
        range.set__from_value(0).set__to_value(15).set__step(1);
        descriptor.integer_range = {range};
        this->declare_parameter("projection_points_radius", 3, descriptor);

        // Read matrix file from parameter
        int error = 0;
        error = readFusionMatrix(this->get_parameter("mat_file").as_string());

        if (error)
        {
            rclcpp::shutdown();
            return;
        }

        // Print matrix
        if (this->get_parameter("print_matrix").as_bool())
        {
            printSensorMatrices(m_img_sensor_name, m_camera_matrix_cv, m_distortion_coef, m_projection_matrix, m_camera_intrinsic_matrix, m_homogeneous_matrix);
        }

        // Subscribers and publishers
        lidar_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("lidar_topic").as_string(), 10, std::bind(&FusionLidarImg::lidar_callback, this, _1));

        img_sub = this->create_subscription<sensor_msgs::msg::Image>(
            this->get_parameter("img_topic").as_string(), 10, std::bind(&FusionLidarImg::img_callback, this, _1));

        fusion_pub = this->create_publisher<sensor_msgs::msg::Image>(this->get_parameter("fusion_topic").as_string(), 10);

        // Color tables for rainbow and intensity color drawing
        initializeColors();
    }

private:
    inline void draw_publish_fusion(const cv::Mat &img, const std::vector<ColorPoint> &lidar_projection) const
    {
        // Draw lidar projection in image and publish
        cv::Mat img_fusion(m_img_height, m_img_width, CV_8UC3);
        img_fusion = img;

        for (ColorPoint p : lidar_projection)
        {
            try
            {
                cv::circle(img_fusion, p.point, this->get_parameter("projection_points_radius").as_int(), p.color, cv::FILLED);
            }
            catch (...)
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "ERROR: Point out of range not filtered: (" << p.point.x << "," << p.point.y << ")\n"
                                                                  "Image dimensions (height, width): (" << m_img_height << ", " << m_img_width << ")");
            }
        }

        sensor_msgs::msg::Image fusion_msg;
        cv_bridge::CvImage fusion_bridge;

        std_msgs::msg::Header header;
        rclcpp::Clock time;
        header.stamp = time.now();
        header.frame_id = m_lidar_sensor_name + "_" + m_img_sensor_name + "_fusion";
        fusion_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img_fusion);
        fusion_bridge.toImageMsg(fusion_msg);
        fusion_pub->publish(fusion_msg);
    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
        if (m_img_buffer.empty())
            return;

        // Do projection and save it
        std::vector<ColorPoint> lidar_projection;
        sensor_msgs::convertPointCloud2ToPointCloud(*msg.get(), m_point_cloud);
        int error = getProjectionList(lidar_projection, m_projection_matrix, m_distortion_coef, m_camera_intrinsic_matrix,
                                      this->get_parameter("pointcloud_color").as_int(),
                                      this->get_parameter("color_value_min").as_int(),
                                      this->get_parameter("color_value_max").as_int());
        if (error)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Drawing pointcloud projection white");
            error = getProjectionList(lidar_projection, m_projection_matrix, m_distortion_coef, m_camera_intrinsic_matrix, WHITE);
        }

        // Search nearest image timestamp to lidar timestamp
        float timestamp_secs = msg.get()->header.stamp.sec + ((float)msg.get()->header.stamp.nanosec / 1000);
        float nearest = 0.0;
        for (const auto &img : m_img_buffer)
        {
            if (abs(img.first - timestamp_secs) < abs(nearest - timestamp_secs))
                nearest = img.first;
        }

        draw_publish_fusion(m_img_buffer[nearest], lidar_projection);
    }

    void img_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
        // Add image to buffer
        cv_bridge::CvImagePtr img_bridge = cv_bridge::toCvCopy(msg, msg->encoding);

        if (m_img_buffer.size() == m_max_buffer_size)
        {
            m_img_buffer.erase(m_img_buffer.begin()); // begin() gives you the smallest key element
        }
        float timestamp_secs = msg.get()->header.stamp.sec + ((float)msg.get()->header.stamp.nanosec / 1000);
        m_img_buffer.insert({timestamp_secs, img_bridge->image});
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fusion_pub;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<FusionLidarImg>());
    rclcpp::shutdown();
    return 0;
}
