// read and use pointcloud2 to calculate volume of a box
// roslaunch realsense2_camera rs_camera.launch filters:=pointcloud

#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h" 


#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>


// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>
#include <opencv2/core/core.hpp>
	typedef cv::Mat Mat;
typedef struct Mesh{Mat vertices, faces; Mesh(Mat v, Mat f):vertices(v), faces(f) {};} Mesh;

#include <vector>
#include <cmath>

using  namespace std;


// TOPICS
static const string PCL_TOPIC = "/camera/depth/color/points";
static const string PUB_TOPIC = "/pclFiltered";

static const double VOL_BOX1 = 0.19*0.1*0.1;//0.198*0.14*0.119;//0.198*0.182*0.074; 


// Global variables
double baseHeight = -1;  // Realsense's Height from platform
double length = 0;      // Box length
double breadth = 0;     // Box breadth
double finalHeight = 0;     // Averaged Height( = newHeight if constant  == 1) 
int countAvg = 0;       // Count to check if == constant
double curr_max = 100;
bool flagH = false;
double testHeight = -1;      // [TEST] to make dynamics height adjustment
double coorX = -0.5;
double coorY = -0.5;


// Publishers
ros::Publisher pub;
ros::Publisher vis_pub;


// Functions
void publishMarkers(double maxX[2], double maxY[2], double minX[2], double minY[2]);
double setHeight(const vector<geometry_msgs::Point32> pointCloud, int size);
void newVolume(const vector<geometry_msgs::Point32> filteredCloud, int size, double newHeight);
void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
double polygonArea(double X[], double Y[], int n);


void publishMarkers(double maxX[2], double maxY[2], double minX[2], double minY[2], double newHeight)
{
    visualization_msgs::Marker points, line_strip;
    geometry_msgs::Point point;

    // point.x = baseHeight - finalHeight;
    if(baseHeight - newHeight < 2 && baseHeight - newHeight > 0)
        point.x = baseHeight - newHeight - 0.01;
    // cout << point.x << endl;

    // origin
    point.y = -coorX;
    point.z = -coorY;
    points.points.push_back(point);
    line_strip.points.push_back(point);

    // pt1
    point.y = -maxX[0];
    point.z = -maxX[1];
    
    points.points.push_back(point);
    line_strip.points.push_back(point);

    // pt3
    // point.y = -minX[0];
    // point.z = -minX[1];
    // points.points.push_back(point);
    // line_strip.points.push_back(point);


    // pt2
    point.y = -maxY[0];
    point.z = -maxY[1];
    points.points.push_back(point);
    line_strip.points.push_back(point);
    
    // // pt4
    // point.y = -minY[0];
    // point.z = -minY[1];
    // points.points.push_back(point);
    // line_strip.points.push_back(point);

    


    points.header.frame_id = line_strip.header.frame_id = "camera_link";
    points.header.stamp = line_strip.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = "points_and_lines";
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;



    points.id = 0;
    line_strip.id = 1;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.02;
    points.scale.y = 0.02;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
    line_strip.scale.x = 0.01;

    

    vis_pub.publish(points);
    vis_pub.publish(line_strip);
}


double setHeight(const vector<geometry_msgs::Point32> pointCloud, int size)
{
    double height = 0;
    int sum = 0;

    for(int j = size/2 - 10; j < size/2 + 10; j++)
    {
        height = height + pointCloud[j].z;
        sum++;
    }
    height = height/sum;
    // cout << "[Height Set] : " << height << endl;
    return height;
}


void newVolume(const vector<geometry_msgs::Point32> filteredCloud, int size, double newHeight)
{
    // double coorX = -0.5;
    // double coorY = -0.5;
    double maxDist = 0;
    double minDist = 1000;
    double prevPt1[2] = {-0.5, filteredCloud[0].y};
    double prevPt3[2] = {5, 5};
    double pt1[2] = {-2, -2};
    double pt2[2] = {-5, -5};
    double pt3[2] = {2, 2};
    double pt4[2] = {5, 5};
    double count = 0;

    for(geometry_msgs::Point32 point:filteredCloud)
    {
        
        if(point.y > pt1[1])
        {
            pt1[0] = point.x;
            pt1[1] = point.y;
        }

        if(point.y < pt3[1])
        {
            pt3[0] = point.x;
            pt3[1] = point.y;
        }
        
        if(point.x > pt2[0])
        {
            pt2[0] = point.x;
            pt2[1] = point.y;
        }
        if(point.x < pt4[0])
        {
            pt4[0] = point.x;
            pt4[1] = point.y;
        }
    }

    coorX = (pt2[0] + pt4[0])/2;
    coorY = (pt1[1] + pt3[1])/2;

    double slope1 = (pt1[1] - pt2[1])/(pt1[0] - pt2[0]);
    double slope2 = (pt1[1] - pt4[1])/(pt1[0] - pt4[0]);
    cout << abs(1 - abs(slope1*slope2)) << endl;

    // for(geometry_msgs::Point32 point:filteredCloud)
    // {
    //     double dist = sqrt(((point.x - coorX),2) + pow((point.y - coorY),2));
    //     // cout << "----------------" << endl;
    //     // cout << "dist diff : " << maxDist - dist << endl;

    //     if(dist >= maxDist - 0.0000001 && (point.y > coorY && point.y > pt4[1] && point.y > pt3[1]))
    //     {
    //         maxDist = dist;
    //         pt1[0] = point.x;
    //         pt1[1] = point.y;
            
    //     }

    //     if(dist >= maxDist && point.y < coorY && point.y < pt3[1])
    //     {
    //         maxDist = dist;
    //         pt3[0] = point.x;
    //         pt3[1] = point.y;
    //     }

        
    //     if(dist >= maxDist && point.x > coorX && point.x > pt2[0])
    //     {
    //         maxDist = dist;
    //         pt2[0] = point.x;
    //         pt2[1] = point.y;
    //     }
    //     if(dist >= maxDist && point.x < coorX && point.x < pt4[0])
    //     {
    //         maxDist = dist;
    //         pt4[0] = point.x;
    //         pt4[1] = point.y;
    //     }
    // }

    for(geometry_msgs::Point32 point:filteredCloud)
    {
        if(abs(1 - abs(slope1*slope2)) <  0.2)
            count++;
    }
    cout << "count : " << count << endl;

    if(abs(1 - abs(slope1*slope2)) >  0.2)
    {
        pt1[0] = pt2[0];
        pt2[1] = pt3[1];
        pt3[0] = pt4[0];
        pt4[1] = pt1[1];
    }

    double length = sqrt(pow((pt1[0] - pt2[0]),2) + pow((pt1[1] - pt2[1]),2));
    double breadth = sqrt(pow((pt1[0] - pt4[0]),2) + pow((pt1[1] - pt4[1]),2));
    double volume = length*breadth*newHeight;
    // double volume = sum*(baseHeight - (curr_max+0.015));
    // cout << "area : " << sum << endl;
    // cout << "h : " << newHeight << endl;
    // cout << "volume : " << volume << endl;
    cout << "error % : " << (VOL_BOX1 - volume)*100/(VOL_BOX1) << endl;

    publishMarkers(pt1,pt2,pt3,pt4, newHeight);
}

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{


    curr_max = 100;
    // cout << "[CALLBACK]" << endl; 

    sensor_msgs::PointCloud out_pointcloud;
    // sensor_msgs::convertPointCloud2ToPointCloud(*msg, out_pointcloud);

    vector<geometry_msgs::Point32> pointArray;

    // ----------------------------------------------------------------------

    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;
    // Convert to PCL data type
    pcl_conversions::toPCL(*msg, *cloud);
    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (0.01f, 0.01f, 0.001f);
    sor.filter (cloud_filtered);


    // sor.setFilterFieldName ("z");
    // sor.setFilterLimits (0.0, curr_max + 0.02);
    // sor.filter(cloud_filtered);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_filtered, output);
    // Publish the data
    // pub.publish (output);



    sensor_msgs::convertPointCloud2ToPointCloud(output, out_pointcloud);
    int rows = msg->height;
    int cols = msg->width;
    double newHeight;

    // cout << "height :  " << rows << endl;
    // cout << "width  :  " << cols/848 << endl;


    for(geometry_msgs::Point32 point:out_pointcloud.points)
    {
        pointArray.push_back(point);
        // pointArray.push_back(out_pointcloud.points[i]);

        if(curr_max > point.z)
            curr_max = point.z;
    }

    if(baseHeight == -1)
    { 
        baseHeight = setHeight(pointArray, pointArray.size());
        cout << "[Height Set] : " << baseHeight << endl;
    }
        

    // filterHeight(pointArray, pointArray.size());
    if(baseHeight - curr_max > 0.03)
    {
        pointArray.clear();
        sor.setFilterFieldName ("z");
        sor.setFilterLimits (curr_max, curr_max + 0.025);
        sor.filter(cloud_filtered);
        pcl_conversions::fromPCL(cloud_filtered, output);
        sensor_msgs::convertPointCloud2ToPointCloud(output, out_pointcloud);
        for(geometry_msgs::Point32 point:out_pointcloud.points)
        {
            pointArray.push_back(point);
            newHeight = setHeight(pointArray, pointArray.size());
            // pointArray.push_back(out_pointcloud.points[i]);

            // if(curr_max > point.z)
            //     curr_max = point.z;
        }
        // cout << "h : " << (baseHeight - (curr_max+0.015)) << endl;

        // newVolume(pointArray, pointArray.size(), (baseHeight - (curr_max+0.015)));

        // newVolume(pointArray, pointArray.size(), baseHeight - newHeight);



    }
    
    // ----------------------------------------------------------------------
    // ----------------------------------------------------------------------
    // filter and area using triangles

    

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNew (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2 (cloud_filtered, *cloudNew);

    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloudNew);
    n.setInputCloud (cloudNew);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloudNew, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (0.1);

    // Set typical values for the parameters
    gp3.setMu (4);
    gp3.setMaximumNearestNeighbors (1000);
    gp3.setMaximumSurfaceAngle(M_PI/8); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

    // // Additional vertex information
    // std::vector<int> parts = gp3.getPartIDs();
    // std::vector<int> states = gp3.getPointStates();

    pcl::PointCloud<pcl::PointXYZ>::Ptr allVertices(new pcl::PointCloud<pcl::PointXYZ>);
    double sum = 0;
    pcl::fromPCLPointCloud2(triangles.cloud, *allVertices);

    for (int i = 0; i < triangles.polygons.size(); i++)
    {
            // std::cout << std::endl;
            double X[3];
            double Y[3];
            int index[] = {triangles.polygons[i].vertices[0],triangles.polygons[i].vertices[1],triangles.polygons[i].vertices[2]};
            for(int p = 0; p < 3; p++)
            {
                X[p] = out_pointcloud.points[index[p]].x;
                Y[p] = out_pointcloud.points[index[p]].y;
            }
            // std::cout << triangles.polygons[i].vertices[0] << std::endl;
            // std::cout << triangles.polygons[i].vertices[1] << std::endl;
            // std::cout << triangles.polygons[i].vertices[2] << std::endl;
            // std::cout << std::endl;
        
            sum = sum + polygonArea(X, Y, 3);
            // cout << out_pointcloud.points[0] << endl;
            // meshArea(sum, triangles.polygons[i].vertices[0], triangles.polygons[i].vertices[1], triangles.polygons[i].vertices[2]);

    }
    double volume = sum*(baseHeight - newHeight);
    cout << "area : "  << 10000*sum << setprecision(5) << endl;
    cout << "h : " << setprecision(4) << 100*(baseHeight - newHeight) << endl;
    cout << "volume : "<< setprecision(6) << 1000000*volume << endl;
    cout << "error % : " << (VOL_BOX1 - volume)*100/(VOL_BOX1) << endl;

    // pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    // viewer->setBackgroundColor (0, 0, 0);
    // viewer->addPointCloud<pcl::PointXYZ> (out_pointcloud, "sample cloud");
   
    // ----------------------------------------------------------------------


    // Convert to ROS data type
    // sensor_msgs::PointCloud2 output;
    // pcl_conversions::fromPCL(cloud_filtered, output);
    // Publish the data
    pub.publish (output);
}

double polygonArea(double X[], double Y[], int n)
{
    // Initialize area
    double area = 0.0;
  
    // Calculate value of shoelace formula
    int j = n - 1;
    for (int i = 0; i < n; i++)
    {
        area += (X[j] + X[i]) * (Y[j] - Y[i]);
        j = i;  // j is previous vertex to i
    }
  
    // Return absolute value
    return abs(area / 2.0);
}

// void timerCB(const ros::TimerEvent&)
// {
//     if(viewer.wasStopped())
//     {
//         ros::shutdown();
//     }
// }

int main(int argc, char** argv)
{
    cout << "[START]" << endl;
    ros::init(argc, argv, "volumetric");
    ros::NodeHandle nh;


    // pcl::visualization::CloudViewer viewer;
    // ros::Timer viewer_timer;
    // viewer_timer = nh.createTimer(ros::Duration(0.1), &cloudHandler::timerCB,this);

    vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    ros::Subscriber sub = nh.subscribe(PCL_TOPIC, 1, cloudCallback);
    
    // pub = nh.advertise<sensor_msgs::PointCloud>(PUB_TOPIC, 1);

    pub = nh.advertise<sensor_msgs::PointCloud2>(PUB_TOPIC, 1);

    ros::spin();
    
    return 0;
}