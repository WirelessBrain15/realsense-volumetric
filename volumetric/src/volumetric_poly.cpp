// read and use pointcloud2 to calculate volume of a box

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
// void publishPCL(const vector<geometry_msgs::Point32> pointCloud, int size);     // NEEDS CHANGES
void publishMarkers(double maxX[2], double maxY[2], double minX[2], double minY[2]);
double setHeight(const vector<geometry_msgs::Point32> pointCloud, int size);
void getVolume(const vector<geometry_msgs::Point32> filteredCloud, int size, double newHeight);
void filterHeight(const vector<geometry_msgs::Point32> pointCloud, int size);
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

void getVolume(const vector<geometry_msgs::Point32> filteredCloud, int size, double newHeight)
{
    double maxX = filteredCloud[0].x;
    double maxY = filteredCloud[0].y;

    double minX = filteredCloud[0].x;
    double minY = filteredCloud[0].y;

    double maxXY;
    double maxYX;
    double minXY;
    double minYX;


    int constant = 1;
    // double slope1; 
    // double slope2; 

    double CmaxX[2];
    double CmaxY[2];
    double CminX[2];
    double CminY[2];

    // for(int i = 1; i < size; i++)
    // {
        
        
    //     if(filteredCloud[i].x > maxX)
    //     {
    //         maxX = filteredCloud[i].x;
    //         maxXY = filteredCloud[i].y;
    //     }
    //     if(filteredCloud[i].y > maxY)
    //     {
    //         maxY = filteredCloud[i].y;
    //         maxYX = filteredCloud[i].x;
    //     }
    //     if(filteredCloud[i].x < minX)
    //     {
    //         minX = filteredCloud[i].x;
    //         minXY = filteredCloud[i].y;
    //     }
    //     if(filteredCloud[i].y < minY && filteredCloud[i].x != 0)
    //     {
    //         minY = filteredCloud[i].y;       
    //         minYX = filteredCloud[i].x;
    //     }

    //     slope1 = (maxXY - maxY)/(maxX - maxYX);
    //     slope2 = (maxY - minXY)/(maxYX - minX);

    //     // if(abs(1 - abs(slope1*slope2)) > 0.01)
    //     // // if((abs(maxX) - abs(maxYX) < 0.015) && (abs(maxY) - abs(minXY) < 0.015) || (abs(maxYX) - abs(minX) < 0.015) && (abs(maxY) - abs(maxXY) < 0.015))
    //     // {
    //     //     cout << "[STRAIGHT]" << endl;
            
    //     // }
        
    // }

    // ------------------------------------------------------------------------------

    double coorX = -0.5;
    double coorY = -0.5;
    double maxDist = 0;
    double minDist = 1000;
    double prevPt1[2] = {-0.5, filteredCloud[0].y};
    double prevPt3[2] = {5, 5};
    double pt1[2] = {-2, -2};
    double pt2[2] = {-5, -5};
    double pt3[2] = {2, 2};
    double pt4[2] = {5, 5};

    int countM = 0;
    int countm = 0;

    for(geometry_msgs::Point32 point:filteredCloud)
    {
        double dist = sqrt(((point.x - coorX),2) + pow((point.y - coorY),2));

        if(dist <= minDist && (pt3[0] > point.x || pt3[1] > point.y))
        {
            minDist = dist;
            pt3[0] = point.x;
            pt3[1] = point.y;

            // if(point.x < prevPt3[0] || point.y < prevPt3[1])
            // {
            //     prevPt3[0] = pt3[0];
            //     prevPt3[1] = pt3[1];
            //     minDist = dist;
            //     pt3[0] = point.x;
            //     pt3[1] = point.y;
            // }
        }

        // else if(point.x < pt3[0] && abs(point.y - pt3[1] < 0.001))
        // {
        //     // prevPt3[0] = pt3[0];
        //     // prevPt3[1] = pt3[1];
        //     minDist = dist;
        //     pt3[0] = point.x;
        //     // pt3[1] = point.y;
        // }


        if(dist >= maxDist && (pt1[0] < point.x || pt1[1] < point.y))
        {
            maxDist = dist;
            pt1[0] = point.x;
            pt1[1] = point.y;
            
            
        }

        

        else if(point.x > pt1[0] && abs(point.y - pt1[1]) < 0.001)
        {
            // cout << "test" << endl;
            // prevPt1[0] = pt1[0];
            // prevPt1[1] = pt1[1];
            // maxDist = dist;
            pt1[0] = point.x;
            // pt1[1] = point.y;
        }

        
        // if(point.y > pt1[1])
        // {
        //     pt1[0] = point.x;
        //     pt1[1] = point.y;
        // }

        // if(point.y < pt3[1])
        // {
        //     pt3[0] = point.x;
        //     pt3[1] = point.y;
        // }
        
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

    double len14 = sqrt(pow((pt1[0] - pt4[0]),2) + pow((pt1[1] - pt4[1]),2));
    double len23 = sqrt(pow((pt2[0] - pt3[0]),2) + pow((pt2[1] - pt3[1]),2));

    double len12 = sqrt(pow((pt1[0] - pt2[0]),2) + pow((pt1[1] - pt2[1]),2));
    double len34 = sqrt(pow((pt4[0] - pt3[0]),2) + pow((pt4[1] - pt3[1]),2));

    double slope1 = (pt1[1] - pt2[1])/(pt1[0] - pt2[0]);
    double slope2 = (pt1[1] - pt4[1])/(pt1[0] - pt4[0]);

    double testD = sqrt(pow((pt1[1] - pt3[1]),2));
    cout << "--------------------" << endl;
    cout << "len14 - 23 : " << len14 - len23 << endl;
    cout << "len12 - 34 : " << len12 - len34 << endl;
    cout << abs(1 - abs(slope1*slope2)) << endl;
    cout << "test diagnol : " << abs(len12 - testD) << endl;
    // cout << "slope : " << slope1*slope2 << endl;
    // cout << "corner1 : " << (abs(abs(pt2[0]) - abs(pt1[0]))) << " || " << (abs(abs(pt1[1]) - abs(pt4[1]))) << endl;
    // cout << "corner2 : " << abs(abs(pt2[1]) - abs(pt1[1])) << " || " << abs(abs(pt1[0]) - abs(pt4[0])) << endl;


    if((abs(len14 - len23) > 0.002 && abs(len12 - len34) > 0.002) && abs(1 - abs(slope1*slope2)) > 0.1 ) // && abs(1 - abs(slope1*slope2)) > 0.12 
    // ((abs(abs(pt2[0]) - abs(pt1[0]))) < 0.06 && (abs(abs(pt1[1]) - abs(pt4[1])) < 0.06) || (abs(abs(pt2[1]) - abs(pt1[1])) < 0.046 && (abs(abs(pt1[0]) - abs(pt4[0])) < 0.06))) && 
    {
        cout << "straight" << endl;
        pt1[0] = pt2[0];
        pt2[1] = pt3[1];
        pt3[0] = pt4[0];
        pt4[1] = pt1[1];

    }

    // cout << "pt2 maxX : " << pt2[0] << endl;
    // cout << "pt1 X : " << pt1[0] << endl;
    // cout << "pt1 Y : " << pt1[1] << endl;
    // cout << "pt1 dist : " << sqrt(((pt1[0] - coorX),2) + pow((pt1[1] - coorY),2)) << endl;  
    // cout << "pt3 X : " << pt3[0] << endl; 
    // cout << "pt3 Y : " << pt1[1] << endl; 
    // cout << "pt3 dist : " << sqrt(((pt3[0] - coorX),2) + pow((pt1[1] - coorY),2)) << endl;

   
    // cout << "slope1 : " << slope1 << endl;
    // cout << "slope2 : " << slope2 << endl;
    // cout << "abs(pt2[0]) - abs(pt1[0]) : " << abs(pt2[0]) - abs(pt1[0]) << endl;
    // cout << "abs(pt1[1]) - abs(pt4[1]) : " << abs(pt1[1]) - abs(pt4[1]) << endl;
    // cout << "abs(pt2[1]) - abs(pt1[1]) : " << abs(pt2[1]) - abs(pt1[1]) << endl;
    // cout << "abs(pt1[0]) - abs(pt4[0]) : " << abs(pt1[0]) - abs(pt4[0]) << endl;

    // if((abs(abs(pt2[0]) - abs(pt1[0])) < 0.04 && abs(abs(pt1[1]) - abs(pt4[1])) < 0.04) )   //|| abs(abs(pt2[1]) - abs(pt1[1])) < 0.03 && abs(abs(pt1[0]) - abs(pt4[0])) < 0.03
    // {
    //     cout << "Straight" << endl;
    //     pt2[0] = pt1[0];
    //     pt2[1] = pt3[1];

    //     pt4[0] = pt3[0];
    //     pt4[1] = pt1[1];
    // }

    // if(abs(abs(pt2[1]) - abs(pt1[1])) < 0.03 && abs(abs(pt1[0]) - abs(pt4[0])) < 0.03)
    // {
    //     cout << "Straight 2" << endl;
    //     pt4[0] = pt1[0];
    //     pt4[1] = pt3[1];

    //     pt1[0] = pt3[0];
    //     pt1[1] = pt1[1];
    // }

    double length = sqrt(pow((pt1[0] - pt2[0]),2) + pow((pt1[1] - pt2[1]),2));
    double breadth = sqrt(pow((pt1[0] - pt4[0]),2) + pow((pt1[1] - pt4[1]),2));
    double volume = length*breadth*newHeight;

    // cout << "minDist : " << minDist << endl;
    // cout << "maxDist : " << maxDist << endl;
    // cout << "dist pt3 ideal : " << sqrt(((pt3[0] - coorX),2) + pow((pt1[1] - coorY),2)) << endl;
    // cout << "length : " << length << endl;
    // cout << "breadth : " << breadth << endl;
    // cout << "height : " << newHeight << endl;
    // cout << "volume : " << volume << endl;
    cout << "area : " << length*breadth << endl;

    cout << "error % : " << (VOL_BOX1 - volume)*100/(VOL_BOX1) << endl;
    



    publishMarkers(pt1,pt2,pt3,pt4, newHeight);



     // ------------------------------------------------------------------------------

// ------------------------------------------------------------------------------

    // // cout << maxY - maxXY << endl;
    // // cout << "---------------------------" << endl;
    // // cout << "max X : " << maxX << endl;
    // // cout << "max XY : " << maxXY << endl;
    // // cout << "max Y : " << maxY << endl;
    // // cout << "max YX : " << maxYX << endl;
    // // cout << "min X : " << minX << endl;
    // // cout << "min XY : " << minXY << endl;
    // // // cout << "min Y : " << minY << endl;

    // // cout << abs(maxYX) - abs(maxX) << " || " <<  (abs(maxXY) - abs(maxY)) << endl;
    // if((abs(maxX) - abs(maxYX) < 0.015) && (abs(maxY) - abs(minXY) < 0.015) || (abs(maxYX) - abs(minX) < 0.015) && (abs(maxY) - abs(maxXY) < 0.015)) //|| (abs(maxYX) - abs(minX) < 0.015) && (abs(maxXY) - abs(maxY) < 0.015)
    // {
    //     // cout << "[straight]" << endl;
    //     length = length + sqrt(pow((maxX - minX),2) + pow((maxY - maxY),2));
    //     breadth = breadth + sqrt(pow((minX - minX),2) + pow((maxY - minY),2));
    
    
    //     //  Visualization
    //     CmaxX[0] = (maxX);
    //     CmaxX[1] = (maxY);
    //     CmaxY[0] = (minX);
    //     CmaxY[1] = (minY);

    //     CminX[0] = (minX);
    //     CminX[1] = (maxY);
    //     CminY[0] = (maxX);
    //     CminY[1] = (minY);


    //     // slope1 = (maxY - maxY)/(maxX - minX);
    //     // slope2 = (maxY - minY)/(minX - minX);
    //     slope1 = 1;
    //     slope2 = -1;
    
    // }

    // else
    // {
    //     length = length + sqrt(pow((maxX - maxYX),2) + pow((maxXY - maxY),2));
    //     breadth = breadth + sqrt(pow((maxYX - minX),2) + pow((maxY - minXY),2));  


    //     //  Visualization
    //     CmaxX[0] = (maxX);
    //     CmaxX[1] = (maxXY);
    //     CmaxY[0] = (maxYX);
    //     CmaxY[1] = (maxY);

    //     CminX[0] = (minX);
    //     CminX[1] = (minXY);
    //     CminY[0] = (minYX);
    //     CminY[1] = (minY);

    //     slope1 = (maxXY - maxY)/(maxX - maxYX);
    //     slope2 = (maxY - minXY)/(maxYX - minX);

    // }
    

    // finalHeight = finalHeight + newHeight;
    // countAvg++;

    // if(countAvg == constant && abs(1 - abs(slope1*slope2)) < 0.1)
    // {
    //     length = length/constant;
    //     breadth = breadth/constant;
    //     finalHeight = finalHeight/constant;
    //     // cout << "[FOUND BOX]" << endl;
    //     double volume = (length)*(breadth)*(finalHeight);

    //     // cout << "length : " << length << endl;
    //     // cout << "breadth : " << breadth << endl;
    //     // cout << "height : " << finalHeight << endl;
    //     cout << "volume : " << volume << endl;


        
    //     // // cout << "slope1 : " << slope1 << endl;
    //     // // cout << "slope2 : " << slope2 << endl;
    //     cout << "slope : " << slope1*slope2 << endl;

    //     // ----------------------------------------------------------------------
    //     //  Visualization
        
    //     publishMarkers(CmaxX, CmaxY, CminX, CminY);

    //     // ----------------------------------------------------------------------



    //     countAvg = 0;
    //     length = 0;
    //     breadth = 0;
    //     finalHeight = 0;

    // }
// ------------------------------------------------------------------------------
    
}

void filterHeight(const vector<geometry_msgs::Point32> pointCloud, int size)
{
    vector<geometry_msgs::Point32> filteredPCL;

    int j = 0;
    int k = 0;
    double newHeight = 0;
    double maxH = 0;
    double minH = 100;


    for(geometry_msgs::Point32 point:pointCloud)
    {
        if(flagH == false && point.z < baseHeight - 0.03)
        {
            // cout << "test1" << endl;
            filteredPCL.push_back(point);
            // cout << "diff : " << point.z - curr_max << endl;
            j++;
            if(point.z < minH)
                minH = point.z;

            if(point.z > maxH)
                maxH = point.z;
            
        }
        
        else if(flagH == true && (point.z - curr_max) <= 0.015)
        {
            // cout << "test2" << endl;
            //  cout << "diff : " << point.z - curr_max << endl;
            filteredPCL.push_back(point);
            j++;
            if(point.z < minH)
                minH = point.z;

            if(point.z > maxH)
                maxH = point.z;
        }
        
        k++;
        // cout << curr_max << endl;
        
    
    }
    cout << "-------" << endl;
    cout << flagH << endl;
    cout << "curr_max : " << curr_max << endl;
    cout << "min, max : " << minH << " " << maxH << endl;
    cout << "J : " << j << endl;
    // cout << "K : " << k << endl;

    if(j != 0 && baseHeight - curr_max > 0.01)
    {
        newHeight = setHeight(filteredPCL, filteredPCL.size());
        testHeight = newHeight;
        flagH = true;
        // cout << "[FOUND BOX]" << endl;
        getVolume(filteredPCL, filteredPCL.size(), baseHeight - newHeight);

    }
    else
    {
        curr_max = 100;
        countAvg = 0;
        length = 0;
        breadth = 0;
        testHeight = -1;
        flagH = false;
        finalHeight = 0;
    }


    
    // publishPCL(filteredPCL, filteredPCL.size());
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