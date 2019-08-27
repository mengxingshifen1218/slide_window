//ROS library
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <slide/state.h>
#include <slide/Pose_.h>
#include <slide/Location.h>
#include <visualization_msgs/Marker.h>

//C++ library
#include <iostream>
#include <iomanip>
#include <math.h>
#include <vector>
#include <deque>

using namespace std;
double pi = 3.14159265358979;

slide::Pose_ gps;
slide::Pose_ odometry;
slide::Location result;

int64_t real_time;

int gps_flag = 1;
int odometry_flag = 1;
int time_flag = 1;
int size_flah = 1;

double gps_stamp_duration;
double gps_stamp_last;
double gps_x_last, gps_y_last, gps_heading_last;
double gps_x_chazhi, gps_y_chazhi, gps_heading_chazhi;

double odometry_stamp_duration;
double odometry_stamp_last;
double odometry_x_last, odometry_y_last, odometry_heading_last;
double odometry_x_chazhi, odometry_y_chazhi, odometry_heading_chazhi;

double result_stamp_last;
double result_stamp_duration;
int64_t time_0, time_1;

long int result_num = 0;
long int size_0, size_1, size_cha;

vector<double> vector_gps;
vector< vector<double> > vector_gps_result;

vector<double> vector_odometry;
vector< vector<double> > vector_odometry_result;


void gpsCallback(const slide::state::ConstPtr& gps_msg)
{
    //gps原始数据
    gps.x = gps_msg->x;
    gps.y = gps_msg->y;
    gps.heading = (270 - gps_msg->heading)*pi/180;
    gps.stamp = int64_t((ros::Time::now().toSec()) * 1000.0);

    //记录初始时间
    if (gps_flag == 1)
    {
        gps_stamp_last = gps.stamp;
        gps_flag = 0;
    }

    //计算时间间隔
    gps_stamp_duration = gps.stamp - gps_stamp_last;

    //插值计算
    if (gps_flag == 2)
    {
//        gps_x_chazhi = gps_x_last + (gps.x - gps_x_last) / gps_stamp_duration * 200;
//        gps_y_chazhi = gps_y_last + (gps.y - gps_y_last) / gps_stamp_duration * 200;
//        gps_heading_chazhi = gps_heading_last + (gps.heading - gps_heading_last) / gps_stamp_duration * 200;
//        cout << "gps.x: " << fixed << gps.x <<endl;
//        cout << "gps_x_last: " << fixed << gps_x_last <<endl;
//        cout << "gps_stamp_duration: " << fixed << gps_stamp_duration <<endl;
//        cout << "(gps.x - gps_x_last): " << fixed << (gps.x - gps_x_last) <<endl;
//        cout << "(gps.x - gps_x_last) / gps_stamp_duration * 200: " << fixed << (gps.x - gps_x_last) / gps_stamp_duration * 200 <<endl;
//        cout << "gps_x_chazhi: " << fixed << gps_x_chazhi <<endl;

        gps_x_chazhi = gps_x_last + (gps.x - gps_x_last) / gps_stamp_duration * 200;
        gps_y_chazhi = gps_y_last + (gps.y - gps_y_last) / gps_stamp_duration * 200;
        gps_heading_chazhi = gps_heading_last + (gps.heading - gps_heading_last) / gps_stamp_duration * 200;

        vector_gps.push_back(gps_x_chazhi);
        vector_gps.push_back(gps_y_chazhi);
        vector_gps.push_back(gps_heading_chazhi);

        vector_gps_result.push_back(vector_gps);
        //cout << "vector_gps_result.size:" << vector_gps_result.size() << endl;
        vector<double>().swap(vector_gps);

        cout <<  "\033[1;31m----> gps_info:\033[0m" << fixed  << "gps_x_chazhi:" << gps_x_chazhi << endl;
        cout <<  endl;

//        for(std::deque<double>::iterator m = deque_gps_x.begin(); m != deque_gps_x.end(); m++ )
//        {
//            cout << *m << endl; //deque
//        }
//        cout << "---------------" << endl;
//        cout << endl;

        //deque_gps_y.push_back(gps_y_chazhi);
        //deque_gps_heading.push_back(gps_heading_chazhi);

//        cout << "result.x1:" << fixed << result.x1 << endl;
//        cout << "result.y1:" << fixed << result.y1<< endl;
//        cout << "result.heading1:" << fixed << result.heading1<< endl;
//        cout << "-----" << endl;
//        cout << endl;
    }

    //更新变量
    gps_flag = 2;
    gps_stamp_last = gps.stamp;
    gps_x_last = gps.x;
    gps_y_last = gps.y;
    gps_heading_last = gps.heading;

//    cout <<  "\033[1;31m----> gps_info:\033[0m" << endl;
//    cout << fixed << gps.x << " , " << gps.y << " , " << gps.heading << " , "
//                  << gps.stamp << " , " << gps_stamp_duration << endl;
//    cout <<  endl;
}

void laserCallback(const nav_msgs::Odometry::ConstPtr& laser_msg)
{
    odometry.x = laser_msg->pose.pose.position.x;
    odometry.y = laser_msg->pose.pose.position.y;
    odometry.heading = laser_msg->pose.pose.position.z;
    odometry.stamp = int64_t((ros::Time::now().toSec()) * 1000.0);

    //记录初始时间
    if (odometry_flag == 1)
    {
        odometry_stamp_last = odometry.stamp;
        odometry_flag = 0;
    }

    //计算时间间隔
    odometry_stamp_duration = odometry.stamp - odometry_stamp_last;

    //插值计算
    if (odometry_flag == 2)
    {
        odometry_x_chazhi = odometry_x_last + (odometry.x - odometry_x_last) / odometry_stamp_duration * 200;
        odometry_y_chazhi = odometry_y_last + (odometry.y - odometry_y_last) / odometry_stamp_duration * 200;
        odometry_heading_chazhi = odometry_heading_last + (odometry.heading - odometry_heading_last) / odometry_stamp_duration * 200;

        vector_odometry.push_back(odometry_x_chazhi);
        vector_odometry.push_back(odometry_y_chazhi);
        vector_odometry.push_back(odometry_heading_chazhi);

        vector_odometry_result.push_back(vector_odometry);
        vector<double>().swap(vector_odometry);

        cout <<  "\033[1;34m----> odometry_info:\033[0m" << fixed  << "odometry_x_chazhi:" << odometry_x_chazhi << endl;
        cout <<  endl;

//        cout << "result.x2:" << fixed << result.x2<< endl;
//        cout << "result.y2:" << fixed << result.y2<< endl;
//        cout << "result.heading2:" << fixed << result.heading2<< endl;
//        cout << "-----" << endl;
//        cout << endl;
    }
    //更新变量
    odometry_flag = 2;
    odometry_stamp_last = odometry.stamp;
    odometry_x_last = odometry.x;
    odometry_y_last = odometry.y;
    odometry_heading_last = odometry.heading;

    //输出每次接收到的里程计信息
//    cout <<  "\033[1;35m----> odometry_info:\033[0m" << endl;
//    cout << fixed << odometry.x << " , " << odometry.y << " , " << odometry.heading << " , "
//                  << odometry.stamp << " , " << odometry_stamp_duration << endl;
//    cout << endl;
}

int flag = 1;
int A = 1;
int B = 0;
int ccc;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bu");
    ros::NodeHandle n;

    ros::Subscriber sub_laser = n.subscribe("/fa_laser", 10, laserCallback);
    ros::Subscriber sub_gps = n.subscribe("/GPSInfo", 10, gpsCallback);

    ros::Publisher pub_result = n.advertise<slide::Location>("pub_result", 10);

    ROS_INFO("\033[1;31m----> Ready!\033[0m");

    real_time = int64_t((ros::Time::now().toSec())*1000.0); //通过int64_只保留到毫秒

    cout << "\033[1;33m----> real_time： \033[0m" << real_time  << endl;

//    std::deque<int> V1;
//    for(int i = 1;i<11;i++)
//    {
//        V1.push_back(i);    //把元素一个一个存入到vector中
//    }
//
//    V1.pop_front();
//    cout << "V1.size():" << V1.size() << endl;
//    V1.pop_front();
//    cout << "V1.size():" << V1.size() << endl;
//    V1.resize(5);
//    cout << "V1.size():" << V1.size() << endl;
//
//    for(std::deque<int>::iterator m = V1.begin(); m != V1.end(); m++ )
//    {
//      cout << *m << endl; //deque
//    }

//    deque< deque<int> >V2;
//    deque<int> V3;
//    V3.push_back(1);
//    V3.push_back(2);
//    V3.push_back(3);
//    V2.push_back(V3);
//    V3.push_back(4);
//    V3.push_back(5);
//    V3.push_back(6);
//    V2.push_back(V3);
//    cout << "V2.size():" << V2.size() << endl;
//    cout << "V2.size():" << V2[1][1] << endl;

    size_0 = vector_gps_result.size();
    cout << size_0 << "!" << endl;

    while(ros::ok())
    {
        time_1 = int64_t((ros::Time::now().toSec())*1000.0);

        if(time_flag == 1)
        {
            time_0 = time_1;
	        time_flag = 0;
	        cout << "ok! " << endl;
        }

        result_stamp_duration = time_1 - time_0;
        //cout << "result_stamp_duration" << result_stamp_duration << endl;

        if(result_stamp_duration > 199)
        {
            //cout << "ok!" << endl;
            if((vector_gps_result.size()!= 0) && (vector_odometry_result.size()!= 0))
            {
                size_1 = vector_gps_result.size();
                size_cha = size_1 - size_0;
                //cout << "size_cha = size_1 - size_0: " << size_cha << endl;

                if(size_1 - size_0 != 0)
                {
                    result.stamp = time_1;
                    result.x1 = vector_gps_result[result_num][0];
                    //cout << "vector_gps_result[result_num++][0]:" << fixed << vector_gps_result[result_num][0] <<endl;
                    result.y1 = vector_gps_result[result_num][1];
                    result.heading1 = vector_gps_result[result_num][2];

                    result.x2 = vector_odometry_result[result_num][0];
                    result.y2 = vector_odometry_result[result_num][1];
                    result.heading2 = vector_odometry_result[result_num][2];
                    result_num++;
                }

                size_0 = size_1;
                time_0 = time_1;
                pub_result.publish(result);

            }
        }

        ros::spinOnce();
    }

    return 0;
}

//cout << "A: " << A << endl;
//cout << "B: " << B << endl;
//cout << "A - B: " << A - B << endl;
//B = A;
//A++;
//cout << "-----" << endl;