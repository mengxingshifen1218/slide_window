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

deque<double>deque_gps_x;
deque<double>deque_gps_y;
deque<double>deque_gps_heading;

deque<double>deque_odometry_x;
deque<double>deque_odometry_y;
deque<double>deque_odometry_heading;

//deque< deque<double> >deque_gps;
//
//deque< deque<double> >deque_odometry;
vector< vector<float> > vfs;

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
        gps_x_chazhi = gps_x_last + (gps.x - gps_x_last) / gps_stamp_duration * 200;
        gps_y_chazhi = gps_y_last + (gps.y - gps_y_last) / gps_stamp_duration * 200;
        gps_heading_chazhi = gps_heading_last + (gps.heading - gps_heading_last) / gps_stamp_duration * 200;

        deque_gps_x.push_back(gps_x_chazhi);

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

        deque_odometry_x.push_back(odometry_x_chazhi);

        vector<float> vs ;
        vs.push_back(odometry_x_chazhi);
        vs.push_back(odometry_y_chazhi);
        vs.push_back(odometry_heading_chazhi);

        cout << vs[0] << endl;
        cout << vs[1] << endl;
        cout << vs[2] << endl;

        vfs.push_back(vs);
        vector<float>().swap(vs);
        int num = 0;

        cout << vfs[num][0] << endl;
        cout << vfs[num][1] << endl;
        cout << vfs[num][2] << endl;
        num ++;

//        vs.clear();
//        cout << vs[0] << endl;
//        cout << vs[1] << endl;
//        cout << vs[2] << endl;

        cout <<  "\033[1;34m----> odometry_info:\033[0m" << fixed  << "odometry_x_chazhi:" << odometry_x_chazhi << endl;
        cout <<  endl;

        //deque_odometry_y.push_back(odometry_y_chazhi);
        //deque_odometry_heading.push_back(odometry_heading_chazhi);

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ke");
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

//    vector< vector<float> > vfs;
//    vector<float> vs ;
//
//    vs.push_back(10.0);
//    vs.push_back(11.0);
//
//    cout<<vs[0]<<endl;
//    cout<<vs[1]<<endl;
//
//    vfs.push_back(vs);
//
//    cout<<vfs[0][0]<<endl;
//    cout<<vfs[0][1]<<endl;

//    vector<float> vs_ref = vfs[0];
//    cout<<vs_ref[1]<<endl;

//    vector<float> vs ;
//    vs.push_back(1);
//    vs.push_back(2);
//    vs.push_back(3);
//
//    cout << vs.size() << endl;
//
//
////        vfs.push_back(vs);
////        cout << vfs[0][0] << endl;
////        cout << vfs[0][1] << endl;
////        cout << vfs[0][2] << endl;
//    vector<float>().swap(vs);
//    //vs.swap(vector<float>(vs));
//    cout << vs.size() << endl;
//    cout << vs.capacity() << endl;


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

        if(result_stamp_duration > 199)
        {
            if((deque_gps_x.size()!= 0) && (deque_odometry_x.size()!= 0))
            {
                result.stamp = time_1;
                result.x1 = deque_gps_x[0];
                result.x2 = deque_odometry_x[0];
                deque_gps_x.pop_front();
                deque_odometry_x.pop_front();
                time_0 = time_1;
                //cout << "result.stamp: " << result.stamp << endl;
                pub_result.publish(result);
            }
        }

        ros::spinOnce();
    }

    return 0;
}

