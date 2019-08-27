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

//Eigen library
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
double pi = 3.14159265358979;

slide::Location result;

int result_flag = 0;
int first_receive_flag = 0;
int slide_window_init_flag = 0;

int64_t time_0, time_1;

deque<double> deque_result;
deque< deque<double> > deque_result_all;

deque<double> deque_orientation_result;
deque< deque<double> > deque_orientation_result_all;

double gps_init_x, gps_init_y, gps_init_heading;
double odometry_init_x, odometry_init_y, odometry_init_heading;

void resultCallback(const slide::Location::ConstPtr& result_msg)
{

      if( first_receive_flag == 0 )
      {
          //记录初始的gps和里程计x,y位置,朝向
          gps_init_x = result_msg -> x1;
          gps_init_y = result_msg -> y1;
          gps_init_heading = result_msg -> heading1;
          odometry_init_x = result_msg -> x2;
          odometry_init_y = result_msg -> y2;
          odometry_init_heading = result_msg -> heading2;

          cout << "gps_init_x: " << gps_init_x << endl;
          cout << "gps_init_y: " << gps_init_y << endl;
          cout << "gps_init_heading: " << gps_init_heading << endl;
          cout << "odometry_init_x: " << odometry_init_x << endl;
          cout << "odometry_init_y: " << odometry_init_y << endl;
          cout << "odometry_init_heading: " << odometry_init_heading << endl;
          cout << "-------------------------->" << endl;

          first_receive_flag = 1;
      }

      if ( slide_window_init_flag == 0 ) //slide_window初始时刻记录的数据
      {
          //G_i - G_0
          //记录的是每次的值 - 初始值
          deque_result.push_back(result_msg->x1 - gps_init_x) ;
          deque_result.push_back(result_msg->y1 - gps_init_y);
          deque_result.push_back(result_msg->x2 - odometry_init_x);
          deque_result.push_back(result_msg->y2 - odometry_init_y);
          deque_result.push_back(result_msg->heading1 - gps_init_heading);
          deque_result.push_back(result_msg->heading1 - odometry_init_heading);

          deque_result_all.push_back(deque_result);
          cout << "\033[1;34mdeque_result_all.size(): \033[0m" << deque_result_all.size() << endl;

//          cout << "deque_result_all[1][0]: " << fixed << deque_result_all[1][0] << endl;
//          cout << "deque_result_all[2][0]: " << fixed << deque_result_all[2][0] << endl;
//          cout << "deque_result_all[size-3][0]: " << fixed << deque_result_all[deque_result_all.size()-3][0] << endl;
//          cout << "deque_result_all[size-2][0]: " << fixed << deque_result_all[deque_result_all.size()-2][0] << endl;
//          cout << "deque_result_all[size-1][0]: " << fixed << deque_result_all[deque_result_all.size()-1][0] << endl;


          deque<double>().swap(deque_result);

      }

      if ( slide_window_init_flag == 1) //slide_window创建完成后,每来一个数据就更新slide_window
      {

          cout << "\033[1;33mslide_window_size: \033[0m" << deque_result_all.size() << endl;
          cout << "gps_x  : [" << fixed << deque_result_all[0][0] << " , " << deque_result_all[1][0] << " , "
               << deque_result_all[2][0] << " ...... " << deque_result_all[deque_result_all.size()-3][0] << " , "
               << deque_result_all[deque_result_all.size()-2][0] << " , " << deque_result_all[deque_result_all.size()-1][0] << "]" << endl;

          cout << "odo_x  : [" << fixed << deque_result_all[0][2] << " , " << deque_result_all[1][2] << " , "
               << deque_result_all[2][2] << " ...... " << deque_result_all[deque_result_all.size()-3][2] << " , "
               << deque_result_all[deque_result_all.size()-2][2] << " , " << deque_result_all[deque_result_all.size()-1][2] << "]" << endl;

          cout << "gps_y  : [" << fixed << deque_result_all[0][1] << " , " << deque_result_all[1][1] << " , "
               << deque_result_all[2][1] << " ...... " << deque_result_all[deque_result_all.size()-3][1] << " , "
               << deque_result_all[deque_result_all.size()-2][1] << " , " << deque_result_all[deque_result_all.size()-1][1] << "]" << endl;

          cout << "odo_y  : [" << fixed << deque_result_all[0][3] << " , " << deque_result_all[1][3] << " , "
               << deque_result_all[2][3] << " ...... " << deque_result_all[deque_result_all.size()-3][3] << " , "
               << deque_result_all[deque_result_all.size()-2][3] << " , " << deque_result_all[deque_result_all.size()-1][3] << "]" << endl;

          cout << "gps_ori: [" << fixed << deque_result_all[0][4] << " , " << deque_result_all[1][4] << " , "
               << deque_result_all[2][4] << " ...... " << deque_result_all[deque_result_all.size()-3][4] << " , "
               << deque_result_all[deque_result_all.size()-2][4] << " , " << deque_result_all[deque_result_all.size()-1][4] << "]" << endl;

          cout << "odo_ori: [" << fixed << deque_result_all[0][5] << " , " << deque_result_all[1][5] << " , "
               << deque_result_all[2][5] << " ...... " << deque_result_all[deque_result_all.size()-3][5] << " , "
               << deque_result_all[deque_result_all.size()-2][5] << " , " << deque_result_all[deque_result_all.size()-1][5] << "]" << endl;

          //朝向差的角度,也要标定?
          const double theta = 30;
          //P(gps) =  T * P(oddmetry)
          //得需要标定
          Eigen::Matrix<double, 3, 3> matrix_T;  // 声明一个3*3的double矩阵
          matrix_T << 1, 2, 1, 1, 2, 1, 0, 0, 1;  //输入数据（初始化）
          //cout << "matrix_T:" << endl << matrix_T << endl; // 输出

          Eigen::MatrixXd matrix_ones = Eigen::MatrixXd::Ones(1,deque_result_all.size());
          //cout << "matrix_ones: " << matrix_ones << endl;
//
//          Eigen::MatrixXd matrix_gps_x(1, deque_result_all.size());
//          Eigen::MatrixXd matrix_gps_y(1, deque_result_all.size());
//
//          Eigen::MatrixXd matrix_odometry_x(1, deque_result_all.size());
//          Eigen::MatrixXd matrix_odometry_y(1, deque_result_all.size());

          Eigen::MatrixXd matrix_heading(2, deque_result_all.size());
          Eigen::MatrixXd matrix_odometry_to_gps_heading(1, deque_result_all.size());

          Eigen::MatrixXd matrix_gps(3, deque_result_all.size());
          Eigen::MatrixXd matrix_odometry(3, deque_result_all.size());

          Eigen::MatrixXd matrix_odometry_to_gps(3, deque_result_all.size());
          Eigen::MatrixXd matrix_gps_to_odometry(3, deque_result_all.size());

          Eigen::MatrixXd gps_distance(2, deque_result_all.size());
          Eigen::MatrixXd odometry_distance(2, deque_result_all.size());

          for(int i = 0; i < 2; i++)
          {
              for(int j = 0; j < deque_result_all.size(); j++)
              {
                  matrix_gps(i,j) = deque_result_all[j][i];
                  matrix_odometry(i,j) = deque_result_all[j][i+2];
                  matrix_heading(i,j) = deque_result_all[j][i+4];
              }
          }

          matrix_gps.row(2) << matrix_ones;
          matrix_odometry.row(2) << matrix_ones;

          cout << "matrix_gps: " << endl << fixed << matrix_gps << endl << "-------" << endl;
          cout << "matrix_odometry: " << endl << fixed << matrix_odometry << endl << "---------" << endl;
          cout << "matrix_heading: " << endl << fixed << matrix_heading << endl << "---------" << endl;

//          //把动态窗口中的数据转换成矩阵形式
//          for(int j = 0; j < deque_result_all.size(); j++)
//          {
//              matrix_gps_x(0,j) = deque_result_all[j][0];
//              matrix_gps_y(0,j) = deque_result_all[j][1];
//              matrix_odometry_x(0,j) = deque_result_all[j][2];
//              matrix_odometry_y(0,j) = deque_result_all[j][3];
//          }
//
//          //cout << "matrix_gps_x: " << fixed << matrix_gps_x << endl;
//
//          matrix_gps.row(0) << matrix_gps_x;
//          matrix_gps.row(1) << matrix_gps_y;
//          matrix_gps.row(2) << matrix_ones;
//          cout << "matrix_gps: " << endl << fixed << matrix_gps << endl << "-------" << endl;
//
//          matrix_odometry.row(0) << matrix_odometry_x;
//          matrix_odometry.row(1) << matrix_odometry_y;
//          matrix_odometry.row(2) << matrix_ones;
//          cout << "matrix_odometry: " << endl << fixed << matrix_odometry << endl << "---------" << endl;

          //odomerty朝向转到gps坐标
          matrix_odometry_to_gps_heading = theta * matrix_ones + matrix_heading.row(1);
          cout << "matrix_odometry_to_gps_heading: " << endl << fixed << matrix_odometry_to_gps_heading << endl << "---------" << endl;

          //odomerty坐标转到gps坐标
          matrix_odometry_to_gps = matrix_T * matrix_odometry;
          cout << "matrix_odometry_to_gps: " << endl << fixed << matrix_odometry_to_gps << endl << "---------" << endl;

          //计算odometry转换得到的gps坐标点和实际gps坐标的对应点距离
          for(int i = 0; i < 2; i++)
          {
              for(int j = 0; j < deque_result_all.size(); j++)
              {
                  gps_distance(i,j) = pow(matrix_odometry_to_gps(i,j) - matrix_gps(i,j),2);
              }
          }
          cout << "gps_distance: " << endl << fixed << gps_distance << endl << "(---------------------------)" << endl;

          //测试用
//          cout << deque_result_all.size() << endl;
//          cout << "deque_result_all[0][0]: " << fixed << deque_result_all[0][0] << endl;
//          cout << "deque_result_all[1][0]: " << fixed << deque_result_all[1][0] << endl;
//          cout << "deque_result_all[2][0]: " << fixed << deque_result_all[2][0] << endl;
//          cout << "deque_result_all[size-3][0]: " << fixed << deque_result_all[deque_result_all.size()-3][0] << endl;
//          cout << "deque_result_all[size-2][0]: " << fixed << deque_result_all[deque_result_all.size()-2][0] << endl;
//          cout << "deque_result_all[size-1][0]: " << fixed << deque_result_all[deque_result_all.size()-1][0] << endl;
//          cout << "***********" << endl;
//
//          cout << "deque_result_all[0][1]: " << fixed << deque_result_all[0][1] << endl;
//          cout << "deque_result_all[1][1]: " << fixed << deque_result_all[1][1] << endl;
//          cout << "deque_result_all[2][1]: " << fixed << deque_result_all[2][1] << endl;
//          cout << "deque_result_all[size-3][1]: " << fixed << deque_result_all[deque_result_all.size()-3][1] << endl;
//          cout << "deque_result_all[size-2][1]: " << fixed << deque_result_all[deque_result_all.size()-2][1] << endl;
//          cout << "deque_result_all[size-1][1]: " << fixed << deque_result_all[deque_result_all.size()-1][1] << endl;
//          cout << "***********" << endl;

          //窗口中最早的数据弹出来
          deque_result_all.pop_front();

          //添加新的数据到当前窗口
          deque_result.push_back(result_msg->x1 - gps_init_x) ;
          deque_result.push_back(result_msg->y1 - gps_init_y);
          deque_result.push_back(result_msg->x2 - odometry_init_x);
          deque_result.push_back(result_msg->y2 - odometry_init_y);
          deque_result.push_back(result_msg->heading1 - gps_init_heading);
          deque_result.push_back(result_msg->heading1 - odometry_init_heading);

          deque_result_all.push_back(deque_result);
          cout << deque_result_all.size() << endl;

          deque<double>().swap(deque_result);

      }

//    cout <<  "\033[1;35m----> odometry_info:\033[0m" << endl;
//    cout << fixed << odometry.x << " , " << odometry.y << " , " << odometry.heading << " , "
//                  << odometry.stamp << " , " << odometry_stamp_duration << endl;
//    cout << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "shou_bu");
    ros::NodeHandle n;

    ros::Subscriber sub_result = n.subscribe("/pub_result", 10, resultCallback);

    ROS_INFO("\033[1;31m----> Ready_shou_bu!\033[0m");

    Eigen::Matrix<double, 3, 3> A;  // 声明一个3*3的double矩阵
    A << 1, 2, 3, 4, 5, 6, 7, 8, 9;  //输入数据（初始化）
    cout << "A: " << endl << fixed << A << endl;

    Eigen::Matrix<double, 3, 3> B;  // 声明一个3*3的double矩阵
    B << 1, 1, 1, 1, 1, 1, 1, 1, 1;  //输入数据（初始化）
    cout << "B: " << endl << fixed << B << endl;

    Eigen::Matrix<double, 3, 3> C;  // 声明一个3*3的double矩阵

    for(int i = 0; i < A.rows(); i++)
    {
        for(int j = 0; j < A.cols(); j++)
        {
            C(i,j) = pow((A(i,j) - B(i,j)),2);
        }

    }
    cout << "C: " << endl << fixed << C << endl;
    //cout << "2+B: " << endl << fixed << 2+B << endl;

    while(ros::ok())
    {

        if( first_receive_flag == 1)
        {
            time_0 = int64_t((ros::Time::now().toSec())*1000.0); //滑动窗口的初始时间,接收到第一帧数据的时间
            cout << "\033[1;33m----> time_0： \033[0m" << time_0  << endl;
            first_receive_flag = 2;
        }

        if( first_receive_flag == 2)
        {
            time_1 = int64_t((ros::Time::now().toSec())*1000.0);

            if( time_1 - time_0 > 4999 )
            {
                cout << "5秒种了！" << endl;
                cout << "窗口初始化完成！" << endl;
                slide_window_init_flag = 1;
                cout << "vector_result_all.size():" << deque_result_all.size() << endl;
                first_receive_flag = 3;

                //time_0 = time_1;
                //cout << "\033[1;33m----> time_1： \033[0m" << time_1  << endl;
            }

        }

        ros::spinOnce();
    }

    return 0;
}
