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
#include <string.h>
#include <math.h>
#include <vector>
#include <deque>


//Eigen library
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
double pi = 3.14159265358979;

struct Car
{
    char brand[32];
    int price;
};

struct Citizen
{
    char name[32];
    int deposite;
    Car* car;  //不能Car car这样定义,Car car这样的意思就是已经有这辆车了
};

void buy_car(Citizen* owner)
{
    Car* car = (Car*)malloc(sizeof(Car));
    strcpy(car->brand,"BMW");
    car->price = 100;

    owner->car = car;
    owner->deposite -= car->price;
}

void discard_car(Citizen* owner)
{
    free(owner->car); //注意本来写成了free(car)
    owner->car = NULL;
}

void sell_car(Citizen* owner, Citizen* other)
{
    //教程是这样写的
    Car* car = owner->car;

    car->price = 0.5 * car->price;
    other->car = car;
    other->deposite -= car->price;  //不明白指针指向->的是啥？

    owner->deposite += car->price;
    owner->car = NULL;

    //我这样写也可以
    other->car = owner->car;
    other->deposite -= owner->car->price;

    owner->deposite += owner->car->price;
    owner->car = NULL;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "C_p");
    ros::NodeHandle n;

    ROS_INFO("\033[1;31m----> Ready_C_p!\033[0m");

    Citizen djq = {"jianqiang", 1000, NULL};
    Citizen dxy = {"xinyi", 2000, NULL};

    cout << djq.name << endl;
    cout << djq.deposite << endl;
    cout << djq.car << endl;

    cout << dxy.name << endl;
    cout << dxy.deposite << endl;
    cout << dxy.car << endl;

    buy_car(&djq);
    cout << djq.name << endl;
    cout << djq.deposite << endl;
    cout << djq.car->brand << endl;
    cout << djq.car->price << endl;

//    discard_car(&djq);
//    cout << djq.name << endl;
//    cout << djq.deposite << endl;
//    cout << djq.car << endl;

    sell_car(&djq, &dxy);
    cout << djq.name << endl;
    cout << djq.deposite << endl;
    cout << djq.car << endl;

    cout << dxy.name << endl;
    cout << dxy.deposite << endl;
    cout << dxy.car << endl;


    return 0;
}
