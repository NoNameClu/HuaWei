#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include "math.h"
using namespace std;

using VectorXd = tuple<double, double, double, double, double>;
using Vector2d = pair<double, double>;

class DWA
{
    double dt;                  //采样时间
    double v_min, v_max, w_min, w_max; //线速度角速度边界
    double predict_time;        //轨迹推算时间长度
    double a_vmax, a_wmax;      //线加速度和角加速度最大值
    double v_sample, w_sample;  //采样分辨率
    double alpha, beta, gamma;  //轨迹评价函数系数
    double radius;              // 用于判断是否到达目标点
    double judge_distance;      //若与障碍物的最小距离大于阈值（例如这里设置的阈值为robot_radius+0.2）,则设为一个较大的常值

    vector<double> calVelLimit();
    vector<double> calAccelLimit(double v, double w);
    vector<double> calObstacleLimit(const VectorXd& state, const vector<Vector2d>& obstacle);
    vector<double> calDynamicWindowVel(double v, double w, const VectorXd& state, const vector<Vector2d>& obstacle);
    double _dist(const VectorXd& state, const vector<Vector2d>& obstacle);
    vector<VectorXd> trajectoryPredict(VectorXd state, double v, double w);
    VectorXd kinematicModel(const VectorXd& state, vector<double>control, double dt);

    double _heading(const vector<VectorXd>& trajectory, Vector2d goal);
    double _velocity(const vector<VectorXd>& trajectory);
    double _distance(const vector<VectorXd>& trajectory, const vector<Vector2d>& obstacle);

    double GetLength(const pair<double, double>& lhs, const pair<double, double>& rhs);

public:
    DWA();

    pair<vector<double>, vector<VectorXd>> trajectoryEvaluation(VectorXd state, Vector2d goal, vector<Vector2d>obstacle);

    void set_var(double aVmax, double aWmax);
};

