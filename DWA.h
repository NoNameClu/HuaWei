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
    double dt;                  //����ʱ��
    double v_min, v_max, w_min, w_max; //���ٶȽ��ٶȱ߽�
    double predict_time;        //�켣����ʱ�䳤��
    double a_vmax, a_wmax;      //�߼��ٶȺͽǼ��ٶ����ֵ
    double v_sample, w_sample;  //�����ֱ���
    double alpha, beta, gamma;  //�켣���ۺ���ϵ��
    double radius;              // �����ж��Ƿ񵽴�Ŀ���
    double judge_distance;      //�����ϰ������С���������ֵ�������������õ���ֵΪrobot_radius+0.2��,����Ϊһ���ϴ�ĳ�ֵ

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

