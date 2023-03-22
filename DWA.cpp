#include "DWA.h"
#define DEBUG

DWA::DWA()
    : dt(0.1), v_min(0), v_max(6), w_min(-M_PI), w_max(M_PI), predict_time(0.5),
    v_sample(0.2), w_sample(M_PI / 5), alpha(0.8), beta(0.1), gamma(0.1),
    judge_distance(10000) {}

vector<double> DWA::calVelLimit() {
    return { v_min,v_max,w_min,w_max };
}

/**
 * ������ٶ�����Vd
 * @return
 */
vector<double> DWA::calAccelLimit(double v, double w) {
    double v_low = v - a_vmax * dt;
    double v_high = v + a_vmax * dt;
    double w_low = w - a_wmax * dt;
    double w_high = w + a_wmax * dt;
    return { v_low, v_high,w_low, w_high };
}

/**
 * �����ϰ�������Va
 * @param state ��ǰ������״̬
 * @param obstacle �ϰ���λ��
 * @return �ƶ������˲�����Χ�ϰ��﷢����ײ���ٶȿռ�Va
 */
vector<double> DWA::calObstacleLimit(const VectorXd& state, const vector<Vector2d>& obstacle) {
    double v_low = v_min;
    double v_high = sqrt(2 * _dist(state, obstacle) * a_vmax);
    double w_low = w_min;
    double w_high = sqrt(2 * _dist(state, obstacle) * a_wmax);
    return { v_low, v_high,w_low, w_high };
}

/**
 * �ٶȲ���,�õ��ٶȿռ䴰��
 * @param v ��ǰʱ�����ٶ�
 * @param w ��ǰʱ�̽��ٶ�
 * @param state ��ǰ������״̬
 * @param obstacle �ϰ���λ��
 * @return [v_low,v_high,w_low,w_high]: ���ղ�������ٶȿռ�
 */
vector<double> DWA::calDynamicWindowVel(double v, double w, const VectorXd& state, const vector<Vector2d>& obstacle) {


    vector<double> Vm = calVelLimit();
    vector<double> Vd = calAccelLimit(v, w);
    vector<double> Va = calObstacleLimit(state, obstacle);
    double a = max({ Vm[0],Vd[0],Va[0] });
    double b = min({ Vm[1],Vd[1],Va[1] });
    double c = max({ Vm[2], Vd[2],Va[2] });
    double d = min({ Vm[3], Vd[3],Va[3] });
    return { a,b,c,d };
}



/**
 * ���㵱ǰ�ƶ������˾����ϰ�������ļ��ξ���
 * @param state ��ǰ������״̬
 * @param obstacle �����ϰ���λ��
 * @return �ƶ������˾����ϰ�������ļ��ξ���
 */
double DWA::_dist(const VectorXd& state, const vector<Vector2d>& obstacle) {
    double min_dist = 100000;
    pair<double, double> cur = make_pair(get<0>(state), get<1>(state));
    for (const Vector2d& obs : obstacle) {
        double distance = GetLength(obs, cur);
        min_dist = min(min_dist, distance);
    }
    return min_dist;
}

/**
 * �������˶�ѧģ��
 * @param state ״̬��---x,y,face,v,w
 * @param control ������---v,w,���ٶȺͽ��ٶ�
 * @param dt ����ʱ��
 * @return ��һ����״̬
 */
VectorXd DWA::kinematicModel(const VectorXd& st, vector<double> control, double dt) {
    VectorXd state;
    get<0>(state) += control[0] * sin(get<2>(st)) * dt;
    get<1>(state) += control[0] * cos(get<2>(st)) * dt;
    get<2>(state) += control[1] * dt;
    get<3>(state) = control[0];
    get<4>(state) = control[1];
    return state;
}

/**
 * �켣����
 * @param state ��ǰ״̬---x,y,face,v,w
 * @param v ��ǰʱ�����ٶ�
 * @param w ��ǰʱ�����ٶ�
 * @return �����Ĺ켣
 */
vector<VectorXd> DWA::trajectoryPredict(VectorXd state, double v, double w) {
    vector<VectorXd> trajectory;
    trajectory.push_back(state);
    double time = 0;
    while (time <= predict_time) {
        state = kinematicModel(state, { v,w }, dt);
        trajectory.push_back(state);
        time += dt;
#ifdef DEBUG
        
#endif // DEBUG

    }
    return trajectory;
}

/**
 * �켣���ۺ���,����Խ�ߣ��켣Խ��
 * @param state ��ǰ״̬---x,y,yaw,v,w
 * @param goal Ŀ���λ�ã�[x,y]
 * @param obstacle �ϰ���λ�ã�dim:[num_ob,2]
 * @return ���ſ����������Ź켣
 */
pair<vector<double>, vector<VectorXd>>DWA::trajectoryEvaluation(VectorXd state, Vector2d goal, vector<Vector2d> obstacle) {
    double G_max = -10000000; //��������
    vector<VectorXd> trajectory_opt; //���Ź켣
    trajectory_opt.push_back(state);
    vector<double> control_opt = { 0.,0. }; // ���ſ���
    vector<double> dynamic_window_vel = calDynamicWindowVel(get<3>(state), get<4>(state), state, obstacle);//��1��--�����ٶȿռ�

    double sum_heading = 0.0, sum_dist = 0.0, sum_vel = 0.0;//ͳ��ȫ�������켣�ĸ�������֮�ͣ��������۵Ĺ�һ��
#ifdef DEBUG
    cerr << dynamic_window_vel[0] << " " << dynamic_window_vel[1] << " " << dynamic_window_vel[2] << " " << dynamic_window_vel[3] << endl;
#endif // DEBUG

    double v = dynamic_window_vel[0];
    double w = dynamic_window_vel[2];
    while (v < dynamic_window_vel[1]) {
        w = dynamic_window_vel[2];
        while (w < dynamic_window_vel[3]) {
            vector<VectorXd> trajectory = trajectoryPredict(state, v, w);
            double heading_eval = _heading(trajectory, goal);
            double dist_eval = _distance(trajectory, obstacle);
            double vel_eval = _velocity(trajectory);
            sum_vel = max(sum_vel, vel_eval);
            sum_dist = max(sum_dist, dist_eval);
            sum_heading = max(sum_heading, heading_eval);
            w += w_sample;
        }
        v += v_sample;
    }
#ifdef DEBUG
    cerr << endl;
#endif // DEBUG


    //sum_heading = 1.0, sum_dist = 1.0, sum_vel = 1.0;//�����й�һ��
    v = dynamic_window_vel[0];
    w = dynamic_window_vel[2];
#ifdef DEBUG
    cerr << sum_heading << " " << sum_dist << " " << sum_vel << endl;
#endif // DEBUG

    while (v < dynamic_window_vel[1]) {
        while (w < dynamic_window_vel[3]) {
            vector<VectorXd>trajectory = trajectoryPredict(state, v, w);//��2��--�켣����

            double heading_eval = alpha * _heading(trajectory, goal) / sum_heading;
            double dist_eval = beta * _distance(trajectory, obstacle) / sum_dist;
            double vel_eval = gamma * _velocity(trajectory) / sum_vel;
            double G = heading_eval + dist_eval + vel_eval; // ��3��--�켣����

            if (G_max <= G) {
                G_max = G;
                trajectory_opt = trajectory;
                control_opt = { v,w };
            }
            w += w_sample;
        }
        v += v_sample;
        w = dynamic_window_vel[2];
    }
#ifdef DEBUG
    cerr << endl;
#endif // DEBUG

    return { control_opt,trajectory_opt };

}

void DWA::set_var(double aVmax, double aWmax)
{
    a_vmax = aVmax, a_wmax = aWmax;
}

/**
 * ��λ�����ۺ���
 * �����ڵ�ǰ�����ٶ��²����Ĺ켣�յ�λ�÷�����Ŀ������ߵļнǵ����
 * @param trajectory �켣��dim:[n,5]
 * @param goal Ŀ���λ��[x,y]
 * @return ��λ��������ֵ
 */
double DWA::_heading(const vector<VectorXd>& trajectory, Vector2d goal) {
    auto target_s = trajectory.back();
    pair<double, double> target = make_pair(get<0>(target_s), get<1>(target_s));
    double y = goal.second - target.second;
    double x = goal.first - target.first;
    double error_angle = atan2(y, x);
    double cost_angle = error_angle - get<2>(target_s);
    if (cost_angle > M_PI) {
        cost_angle -= 2 * M_PI;
    }
    else if (cost_angle < (-M_PI)) {
        cost_angle += 2 * M_PI;
    }
    double cost = M_PI - abs(cost_angle);
    return cost;
}

/**
 * �ٶ����ۺ���
 * ��ʾ��ǰ���ٶȴ�С��������ģ��켣ĩ��λ�õ����ٶȵĴ�С����ʾ
 * @param trajectory �켣��dim:[n,5]
 * @return �ٶ�����ֵ
 */
double DWA::_velocity(const vector<VectorXd>& trajectory) {
    return get<3>(trajectory[trajectory.size() - 1]);
}

/**
 * �������ۺ���
 * ��ʾ��ǰ�ٶ��¶�Ӧģ��켣���ϰ���֮���������룻
 * ���û���ϰ�����������������趨����ֵ����ô�ͽ���ֵ��Ϊһ���ϴ�ĳ���ֵ��
 * @param trajectory �켣��dim:[n,5]
 * @param obstacle �ϰ���λ�ã�dim:[num_ob,2]
 * @return ��������ֵ
 */
double DWA::_distance(const vector<VectorXd>& trajectory, const vector<Vector2d>& obstacle) {
    double min_r = 10000000;
    for (Vector2d obs : obstacle) {
        for (VectorXd state : trajectory) {
            pair<double, double> point = make_pair(get<0>(state), get<1>(state));
            double r = GetLength(obs, point);
            min_r = min(min_r, r);
        }
    }
    if (min_r < 3.5) {
        return min_r;
    }
    else {
        return judge_distance;
    }
}

double DWA::GetLength(const pair<double, double>& lhs, const pair<double, double>& rhs)
{
    int x = lhs.first - rhs.first, y = lhs.second - rhs.second;
    return sqrt(x * x + y * y);
}
