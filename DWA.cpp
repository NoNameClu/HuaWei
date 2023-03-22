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
 * 计算加速度限制Vd
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
 * 环境障碍物限制Va
 * @param state 当前机器人状态
 * @param obstacle 障碍物位置
 * @return 移动机器人不与周围障碍物发生碰撞的速度空间Va
 */
vector<double> DWA::calObstacleLimit(const VectorXd& state, const vector<Vector2d>& obstacle) {
    double v_low = v_min;
    double v_high = sqrt(2 * _dist(state, obstacle) * a_vmax);
    double w_low = w_min;
    double w_high = sqrt(2 * _dist(state, obstacle) * a_wmax);
    return { v_low, v_high,w_low, w_high };
}

/**
 * 速度采样,得到速度空间窗口
 * @param v 当前时刻线速度
 * @param w 当前时刻角速度
 * @param state 当前机器人状态
 * @param obstacle 障碍物位置
 * @return [v_low,v_high,w_low,w_high]: 最终采样后的速度空间
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
 * 计算当前移动机器人距离障碍物最近的几何距离
 * @param state 当前机器人状态
 * @param obstacle 所有障碍物位置
 * @return 移动机器人距离障碍物最近的几何距离
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
 * 机器人运动学模型
 * @param state 状态量---x,y,face,v,w
 * @param control 控制量---v,w,线速度和角速度
 * @param dt 采样时间
 * @return 下一步的状态
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
 * 轨迹推算
 * @param state 当前状态---x,y,face,v,w
 * @param v 当前时刻线速度
 * @param w 当前时刻线速度
 * @return 推算后的轨迹
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
 * 轨迹评价函数,评价越高，轨迹越优
 * @param state 当前状态---x,y,yaw,v,w
 * @param goal 目标点位置，[x,y]
 * @param obstacle 障碍物位置，dim:[num_ob,2]
 * @return 最优控制量、最优轨迹
 */
pair<vector<double>, vector<VectorXd>>DWA::trajectoryEvaluation(VectorXd state, Vector2d goal, vector<Vector2d> obstacle) {
    double G_max = -10000000; //最优评价
    vector<VectorXd> trajectory_opt; //最优轨迹
    trajectory_opt.push_back(state);
    vector<double> control_opt = { 0.,0. }; // 最优控制
    vector<double> dynamic_window_vel = calDynamicWindowVel(get<3>(state), get<4>(state), state, obstacle);//第1步--计算速度空间

    double sum_heading = 0.0, sum_dist = 0.0, sum_vel = 0.0;//统计全部采样轨迹的各个评价之和，便于评价的归一化
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


    //sum_heading = 1.0, sum_dist = 1.0, sum_vel = 1.0;//不进行归一化
    v = dynamic_window_vel[0];
    w = dynamic_window_vel[2];
#ifdef DEBUG
    cerr << sum_heading << " " << sum_dist << " " << sum_vel << endl;
#endif // DEBUG

    while (v < dynamic_window_vel[1]) {
        while (w < dynamic_window_vel[3]) {
            vector<VectorXd>trajectory = trajectoryPredict(state, v, w);//第2步--轨迹推算

            double heading_eval = alpha * _heading(trajectory, goal) / sum_heading;
            double dist_eval = beta * _distance(trajectory, obstacle) / sum_dist;
            double vel_eval = gamma * _velocity(trajectory) / sum_vel;
            double G = heading_eval + dist_eval + vel_eval; // 第3步--轨迹评价

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
 * 方位角评价函数
 * 评估在当前采样速度下产生的轨迹终点位置方向与目标点连线的夹角的误差
 * @param trajectory 轨迹，dim:[n,5]
 * @param goal 目标点位置[x,y]
 * @return 方位角评价数值
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
 * 速度评价函数
 * 表示当前的速度大小，可以用模拟轨迹末端位置的线速度的大小来表示
 * @param trajectory 轨迹，dim:[n,5]
 * @return 速度评价值
 */
double DWA::_velocity(const vector<VectorXd>& trajectory) {
    return get<3>(trajectory[trajectory.size() - 1]);
}

/**
 * 距离评价函数
 * 表示当前速度下对应模拟轨迹与障碍物之间的最近距离；
 * 如果没有障碍物或者最近距离大于设定的阈值，那么就将其值设为一个较大的常数值。
 * @param trajectory 轨迹，dim:[n,5]
 * @param obstacle 障碍物位置，dim:[num_ob,2]
 * @return 距离评价值
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
