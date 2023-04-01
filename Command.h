#ifndef COMMAND
#define COMMAND
#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <list>
#include <vector>
#include <string.h>
#include <cmath>
#include <sstream>

#include "math.h"

using namespace std;

const int OBJECT_NULL = 1;	// �޸� ������
const int OBJECT_ONE = 1 << 1;
const int OBJECT_TWO = 1 << 2;
const int OBJECT_THREE = 1 << 3;
const int OBJECT_FOUR = 1 << 4;
const int OBJECT_FIVE = 1 << 5;
const int OBJECT_SIX = 1 << 6;
const int OBJECT_SEVEN = 1 << 7; 

const int OK = 0;
const int NO_MONEY = 1;
const int NO_PRODUCT = 1 << 1;
const int NO_NEED = 1 << 2;
const int OCC_S = 1 << 3;
const int OCC_E = 1 << 4;

const double lengthOneFrame = 0.15;
const double lastbuyselect = 0.10;
const int LAST_SELL_T = 8000;
const double OVER = 0.5;
const double coll_frame = 9;

const double VALUE_WEIGHT = 0.2;
const double LENGTH_WEIGHT = 0.8;
const double MID_WEIGHT = 0.0;
const double FIN_WEIGHT = 0.0;
const double SIDE_SPEED = 0.1;
const double COLL_RADIUS = 3.5;
const double COLL_ANGLE = M_PI_8;
const double OUTLINE_RADIUS = M_PI_6;
const double OBCMINDIS = 0.883553390593273762;

const vector<vector<int>> dic{ {1,0},{0,1},{-1,0},{0,-1} };

enum robot_state {
	//���none״̬����ʾ����û�з��乤��
	NONE,
	BEFORE,
	AFTER
};

struct worker {
	int style;
	int hold_object;
	int need_object;					//���need����ʾ����Ҫ�ģ�����ı�ʾ���е�
	int product_object;					//������Ʒ
	int need_money, sell_money;			//�ù���̨�������Ʒ����۸���ۼ۸�
	int time;							//ʣ��ʱ��
	bool output;						//��û�в���

	pair<int, int> pos;
	pair<double, double> real_pos;

	worker() : hold_object(0) {};
};

struct route {
	int robot;						//��ǵ�ǰ���ĸ�
	int start, end;
	int base;						//�������СǮ��
	double value;					//����һ��ʼ��Ҫ����һ������
	double length;
	int object;						//��ǰ·�߲�������Ʒ
	int stat;						//��ʾ��ǰ·�ߵ�״̬
	vector<int> line;				//·������

	route() = default;
	route(int s, int e) : start(s), end(e) {};
};

struct robot {
	bool on_job;
	bool can_buy, can_sell;
	bool on_coll;						//���ڶ����ײ
	bool on_side, hold_some;						//�߽���Χ
	int coll_count;
	int coll_num_wait, coll_num_hide;	//��¼��ǰ���ڸ�˭��·
	pair<double, double> object_target;
	vector<int> before_way, after_way;
	robot_state state;
	route cur;

	double face;						//����
	double a_speed;						//���ٶ�
	double coll_angle;					//������ת��
	pair<double, double> l_speed;		//���ٶ�
	pair<double, double> real_pos;		//�����˵ĵ�ǰ���꣬ÿһ֡��ȡ��ʱ��Ҫ�ı�

	robot() : on_job(false), can_buy(false), can_sell(false),
		state(NONE), face(0), a_speed(0), l_speed(make_pair(0, 0)),
		coll_num_wait(0), coll_num_hide(0), on_coll(false) {};

};

class Command
{
	enum Command_stat {
		NORMAL,
		MIDP_OVER,
		FIN_OVER
	};

	int money;
	int worker_num;
	int frame;		//֡
	int total_num;
	int mid_count;
	int seven_need = OBJECT_NULL;
	int min_product, min_need;
	unordered_map<int, int> se_Need;
	Command_stat stat;
	list<route> avaliable, unavaliable, maybe_avaliable;		//����·�ߣ�������·��, ���ܿ��õ�·�ߣ�ֻ����Ϊ��Ʒû�������²����õĶ��У�
	vector<robot> robots;										//����������
	vector<string> buf, response;								//���뻺���������������
	vector<int> robots_coll_map;								//�����˺Ͷ�Ӧ�赲�Ļ�����ӳ��
	unordered_map<int, worker> idToworker;						//id����worker��������12,13,1213�� 45.75 49.25   (x - 0.25) / 0.5
	vector<pair<int, double>> forward_s, angle_s;				//������
	vector<string> map;
	vector<int> obcTot;											//���������ϰ�������

	const static unordered_set<int> style;	//��������������Ҫ���ڶ���ͼ������ʱ��

	bool ReadUntilOK();
	void Clear();
	void Add_OK();		//
	void Add_frame();	//
	void Add_work();
	void Response();

	void initMap();		//��
	void UpdateInfo();	//Ҷ
	void RobotDoWork();		//��
	void RobotSelectWork();	//Ҷ
	void RobotColl();
	bool GetRoute(const robot&, route&, int id, const unordered_map<int, double>&);
	void caculate_nextWay(robot& rb, bool is_before);

	void mapToreal(pair<int, int>, pair<double, double>&);						//	����ת��
	void realTomap(pair<double, double>, pair<int, int>&);
	double GetLength(const pair<double, double>&, const pair<double, double>&);	//	�������㳤��
	bool isNear(const pair<double, double>&, const pair<double, double>&, double);
	bool IsOnmyway(const robot& target, const robot& check, double pi);
	void normal_caculate(const pair<double, double>& target, const pair<double, double>& cur, const double& face, double& speed, double& angle);
	void coll_angle_caculate(const pair<double, double>& target, const pair<double, double>& cur, const double& t_face, const double& c_face, double& angle, double base);
	bool route_caculate(const route& cur_route, const robot& rb, const unordered_map<int, double>& accessible, double& maxValue, double& distance, double& score, bool is_first, int id);
	bool can_select(const route& cur, const unordered_map<int, double>&);
	bool closeToside(const robot& cur);
	bool outline_check(const robot& target, const robot& check, double pi);
	bool worker_avaliable(int x, int y);
	bool is_range(int x, int y);
	void Get_acc(const robot& rb, unordered_map<int, double>& accessible);
	void get_closePoint(const robot& rb, int& x, int& y);
	bool is_noneObc(int id, const robot& rb);
	bool test_side(int step, int x, int y);
	vector<int> can_reach(const worker& start, const worker& end, double& distance);
	vector<int> get_way(int id, const robot& rb);
	vector<int> BFS(const pair<int, int>& start, const pair<int, int>& end, double& distance);

	void Clean_list();
	void flush_list();
	void flush_money_stat();
	void takeoff_product_stat(int id);
	void takeoff_need_stat();
	void puton_product_stat(int id);
	void puton_occ_stat(int id);
	void puton_occ_stat(int id, int object);
	void puton_need_stat(int id, int object);
public:
	Command() = default;
	~Command() = default;

	void init();

	void start();
};

#endif // COMMAND