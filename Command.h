#pragma once
#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <list>
#include <vector>
#include <string>
#include "math.h"
#include<sstream>

using namespace std;

const int OBJECT_NULL = 1;	// 修改 重名了
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
const int OCC = 1 << 3;

enum robot_state {
	//添加none状态，表示现在没有分配工作
	NONE,			
	BEFORE,
	AFTER
};

struct worker {
	int style;
	int hold_object;
	int need_object;					//添加need，表示所需要的，上面的表示持有的
	int product_object;					//生产物品
	int need_money, sell_money;			//该工作台制造的物品的买价格和售价格
	int time;							//剩余时间
	bool output;						//有没有产物

	pair<int, int> pos;
	pair<double, double> real_pos;

	worker() : hold_object(0) {};
};

struct route {
	int start, end;
	int base;						//所需的最小钱数
	double value;					//这里一开始就要做归一化操作
	int length;
	int object;						//当前路线操作的物品
	int stat;						//表示当前路线的状态

	route() {};
	route(int s, int e) : start(s), end(e) {};
};

struct robot {
	bool on_job;
	bool can_buy, can_sell;
	pair<int, int> object_target;
	robot_state state;
	route cur;
	
	double face;						//朝向
	double a_speed;						//角速度
	pair<double, double> l_speed;		//线速度
	pair<double, double> real_pos;		//机器人的当前坐标，每一帧读取的时候要改变

	robot() : on_job(false), can_buy(false), can_sell(false),
		state(NONE), face(0), a_speed(0), l_speed(make_pair(0, 0)) {};

};

class Command
{
	enum Command_stat {
		NORMAL,
		WRONG
	};

	int money;
	int worker_num;
	int frame;		//帧
	Command_stat stat;
	list<route> avaliable, unavaliable;		//可用路线，不可用路线
	vector<robot> robots;					//机器人数组
	vector<string> buf, response;			//读入缓冲区，输出缓冲区
	unordered_map<int, worker> idToworker;	//id，到worker的索引，12,13,1213。 45.75 49.25   (x - 0.25) / 0.5

	const static unordered_set<int> style;	//添加这个常量，主要用在读地图操作的时候

	bool ReadUntilOK();		
	void Clear();
	void Add_OK();		//
	void Add_frame();	//
	void Response();

	void initMap();		//我
	void UpdateInfo();	//叶
	void RobotDoWork();		//我
	void RobotSelectWork();	//叶

	void mapToreal(pair<int, int>, pair<double, double>&);
	void realTomap(pair<double, double>, pair<int, int>&);
	double GetLength(const pair<double, double>&, const pair<double, double>&);

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

