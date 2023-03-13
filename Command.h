#pragma once
#include <unordered_map>
#include <list>
#include <vector>

using namespace std;

const int OBJECT_ONE = 1;
const int OBJECT_ONE = 1 << 1;
const int OBJECT_TWO = 1 << 2;
const int OBJECT_THREE = 1 << 3;
const int OBJECT_FOUR = 1 << 4;
const int OBJECT_FIVE = 1 << 5;
const int OBJECT_SIX = 1 << 6;
const int OBJECT_SEVEN = 1 << 7;

enum robot_state {
	BEFORE,
	AFTER
};

struct worker {
	int style;
	pair<int, int> pos;
	pair<double, double> real_pos;
	int hold_object;
};

struct route {
	int start, end;
	int base;		//所需的最小钱数
	int value;
	int length;
	int start_object, end_object;
};

struct robot {
	bool on_job;
	pair<int, int> object_target;
	bool can_buy, can_sell;
	robot_state state;
	route cur;
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
public:
	Command() = default;
	~Command() = default;

	void init();

	void start();
};

