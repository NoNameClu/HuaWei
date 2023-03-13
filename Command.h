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
	int base;		//�������СǮ��
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
	int frame;		//֡
	Command_stat stat;
	list<route> avaliable, unavaliable;		//����·�ߣ�������·��
	vector<robot> robots;					//����������
	vector<string> buf, response;			//���뻺���������������
	unordered_map<int, worker> idToworker;	//id����worker��������12,13,1213�� 45.75 49.25   (x - 0.25) / 0.5

	bool ReadUntilOK();		
	void Clear();
	void Add_OK();		//
	void Add_frame();	//
	void Response();

	void initMap();		//��
	void UpdateInfo();	//Ҷ
	void RobotDoWork();		//��
	void RobotSelectWork();	//Ҷ

	void mapToreal(pair<int, int>, pair<double, double>&);
	void realTomap(pair<double, double>, pair<int, int>&);
public:
	Command() = default;
	~Command() = default;

	void init();

	void start();
};

