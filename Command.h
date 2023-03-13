#pragma once
#include <unordered_map>
#include <unordered_set>
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
	//���none״̬����ʾ����û�з��乤��
	NONE,			
	BEFORE,
	AFTER
};

struct worker {
	int style;
	int hold_object;
	int need_object;					//���need����ʾ����Ҫ�ģ�����ı�ʾ���е�
	int need_money, sell_money;			//�ù���̨�������Ʒ����۸���ۼ۸�

	pair<int, int> pos;
	pair<double, double> real_pos;

	worker() : hold_object(0) {};
};

struct route {
	int start, end;
	int base;					//�������СǮ��
	int value;					//����һ��ʼ��Ҫ����һ������
	int length;
	int start_object, end_object;

	route() {};
	route(int s, int e) : start(s), end(e) {};
};

struct robot {
	bool on_job;
	bool can_buy, can_sell;
	pair<int, int> object_target;
	robot_state state;
	route cur;
	
	double a_speed;						//���ٶ�
	pair<double, double> l_speed;		//���ٶ�
	pair<double, double> real_pos;		//�����˵ĵ�ǰ���꣬ÿһ֡��ȡ��ʱ��Ҫ�ı�

	robot() : on_job(false), can_buy(false), can_sell(false),
		state(NONE), a_speed(0), l_speed(make_pair(0, 0)) {};

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

	const static unordered_set<int> style;	//��������������Ҫ���ڶ���ͼ������ʱ��

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

