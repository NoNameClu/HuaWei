#include "Command.h"

const unordered_set<int> Command::style{ 1,2,3,4,5,6,7,8,9 };

void Command::init()
{
	ReadUntilOK();
	initMap();		//这一步中将两个路线队列初始化完毕。
	Clear();
	Add_OK();
	Response();
}

void Command::start()
{
	while (ReadUntilOK()) {
		Clear();
		UpdateInfo();
		Add_frame();
		RobotDoWork();
		RobotSelectWork();
		Add_OK();
		Response();
	}
}

bool Command::ReadUntilOK()
{
}

void Command::Clear()
{
}

void Command::Add_OK()
{
}

void Command::Add_frame()
{
}

void Command::Response()
{
}

void Command::initMap()
{
	//循环地图数据，先保存下所有工作台和机器人的初始位置，此处默认缓冲区buf末尾没有ok。
	for (int i = 0; i < buf.size(); ++i) {
		for (int j = 0; j < buf[0].size(); ++j) {
			if (buf[i][j] == 'A') {
				robot temp;
				temp.state = NONE;
				robots.push_back(temp);
			}
			if (style.find(buf[i][j] - '0') != style.end()) {
				worker temp;
				pair<int, int> pos = make_pair(i, j);
				pair<double, double> real_pos;
				mapToreal(pos, real_pos);
				temp.pos = pos;
				temp.real_pos = real_pos;
				temp.style = buf[i][j] - '0';
				switch (temp.style) {
				case 1:
					temp.need_money = 3000;
					temp.sell_money = 6000;
					temp.product_object = OBJECT_ONE;
					break;
				case 2:
					temp.need_money = 4400;
					temp.sell_money = 7600;
					temp.product_object = OBJECT_TWO;
					break;
				case 3:
					temp.need_money = 5800;
					temp.sell_money = 9200; 
					temp.product_object = OBJECT_THREE;
					break;
				case 4:
					temp.need_money = 15400;
					temp.sell_money = 22500;
					temp.product_object = OBJECT_FOUR;
					temp.need_object = (OBJECT_ONE | OBJECT_TWO);
					break;
				case 5:
					temp.need_money = 17200;
					temp.sell_money = 25000;
					temp.product_object = OBJECT_FIVE;
					temp.need_object = (OBJECT_ONE | OBJECT_THREE);
					break;
				case 6:
					temp.need_money = 19200;
					temp.sell_money = 27500;
					temp.product_object = OBJECT_SIX;
					temp.need_object = (OBJECT_TWO | OBJECT_THREE);
					break;
				case 7:
					temp.need_money = 76000;
					temp.sell_money = 105000;
					temp.product_object = OBJECT_SEVEN;
					temp.need_object = (OBJECT_FOUR | OBJECT_FIVE | OBJECT_SIX);
					break;
				case 8:
					temp.need_money = 0;
					temp.sell_money = 0;
					temp.need_object = (OBJECT_SEVEN);
					break;
				case 9:
					temp.need_money = 0;
					temp.sell_money = 0;
					temp.need_object = (OBJECT_ONE | OBJECT_TWO | OBJECT_THREE | OBJECT_FOUR | OBJECT_FIVE | OBJECT_SIX | OBJECT_SEVEN);
				default:
					break;
				}
				idToworker[i * 100 + j] = temp;
			}
		}
	}

	//循环判断计算路径
	for (auto p : idToworker) {
		int start_id = p.first;
		worker start = p.second;
		for (auto q : idToworker) {
			int end_id = q.first;
			worker end = q.second;
			if (start_id == end_id) {
				continue;
			}
			if ((start.product_object & end.need_object) == 0) {
				continue;
			}
			route temp(start_id, end_id);
			temp.base = start.need_money;
			temp.value = start.sell_money - start.need_money;
			temp.object = start.product_object;
			temp.length = GetLength(start.real_pos, end.real_pos);
			unavaliable.push_back(temp);
		}
	}
}

void Command::UpdateInfo()
{
}

void Command::RobotDoWork()
{
	worker next;
	//循环判断哪些robot已经被选择了路线
	for (auto rt : robots) {
		if (!rt.on_job) {
			continue;
		}
		switch (rt.state) {
		case BEFORE:
			next = idToworker[rt.cur.start];
		case AFTER:
			next = idToworker[rt.cur.end];
		default:
			//不应该出现这种情况，若出现这种情况应当是程序逻辑有误
		}

		//找到了目标
		double speed, angle;
		double x = next.real_pos.second - rt.real_pos.second,
			y = next.real_pos.first - rt.real_pos.first;
		double target_angle = atan2(y, x);
		double a_diff = abs(target_angle - rt.face);
		if (a_diff >= M_PI_2) {
			speed = 0.5;
			angle = M_PI;
		}
		else if(a_diff >= M_PI_8) {
			speed = 3;
			angle = M_PI_4;
		}
		else if(a_diff >= M_PI_32) {
			speed = 5;
			angle = M_PI_8;
		}
		else {
			speed = 6;
			angle = 0;
		}


	}
}

void Command::RobotSelectWork()
{
}

void Command::mapToreal(pair<int, int> old, pair<double, double>& ret)
{
	ret.first = static_cast<double>(old.first) / 2 + 0.25;
	ret.second = static_cast<double>(old.second) / 2 + 0.25;
}

void Command::realTomap(pair<double, double> old, pair<int, int>& ret)
{
	ret.first = (old.first - 0.25) * 2;
	ret.second = (old.second - 0.25) * 2;
}

double Command::GetLength(const pair<double, double>& lhs, const pair<double, double>& rhs) {
	double x = lhs.first - rhs.first, y = lhs.second - rhs.second;
	return sqrt(x * x + y * y);
}
