#include "Command.h"

const unordered_set<int> Command::style{ 1,2,3,4,5,6,7,8,9 };

void Command::init()
{
	ReadUntilOK();
	initMap();		//这一步中将两个路线队列初始化完毕。
	Add_OK();
	Response();
	Clear();
}

void Command::start()
{
	while (ReadUntilOK()) {
		UpdateInfo();
		Add_frame();
		RobotDoWork();
		RobotSelectWork();
		Add_OK();
		Response();
		Clear();
	}
}

bool Command::ReadUntilOK()
{
	string line;
	while (getline(cin, line)) {
		if (line[0] == 'O' && line[1] == 'K')
			return true;

		buf.push_back(line);
	}
	return false;
}

void Command::Clear()
{
	buf.clear();
	response.clear();
}

void Command::Add_OK()
{
	response.push_back("OK");
}

void Command::Add_frame()
{
	response.push_back(to_string(frame));
}

void Command::Response()
{
	for (const auto& res : response) {
		cout << res << endl;
	}
}

void Command::initMap()
{
	//循环地图数据，先保存下所有工作台和机器人的初始位置，此处默认缓冲区buf末尾没有ok。
	for (int i = 0; i < buf.size(); ++i) {
		for (int j = 0; j < buf[0].size(); ++j) {
			if (buf[i][j] == 'A') {		// 机器人
				robot temp;
				temp.state = NONE;
				robots.push_back(temp);
			}
			if (style.find(buf[i][j] - '0') != style.end()) {	// 机器人
				worker temp;
				pair<int, int> pos = make_pair(i, j);	//	工作台int坐标
				pair<double, double> real_pos;
				mapToreal(pos, real_pos);				//	int坐标转double实际坐标
				temp.pos = pos;
				temp.real_pos = real_pos;
				temp.style = buf[i][j] - '0';
				switch (temp.style) {
				case 1:
					temp.need_money = 3000;
					temp.sell_money = 6000;
					temp.need_object = OBJECT_NULL;
					temp.product_object = OBJECT_ONE;
					break;
				case 2:
					temp.need_money = 4400;
					temp.sell_money = 7600;
					temp.need_object = OBJECT_NULL;
					temp.product_object = OBJECT_TWO;
					break;
				case 3:
					temp.need_money = 5800;
					temp.sell_money = 9200; 
					temp.need_object = OBJECT_NULL;
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
	double max_value = 0;
	for (auto& p : idToworker) {
		int start_id = p.first;
		worker start = p.second;
		for (auto& q : idToworker) {
			int end_id = q.first;
			worker end = q.second;
			if (start_id == end_id) {
				continue;
			}
			if ((start.product_object & end.need_object) == 0) {	//	前面生产的是后面需要的，此路线才有效
				continue;
			}
			route temp(start_id, end_id);
			temp.base = start.need_money;
			temp.value = start.sell_money - start.need_money;
			if (end.style >= 4 && end.style <= 6) {
				temp.value += (end.sell_money - end.need_money) / 2;
			}
			else if(end.style ) {
				temp.value += (end.sell_money - end.need_money) / 3;
			}
			max_value = max(temp.value, max_value);
			temp.object = start.product_object;
			temp.length = GetLength(start.real_pos, end.real_pos);
			temp.stat = NO_PRODUCT;
			unavaliable.push_back(temp);
		}
	}

	//先对收益归一化
	for (auto& rou : unavaliable) {
		rou.value /= max_value;		//0 - 1 小数
	}
}

void Command::UpdateInfo()
{
	string a;
	stringstream sa;

	//	读第一行
	int index = 0;
	a = buf[index++];
	sa << a;
	sa >> frame;
	int m_temp;
	sa >> m_temp;
	if (m_temp != money) {
		stat = WRONG;
		Clean_list();
	}
	money = m_temp;
	flush_money_stat();

	//	读第二行
	a = buf[index++];
	sa << a;
	sa >> worker_num;

	int n = worker_num;
	while (n--) {
		//	读第3到n+3行
		a = buf[index++];
		sa << a;

		worker tmp;
		sa >> tmp.style;
		pair<double, double> real_pos;
		sa >> real_pos.first;
		sa >> real_pos.second;
		tmp.real_pos = real_pos;	//	坐标转化
		mapToreal(tmp.pos, tmp.real_pos); 

		int time;
		sa >> time;		// 工作台剩余的生产时间（没有用到）
		tmp.time = time;
		sa >> tmp.hold_object;
		sa >> tmp.output;
		int id = tmp.pos.first * 100 + tmp.pos.second;
		worker tmp2 = idToworker[id];	// tmp2记录了此工作台上一帧的状态，但还没有与当前帧比较判错
		if (tmp.output) {
			takeoff_product_stat(id);
		}
		idToworker[id] = tmp;
	}

	for (int i = 0; i < 4; i++) {
		a = buf[index++];
		sa << a;

		int workerID;
		sa >> workerID;
		int item;
		sa >> item;

		double time_weight, collide_weight;		// 时间价值系数和碰撞价值系数，没有用上
		sa >> time_weight;
		sa >> collide_weight;

		double a_speed,face;
		pair<double, double> l_speed, real_pos;
		sa >> a_speed >> l_speed.first >> l_speed.second;
		sa >> face;
		sa >> real_pos.first >> real_pos.second;
		
		//robots[i].on_job = item > 0;	// 携带物品编号大于0就表示在工作中
		robots[i].face = face;			// 更新机器人信息
		robots[i].a_speed = a_speed;
		robots[i].l_speed = l_speed;
		robots[i].real_pos = real_pos;
		if (robots[i].on_job) {
			if (robots[i].state == BEFORE && isNear(robots[i].real_pos, idToworker[robots[i].cur.start].real_pos, 0.45)) {
				robots[i].can_buy = true;
			}
			if (robots[i].state == AFTER && isNear(robots[i].real_pos, idToworker[robots[i].cur.end].real_pos, 0.53)) {
				robots[i].can_sell = true;
			}
		}
	}

	takeoff_need_stat();
	flush_list();
}

void Command::RobotDoWork()
{
	if (stat == WRONG) {	//	状态错误时，清空机器人物品
		for (int i = 0; i < robots.size(); ++i) {
			string first = "", second = "", third = "";
			first += "forward ";
			first += to_string(i);
			first += " ";
			first += to_string(0);
			second += "rotate ";
			second += to_string(i);
			second += " ";
			second += to_string(0);
			third += "destroy ";
			third += to_string(i);
			response.push_back(first);
			response.push_back(second);
			response.push_back(third);
		}
		return;
	}

	worker next;
	string fd, ro;
	//循环判断哪些robot已经被选择了路线
	for (int i = 0; i < robots.size(); ++i) {
		robot rt = robots[i];
		if (!rt.on_job) {
			continue;
		}
		if (rt.can_buy && rt.state == BEFORE) {	//	能购买（靠近目标工作台）
			string temp = "";
			temp += "buy ";
			temp += to_string(i);
			response.push_back(temp);

			rt.state = AFTER;
			rt.can_buy = false;
			money -= rt.cur.base;

			puton_product_stat(rt.cur.start);
		}

		if (rt.can_sell && rt.state == AFTER) {	//	能出售
			string temp = "";
			temp += "sell ";
			temp += to_string(i);
			response.push_back(temp);

			rt.state = NONE;
			rt.can_sell = false;
			rt.on_job = false;
			money += rt.cur.value + rt.cur.base;
			
			puton_need_stat(rt.cur.end, rt.cur.object);
		}

		switch (rt.state) {
		case BEFORE:
			next = idToworker[rt.cur.start];
			break;
		case AFTER:
			next = idToworker[rt.cur.end];
			break;
		case NONE:
			fd += "forward ";
			fd += to_string(i);
			fd += " ";
			fd += to_string(0);
			ro += "rotate ";
			ro += to_string(i);
			ro += " ";
			ro += to_string(0);
			response.push_back(fd);
			response.push_back(ro);
			return;
		}

		//找到了目标
		double speed, angle;
		double x = next.real_pos.second - rt.real_pos.second,
			y = next.real_pos.first - rt.real_pos.first;
		double target_angle = atan2(y, x);	//	计算目标角度与x正半轴夹角
		double a_diff = target_angle + (-rt.face);	//	计算目标角度与朝向的差值
		if (a_diff > M_PI) {
			a_diff -= 2 * M_PI;
		}
		else if (a_diff < (-M_PI)) {
			a_diff += 2 * M_PI;
		}
		int dir = a_diff > 0 ? 1 : -1;
		a_diff = abs(a_diff);
		//当机器人和目标地点偏角很大
		if (a_diff >= M_PI_2) {
			speed = 0.5;
			angle = M_PI * dir;
		}
		//当机器人和目标地点偏角较大
		else if(a_diff >= M_PI_8) {
			speed = 3;
			angle = M_PI_4 * dir;
		}
		//做微调  
		else if(a_diff >= M_PI_32) {
			speed = 5;
			angle = M_PI_16 * dir;
		}
		//前进
		else {
			speed = 6;
			angle = 0;
		}

		fd += "forward ";
		fd += to_string(i);
		fd += " ";
		fd += to_string(speed);
		ro += "rotate ";
		ro += to_string(i);
		ro += " ";
		ro += to_string(angle);
		response.push_back(fd);
		response.push_back(ro);
	}

	flush_money_stat();
	flush_list();
}

void Command::RobotSelectWork()
{
	for (int i = 0; i < robots.size(); i++) {
		//判断当前机器人是否有工作
		//拿到当前机器人的坐标，
		//遍历可选的路线，统计可选路线各个的长度
		//first：总长度，second：路线
		robot rb = robots[i];
		if (rb.state != NONE) continue;	// 当前机器人没有分配工作

		pair<double, double> pos = rb.real_pos;
		pair<double, route> ;

		//	寻找maxValue和maxDistance，方便归一化以及正向化
		double maxValue = 0, maxDistance = 0;
		for (auto &cur_route:avaliable) {
			worker route_start_worker = idToworker[cur_route.start];
			pair<double, double> cur_worker_pos = route_start_worker.real_pos;
			double distance = GetLength(rb.real_pos, cur_worker_pos);	//	计算机器人到起始点距离
			distance += cur_route.length;
			maxValue = max(maxValue, cur_route.value);
			maxDistance = max(maxDistance,distance);
		}
		//	将归一化和正向化之后的指标加权赋分给路线，然后放进priority_queue
		priority_queue<pair<double,route>> pq;	//	！这边以double升序
		for (auto& cur_route : avaliable) {
			worker route_start_worker = idToworker[cur_route.start];
			pair<double, double> cur_worker_pos = route_start_worker.real_pos;
			double distance = GetLength(rb.real_pos, cur_worker_pos);	//	计算机器人到起始点距离
			distance += cur_route.length;
			distance = distance / maxDistance;	//路线越小越好，这是一个极小型指标
			maxValue = 1 - cur_route.value / maxValue;	// 价值越大越好，这一个极大型指标，需要正向化
			double score = distance * 0.5 + maxValue * 0.5;	//	权重都置0.5
			pq.push(make_pair(score, cur_route));	//选中的路线给它评分，越小越好，进堆	
		}
		

		pair<double, route> my_route = pq.top();
		pq.pop();

		//你取出来了这个路线
		//start,end
		//刷新路线
		puton_occ_stat(my_route.second.start);
		puton_occ_stat(my_route.second.end, my_route.second.object);
		flush_list();
		rb.cur = my_route.second; 
	}
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

//	算两个点之间的距离
double Command::GetLength(const pair<double, double>& lhs, const pair<double, double>& rhs) {
	double x = lhs.first - rhs.first, y = lhs.second - rhs.second;
	return sqrt(x * x + y * y);
}

bool Command::isNear(const pair<double, double>& lhs, const pair<double, double>& rhs, double dis)
{
	if (GetLength(lhs, rhs) <= dis) {
		return true;
	}
	return false;
}

//	清除avaliable，重置unavaliable
void Command::Clean_list() {
	for (auto p = avaliable.begin(); p != avaliable.end();) {
		unavaliable.insert(unavaliable.end(), *p);
		p = avaliable.erase(p);
	}
	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		p->stat = NO_PRODUCT;
	}
}

//	更新被选中的路线
void Command::flush_list() {
	for (auto p = avaliable.begin(); p != avaliable.end();) {
		if (p->stat != 0) {
			unavaliable.insert(unavaliable.end(), *p);
			p = avaliable.erase(p);
		}
		else {
			++p;
		}
	}

	for (auto p = unavaliable.begin(); p != unavaliable.end();) {
		if (p->stat == 0) {
			avaliable.insert(avaliable.end(), *p);
			p = unavaliable.erase(p);
		}
		else {
			++p;
		}
	}
}

void Command::flush_money_stat() {
	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		if (p->base >= money) {
			p->stat &= (~NO_MONEY);
		}
		if (p->base < money) {
			p->stat |= NO_MONEY;
		}
	}
	for (auto p = avaliable.begin(); p != avaliable.end(); ++p) {
		if (p->base < money) {
			p->stat |= NO_MONEY;
		}
	}
}

void Command::takeoff_product_stat(int id) {
	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		if (p->start == id) {
			p->stat &= (~NO_PRODUCT);
		}
	}
}

void Command::puton_product_stat(int id) {
	for (auto p = avaliable.begin(); p != avaliable.end(); ++p) {
		if (p->start == id) {
			p->stat &= (~OCC);
			p->stat |= NO_PRODUCT;
		}
	}

	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		if (p->start == id) {
			p->stat &= (~OCC);
			p->stat |= NO_PRODUCT;
		}
	}
}

//start版本
void Command::puton_occ_stat(int id) {
	for (auto p = avaliable.begin(); p != avaliable.end(); ++p) {
		if (p->start == id) {
			p->stat |= OCC;
		}
	}

	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		if (p->start == id) {
			p->stat |= OCC;
		}
	}
}

//end版本
void Command::puton_occ_stat(int id, int object) {
	for (auto p = avaliable.begin(); p != avaliable.end(); ++p) {
		if (p->end == id && p->object == object) {
			p->stat |= OCC;
		}
	}

	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		if (p->end == id && p->object == object) {
			p->stat |= OCC;
		}
	}
}

void Command::puton_need_stat(int id, int object) {
	for (auto p = avaliable.begin(); p != avaliable.end(); ++p) {
		if (p->end == id && p->object == object) {
			p->stat &= (~OCC);
			p->stat |= NO_NEED;
		}
	}

	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		if (p->end == id && p->object == object) {
			p->stat &= (~OCC);
			p->stat |= NO_NEED;
		}
	}
}

void Command::takeoff_need_stat() {
	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		if ((idToworker[p->end].hold_object & p->object) == 0) {
			p->stat &= (~NO_NEED);
		}
	}
}