#include "Command.h"

//#define DEBUG

const unordered_set<int> Command::style{ 1,2,3,4,5,6,7,8,9 };

const unordered_set<int> id1{ 4998, 101, 9801,
							4034, 4331,
							5840,
							5228,
							4637, 5237, 4937 };

const unordered_set<int> id2{ 4998, 9898, 101, 4901 };

const unordered_set<int> id3{ 8239, 9539, 9452, 8265, 9465, 9841, 9855 };

const unordered_set<int> id4{ 4952, 4942, 4902 };

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
		RobotColl();
		RobotDoWork();
		Add_work();
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
	forward_s.clear();
	angle_s.clear();
	se_Need.clear();
}

void Command::Add_OK()
{
	response.push_back("OK");
}

void Command::Add_frame()
{
	response.push_back(to_string(frame));
}

void Command::Add_work()
{
	string fd = "", ro = "";
	for (const auto& [i, speed] : forward_s) {
		fd.clear();
		fd += "forward ";
		fd += to_string(i);
		fd += " ";
		fd += to_string(speed);
		response.push_back(fd);
	}
	for (const auto& [i, angle] : angle_s) {
		ro.clear();
		ro += "rotate ";
		ro += to_string(i);
		ro += " ";
		ro += to_string(angle);
		response.push_back(ro);
	}
}

void Command::Response()
{
	for (const auto& res : response) {
		cout << res << endl;
	}
	fflush(stdout);
}

void Command::initMap()
{
	//循环地图数据，先保存下所有工作台和机器人的初始位置，此处默认缓冲区buf末尾没有ok。

	total_num = 0;
	mid_count = 0;
	product[4] = 0, product[5] = 0, product[6] = 0;
	need[4] = 0, need[5] = 0, need[6] = 0;
	min_need = 10000, min_product = 10000;
	for (int j = buf.size() - 1; j >= 0; --j) {
		for (int i = 0; i < buf[0].size(); ++i) {
			if (buf[j][i] == 'A') {		// 机器人
				robot temp;
				temp.state = NONE;
				robots.push_back(temp);
			}
			if (style.find(buf[j][i] - '0') != style.end()) {	// 工作台
				worker temp;
				pair<int, int> pos = make_pair(i, buf.size() - j - 1);	//	工作台int坐标
				pair<double, double> real_pos;
				mapToreal(pos, real_pos);				//	int坐标转double实际坐标
				temp.pos = pos;
				temp.real_pos = real_pos;
				temp.style = buf[j][i] - '0';
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
				idToworker[i * 100 + (buf.size() - 1 - j)] = temp;
#ifdef DEBUG
				cerr << i * 100 + (buf.size() - 1 - j) << endl;
#endif // DEBUG

			}
		}
	}
	map_id = idToworker.size();

	if (map_id == 43) {
		for (auto p = idToworker.begin(); p != idToworker.end();) {
			if (id1.find(p->first) == id1.end()) {
				p = idToworker.erase(p);
			}
			else {
				++p;
			}
		}
	}
	else if (map_id == 50) {
		/*for (auto p = idToworker.begin(); p != idToworker.end();) {
			if (p->second.style == 4) {
				p = idToworker.erase(p);
			}
			else {
				++p;
			}
		}*/
	}
	else if (map_id == 18) {
		for (auto p = idToworker.begin(); p != idToworker.end();) {
			if (p->second.style > 3 && p->second.style < 7) {
				if (id4.find(p->first) == id4.end()) {
					p = idToworker.erase(p);
					continue;
				}
			}
			++p;
		}
	}
	else if (map_id == 25) {
		for (auto p = idToworker.begin(); p != idToworker.end();) {
			if (id2.find(p->first) != id2.end()) {
				p = idToworker.erase(p);
			}
			++p;
		}
	}

	for (const auto& [id, wo] : idToworker) {
		if (wo.style > 3 && wo.style <= 6) {
			++total_num;
		}
	}

	robots_coll_map.resize(robots.size());
	//循环判断计算路径
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
			if (map_id == 18 && end.style == 4) {
				temp.value += 100000;
			}
			else if (map_id == 25 && (start.style == 6 || end.style == 6)) {
				temp.value += 9000;
			}
			if (end.style >= 4 && end.style <= 6)
			{
				temp.value += (end.sell_money - end.need_money) * MID_WEIGHT;
			}
			else if (end.style == 7) {
				temp.value += (end.sell_money - end.need_money) * FIN_WEIGHT;
			}
			temp.object = start.product_object;
			temp.length = GetLength(start.real_pos, end.real_pos);
			temp.stat = NO_PRODUCT;
			if (map_id == 43) {
				if (end.style > 7 && start.style < 7) {
					continue;
				}
			}
			else if (map_id == 50) {
				/*if (end.style > 7 && start.style < 6) {
					continue;
				}*/
			}
			unavaliable.push_back(temp);
		}
	}
}

void Command::UpdateInfo()
{
	string a;

	//	读第一行
	int index = 0;
	a = buf[index++];
	stringstream sa;
	sa.str(a);
	sa >> frame;

#ifdef DEBUG
	cerr << frame << endl;
#endif // DEBUG


	int m_temp;
	sa >> m_temp;

	/*if (m_temp != money) {
		stat = WRONG;
		Clean_list();
	}*/

	money = m_temp;
	flush_money_stat();

	//	读第二行
	a = buf[index++];

	stringstream sb(a);
	sb >> worker_num;

	int n = worker_num;
	int over_num = 0;
	seven_need = OBJECT_NULL;
	product[4] = 0, product[5] = 0, product[6] = 0;
	min_product = 100000;
	while (n--) {
		//	读第3到n+3行
		a = buf[index++];
		stringstream s_for;
		s_for.str(a);

		worker tmp;
		s_for >> tmp.style;
		pair<double, double> real_pos;
		s_for >> real_pos.first;
		s_for >> real_pos.second;
		tmp.real_pos = real_pos;	//	坐标转化
		realTomap(tmp.real_pos, tmp.pos);
		int id = tmp.pos.first * 100 + tmp.pos.second;
		worker& temp_w = idToworker[id];

		int time;
		s_for >> time;		// 工作台剩余的生产时间（没有用到）
		temp_w.time = time;
		s_for >> temp_w.hold_object;
		s_for >> temp_w.output;

		
		// tmp2记录了此工作台上一帧的状态，但还没有与当前帧比较判错
		if (temp_w.output) {
			takeoff_product_stat(id);
		}
		if (temp_w.style > 3 && temp_w.output) {
			over_num++;
		}
		if (temp_w.style > 3 && temp_w.style < 7 && temp_w.time >= 0) {
			++product[temp_w.style];
		}
//		if (tmp.style == 7 && tmp.time < 0) {
//			int temp = (OBJECT_FOUR | OBJECT_FIVE | OBJECT_SIX) & tmp.hold_object;
//#ifdef DEBUG
//			cerr << "temp: " << temp << " " << tmp.need_object << " " << tmp.hold_object << endl;
//#endif // DEBUG
//			temp = ~temp;
//			if ((OBJECT_FOUR & temp) == 0) {
//				seven_need |= OBJECT_FOUR;
//				++se_Need[4];
//			}
//			if ((OBJECT_FIVE & temp) == 0) {
//				seven_need |= OBJECT_FIVE;
//				++se_Need[5];
//			}
//			if ((OBJECT_SIX & temp) == 0) {
//				seven_need |= OBJECT_SIX;
//				++se_Need[6];
//			}
//		}
	}

//#ifdef DEBUG
//	cerr << "info, seven_need:" << seven_need << endl;
//#endif // DEBUG

	for (const auto& [style, count] : product) {
		min_product = min(min_product, count);
	}


	if (over_num > (total_num * OVER)) {
		stat = MIDP_OVER;
	}
	else {
		stat = NORMAL;
	}
	/*if (frame >= LAST_SELL_T) {
		stat = FIN_OVER;
	}*/

	bool flag = true;

	for (int i = 0; i < robots.size(); i++) {
		double coll_frame = 9;
		if (map_id == 50) {
			coll_frame = 7;
		}
		if (map_id == 25) {
			coll_frame = 3;
		}
		a = buf[index++];
		stringstream s_for;
		s_for.str(a);

		int workerID;
		s_for >> workerID;
		int item;
		s_for >> item;

		double time_weight, collide_weight;		// 时间价值系数和碰撞价值系数，没有用上
		s_for >> time_weight;
		s_for >> collide_weight;

		double a_speed, face;
		pair<double, double> l_speed, real_pos;
		s_for >> a_speed >> l_speed.first >> l_speed.second;
		s_for >> face;
		s_for >> real_pos.first >> real_pos.second;

		//robots[i].on_job = item > 0;	// 携带物品编号大于0就表示在工作中
		robots[i].face = face;			// 更新机器人信息
		robots[i].a_speed = a_speed;
		robots[i].l_speed = l_speed;
		robots[i].real_pos = real_pos;
		robots[i].coll_angle = 0;
		robots[i].can_buy = false;
		robots[i].can_sell = false;
		robots[i].on_side = false;
		if (item != 0) {
			robots[i].hold_some = true;
		}
		else {
			robots[i].hold_some = false;
		}
		
		if (robots[i].on_job) {
			if (robots[i].state == BEFORE && isNear(robots[i].real_pos, idToworker[robots[i].cur.start].real_pos, 0.4)) {
				robots[i].can_buy = true;
			}
			if (robots[i].state == AFTER && isNear(robots[i].real_pos, idToworker[robots[i].cur.end].real_pos, 0.4)) {
				robots[i].can_sell = true;
			}
		}

		if (robots[i].on_coll && frame - robots[i].coll_count >= coll_frame) {
			robots[i].on_coll = false;
			robots_coll_map[i] = -1;
		}

		if (robots[i].state == AFTER && item == 0) {
			robots[i].state = BEFORE;
		}

		/*if (robots[i].l_speed.first != 0 || robots[i].l_speed.second != 0 || robots[i].a_speed != 0) {
			flag = false;
		}*/
	}

	if (map_id != 25) {
		for (int i = 0; i < robots.size(); ++i) {
			double distance = 999;
			int index = i;
			for (int j = 0; j < robots.size(); ++j) {
				if (i == j || robots[i].on_coll) {
					continue;
				}
				if (IsOnmyway(robots[j], robots[i], COLL_ANGLE)) {
					robots[i].on_coll = true;
					robots[i].coll_count = frame;
					robots[i].coll_num_hide |= (1 << (j + 1));
					double dis_temp = GetLength(robots[i].real_pos, robots[j].real_pos);
					if (dis_temp < distance) {
						distance = dis_temp;
						index = j;
					}
				}
				else {
					robots[i].coll_num_hide &= (~(1 << (j + 1)));
				}
				robots_coll_map[i] = distance > COLL_RADIUS ? robots_coll_map[i] : index;
			}
		}
	}

	/*for (int i = 0; i < robots.size(); ++i) {
		if (closeToside(robots[i])) {
			robots[i].on_side = true;
		}
	}*/

	/*if (flag && stat == WRONG) {
		stat = NORMAL;
	}*/

	takeoff_need_stat();
	flush_list();
}

void Command::RobotDoWork()
{
	//if (stat == WRONG) {	//	状态错误时，清空机器人物品
	//	for (int i = 0; i < robots.size(); ++i) {
	//		string first = "", second = "", third = "";
	//		first += "forward ";
	//		first += to_string(i);
	//		first += " ";
	//		first += to_string(0);
	//		second += "rotate ";
	//		second += to_string(i);
	//		second += " ";
	//		second += to_string(0);
	//		third += "destroy ";
	//		third += to_string(i);
	//		response.push_back(first);
	//		response.push_back(second);
	//		response.push_back(third);
	//		robots[i].can_buy = false;
	//		robots[i].can_sell = false;
	//	}
	//	return;
	//}


	//循环判断哪些robot已经被选择了路线
	for (int i = 0; i < robots.size(); ++i) {
		robot& rt = robots[i];

		if (!rt.on_job  || rt.on_coll) {
			/*if (map_id == 43 && !rt.on_job) {
				double speed, angle;
				pair<double, double> target = make_pair(49.25, 0.75);
				normal_caculate(target, rt.real_pos, rt.face, speed, angle);
				angle_s.push_back(make_pair(i, angle));
				if (!rt.on_side) {
					forward_s.push_back(make_pair(i, speed));
				}
				else {
					forward_s.push_back(make_pair(i, SIDE_SPEED));
				}
			}*/
			continue;
		}

		if (rt.can_buy && rt.state == BEFORE) {	//	能购买（靠近目标工作台）
			string temp = "";
			temp += "buy ";
			temp += to_string(i);
			response.push_back(temp);

			robots[i].state = AFTER;
			robots[i].can_buy = false;

			puton_product_stat(rt.cur.start);
		}

		if (rt.can_sell && rt.state == AFTER) {	//	能出售
			string temp = "";
			temp += "sell ";
			temp += to_string(i);
			response.push_back(temp);

			robots[i].state = NONE;
			robots[i].can_sell = false;
			robots[i].on_job = false;

			puton_need_stat(rt.cur.end, rt.cur.object);
		}

		switch (rt.state) {
		case BEFORE:
			rt.object_target = idToworker[rt.cur.start].real_pos;
			break;
		case AFTER:
			rt.object_target = idToworker[rt.cur.end].real_pos;
			break;
		case NONE:
			continue;
		}

		double speed, angle;
		normal_caculate(rt.object_target, rt.real_pos, rt.face, speed, angle);

		angle_s.push_back(make_pair(i, angle));
		if (!rt.on_side) {
			forward_s.push_back(make_pair(i, speed));
		}
		else {
			forward_s.push_back(make_pair(i, min(SIDE_SPEED, speed)));
		}
	}

	flush_money_stat();
	flush_list();
}

void Command::RobotSelectWork()
{
	/*if (stat == WRONG) {
		for (int i = 0; i < robots.size(); ++i) {
			robots[i].on_job = false;
			return;
		}
	}*/

	for (int i = 0; i < robots.size(); i++) {
		robot rb = robots[i];
		if (rb.on_job) continue;	// 当前机器人没有分配工作

		if (stat == MIDP_OVER && mid_count == 0) {
			stat = NORMAL;
		}
#ifdef DEBUG
		cerr << "状态 " << stat << " mid_count:" << mid_count << endl;
#endif // DEBUG
	
		route my_route;
		if (!GetRoute(rb, my_route)) {
			continue;
		}
		puton_occ_stat(my_route.start);
		puton_occ_stat(my_route.end, my_route.object);
		flush_list();
		robots[i].cur = my_route;
		robots[i].on_job = true;
		robots[i].state = BEFORE;
	}
}

void Command::RobotColl()
{
	double hold_base = M_PI, no_hold_base = M_PI_2;
	if (map_id == 18) {
		no_hold_base = M_PI_W;
		hold_base = M_PI3_2;
	}
	unordered_set<int> visit;
	for (int i = 0; i < robots.size(); ++i) {
		if (!robots[i].on_coll || visit.find(i) != visit.end()) {
			continue;
		}
#ifdef DEBUG
		cerr << "机器人" << i << "避障" << endl;
#endif // DEBUG


		double angle, speed, trash;
		int another = robots_coll_map[i];
		robot target = robots[robots_coll_map[i]], cur = robots[i];
		double base = (cur.state == AFTER ? hold_base : no_hold_base);
		coll_angle_caculate(target.real_pos, cur.real_pos, target.face, cur.face, angle, base);
		normal_caculate(cur.object_target, cur.real_pos, cur.face, speed, trash);

#ifdef DEBUG
		cerr << i << "机器人speed:" << speed << "angle:" << angle << endl;
#endif // DEBUG

		if (i == robots_coll_map[another]) {
#ifdef DEBUG
			cerr << "机器人" << another << "避障k" << endl;
#endif // DEBUG
			double d_angle = 0, d_speed = 0;
			double base = (target.state == AFTER ? hold_base : no_hold_base);
			coll_angle_caculate(cur.real_pos, target.real_pos, cur.face, target.face, d_angle, base);
			normal_caculate(target.object_target, target.real_pos, target.face, d_speed, trash);

			visit.insert(another);
			if (angle * d_angle < 0) {
				angle_s.push_back(make_pair(another, -d_angle));
#ifdef DEBUG
				cerr << i << "机器人speed:" << d_speed << "angle:" << -d_angle << endl;
#endif // DEBUG
			}
			else {
				angle_s.push_back(make_pair(another, d_angle));
#ifdef DEBUG
				cerr << i << "机器人speed:" << d_speed << "angle:" << d_angle << endl;
#endif // DEBUG
			}
			if (d_speed < 3 ) {
				if (map_id == 43 || map_id == 18) {
					d_speed = speed;
				}
				if (!target.on_side) {
					forward_s.push_back(make_pair(another, d_speed));
				}
				else {
					forward_s.push_back(make_pair(another, min(SIDE_SPEED, d_speed)));
				}
			}
		}
		angle_s.push_back(make_pair(i, angle));
		if (speed < 3) {
			if (!cur.on_side) {
				forward_s.push_back(make_pair(i, speed));
			}
			else {
				forward_s.push_back(make_pair(i, min(SIDE_SPEED, speed)));
			}
		}
		visit.insert(i);
	}
}

bool Command::GetRoute(const robot& rb, route& ret)
{
	//	寻找maxValue和maxDistance，方便归一化以及正向化
	double maxValue = 0, maxDistance = 0, score = 0;
	for (const auto& cur_route : avaliable) {
		if (!can_select(cur_route)) {
			continue;
		}
		route_caculate(cur_route, rb, maxValue, maxDistance, score, true);
	}

	for (const auto& cur_route : maybe_avaliable) {
if (!can_select(cur_route)) {
	continue;
}
if (idToworker[cur_route.start].time <= 0) {
	continue;
}
route_caculate(cur_route, rb, maxValue, maxDistance, score, true);
	}
	//	将归一化和正向化之后的指标加权赋分给路线，然后放进priority_queue

	auto cmp = [](const pair<double, route>& lhs, const pair<double, route>& rhs) {
		return lhs.first > rhs.first;
	};
	priority_queue<pair<double, route>, vector<pair<double, route>>, decltype(cmp)> pq(cmp);	//	！这边以double升序


	for (const auto& cur_route : avaliable) {
		if (!can_select(cur_route)) {
			continue;
		}
		if (!route_caculate(cur_route, rb, maxValue, maxDistance, score, false)) {
			continue;
		}
		pq.push(make_pair(score, cur_route));	//选中的路线给它评分，越小越好，进堆	
	}

	for (const auto& cur_route : maybe_avaliable) {
		if (!can_select(cur_route)) {
			continue;
		}
		if (idToworker[cur_route.start].time <= 0) {
			continue;
		}
		if (!route_caculate(cur_route, rb, maxValue, maxDistance, score, false)) {
			continue;
		}
		pq.push(make_pair(score, cur_route));	//选中的路线给它评分，越小越好，进堆	
	}

	if (pq.empty()) {
		return false;
	}

	ret = pq.top().second;
	/*worker start = idToworker[ret.start];
	se_Need[start.style] = min(se_Need[start.style] - 1, 0);
	if (se_Need[start.style] == 0) {
		seven_need &= (~(1 << start.style));
	}*/
#ifdef DEBUG
	cerr << "seven_need:" << seven_need << endl;
#endif // DEBUG


	return true;
}

void Command::mapToreal(pair<int, int> old, pair<double, double>& ret)
{
	ret.first = static_cast<double>(old.first) / 2 + 0.25;
	ret.second = (static_cast<double>(old.second) / 2 + 0.25);

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

bool Command::IsOnmyway(const robot& target, const robot& check, double pi)
{
	double x = target.real_pos.second - check.real_pos.second,
		y = (target.real_pos.first - check.real_pos.first);
	double target_angle = atan2(x, y);	//	计算目标角度与x正半轴夹角
	double a_diff = target_angle + (-check.face);	//	计算目标角度与朝向的差值
	if (a_diff > M_PI) {
		a_diff -= 2 * M_PI;
	}
	else if (a_diff < (-M_PI)) {
		a_diff += 2 * M_PI;
	}
	a_diff = abs(a_diff);
	if (a_diff <= pi && isNear(target.real_pos, check.real_pos, COLL_RADIUS)) {
		return true;
	}
	if ((outline_check(target, check, pi) && a_diff <= OUTLINE_RADIUS) && isNear(target.real_pos, check.real_pos, COLL_RADIUS)) {
		return true;
	}
	return false;
}

void Command::normal_caculate(const pair<double, double>& target, const pair<double, double>& cur, const double& face, double& speed, double& angle)
{
	//找到了目标
	double x = target.second - cur.second,
		y = (target.first - cur.first);
	double target_angle = atan2(x, y);	//	计算目标角度与x正半轴夹角
	double a_diff = target_angle + (-face);	//	计算目标角度与朝向的差值
	double distance = GetLength(target, cur);

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
		speed = 0.1;
		if (distance >= 5) {
			speed = 3;
		}
		angle = M_PI * dir;
	}
	//当机器人和目标地点偏角较大
	else if (a_diff >= M_PI_8) {
		speed = 1;
		if (distance >= 7) {
			speed = 6;
		}
		angle = M_PI * dir;
	}
	//做微调  
	else if (a_diff >= M_PI_32) {
		speed = 6;
		angle = M_PI_4 * dir;
	}
	else if (a_diff >= M_PI_64) {
		speed = 6;
		angle = M_PI_8 * dir;
	}
	//前进
	else {
		speed = 6;
		angle = 0;
	}

	if (map_id == 43) {
		if (target.first <= 1 || target.first >= 49 || target.second <= 1 || target.second >= 49) {
			if (cur.first <= 2 || cur.first >= 48 || cur.second <= 2 || cur.second >= 48) {
				return;
			}
			if (distance >= 2) {
				angle -= M_PI_4;
			}
		}
	}
}

void Command::coll_angle_caculate(const pair<double, double>& target, const pair<double, double>& cur, const double& t_face, const double& c_face, double& angle, double base)
{
	double x = target.second - cur.second,
		y = (target.first - cur.first);
	double target_angle = atan2(x, y);	//	计算目标角度与x正半轴夹角
	double a_diff = target_angle + (-c_face);	//	计算目标角度与朝向的差值
	if (a_diff > M_PI) {
		a_diff -= 2 * M_PI;
	}
	else if (a_diff < (-M_PI)) {
		a_diff += 2 * M_PI;
	}
	int dir = a_diff > 0 ? -1 : 1;
	
	double face_dif = t_face - c_face;
	face_dif = abs(face_dif);

	int face_dir = face_dif <= M_PI_2 ? -1 : 1;

	angle = base * dir * (face_dif > M_PI_H ? face_dir : 1);
}

bool Command::route_caculate(const route& cur_route, const robot& rb, double& maxValue, double& maxDistance, double& score, bool is_first)
{
	worker route_start_worker = idToworker[cur_route.start];
	pair<double, double> cur_worker_pos = route_start_worker.real_pos;
	double distance = GetLength(rb.real_pos, cur_worker_pos); //	计算机器人到起始点距离
	if (cur_route.stat == NO_PRODUCT && (lengthOneFrame * route_start_worker.time) > distance) {
		return false;
	}
	distance += cur_route.length;
	if (lastbuyselect * (9000 - frame) < distance) {
		return false;
	}
	if (is_first) {
		maxValue = max(maxValue, cur_route.value);
		maxDistance = max(maxDistance, distance);
	}
	else {
		distance = distance / maxDistance;	//路线越小越好，这是一个极小型指标
		double temp_value;
		temp_value = 1 - cur_route.value / maxValue;	// 价值越大越好，这一个极大型指标，需要正向化
		score = temp_value * VALUE_WEIGHT + distance * LENGTH_WEIGHT;	//	权重都置0.5
	}
	return true;
}

bool Command::can_select(const route& cur)
{
	worker route_start_worker = idToworker[cur.start];
	if (stat == MIDP_OVER && route_start_worker.style <= 3) {
		return false;
	}
	if (seven_need != OBJECT_NULL) {
		if (stat == MIDP_OVER && (route_start_worker.product_object & seven_need) == 0) {
			return false;
		}
	}
	return true;
}

bool Command::closeToside(const robot& cur)
{
	int x = cur.real_pos.first, y = cur.real_pos.second;
	double timex1 = abs((x - 0.5) / cur.l_speed.first), timex2 = abs((49.5 - x) / cur.l_speed.second);
	double timey1 = abs((y - 0.5) / cur.l_speed.second), timey2 = abs((49.5 - y) / cur.l_speed.second);
#ifdef DEBUG
	cerr << timex1 << " " << timex2 << " " << timey1 << " " << timey2 << endl;
#endif // DEBUG

 	/*if (x <= 0.5 || x >= 49.5) {
		return true;
	}
	if (y <= 0.5 || y >= 49.5) {
		return true;
	}*/
	if (min(timex1, timex2) <= 0.3) {
		return true;
	}
	if (min(timey1, timey2) <= 0.3) {
		return true;
	}
	return false;
}

bool Command::outline_check(const robot& target, const robot& check, double pi)
{
	double k1 = tan(check.face + pi);
	double k2 = tan(check.face - pi);
	double x1 = check.real_pos.first, y1 = check.real_pos.second;
	double x2 = target.real_pos.first, y2 = target.real_pos.second;
	double dis1 = abs(k1 * (x2 - x1) - (y2 - y1)) / sqrt(k1 * k1 + 1);
	double dis2 = abs(k2 * (x2 - x1) - (y2 - y1)) / sqrt(k2 * k2 + 1);
	if (dis1 < target.state == AFTER ? 0.53 : 0.45) {
		return true;
	}
	if (dis2 < target.state == AFTER ? 0.53 : 0.45) {
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

	for (auto p = maybe_avaliable.begin(); p != maybe_avaliable.end(); ++p) {
		unavaliable.insert(unavaliable.end(), *p);
		p = maybe_avaliable.erase(p);
	}

	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		p->stat = NO_PRODUCT;
		if ((idToworker[p->end].hold_object & p->object) != 0) {
			p->stat |= NO_NEED;
		}
	}
}

//	更新被选中的路线
void Command::flush_list() {
	mid_count = 0;
	for (auto p = avaliable.begin(); p != avaliable.end();) {
		if ((p->stat ^ NO_PRODUCT) == 0) {
			maybe_avaliable.insert(maybe_avaliable.end(), *p);
			p = avaliable.erase(p);
		}
		else if (p->stat != 0) {
			unavaliable.insert(unavaliable.end(), *p);
			p = avaliable.erase(p);
		}
		else {
			if (p->object > OBJECT_THREE) {
				++mid_count;
			}
			++p;
		}
	}

	for (auto p = maybe_avaliable.begin(); p != maybe_avaliable.end();) {
		if ((p->stat ^ NO_PRODUCT) == 0) {
			++p;
		}
		else if (p->stat != 0) {
			unavaliable.insert(unavaliable.end(), *p);
			p = maybe_avaliable.erase(p);
		}
		else {
			avaliable.insert(avaliable.end(), *p);
			p = maybe_avaliable.erase(p);
		}
	}

	for (auto p = unavaliable.begin(); p != unavaliable.end();) {
		if ((p->stat ^ NO_PRODUCT) == 0) {
			maybe_avaliable.insert(maybe_avaliable.end(), *p);
			p = unavaliable.erase(p);
		}
		else if (p->stat != 0) {
			++p;
		}
		else {
			avaliable.insert(avaliable.end(), *p);
			p = unavaliable.erase(p);
		}
	}
}

void Command::flush_money_stat() {
	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		if (p->base <= money) {
			p->stat &= (~NO_MONEY);
		}
		if (p->base > money) {
			p->stat |= NO_MONEY;
		}
	}

	for (auto p = maybe_avaliable.begin(); p != maybe_avaliable.end(); ++p) {
		if (p->base > money) {
			p->stat |= NO_MONEY;
		}
	}

	for (auto p = avaliable.begin(); p != avaliable.end(); ++p) {
		if (p->base > money) {
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

	for (auto p = maybe_avaliable.begin(); p != maybe_avaliable.end(); ++p) {
		if (p->start == id) {
			p->stat &= (~NO_PRODUCT);
		}
	}
}

void Command::puton_product_stat(int id) {
	for (auto p = avaliable.begin(); p != avaliable.end(); ++p) {
		if (p->start == id) {
			p->stat &= (~OCC_S);
			p->stat |= NO_PRODUCT;
		}
	}

	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		if (p->start == id) {
			p->stat &= (~OCC_S);
			p->stat |= NO_PRODUCT;
		}
	}
}

//start版本
void Command::puton_occ_stat(int id) {
	for (auto p = avaliable.begin(); p != avaliable.end(); ++p) {
		if (p->start == id) {
			p->stat |= OCC_S;
		}
	}

	for (auto p = maybe_avaliable.begin(); p != maybe_avaliable.end(); ++p) {
		if (p->start == id) {
			p->stat |= OCC_S;
		}
	}

	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		if (p->start == id) {
			p->stat |= OCC_S;
		}
	}
}

//end版本
void Command::puton_occ_stat(int id, int object) {
	for (auto p = avaliable.begin(); p != avaliable.end(); ++p) {
		if (p->end == id && p->object == object) {
			p->stat |= OCC_E;
		}
	}

	for (auto p = maybe_avaliable.begin(); p != maybe_avaliable.end(); ++p) {
		if (p->end == id && p->object == object) {
			p->stat |= OCC_E;
		}
	}

	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		if (p->end == id && p->object == object) {
			p->stat |= OCC_E;
		}
	}
}

void Command::puton_need_stat(int id, int object) {
	for (auto p = avaliable.begin(); p != avaliable.end(); ++p) {
		if (p->end == id && p->object == object) {
			p->stat &= (~OCC_E);
			p->stat |= NO_NEED;
		}
	}

	for (auto p = maybe_avaliable.begin(); p != maybe_avaliable.end(); ++p) {
		if (p->end == id && p->object == object) {
			p->stat &= (~OCC_E);
			p->stat |= NO_NEED;
		}
	}

	for (auto p = unavaliable.begin(); p != unavaliable.end(); ++p) {
		if (p->end == id && p->object == object) {
			p->stat &= (~OCC_E);
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