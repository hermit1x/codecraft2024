//
// Created by 华华 on 2024/4/3.
//

#ifndef CODECRAFT2024_ICT_ROBOT_H
#define CODECRAFT2024_ICT_ROBOT_H

#include "common.h"
#include "ict_map.h"
#include "ict_berth.h"
#include "ict_obj.h"

//#define DEBUG_ROBOT
//#define DEBUG_ROBOT_SCHEDULE

using robotBirthPQnode = berthPQnode;
std::priority_queue<robotBirthPQnode> robotBirthPQ;

class RobotBirth {
public:
    std::vector<Pos> poses;
    int dis[201][201];
};
std::vector<RobotBirth> robotBirths;

void init_robot_birth() {
    int vis[201][201];
    for (int i = 0; i < 200; ++i) {
        for (int j = 0; j < 200; ++j) {
            vis[i][j] = 0;
        }
    }
    for (int i = 0; i < 200; ++i) {
        for (int j = 0; j < 200; ++j) {
            if (mat[i][j].type == MAP_ROBOT_BUY && !vis[i][j]) {
                robotBirths.emplace_back(RobotBirth());

                robotBirthPQ.push(berthPQnode(Pos(i, j), 0));
                while (!robotBirthPQ.empty()) {
                    robotBirthPQnode top = robotBirthPQ.top();
                    robotBirthPQ.pop();
                    if (vis[top.p.x][top.p.y]) continue;
                    vis[top.p.x][top.p.y] = 1;

                    robotBirths.back().poses.push_back(top.p);
                    for (int k = 0; k < 4; ++k) {
                        Pos next = top.p + mov[k];
                        if (is_legal_bot(next)) {
                            robotBirthPQ.push(berthPQnode(next, 0));
                        }
                    }
                }

                for (auto p : robotBirths.back().poses) {
                    robotBirths.back().dis[p.x][p.y] = 0;
                    vis[p.x][p.y] = 0;
                    robotBirthPQ.push(berthPQnode(p, 0));
                }
                for (int ii = 0; ii < 200; ++ii) {
                    for (int jj = 0; jj < 200; ++jj) {
                        robotBirths.back().dis[ii][jj] = INF;
                    }
                }
                while (!robotBirthPQ.empty()) {
                    robotBirthPQnode top = robotBirthPQ.top();
                    robotBirthPQ.pop();
                    if (vis[top.p.x][top.p.y]) continue;
                    vis[top.p.x][top.p.y] = 1;
                    for (int k = 0; k < 4; ++k) {
                        Pos next = top.p + mov[k];
                        if (is_legal_bot(next) && robotBirths.back().dis[next.x][next.y] > top.d + 1) {
                            robotBirths.back().dis[next.x][next.y] = top.d + 1;
                            robotBirthPQ.push(berthPQnode(next, top.d + 1));
                        }
                    }
                }
            }
        }
    }
#ifdef DEBUG_FLAG
    fprintf(stderr, "# Robot Births: %lu\n", robotBirths.size());
#endif
}

enum robot_status { NOTHING, TO_OBJ, AT_OBJ, TO_SHIP, AT_SHIP, RECOVERY };
enum robot_mov { ROBOT_GET, ROBOT_PULL, ROBOT_NOTHING };

struct RobotMove {
    Pos p;
    int dir;
    int value;
    RobotMove(Pos _p, int _dir, int _v): p(_p), dir(_dir), value(_v) {}
    
    bool operator < (const RobotMove &rhs) const {
        return value < rhs.value;
    }
};

struct robotAstarNode {
    Pos p;
    int d;
    int star_d;

    robotAstarNode(Pos _p, int _d, int _star_d): p(_p), d(_d), star_d(_star_d) {}
    
    bool operator < (const robotAstarNode &rhs) const {
        return star_d > rhs.star_d;
    }
};
std::priority_queue<robotAstarNode> robotAstarPQ;

struct BerthInfo {
    int bid, dis;
    double dif; // = value - workers
    BerthInfo(int _bid, double _dif, int _dis) : bid(_bid), dif(_dif), dis(_dis) {};
    bool operator < (const BerthInfo &x) const {
        return dis < x.dis;
    }
};

void calc_is_safe(int id);

using robotPQnode = berthPQnode;
std::priority_queue<robotPQnode> robotPQ;

class Robot {
public:
    Pos p;
    int id;
    int frame;
    int carry, capacity;
    int dest_berth;
    robot_status status;
    Obj dest_obj;
    Obj carry_objs[2];

    int dis[201][201];
    int vis[201][201];
    int reachable[201][201];

    Robot(int _x, int _y, int _id, int _frame, int birth_id, int _cap) : p({_x, _y}), id(_id), status(NOTHING), frame(_frame), carry(0), dest_berth(0), capacity(_cap) {
        for (int i = 0; i < 200; ++i) {
            for (int j = 0; j < 200; ++j) {
                dis[i][j] = INF;
                vis[i][j] = 0;
                reachable[i][j] = (robotBirths[birth_id].dis[i][j] < INF);
            }
        }
        calc_best_berth();
    }

    void sync(int _carry, int _x, int _y, int _frame) {
        if (_x != p.x || _y != p.y || _frame != frame || _carry != carry) {
#ifdef DEBUG_ROBOT
            fprintf(stderr, "# ROBOT %d Sync Error\n", id);
            fprintf(stderr, "# rec: (%d, %d) %d, carry:%d\n", p.x, p.y, frame, carry);
            fprintf(stderr, "# get: (%d, %d) %d, carry:%d\n", _x, _y, _frame, _carry);
#endif
            p.x = _x; p.y = _y; frame = _frame; carry = _carry;
        }
        if (status == TO_OBJ && dest_obj.p == p) {
            status = AT_OBJ;
        }
        if (status == TO_SHIP && is_berth(p)) {
            status = AT_SHIP;
        }

        if (status == TO_OBJ && dest_obj.time_appear + 1000 <= frame) {
            status = NOTHING;
        }

        frame++;
    }

    robot_mov act_before_move;
    int is_safe;
    int assign_move_id;
    std::vector<RobotMove> want_moves;
    
    void think() {
        // 大状态机！
        // enum robot_status { NOTHING, TO_OBJ, AT_OBJ, TO_SHIP, AT_SHIP, RECOVERY };
        act_before_move = ROBOT_NOTHING;
        is_safe = 1;
        assign_move_id = -1;
        want_moves.clear();

        if (status == TO_OBJ) {
            calc_moves();
            return;
        }
        if (status == AT_OBJ) {
//            fprintf(stderr, "[ robot:%d ] AT_OBJ\n", id);
            act_before_move = ROBOT_GET;
            carry_objs[carry] = dest_obj;
            carry++;
            if (capacity == 1) {
//                fprintf(stderr, "[ robot:%d ] type 1: cap1 carry1\n", id);
                status = TO_SHIP;
                calc_best_berth();
                calc_moves();
                return;
            }
            if (capacity == 2) {
                if (carry == 2) {
                    fprintf(stderr, "[ robot:%d ] type 2: carry 2 obj\n", id);
                    status = TO_SHIP;
                    calc_best_berth();
                    calc_moves();
                    return;
                }
                else {
                    fprintf(stderr, "[ robot:%d ] type 3: pick second obj\n", id);
                    status = TO_OBJ;
                    calc_second_obj();
                    clear_dis();
                    calc_moves();
                    return;
                }
            }
        }
        if (status == TO_SHIP) {
            if (frame > 14000 && berths[dest_berth].closed) calc_best_berth();
            calc_moves();
            return;
        }
        if (status == AT_SHIP) {
            act_before_move = ROBOT_PULL;
//            fprintf(stderr, "[ robot:%d ] type 4: pull, carry: %d\n", id, carry);
            if (carry > 1) {
                fprintf(stderr, "[ robot:%d ] type 4: pull not finish\n", id);
                // 再等一帧
                status = TO_SHIP;
                calc_moves();

                return;
            }
//            fprintf(stderr, "[ robot:%d ] type 4: pull is finish\n", id);
            status = NOTHING;
            // 下面接NOTHING的判断，看看有没有物体能捡
        }
        if (status == NOTHING) {
            if (choose_obj()) {
                status = TO_OBJ;
#ifdef DEBUG_ROBOT
                fprintf(stderr, "# ROBOT:%d get object\n", id);
#endif
                clear_dis();
                calc_moves();
            }
            else {
#ifdef DEBUG_ROBOT
                fprintf(stderr, "# ROBOT:%d no object\n", id);
#endif
                calc_moves();
            }
            return;
        }
#ifdef DEBUG_ROBOT
        if (status == RECOVERY) {
            fprintf(stderr, "# ROBOT:%d is in RECOVERY\n", id);
        }
#endif
    }

    void act() {
#ifdef DEBUG_ROBOT
        fprintf(stderr, "[ robot:%d ][ act ], act.size:%d\n", id, (int)want_moves.size());
        if ((int)want_moves.size() == 0) {
            fprintf(stderr, "[ robot:%d ] status: %d, carry: %d\n", id, status, carry);
        }
        for (int i = 0; i < want_moves.size(); ++i) {
            fprintf(stderr, "[ act:%d ] (%d,%d)->(%d,%d), dis:%d\n", i, p.x, p.y, want_moves[i].p.x, want_moves[i].p.y, want_moves[i].value);
        }
#endif
        if (act_before_move == ROBOT_GET) {
            fprintf(stderr, "[ robot:%d ][ GET ] obj: (%d,%d) v:%d\n", id, carry_objs[carry-1].p.x, carry_objs[carry-1].p.y, carry_objs[carry-1].value);
            if (mat[p.x][p.y].has_obj == false) {
                fprintf(stderr, "[ robot:%d ] ERROR, obj not exist\n", id);
            }
            printf("get %d\n", id);
//            berth[dest_berth].booked += 1;

            // 消掉future_value
//            int d = INF, b = berth_num, ox = dest_obj.x, oy = dest_obj.y;
//            for (int j = 0; j < berth_num; ++j) {
//                if (d > mat[ox][oy].dist[j]) {
//                    d = mat[ox][oy].dist[j];
//                    b = j;
//                }
//            }
#ifdef DEBUG_ROBOT_SCHEDULE
            fprintf(stderr, "future value sub %f to berth %d, v: %d, d: %d\n", 1.0 * dest_obj.value / d, b, dest_obj.value, d);
#endif
            carry = 1;
        }
        if (act_before_move == ROBOT_PULL) {
            carry--;
            fprintf(stderr, "[ robot:%d ][ PULL ] obj: (%d,%d) v:%d\n", id, carry_objs[carry].p.x, carry_objs[carry].p.y, carry_objs[carry].value);
            berths[dest_berth].stocks.push(carry_objs[carry].value);
            berths[dest_berth].stock += 1;
            berths[dest_berth].remain_value += carry_objs[carry].value;
            berths[dest_berth].tot_stock += 1;
            berths[dest_berth].tot_value += carry_objs[carry].value;

            printf("pull %d\n", id);
#ifdef DEBUG_ROBOT
            fprintf(stderr, "[%5d] #ROBOT:%d pull, berth:%d, berth_stock:%d, que.size:%lu\n", frame, id, dest_berth, berths[dest_berth].stock, berths[dest_berth].stocks.size());
#endif
            carry = 0;
        }
        if (want_moves[assign_move_id].dir != 4) {
            printf("move %d %d\n", id, want_moves[assign_move_id].dir);
#ifdef DEBUG_ROBOT
            fprintf(stderr, "[ final act:%d ]: (%d,%d)->(%d,%d)\n", assign_move_id, p.x, p.y, want_moves[assign_move_id].p.x, want_moves[assign_move_id].p.y);
#endif

            p = want_moves[assign_move_id].p;
        }
#ifdef DEBUG_ROBOT
        else {
            fprintf(stderr, "[ final act:%d ]: (%d,%d) NO_MOVE\n", assign_move_id, p.x, p.y);
//            for (int i = 0; i < want_moves.size(); ++i) {
//                fprintf(stderr, "# ROBOT:%d MOVEINFO mid:%d, dir:%d, (%d,%d)->(%d,%d), dis:%d\n", id, i, want_moves[i].dir, p.x, p.y, want_moves[i].p.x, want_moves[i].p.y, want_moves[i].value);
//            }
        }
#endif
    }

    int calc_best_obj() {
#ifdef DEBUG_ROBOT
        fprintf(stderr, "cal calc_best_obj, dest_berth:%d\n", dest_berth);
#endif
        // 最大化 value / dist
        double max_value = 0;
        int res = -1, obj_dis, obj_v;
        Pos obj_p;
        for (int i = 0; i < objs.size(); ++i) {
            obj_p = objs[i].p;
            obj_v = objs[i].value;
            obj_dis = mat[obj_p.x][obj_p.y].land_dist[dest_berth];
#ifdef DEBUG_ROBOT
            fprintf(stderr, "picking obj: (%d,%d) reachable:%d\n", obj_p.x, obj_p.y, reachable[obj_p.x][obj_p.y]);
#endif
            if (!reachable[obj_p.x][obj_p.y]) continue;
//            if (objs[i].time_appear + 1000 < frame + obj_dis) continue;
//            fprintf(stderr, "obj: (%d,%d) v:%d, d:%d\n", obj_p.x, obj_p.y, obj_v, obj_dis);
            if (max_value < 1.0 * obj_v / obj_dis) {
                max_value = 1.0 * obj_v / obj_dis;
                res = i;
            }
        }
        return res;
    }

    bool choose_obj() {
        int obj_id = calc_best_obj();
        if (obj_id == -1) return false;
        dest_obj = objs[obj_id];
//        fprintf(stderr, "[ robot:%d ] choose obj: %d (%d,%d)\n", id, obj_id, dest_obj.p.x, dest_obj.p.y);
        objs.erase(objs.begin() + obj_id);
        return true;
    }

    void calc_moves() {
        calc_is_safe(id);
        Pos nxt;
        if (status == NOTHING) {
            // 没有目标，随便走
            for (int i = 0; i < 5; ++i) {
                nxt = p + mov[i];
                if (is_legal_bot(nxt)) {
                    want_moves.emplace_back(nxt, i, std::rand());
                }
            }
        }
        if (status == TO_OBJ) {
            // 根据dis下降
            if (!vis[p.x][p.y]) calc_path_to_obj();
            for (int i = 0; i < 5; ++i) {
                nxt = p + mov[i];
                if (is_legal_bot(nxt)) {
//                    if (!vis[tx][ty]) calc_path_to_obj();
                    want_moves.emplace_back(nxt, i, dis[nxt.x][nxt.y]);
                }
            }
        }
        if (status == TO_SHIP) {
            for (int i = 0; i < 5; ++i) {
                nxt = p + mov[i];
                if (is_legal_bot(nxt)) {
                    want_moves.emplace_back(nxt, i, mat[nxt.x][nxt.y].land_dist[dest_berth]);
                }
            }
        }

        std::sort(want_moves.begin(), want_moves.end());
        if (is_safe) {
            assign_move_id = 0;
        }
#ifdef DEBUG_ROBOT
        if (want_moves.size() == 1) fprintf(stderr, "# ERROR, ROBOT:%d NO WAY\n", id);
//        for (int i = 0; i < want_moves.size(); ++i) {
//            fprintf(stderr, "# ROBOT:%d dir:%d dis:%d\n", id, want_moves[i].dir, want_moves[i].v);
//        }
#endif
    }

    void clear_dis() {
        for (int i = 0; i < 200; ++i) {
            for (int j = 0; j < 200; ++j) {
                dis[i][j] = INF;
                vis[i][j] = 0;
            }
        }
    }

    void calc_path_to_obj() {
#ifdef DEBUG_ROBOT
        fprintf(stderr, "# ROBOT:%d call calc_path_to_obj()\n", id);
#endif
        // 以 vx,vy 为源的最短路，从 x,y 出发
        /* A* 寻路，评估权重是 已经走的距离+曼哈顿距离
         * 同时存下来搜的时候的距离矩阵
         * */
        clear_dis();

        robotAstarPQ.push(robotAstarNode(dest_obj.p, 0, 0));
        Pos nxt;
        while (!robotAstarPQ.empty()) {
            robotAstarNode top = robotAstarPQ.top();
            robotAstarPQ.pop();
//        fprintf(stderr, "PQ2 loop: (%d, %d): %d\n", p.x, p.y, p.dr);
            if (vis[top.p.x][top.p.y]) continue;
            vis[top.p.x][top.p.y] = 1;
            dis[top.p.x][top.p.y] = top.d;



            int rand_base = rand() % 4;
            for (int i, j = 0; j < 4; ++j) {
                i = (rand_base + j) % 4;
                nxt = top.p + mov[i];

                if (is_legal_bot(nxt)) {
                    if (top.d < dis[nxt.x][nxt.y]) {
                        robotAstarPQ.emplace(
                                nxt,
                                top.d + 1,
                                top.star_d + 2 * dis_man(nxt, p)
                                // 加入曼哈顿距离，A star 思想
                                );
                    }
                }
            }

            if (p == top.p) {
                // 找到了
                break;
            }
        }

        while (!robotAstarPQ.empty()) {
            // 拓宽道路，至少有横向宽度为3的路可以走
            robotAstarNode top = robotAstarPQ.top();
            robotAstarPQ.pop();
            if (vis[top.p.x][top.p.y]) continue;
            vis[top.p.x][top.p.y] = 1;
            dis[top.p.x][top.p.y] = top.d;
        }

#ifdef OUTPUT_DIJKSTRA
        fprintf(stderr, "#3 get_path() final dist: %d\n", dis[x][y]);
    for (int i = 0; i < 200; ++i) {
        for (int j = 0; j < 200; ++j) {
            int x = dis[i][j];
            if (x == inf_dist) x = -1;
            fprintf(stderr, "%4d", x);
        }
        fprintf(stderr, "\n");
    }
    fflush(stderr);
#endif

    }

    void calc_best_berth() {
        double l_wokers[berth_num], l_future[berth_num], sum_future = 0;
        for (int i = 0; i < berth_num; ++i) {
            l_wokers[i] = berths[i].workers;
            l_future[i] = berths[i].remain_value;
//            fprintf(stderr, "# berth %d: before calc w:%f, v:%f\n", i, l_wokers[i], l_future[i]);
            sum_future += l_future[i];
        }
        for (int i = 0; i < berth_num; ++i) {
            l_wokers[i] /= 1.0 * berth_num;
            l_future[i] /= sum_future;
        }
        if (l_wokers[dest_berth] - l_future[dest_berth] > 0.15) {
#ifdef DEBUG_ROBOT_SCHEDULE
            fprintf(stderr, "# NOT BALANCE by bot:%d, dest:%d\n", id, dest_berth);
            for (int i = 0; i < berth_num; ++i) {
                fprintf(stderr, "# berth %d: w:%.0f, v:%f\n", i, l_wokers[i] * 10, l_future[i]);
            }
#endif

            std::vector<BerthInfo> berth_vec;
            for (int i = 0; i < berth_num; ++i) {
                if (
                        mat[p.x][p.y].land_dist[i] != INF // is_reachable(px,py)
                        && l_future[i] - l_wokers[i] > 0.15) {
                    berth_vec.push_back(BerthInfo(i, l_future[i] - l_wokers[i], mat[p.x][p.y].land_dist[i]));
                }
            }
            std::sort(berth_vec.begin(), berth_vec.end());
#ifdef DEBUG_ROBOT_SCHEDULE
            for (int i = 0; i < berth_vec.size(); ++i) {
                fprintf(stderr, "berth id:%d, dis:%d, val:%f\n", berth_vec[i].bid, berth_vec[i].dis, berth_vec[i].dif);
            }
#endif
            if (berth_vec.size() != 0 && berth_vec[0].dis <= 200) {
                berths[dest_berth].workers -= 1;
                dest_berth = berth_vec[0].bid;
                berths[dest_berth].workers += 1;
#ifdef DEBUG_ROBOT_SCHEDULE
                fprintf(stderr, "# REARRANGE bot:%d, to dest:%d\n", id, dest_berth);
#endif
                return;
            }
        }

        berths[dest_berth].workers -= 1;
        int min_dis = INF, min_berth = -1;
        for (int i = 0; i < berth_num; ++i) {
            if (berths[i].closed) continue;
            if (min_dis > mat[p.x][p.y].land_dist[i]) {
                min_dis = mat[p.x][p.y].land_dist[i];
                min_berth = i;
            }
        }
        dest_berth = min_berth;
        if (min_berth == -1) {
#ifdef DEBUG_ROBOT
            fprintf(stderr, "# ROBOT:%d NO Reachable Berth\n", id);
#endif
            dest_berth = 0;
        }
        berths[dest_berth].workers += 1;
    }

    void calc_second_obj() {
#ifdef DEBUG_ROBOT
        fprintf(stderr, "# ROBOT:%d call calc_path_to_obj()\n", id);
#endif
        // 以bot为中心做一个bfs，距离50以内的物体，选价值最高
        clear_dis();

        std::vector<Obj> near_objs;

        robotPQ.push(robotPQnode (p, 0));
        Pos nxt;
        while (!robotPQ.empty()) {
            robotPQnode top = robotPQ.top();
            robotPQ.pop();
//        fprintf(stderr, "PQ2 loop: (%d, %d): %d\n", p.x, p.y, p.dr);
            if (vis[top.p.x][top.p.y]) continue;
            vis[top.p.x][top.p.y] = 1;
            dis[top.p.x][top.p.y] = top.d;
            if (mat[top.p.x][top.p.y].has_obj) {
                int obj_id = find_obj(top.p);
                if (obj_id != -1) {
                    near_objs.push_back(objs[obj_id]);
                }
            }
            if (top.d > 50) continue;

            int rand_base = rand() % 4;
            for (int i, j = 0; j < 4; ++j) {
                i = (rand_base + j) % 4;
                nxt = top.p + mov[i];

                if (is_legal_bot(nxt)) {
                    if (top.d < dis[nxt.x][nxt.y]) {
                        robotPQ.emplace(
                                nxt,
                                top.d + 1
                        );
                    }
                }
            }
        }


        // 最大化 value / dist
        double max_value = 0;
        int res = -1, obj_dis, obj_v;
        Pos obj_p;
        for (int i = 0; i < near_objs.size(); ++i) {
            obj_p = near_objs[i].p;
            obj_v = near_objs[i].value;
            obj_dis = dis[obj_p.x][obj_p.y];
//            if (!reachable[obj_p.x][obj_p.y]) continue;
//            if (objs[i].time_appear + 1000 < frame + obj_dis) continue;
//            fprintf(stderr, "obj: (%d,%d) v:%d, d:%d\n", obj_p.x, obj_p.y, obj_v, obj_dis);
            if (max_value < 1.0 * obj_v / obj_dis) {
                max_value = 1.0 * obj_v / obj_dis;
                res = i;
            }
        }
        if (res == -1) {
            fprintf(stderr, "NO NEAR SECOND OBJ, USE GLOBAL\n");
            choose_obj();
            calc_best_berth();
            return;
        }

        dest_obj = near_objs[res];
//        fprintf(stderr, "[ robot:%d ] choose obj: %d (%d,%d)\n", id, obj_id, dest_obj.p.x, dest_obj.p.y);

    }
};
std::vector<Robot> robots;

void calc_is_safe(int id) {
    for (int i = 0; i < robot_num; ++i) {
        if (i == id) continue;
        if (dis_man(robots[id].p, robots[i].p) < 3) {
            robots[id].is_safe = 0;
            break;
        }
    }
}

struct Dsu {
    // 并查集
    std::vector<int> f;

    void init() {
        f.resize(robot_num);
        for (int i = 0; i < robot_num; ++i) f[i] = i;
    }
    int find(int x) { return f[x] == x ? x : f[x] = find(f[x]); }
    void unite(int x, int y) { f[find(x)] = find(y); }
} dsu;


int conflict_mat_robot[201][201][2];
std::vector<int> robot_priority;
std::vector<std::vector<int> > conflict_vec;
int conflict_vec_cnt = 0;

bool conflict_dfs(int vec_id, int rpp) {
    // rpp表示在robot_priority里处理到第几个了
//    fprintf(stderr, "dfs rpp:%d\n", rpp);
    if (rpp == conflict_vec[vec_id].size()) return true;

    int robot_id = conflict_vec[vec_id][rpp];
    int moves_len = robots[robot_id].want_moves.size();
    Pos tp;
    for (int i = 0; i < moves_len; ++i) {
        // 检查conflict
        tp = robots[robot_id].want_moves[i].p;
        if (!is_land_main(tp) && conflict_mat_robot[tp.x][tp.y][1] != -1) continue;
        if (!is_land_main(tp) && !is_land_main(robots[robot_id].p) && conflict_mat_robot[tp.x][tp.y][0] != -1 && conflict_mat_robot[tp.x][tp.y][0] == conflict_mat_robot[robots[robot_id].p.x][robots[robot_id].p.y][1]) {
            // 这一句表示对方从 tp 到我们这个位置上来
            // 我们默认是要去tp的位置
            continue;
        }
        conflict_mat_robot[tp.x][tp.y][1] = robot_id;
        robots[robot_id].assign_move_id = i;
        if (conflict_dfs(vec_id, rpp+1)) return true;
        conflict_mat_robot[tp.x][tp.y][1] = -1;
    }
    return false;
}

bool conflict_error_robot = false;
void handle_conflict_robot() {

    for (int i = 0; i < 200; ++i) {
        for (int j = 0; j < 200; ++j) {
            conflict_mat_robot[i][j][0] = -1;
            conflict_mat_robot[i][j][1] = -1;
        }
    }

    if (conflict_vec.size() != robot_num) {
        conflict_vec.resize(robot_num);
        robot_priority.resize(robot_num);
    }
    conflict_vec_cnt = 0;
    for (int i = 0; i < robot_num; ++i) {
        conflict_vec[i].clear();
    }
    for (int i = 0; i < robot_num; ++i) {
        conflict_mat_robot[ robots[i].p.x ][ robots[i].p.y ][0] = i;
    }

    // 分组填入conflict_vec, 把一个大的dfs拆成几个小的
    dsu.init();
    for (int i = 0; i < robot_num; ++i) {
        if (robots[i].is_safe) continue;
        for (int j = i + 1; j < robot_num; ++j) {
            if (robots[j].is_safe) continue;
            if (dis_man(robots[i].p, robots[j].p) < 3) {
                if (dsu.find(i) != dsu.find(j)) {
                    dsu.unite(j, i);
                }
            }
        }
    }

    /* 1. 有货的优先
     * 2. id小的优先
     * */
    int robot_pp = 0;
    for (int i = 0; i < robot_num; ++i) {
        if (robots[i].status == TO_SHIP) robot_priority[robot_pp++] = i;
    }
    for (int i = 0; i < robot_num; ++i) {
        if (robots[i].status != TO_SHIP) robot_priority[robot_pp++] = i;
    }

//    for (int i = 0; i < robot_num; ++i) {
//        fprintf(stderr, "%d -> ", robot_priority[i]);
//    }
//    fprintf(stderr, "\n");

    int rbi, rbj;
    for (int i = 0; i < robot_num; ++i) {
        rbi = robot_priority[i];
        if (robots[rbi].is_safe) continue;
        if (dsu.find(rbi) != rbi) continue;
        // 对于并查集中不是根结点的，先跳过
        conflict_vec[conflict_vec_cnt].push_back(rbi);
        for (int j = 0; j < robot_num; ++j) {
            rbj = robot_priority[j];
            if (rbj == rbi) continue;
            if (robots[rbj].is_safe) continue;
            if (dsu.find(rbj) == rbi) {
                conflict_vec[conflict_vec_cnt].push_back(rbj);
            }
        }
        conflict_vec_cnt++;
    }

    for (int i = 0; i < conflict_vec_cnt; ++i) {
//        fprintf(stderr, "[conflict_vec:%d]: ", i);
//        for (auto rid : conflict_vec[i]) {
//            fprintf(stderr, "%d ", rid);
//        }
//        fprintf(stderr, "\n");
        if (!conflict_dfs(i, 0)) {
            fprintf(stderr, "# ERROR: NO POSSIBLE SOLUTION\n");
            conflict_error_robot = true;
        }
    }
}

void update_robot(int frame_id) {
    int R; // robot数量
    scanf("%d", &R);
    for (int i = 0; i < R; ++i) {
        int id, carry, x, y;
        scanf("%d%d%d%d", &id, &carry, &x, &y);
#ifdef DEBUG_ROBOT
        fprintf(stderr, "robot input id: %d, carry: %d, (%d,%d)\n", id, carry, x, y);
#endif
        robots[id].sync(carry, x, y, frame_id);
    }
}

void test_buy_robot(int frame, int &money) {
    static int cnt = 0;
//    fprintf(stderr, "[ %5d ][ test_buy ] money:%d, cnt:%d, robot_num:%d\n", frame, money, cnt, robot_num);
    while (cnt < want_robot_num) {
        for (auto i : robotBirths) {
            if (use_robot_type == 1) {
                if (money < 2000) break;
                printf("lbot %d %d 0\n", i.poses[0].x, i.poses[0].y);
                money -= 2000;
                fprintf(stderr, "[ %5d ] buy_bot:%d (%d,%d), money_left:%d\n", frame, cnt++, i.poses[0].x, i.poses[0].y, money);
                robots.push_back(Robot(i.poses[0].x, i.poses[0].y, robot_num++, frame, 0, 1));
            }
            else {
                if (money < 5000) break;
                printf("lbot %d %d 1\n", i.poses[0].x, i.poses[0].y);
                money -= 5000;
                fprintf(stderr, "buy bot:%d (%d,%d), money_left:%d\n", cnt++, i.poses[0].x, i.poses[0].y, money);
                robots.push_back(Robot(i.poses[0].x, i.poses[0].y, robot_num++, frame, 0, 2));
            }
        }
        if (use_robot_type == 1) { if (money < 2000) break; }
        else { if (money < 5000) break; }
    }
}

#endif //CODECRAFT2024_ICT_ROBOT_H
