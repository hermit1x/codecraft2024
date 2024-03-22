#include <iostream>
#include <algorithm>
#include <queue>
#include <vector>
#include <memory>
#include <time.h>

//#define DEBUG_FLAG
//#define DEBUG_ROBOT
#define DEBUG_SHIP
#define DEBUG_BERTH
//#define OUTPUT_DIJKSTRA
#define min(x, y) ((x) < (y) ? (x) : (y))

enum mat_enum { LAND, OCEAN, HILL, BERTH };
enum robot_status { NOTHING, TO_OBJ, AT_OBJ, TO_SHIP, AT_SHIP, RECOVERY };
enum robot_mov { ROBOT_GET, ROBOT_PULL, ROBOT_NOTHING };
enum ship_enum { SHIP_SHIPPING, SHIP_NORMAL, SHIP_WAITING };

const int robot_num = 10;
const int berth_num = 10;
const int ship_num = 5;
const int inf_dist = 1e6;

int tot_value = 0;

int v_full = 0;

// 右 左 上 下 不动
const int mov_x[6] = {0, 0, -1, 1, 0, 0};
const int mov_y[6] = {1, -1, 0, 0, 0, 0};

inline int dis_man(int ax, int ay, int bx, int by) {
    return abs(ax - bx) + abs(ay - by);
}

inline int dis_oc(int ax, int ay, int bx, int by) {
    return abs(ax - bx)*abs(ax - bx) + abs(ay - by)*abs(ay - by);
}

inline bool in_mat(int x, int y) {
    return 0 <= x && x < 200 && 0 <= y && y < 200;
}

class Mat {
public:
    mat_enum type;
    bool has_obj;
    int dist[berth_num];
} mat[207][207];

inline bool is_land(int x, int y) {
    return mat[x][y].type == LAND || mat[x][y].type == BERTH;
}

inline bool is_legal(int x, int y) {
    return in_mat(x, y) && is_land(x, y);
}

class Obj {
public:
    Obj(): x(-1), y(-1) {};
    Obj(int _x, int _y, int _value, int _t) : x(_x), y(_y), value(_value), time_appear(_t), obtain(0) {};

    int x, y, value, time_appear, obtain;
};
std::vector<Obj> objects;
void remove_outdated_obj(int frame_id) {
    std::vector<Obj>::iterator it = objects.begin();
    while (it != objects.end()) {
        if (it->time_appear + 1000 < frame_id) {
            mat[it->x][it->y].has_obj = false;
            it++;
        }
        else break;
    }
    objects.erase(objects.begin(), it);
#ifdef DEBUG_FLAG
    fprintf(stderr, "# Frame %d objects.size() = %lu\n", frame_id, objects.size());
#endif
}

class Berth {
public:
    Berth() : value(0.0), tot_value(0), tot_stock(0), stock(0), capacity(0), booked(0), closed(0) {}

    bool is_in(int ax, int ay) {
        return x <= ax && ax <= x + 3 && y <= ay && ay <= y + 3;
    }

    int id, x, y, transport_time, velocity;
    int occupy;
    int stock, capacity, booked, closed, remain_value;
    double value;
    int tot_value, tot_stock;
    std::queue<int> stocks;
} berth[berth_num+2];

struct PQnode2 {
    PQnode2() {};
    PQnode2(int _x, int _y, int _d, int _dr) : x(_x), y(_y), d(_d), dr(_dr) {};
    int x, y, d, dr; // dist real

    bool operator < (const PQnode2 &x) const {
        return d > x.d;
    }
};
std::priority_queue<PQnode2> pqueue2;

int robot_cnt = 0;

struct Pos {
    Pos() {}
    Pos(int _x, int _y, int _dir, int _v) : x(_x), y(_y), dir(_dir), v(_v) {}

    int x, y, dir, v;
    bool operator < (const Pos &x) const {
        return v < x.v;
    }
};

void calc_is_safe(int id);

class Robot {
public:
    Robot() {}
    Robot(int _x, int _y, int _id) : x(_x), y(_y), id(_id), status(NOTHING), frame(1), carry(0) {
        for (int i = 0; i < 200; ++i) {
            for (int j = 0; j < 200; ++j) {
                dis[i][j] = inf_dist;
                vis[i][j] = 0;
            }
        }
    }

    void init() {
        for (int i = 0; i < 200; ++i) {
            for (int j = 0; j < 200; ++j) {
                reachable[i][j] = 0;
            }
        }

        pqueue2.push(PQnode2(x, y, 0, 0));
        reachable[x][y] = 1;
        int tx, ty;
        while (!pqueue2.empty()) {
            PQnode2 p = pqueue2.top();
            pqueue2.pop();

            for (int i = 0; i < 4; ++i) {
                tx = p.x + mov_x[i];
                ty = p.y + mov_y[i];

                if (is_legal(tx, ty)) {
                    if (!reachable[tx][ty]) {
                        reachable[tx][ty] = 1;
                        pqueue2.push(PQnode2(tx, ty, 0, 0));
                    }
                }
            }
        }

        calc_best_berth();
    }

    void sync(int _carry, int _x, int _y, int status_code, int _frame) {
        if (_x != x || _y != y || _frame != frame || _carry != carry) {
#ifdef DEBUG_ROBOT
            fprintf(stderr, "# ROBOT %d Sync Error\n", id);
            fprintf(stderr, "# rec: (%d, %d) %d, carry:%d\n", x, y, frame, carry);
            fprintf(stderr, "# get: (%d, %d) %d, carry:%d\n", _x, _y, _frame, _carry);
#endif
            x = _x; y = _y; frame = _frame; carry = _carry;
        }
#ifdef DEBUG_ROBOT
        if (status_code == 0) {
            fprintf(stderr, "#ERROR# ROBOT %d has code 0\n", id);
//            if (status == RECOVERY) return;
//            status = RECOVERY;
        }
#endif
        if (status == TO_OBJ && dest_obj.x == x && dest_obj.y == y) {
            status = AT_OBJ;
        }
        if (status == TO_SHIP && berth[dest_berth].is_in(x, y)) {
            status = AT_SHIP;
        }

        if (status == TO_OBJ && dest_obj.time_appear + 1000 <= frame) {
            status = NOTHING;
        }

        frame++;
    }

    void think() {
        // 大状态机！
        // enum robot_status { NOTHING, TO_OBJ, AT_OBJ, TO_SHIP, AT_SHIP, RECOVERY };
        act_before_move = ROBOT_NOTHING;
        is_safe = 1;
        assign_move_id = -1;
        next_moves.clear();

        if (status == TO_OBJ) {
            calc_moves();
            return;
        }
        if (status == AT_OBJ) {
            act_before_move = ROBOT_GET;
            status = TO_SHIP;
            calc_best_berth();
            calc_moves();
            return;
        }
        if (status == TO_SHIP) {
            if (frame > 12000 && berth[dest_berth].closed) calc_best_berth();
            calc_moves();
            return;
        }
        if (status == AT_SHIP) {
            act_before_move = ROBOT_PULL;
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
        if (act_before_move == ROBOT_GET) {
            printf("get %d\n", id);
            berth[dest_berth].booked += 1;
            carry = 1;
        }
        if (act_before_move == ROBOT_PULL) {
#ifdef DEBUG_BERTH
            tot_value += dest_obj.value;
            if (frame > 15000 - berth[dest_berth].transport_time) berth[dest_berth].stock -= 1; // 载不到的就不记了
#endif
            berth[dest_berth].stocks.push(dest_obj.value);
            berth[dest_berth].remain_value += dest_obj.value;
            berth[dest_berth].tot_value += dest_obj.value;
            berth[dest_berth].tot_stock += 1;
            berth[dest_berth].stock += 1;
            berth[dest_berth].booked -= 1;
            printf("pull %d\n", id);
            carry = 0;
        }
        if (next_moves[assign_move_id].dir != 4) {
            printf("move %d %d\n", id, next_moves[assign_move_id].dir);
//            fprintf(stderr, "robot %d: (%d,%d)->(%d,%d)\n", id, x, y, next_moves[assign_move_id].x, next_moves[assign_move_id].y);
            x = next_moves[assign_move_id].x;
            y = next_moves[assign_move_id].y;
#ifdef DEBUG_ROBOT
            if (!is_legal(x, y)) {
                fprintf(stderr, "robot %d: (%d,%d)->(%d,%d)\n", id, x, y, next_moves[assign_move_id].x, next_moves[assign_move_id].y);
            }
#endif
        }
#ifdef DEBUG_ROBOT
        else {
            fprintf(stderr, "robot %d: no move, amid:%d, status: %d, carry: %d, obj:(%d,%d)\n", id, assign_move_id, status, carry, dest_obj.x, dest_obj.y);
            for (int i = 0; i < next_moves.size(); ++i) {
                fprintf(stderr, "# ROBOT:%d MOVEINFO mid:%d, dir:%d, (%d,%d)->(%d,%d), dis:%d\n", id, i, next_moves[i].dir, x, y, next_moves[i].x, next_moves[i].y, next_moves[i].v);
            }
        }
#endif
    }

    bool choose_obj() {
        int obj_id = calc_best_obj();
        if (obj_id == -1) return false;

        dest_obj = objects[obj_id];
        objects.erase(objects.begin() + obj_id);
        return true;
    }

    void calc_moves() {
        calc_is_safe(id);

        int tx, ty;
        if (status == NOTHING) {
            // 没有目标，随便走
            for (int i = 0; i < 5; ++i) {
                tx = x + mov_x[i];
                ty = y + mov_y[i];
                if (is_legal(tx, ty)) {
                    next_moves.push_back(Pos(tx, ty, i, rand()));
                }
            }
        }
        if (status == TO_OBJ) {
            // 根据dis下降
            if (!vis[x][y]) calc_path_to_obj();
            for (int i = 0; i < 5; ++i) {
                tx = x + mov_x[i];
                ty = y + mov_y[i];
                if (is_legal(tx, ty)) {
//                    if (!vis[tx][ty]) calc_path_to_obj();
                    next_moves.push_back(Pos(tx, ty, i, dis[tx][ty]));
                }
            }
        }
        if (status == TO_SHIP) {
            for (int i = 0; i < 5; ++i) {
                tx = x + mov_x[i];
                ty = y + mov_y[i];
                if (is_legal(tx, ty)) {
                    next_moves.push_back(Pos(tx, ty, i, mat[tx][ty].dist[dest_berth]));
                }
            }
        }

        std::sort(next_moves.begin(), next_moves.end());
        if (is_safe) {
            assign_move_id = 0;
        }
#ifdef DEBUG_ROBOT
        if (next_moves.size() == 1) fprintf(stderr, "# ERROR, ROBOT:%d NO WAY\n", id);
//        for (int i = 0; i < next_moves.size(); ++i) {
//            fprintf(stderr, "# ROBOT:%d dir:%d dis:%d\n", id, next_moves[i].dir, next_moves[i].v);
//        }
#endif
    }

    int calc_best_obj() {
        // 最大化 value / dist
        double max_value = 0;
        int res = -1, obj_x, obj_y, obj_dis, obj_v, bias = 0;
        for (int i = 0; i < objects.size(); ++i) {
            obj_x = objects[i].x;
            obj_y = objects[i].y;
            obj_v = objects[i].value;
            obj_dis = mat[obj_x][obj_y].dist[dest_berth];
            if (!reachable[obj_x][obj_y]) continue;
            if (objects[i].time_appear + 1000 < frame + obj_dis) continue;
            if (max_value < 1.0 * obj_v / (obj_dis - bias)) {
                max_value = 1.0 * obj_v / (obj_dis - bias);
                res = i;
            }
        }
        return res;
        /*
        // 选当前价值最大的且能拿得到的
        int max_val = 0, res = -1;
        int obj_x, obj_y, obj_dis, obj_v;
        for (int i = 0; i < objects.size(); ++i) {
            obj_x = objects[i].x;
            obj_y = objects[i].y;
            obj_v = objects[i].value;
            obj_dis = mat[obj_x][obj_y].dist[dest_berth];
            if (objects[i].time_appear + 1000 < frame + obj_dis) continue;
            if (max_val < obj_v) {
                max_val = obj_v;
                res = i;
            }
        }
        return res;
        */
        /*
        // 简单以最近的来选
        int min_dis = inf_dist, res = -1;
        int obj_x, obj_y, obj_dis;
        for (int i = 0; i < objects.size(); ++i) {
            obj_x = objects[i].x;
            obj_y = objects[i].y;
            obj_dis = mat[obj_x][obj_y].dist[dest_berth];
            if (!reachable[obj_x][obj_y]) continue;
            if (objects[i].time_appear + 1000 + 500 < frame + obj_dis) continue;
            if (min_dis > obj_dis) {
                min_dis = obj_dis;
                res = i;
            }
        }
        return res;
         */
    }

    void clear_dis() {
        for (int i = 0; i < 200; ++i) {
            for (int j = 0; j < 200; ++j) {
                dis[i][j] = inf_dist;
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
        int vx = dest_obj.x;
        int vy = dest_obj.y;
        clear_dis();

        pqueue2.push(PQnode2(vx, vy, 0, 0));
        int tx, ty;
        while (!pqueue2.empty()) {
            PQnode2 p = pqueue2.top();
            pqueue2.pop();
//        fprintf(stderr, "PQ2 loop: (%d, %d): %d\n", p.x, p.y, p.dr);
            if (vis[p.x][p.y]) continue;
            vis[p.x][p.y] = 1;
            dis[p.x][p.y] = p.dr;



            int rand_base = rand() % 4;
            for (int i, j = 0; j < 4; ++j) {
                i = (rand_base + j) % 4;
                tx = p.x + mov_x[i];
                ty = p.y + mov_y[i];

                if (is_legal(tx, ty)) {
                    if (p.dr < dis[tx][ty]) {
                        pqueue2.push(PQnode2(
                                tx, ty,
                                p.dr + 2 * dis_man(tx, ty, x, y),
                                // 加入曼哈顿距离，A star 思想
                                p.dr + 1));
                    }
                }
            }

            if (p.x == x && p.y == y) {
                // 找到了
                break;
            }
        }

        while (!pqueue2.empty()) {
            // 拓宽道路，至少有横向宽度为3的路可以走
            PQnode2 p = pqueue2.top();
            pqueue2.pop();
            if (vis[p.x][p.y]) continue;
            vis[p.x][p.y] = 1;
            dis[p.x][p.y] = p.dr;
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
        int min_dis = inf_dist, min_berth = -1;
        for (int i = 0; i < berth_num; ++i) {
            if (berth[i].closed) continue;
            if (min_dis > mat[x][y].dist[i]) {
                min_dis = mat[x][y].dist[i];
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
    }


    int id, x, y, frame, carry;
    robot_status status;
    int dis[201][201], vis[201][201], reachable[201][201];
    int dest_berth;
    Obj dest_obj;
    robot_mov act_before_move;
    std::vector<Pos> next_moves;
    int assign_move_id;
    int is_safe;
} robot[robot_num+1];

void calc_is_safe(int id) {
    for (int i = 0; i < robot_num; ++i) {
        if (i == id) continue;
        if (dis_man(robot[id].x, robot[id].y, robot[i].x, robot[i].y) < 3) {
            robot[id].is_safe = 0;
            break;
        }
    }
}


struct Dsu {
    // 并查集
    int f[robot_num];

    void init() { for (int i = 0; i < robot_num; ++i) f[i] = i; }
    int find(int x) { return f[x] == x ? x : f[x] = find(f[x]); }
    void unite(int x, int y) { f[find(x)] = find(y); }
} dsu;


int conflict_mat[201][201][2];
int robot_priority[robot_num+1];
std::vector<int> conflict_vec[robot_num];
int conflict_vec_cnt = 0;

bool conflict_dfs(int vec_id, int rpp) {
    // rpp表示在robot_priority里处理到第几个了
//    fprintf(stderr, "dfs rpp:%d\n", rpp);
    if (rpp == conflict_vec[vec_id].size()) return true;

    int robot_id = conflict_vec[vec_id][rpp];
    int moves_len = robot[robot_id].next_moves.size();
    Pos tp;
    for (int i = 0; i < moves_len; ++i) {
        // 检查conflict
        tp = robot[robot_id].next_moves[i];
        if (conflict_mat[tp.x][tp.y][1] != -1) continue;
        if (conflict_mat[tp.x][tp.y][0] != -1 && conflict_mat[tp.x][tp.y][0] == conflict_mat[robot[robot_id].x][robot[robot_id].y][1]) {
            // 这一句表示对方从 tp 到我们这个位置上来
            // 我们默认是要去tp的位置
            continue;
        }
        conflict_mat[tp.x][tp.y][1] = robot_id;
        robot[robot_id].assign_move_id = i;
        if (conflict_dfs(vec_id, rpp+1)) return true;
        conflict_mat[tp.x][tp.y][1] = -1;
    }
    return false;
}

bool conflict_error = false;
void handle_conflict() {
    for (int i = 0; i < 200; ++i) {
        for (int j = 0; j < 200; ++j) {
            conflict_mat[i][j][0] = -1;
            conflict_mat[i][j][1] = -1;
        }
    }
    conflict_vec_cnt = 0;
    for (int i = 0; i < robot_num; ++i) {
        conflict_vec[i].clear();
    }
    for (int i = 0; i < robot_num; ++i) {
        conflict_mat[ robot[i].x ][ robot[i].y ][0] = i;
    }

    // 分组填入conflict_vec, 把一个大的dfs拆成几个小的
    dsu.init();
    for (int i = 0; i < robot_num; ++i) {
        if (robot[i].is_safe) continue;
        for (int j = i + 1; j < robot_num; ++j) {
            if (robot[j].is_safe) continue;
            if (dis_man(robot[i].x, robot[i].y, robot[j].x, robot[j].y) < 3) {
                if (dsu.find(i) != dsu.find(j)) {
                    dsu.unite(i, j);
                }
            }
        }
    }

    /* 1. 有货的优先
     * 2. id小的优先
     * */
    int robot_pp = 0;
    for (int i = 0; i < robot_num; ++i) {
        if (robot[i].status == TO_SHIP) robot_priority[robot_pp++] = i;
    }
    for (int i = 0; i < robot_num; ++i) {
        if (robot[i].status != TO_SHIP) robot_priority[robot_pp++] = i;
    }

    int rbi, rbj;
    for (int i = 0; i < robot_num; ++i) {
        rbi = robot_priority[i];
        if (robot[rbi].is_safe) continue;
        if (dsu.find(rbi) != rbi) continue;
        // 对于并查集中不是根结点的，先跳过
        conflict_vec[conflict_vec_cnt].push_back(rbi);
        for (int j = 0; j < robot_num; ++j) {
            rbj = robot_priority[j];
            if (rbj == rbi) continue;
            if (robot[rbj].is_safe) continue;
            if (dsu.find(rbj) == rbi) {
                conflict_vec[conflict_vec_cnt].push_back(rbj);
            }
        }
        conflict_vec_cnt++;
    }

    for (int i = 0; i < conflict_vec_cnt; ++i) {
        if (!conflict_dfs(i, 0)) {
            fprintf(stderr, "# ERROR: NO POSSIBLE SOLUTION\n");
            conflict_error = true;
        }
    }

}

int ship_capacity;
class Ship {
public:
    Ship() : status(SHIP_NORMAL), berth_id(-1), frame(1) {};

    void go() {
        if (berth_id == -1) return;
#ifdef DEBUG_SHIP
        fprintf(stderr, "#SHIP:%d LEAVE %d\n", id, berth_id);
#endif
        printf("go %d\n", id);
        berth[berth_id].occupy -= 1;
        berth[berth_id].capacity -= ship_capacity - loads; // 没装满的部分 从记录中撤回来
        count_down = berth[berth_id].transport_time;
        berth_id = -1;
        status = SHIP_SHIPPING;
        loads = 0;
        return;
    }

    void move(int dest) {
        if (berth_id == -1) {
            count_down = berth[dest].transport_time;
            if (frame + 2 * berth[dest].transport_time > 15000) return;
        }
        else {
            berth[berth_id].occupy -= 1;
            berth[berth_id].capacity -= ship_capacity - loads;
            count_down = 500;
        }
        berth[dest].occupy += 1;
        berth[dest].capacity += ship_capacity - loads;
        berth_id = dest;
        status = SHIP_SHIPPING;
#ifdef DEBUG_SHIP
        fprintf(stderr, "#SHIP:%d TRANSPORT to %d\n", id, dest);
#endif
        printf("ship %d %d\n", id, dest);
        return;
    }

    void update(int state_code, int _id, int _frame) {
        frame++;
#ifdef DEBUG_SHIP
        if (_frame != frame) {
            fprintf(stderr, "#SHIP:%d FRAME SYNC FAILD rec: %d, get: %d\n", id, frame, _frame);
        }
#endif
        if (_frame != frame) {
            count_down -= _frame - frame - 1;
            frame = _frame;
        }
        count_down--;
        if (state_code == 0)
            _status = SHIP_SHIPPING;
        else if (state_code == 1)
            _status = SHIP_NORMAL;
        else {
            _status = SHIP_WAITING;
        }

        if (status == SHIP_SHIPPING) {
            if (count_down == 0) {
                status = SHIP_NORMAL;
#ifdef DEBUG_SHIP
                fprintf(stderr, "#SHIP:%d Arrive berth:%d\n", id, berth_id);
#endif
            }
        }

        if (_status != status || _id != berth_id) {
//#ifdef DEBUG_SHIP
            fprintf(stderr, "#ERROR Ship %d Sync Faild\n", id);
            fprintf(stderr, "#status: rec %d, get %d\n", status, _status);
            fprintf(stderr, "#berth_id: rec %d, get %d\n", berth_id, _id);
            fprintf(stderr, "#count_down: %d\n", count_down);
//#endif
            status = _status;
            berth_id = _id;
        }
        if (status == SHIP_WAITING) count_down++;

    }

    void act() {
        if (berth_id != -1 && frame + berth[berth_id].transport_time == 15000) {
#ifdef DEBUG_SHIP
            fprintf(stderr, "#SHIP%d: TIME UP, frame:%d, time:%d, stock_remain:%d\n", id, frame, berth[berth_id].transport_time, berth[berth_id].stock);
            fprintf(stderr, "#status: rec %d\n", status);
            fprintf(stderr, "#berth_id: rec %d\n", berth_id);
#endif
            go();
            return;
        }
        if (status == SHIP_NORMAL) {
#ifdef DEBUG_SHIP
//            fprintf(stderr, "#SHIP:%d normal count_down: %d\n", id, count_down);
#endif
            if (berth_id == -1) {
                int next_berth = calc_next_berth();
                fprintf(stderr, "#SHIP:%d BEGIN, to: %d\n", id, next_berth);
                move(next_berth);
                return;
            }
            // 在泊位上

            // loads < ship_capacity
            int max_obj_num = inf_dist;
            max_obj_num = min(max_obj_num, berth[berth_id].stock);
            max_obj_num = min(max_obj_num, berth[berth_id].velocity);
            max_obj_num = min(max_obj_num, ship_capacity - loads);
            berth[berth_id].stock -= max_obj_num;
            loads += max_obj_num;
            berth[berth_id].capacity -= max_obj_num;
            for (int i = 0; i < max_obj_num; ++i) {
                berth[berth_id].remain_value -= berth[berth_id].stocks.front();
                berth[berth_id].stocks.pop();
            }
            if (max_obj_num == berth[berth_id].velocity) v_full++;

            if (loads == ship_capacity) {
                // 装满了，走人
#ifdef DEBUG_SHIP
                fprintf(stderr, "#SHIP:%d FULL, count down: %d\n", id, count_down);
#endif
                go();
                return;
            }

//            if (count_down <= -20) {
//                int next_berth = calc_next_berth();
//                if (next_berth == berth_id) return;
//                if (frame + 500 + berth[next_berth].transport_time > 15000) return;
//                move(next_berth);
//            }
            if (berth[berth_id].stock == 0) {
                int next_berth = calc_next_berth();
                if (next_berth == berth_id) return;
                if (frame + 500 + berth[next_berth].transport_time > 15000) return;
                if (rand() % 100 < 10) {
#ifdef DEBUG_SHIP
                    fprintf(stderr, "#SHIP:%d EARLY LEAVE %d, loads_remain: %d\n", id, berth_id, ship_capacity - loads );
#endif
                    move(next_berth); // 平均期望也是20次就走？
                    return;
                }
//                if (frame > 8000 && rand() % 100 < 10) {
//#ifdef DEBUG_SHIP
//                    fprintf(stderr, "#SHIP:%d EARLY LEAVE %d, loads_remain: %d\n", id, berth_id, ship_capacity - loads );
//#endif
//                    move(next_berth); // 平均期望也是20次就走？
//                    return;
//                }
            }
        }
    }

    int calc_next_berth() {
        if (frame <= 5) {
            for (int i = rand(); ; i = rand()) {
                fprintf(stderr, "rand: %d\n", i % berth_num);
                if (berth[i % berth_num].occupy > 0) continue;
                return i % berth_num;
            }
        }
        int max_stock = 0, next_berth_id = berth_id;
        for (int i = 0; i < berth_num; ++i) {
            if (berth[i].stock + berth[i].booked - berth[i].capacity > max_stock) {
                max_stock = berth[i].stock + berth[i].booked - berth[i].capacity;
                next_berth_id = i;
            }
        }
        return next_berth_id;
//        if (max_stock - berth[berth_id].stock > ship_capacity) {
//            return next_berth_id;
//        }
//        else {
//            return berth_id;
//        }
    }

    int id, frame;
    ship_enum status, _status;
    int berth_id;
    int loads;
    int count_down;
} ship[ship_num+1];


struct PQnode {
    PQnode() {};
    PQnode(int _x, int _y, int _d) : x(_x), y(_y), d(_d) {};
    int x, y, d;

    bool operator < (const PQnode &x) const {
        return d > x.d;
    }
};
std::priority_queue<PQnode> pqueue;

void dijkstra(int berth_id) {
    // 给每个泊位计算最短路
    int base_x, base_y;
    base_x = berth[berth_id].x;
    base_y = berth[berth_id].y;

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            mat[ base_x + i ][ base_y + j ].dist[berth_id] = 0;
            pqueue.push(PQnode(base_x + i, base_y + j, 0));
        }
    }

    int tx, ty;
    int vis[201][201];
    for (int i = 0; i < 200; ++i) {
        for (int j = 0; j < 200; ++j) {
            vis[i][j] = 0;
        }
    }

    while (!pqueue.empty()) {
        PQnode p = pqueue.top();
        pqueue.pop();
        if (vis[p.x][p.y]) continue;
//        fprintf(stderr, "dijk (%d,%d) d:%d\n", p.x, p.y, p.d);
        vis[p.x][p.y] = 1;
        mat[p.x][p.y].dist[berth_id] = p.d;

        for (int i = 0; i < 4; ++i) {
            tx = p.x + mov_x[i];
            ty = p.y + mov_y[i];

            if (in_mat(tx, ty) && is_land(tx, ty)) {
                if (mat[tx][ty].dist[berth_id] > p.d + 1) {
                    pqueue.push(PQnode(tx, ty, p.d+1));
                }
            }
        }
    }

}

void debug_print_dist(int berth_id) {
    for (int i = 0; i < 200; ++i) {
        for (int j = 0; j < 200; ++j) {
            int x = mat[i][j].dist[berth_id];
            if (x == inf_dist) x = -1;
            fprintf(stderr, "%3d", x);
        }
        fprintf(stderr, "\n");
    }
    fflush(stderr);
}


void Init() {
    srand((unsigned int)time(NULL));
    // 地图+机器人
    char tmp[210];
    for (int row = 0; row < 200; ++row) {
        scanf("%s", tmp);
        for (int col = 0; col < 200; ++col) {
            switch (tmp[col]) {
                case '.': mat[row][col].type = LAND; break;
                case '*': mat[row][col].type = OCEAN; break;
                case '#': mat[row][col].type = HILL; break;
                case 'A':
                    mat[row][col].type = LAND;
                    robot[robot_cnt] = Robot(row, col, robot_cnt);
                    robot_cnt++;
                    break;
                case 'B': mat[row][col].type = BERTH; break;
            }
            for (int i = 0; i < berth_num; ++i) {
                mat[row][col].dist[i] = inf_dist;
            }
        }
    }


    // 泊位信息
    for (int i = 0; i < 10; ++i) {
        scanf("%d%d%d%d%d", &berth[i].id, &berth[i].x, &berth[i].y, &berth[i].transport_time, &berth[i].velocity);
    }

    // 船舶容量
    scanf("%d", &ship_capacity);

    char okk[100];
    scanf("%s", okk);

    // 初始化船
    for (int i = 0; i < ship_num; ++i) {
        ship[i].id = i;
        ship[i].frame = 0;
    }

#ifdef DEBUG_FLAG
    for (int r = 0; r < 10; ++r) {
        fprintf(stderr, "robot %d: (%d, %d)\n", r, robot[r].x, robot[r].y);
    }
    for (int i = 0; i < 10; ++i) {
        fprintf(stderr, "berth %d: (%d, %d) t:%d v:%d\n", berth[i].id, berth[i].x, berth[i].y, berth[i].transport_time, berth[i].velocity);
    }
    fprintf(stderr, "capacity %d\n", ship_capacity);
    fprintf(stderr, "OK from Judge: %s\n", okk);
#endif

    /*
     * 给每一个泊位都初始化一下全图的最短路
     * 存在mat[x][y].dist[berth_id]里
     * 可以在机器人建起物品后快速判断去哪个泊位
     * 同时也用于去向泊位的过程中选择路径
     * */
    for (int i = 0; i < berth_num; ++i) {
        dijkstra(i);
    }


    // 每个机器人初始化可到达的范围、可到达的港口
    for (int i = 0; i < robot_num; ++i) {
        robot[i].init();
    }
//    debug_print_dist(0);

    /* 计算港口价值
     * 价值 = \sum 1/(dis[x][y])
     * 对于多个地方都能抵达的陆地，对港口i的贡献为
     * (1/Di) * [(1/Di) / (1/Di + 1/Dj + 1/Dk)]
     *
     * */
    double dominator = 0; // 分母
    for (int x = 0; x < 200; ++x) {
        for (int y = 0; y < 200; ++y) {
            dominator = 0;
            for (int i = 0; i < berth_num; ++i) {
                if (mat[x][y].dist[i] != 0 && mat[x][y].dist[i] != inf_dist) {
                    dominator += 1.0 / (double)mat[x][y].dist[i];
                }
            }
            for (int i = 0; i < berth_num; ++i) {
                if (mat[x][y].dist[i] != 0 && mat[x][y].dist[i] != inf_dist) {
                    berth[i].value += 1.0 / ((double)mat[x][y].dist[i] * (double)mat[x][y].dist[i]) / dominator;
                }
            }
        }
    }
    /* 港口可调度的机器人数量
     * 直接乘进港口价值
     * */
    int sum_robot;
    for (int i = 0; i < berth_num; ++i) {
        sum_robot = 0;
        for (int j = 0; j < robot_num; ++j) {
            if (mat[ robot[j].x ][ robot[j].y ].dist[i] != inf_dist) sum_robot++;
        }
        berth[i].value *= sum_robot;
    }


//    calc_pd();
    printf("OK\n");
    fflush(stdout);
#ifdef DEBUG_FLAG
    fprintf(stderr, "#1 Init Finish\n");
#endif
}

void Input(int &frame_id) {
    int money;
    int k;
    scanf("%d%d", &frame_id, &money);
    scanf("%d", &k);

    fprintf(stderr, "#2 Input frame: %d, money: %d, new_obj: %d\n", frame_id, money, k);
    // 新增的货物信息
    int x, y, value;
    for (int i = 0; i < k; ++i) {
        scanf("%d%d%d", &x, &y, &value);
        objects.push_back(Obj(x, y, value, frame_id));
        mat[x][y].has_obj = true;
    }

    // 当前的机器人信息
    int carry, status; // 还有x, y
    for (int i = 0; i < 10; ++i) {
        scanf("%d%d%d%d", &carry, &x, &y, &status);
        robot[i].sync(carry, x, y, status, frame_id);
    }

    // 当前的船信息
    int dest_berth;
    for (int i = 0; i < 5; ++i) {
        scanf("%d%d", &status, &dest_berth);
        ship[i].update(status, dest_berth, frame_id);
    }
    char okk[100];
    scanf("%s", okk);
}


void calc_close_berth(int frame) {
    for (int i = 0; i < berth_num; ++i) {
        if (berth[i].closed) continue;
        if (berth[i].occupy == 0 && berth[i].transport_time + 500 + frame > 15000) {
            fprintf(stderr, "# BERTH%d close\n", i);
            berth[i].closed = 1;
        }
    }
}



int main() {
#ifdef DEBUG_FLAG
    fprintf(stderr, "#0 Program start");
#endif
    Init();
    int frame_id;
    for (int frame = 1; frame <= 15000; frame++) {

        Input(frame_id);
#ifdef DEBUG_FLAG
        fprintf(stderr, "#2 Input Finish\n");
        if (frame_id != frame) {
            fprintf(stderr, "# LOOP SYNC ERROR: loop:%d, frame get:%d\n", frame, frame_id);
        }
#endif
        if (frame_id != frame) {
            frame = frame_id;
        }
        remove_outdated_obj(frame_id);

        for (int i = 0; i < robot_num; ++i) robot[i].think();
        handle_conflict();
        if (conflict_error) {
            fprintf(stderr, "-> frame: %d, frame_id: %d\n", frame, frame_id);
        }

#ifdef DEBUG_SHIP
        fprintf(stderr, "SHIPS: \n");
        for (int i = 0; i < ship_num; ++i) {
            fprintf(stderr, "     - %d in %d\n", i, ship[i].berth_id);
        }
#endif

        for (int i = 0; i < robot_num; ++i) robot[i].act();
        for (int i = 0; i < ship_num; ++i) ship[i].act();
//        if (frame == 12500) {
//            for (int i = 0; i < 5; ++i) {
//                if (ship[i].loads > ship_capacity * 0.8) ship[i].go();
//            }
//        }
        if (frame > 12000) calc_close_berth(frame);
        printf("OK\n");
        fflush(stdout);
#ifdef DEBUG_FLAG
        fprintf(stderr, "#4 Output Finish\n");
#endif
    }
    fprintf(stderr, "v_full speed: %d\n\n", v_full);
#ifdef DEBUG_BERTH
    fprintf(stderr, "#TOTAL VALUE %d\n", tot_value);
    for (int i = 0; i < berth_num; ++i) {
        fprintf(stderr, "#BERTH%d: remain value:%d, tot value:%d, tot stock %d, remain stock %d, t_time %d\n", i, berth[i].remain_value, berth[i].tot_value, berth[i].tot_stock, berth[i].stock, berth[i].transport_time);
    }
    fprintf(stderr, "\n\n");
    fflush(stderr);
#endif
    return 0;
}