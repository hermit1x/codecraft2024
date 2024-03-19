#include <iostream>
#include <algorithm>
#include <queue>
#include <vector>
#include <memory>
#include <time.h>

#define DEBUG_FLAG
#define G 16
#define min(x, y) ((x) < (y) ? (x) : (y))

enum mat_enum { LAND, OCEAN, HILL, BERTH };
enum robot_enum { NOTHING, TO_OBJ, TO_SHIP, RECOVERY };
enum robot_mov { ROBOT_MOV, ROBOT_GET, ROBOT_PULL, ROBOT_RECOVER, ROBOT_TICK };
enum ship_enum { SHIP_SHIPPING, SHIP_NORMAL, SHIP_WAITING };

const int robot_num = 10;
const int berth_num = 10;
const int ship_num = 5;
const int inf_dist = 1e6;

int tot_value = 0;

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
    fprintf(stderr, "# Frame %d objects.size() = %lu\n", frame_id, objects.size());
}

class Berth {
public:
    Berth() {}

    int id, x, y, transport_time, velocity;
    int occupy;
    int stock;
}berth[berth_num+1];

struct PQnode2 {
    PQnode2() {};
    PQnode2(int _x, int _y, int _d, int _dr) : x(_x), y(_y), d(_d), dr(_dr) {};
    int x, y, d, dr; // dist real

    bool operator < (const PQnode2 &x) const {
        return d > x.d;
    }
};
std::priority_queue<PQnode2> pqueue2;

int calc_force(int id, int tx, int ty);
class Robot {
public:
    Robot() {}
    Robot(int _x, int _y, int _id) : x(_x), y(_y), id(_id), status(NOTHING), frame(1) {}

    void calc_want() {
        if (status == NOTHING) {
            fprintf(stderr, "# Robot %d resting, assign obj\n", id);
            int obj_id = calc_best_obj();

            if (obj_id == -1) {
//                fprintf(stderr, "# Robot %d assign failed, NO Object\n", id);
                want_mov = ROBOT_TICK;
                want_dir = 5;
                return;
            }

            obj = objects[obj_id];
            tot_value += obj.value;
            fprintf(stderr, "OBJ VALUE: %d\n", obj.value);
            objects.erase(objects.begin() + obj_id);

            calc_path_to_obj(obj.x, obj.y);
//            fprintf(stderr, "# Robot %d assign finish\n", id);
            status = TO_OBJ;
        }
        if (status == RECOVERY) {
            want_mov = ROBOT_RECOVER;
            want_dir = 5;
            return;
        }
        if (status == TO_OBJ) {
            if (dis[x][y] == 0 && mat[x][y].has_obj) {
                want_mov = ROBOT_GET;
                want_dir = 5;
                calc_best_berth();
                return;
            }
            else if (dis[x][y] == 0) {
                fprintf(stderr, "# Robot %d ERROR, No obj found at (%d,%d)!\n", id, x, y);
                status = NOTHING;
                want_mov = ROBOT_TICK;
                want_dir = 5;
                return;
            }
            if (dis[x][y] == inf_dist) {
                calc_path_to_obj(obj.x, obj.y);
                return;
            }
            // 用人工势场法避障
            if (calc_force(id, x, y) > 3) {
                calc_path_to_obj(obj.x, obj.y);
                return;
            }
            int dir = 5, min_dis = inf_dist;
            int cur_dist = dis[x][y], tx, ty, tf;
            int rand_base = rand();
            for (int i, j = 0; j < 4; ++j) {
                i = (rand_base + j) % 4;
                tx = x + mov_x[i];
                ty = y + mov_y[i];
                if (in_mat(tx, ty) && is_land(tx, ty)) {
                    tf = calc_force(id, tx, ty);
                    if (min_dis > (dis[tx][ty] - cur_dist) + tf) {
                        min_dis = (dis[tx][ty] - cur_dist) + tf;
                        dir = i;
                    }
                }
            }
            want_mov = ROBOT_MOV;
            want_dir = dir;
            return;
        }
        if (status == TO_SHIP) {
            int cur_dist = mat[x][y].dist[dest_berth];
            if (cur_dist == 0) {
                // 进入泊位
//                fprintf(stderr, "# Robot %d Enter Berth %d\n", id, dest_berth);
                want_mov = ROBOT_PULL;
                want_dir = 5;
                return;
            }
            int dir = 5, min_dis = inf_dist;
            // 用人工势场法避障
            int cur_f = calc_force(id, x, y), tx, ty, tf;
            cur_f = 0;
            int rand_base = rand();
            for (int i, j = 0; j < 4; ++j) {
                i = (rand_base + j) % 4;
                tx = x + mov_x[i];
                ty = y + mov_y[i];
                if (in_mat(tx, ty) && is_land(tx, ty)) {
                    tf = calc_force(id, tx, ty);
                    if (min_dis > (mat[tx][ty].dist[dest_berth] - cur_dist) + (tf - cur_f)) {
                        min_dis = (mat[tx][ty].dist[dest_berth] - cur_dist) + (tf - cur_f);
                        dir = i;
                    }
                }
            }
            if (dir == 5) {
                fprintf(stderr, "# Robot %d ERROR, No way to ship!\n", id);
            }
            want_mov = ROBOT_MOV;
            want_dir = dir;
        }
    }

    void move() {
        frame++;
        switch (want_mov) {
            case ROBOT_MOV:
                printf("move %d %d\n", id, want_dir);
                x += mov_x[want_dir];
                y += mov_y[want_dir];
//                fprintf(stderr, "# ROBOT %d move: %d\n", id, want_dir);
                break;
            case ROBOT_GET:
                printf("get %d\n", id);
//                fprintf(stderr, "# ROBOT %d get\n", id);
                mat[x][y].has_obj = false;
                calc_best_berth();
                status = TO_SHIP;
                break;
            case ROBOT_PULL:
                printf("pull %d\n", id);
//                fprintf(stderr, "# ROBOT %d pull\n", id);
                berth[dest_berth].stock += 1;
                status = NOTHING;
                break;
            case ROBOT_RECOVER:
//                fprintf(stderr, "# ROBOT %d recovering\n", id);
                break;
            case ROBOT_TICK:
//                fprintf(stderr, "# ROBOT %d waiting...\n", id);
                break;
        }
    }

    void calc_path_to_obj(int vx, int vy) {
//        moves = get_path(x, y, obj_x, obj_y);
        /* A* 寻路，评估权重是 已经走的距离+曼哈顿距离
         * 同时存下来搜的时候的距离矩阵，供人工势场
         * */
        for (int i = 0; i < 200; ++i) {
            for (int j = 0; j < 200; ++j) {
                dis[i][j] = inf_dist;
                vis[i][j] = 0;
            }
        }

        pqueue2.push(PQnode2(vx, vy, 0, 0));
        int tx, ty;
        while (!pqueue2.empty()) {
            PQnode2 p = pqueue2.top();
            pqueue2.pop();
//        fprintf(stderr, "PQ2 loop: (%d, %d): %d\n", p.x, p.y, p.dr);
            if (vis[p.x][p.y]) continue;
            vis[p.x][p.y] = 1;
            dis[p.x][p.y] = p.dr;

            if (p.x == x && p.y == y) {
//            fprintf(stderr, "Dijk Early Stop!!\n");
                break;
            }

            int rand_base = rand() % 4;
            for (int i, j = 0; j < 4; ++j) {
                i = (rand_base + j) % 4;
                tx = p.x + mov_x[i];
                ty = p.y + mov_y[i];

                if (in_mat(tx, ty) && is_land(tx, ty)) {
                    if (p.dr < dis[tx][ty]) {
                        pqueue2.push(PQnode2(
                                tx, ty,
                                p.dr + 2 * dis_man(tx, ty, x, y) + calc_force(id, tx, ty), // 加入曼哈顿距离，A star 思想
                                p.dr + 1));
                    }
                }
            }
        }

        while (!pqueue2.empty()) {
            PQnode2 p = pqueue2.top();
            pqueue2.pop();
            if (vis[p.x][p.y]) continue;
            vis[p.x][p.y] = 1;
            dis[p.x][p.y] = p.dr;
        }

#ifdef DEBUG_FLAG
//    fprintf(stderr, "#3 get_path() final dist: %d\n", dis[x][y]);
//    for (int i = 0; i < 200; ++i) {
//        for (int j = 0; j < 200; ++j) {
//            int x = dis[i][j];
//            if (x == inf_dist) x = -1;
//            fprintf(stderr, "%4d", x);
//        }
//        fprintf(stderr, "\n");
//    }
//    fflush(stderr);
#endif

    }

    void calc_best_berth() {
//        int min_dis = inf_dist, min_berth = -1;
//        for (int i = 0; i < berth_num; ++i) {
//            if (min_dis > mat[x][y].dist[i]) {
//                min_dis = mat[x][y].dist[i];
//                min_berth = i;
//            }
//        }
//        dest_berth = min_berth;
        dest_berth = 0;
    }

    void update(int _carry, int _x, int _y, int status_code, int _frame) {
        if (_x != x || _y != y || _frame != frame) {
            fprintf(stderr, "# ROBOT %d Sync Error\n", id);
            fprintf(stderr, "# rec: (%d, %d) %d\n", x, y, frame);
            fprintf(stderr, "# get: (%d, %d) %d\n", _x, _y, _frame);
            x = _x; y = _y; frame = _frame;
        }

        if (status_code == 0) {
            fprintf(stderr, "#ERROR# ROBOT %d has code 0\n", id);
            if (status == RECOVERY) return;
            status = RECOVERY;
        }
        if (pre_status == 0 && status_code == 1) {
            // 恢复信息
            if (_carry) {
                // TODO:还差一帧从OBJ位置捡东西的
                status = TO_SHIP;
            }
            else {
                status = NOTHING;
            }
        }
        pre_status = status_code;
    }

    int calc_best_obj() {
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
        /*
        // 简单以最近的来选
        int min_dis = inf_dist, res = -1;
        int obj_x, obj_y, obj_dis;
        for (int i = 0; i < objects.size(); ++i) {
            obj_x = objects[i].x;
            obj_y = objects[i].y;
            obj_dis = mat[obj_x][obj_y].dist[dest_berth];
            if (objects[i].time_appear + 1000 < frame + obj_dis) continue;
            if (min_dis > obj_dis) {
                min_dis = obj_dis;
                res = i;
            }
        }
        return res;
         */
    }

    void assign(int dir) {
        want_dir = dir;
//        if (status == TO_OBJ)
//            moves->push_back(want_dir);
//        want_dir = dir;
//        if (dir != 4)
//            moves->push_back(dir ^ 1);
//        fprintf(stderr, "# ROBOT %d assign: %d\n", id, dir);
    }



    int id, x, y, frame, pre_status;
    int want_dir, dest_berth;
    int dis[201][201], vis[201][201];
    Obj obj;
    robot_enum status;
    robot_mov want_mov;
} robot[robot_num+1];
int robot_cnt = 0;

int calc_force(int id, int tx, int ty) {
    int f = 0, d;
    for (int i = 0; i < id; ++i) {
//        if (i == id) continue;
        d = dis_man(tx, ty, robot[i].x, robot[i].y);
        f += G / (d*d);
        d = dis_man(tx, ty, robot[i].x + mov_x[robot[i].want_dir], robot[i].y + mov_y[mov_x[robot[i].want_dir]]);
        f += G / (d*d);
    }
    return f;
}

void handle_conflict() {
//    fprintf(stderr, "#3 Handle Conflict\n");
    static int m[201][201];
    for (int i = 0; i < 200; ++i) {
        for (int j = 0; j < 200; ++j) {
            m[i][j] = 0;
        }
    }
    // 算势场

    /* 1. 有货的优先
     * 2. id小的优先
     * */
    static int robot_p[robot_num+1];
    int robot_pp = 0;
    for (int i = 0; i < robot_num; ++i) {
        if (robot[i].status == TO_SHIP) robot_p[robot_pp++] = i;
    }
    for (int i = 0; i < robot_num; ++i) {
        if (robot[i].status != TO_SHIP) robot_p[robot_pp++] = i;
    }

    // 先占住当前的位置
    // TODO:把这个m合并到mat里
    int ri, rdir, rnx, rny;
    for (int i = 0; i < robot_num; ++i) {
        ri = robot_p[i];
        m[robot[ri].x][robot[ri].y] = 1;
    }

    for (int i = 0; i < robot_num; ++i) {
        ri = robot_p[i];
        rdir = robot[ri].want_dir;
        if (rdir == 5) continue;
        rnx = robot[ri].x + mov_x[rdir];
        rny = robot[ri].y + mov_y[rdir];
        if (!m[rnx][rny]) {
            m[rnx][rny] = 1;
            continue;
        }
        else {
            int rand_base = rand();
//            int rand_base;
//            if (rdir == 0 || rdir == 1) rand_base = 2; // 先向自己左手边走
//            if (rdir == 2) rand_base = 3;
//            if (rdir == 2) rand_base = 1;
            if (rand() % 5 == 0) {
                robot[ri].assign(4);
                return;
            }
            for (int k = 0, j; k < 5; ++k) {
                if (k == 4) {
                    robot[ri].assign(4);
                    break;
                }
                j = (k + rand_base) % 4;
                rnx = robot[ri].x + mov_x[j];
                rny = robot[ri].y + mov_y[j];
                if (!m[rnx][rny]) {
                    m[rnx][rny] = 1;
                    robot[ri].assign(j);
                    break;
                }
            }
        }
    }
}


int ship_capacity;
class Ship {
public:
    Ship() {};

    void go() {
        if (berth_id == -1) return;
        fprintf(stderr, "#SHIP LEAVE\n");
        printf("go %d\n", id);
        berth[berth_id].occupy = 0;
        berth_id = -1;
        status = SHIP_SHIPPING;
        return;
    }

    void update(int state_code, int _id, int _frame) {
        frame++;
        if (state_code == 0)
            _status = SHIP_SHIPPING;
        else if (state_code == 1)
            _status = SHIP_NORMAL;
        else {
            _status = SHIP_WAITING;
        }

        if (_status != status || _id != berth_id) {
            fprintf(stderr, "#ERROR Ship %d Sync Faild\n", id);
            status = _status;
            berth_id = _id;
        }
        if (status == SHIP_NORMAL) {
            if (berth_id == -1) {
//                for (int i = 0; i < berth_num; ++i) {
//                    if (berth[i].occupy == 0) {
//                        berth[i].occupy = 1;
//                        fprintf(stderr, "#SHIP %d CAME %d\n", id, i);
//                        berth_id = i;
//                        state_code = SHIP_SHIPPING;
//                        return;
//                    }
//                }
                if (!berth[0].occupy) {
                    fprintf(stderr, "#SHIP %d CAME %d\n", id, 0);
                    printf("ship %d %d\n", id, 0);
                    berth_id = 0;
                    state_code = SHIP_SHIPPING;
                    return;
                }
            }
            // 在泊位上
            if (loads == ship_capacity) {
                // 装满了，走人
                fprintf(stderr, "#SHIP FULL\n");
                printf("go %d\n", id);
                berth[berth_id].occupy = 0;
                berth_id = -1;
                status = SHIP_SHIPPING;
                return;
            }
            // loads < ship_capacity
            int max_obj_num = inf_dist;
            max_obj_num = min(max_obj_num, berth[berth_id].stock);
            max_obj_num = min(max_obj_num, berth[berth_id].velocity);
            max_obj_num = min(max_obj_num, ship_capacity - loads);
            berth[berth_id].stock -= max_obj_num;
            loads += max_obj_num;
        }
    }

    int id, frame;
    ship_enum status, _status;
    int berth_id;
    int loads;
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
//    debug_print_dist(0);


    fprintf(stdout, "OK\n");
    fflush(stdout);

    fprintf(stderr, "#1 Init Finish\n");
}

void Input(int &frame_id) {
    int money;
    int k;
    scanf("%d%d", &frame_id, &money);
    scanf("%d", &k);

//    fprintf(stderr, "#2 Input frame: %d, money: %d, k: %d\n", frame_id, money, k);
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
        robot[i].update(carry, x, y, status, frame_id);
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



int main() {
    fprintf(stderr, "#0 Program start");
    Init();
    int frame_id;
    for (int frame = 0; frame < 15000; frame++) {
        if (frame == 12900) {
            for (int i = 0; i < 5; ++i) {
                ship[i].go();
            }
        }
        Input(frame_id);
        fprintf(stderr, "#2 Input Finish\n");
        remove_outdated_obj(frame_id);

        for (int i = 0; i < robot_num; ++i) robot[i].calc_want();
        handle_conflict();
        for (int i = 0; i < robot_num; ++i) robot[i].move();
        fprintf(stdout, "OK\n");
        fflush(stdout);
        fprintf(stderr, "#4 Output Finish\n");
    }
    fprintf(stderr, "#TOTAL VALUE %d\n", tot_value);
    return 0;
}