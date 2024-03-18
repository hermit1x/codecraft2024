#include <iostream>
#include <algorithm>
#include <queue>
#include <vector>
#include <memory>
#include <time.h>


#define DEBUG_FLAG

enum mat_enum { LAND, OCEAN, HILL, BERTH };
enum robot_enum { NOTHING, TO_OBJ, TO_SHIP, RECOVERY };
enum robot_mov { ROBOT_MOV, ROBOT_GET, ROBOT_PULL, ROBOT_RECOVER, ROBOT_TICK };
enum ship_enum { SHIPPING, NORMAL, WAITING };

const int robot_num = 10;
const int berth_num = 10;
const int ship_num = 5;
const int inf_dist = 1e6;

// 右 左 上 下 不动
const int mov_x[6] = {0, 0, -1, 1, 0, 0};
const int mov_y[6] = {1, -1, 0, 0, 0, 0};


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
    Obj() {};
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

std::shared_ptr<std::vector<int> > get_path(int ux, int uy, int vx, int vy);
class Robot {
public:
    Robot() {}
    Robot(int _x, int _y, int _id) : x(_x), y(_y), id(_id), status(NOTHING), frame(1) {}

    void calc_want() {
        if (status == NOTHING) {
            fprintf(stderr, "# Robot %d resting, assign obj\n", id);
            int obj_id = calc_best_obj();

            if (obj_id == -1) {
                fprintf(stderr, "# Robot %d assign error, NO Object\n", id);
                want_mov = ROBOT_TICK;
                return;
            }

            Obj obj = objects[obj_id];
            objects.erase(objects.begin() + obj_id);

            calc_path_to_obj(obj.x, obj.y);
            fprintf(stderr, "# Robot %d assign finish\n", id);
            status = TO_OBJ;
        }
        if (status == RECOVERY) {
            want_mov = ROBOT_RECOVER;
            want_dir = 5;
            return;
        }
        if (!moves->empty()) {
            want_mov = ROBOT_MOV;
            want_dir = *(moves->end() - 1);
            moves->pop_back();
            return;
        }
        if (status == TO_OBJ) {
            if (mat[x][y].has_obj) {
                want_mov = ROBOT_GET;
                want_dir = 5;
                calc_best_berth();
            }
            else {
                fprintf(stderr, "# Robot %d ERROR, No obj found at (%d,%d)!\n", id, x, y);
            }
            return;
        }
        if (status == TO_SHIP) {
            int cur_dist = mat[x][y].dist[dest_berth];
            if (cur_dist == 0) {
                // 进入泊位
                fprintf(stderr, "# Robot %d Enter Berth %d\n", id, dest_berth);
                want_mov = ROBOT_PULL;
                return;
            }
            int dir = 5;
            int rand_base = rand();
            for (int i, j = 0; j < 4; ++j) {
                i = (rand_base + j) % 4;
                if (mat[x+mov_x[i]][y+mov_y[i]].dist[dest_berth] == cur_dist - 1) {
                    dir = i;
                    break;
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
                fprintf(stderr, "# ROBOT %d move: %d\n", id, want_dir);
                break;
            case ROBOT_GET:
                printf("get %d\n", id);
                fprintf(stderr, "# ROBOT %d get\n", id);
                mat[x][y].has_obj = false;
                calc_best_berth();
                status = TO_SHIP;
                break;
            case ROBOT_PULL:
                printf("pull %d\n", id);
                fprintf(stderr, "# ROBOT %d pull\n", id);
                status = NOTHING;
                break;
            case ROBOT_RECOVER:
                fprintf(stderr, "# ROBOT %d recovering\n", id);
                break;
            case ROBOT_TICK:
                fprintf(stderr, "# ROBOT %d waiting...\n", id);
                break;
        }
    }

    void calc_path_to_obj(int obj_x, int obj_y) {
        moves = get_path(x, y, obj_x, obj_y);
    }

    void calc_best_berth() {
        int min_dis = inf_dist, min_berth = -1;
        for (int i = 0; i < berth_num; ++i) {
            if (min_dis > mat[x][y].dist[i]) {
                min_dis = mat[x][y].dist[i];
                min_berth = i;
            }
        }
        dest_berth = min_berth;
    }

    void update(int _carry, int _x, int _y, int status_code, int _frame) {
        if (_x != x || _y != y || _frame != frame) {
            fprintf(stderr, "# ROBOT %d Sync Error\n", id);
            fprintf(stderr, "# rec: (%d, %d) %d\n", x, y, frame);
            fprintf(stderr, "# get: (%d, %d) %d\n", _x, _y, _frame);
        }
        x = _x; y = _y; frame = _frame;

        if (status_code == 0) {
            fprintf(stderr, "# ROBOT %d get Error\n", id);
            if (status == RECOVERY) return;
            status = RECOVERY;
            moves->push_back(want_dir);
        }
        if (pre_status == 0 && status_code == 1) {
            // 恢复信息
            if (!moves->empty()) {
                // 有指定任务
                status = TO_OBJ;
            }
            else if (_carry) {
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
    }

    int id, x, y, status, pre_status, frame;
    int want_dir, dest_berth;
    robot_mov want_mov;
    std::shared_ptr<std::vector<int> > moves = std::make_shared<std::vector<int> >();
} robot[robot_num+1];
int robot_cnt = 0;

class Berth {
public:
    Berth() {}

    bool is_inside(int px, int py) const {
        return x <= px && px < x + 4 && y <= py && py < y + 4;
    }

    int id, x, y, transport_time, velocity;
}berth[berth_num+1];

class Ship {
public:
    Ship() {};

    ship_enum status;
    int berth_id;
    int loads;
} ship[ship_num+1];

int ship_capacity;

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
#ifdef DEBUG_FLAG
//    fprintf(stderr, "#2 Input start\n");
    fflush(stderr);
#endif
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
    }
    char okk[100];
    scanf("%s", okk);

#ifdef DEBUG_FLAG
//    fprintf(stderr, "#2 Input finish\n");
    fflush(stderr);
#endif
}

//int dist_robot_obj_fake(int rx, int ry, int ox, int oy) {
//    /*
//     * 重要函数，玄学评估机器人和物品的距离
//     * 目前采用的思路是：曼哈顿距离*20 + 每个港口的条件下两个点的距离差（表示层次接近）
//     * */
//    int sum = 20 * (abs(rx - ox) + abs(ry - oy)); // 曼哈顿距离
//    for (int i = 0; i < berth_num; ++i) {
//        sum += abs( mat[rx][ry].dist[i] - mat[ox][oy].dist[i] );
//    }
//
//    return sum;
//}


struct PQnode2 {
    PQnode2() {};
    PQnode2(int _x, int _y, int _d, int _dr) : x(_x), y(_y), d(_d), dr(_dr) {};
    int x, y, d, dr; // dist real

    bool operator < (const PQnode2 &x) const {
        return d > x.d;
    }
};
std::priority_queue<PQnode2> pqueue2;

std::shared_ptr<std::vector<int> > get_path(int ux, int uy, int vx, int vy) {
    fprintf(stderr, "#3 get_path() start (%d,%d)->(%d,%d)\n", ux, uy, vx, vy);
    /* A* 寻路，评估权重是 已经走的距离+2*曼哈顿距离
     * 返回的是一个倒装的数据，从v[-1]一个个到v[0]，表示是mov中的行动，从u走到v
     * */
    int dis[201][201], vis[201][201];
    for (int i = 0; i < 200; ++i) {
        for (int j = 0; j < 200; ++j) {
            dis[i][j] = inf_dist;
            vis[i][j] = 0;
        }
    }

    // 这里的最短路是从v（目的地）开始到u的最短路
    // 回头从u开始通过爬梯子走到v，然后一个一个push，最后最后再reverse一下
//    dis[vx][vy] = 0;
    pqueue2.push(PQnode2(vx, vy, 0, 0));
    int tx, ty;
    while (!pqueue2.empty()) {
        PQnode2 p = pqueue2.top();
        pqueue2.pop();
//        fprintf(stderr, "PQ2 loop: (%d, %d): %d\n", p.x, p.y, p.dr);
        if (vis[p.x][p.y]) continue;
        vis[p.x][p.y] = 1;
        dis[p.x][p.y] = p.dr;

        if (p.x == ux && p.y == uy) {
//            fprintf(stderr, "Dijk Early Stop!!\n");
            break;
        }

        int rand_base = rand() % 4;
        for (int i, j = 0; j < 4; ++j) {
            i = (rand_base + j) % 4;
            tx = p.x + mov_x[i];
            ty = p.y + mov_y[i];

            if (in_mat(tx, ty) && is_land(tx, ty)) {
                if (p.dr + 1 < dis[tx][ty]) {
                    pqueue2.push(PQnode2(
                            tx, ty,
                            p.dr + 2 * (abs(tx-ux) + abs(ty-uy)), // 加入曼哈顿距离，A star 思想
                            p.dr + 1));
                }
            }
        }
    }

//    while (!pqueue2.empty()) pqueue2.pop();
    pqueue2 = std::priority_queue<PQnode2>();

#ifdef DEBUG_FLAG
    fprintf(stderr, "#3 get_path() final dist: %d\n", dis[ux][uy]);
//    for (int i = 0; i < 200; ++i) {
//        for (int j = 0; j < 200; ++j) {
//            int x = dis[i][j];
//            if (x == inf_dist) x = -1;
//            fprintf(stderr, "%4d", x);
//        }
//        fprintf(stderr, "\n");
//    }
    fflush(stderr);
#endif

    // 从u一路下降走到v
    std::shared_ptr<std::vector<int> > ret = std::make_shared<std::vector<int> >();
    while (!(ux == vx && uy == vy)) {
        for (int i = 0; i < 4; ++i) {
            tx = ux + mov_x[i];
            ty = uy + mov_y[i];
            if (dis[tx][ty] == dis[ux][uy]-1) {
                ret->push_back(i);
                ux = tx;
                uy = ty;
                break;
            }
        }
    }
    std::reverse(ret->begin(), ret->end());
#ifdef DEBUG_FLAG
    fprintf(stderr, "#2 Generate path, len: %d\n", (int)(ret->size()));
//    for (int i = 0; i < ret->size(); ++i) {
//        fprintf(stderr, "_%d", (*ret)[i]);
//    }
//    fprintf(stderr, "\n");


//    tx = uux, ty = uuy;
//    fprintf(stderr, "#2 Eval path, from (%d,%d)->", tx, ty);
//    for (int i = (int)(ret->size()) - 1; i >= 0; --i) {
//        tx += mov_x[(*ret)[i]];
//        ty += mov_y[(*ret)[i]];
//    }
//    fprintf(stderr, "(%d,%d)\n", tx, ty);
#endif
    return ret; // 从后往前，可以从u走到v
}

void Calc() {
#ifdef DEBUG_FLAG
//    fprintf(stderr, "#2 Calc start\n");
#endif
    static robot_enum flag = NOTHING;
    static std::shared_ptr<std::vector<int> > path;
    if (flag == NOTHING) {
        /* 暂时先只动 #0 号机器人 */
//        int min_dist = 40 * inf_dist;
//        int dist_id = -1;
//        int tmp;
//
//        int rx = robot[0].x;
//        int ry = robot[0].y;

//        for (int i = 0; i < objects.size(); ++i) {
//            tmp = dist_robot_obj_fake(rx, ry, objects[i].x, objects[i].y);
//            if (tmp < min_dist) {
//                min_dist = tmp;
//                dist_id = i;
//            }
//        }

//        path = get_path(robot[0].x, robot[0].y, objects[2].x, objects[2].y);
        int obj_x = (objects.end()-1)->x;
        int obj_y = (objects.end()-1)->y;
        robot[0].calc_path_to_obj(obj_x, obj_y);
        robot[0].status = TO_OBJ;
        flag = TO_OBJ;
    }
    if (flag == TO_OBJ) {
//        if (!path->empty()) {
//            printf("move 0 %d\n", *(path->end()-1));
//            robot[0].x += mov_x[*(path->end()-1)];
//            robot[0].y += mov_y[*(path->end()-1)];
//            fprintf(stderr, "### MOVINT TO OBJ ###\n");
//            path->pop_back();
//        }
//        else {
//            printf("get 0\n");
//            fprintf(stderr, "### PICK UP ###\n");
//            for (int i = 0; i < berth_num; ++i) {
//                fprintf(stderr, "# TO berth %d: %d\n", i, mat[robot[0].x][robot[0].y].dist[i]);
//            }
//            path = get_path(robot[0].x, robot[0].y, berth[1].x + 2, berth[1].y + 2);
//            flag = TO_SHIP;
//        }
        robot[0].calc_want();
        robot[0].move();
        if (robot[0].status == TO_SHIP) flag = TO_SHIP;
    }
    if (flag == TO_SHIP) {
//        if (!path->empty()) {
//            printf("move 0 %d\n", *(path->end()-1));
//            fprintf(stderr, "### MOVINT TO SHIP ###\n");
//            path->pop_back();
//        }
//        else {
//            printf("pull 0\n");
//            fprintf(stderr, "### PUT DOWN ###\n");
//            flag = RECOVERY;
//        }
        robot[0].calc_want();
        robot[0].move();
        if (robot[0].status == NOTHING) flag = NOTHING;
    }
    fflush(stderr);
}


int main() {
    fprintf(stderr, "#0 Program start");
    Init();
    int frame_id;
    for (int frame = 0; frame < 15000; frame++) {
        if (frame == 0) printf("ship 0 1\n");
        if (frame == 10000) printf("go 0\n");
        Input(frame_id);
        remove_outdated_obj(frame_id);
//        Calc();
        for (int i = 0; i < 10; ++i) robot[i].calc_want();
        for (int i = 0; i < 10; ++i) robot[i].move();
        fprintf(stdout, "OK\n");
        fflush(stdout);
    }
    return 0;
}