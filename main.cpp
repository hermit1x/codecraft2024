#include <iostream>
#include <algorithm>
#include <queue>
#include <vector>
#include <memory>


#define DEBUF_FLAG

enum mat_enum { LAND, OCEAN, HILL, BERTH };
enum robot_enum { NOTHING, TO_OBJ, TO_SHIP, RECOVERY };
enum ship_enum { SHIPPING, NORMAL, WAITING };

const int robot_num = 10;
const int berth_num = 10;
const int ship_num = 5;
const int inf_dist = 1e6;

const int mov_x[4] = {0, 0, -1, 1};
const int mov_y[4] = {1, -1, 0, 0};

inline bool in_mat(int x, int y) {
    return 0 <= x && x < 200 && 0 <= y && y < 200;
}

class Mat {
public:
    mat_enum type;
    int dist[berth_num];
} mat[207][207];

inline bool is_land(int x, int y) {
    return mat[x][y].type == LAND || mat[x][y].type == BERTH;
}

class Robot {
public:
    Robot() {}
    Robot(int _x, int _y) : pos_x(_x), pos_y(_y), status(NOTHING) {}

    int pos_x, pos_y, status;
    int dest_x, dest_y;
} robot[robot_num+1];
int robot_cnt = 0;

class Berth {
public:
    Berth() {}

    int id, pos_x, pos_y, transport_time, velocity;
}berth[berth_num+1];

class Ship {
public:
    Ship() {};

    ship_enum status;
    int berth_id;
    int loads;
} ship[ship_num+1];

int ship_capacity;

class Obj {
public:
    Obj() {};
    Obj(int _x, int _y, int _value) : x(_x), y(_y), value(_value), obtain(0) {};

    int x, y, value, obtain;
};
std::vector<Obj> objects;

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
    base_x = berth[berth_id].pos_x;
    base_y = berth[berth_id].pos_y;

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
                    robot[robot_cnt++] = Robot(row, col);
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
        scanf("%d%d%d%d%d", &berth[i].id, &berth[i].pos_x, &berth[i].pos_y, &berth[i].transport_time, &berth[i].velocity);
    }

    // 船舶容量
    scanf("%d", &ship_capacity);

    char okk[100];
    scanf("%s", okk);
#ifdef DEBUF_FLAG
    for (int r = 0; r < 10; ++r) {
        fprintf(stderr, "robot %d: (%d, %d)\n", r, robot[r].pos_x, robot[r].pos_y);
    }
    for (int i = 0; i < 10; ++i) {
        fprintf(stderr, "berth %d %d %d %d %d\n", berth[i].id, berth[i].pos_x, berth[i].pos_y, berth[i].transport_time, berth[i].velocity);
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

void Input() {
#ifdef DEBUF_FLAG
    fprintf(stderr, "#2 Input start\n");
    fflush(stderr);
#endif
    int frame_id, money;
    int k;
    scanf("%d%d", &frame_id, &money);
    scanf("%d", &k);

    fprintf(stderr, "#2 Input frame: %d, money: %d, k: %d\n", frame_id, money, k);
    // 新增的货物信息
    int x, y, value;
    for (int i = 0; i < k; ++i) {
        scanf("%d%d%d", &x, &y, &value);
        objects.push_back(Obj(x, y, value));
    }

    fprintf(stderr, "#2 Input finish1\n");
    // 当前的机器人信息
    int carry, status; // 还有x, y
    for (int i = 0; i < 10; ++i) {
        scanf("%d%d%d%d", &carry, &x, &y, &status);
    }

    fprintf(stderr, "#2 Input finish1\n");
    // 当前的船信息
    int dest_berth;
    for (int i = 0; i < 5; ++i) {
        scanf("%d%d", &status, &dest_berth);
    }
    fprintf(stderr, "#2 Input finish1\n");
    char okk[100];
    scanf("%s", okk);

#ifdef DEBUF_FLAG
    fprintf(stderr, "#2 Input finish\n");
    fflush(stderr);
#endif
}

int dist_robot_obj_fake(int rx, int ry, int ox, int oy) {
    /*
     * 重要函数，玄学评估机器人和物品的距离
     * 目前采用的思路是：曼哈顿距离*20 + 每个港口的条件下两个点的距离差（表示层次接近）
     * */
    int sum = 20 * (abs(rx - ox) + abs(ry - oy)); // 曼哈顿距离
    for (int i = 0; i < berth_num; ++i) {
        sum += abs( mat[rx][ry].dist[i] - mat[ox][oy].dist[i] );
    }

    return sum;
}


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
            fprintf(stderr, "Early Stop!!");
            break;
        }

        for (int i = 0; i < 4; ++i) {
            tx = p.x + mov_x[i];
            ty = p.y + mov_y[i];

            if (in_mat(tx, ty) && is_land(tx, ty)) {
                if (p.dr + 1 < dis[tx][ty]) {
                    pqueue2.push(PQnode2(
                            tx, ty,
                            p.dr,
//                            p.dr + (abs(tx-ux) + abs(ty-uy)),
//                            p.dr * 30 + dist_robot_obj_fake(tx, ty, ux, uy),
                            p.dr + 1));
                }
            }
        }
    }

    while (!pqueue2.empty()) pqueue2.pop();

#ifdef DEBUF_FLAG
    fprintf(stderr, "#DEBUG get_path() final dist: %d\n", dis[ux][uy]);
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
#ifdef DEBUF_FLAG
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
#ifdef DEBUF_FLAG
    fprintf(stderr, "#2 Calc start\n");
#endif
    static robot_enum flag = NOTHING;
    static std::shared_ptr<std::vector<int> > path;
    if (flag == NOTHING) {
        /* 暂时先只动 #0 号机器人 */
        int min_dist = 40 * inf_dist;
        int dist_id = -1;
        int tmp;

        int rx = robot[0].pos_x;
        int ry = robot[0].pos_y;

//        for (int i = 0; i < objects.size(); ++i) {
//            tmp = dist_robot_obj_fake(rx, ry, objects[i].x, objects[i].y);
//            if (tmp < min_dist) {
//                min_dist = tmp;
//                dist_id = i;
//            }
//        }

        path = get_path(robot[0].pos_x, robot[0].pos_y, objects[2].x, objects[2].y);
        flag = TO_OBJ;
    }
    if (flag == TO_OBJ) {
        if (!path->empty()) {
            printf("move 0 %d\n", *(path->end()-1));
            robot[0].pos_x += mov_x[*(path->end()-1)];
            robot[0].pos_y += mov_y[*(path->end()-1)];
            fprintf(stderr, "### MOVINT TO OBJ ###\n");
            path->pop_back();
        }
        else {
            printf("get 0\n");
            fprintf(stderr, "### PICK UP ###\n");
            for (int i = 0; i < berth_num; ++i) {
                fprintf(stderr, "# TO berth %d: %d\n", i, mat[robot[0].pos_x][robot[0].pos_y].dist[i]);
            }
            path = get_path(robot[0].pos_x, robot[0].pos_y, berth[1].pos_x + 2, berth[1].pos_y + 2);
            flag = TO_SHIP;
        }
    }
    if (flag == TO_SHIP) {
        if (!path->empty()) {
            printf("move 0 %d\n", *(path->end()-1));
            fprintf(stderr, "### MOVINT TO SHIP ###\n");
            path->pop_back();
        }
        else {
            printf("pull 0\n");
            fprintf(stderr, "### PUT DOWN ###\n");
            flag = RECOVERY;
        }
    }
    fflush(stderr);
}

void Output() {
    for (int i = 1; i < 10; ++i) {
        fprintf(stdout, "move %d %d\n", i, rand() % 4);
    }
    fprintf(stdout, "OK\n");
    fflush(stdout);
}

int main() {
    fprintf(stderr, "#0 Program start");
    Init();
    for (int frame = 0; frame < 15000; frame++) {
        if (frame == 0) printf("ship 0 1\n");
        if (frame == 5000) printf("go 0\n");
        Input();
        Calc();
        Output();
    }
    return 0;
}