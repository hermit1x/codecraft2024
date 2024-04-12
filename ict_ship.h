//
// Created by 华华 on 2024/4/4.
//

#ifndef CODECRAFT2024_ICT_SHIP_H
#define CODECRAFT2024_ICT_SHIP_H

//#define DEBUG_SHIP

#include "common.h"
#include "ict_berth.h"

/*
 * 船的状态：
 * 1. 等待指派运输任务 <- (刚出生，到卸货点）
 * 2. 前往泊位途中 <- (1. 卸货点到泊位, 4. 换泊位)
 * 3. 准备停靠泊位 <- (2+到达范围内)
 * 4. 在泊位上 <- (3)
 * 5. 回卸货点 <- (4. 装满 or 到时)
 * 恢复状态用一个单独的flag来存
 */
enum ship_status { SHIP_AWAIT, SHIP_TO_BERHT, SHIP_IN_ANCHORAGE, SHIP_IN_BERTH, SHIP_TO_OFFLOAD };
char status_strs[][20] = {"AWAIT", "TO_BERTH", "IN_ANCHORAGE", "IN_BERTH", "TO_OFFLOAD"};
enum ship_cmds { SHIP_CMD_DEPT, SHIP_CMD_BERTH, SHIP_CMD_ROT0, SHIP_CMD_ROT1, SHIP_CMD_SHIP, SHIP_CMD_NOTHING };
char cmd_strs[][20] = {"DEPT", "BERTH", "ROT0", "ROT1", "SHIP", "NOTHING"};
/*
 * 船的指令：
 * 1. 离开
 * 2. 停靠
 * 3. 顺时针旋转
 * 4. 逆时针旋转
 * 5. 向前一步
 * 6. 啥都不做
 */




using shipPQnode = berthPQnode;
std::priority_queue<shipPQnode> shipPQ;


// 右左上下
Mov ship_dir_mapping[4][6] = {
        /*
         * 0: 右
         * [0]1 2
         *  3 4 5
         */
        {{0, 0}, {0, 1}, {0, 2},
             {1, 0}, {1, 1}, {1, 2}},
        /*
         * 1: 左
         * 5 4 3
         * 2 1[0]
         */
        {{0, 0}, {0, -1}, {0, -2},
             {-1, 0}, {-1, -1}, {-1, -2}},
        /*
         * 2: 上
         *  2 5
         *  1 4
         * [0]3
         */
        {{0, 0}, {-1, 0}, {-2, 0},
             {0, 1}, {-1, 1}, {-2, 1}},
        /*
         * 3: 下
         * 3[0]
         * 4 1
         * 5 2
         */
        {{0, 0}, {1, 0}, {2, 0},
             {0, -1}, {1, -1}, {2, -1}}
};

int rot_next[4][2] = {{3, 2} // 右to下、上
                      , {2, 3} // 左to上、下
                      , {0, 1} // 上to右、左
                      , {1, 0}}; // 下to左、右
inline int rot_from(int cur_dir, int rot) {
    // 通过什么rot转到现在的dir
    return rot_next[cur_dir][rot^1];
}

/*
* 根据 x,y,dir做全图的最短路预处理
* 每个点有出边 rot0, rot1, ship
*
* 对每个berth，每个送货点维护一个200x200x4的图
*/

struct PosDir {
    Pos pos;
    int dir;

    PosDir() : pos(Pos(-1, -1)), dir(-1) {}
    PosDir(Pos _pos, int _dir) : pos(_pos), dir(_dir) {}

    bool is_legal() const {
        for (int i = 0; i < 6; ++i) {
            Pos ppp = pos + ship_dir_mapping[dir][i];
            if (!is_legal_ship(ppp)) return false;
        }
        return true;
    }

    bool has_main() const {
        for (int i = 0; i < 6; ++i) {
            Pos ppp = pos + ship_dir_mapping[dir][i];
            if (is_sea_main(ppp)) return true;
        }
        return false;
    }

    PosDir next_rot0() const {
        int n_dir = rot_next[dir][0];
        return PosDir(pos + mov[dir] + mov[dir], n_dir);
    }
    PosDir next_rot1() const {
        int n_dir = rot_next[dir][1];
        return PosDir(pos + mov[dir] - mov[n_dir], n_dir);
    }
    PosDir next_ship() const {
        return PosDir(pos + mov[dir], dir);
    }

    PosDir pre_rot0() {
        int p_dir = rot_from(dir, 0);
        return PosDir(pos - mov[p_dir] - mov[p_dir], p_dir);
    }
    PosDir pre_rot1() const {
        int p_dir = rot_from(dir, 1);
        return PosDir(pos + mov[dir] - mov[p_dir], p_dir);
    }
    PosDir pre_ship() const {
        return PosDir(pos - mov[dir], dir);
    }

    bool operator == (const PosDir &rhs) const {
        return pos == rhs.pos && dir == rhs.dir;
    }
};

struct PosDirValue {
    PosDir pd;
    int value;
    bool operator < (const PosDirValue &rhs) const {
        return value > rhs.value;
    }
};
std::priority_queue<PosDirValue> pdvPQ;

class BerthMap {
public:
    int dis[200][200][4];

    BerthMap(std::vector<Pos> &poses) {
        int vis[200][200][4];
        for (int i = 0; i < 200; ++i) {
            for (int j = 0; j < 200; ++j) {
                for (int k = 0; k < 4; ++k) {
                    dis[i][j][k] = INF;
                    vis[i][j][k] = 0;
                }
            }
        }

        for (auto p : poses) {
//            fprintf(stderr, "(%d, %d) ", p.x, p.y);
            for (int k = 0; k < 4; ++k) {
                if (PosDir(p, k).is_legal()) {
                    dis[p.x][p.y][k] = 0;
                    pdvPQ.push({PosDir(p, k), 0});
                }
            }
        }
//        fprintf(stderr, "\n");
        // 倒着做预处理

        while (!pdvPQ.empty()) {
            auto cur = pdvPQ.top();
            pdvPQ.pop();
            if (vis[cur.pd.pos.x][cur.pd.pos.y][cur.pd.dir]) {
                continue;
            }
            vis[cur.pd.pos.x][cur.pd.pos.y][cur.pd.dir] = 1;

            PosDir pre;
            int step = cur.pd.has_main() ? 2 : 1; // 如果有主航道，则到当前位置上后得歇一帧，即速度=一半
            // rot0
            pre = cur.pd.pre_rot0();
            if (pre.is_legal()) {
                if (dis[pre.pos.x][pre.pos.y][pre.dir] > dis[cur.pd.pos.x][cur.pd.pos.y][cur.pd.dir] + step) {
                    dis[pre.pos.x][pre.pos.y][pre.dir] = dis[cur.pd.pos.x][cur.pd.pos.y][cur.pd.dir] + step;
                    pdvPQ.push({pre, dis[pre.pos.x][pre.pos.y][pre.dir]});
                }
            }
            // rot1
            pre = cur.pd.pre_rot1();
            if (pre.is_legal()) {
                if (dis[pre.pos.x][pre.pos.y][pre.dir] > dis[cur.pd.pos.x][cur.pd.pos.y][cur.pd.dir] + step) {
                    dis[pre.pos.x][pre.pos.y][pre.dir] = dis[cur.pd.pos.x][cur.pd.pos.y][cur.pd.dir] + step;
                    pdvPQ.push({pre, dis[pre.pos.x][pre.pos.y][pre.dir]});
                }
            }
            // ship
            pre = cur.pd.pre_ship();
            if (pre.is_legal()) {
                if (dis[pre.pos.x][pre.pos.y][pre.dir] > dis[cur.pd.pos.x][cur.pd.pos.y][cur.pd.dir] + step) {
                    dis[pre.pos.x][pre.pos.y][pre.dir] = dis[cur.pd.pos.x][cur.pd.pos.y][cur.pd.dir] + step;
                    pdvPQ.push({pre, dis[pre.pos.x][pre.pos.y][pre.dir]});
                }
            }
        }
//        fprintf(stderr, "berthmap debug\n");
//        for (int k = 0; k < 4; ++k) {
//            for (int i = 0; i < 200; ++i) {
//                for (int j = 0; j < 200; ++j) {
//                    fprintf(stderr, "%4d", dis[i][j][k] == INF ? 0 : dis[i][j][k]);
//                }
//                fprintf(stderr, "\n");
//            }
//            fprintf(stderr, "\n\n\n");
//        }
    }

    int getdis(PosDir pd) const {
        return dis[pd.pos.x][pd.pos.y][pd.dir];
    }
};
std::vector<BerthMap> berthmaps, offloadmaps;

void init_sea_map() {
//    fprintf(stderr, "[INIT] SEAMAP\n");
    for (int i = 0; i < berth_num; ++i) {
//        fprintf(stderr, "[INIT] [SEAMAP] berth %d\n", i);
        berthmaps.emplace_back(berths[i].poses);
    }
    for (int i = 0; i < offload_num; ++i) {
//        fprintf(stderr, "[INIT] [SEAMAP] offload %d\n", i);
        offloadmaps.emplace_back(offloads[i].poses);
    }
//    fprintf(stderr, "[INIT] SEAMAP INIT FINISH\n");
}

int ship_capacity;

class Ship {
    int id, frame;
    int loads;
    ship_status status;
    bool recovering;
    int dest_berth, dest_offloads;

    BerthMap *seamap;
public:
    Ship(int x, int y) : pd(Pos(x, y), 0), status(SHIP_AWAIT), dest_berth(-1), dest_offloads(-1), frame(1) {};

    void sync(int _frame, int _id, int _loads, Pos _p, int _dir, int _status) {
        frame++;
#ifdef DEBUG_SHIP
        fprintf(stderr, "[%5d] ship:%d SYNC status code:%d\n", frame, id, _status);
        if (_frame != frame || _id != id || _loads != loads || _p != pd.pos || _dir != pd.dir) {
            fprintf(stderr, "#SHIP:%d SYNC FAILED\n", id);
            fprintf(stderr, " ------- rec -|- get -\n");
            fprintf(stderr, " frame: %5d | %5d\n", frame, _frame);
            fprintf(stderr, " id:    %5d | %5d\n", id, _id);
            fprintf(stderr, " loads: %5d | %5d\n", loads, _loads);
            fprintf(stderr, " p:     %5d | %5d\n", pd.pos.x, _p.x);
            fprintf(stderr, "        %5d | %5d\n", pd.pos.y, _p.y);
            fprintf(stderr, " dir:   %5d | %5d\n", pd.dir, _dir);
        }
#endif
        frame = _frame;
        id = _id;
        loads = _loads;
        pd.pos = _p;
        pd.dir = _dir;
        if (_status == 0 && status == SHIP_IN_BERTH) {
            // 除了 inberth 都是 0 正常行驶状态
            fprintf(stderr, "#SHIP:%d SYNC_ERROR type:1\n", id);
        }
        if (_status == 2 && status != SHIP_IN_BERTH) {
            fprintf(stderr, "#SHIP:%d SYNC_ERROR type:2\n", id);
        }
        if (_status == 1) {
            // 恢复状态
            recovering = true;
        }
        else {
            recovering = false;
        }
#ifdef DEBUG_SHIP
        fprintf(stderr, "[%5d] [ship:%d] SYNC status: %s\n", frame, id, status_strs[status]);
#endif
    }

    void dept() {
        printf("dept %d\n", id);
//        fprintf(stderr, "- [ship:%d] cmd:DEPT\n", id);
        berths[dest_berth].occupy = 0;
    }

    void berth() {
        printf("berth %d\n", id);
//        fprintf(stderr, "- [ship:%d] cmd:BERTH (%d,%d)\n", id, pd.pos.x, pd.pos.y);
        berths[dest_berth].occupy = 1;
    }

    void rot0() {
        /* o x x    . x o
         * x x x -> . x x
         * . . .    . x x
         * 核心点朝前进方向走两格
         */
        printf("rot %d 0\n", id);
//        fprintf(stderr, "- [ship:%d] cmd:ROT0 dir: %d->%d", id, pd.dir, rot_next[pd.dir][0]);
        pd = pd.next_rot0();
    }

    void rot1() {
        /* . . .    . x x
         * o x x -> . x x
         * x x x    . o x
         * 核心点走一格，换新方向，退一格
         */
        printf("rot %d 1\n", id);
//        fprintf(stderr, "- [ship:%d] cmd:ROT1 dir: %d->%d", id, pd.dir, rot_next[pd.dir][1]);
        pd = pd.next_rot1();
    }

    void ship() {
        printf("ship %d\n", id);
//        fprintf(stderr, "- [ship:%d] cmd:SHIP (%d,%d)", id, pd.pos.x, pd.pos.y);
        pd = pd.next_ship();
//        fprintf(stderr, "->(%d,%d)\n", pd.pos.x, pd.pos.y);
    }

    struct ShipActNode {
        ship_cmds act;
        PosDir pd;
        int value;
        bool operator < (const ShipActNode &rhs) const {
            return value < rhs.value;
        }
    };
    std::vector<ShipActNode> want_acts;
    int assign_act_id;
    void assign_act(int x) { assign_act_id = x; }
    void assign_dept() {
        want_acts.clear();
        want_acts.push_back({SHIP_CMD_DEPT, PosDir(), 0});
        assign_act_id = 0;
    }


    void think() {
        want_acts.clear();

        if (recovering) {
            want_acts.push_back({SHIP_CMD_NOTHING,PosDir(),0});
            recovering = false;
            return;
        }

        // TODO: 临终测量还得改
        if (frame >= 14500 && status != SHIP_TO_OFFLOAD) {
            if (is_berth(pd.pos)) want_acts.push_back({SHIP_CMD_DEPT, PosDir(), 0});
            else want_acts.push_back({SHIP_CMD_NOTHING, PosDir(), 0});
            status = SHIP_TO_OFFLOAD;
            dest_berth = -1;
            return;
        }
#ifdef DEBUG_SHIP
        fprintf(stderr, "[%5d] [ship:%d] status: %s, to_berth:%d, to_offlaod:%d\n", frame, id, status_strs[status], dest_berth, dest_offloads);
#endif
        if (status == SHIP_AWAIT) {
            dest_berth = calc_next_berth();
            berths[dest_berth].occupy = 1;
            calc_path_to_berth();
            status = SHIP_TO_BERHT;
        }
        if (status == SHIP_TO_BERHT) {
#ifdef DEBUG_SHIP
            fprintf(stderr, "[%5d] ship:%d TO_BERTH, pos: (%d,%d), berth:%d, dis:%d, is_anchorage:%d\n", frame, id, pd.pos.x, pd.pos.y, dest_berth, seamap->getdis(pd), is_anchorage(pd.pos));
#endif
            if (seamap->getdis(pd) < 20 && is_anchorage(pd.pos)) {
                status = SHIP_IN_ANCHORAGE;
            }
            else {
                calc_acts();
                return;
            }
        }
        if (status == SHIP_IN_ANCHORAGE) {
            want_acts.push_back({SHIP_CMD_BERTH, PosDir(), 0});
            return;
        }
        if (status == SHIP_IN_BERTH) {
            // 先执行船舶指令，后进行装载F
            // 装完回卸货点 dest_berth = -1;
            if (loads == ship_capacity) {
                want_acts.push_back({SHIP_CMD_DEPT, PosDir(), 0});
                status = SHIP_TO_OFFLOAD;
                dest_berth = -1;
#ifdef DEBUG_SHIP
                fprintf(stderr, "[%5d] ship:%d IN_BERTH, berth:%d, load_full\n", frame, id, dest_berth);
#endif
                return;
            }
            fprintf(stderr, "[%5d] ship:%d IN_BERTH, berth:%d, stock:%d, loads:%d\n", frame, id, dest_berth, berths[dest_berth].stock, loads);
            if (berths[dest_berth].stock == 0) {
                if (rand() % 100 < 20) {
                    want_acts.push_back({SHIP_CMD_DEPT, PosDir(), 0});
                    status = SHIP_AWAIT;
#ifdef DEBUG_SHIP
                    fprintf(stderr, "[%5d] ship:%d IN_BERTH, berth:%d, empty DEPT\n", frame, id, dest_berth);
#endif
                    return;
                }
            }
            // 不走，继续装
            want_acts.push_back({SHIP_CMD_NOTHING, PosDir(), 0});
            int max2load = min(berths[dest_berth].stock, ship_capacity - loads);
            max2load = min(max2load, berths[dest_berth].velocity);
            loads += max2load;
            fprintf(stderr, "- load add:%d\n", max2load);

            if (berths[dest_berth].stocks.size() != berths[dest_berth].stock) {
                fprintf(stderr, "### STOCK_NOT_MATCH ERROR ###\n");
            }

            for (int i = 0; i < max2load; ++i) {
                berths[dest_berth].remain_value -= berths[dest_berth].stocks.front();
                berths[dest_berth].stocks.pop();
            }
            berths[dest_berth].stock -= max2load;
            return;
        }
        if (status == SHIP_TO_OFFLOAD) {
            if (dest_berth == -1) {
                dest_berth = -2;
                dest_offloads = calc_best_offload();
#ifdef DEBUG_SHIP
                fprintf(stderr, "[%5d] ship:%d TO_OFFLOAD, offload:%d\n", frame, id, dest_offloads);
#endif
                calc_path_to_offload();
            }
            if (dest_berth == -2 && is_offload(pd.pos)) {
                status = SHIP_AWAIT;
                think(); // 希望这里递归不会出事？
                return;
            }
            calc_acts();
        }
    }

    void act() {
#ifdef DEBUG_SHIP
        fprintf(stderr, "[%5d] ship:%d FINAL_ACT: %s\n", frame, id, cmd_strs[want_acts[assign_act_id].act]);
#endif
        switch (want_acts[assign_act_id].act) {
            case SHIP_CMD_DEPT:
                dept();
                break;
            case SHIP_CMD_BERTH:
                berth();
                status = SHIP_IN_BERTH;
                break;
            case SHIP_CMD_ROT0:
                rot0();
                break;
            case SHIP_CMD_ROT1:
                rot1();
                break;
            case SHIP_CMD_SHIP:
                ship();
                break;
            case SHIP_CMD_NOTHING:
                break;
        }
    }

    void calc_acts() {
        PosDir nxt;
        int cur_dis = seamap->getdis(pd);
        nxt = pd.next_rot0();
        if (nxt.is_legal() && seamap->getdis(nxt) <= cur_dis) want_acts.push_back({SHIP_CMD_ROT0, nxt, seamap->getdis(nxt)});
        nxt = pd.next_rot1();
        if (nxt.is_legal() && seamap->getdis(nxt) <= cur_dis) want_acts.push_back({SHIP_CMD_ROT1, nxt, seamap->getdis(nxt)});
        nxt = pd.next_ship();
        if (nxt.is_legal() && seamap->getdis(nxt) <= cur_dis) want_acts.push_back({SHIP_CMD_SHIP, nxt, seamap->getdis(nxt)});
        want_acts.push_back({SHIP_CMD_NOTHING, pd, cur_dis});
        std::sort(want_acts.begin(), want_acts.end());
#ifdef DEBUG_SHIP
        for (auto &x : want_acts) {
            fprintf(stderr, "[     ] ship:%d ACT: %s, dis:%d\n", id, cmd_strs[x.act], x.value);
        }
#endif
    }

    int calc_next_berth() {
        // TODO: 继续优化这里的算法
        double mx_value = 0, mx_berth = rand() % berth_num;
        for (int i = 0; i < berth_num; ++i) {
            if (i == dest_berth) continue;
            if (berths[i].occupy || berths[i].closed) continue;
            if (mx_value < 1.0 * berths[i].stock / berthmaps[i].getdis(pd)) {
                mx_value = 1.0 * berths[i].stock / berthmaps[i].getdis(pd);
                mx_berth = i;
            }
        }
        return mx_berth;
    }

    int calc_best_offload() {
        int mx_value = 0, dest_offload = rand() % offload_num;
        for (int i = 0; i < offload_num; ++i) {
            if (mx_value < offloadmaps[i].getdis(pd)) {
                mx_value = offloadmaps[i].getdis(pd);
                dest_offload = i;
            }
        }
        return dest_offload;
    }

    void calc_path_to_berth() {
        seamap = &berthmaps[dest_berth];
    }

    void calc_path_to_offload() {
        seamap = &offloadmaps[dest_offloads];
    }

    PosDir pd;
};
std::vector<Ship> ships;

int ship_conflict_mat[200][200];

bool ship_conflict_test(const PosDir &pd) {
//    fprintf(stderr, "[test conflict] pd: (%d,%d)\n", pd.pos.x, pd.pos.y);
    for (int i = 0; i < 6; ++i) {
        Pos p = pd.pos + ship_dir_mapping[pd.dir][i];
        if (is_sea_main(p)) continue;
//        fprintf(stderr, "[test conflict] (%d,%d)\n", p.x, p.y);
        if (ship_conflict_mat[p.x][p.y] != -1) return false;
    }
    return true;
}

int ship_conflict_get(const PosDir &pd) {
    int conflict_with = -1;
//    fprintf(stderr, "[get conflict] pd: (%d,%d)\n", pd.pos.x, pd.pos.y);
    for (int i = 0; i < 6; ++i) {
        Pos p = pd.pos + ship_dir_mapping[pd.dir][i];
        if (is_sea_main(p)) continue;
        if (ship_conflict_mat[p.x][p.y] != -1) {
//            fprintf(stderr, "[get conflict] p:  (%d, %d): %d\n", p.x, p.y, ship_conflict_mat[p.x][p.y]);
            if (conflict_with == -1) conflict_with = ship_conflict_mat[p.x][p.y];
            else if (conflict_with != ship_conflict_mat[p.x][p.y]) {
//                fprintf(stderr, "[get conflict] conflict with multi: %d + %d\n", conflict_with, ship_conflict_mat[p.x][p.y]);
                return -1;
            }
        }
    }
    return conflict_with;
}

void ship_conflict_erase(const PosDir &pd) {
    for (int i = 0; i < 6; ++i) {
        Pos p = pd.pos + ship_dir_mapping[pd.dir][i];
        if (is_sea_main(p)) continue;
        ship_conflict_mat[p.x][p.y] = -1;
//        fprintf(stderr, "[conflict] erase (%d,%d)\n", p.x, p.y);
    }
}

void ship_conflict_occupy(const PosDir &pd, int ship_id) {
    for (int i = 0; i < 6; ++i) {
        Pos p = pd.pos + ship_dir_mapping[pd.dir][i];
        if (is_sea_main(p)) continue;
        ship_conflict_mat[p.x][p.y] = ship_id;
//        fprintf(stderr, "[conflict] occupy (%d,%d)\n", p.x, p.y);
    }
}

void ship_conflict_dfs(int ship_id) {
    if (ship_id == ship_num) return;
//    fprintf(stderr, "[conflict] DFS ship:%d, act.size:%lu\n", ship_id, ships[ship_id].want_acts.size());
    ship_conflict_erase(ships[ship_id].pd);
    if (ships[ship_id].want_acts.size() == 1) {
//        fprintf(stderr, "[conflict] ship:%d only one act: %s\n", ship_id, cmd_strs[ships[ship_id].want_acts[0].act]);
        if (ships[ship_id].want_acts[0].pd == PosDir()) {
//            fprintf(stderr, "[conflict] ship:%d not move cmd\n", ship_id);
            ships[ship_id].assign_act(0);
            ship_conflict_dfs(ship_id + 1);
            return;
        }
//        fprintf(stderr, "[conflict] ship:%d is move act\n", ship_id);
    }
    for (int i = 0; i < ships[ship_id].want_acts.size(); ++i) {
        ships[ship_id].assign_act(i);
//        fprintf(stderr, "[conflict] act_id:%d\n", i);
//        fprintf(stderr, "[        ] act: %s\n", cmd_strs[ships[ship_id].want_acts[i].act]);
        if (ship_conflict_test(ships[ship_id].want_acts[i].pd)) {
            // 有合法的就直接走，不管后面的死活
//            fprintf(stderr, "[conflict] ship:%d act:%s legal\n", ship_id, cmd_strs[ships[ship_id].want_acts[i].act]);
            ship_conflict_occupy(ships[ship_id].want_acts[i].pd, ship_id);
            ship_conflict_dfs(ship_id + 1);
            return;
        }
        else {
            int conflict_id = ship_conflict_get(ships[ship_id].want_acts[i].pd);
            if (conflict_id == -1) continue;
//            fprintf(stderr, "[conflict] ship:%d act:%s conflict with %d\n", ship_id, cmd_strs[ships[ship_id].want_acts[i].act], conflict_id);
            if (conflict_id > ship_id) continue;
            if (ships[conflict_id].want_acts[ ships[conflict_id].assign_act_id ].act == SHIP_CMD_NOTHING) {
                // 赶走前一个NOTHING
                ships[conflict_id].assign_dept();
//                fprintf(stderr, "[conflict] ship:%d FORCE DEPT %d\n", ship_id, conflict_id);
                ship_conflict_dfs(conflict_id + 1);
                return;
            }
        }

    }
    // 如果包括 NOTHING 在内都没有合法的，那就 BERTH 重置了
    ships[ship_id].assign_dept();
    ship_conflict_dfs(ship_id + 1);
//    fprintf(stderr, "[conflict] SHIP:%d FORCE DEPT\n", ship_id);
    return;
    /*
     * 这套逻辑不一定是最佳的，但可以解决
     * 1. 两个船对顶着，都NOTHING，合法但不对
     * 2. 两个船相互让位，比如都一起rotate，合法但不对
     *
     * 可能的改进方向：
     * 统计所有合法的可能，然后选全局最优？
     */
}

void handle_conflict_ship() {
    for (int i = 0; i < ship_num; ++i) {
        ships[i].assign_act(0);
    }
    return;

    fprintf(stderr, "[conflict] start\n");
    for (int i = 0; i < 200; ++i) {
        for (int j = 0; j < 200; ++j) {
            ship_conflict_mat[i][j] = -1;
        }
    }
    fprintf(stderr, "[conflict] mat clear finish\n");
    for (int i = 0; i < ship_num; ++i) {
        ship_conflict_occupy(ships[i].pd, i);
    }
    fprintf(stderr, "[conflict] mat init finish\n");
    ship_conflict_dfs(0);
}

void update_ship(int frame_id) {
    int B;
    scanf("%d", &B);
//    fprintf(stderr, "ship_num: %d\n", B);
    int id, loads, sx, sy, sdir, sstatus;
    for (int i = 0; i < B; ++i) {
        scanf("%d%d%d%d%d%d", &id, &loads, &sx, &sy, &sdir, &sstatus);
//        fprintf(stderr, "#SHIP:%d load:%d p:(%d,%d), dir:%d, status:%d\n", id, loads, sx, sy, sdir, sstatus);
        ships[id].sync(frame_id, id, loads, {sx, sy}, sdir, sstatus);
    }
}

void test_buy_ship(int frame, int &money) {
//    fprintf(stderr, "test_buy_ship frame:%d\n", frame);
    if (ship_num >= want_ship_num) return;
    if (money < 8000) return;
    fprintf(stderr, "can_buy_ship\n");
    for (int i = 0; i < 200; ++i) {
        for (int j = 0; j < 200; ++j) {
            if (is_ship_buy({i, j})) {
                // 如果有近的可能会撞的船，skip
                bool flag = false;
                for (auto s : ships) {
                    if (dis_man(s.pd.pos, {i, j}) < 20) {
                        flag = true;
                        break;
                    }
                }
                if (flag) continue;

                printf("lboat %d %d\n", i, j);
                fprintf(stderr, "buy_ship (%d, %d)\n", i, j);
                ships.push_back(Ship(i, j));
                ship_num += 1;
                return;
            }
        }
    }
}

#endif //CODECRAFT2024_ICT_SHIP_H
