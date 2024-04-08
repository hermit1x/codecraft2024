//
// Created by 华华 on 2024/4/4.
//

#ifndef CODECRAFT2024_ICT_SHIP_H
#define CODECRAFT2024_ICT_SHIP_H

#define DEBUG_SHIP

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
enum ship_cmds { SHIP_CMD_DEPT, SHIP_CMD_BERTH, SHIP_CMD_ROT0, SHIP_CMD_ROT1, SHIP_CMD_SHIP, SHIP_CMD_NOTHING };
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
             {0, 1}, {-1, -1}, {-2, -1}},
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


int ship_capacity;

class Ship {
    int id, frame;
    Pos p;
    int ship_dir;
    int loads;
    ship_status status;
    bool recovering;
    int dest_berth;

    int dis[201][201];
    int vis[201][201];
public:
    Ship(int x, int y) : p(Pos(x, y)), status(SHIP_AWAIT), dest_berth(-1), frame(1), ship_dir(0) {};

    void sync(int _frame, int _id, int _loads, Pos _p, int _dir, int _status) {
        frame++;
        fprintf(stderr, "#SHIP:%d SYNC status code:%d\n", id, _status);
        if (_frame != frame || _id != id || _loads != loads || _p != p || _dir != ship_dir) {
            fprintf(stderr, "#SHIP:%d SYNC FAILED\n", id);
            fprintf(stderr, " ------- rec -|- get -\n");
            fprintf(stderr, " frame: %5d | %5d\n", frame, _frame);
            fprintf(stderr, " id:    %5d | %5d\n", id, _id);
            fprintf(stderr, " loads: %5d | %5d\n", loads, _loads);
            fprintf(stderr, " p:     %5d | %5d\n", p.x, _p.x);
            fprintf(stderr, "        %5d | %5d\n", p.y, _p.y);
            fprintf(stderr, " dir:   %5d | %5d\n", ship_dir, _dir);
        }
        frame = _frame;
        id = _id;
        loads = _loads;
        p = _p;
        ship_dir = _dir;
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
        fprintf(stderr, "ship status: %d\n", status);
    }

    void dept() {
        printf("dept %d\n", id);

        fprintf(stderr, "#SHIP:%d CMD: dept\n", id);

        if (status == SHIP_IN_BERTH) {
            berths[dest_berth].occupy = 0;
        }
        status = SHIP_AWAIT;
    }

    void berth() {
        printf("berth %d\n", id);
        fprintf(stderr, "#SHIP:%d CMD: berth (%d,%d)\n", id, p.x, p.y);

        berths[dest_berth].occupy = 1;
        status = SHIP_IN_BERTH;
    }

    Pos calc_next_p(ship_cmds cmd) {
        if (cmd != SHIP_CMD_ROT0 && cmd != SHIP_CMD_ROT1) return Pos(-1, -1);
        if (cmd == SHIP_CMD_ROT0) {
            return p + dir[ship_dir] + dir[ship_dir];
        }
        if (cmd == SHIP_CMD_ROT1) {
            return p + dir[ship_dir] - dir[rot_next[ship_dir][1]];
        }
    }

    void rot0() {
        /* o x x    . x o
         * x x x -> . x x
         * . . .    . x x
         * 核心点朝前进方向走两格
         */
        printf("rot %d 0\n", id);
        fprintf(stderr, "#SHIP:%d ROT0 dir: %d->%d", id, ship_dir, rot_next[ship_dir][0]);
        p = calc_next_p(SHIP_CMD_ROT0);
        ship_dir = rot_next[ship_dir][0];
    }

    void rot1() {
        /* . . .    . x x
         * o x x -> . x x
         * x x x    . o x
         * 核心点走一格，换新方向，退一格
         */
        printf("rot %d 1\n", id);
        fprintf(stderr, "#SHIP:%d ROT1 dir: %d->%d", id, ship_dir, rot_next[ship_dir][1]);
        p = calc_next_p(SHIP_CMD_ROT1);
        ship_dir = rot_next[ship_dir][1];
    }

    void ship() {
        printf("ship %d\n", id);
        fprintf(stderr, "#SHIP:%d CMD: ship (%d,%d)->(%d,%d)\n", id, p.x, p.y, p.x + dir[ship_dir].x, p.y + dir[ship_dir].y);
        p = p + dir[ship_dir];
    }

    struct ShipActNode {
        ship_cmds act;
        int value;
        bool operator < (const ShipActNode &rhs) const {
            return value < rhs.value;
        }
    };
    std::vector<ShipActNode> want_acts;
    int assign_act_id;
    void assign_act(int x) { assign_act_id = x; }
    void think() {
        want_acts.clear();
        fprintf(stderr, "#SHIP:%d THINK BEGIN, status: %d\n", id, status);
        if (recovering) {
            want_acts.push_back({SHIP_CMD_NOTHING, 0});
            recovering = false;
            return;
        }

        if (frame >= 13000 && status != SHIP_TO_OFFLOAD) {
            if (is_berth(p)) want_acts.push_back({SHIP_CMD_DEPT, 0});
            else want_acts.push_back({SHIP_CMD_NOTHING, 0});
            status = SHIP_TO_OFFLOAD;
            dest_berth = -1;
            return;
        }

        if (status == SHIP_AWAIT) {
            dest_berth = calc_next_berth();
            calc_path_to_berth();
            status = SHIP_TO_BERHT;
        }
        if (status == SHIP_TO_BERHT) {
            if (dis[p.x][p.y] < 20 && is_anchorage(p)) {
                status = SHIP_IN_ANCHORAGE;
            }
            else {
                calc_acts();
                return;
            }
        }
        if (status == SHIP_IN_ANCHORAGE) {
            want_acts.push_back({SHIP_CMD_BERTH, 0});
            return;
        }
        if (status == SHIP_IN_BERTH) {
            // 先执行船舶指令，后进行装载
            // 装完回卸货点 dest_berth = -1;
            if (loads == ship_capacity) {
                want_acts.push_back({SHIP_CMD_DEPT, 0});
                status = SHIP_TO_OFFLOAD;
                dest_berth = -1;
                return;
            }
            fprintf(stderr, "#SHIP:%d IN BERTH, berth:%d, stock:%d, loads:%d\n", id, dest_berth, berths[dest_berth].stock, loads);
            if (berths[dest_berth].stock == 0) {
                if (rand() % 100 < 5) {
                    want_acts.push_back({SHIP_CMD_DEPT, 0});
                    return;
                }
            }
            // 不走，继续装
            want_acts.push_back({SHIP_CMD_NOTHING, 0});
            int max2load = min(berths[dest_berth].stock, ship_capacity - loads);
            max2load = min(max2load, berths[dest_berth].velocity);
            loads += max2load;
            fprintf(stderr, " load add: %d\n", max2load);
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
                calc_path_to_offload();
            }
            if (dest_berth == -2 && is_offload(p)) {
                status = SHIP_AWAIT;
                think(); // 希望这里递归不会出事？
                return;
            }
            calc_acts();
        }
    }

    void act() {
        fprintf(stderr, "#SHIP:%d ACT BEGIN, assign id %d\n", id, assign_act_id);
        for (int i = 0; i < want_acts.size(); ++i) {
            fprintf(stderr, "act %d: %d, value: %d\n", i, want_acts[i].act, want_acts[i].value);
        }
        switch (want_acts[assign_act_id].act) {
            case SHIP_CMD_DEPT:
                dept();
                break;
            case SHIP_CMD_BERTH:
                berth();
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

    bool all_legal(Pos pp, int ddir) {
        for (int i = 0; i < 6; ++i) {
            Pos ppp = pp + ship_dir_mapping[ddir][i];
            if (!is_legal_ship(ppp)) return false;
        }
//        fprintf(stderr, " ALL_LEGAL, values:\n");
//        for (int i = 0; i < 6; ++i) {
//            Pos ppp = pp + ship_dir_mapping[ddir][i];
//            fprintf(stderr, "%d: (%d,%d), %d %d\n", i, ppp.x, ppp.y, is_legal_ship(ppp), mat[ppp.x][ppp.y].sea_dist[dest_berth]);
//        }
        return true;
    }
    int calc_value(Pos pp, int ddir) {
        int act_value = 0;
        for (int i = 0; i < 6; ++i) {
            Pos ppp = pp + ship_dir_mapping[ddir][i];
            act_value += dis[ppp.x][ppp.y];
        }
        return act_value;
    }

    void calc_acts() {
        /*
         * 先看看哪个方向是梯度下降的方向，然后转
         * 如果不用转那就走
         * 不然就两边转一下，算一下值，sort
         * 只会产生 SHIP，ROT0，ROT1的指令
         */
        // ship
        Pos next_p = p + dir[ship_dir];
        int next_dir = ship_dir;
        if (all_legal(next_p, ship_dir)) {
            if (dis[next_p.x][next_p.y] < dis[p.x][p.y]) {
                // 多给一点，不然打转有时候更有优势
                want_acts.push_back({SHIP_CMD_SHIP, calc_value(next_p, ship_dir) - 5});
            }
            else {
                want_acts.push_back({SHIP_CMD_SHIP, calc_value(next_p, ship_dir)});
            }
        }
        // rot0
        next_p = calc_next_p(SHIP_CMD_ROT0);
        next_dir = rot_next[ship_dir][0];
        if (all_legal(next_p, next_dir)) {
            want_acts.push_back({SHIP_CMD_ROT0, calc_value(next_p, next_dir)});
        }
        // rot1
        next_p = calc_next_p(SHIP_CMD_ROT1);
        next_dir = rot_next[ship_dir][1];
        if (all_legal(next_p, next_dir)) {
            want_acts.push_back({SHIP_CMD_ROT1, calc_value(next_p, next_dir)});
        }

        if (want_acts.empty()) {
            fprintf(stderr, "#SHIP:%d CALC_ACT ERROR\n", id);
            want_acts.push_back({SHIP_CMD_NOTHING, 0});
        }
        else {
            std::sort(want_acts.begin(), want_acts.end());
        }
#ifdef DEBUG_SHIP
        fprintf(stderr, "#SHIP:%d DEBUG CALC_ACT\n", id);
        for (auto act : want_acts) {
            switch (act.act) {
                case SHIP_CMD_DEPT:
                    fprintf(stderr, "cmd: dept error\n");
                    break;
                case SHIP_CMD_BERTH:
                    fprintf(stderr, "cmd: berth error\n");
                    break;
                case SHIP_CMD_ROT0:
                    fprintf(stderr, "cmd: rot0, value: %d\n", act.value);
                    break;
                case SHIP_CMD_ROT1:
                    fprintf(stderr, "cmd: rot1, value: %d\n", act.value);
                    break;
                case SHIP_CMD_SHIP:
                    fprintf(stderr, "cmd: ship, value: %d\n", act.value);
                    break;
                case SHIP_CMD_NOTHING:
                    fprintf(stderr, "cmd: nothing error\n");
                    break;
            }
        }
#endif //DEBUG_SHIP
    }

    static int calc_next_berth() {
        // TODO:
        return rand() % berth_num;
    }

    void calc_path_to_berth() {
        for (int i = 0; i < 200; ++i) {
            for (int j = 0; j < 200; ++j) {
                dis[i][j] = INF;
                vis[i][j] = 0;
            }
        }

        for (auto p : berths[dest_berth].poses) {
            dis[p.x][p.y] = 0;
            shipPQ.push({p, 0});
        }

        while (!shipPQ.empty()) {
            auto cur = shipPQ.top();
            shipPQ.pop();
            if (vis[cur.p.x][cur.p.y]) {
                continue;
            }
            vis[cur.p.x][cur.p.y] = 1;
            for (int i = 0; i < 4; ++i) {
                auto nxt = cur.p + dir[i];
                if (in_mat(nxt) && !vis[nxt.x][nxt.y] && is_legal_ship(nxt)) {
                    if (is_sea_main(nxt)) {
                        if (dis[nxt.x][nxt.y] > dis[cur.p.x][cur.p.y] + 2) {
                            dis[nxt.x][nxt.y] = dis[cur.p.x][cur.p.y] + 2;
                            shipPQ.push({nxt, dis[nxt.x][nxt.y]});
                        }
                    }
                    else {
                        if (dis[nxt.x][nxt.y] > dis[cur.p.x][cur.p.y] + 1) {
                            dis[nxt.x][nxt.y] = dis[cur.p.x][cur.p.y] + 1;
                            shipPQ.push({nxt, dis[nxt.x][nxt.y]});
                        }
                    }
                }
            }
        }
    }

    void calc_path_to_offload() {
        for (int i = 0; i < 200; ++i) {
            for (int j = 0; j < 200; ++j) {
                dis[i][j] = mat[i][j].dis2offload;
            }
        }
    }
};
std::vector<Ship> ships;

bool conflict_error_ship = false;
void handle_conflict_ship() {
    // TODO:
    for (int i = 0; i < ship_num; ++i) {
        ships[i].assign_act(0);
    }
}

void update_ship(int frame_id) {
    int B;
    scanf("%d", &B);
    fprintf(stderr, "ship_num: %d\n", B);
    int id, loads, sx, sy, sdir, sstatus;
    for (int i = 0; i < B; ++i) {
        scanf("%d%d%d%d%d%d", &id, &loads, &sx, &sy, &sdir, &sstatus);
        fprintf(stderr, "#SHIP:%d load:%d p:(%d,%d), dir:%d, status:%d\n", id, loads, sx, sy, sdir, sstatus);
        ships[id].sync(frame_id, id, loads, {sx, sy}, sdir, sstatus);
    }
}

void test_buy_ship() {
    for (int i = 0; i < 200; ++i) {
        for (int j = 0; j < 200; ++j) {
            if (is_ship_buy({i, j})) {
                printf("lboat %d %d\n", i, j);
                ships.push_back(Ship(i, j));
                ship_num = 1;
                return;
            }
        }
    }
}

#endif //CODECRAFT2024_ICT_SHIP_H
