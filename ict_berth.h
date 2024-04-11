//
// Created by 华华 on 2024/4/3.
//

#ifndef CODECRAFT2024_ICT_BERTH_H
#define CODECRAFT2024_ICT_BERTH_H

#include <vector>
#include <queue>
#include "common.h"
#include "ict_map.h"
#include "dijkstra.h"


class Berth {
public:
    Berth() {}
    Berth(int id, int x, int y, int velocity) : id(id), loc({x, y}), velocity(velocity) {
        closed = false;
    }

    int id, velocity;
    Pos loc; // 船停靠的时候核心点的位置
    std::vector<Pos> poses;

    int workers, future_value, booked; // for robot use
    int occupy, stock, remain_value; // for ship use
    int estimate_value, tot_value, tot_stock;

    bool closed;
    std::queue<int> stocks;
} ;
std::vector<Berth> berths;

struct berthPQnode {
    Pos p;
    int d;

    berthPQnode() {}
    berthPQnode(Pos _x, int _d) : p(_x), d(_d) { }

    bool operator<(const berthPQnode &rhs) const {
        return d > rhs.d;
    }
};
std::priority_queue<berthPQnode> berthPQ;


void init_berth() {
    scanf("%d", &berth_num);
    int id, x, y, velocity;
    for (int i = 0; i < berth_num; ++i) {
        scanf("%d%d%d%d", &id, &x, &y, &velocity);
        if (id != i) {
            fprintf(stderr, "[ERR] berth id not match\n");
        }
        berths.emplace_back(id, x, y, velocity); // emplace_back可以原地构造对象，避免了拷贝

    /*
     * 给每一个泊位都初始化一下全图的最短路
     * 存在mat[x][y].dist[berth_id]里
     * 可以在机器人建起物品后快速判断去哪个泊位
     * 同时也用于去向泊位的过程中选择路径
     *
     * update: 只算陆地的，海的另算
     * */
        // 给每个泊位初始化距离
        int dis[201][201], vis[201][201];
        for (int k = 0; k < 201; ++k) {
            for (int j = 0; j < 201; ++j) {
                dis[k][j] = INF;
                vis[k][j] = 0;
            }
        }

        // 一个退化成bfs的dijkstra，先统计一个港口有多少土地
        berthPQ.push({ Pos(x, y), 0});
        while (!berthPQ.empty()) {
            auto cur = berthPQ.top();
            berthPQ.pop();
            if (vis[cur.p.x][cur.p.y]) continue;
            vis[cur.p.x][cur.p.y] = 1;

            berths.back().poses.push_back(cur.p);
            for (int j = 0; j < 4; ++j) {
                Pos nxt = cur.p + mov[j];
                if (in_mat(nxt) && is_berth(nxt)) {
                    berthPQ.push({nxt, 0});
                }
            }
        }

        for (auto p : berths[i].poses) {
            dis[p.x][p.y] = 0;
            vis[p.x][p.y] = 0;
            berthPQ.push({p, 0});
        }

        for (auto p : berths[i].poses) {
            dis[p.x][p.y] = 0;
            vis[p.x][p.y] = 0;
            berthPQ.push({p, 0});
        }
        while (!berthPQ.empty()) {
            auto cur = berthPQ.top();
            berthPQ.pop();
            if (vis[cur.p.x][cur.p.y]) continue;
            vis[cur.p.x][cur.p.y] = 1;
            for (int j = 0; j < 4; ++j) {
                Pos nxt = cur.p + mov[j];
//                if (in_mat(nxt) && is_land(nxt) && dis[nxt.x][nxt.y] > dis[cur.p.x][cur.p.y] + 1) {
//                    dis[nxt.x][nxt.y] = dis[cur.p.x][cur.p.y] + 1;
//                    berthPQ.push({nxt, dis[nxt.x][nxt.y]});
//                }
                if (is_legal_bot(nxt) && !vis[nxt.x][nxt.y]) {
                    if (is_land_main(nxt)) {
                        if (dis[nxt.x][nxt.y] > dis[cur.p.x][cur.p.y] + 1) {
                            dis[nxt.x][nxt.y] = dis[cur.p.x][cur.p.y] + 1;
                            berthPQ.push({nxt, dis[nxt.x][nxt.y]});
                        }
                    }
                    else {
                        if (dis[nxt.x][nxt.y] > dis[cur.p.x][cur.p.y] + 2) {
                            dis[nxt.x][nxt.y] = dis[cur.p.x][cur.p.y] + 2;
                            berthPQ.push({nxt, dis[nxt.x][nxt.y]});
                        }
                    }
                }
            }
        }

        for (int ii = 0; ii < 200; ++ii) {
            for (int jj = 0; jj < 200; ++jj) {
                mat[ii][jj].land_dist.push_back(dis[ii][jj]);
            }
        }
    }
#ifdef DEBUG_FLAG
    fprintf(stderr, "# LAND\n");
    for (int i = 0; i < 200; ++i) {
        for (int j = 0; j < 200; ++j) {
            fprintf(stderr, "%4d", mat[i][j].land_dist[0] == INF ? 0 : mat[i][j].land_dist[0]);
        }
        fprintf(stderr, "\n");
    }
#endif
}

class Offload {
public:
    std::vector<Pos> poses;
};
std::vector<Offload> offloads;

void init_offload() {
    int calced[200][200];
    for (int i = 0; i < 200; ++i) {
        for (int j = 0; j < 200; ++j) {
            calced[i][j] = 0;
        }
    }
    for (int i = 0; i < 200; ++i) {
        for (int j = 0; j < 200; ++j) {
            if (calced[i][j]) continue; // 这个集合被算过了
            if (!is_offload(Pos(i, j))) continue;
            offloads.push_back(Offload());

            // 一个退化成bfs的dijkstra，先统计一个港口有多少土地
            berthPQ.push({ Pos(i, j), 0});
            while (!berthPQ.empty()) {
                auto cur = berthPQ.top();
                berthPQ.pop();
                if (calced[cur.p.x][cur.p.y]) continue;
                calced[cur.p.x][cur.p.y] = 1;

                offloads.back().poses.push_back(cur.p);
                for (int k = 0; k < 4; ++k) {
                    Pos nxt = cur.p + mov[k];
                    if (in_mat(nxt) && is_offload(nxt)) {
                        berthPQ.push({nxt, 0});
                    }
                }
            }
        }
    }
    offload_num = (int)offloads.size();
    fprintf(stderr, "[INIT] Offload inited, num: %d\n", offload_num);
}


#endif //CODECRAFT2024_ICT_BERTH_H
