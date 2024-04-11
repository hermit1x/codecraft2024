//
// Created by 华华 on 2024/4/3.
//

#ifndef CODECRAFT2024_COMMON_H
#define CODECRAFT2024_COMMON_H

const int INF = 0x3f3f3f3f;

int berth_num;
int ship_num;
int robot_num = 0;
int offload_num;

struct Pos {
    int x, y;

    Pos() : x(0), y(0) {}
    Pos(int _x, int _y): x(_x), y(_y) {}
    Pos(const Pos &rhs) : x(rhs.x), y(rhs.y) {}

    Pos operator + (const Pos &rhs) const {
        return {x + rhs.x, y + rhs.y};
    }
    Pos operator - (const Pos &rhs) const {
        return {x - rhs.x, y - rhs.y};
    }
    bool operator == (const Pos &rhs) const {
        return x == rhs.x && y == rhs.y;
    }
    bool operator != (const Pos &rhs) const {
        return x != rhs.x || y != rhs.y;
    }
};

using Mov = Pos;
// 右 左 上 下 不动
const Mov mov[5] = {{0, 1}, {0, -1}, {-1, 0}, {1, 0}, {0, 0}};

#define min(x, y) ((x) < (y) ? (x) : (y))
#define max(x, y) ((x) > (y) ? (x) : (y)

#endif //CODECRAFT2024_COMMON_H
