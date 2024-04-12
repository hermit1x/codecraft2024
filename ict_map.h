//
// Created by 华华 on 2024/4/3.
//

#ifndef CODECRAFT2024_ICT_MAP_H
#define CODECRAFT2024_ICT_MAP_H

#include <vector>
#include "common.h"

using map_type = int;

/*
 * map_type状压
 * bit :   8   |   7   |  6  |  5  |   4   |  3  |    2    |   1   |  0
 * 信息 : 交货点 | 靠泊区 | 泊位 | 买船 | 主航道 | 海洋 | 买机器人 | 主干道 | 陆地
 */

#define MAP_FLAG_LAND        0b00000001
#define MAP_FLAG_LAND_MAIN   0b00000010
#define MAP_FLAG_ROBOT_BUY   0b00000100
#define MAP_FLAG_SEA         0b00001000
#define MAP_FLAG_SEA_MAIN    0b00010000
#define MAP_FLAG_SHIP_BUY    0b00100000
#define MAP_FLAG_MULTILEVEL  0b00001001
#define MAP_FLAG_BERTH       0b01000000
#define MAP_FLAG_ANCHORAGE   0b10000000
#define MAP_FLAG_OFFLOAD    0b100000000

#define MAP_BARRIER         0b00000000
#define MAP_LAND            0b00000001
#define MAP_LAND_MAIN       0b00000011
#define MAP_ROBOT_BUY       0b00000111
#define MAP_SEA             0b00001000
#define MAP_SEA_MAIN        0b00011000
#define MAP_SHIP_BUY        0b00111000
#define MAP_MULTILEVEL      0b00001001
#define MAP_MULTILEVEL_MAIN 0b00011011
#define MAP_BERTH           0b11011011
#define MAP_ANCHORAGE       0b10011000
#define MAP_OFFLOAD        0b100011000

class Mat {
public:
    map_type type;

    bool has_obj;
    std::vector<int> land_dist;
} mat[207][207];

#define macro_dis_man(x1, y1, x2, y2) (abs(x1 - x2) + abs(y1 - y2))
#define macro_in_mat(x, y) (0 <= x && x < 200 && 0 <= y && y < 200)
#define macro_is_land(x, y) (mat[x][y].type & MAP_FLAG_LAND)
#define macro_is_land_main(x, y) (mat[x][y].type & MAP_FLAG_LAND_MAIN)
#define macro_is_robot_buy(x, y) (mat[x][y].type & MAP_FLAG_ROBOT_BUY)
#define macro_is_sea(x, y) (mat[x][y].type & MAP_FLAG_SEA)
#define macro_is_sea_main(x, y) (mat[x][y].type & MAP_FLAG_SEA_MAIN)
#define macro_is_ship_buy(x, y) (mat[x][y].type & MAP_FLAG_SHIP_BUY)
#define macro_is_berth(x, y) (mat[x][y].type & MAP_FLAG_BERTH)
#define macro_is_anchorage(x, y) (mat[x][y].type & MAP_FLAG_ANCHORAGE)
#define macro_is_offload(x, y) (mat[x][y].type & MAP_FLAG_OFFLOAD)

#define macro_is_legal_bot(x, y) (macro_in_mat(x, y) && macro_is_land(x, y))
#define macro_is_legal_ship(x, y) (macro_in_mat(x, y) && macro_is_sea(x, y))

inline int dis_man(const Pos &a, const Pos &b) { return macro_dis_man(a.x, a.y, b.x, b.y); }
inline bool in_mat(const Pos &p) { return macro_in_mat(p.x, p.y); }
inline bool is_land(const Pos &p) { return macro_is_land(p.x, p.y); }
inline bool is_land_main(const Pos &p) { return macro_is_land_main(p.x, p.y); }
inline bool is_robot_buy(const Pos &p) { return macro_is_robot_buy(p.x, p.y); }
inline bool is_sea(const Pos &p) { return macro_is_sea(p.x, p.y); }
inline bool is_sea_main(const Pos &p) { return macro_is_sea_main(p.x, p.y); }
inline bool is_ship_buy(const Pos &p) { return macro_is_ship_buy(p.x, p.y); }
inline bool is_berth(const Pos &p) { return macro_is_berth(p.x, p.y); }
inline bool is_anchorage(const Pos &p) { return macro_is_anchorage(p.x, p.y); }
inline bool is_offload(const Pos &p) { return macro_is_offload(p.x, p.y); }
inline bool is_legal_bot(const Pos &p) { return macro_is_legal_bot(p.x, p.y); }
inline bool is_legal_ship(const Pos &p) { return macro_is_legal_ship(p.x, p.y); }

void init_map() {
    char tmp[210];
    for (int row = 0; row < 200; ++row) {
        scanf("%s", tmp);
        for (int col = 0; col < 200; ++col) {
            switch (tmp[col]) {
                case '.': mat[row][col].type = MAP_LAND;            break;
                case '>': mat[row][col].type = MAP_LAND_MAIN;       break;
                case '*': mat[row][col].type = MAP_SEA;             break;
                case '~': mat[row][col].type = MAP_SEA_MAIN;        break;
                case '#': mat[row][col].type = MAP_BARRIER;         break;
                case 'R': mat[row][col].type = MAP_ROBOT_BUY;       break;
                case 'S': mat[row][col].type = MAP_SHIP_BUY;        break;
                case 'B': mat[row][col].type = MAP_BERTH;           break;
                case 'K': mat[row][col].type = MAP_ANCHORAGE;       break;
                case 'C': mat[row][col].type = MAP_MULTILEVEL;      break;
                case 'c': mat[row][col].type = MAP_MULTILEVEL_MAIN; break;
                case 'T': mat[row][col].type = MAP_OFFLOAD;         break;
            }
        }
    }
#ifdef DEBUG_FLAG
    fprintf(stderr, "# Map initialized\n");
#endif
}


#endif //CODECRAFT2024_ICT_MAP_H
