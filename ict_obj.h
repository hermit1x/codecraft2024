//
// Created by 华华 on 2024/4/3.
//

#ifndef CODECRAFT2024_ICT_OBJ_H
#define CODECRAFT2024_ICT_OBJ_H

#include "common.h"
#include <queue>

class Obj {
public:
    Pos p;
    int value, time_appear;

    Obj(): time_appear(-10000) {}
    Obj(int _x, int _y, int _value, int _time_appear): p({_x, _y}), value(_value), time_appear(_time_appear) {}
};

std::vector<Obj> objs;

void remove_outdated_obj(int frame_id) {
    std::vector<Obj>::iterator it = objs.begin();
    while (it != objs.end()) {
        if (it->time_appear + 1000 < frame_id) {
            it++;
        }
        else break;
    }
    objs.erase(objs.begin(), it);
#ifdef DEBUG_FLAG
    fprintf(stderr, "# Frame %d objects.size() = %lu\n", frame_id, objs.size());
#endif
}

void update_obj(int frame_id) {
    int k; // 变化物品数量
    scanf("%d", &k);

    int x, y, value;
    for (int i = 0; i < k; ++i) {
        scanf("%d%d%d", &x, &y, &value);
        if (value == 0) {
            std::vector<Obj>::iterator it = objs.begin();
            while (it != objs.end()) {
                if (it->p.x == x && it->p.y == y) {
                    objs.erase(it);
                    break;
                }
                it++;
            }
        }
        else {
            objs.emplace_back(x, y, value, frame_id);
        }
    }
}

#endif //CODECRAFT2024_ICT_OBJ_H
