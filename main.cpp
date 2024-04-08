#include <iostream>
#include <algorithm>
#include <queue>
#include <vector>
#include <memory>
#include <time.h>

#define DEBUG_FLAG
//#define DEBUG_ROBOT
//#define DEBUG_ROBOT_SCHEDULE
//#define DEBUG_SHIP
//#define DEBUG_BERTH
//#define OUTPUT_DIJKSTRA


#include "ict_map.h"
#include "ict_berth.h"
#include "ict_obj.h"
#include "ict_robot.h"
#include "ict_ship.h"


int tot_value = 0;

void Init() {
    srand((unsigned int)time(NULL));
    init_map();
    init_berth();
    init_offload();
    init_robot_birth();
    scanf("%d", &ship_capacity);
    fprintf(stderr, "ship_capacity inited\n");
    char okk[100];
    scanf("%s", okk);

//    debug_print_dist(0);

    /* 计算港口价值
     * 价值 = \sum 1/(dis[x][y])
     * 对于多个地方都能抵达的陆地，对港口i的贡献为
     * (1/Di) * [(1/Di) / (1/Di + 1/Dj + 1/Dk)]
     *
     * */
//    double dominator = 0; // 分母
//    for (int x = 0; x < 200; ++x) {
//        for (int y = 0; y < 200; ++y) {
//            dominator = 0;
//            for (int i = 0; i < berth_num; ++i) {
//                if (is_reachable({x, y}, i) &&  mat[x][y].dist[i] != 0) {
//                    dominator += 1.0 / (double)mat[x][y].dist[i];
//                }
//            }
//            for (int i = 0; i < berth_num; ++i) {
//                if (is_reachable({x, y}, i) &&  mat[x][y].dist[i] != 0) {
//                    berth[i].estimate_value += 1.0 / ((double)mat[x][y].dist[i] * (double)mat[x][y].dist[i]) / dominator;
//                }
//            }
//        }
//    }


//    calc_pd();
    printf("OK\n");
    fflush(stdout);
#ifdef DEBUG_FLAG
    fprintf(stderr, "#1 Init Finish\n");
#endif
}

void Input(int &frame_id) {
    int money; // 帧数，金钱
    scanf("%d%d", &frame_id, &money);
    fprintf(stderr, "  input1\n");
    update_obj(frame_id);
    fprintf(stderr, "  input2\n");
    update_robot(frame_id);
    fprintf(stderr, "  input3\n");
    update_ship(frame_id);
    fprintf(stderr, "  input4\n");
    char okk[100];
    scanf("%s", okk);
}



int main() {
#ifdef DEBUG_FLAG
    fprintf(stderr, "#0 Program start\n");
#endif
    Init();
    int frame_id;
    for (int frame = 1; frame <= 15000; frame++) {
#ifdef DEBUG_FLAG
        fprintf(stderr, "#1 Frame start\n");
#endif
        Input(frame_id);
#ifdef DEBUG_FLAG
        fprintf(stderr, "#2 Input Finish\n");
        if (frame_id != frame) {
            fprintf(stderr, "# LOOP SYNC ERROR: loop:%d, frame get:%d\n", frame, frame_id);
        }
#endif
        if (frame_id == 1) {
//            test_buy_robot();
            test_buy_ship();
        }
        if (frame_id != frame) {
            frame = frame_id;
        }
        fprintf(stderr, "#3 Think begin\n");
        for (int i = 0; i < robot_num; ++i) robots[i].think();
        for (int i = 0; i < ship_num; ++i) ships[i].think();
        fprintf(stderr, "#4 Think finish\n");
        handle_conflict_robot();
        handle_conflict_ship();
        fprintf(stderr, "#5 Conflict finish\n");
        if (conflict_error_robot || conflict_error_ship) {
            fprintf(stderr, "-> frame: %d, frame_id: %d\n", frame, frame_id);
        }

        for (int i = 0; i < robot_num; ++i) robots[i].act();
        fprintf(stderr, "#6 Robot Act finish\n");
        for (int i = 0; i < ship_num; ++i) ships[i].act();
        fprintf(stderr, "#7 Ship Act finish\n");
//        if (frame > 12000) calc_close_berth(frame);
        if (frame <= 16 && (frame & 1)) test_buy_robot();
        printf("OK\n");
        fflush(stdout);
#ifdef DEBUG_FLAG
        fprintf(stderr, "#9 Output Finish\n");
#endif
    }
//    fprintf(stderr, "v_full speed: %d\n\n", v_full);
//#ifdef DEBUG_BERTH
    fprintf(stderr, "#TOTAL VALUE %d\n", tot_value);
    for (int i = 0; i < berth_num; ++i) {
        fprintf(stderr, "#BERTH%d: remain value:%d, tot value:%d, tot stock %d, remain stock %d\n", i, berths[i].remain_value, berths[i].tot_value, berths[i].tot_stock, berths[i].stock);
    }
    fprintf(stderr, "\n\n");
    fflush(stderr);
//#endif
    return 0;
}