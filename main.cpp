#include <iostream>
#include <algorithm>
#include <queue>
#include <vector>
#include <memory>
#include <ctime>

//#define DEBUG_FLAG
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

double t_tot = 0;
double t_init = 0;
double t_r_in = 0, t_r_think = 0, t_r_act = 0;
double t_s_in = 0, t_s_think = 0, t_s_act = 0;
#define my_difftime(t1, t0) ((double)(t1 - t0) / CLOCKS_PER_SEC)

int map1 = 0, map2 = 0;

void Init() {
    srand((unsigned int)time(NULL));
    init_map();
    if (is_ship_buy(Pos(2, 2))) {
        // map1
        map1 = 1;
        want_ship_num = 2;
        want_robot_num = 14 ;
        SHIP_STRATEGY = SHIP_VALUE_BY_DIS;
    }
    if (is_ship_buy(Pos(3, 22))) {
        // map2
        map2 = 1;
        want_ship_num = 1;
        want_robot_num = 20;
        SHIP_STRATEGY = SHIP_VALUE_BY_DIS;
        /*
         * 1+20 69300
         */
    }

    init_berth();
    init_offload();
    init_robot_birth();
    scanf("%d", &ship_capacity);
    fprintf(stderr, "ship_capacity inited\n");
    char okk[100];
    scanf("%s", okk);
    init_sea_map();

    printf("OK\n");
    fflush(stdout);
#ifdef DEBUG_FLAG
    fprintf(stderr, "#1 Init Finish\n");
#endif
}

void Input(int &frame_id, int &money) {
    scanf("%d%d", &frame_id, &money);
//    fprintf(stderr, "[tick] frame_id: %d, money: %d\n", frame_id, money);
    update_obj(frame_id);
//    fprintf(stderr, "obj updated, ");
    clock_t t0 = clock();
    update_robot(frame_id);
//    fprintf(stderr, "robot updated, ");
    clock_t t1 = clock();
    update_ship(frame_id);
//    fprintf(stderr, "ship updated \n");
    clock_t t2 = clock();
    t_r_in += my_difftime(t1, t0);
    t_s_in += my_difftime(t2, t1);

    char okk[100];
    scanf("%s", okk);
}



int main() {
#ifdef DEBUG_FLAG
    fprintf(stderr, "#0 Program start\n");
#endif
    clock_t t00 = clock();
    Init();
    clock_t t01 = clock();
    t_init = my_difftime(t01, t00);
    if (map2) berths[1].closed = 1;
    int frame_id, money;
    for (int frame = 1; frame <= 15000; frame++) {
        if (map1 && frame > 13000) berths[0].closed = 1;
#ifdef DEBUG_FLAG
        fprintf(stderr, "#1 Frame start\n");
#endif
        Input(frame_id, money);
//        fprintf(stderr, "\n");
#ifdef DEBUG_FLAG
        fprintf(stderr, "#2 Input Finish\n");
        if (frame_id != frame) {
            fprintf(stderr, "# LOOP SYNC ERROR: loop:%d, frame get:%d\n", frame, frame_id);
        }
#endif
        if (frame_id != frame) {
            frame = frame_id;
            fprintf(stderr, "### FRAME SYNC ERROR ###\n");
        }

//        fprintf(stderr, "#3 Think begin\n");
        clock_t t0 = clock();
        for (int i = 0; i < robot_num; ++i) robots[i].think();
        handle_conflict_robot();
//        fprintf(stderr, "robot think finish\n");
        clock_t t1 = clock();
        for (int i = 0; i < ship_num; ++i) ships[i].think();
//        fprintf(stderr, "ship think finish\n");
        handle_conflict_ship();
//        fprintf(stderr, "ship conflict finish\n");
        clock_t t2 = clock();

        t_r_think += my_difftime(t1, t0);
        t_s_think += my_difftime(t2, t1);

        if (conflict_error_robot) {
            fprintf(stderr, "CONFLICT ERROR frame: %d, frame_id: %d\n", frame, frame_id);
        }

        clock_t t3 = clock();
        for (int i = 0; i < robot_num; ++i) robots[i].act();
        clock_t t4 = clock();
//        fprintf(stderr, "#6 Robot Act finish\n");
        for (int i = 0; i < ship_num; ++i) ships[i].act();
        clock_t t5 = clock();

        t_r_act += my_difftime(t4, t3);
        t_s_act += my_difftime(t5, t4);
//        fprintf(stderr, "#7 Ship Act finish\n");
//        if (frame > 12000) calc_close_berth(frame);

        test_buy_ship(frame, money);
        test_buy_robot(frame, money);

        printf("OK\n");
        fflush(stdout);
#ifdef DEBUG_FLAG
        fprintf(stderr, "#9 Output Finish\n");
#endif
    }
//    fprintf(stderr, "v_full speed: %d\n\n", v_full);
//#ifdef DEBUG_BERTH
    fprintf(stderr, "# ship_capacity %d\n", ship_capacity);
    fprintf(stderr, "# tot_obj_value %d\n", tot_obj_value);
    int sum_remain_value = 0, sum_tot_value = 0, sum_loads_value = 0;
    for (int i = 0; i < berth_num; ++i) {
        fprintf(stderr, "#BERTH%d: remain value:%d, tot value:%d, tot stock %d, remain stock %d\n", i, berths[i].remain_value, berths[i].tot_value, berths[i].tot_stock, berths[i].stock);
        sum_remain_value += berths[i].remain_value;
        sum_tot_value += berths[i].tot_value;
    }
    for (int i = 0; i < ship_num; ++i) {
        sum_loads_value += ships[i].loads_value;
    }
    fprintf(stderr, "score: %d\n", tot_score);
    fprintf(stderr, "berth total_value:%d\n", sum_tot_value);
    fprintf(stderr, "ship_remain_value:%d\n", sum_loads_value);
    fprintf(stderr, "sum remain_value:%d\n\n", sum_remain_value);


    clock_t t02 = clock();
    t_tot = my_difftime(t02, t00);
    fprintf(stderr, "t_init: %lf, t_tot: %lf\n\n", t_init, t_tot);
    fprintf(stderr, "t_r_in: %lf\nt_r_think: %lf\nt_r_act: %lf\n\n", t_r_in, t_r_think, t_r_act);
    fprintf(stderr, "t_s_in: %lf\nt_s_think: %lf\nt_s_act: %lf\n\n", t_s_in, t_s_think, t_s_act);
    fflush(stderr);
//#endif
    return 0;
}