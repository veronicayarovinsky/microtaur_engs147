/**
 * @file globals.h
 * @brief shared data structures in namespace Micromouse
 */

#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include "config.h"

namespace Micromouse {

    struct EncoderData {
        long  enc_left;
        long  enc_right;
        float omega_left_rad_s;
        float omega_right_rad_s;
        float dist_traveled_m;
    };

    struct ImuData {
        float heading_rad;
        float omega_imu_rad_s;
        bool  ready;
    };

    struct TofData {
        float dist_left_m = TOF_MAX_RANGE_M;
        float dist_right_m = TOF_MAX_RANGE_M;
        float dist_front_m = TOF_MAX_RANGE_M;

        // 45 degree sensors
        float dist_front_left_m = TOF_MAX_RANGE_M;
        float dist_front_right_m = TOF_MAX_RANGE_M;

        bool wall_left = false;
        bool wall_right = false;
        bool wall_front = false;

        // 45 degree sensor flags
        bool wall_front_left = false;
        bool wall_front_right = false;

        bool data_refreshed = false;
    };

    struct DriveCommand {
        float v_base_m_s      = 0.0f;
        float heading_ref_rad = 0.0f;
    };

    struct MotorCommands {
        int pwm_left  = 0;
        int pwm_right = 0;
    };

    struct GridCell {
        int x = 0;
        int y = 0;
    };

    struct WallsCurrentCell {
        bool left  = false;
        bool right = false;
        bool front = false;
    };

    enum class State {
        INIT,
        AT_CELL,
        TURNING,
        DRIVING_FORWARD,
        PAUSED,
        GOAL,
        DONE
    };

    struct MazeMap {
        unsigned short walls[MAZE_SIZE][MAZE_SIZE] = {};
        bool visited[MAZE_SIZE][MAZE_SIZE] = {};
    };

    extern EncoderData encoders;
    extern ImuData imu;
    extern TofData tof;
    extern DriveCommand drive_command;
    extern MotorCommands motors;
    extern GridCell gridcell;
    extern WallsCurrentCell walls;
    extern MazeMap maze;
    extern State state;
}

#endif

