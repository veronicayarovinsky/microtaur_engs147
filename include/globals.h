/**
 * @file globals.h
 * @brief shared data structures in namespace Micromouse
 */

#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include "config.h"

namespace Micromouse {

    struct RobotPose {
        int x = X_START;
        int y = Y_START;
        int a = A_START;
    };

    struct EncoderData {
        // raw counts
        long  enc_left;
        long  enc_right;

        // per sample changes
        float fwd_change_mm;       // (d_L + d_R) / 2      <-- used by pos controller
        float rot_change_rad;     // (d_R - d_L) / TRACK_WIDTH      <— used by angle controller

        // instantaneous rates --> used by individual motor speed control
        float omega_left_rad_s;     // d_L / r / dt
        float omega_right_rad_s;    // d_R / r / dt

        // accumulated odometry
        float dist_traveled_mm;      // sum of fwd_change_mm
        float heading_rad;          // sum of rot_change_rad
    };

    struct ImuData {
        float heading_rad;
        float omega_imu_rad_s;
        bool  ready;
    };

    struct TofData {
        int16_t  dist_left_mm = TOF_MAX_RANGE_MM;
        int16_t  dist_right_mm = TOF_MAX_RANGE_MM;
        int16_t  dist_front_mm = TOF_MAX_RANGE_MM;

        // 45 degree sensors
        int16_t  dist_diag_left_mm = TOF_MAX_RANGE_MM;
        int16_t  dist_diag_right_mm = TOF_MAX_RANGE_MM;

        bool data_refreshed = false;
    };

    // 
    // 0 = no wall, 1 = wall detected, -1 = unknown (invalid readings, etc) <-- these apply to both walls_current_cell & walls_next_cell
    // determined exclusively by TOF sensors, so no knowledge of maze map
    // threshold distance for wall yes/no is defined in config.h (different thresholds for walls_current_cell & walls_next_cell)
    struct WallsCurrentCell {
        int left  = -1;
        int right = -1;
        int front = -1;
    };

    // IMPORTANT: walls_next_cell.left is set by DIAG_LEFT sensor (NOT by LEFT sensor)
    //          & walls_next_cell.right is set by DIAG_RIGHT sensor (NOT by RIGHT sensor)
    //          & walls_next_cell.front is determined by different threshold than walls_current_cell.front
    // If walls_current_cell.front = 1 (meaning current cell has a wall in front),
    //  then ALL NEXT CELL WALL READINGS INVALID (= -1) bc tof sensors cannot see past wall into next cell
    struct WallsNextCell {
        int left  = -1;      // set by DIAG_LEFT sensor
        int right = -1;      // set by DIAG_RIGHT sensor
        int front = -1;
    };

    struct DriveCommand {
        float v_base_mm_s      = 0.0f;
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

    extern RobotPose pose;
    extern EncoderData encoders;
    extern ImuData imu;
    extern TofData tof;
    extern WallsCurrentCell walls_current_cell;
    extern WallsNextCell walls_next_cell;
    extern DriveCommand drive_command;
    extern MotorCommands motors;
    extern GridCell gridcell;
    extern MazeMap maze;
    extern State state;
}

#endif

