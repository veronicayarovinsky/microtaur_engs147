/**
 * @file globals.h
 * @brief shared data structures in namespace Micromouse
 */

#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include "config.h"

namespace Micromouse {

    // ***** Encoder data — written by encoder.cpp *****
    struct EncoderData {
        long  enc_left;
        long  enc_right;
        float omega_left_rad_s;
        float omega_right_rad_s;
        float dist_traveled_m;
    };

    // ***** IMU data — written by imu.cpp *****
    struct ImuData {
        float heading_rad;
        float omega_imu_rad_s;
        bool  ready;
    };

    struct ImuCalibrationStatus {
        int   sys_cal   = 0;
        int   gyr_cal   = 0;
        int   acc_cal   = 0;
        int   mag_cal   = 0;
    };

    // ***** TOF data — written by tof.cpp *****
    struct TofData {
        // Raw distances
        float dist_left_mm = TOF_MAX_RANGE_MM;
        float dist_diagL_mm = TOF_MAX_RANGE_MM;
        float dist_right_mm = TOF_MAX_RANGE_MM;
        float dist_diagR_mm = TOF_MAX_RANGE_MM;
        float dist_front_mm = TOF_MAX_RANGE_MM;
        // Wall presence (computed by tof.cpp: dist < WALL_PRESENT_MM)
        bool  wall_left = false;
        bool  wall_right = false;
        bool  wall_front = false;
        bool  data_refreshed = false;
    };

    // ***** Drive Setpoints — written by fsm.cpp *****
    // reference values / what the micromouse is told to do
    // (v_base_m_s = 0) & (heading_ref = target) --> turn
    // (v_base_m_s > 0) & (heading_ref = fixed)  --> drive straight
    struct DriveCommand {
        float v_base_m_s      = 0.0f;
        float heading_ref_rad = 0.0f;
    };

    // ***** Motor Commands — written by motor_pi.cpp, read by drive.cpp *****
    // pwm values actually sent to the motors
    struct MotorCommands {
        int pwm_left  = 0;
        int pwm_right = 0;
    };

    // ***** Current Grid Cell written by fsm.cpp *****
    // written by fsm.cpp
    struct GridCell {
        int x = 0;
        int y = 0;
    };

    // ***** Walls At Current Cell – written by fsm.cpp *****
    // read from TofData.wall_* when micromouse is in AT_CELL state
    // stored separately from TofData so that FSM / maze logic has stable values
    // since TofData is overwritten on every TOF control loop (~15ms)
    // ++++++++++++++++++
    // TODO: not sure if this is actually needed lol, maybe just reading from TofData is ok
    // ++++++++++++++++++
    struct Walls {
        bool left  = false;
        bool leftdiag = false;
        bool right = false;
        bool rightdiag = false;
        bool front = false;
    };

    // ***** FSM state – written by fsm.cpp *****
    enum class State {
        INIT,               // waiting for button press
        AT_CELL,            // stopped at cell — sense walls, decide next move
        TURNING,            // drive_turn()
        DRIVING_FORWARD,    // drive_forward()
        PAUSED,             // stopped mid-run (for debugging); resumes when btn pressed
        GOAL,               // reached goal cell (maze solved)
        DONE
    };

    // ***** Maze Map *****
    struct MazeMap {
        unsigned short walls[MAZE_SIZE][MAZE_SIZE] = {};
        bool    visited[MAZE_SIZE][MAZE_SIZE] = {};
    };
 
    extern EncoderData          encoders;
    extern ImuData              imu;
    extern TofData              tof;
    extern DriveCommand         drive_command;
    extern MotorCommands        motors;
    extern GridCell             gridcell;
    extern Walls                walls;
    extern MazeMap              maze;
    extern State                state;
}

#endif
