#include "globals.h"

namespace Micromouse {
    EncoderData          encoders            = {};
    ImuData              imu                = {};
    TofData              tof                = {};
    ImuCalibrationStatus imu_calibration    = {};
    DriveCommand         drive_command      = {};
    MotorCommands        motors             = {};
    GridCell             gridcell           = {};
    WallsCurrentCell     walls              = {};
    MazeMap              maze               = {};
    State                state              = State::INIT;
}
