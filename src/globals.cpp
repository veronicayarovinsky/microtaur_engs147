#include "globals.h"

namespace Micromouse {
    EncoderData          encoders            = {};
    ImuData              imu                = {};
    TofData              tof                = {};
    DriveCommand         drive_command      = {};
    MotorCommands        motors             = {};
    GridCell             gridcell           = {};
    WallsCurrentCell     walls              = {};
    MazeMap              maze               = {};
    State                state              = State::INIT;
}
