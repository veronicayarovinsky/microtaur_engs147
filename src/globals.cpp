#include "globals.h"
#include "config.h"


namespace Micromouse {
    RobotPose            pose               = {};
    EncoderData          encoders            = {};
    ImuData              imu                = {};
    TofData              tof                = {};
    // ImuCalibrationStatus imu_calibration    = {};
    DriveCommand         drive_command      = {};
    MotorCommands        motors             = {};
    GridCell             gridcell           = {};
    WallsCurrentCell     walls_current_cell = {};
    WallsNextCell        walls_next_cell    = {};
    MazeMap              maze               = {};
    State                state              = State::INIT;
}
