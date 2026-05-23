void  lateral_pd_init();
void  lateral_pd_reset();
void  lateral_pd_set_enabled(bool enabled);

// call when tof_read_if_ready() returns true
void  lateral_pd_update();

// returns the heading correction [rad] to pass to heading_pd_update().
// returns 0.0 when disabled
// +++++++++++++++++++++++++
// NEED TO CHANGE SO IT DOES THINGS WHEN ONLY ONE WALL
// or walls not present on both sides ???
// ++++++++++++++++++++++++++
float lateral_pd_get_correction();
