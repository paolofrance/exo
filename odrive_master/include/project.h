#pragma once

#include "ODrive.h"

#include <boost/format.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <fstream>
#include <sstream>
#include "AsyncSerial.h"
#include "BufferedAsyncSerial.h"
#include <ncurses.h>
#include <math.h>


#include "version.h"
#include "logging.h"

// ODrive logfile
std::string log_filename;

// Reboot flag (default: false), enable with option --reboot
volatile bool reboot_odrive = false; 
// Debug flag (default: false), enable with option --debug
volatile bool debug = false;
// Use shared-memory flag (default: true), disable with option --master-mode
volatile bool use_shmem = true;

#define TORQUE_CONTROL      0
#define VELOCITY_CONTROL    1
#define POSITION_CONTROL    2

#define AXIS_M0             0
#define AXIS_M1             1
#define DUAL_AXIS           2
