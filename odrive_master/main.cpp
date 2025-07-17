#include "project.h"

// ODrive Serial Port Name
std::string dev_name = "/dev/ttyACM0";
// Control rate [Hz]
const float freq = 1000.0f; 

// Create odrv0 object without opening the serial
ODrive odrv0(freq);

void sigint_handler(int sig);

int main(int argc, char** argv) {

    signal(SIGINT, sigint_handler);

    // ODrive stops with ^C or after a specified time
    odrv0.t_stop = INFINITY;

    // Priority for the "normal" threads
    // We don't need/want real-time (RT) priority here; we set it to -50
    struct sched_param schedp;
    schedp.sched_priority = 49;
    sched_setscheduler(0, SCHED_FIFO, &schedp);

    /* Parse command-line options */
    if( argc > 1 ) {
        if( strcmp(argv[1], "--version") == 0 || strcmp(argv[1], "-V") == 0 ) {
            std::cout << "  ODrive-Master " << __ODriveMaster_VERSION__ << std::endl;
            return 0;
        }
        else if( strcmp(argv[1], "--help") == 0 || strcmp(argv[1], "-h") == 0 ) {
            std::cout << "  ODrive-Master " << __ODriveMaster_VERSION__ << std::endl;
            std::cout << "  For help and documentation, check the README.md file and the code documentation in the folder 'doc'\n";
            return 0;
        }
        
        for( size_t i = 1; i < argc; i++ ) {
            // Set reboot flag
            reboot_odrive |= strcmp(argv[i], "--reboot") == 0 ? true : false;
            // Set debug flag
            debug |= strcmp(argv[i], "--debug") == 1 ? true : false;
            // Get serial port 
            if( strcmp(argv[i], "--port") == 0 || strcmp(argv[i], "-P") == 0 ) {
                dev_name = argv[i+1];
            }
            // Get filename for logging (if any)
            if( strcmp(argv[i], "--filename") == 0 || strcmp(argv[i], "-f") == 0 ) {
                log_filename = argv[i+1];
            }    
            // Set verbosity level 
            if( strcmp(argv[i], "--verbose") == 0 || strcmp(argv[i], "-v") == 0 ) {
                odrv0.set_verbose(atoi(argv[i+1]));
            }
            // Disable watchdog
            if( strcmp(argv[i], "--disable-watchdog") == 0 ) {
                odrv0.watchdog_enable[0] = false;
                odrv0.watchdog_enable[1] = false;
            }
            // Master mode (disabeld shared memory)
            if( strcmp(argv[i], "--master-mode") == 0 ) {
                use_shmem = false;
            }
        }
    }

    /* Set flags and verbosity level */
    // Enable benchmarking for serial communication/control loop rate
    odrv0.benchmark = true;
    // Log data to file
    odrv0.logging = true;
    // Set debug flag as requested by the user
    odrv0.debug = debug;
    // Select whether we are using the SEA torque sensor
    odrv0.sea_torque_enable = false;
    // Use shared memory for high-level control (default: true; disable with '--master-mode')
    odrv0.use_shmem = use_shmem;
    
    // Init ODrive
    odrv0.init(dev_name, reboot_odrive);
    odrv0.set_verbose(LOG_DEBUG); // use either LOG_INFO or LOG_DEBUG

    // Safety flags
    odrv0.DRY_RUN = false;  // if 'true', ODrive remains IDLE
    odrv0.SAFE_MODE = false; // if 'true', the comamnded torque will always be 0
    // Warning for "normal operation"
    if( odrv0.SAFE_MODE == false && odrv0.DRY_RUN == false ) {
        odrv0.console_log.log_message(LOG_WARNING, "SAFE MODE is disabled!\nPlease make sure the motors can safely output torque as intended.");
    }
    if( odrv0.SAFE_MODE ) {
        odrv0.console_log.log_message(LOG_WARNING, "SAFE MODE is enabled: motor torque will always be 0");
    }

    // Manually add damping to axis 0
    // TODO: tune this parameters
    odrv0.position_controller[0].K_d = 0.10f;
    odrv0.position_controller[1].K_d = 0.10f;

    // Lower the safety limits
    odrv0.torque_limits[0] = 0.5f;
    odrv0.torque_limits[1] = 0.5f;

    // Start in impedance mode
    odrv0.working_mode = POSCONTROL_MODE;
    //odrv0.working_mode = SAFE_MODE;

    
    // Press ENTER to start or 'q' to exit
    odrv0.wait_for_start();

    // Enable odrive: dual axis, torque control mode
    // The enable() function is just a wrapper for the private method __do_enable()
    // __do_enable() creates a thread that handles the boost deadline_timer, allowing the 1 kHz loop
    // The function also sets the thread's priority to RT
    odrv0.enable(TORQUE_CONTROL, DUAL_AXIS);
    
    // Press CTRL+C to stop
    while( odrv0.run ) { 

        // Stop signal from the high-level control
        if( odrv0.use_shmem ) {
            if( sem_trywait(odrv0.sem_sync) == 0 ) odrv0.run = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
        
    }

    // Stop motors
    odrv0.stop();

    // Save log file (if any)...
    if (debug)
        odrv0.save_logfile(log_filename);
    // ... and exit
    odrv0.quit(0);

    return 0;

}

// Stop on ^C
void sigint_handler(int sig) {

    odrv0.run = false;

}