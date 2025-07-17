#include "ODrive.h"
#include "ODrive_base.h"

ODrive::~ODrive() {}


std::string ODrive::read_property(const std::string& property) {

    std::string result, command;
    uint8_t count = 0;

    command.append(std::string("r ")); // Read command
    command.append(property);          // Property
    command.push_back('\n');           // Terminator

    console_log.log_message(LOG_TRACE, command.c_str());

    // Send the read command and get the response
    // The communication is asynchronous, so we wait
    // before we try to get the reply...
    // Do not use this in the control loop!!
    while( result.length() == 0 ) {

        // TX command
        serial_port.writeString(command);
        // RX reply: allow for several reading tries (async read function)
        for( uint8_t i = 0; i < 10; i++ ) {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            result = serial_port.readStringUntil();
            if( (strcmp(result.c_str(), "unknown command\r") == 0) || (strcmp(result.c_str(), "invalid property\r") == 0) ) {
                result = "";
            }
            if( result.length() > 0 ) break;
        }

        if( (strcmp(result.c_str(), "unknown command\r") == 0) || (strcmp(result.c_str(), "invalid property\r") == 0) ) {
            result = "";
        }

        count++;
        if( count > 2 ) {
            console_log.ss_buf << "Unable to read property " << property;
            console_log.log_message(LOG_WARNING, console_log.ss_buf.str().c_str());
            return "";
        }

        // Before another TX sleep for 400 ms
        std::this_thread::sleep_for(std::chrono::milliseconds(400));

    }

    console_log.ss_buf << property << " = " << result;
    console_log.log_message(LOG_TRACE, console_log.ss_buf.str().c_str());

    return result;

}


void ODrive::async_read_property(const std::string& property) {

    std::string result, command;
    command.append(std::string("r ")); // Read command
    command.append(property);          // Property
    command.push_back('\r');           // Terminator

    console_log.log_message(LOG_TRACE, command.c_str());

    // Send the read command
    this->serial_port.writeString(command);

    // Set the flag for an async read
    async_property_read_pending = true;

    // TODO
    // How to actually put the reply in this->rx_msg ??
    // We should modify the callback... or check the flag in the control loop ?

}


void ODrive::init(std::string& dev_name, bool reboot) {

    // SEA torque sensor not implemented in the exoskeleton
    if( sea_torque_enable == true ) {
        console_log.log_message(LOG_FATAL, "SEA torque sensor may be unavailable. Check the exoskeleton and disable this warning to continue.\nExiting...");
        exit(1);
    }

    // Set the serial port name
    this->devname = dev_name;
    // Perform init
    __do_init(reboot);   

}


void ODrive::init(bool reboot) {

    __do_init(reboot);

}


// Private method
void ODrive::__do_init(bool reboot) {

    // Init screen
    console_log.init_screen();
    logger::current_line++;
    // Default logging level: info
    verbose = LOG_DEBUG;
    console_log.set_level(verbose);

    __do_open_serial();
    // Connect successful
    console_log.log_message(LOG_INFO, (boost::format("Connected to %1%") % devname).str().c_str());

    // Reboot if necessary
    __do_reboot(reboot);
    if( reboot_ok ) console_log.log_message(LOG_DEBUG, "Rebook ok");

    if( debug ) {
        console_log.log_message(LOG_WARNING, "Debug mode is active!");
    }
    if( sea_torque_enable ) {
        console_log.log_message(LOG_WARNING, "Using the SEA torque sensor (analog pot).");
    }

    // Flush serial
    serial_port.read(nullptr, serial_port.readBufferSize);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    serial_port.countDataAvailable = 0;

    battery_check();

    // Set the interval for the timer
    this->interval = boost::posix_time::microseconds(static_cast<long>(dt_us));

    console_log.log_message(LOG_TRACE, "Init complete.");

    // Init shared memory if necessary
    if( use_shmem ) {

        shmem.init("/usr/local/include/shmem.hpp", 4096);
        // Open only (do not create)
        shmem.attach_memory_block(false);

        if( shmem.check_attached() != true ) {
            console_log.log_message(LOG_FATAL, "shmem init failed!");
            sleep(5);
            exit(-1);
        }

        // Open the semaphores
        sems.push_back(sem_open("/sem_master_impedance", 0)); // sems[0]
        sems.push_back(sem_open("/sem_client_impedance", 0)); // sems[1]
        sems.push_back(sem_open("/sem_master_feedback", 0));  // sems[2]
        sems.push_back(sem_open("/sem_client_feedback", 0));  // sems[3]
        sems.push_back(sem_open("/sem_elio_sync", 0));        // sems[4]
        sem_sync = sems[4];

        // Wake up the master
        sem_post(sems[4]);

    }
    
}


void ODrive::enable(uint8_t control_mode, uint8_t axis) {

    // Create and start the console logger thread with its callback
    t_logger = new std::thread(std::bind(&ODrive::f_consoleLogging, this));
    // Create and start the user input thread with its callback
    t_input = new std::thread(std::bind(&ODrive::f_getUserInput, this));
    // Create and start the shared memory threads
    if( use_shmem ) {
        t_shmem_rx = new std::thread(std::bind(&ODrive::f_shmem_rx, this));
        t_shmem_tx = new std::thread(std::bind(&ODrive::f_shmem_tx, this));
    }

    worker = new boost::thread([&] { __do_enable(control_mode, axis); });

    // Wait for the axes being enabled before returning to main() from this function
    while( axes_enable[0] == false && axes_enable[1] == false ) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

}


void ODrive::stop() {

    // Stop the active axes
    for( uint8_t i = 0; i < 2; i++ ) {

        if( axes_enable[i] ) {

            axes_enable[i] = false;
            torque_command(i, 0.0f);
            std::this_thread::sleep_for(std::chrono::milliseconds(150));
            if( i == 0 ) {
                send_command("w axis0.requested_state 1\n");
            }
            else if( i == 1 ) {
                send_command("w axis1.requested_state 1\n");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

        }

    }

    run = false;
        
    // Cancel and stop the deadline timer (avoid an extra loop)
    timer.cancel();
    timer_io.stop();

    // Join all the running threads
    // t_sharedmem->join();
    t_feedback->join();
    t_logger->join();
    t_input->join();

    console_log.clear_screen();

    worker->join();

    // Close the serial port
    serial_port.close();
    
    // Display benchmark data
    logger::current_line += 4;
    if( benchmark ) compute_benchmark();

}


void ODrive::battery_check() {

    float vbat_nominal, vbat_critical;
    #ifdef VBAT_NOMINAL_V
    vbat_nominal = VBAT_NOMINAL_V;
    #else
    vbat_nominal = 14.8f;
    #endif
    #ifdef VBAT_CRITICAL_V
    vbat_critical = VBAT_CRITICAL_V;
    #else
    vbat_critical = 14.0f;
    #endif

    vbat = read_vbus_voltage();

    if( vbat < vbat_nominal && vbat >= vbat_critical ) {
        console_log.log_message(LOG_WARNING, (boost::format("Battery voltage is below nominal (%1% V)\nConsider charging or replacing the battery.") % vbat).str().c_str());
    }
    else if( vbat < vbat_critical ) {
        console_log.log_message(LOG_FATAL, "Battery voltage is CRITICALLY LOW: charge or replace the battery.\nExiting...");
        exit(1);
    }

}


uint8_t ODrive::set_verbose(uint8_t verbose_level) {

    verbose = verbose_level < LOG_FATAL ? verbose_level : verbose;
    verbose = verbose_level > LOG_TRACE ? verbose_level : verbose;

    return verbose;

}


float ODrive::chirp(uint8_t axis) {

    float t = loop_counter*dt;
    float beta = (chirp_f1 - chirp_f0) * (1.0/chirp_tf);
    float y = chirp_A0;
    y += chirp_Ai * sin(2*M_PI*(beta/(2)*(pow(t, 2)) + chirp_f0*t));
    return y;

}


float ODrive::reducer_comp(uint8_t axis) {

    float tau_comp = 0.0f;
    float _pos = pos[axis]/30.0f, _vel = vel[axis];

    // Model-based compensation
    tau_comp = 0.0050f*sin(23.03*_pos + deg2rad(182.3)) + 0.0035f*sin(15.44*_pos + deg2rad(1.7)) + 0.0010f*sin(193.38*_pos + deg2rad(2.1)) + 0.0008f*sin(184.66*_pos);
    // Offset to provide only if friction compensation is disabled
    if( fcomp[axis] == false ) tau_comp += 0.0300f;

    // Provide this term only for steady-state regime
    tau_comp *= copysignf(friction_comp_params.steady_state, _vel);

    // limit for safety
    if( fabs(tau_comp) > 0.0800f ) tau_comp = copysignf(0.0800f, _vel);
    return tau_comp;

}


void ODrive::save_logfile(std::string fname) {

    if( logging == false ) {
        console_log.log_message(LOG_WARNING, "Logging to file is not enabled (!)");
        return;
    }

    // Log to file
    if( debug == true ) {
        logfile.open("./debug.csv");
        if( !logfile.is_open() ) perror("Cannot open debug.csv");
    }
    else{
        if( fname.size() == 0 ) {
            auto now = std::time(nullptr);
            std::stringstream sname;
            sname << "./log_" << std::put_time(std::gmtime(&now), "%d_%m_%Y_%H_%M_%S") << ".csv";
            logfile.open(sname.str());
        }
        else {
            logfile.open(fname);
        }
        if( !logfile.is_open() ) perror("Cannot open logfile");
    }

    // Header
    //logfile << "n_evt,torque_cmd_0,pos_0,vel_0,torque_0,theta_ref_0,theta_ref_mod_0,err_tor_0,pos_1,torque_cmd_1,theta_ref_1\n";
    logfile << "n_evt,assistance,torque_cmd_0,pos_0,est_torque_0,torque_0,current_0,theta_ref_0,torque_cmd_1,pos_1,vel_0,torque_1,current_1,theta_ref_1,error_integral_0,error_integral_1,ext_torque,filtered_ext_torque,vel_ref_0,filtered_vel_0,vel_1\n";
    
    char buf[1024];
    for( size_t i = 0; i < loop_counter; i++ ) {
        //sprintf(buf, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", evt_log[i], torque_cmd_log[0][i], pos_log[0][i], vel_log[0][i], torque_log[0][i], theta_ref_log[0][i], qobs_log[0][i], err_tor_log[0][i], pos_log[1][i], torque_cmd_log[1][i], theta_ref_log[1][i]);
        sprintf(buf, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", evt_log[i], assistance_log[i], torque_cmd_log[0][i], pos_log[0][i], obs_tor_log[0][i], torque_log[0][i],current_log[0][i], theta_ref_log[0][i], torque_cmd_log[1][i], pos_log[1][i], vel_log[0][i], torque_log[1][i],current_log[1][i], theta_ref_log[1][i],error_integral_log[0][i],error_integral_log[1][i],ext_tor_log[0][i],filt_ext_tor_log[0][i],vel_ref_log[0][i],filt_vel_log[0][i],vel_log[1][i]);
        logfile << buf;
    }
    logfile.close();

}


void ODrive::wait_for_start() {

    logger::current_line += 2;
    console_log.print("Press ENTER to start or 'Q' to quit... \n \n");
    fflush(stdin);
    this->in = getch();
    if( toupper(this->in) == 'Q' ) {
        endwin();
        exit(0);
    }
    logger::current_line += 2;

    // Disable delay on getch() function for the user input loop function
    nodelay(stdscr, 1);

}


void ODrive::quit(uint8_t exit_code) {

    if( use_shmem ) {
        // Detach the threads and exit ('join' would not work on sem_wait)
        // The threads are killed when the process exits
        t_shmem_rx->detach();
        t_shmem_tx->detach();
        // Signal to the master that we are exiting...
        sem_post(sems[4]);
    }

    // Free the memory
    delete(t_logger);
    delete(t_input);
    if( use_shmem ) {
        delete(t_shmem_rx);
        delete(t_shmem_tx);
    }
    
    // Re-set delay on getch() and wait for user input to stop and quit
    nodelay(stdscr, 0);
    fflush(stdin);
    getch();
    endwin();

    exit(exit_code);

}


void ODrive::compute_benchmark() {

    std::vector<std::chrono::duration<double>> dts;
    float min = 2000.0, avg = 0.0, max = -1.0, stdev = 0.0, temp_val;
    uint32_t i;
    float count_below_cycle_time = 0.0;

    // Compute average
    for( i = 1; i < this->loop_counter; i++ ) {
        
        dts.push_back(std::chrono::duration_cast<std::chrono::duration<double>>( t_loops_benchmark[i] - t_loops_benchmark[i-1] ));
        temp_val = dts[i-1].count();
        if( temp_val <= dt*1.03 ) { // 3% tolerance
            count_below_cycle_time++;
        }
        min = temp_val < min ? temp_val : min;
        max = temp_val > max ? temp_val : max;
        avg += temp_val;

    }
    avg /= i;

    // Compute standard deviation
    for( i = 1; i < this->loop_counter; i++ ) {
        stdev += pow(dts[i-1].count() - avg, 2);
    }
    stdev = sqrt(stdev/i);

    logger::current_line += 2;
    console_log.log_message(LOG_INFO, "----- BENCHMARK results");
    console_log.log_message(LOG_INFO, (boost::format("min | avg +/- std | max : %1% | %2% +/- %3% | %4% [ms]\n") % (min*1000) % (avg*1000) % (stdev*1000) % (max*1000)).str().c_str());
    console_log.log_message(LOG_INFO, (boost::format("# loops <= cycle_time: %1% (%2$4.2f %%)\n") % count_below_cycle_time % (100*count_below_cycle_time/this->loop_counter)).str().c_str());
    console_log.log_message(LOG_INFO, (boost::format("RX valid packets = %1%") % valid_packets).str().c_str());

}


void ODrive::f_consoleLogging() {

    // Wait for ODrive to be enabled
    while( axes_enable[0] == false && axes_enable[1] == false );

    static const char header[] = "   POS       VEL       TAU        POS       VEL       TAU  ";
    
    // Header
    uint16_t header_line = logger::current_line;
    console_log.print(header);

    // Display options on the bottom
    mvaddstr(LINES-2, (COLS-strlen(opt))/2, opt);

    // Set color
    logger::current_line++;
    mvchgat(console_log.get_current_line(), 4, COLS-5, 0, 2, nullptr);
    // C-string output buffer for logging
    char str[COLS];
    // Loop count
    static uint64_t count = 0;

    while( this->run ) {

        if( count % 50 == 0 ) {
            console_log.clear_screen();
            // Print header
            console_log.print(header, header_line);
            // Print options
            mvaddstr(LINES-2, (COLS-strlen(opt))/2, opt);
        }

        if( update_screen != ERR && update_screen != 0 ) {
            
            switch(tolower(update_screen)) {
                case 'k': 
                {
                    mvchgat(LINES-2, (COLS-strlen(opt))/2 + 19*0, 19, 0, 2, nullptr);
                    mvchgat(LINES-2, (COLS-strlen(opt))/2 + 19*1, 19, 0, 1, nullptr);
                }
                break;

                case 'z':
                {
                    console_log.clear_line(logger::current_line+2);
                    console_log.clear_line(logger::current_line+3);
                    mvchgat(LINES-2, (COLS-strlen(opt))/2 + 19*0, 19, 0, 1, nullptr);
                    mvchgat(LINES-2, (COLS-strlen(opt))/2 + 19*1, 19, 0, 2, nullptr);

                }
                break;

                case 'f':
                {
                    if( fcomp[0] ) {
                        mvchgat(LINES-2, (COLS-strlen(opt))/2 + 19*2 -1, 19, 0, 2, nullptr);
                    }
                    else {
                        mvchgat(LINES-2, (COLS-strlen(opt))/2 + 19*2 -1, 19, 0, 1, nullptr);
                    }
                }
                break;

                case 'r':
                {
                    if( rcomp[0] ) {
                        mvchgat(LINES-2, (COLS-strlen(opt))/2 + 19*3 -1, 19, 0, 2, nullptr);
                    }
                    else {
                        mvchgat(LINES-2, (COLS-strlen(opt))/2 + 19*3 -1, 19, 0, 1, nullptr);
                    }
                }
                break;
                default:
                break;
            }
            refresh();

            update_screen = 0;

        }
        else if( update_screen == ERR ) {

            mvaddstr(LINES-4, 2, "Invalid selection. ");
            refresh();

        }

        // 10 Hz console refresh
        if( count % 5 == 0 ) {

            sprintf(str, "%+8.4f  %+8.4f  %+8.4f %+8.4f  %+8.4f  %+8.4f  ", pos[0], vel[0], sea[0], pos[1], vel[1], sea[1]);
            console_log.print(str);

            if( working_mode == POSCONTROL_MODE ) {
                sprintf(str, "\tReference Set-Point\t%+8.4f\t%+8.4f", position_controller[0].theta_ref, position_controller[1].theta_ref);
                console_log.print(str, logger::current_line+2);
                sprintf(str, "\tProportional Gain       \t%+8.4f\t%8.4f", position_controller[0].K_p, position_controller[1].K_p);
                console_log.print(str, logger::current_line+3);
            }

            sprintf(str, "System error: %x, Axis0 error: %x, Axis1 error: %x   |  Battery Voltage %.2f V", error, controller_error[0],controller_error[1], vbat); //controller_error[1]
            console_log.print(str, LINES-4);

        }

        count++;

        // ~50 Hz
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

    }

}

void ODrive::f_getUserInput() {

    // Wait for ODrive to be enabled (at least one axis)
    while( axes_enable[0] == false && axes_enable[1] == false );

    bool valid = true;

    fflush(stdin);

    while( this->run ) {

        // non-blocking
        this->in = getch();

        switch(tolower(in)) {

            case 'k':
                working_mode = POSCONTROL_MODE;
            break;

            case 'z':
                working_mode = ZERO_TORQUE_MODE;
            break;
            
            case 'f':
                fcomp[0] ^= true;
                fcomp[1] ^= true;
            break;
            
            case 'r':
                rcomp[0] ^= true;
                rcomp[1] ^= true;
            break;
            
            default:
                //valid = false;
                //update_screen = ERR;
            break;
        
        }

        if( valid ) update_screen = tolower(in);

        std::this_thread::sleep_for(std::chrono::microseconds(500));

    }

}


void ODrive::f_shmem_rx() {

    static_assert(sizeof(float) == 4);

    // Return value
    int r = 0;
    // Local temporary buffer
    float temp[17*sizeof(float)];

    while( run ) {

    /** Critical Section **/
        r = sem_wait(sems[0]);
        shmem.read((uint8_t*) temp, 17*sizeof(float), 0);
        sem_post(sems[1]);
    /** Critical Section **/

        position_controller[0].ext_torque = temp[0];
        position_controller[0].theta_ref = temp[2];
        position_controller[0].t_obs = temp[4];
        position_controller[0].filtered_ext_torque = temp[6];
        position_controller[0].vel_ref = temp[8];
        position_controller[0].filtered_vel = temp[11];
        position_controller[0].startpos = temp[14];
        
        position_controller[1].ext_torque = temp[1];
        position_controller[1].theta_ref = temp[3];
        position_controller[1].t_obs = temp[5];
        position_controller[1].filtered_ext_torque = temp[7];
        position_controller[1].vel_ref = temp[9];
        position_controller[1].filtered_vel = temp[12];
        position_controller[1].startpos = temp[15];
        

        assistance = temp[10];
        mode = temp[13];
        cmode = temp[16];
        //memcpy(assistance,temp[12],sizeof(bool));

    }

}


void ODrive::f_shmem_tx() {

    static_assert(sizeof(float) == 4);

    // Return value
    int r = 0;
    // Local temporary buffer
    float temp[8];
    uint32_t prev_count = 0;

    while( run ) {

        if( prev_count < loop_counter ) {
        /** Critical Section **/
            r = sem_wait(sems[3]);
            // Position
            memcpy(&temp[0], pos, 2*sizeof(float));
            // Torque
            memcpy(&temp[2], sea, 2*sizeof(float));
            // Velocity
            memcpy(&temp[4], vel, 2*sizeof(float));
            // Commanded Torque
            memcpy(&temp[6], torque_cmd , 2*sizeof(float));
            
            shmem.write((uint8_t*) temp, 8*sizeof(float), 17*sizeof(float));
            sem_post(sems[2]);
            
            
        /** Critical Section **/
            prev_count++;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(500));

    }

}
