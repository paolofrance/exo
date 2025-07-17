#include "ODrive_base.h"
#include "Eigen/Dense"
#include <cmath>

using namespace Eigen;

ODrive_base::~ODrive_base() {}

void ODrive_base::__init_buffers() {

    // Init the circular buffers
    b_pos[0] = boost::circular_buffer<float>(12);
    b_pos[1] = boost::circular_buffer<float>(12);
    b_vel[0] = boost::circular_buffer<float>(12);
    b_vel[1] = boost::circular_buffer<float>(12);
    b_Iqm[0] = boost::circular_buffer<float>(12);
    b_Iqm[1] = boost::circular_buffer<float>(12);
    b_sea[0] = boost::circular_buffer<float>(12);
    b_sea[1] = boost::circular_buffer<float>(12);

}


void ODrive_base::send_command(const std::string& command) {

    if( command.back() != '\r' || command.back() != '\n' ) {
        std::string cmd = command;
        cmd.push_back('\r');
        this->serial_port.writeString(cmd);
    } else {
        this->serial_port.writeString(command);
    }

}


std::string ODrive_base::read_property(const std::string& property) {

    std::string result, command;
    uint8_t count = 0;

    command.append(std::string("r ")); // Read command
    command.append(property);          // Property
    command.push_back('\n');           // Terminator

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
            return "";
        }

        // Before another TX sleep for 400 ms
        std::this_thread::sleep_for(std::chrono::milliseconds(400));

    }

    return result;

}


void ODrive_base::async_read_property(const std::string& property) {

    std::string result, command;
    command.append(std::string("r ")); // Read command
    command.append(property);          // Property
    command.push_back('\r');           // Terminator

    // BOOST_LOG_TRIVIAL(debug) << command;

    // Send the read command
    this->serial_port.writeString(command);

    // Set the flag for an async read
    async_property_read_pending = true;

    // TODO
    // How to actually put the reply in this->rx_msg ??
    // We should modify the callback... or check the flag in the control loop ?

}


void ODrive_base::init(std::string& dev_name, bool reboot) {

    // Set the serial port name
    this->devname = dev_name;
    // Perform init
    __do_init(reboot);

}


void ODrive_base::init(bool reboot) {

    __do_init(reboot);

}


void ODrive_base::__do_init(bool reboot) {

    // Open serial port
    __do_open_serial();

    // Reboot if necessary
    __do_reboot(reboot);

    // Flush serial
    serial_port.read(nullptr, serial_port.readBufferSize);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    serial_port.countDataAvailable = 0;

    // Set the interval for the timer
    this->interval = boost::posix_time::microseconds(static_cast<long>(dt_us));

}


void ODrive_base::__do_open_serial() {

    // Try opening the serial port; this may fail soon after power on
    uint8_t count = 0;
    while( !serial_port.isOpen() ) {

        try {
            serial_port.open(devname, 115200);
        }
        catch(const std::exception& e) {
            count++;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if( count > 10 ) throw(e);
        }        

    }

}


void ODrive_base::__do_reboot(bool reboot) {

    if( reboot ) {

        // Get the n_evt_control_loop counter from ODrive
        // We use this to check whether the reboot failed: after rebooting, n_evt_control_loop must be 
        // lower than the value we sample here
        int32_t n_evt = atoi(read_property("n_evt_control_loop").c_str());

        // Reboot ODrive
        send_command("sr");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        serial_port.close();
        std::this_thread::sleep_for(std::chrono::seconds(2));

        uint8_t count = 0;
        // Try re-opening the serial port
        while( !serial_port.isOpen() ) {

            try {
                serial_port.open(devname, 115200);
            }
            catch(const std::exception& e) {
                count++;
                std::this_thread::sleep_for(std::chrono::seconds(2));
                if( count > 10 ) throw(e);
            }        

        }
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Reboot ok if (and only if?) n_evt is now < previous value (i.e., n_evt < 0)
        n_evt = atoi(read_property("n_evt_control_loop").c_str()) - n_evt;
        reboot_ok = (n_evt < 0) ? true : false;
    
    }
    else {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

}


void ODrive_base::enable(uint8_t control_mode, uint8_t axis) {

    worker = new boost::thread([&] { __do_enable(control_mode, axis); });

    // Wait for the axes being enabled before returning from this function
    while( axes_enable[0] == false && axes_enable[1] == false ) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

}


void ODrive_base::__do_enable(uint8_t control_mode, uint8_t axis) {

    // Set real-time priority for this thread (i.e., boost::thread worker)
    struct sched_param schedp;
    schedp.sched_priority = 99;
    sched_setscheduler(getpid(), SCHED_FIFO, &schedp);

    std::stringstream scmd;

    for( uint8_t ax = 0; ax < 2; ax++ ) {

        // Check whether we need to skip the axis
        if( ax == 0 && axis == 1 ) continue;
        if( ax == 1 && axis == 0 ) continue;

        // Set the control mode
        scmd << "w axis" << std::to_string(ax) << ".controller.config.control_mode " << std::to_string(control_mode+1);
        send_command(scmd.str());
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        scmd.str("");

        // Set the input mode: this requires the custom firmware for ODrive
        if( control_mode < 2 ) {
            // Custom input mode (either pass-through or SEA torque loop)
            scmd << "w axis" << std::to_string(ax) << ".controller.config.input_mode 9";
        }
        else {
            // Filtered position mode
            scmd << "w axis" << std::to_string(ax) << ".controller.config.input_mode 3";
        }
        send_command(scmd.str());
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        scmd.str("");

        // Reset the setpoint
        if( control_mode < 2 ) {
            std::vector<std::string> sp;
            sp.push_back("input_torque");
            sp.push_back("input_vel");
            scmd << "w axis" << std::to_string(ax) << ".controller." << sp[control_mode] << " 0";
            send_command(scmd.str());
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        scmd.str("");

        // Go to closed-loop control
        scmd << "w axis" << std::to_string(ax) << ".requested_state 8";
        send_command(scmd.str());
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        scmd.str("");

        // Enable watchdog
        if( watchdog_enable[ax] ) {

            scmd << "w axis" << std::to_string(ax) << ".config.enable_watchdog 1";
            send_command(scmd.str());
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            scmd.str("");

        }

        // axis enabled
        axes_enable[ax] = true;

    }

    // Create the thread to launch the get_feedback function
    // Lower-priority/lower-deadline-precision loop function @ 4 kHz
    // TODO : re-implement as a deadline_timer ?
    t_feedback = new std::thread(std::bind(&ODrive_base::get_feedback, this));

    timer.expires_from_now(boost::posix_time::milliseconds(1));
    // https://stackoverflow.com/a/69335704/9664143
    // https://isocpp.org/wiki/faq/pointers-to-members
    timer.async_wait(boost::bind(&ODrive_base::__control_loop, this));
    // Enter IO loop.   
    // Re-arm the timer in its callback to enter an infinite loop at the specified time interval
    timer_io.run();

    /**
    **** NOTE: timer_io.run() is blocking the worker thread --> this functions NEVER RETURNS!
    **/
    
}


void ODrive_base::stop() {

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

    // Stop the deadline timer
    timer.cancel();
    timer_io.stop();

    // Stop the sub threads
    worker->join();
    t_feedback->join();

    serial_port.close();

    // Free the memory
    delete(worker);
    delete(t_feedback);

    if( benchmark ) compute_benchmark();

}


void ODrive_base::torque_command(uint8_t axis, float torque_setpoint) {

    static_assert(sizeof(float) == 4); // this is evaluated at compile time

    if( SAFE_MODE ) {
        torque_setpoint = 0.0f;
    }
    else {
        torque_setpoint = torque_limit_clip(torque_setpoint, torque_limits[axis]);
    }


    size_t len = 2+enc_size_float;
    uint8_t t_cmd[len];
    t_cmd[0] = 'a' + axis;  // Start byte: 'a' for axis 0, 'b' for axis 1
    t_cmd[len-1] = 0x0A;    // Terminator: '\n'
    // Encode torque setpoint
    encode_ascii85((const uint8_t*) &torque_setpoint, sizeof(float), &t_cmd[1], enc_size_float);

    // TX data
    serial_port.write(t_cmd, len);

}


void ODrive_base::torque_command(float* torque_setpoint) {

    static_assert(sizeof(float) == 4 ); // this is evaluated at compile time

    // Check torque limits and safe-mode settings
    if( SAFE_MODE ) {
        torque_setpoint[0] = 0.0f;
        torque_setpoint[1] = 0.0f;
    }
    else {
              
        torque_setpoint[0] = torque_limit_clip(torque_setpoint[0], torque_limits[0]);
        torque_setpoint[1] = torque_limit_clip(torque_setpoint[1], torque_limits[1]);

    }
    
    torque_setpoint[0] = torque_setpoint[0];
    torque_setpoint[1] = -torque_setpoint[1];
    
    // Packet length
    size_t len = 2+2*enc_size_float;
    // Buffer
    uint8_t t_cmd[len];

    // Start byte
    t_cmd[0] = 'T';
    // Terminator: '\n'
    t_cmd[len-1] = 0x0A;

    // Encode torque setpoint values
    encode_ascii85((const uint8_t*) torque_setpoint, 2*sizeof(float), &t_cmd[1], 2*enc_size_float);

    // TX data
    serial_port.write(t_cmd, len);

}


void ODrive_base::vel_command(uint8_t axis, float vel_setpoint) {

    static_assert(sizeof(float) == 4); // this is evaluated at compile time

    size_t len = 2+enc_size_float;
    uint8_t v_cmd[len];
    v_cmd[0] = 'x' + axis;  // Start byte: 'x' for axis 0, 'y' for axis 1
    v_cmd[len-1] = 0x0A;    // Terminator: '\n'
    // Encode vel setpoint
    encode_ascii85((const uint8_t*) &vel_setpoint, sizeof(float), &v_cmd[1], enc_size_float);

    // TX data
    serial_port.write(v_cmd, len);

}


void ODrive_base::vel_command(float* vel_setpoint) {

    static_assert(sizeof(float) == 4 ); // this is evaluated at compile time

    // Packet length
    size_t len = 2+2*enc_size_float;
    // Buffer
    uint8_t v_cmd[len];

    // Start byte
    v_cmd[0] = 'V';
    // Terminator: '\n'
    v_cmd[len-1] = 0x0A;

    // Encode torque setpoint values
    encode_ascii85((const uint8_t*) vel_setpoint, 2*sizeof(float), &v_cmd[1], 2*enc_size_float);

    // TX data
    serial_port.write(v_cmd, len);

}


void ODrive_base::pos_command(uint8_t axis, float pos_setpoint) {

    static_assert(sizeof(float) == 4); // this is evaluated at compile time

    size_t len = 2+enc_size_float;
    uint8_t p_cmd[len];
    p_cmd[0] = 'm' + axis;  // Start byte: 'x' for axis 0, 'y' for axis 1
    p_cmd[len-1] = 0x0A;    // Terminator: '\n'
    // Encode vel setpoint
    encode_ascii85((const uint8_t*) &pos_setpoint, sizeof(float), &p_cmd[1], enc_size_float);

    // TX data
    serial_port.write(p_cmd, len);

}


void ODrive_base::pos_command(float* pos_setpoint) {

    static_assert(sizeof(float) == 4 ); // this is evaluated at compile time

    // Packet length
    size_t len = 2+2*enc_size_float;
    // Buffer
    uint8_t p_cmd[len];

    // Start byte
    p_cmd[0] = 'P';
    // Terminator: '\n'
    p_cmd[len-1] = 0x0A;

    // Encode torque setpoint values
    encode_ascii85((const uint8_t*) pos_setpoint, 2*sizeof(float), &p_cmd[1], 2*enc_size_float);

    // TX data
    serial_port.write(p_cmd, len);
}


// This is the boost::thread t_feedback thread's function
void ODrive_base::get_feedback() {

    float data[6] = {0.0f};
    uint64_t t_sleep = control_rate_us/4;

    // Exit the loop when ODrive_base::stop() is called, so that this thread is joinable.
    while( axes_enable[0] || axes_enable[1] ) {

        if( serial_port.countDataAvailable > 0 ) {

            rx_bytes = serial_port.readUntil(rx_buf, packet_size, (char) 0x0A);      

            // We got a full packet from ODrive
            if( rx_bytes == packet_size || rx_bytes == packet_size+5 || rx_bytes == packet_size+10 ) {

                valid_packets++;
                // Decode serial data
                decode_ascii85(&rx_buf[0],               enc_size_uint32,  (uint8_t*) &evt, sizeof(uint32_t));
                decode_ascii85(&rx_buf[enc_size_uint32], 6*enc_size_float, (uint8_t*) data, 6*sizeof(float));
                // Update the circular buffers and do the average
                update_filter_pos(0, data[0]);  // axis 0
                update_filter_vel(0, data[1]); 
                update_filter_sea(0, data[2]); 
                update_filter_Iqm(0, data[2]/0.025060605);
                update_filter_pos(1, data[3]);  // axis 1
                update_filter_vel(1, data[4]);
                update_filter_sea(1, data[5]);
                update_filter_Iqm(1, data[5]/0.025060605);
                // Error
                if( rx_bytes >= packet_size + 5 ) {
                    uint32_t temp = 0u;
                    decode_ascii85(&rx_buf[packet_size],     enc_size_uint32,  (uint8_t*) &temp, sizeof(uint32_t));
                    error               = temp & 0x000000ff;
                    controller_error[0] = temp & 0x0000ff00 >> 8;
                    controller_error[1] = temp & 0x00ff0000 >> 16;
                    memset(rx_buf, 0, packet_size+6);
                }
                // Battery voltage
                else if( rx_bytes == packet_size + 10 ) {
                    decode_ascii85(&rx_buf[packet_size+5], enc_size_float, (uint8_t*) &vbat, sizeof(float));
                    memset(rx_buf, 0, packet_size+11);
                }
                else {
                    memset(rx_buf, 0, packet_size+1);
                }

                rx_bytes = 0;

            }
        
        }

        boost::this_thread::sleep_for(boost::chrono::microseconds{t_sleep});

    }

}


void ODrive_base::get_encoder_feedback(uint8_t axis, float* data) {

    data[0] = pos[axis];    // position
    data[1] = vel[axis];    // velocity

}


float ODrive_base::read_encoder_pos(uint8_t axis) {

    return pos[axis];

}


float ODrive_base::read_encoder_vel(uint8_t axis) {

    return vel[axis];

}


float ODrive_base::read_motor_current(uint8_t axis) {

    return Iqm[axis];

}


float ODrive_base::read_vbus_voltage() {

    std::string ans = read_property("vbus_voltage");

    // Parse VBUS voltage from string
    float vvbus = 0.0f;
    if( sscanf(ans.c_str(), "%f", &vvbus) != 1 ) {
        vvbus = -1.0;
    }

    return vvbus;

}
int immode = 0;

float ODrive_base::position_control(uint8_t axis)
 {

    // Avoid runtime errors due to bad user code in the control loop
    assert(axis >= 0 && axis < 2);

    /**
     * The high-level control system can vary the Position Controller parameters (K_p, K_d, and K_i)
     * and the reference position (theta_ref).
     * These variables are updated reading from shared memory (they are not directly shared)
     * The control master (high-level controller) writes into shared memory and notifies a thread of
     * ODrive-Master, which updates the parameters in the position_controller struct.
    **/

   
    float _pos = pos[axis];
    float _vel = vel[axis];

    
    error_pos[axis] = position_controller[axis].theta_ref - _pos;
    float error = position_controller[axis].theta_ref - _pos;
    
    error_integral[axis] = error_integral[axis]+error_pos[axis]*dt;
    if (abs(error_pos[axis])<0.005){
        steady_state[axis] += 1;
    }
    else {
        steady_state[axis] = 0;
    }
    if (steady_state[axis]>=100)
    {
        antiwindup[axis] = true;
    }

    if (antiwindup[axis] == true)
    {
        error_integral[axis] = 0;
        antiwindup[axis] = false;
    }
    // if (abs(error)<0.003) 
    // {
    //     error = 0;
    // }
    float tau = 0;
    if (assistance<0.5) {
        if (error_integral_reset[axis] == false){
            error_integral_reset[axis] = true;    
            error_integral[axis] = 0;
        }
        if ((mode<1.5)&&(mode>0.5)){
            tau = 1.7897*error_pos[axis]+ 0.1914*(position_controller[axis].vel_ref-_vel)+0.7746*error_integral[axis]+0.084305*((error_pos[axis]>0.004)-(error_pos[axis]<-0.004));
            
        }
        else{
            tau = 1.7897*error_pos[axis]+ 0.1914*(position_controller[axis].vel_ref-_vel)+0.7746*error_integral[axis]+0.14*tanh(_vel/0.06);
        }
        
        // if ((tau<0.06)&&(tau>-0.06))
        // {
        //     tau = 0;
        // }
        if ((abs(position_controller[0].filtered_ext_torque)<0.028)&&(mode<0.5)){
            tau = 0;
        }

        if (cmode ==2){
            tau -=  (2*9.81*0.2*sin((_pos-position_controller[axis].startpos)))/(60);
            tau -=  ((0.5*9.81*0.2)/60) * ((axis==0)-(axis==1));
            if (_vel>1.6){
                tau += 0.1*(1.6-_vel);
            }
            if (_vel<-1.6){
                tau += 0.1*(-1.6-_vel);
            }
        }

    }
    else if (assistance>0.5){
        if (cmode == 1)
        {
            tau = 1.7897*error_pos[axis]+ 0.1914*(position_controller[axis].vel_ref-_vel)+0.7746*error_integral[axis]+0.094305*((error_pos[axis]>0.004)-(error_pos[axis]<-0.004));

            tau -=  (1.5*9.81*0.2*sin(0.7*(_pos-position_controller[axis].startpos)))/(60);
        
            error_integral_reset[axis] = false;
        
            if (_vel>1.4){
                tau += 0.4*(1.4-_vel);
            }
            if (_vel<-1.4){
                tau += 0.4*(-1.4-_vel);
            }
            
        }
        else if (cmode ==2)
        {
            if (error_integral_reset[axis] == false)
            {
                error_integral_reset[axis] = true;    
                error_integral[axis] = 0;
            }

            tau = 1.7897*error_pos[axis]+ 0.1914*(position_controller[axis].vel_ref-_vel)+0.7746*error_integral[axis]+0.14*tanh(_vel/0.06);
            if ((abs(position_controller[0].filtered_ext_torque)<0.028)&&(mode<0.5))
            {
                tau = 0;
            }

            tau -=  (6*9.81*0.2*sin(0.7*(_pos-position_controller[axis].startpos)))/(60);
            
            tau -=  ((3*9.81*0.2)/60) * ((axis==0)-(axis==1));
            error_integral_reset[axis] = false;
        
            if (_vel>0.8){
                tau += 0.15*(0.8-_vel);
            }
            if (_vel<-0.8){
                tau += 0.15*(-0.8-_vel);
            }
        }

    }

    
    if ((tau>=torque_limits[axis])){
        error_integral[axis] = error_integral[axis]-error_pos[axis]*dt;
        tau = torque_limits[axis];
    }
    if ((tau<=-torque_limits[axis])){
        error_integral[axis] = error_integral[axis]-error_pos[axis]*dt;
        tau = -torque_limits[axis];
    }
    tau = torque_limit_clip(tau, torque_limits[axis]);
    
    return tau;

}

float ODrive_base::friction_comp(uint8_t axis) {

    // Feed-forward term to be computed and returned from this function
    float tau_comp = 0.0f;

    // Get the current position and velocity ("fix" them)
    float _pos = pos[axis], _vel = vel[axis];
    
    // Get velocity in rad/s at the output
    _vel /= (2.0f*M_PI);            // rounds/s -> rad/s        (motor side)
    _vel /= reduction_ratio;        // reduction ratio (30:1)   (output side)

    // Get position in turn at the input (keep it between -1 and +1 for the sin())
    _pos = fmod(_pos, 1.0f);

    // Limit the velocity
    if( fabs(_vel) > friction_comp_params.vel_limit )  {
        _vel = copysignf(friction_comp_params.vel_limit, _vel);
    }

    // If the absolute value of the velocity goes above a threshold
    // start computing the integral
    // TODO: do this with the encoder position for reduced noise...
    if( fabs(_vel) > friction_comp_params.vel_dead_band ) {
        friction_comp_params.ivel += _vel;
    }
    else {
        friction_comp_params.flag = false;
        friction_comp_params.steady_state = 0;
        friction_comp_params.ivel = 0.0f;
    }

    // When the integral of the velocity goes above a certain threshold, 
    // we start compensating friction
    if( friction_comp_params.ivel > friction_comp_params.vel_pos || friction_comp_params.ivel < friction_comp_params.vel_neg ) {
        friction_comp_params.flag = true;
    }
    if( friction_comp_params.flag ) {
        
        // Detect the steady-state when the integral goes above the higher threshold (randomly selected...)
        if( fabs(friction_comp_params.ivel) > 3.5f ) {
            friction_comp_params.ivel = 0.0f;
            friction_comp_params.steady_state = 1;
        }
        // Compute the compensation torque with the exponential law (stiction)
        tau_comp = copysignf(1.0f, _vel)*(0.0350 + (0.0600 - 0.0350)*exp(-fabs(_vel)/0.0150));
        // If we reach the steady state, provide a linear torque
        // This avoids increasing compensation torque when we are slowing down WITHOUT the need
        // to compute the acceleration (which is too much noisy with this setup)
        
        // Exponential decay with increasing speed: more torque to "win" stiction
        tau_comp = (1-friction_comp_params.steady_state)*(tau_comp);

        // Linear steady-state term
        tau_comp += copysignf(friction_comp_params.steady_state, _vel)*(0.0435);
        
        // Scale and adjust
        tau_comp = (tau_comp - 0.0035)*friction_comp_params.k_t;

    }
    
    // limit for safety
    if( fabs(tau_comp) > 0.2f ) tau_comp = copysignf(0.2f, _vel);
    return tau_comp;

}


// Override this implementation in the derived class
float ODrive_base::reducer_comp(uint8_t axis) {

    return 0.0f; 

}


void ODrive_base::save_logfile(std::string fname) {

    if( logging == false ) {
        std::cout << "FIle logging is disabled!\n";
        return;
    }

    // Log to file
    if( debug == true ) {
        logfile.open("../data/debug.csv");
        if( !logfile.is_open() ) perror("Cannot open debug.csv");
    }
    else{
        if( fname.size() == 0 ) {
            auto now = std::time(nullptr);
            std::stringstream sname;
            sname << "../data/log_" << std::put_time(std::gmtime(&now), "%d_%m_%Y_%H_%M_%S") << ".csv";
            logfile.open(sname.str());
        }
        else {
            fname.insert(0, "../data/");
            logfile.open(fname);
            if( !logfile.is_open() ) perror("Cannot open logfile");
        }
    }

    logfile << "n_evt,assistance,torque_cmd_0,pos_0,est_torque_0,torque_0,current_0,qobs_0,pos_1,est_torque_1,torque_1,current_1,error_integral_0,error_integral_1,ext_torque,filtered_ext_torque,vel_ref_0,filtered_vel_0,vel_1\n";
    char buf[1024];
    for( size_t i = 0; i < loop_counter; i++ ) {
        sprintf(buf, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", evt_log[i],assistance_log[i], torque_cmd_log[0][i], pos_log[0][i], obs_tor_log[0][i], torque_log[0][i],current_log[0][i], qobs_log[1][i], pos_log[1][i], vel_log[0][i], torque_log[1][i],current_log[1][i],error_integral_log[0][i],error_integral_log[1][i],ext_tor_log[0][i],filt_ext_tor_log[0][i],vel_ref_log[0][i],filt_vel_log[0][i],vel_log[1][i]);
        logfile << buf;
    }
    logfile.close();

}



// ----------- Private methods

void ODrive_base::update_filter_pos(uint8_t axis, float value) {

    // Convert position from motor-side [turns] to output [rad]
    value = turn2rad(value) / reduction_ratio;

    b_pos[axis].push_back(value);
    pos[axis] = std::accumulate(b_pos[axis].begin(), b_pos[axis].end(), 0.0f) / b_pos[axis].size();

}


void ODrive_base::update_filter_vel(uint8_t axis, float value) {

    // Convert position from motor-side [turn/s] to output [rad/s]
    value = turn2rad(value) / reduction_ratio;

    b_vel[axis].push_back(value);
    vel[axis] = std::accumulate(b_vel[axis].begin(), b_vel[axis].end(), 0.0f) / b_vel[axis].size();

}


void ODrive_base::update_filter_Iqm(uint8_t axis, float value) {

    b_Iqm[axis].push_back(value);
    Iqm[axis] = std::accumulate(b_Iqm[axis].begin(), b_Iqm[axis].end(), 0.0f) / b_Iqm[axis].size();

}


void ODrive_base::update_filter_sea(uint8_t axis, float value) {

    b_sea[axis].push_back(value);
    sea[axis] = std::accumulate(b_sea[axis].begin(), b_sea[axis].end(), 0.0f) / b_sea[axis].size();

}



void ODrive_base::compute_benchmark(void) {

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

    std::cout << "----- BENCHMARK results\n";
    std::cout << " min | avg +/- std | max : " << std::to_string(min*1000) << " | " << std::to_string(avg*1000) << " +/- " << std::to_string(stdev*1000) << " | " << std::to_string(max*1000) << " [ms] \n";
    std::cout << " # loops <= cycle_time: " << std::to_string(count_below_cycle_time) << " (" << std::to_string((100*count_below_cycle_time/this->loop_counter)) << " %)\n";
    std::cout << " RX valid packets: " << std::to_string(valid_packets) << std::endl;

}


void ODrive_base::__control_loop() {

    // Reschedule the timer
    timer.expires_at(timer.expires_at() + interval);
    // Posts the timer event
    timer.async_wait(boost::bind(&ODrive_base::__control_loop, this));

    // Run the user-defined control loop 
    control_loop();

    // LOG 
    for( uint8_t i = 0; i < 2; i++ ) {
        torque_cmd_log[i].push_back(torque_cmd[i]);                     // Torque command
        pos_log[i].push_back(pos[i]);                                   // Encoder position
        vel_log[i].push_back(vel[i]);                                   // Encoder velocity
        obs_tor_log[i].push_back(position_controller[i].t_obs);
        ext_tor_log[i].push_back(position_controller[i].ext_torque);
        filt_ext_tor_log[i].push_back(position_controller[i].filtered_ext_torque);
        filt_vel_log[i].push_back(position_controller[i].filtered_vel);
                                  
        torque_log[i].push_back(sea[i]);           
        current_log[i].push_back(Iqm[i]);                   // Estimated motor current
        theta_ref_log[i].push_back(position_controller[i].theta_ref);  // Position Control reference
        vel_ref_log[i].push_back(position_controller[i].vel_ref);
        qobs_log[i].push_back(position_controller[i].qobs);            // Observer Reference
        //err_tor_log[i].push_back(position_controller[i].err_tor);
        error_integral_log[i].push_back(error_integral[i]);
    }
    evt_log.push_back(evt);
    assistance_log.push_back(assistance);
    

    if( benchmark ) {        
        t_loops_benchmark.push_back(std::chrono::high_resolution_clock::now());
    }

    // Update loop counter  
    loop_counter++;

}
