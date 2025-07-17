#pragma once

#include "ascii85.h"
#include "BufferedAsyncSerial.h"

#include <string>
#include <iostream>
#include <fstream>
#include <thread>
#include <boost/circular_buffer.hpp>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <boost/thread/scoped_thread.hpp>
#include <boost/chrono.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <numeric>

#define ZERO_TORQUE_MODE    0
#define POSCONTROL_MODE      1

// Useful inline functions for deg<-->rad conversion
inline float deg2rad(float x)  { return (x/180.0f)*M_PI; }
inline float rad2deg(float x)  { return (x/M_PI)*180.0f; }
// Useful inline functions for turn<-->deg conversion
inline float turn2deg(float x) { return (x/360.0f); }
inline float deg2turn(float x) { return (x*360.0f); }
// Useful inline functions for turn<-->rad conversion
inline float turn2rad(float x) { return (x*(2.0*M_PI)); }
inline float rad2turn(float x) { return (x/(2.0*M_PI)); }
// Torque limiting
static inline float torque_limit_clip(float val, float lim) {
    return fabs(val) > lim ? copysignf(1.0f, val)*lim : val;
}

const size_t  enc_size_uint32 = ascii85_get_max_encoded_length(sizeof(uint32_t));
const size_t  enc_size_float  = ascii85_get_max_encoded_length(sizeof(float));
const uint8_t packet_size     = 35; // bytes, no terminator

/**
 * \class ODrive_base
 * \brief C++ interface for ODrive (dual-axis motor driver and controller) for Linux
 * 
 * This is the base class that can either be inherited by upper classes (e.g., ODrive) 
 * or included into other hierarchical structures, 
 * as well as used as standalone
 * 
 */
class ODrive_base
{
public:
    /**
     * @brief Empty constructor, init with default values
     * 
     */
    ODrive_base() : devname("/dev/null"),
                    control_rate(1000.f),
                    dt(1.0/control_rate),
                    dt_us(1000000*dt),
        interval(boost::posix_time::microseconds(static_cast<long>(dt_us))),
                    timer(timer_io, interval)
    {

        __init_buffers();

    }

    /**
     * @brief Construct a new ODrive object specifying the control rate 
     * 
     * @param control_rate : frequency of the control loop [Hz]
     */
    ODrive_base(const float control_rate) : devname("/dev/null"),
                                            control_rate(control_rate),
                                            dt(1.0/control_rate),
                                            dt_us(1000000*dt),
                                            interval(boost::posix_time::microseconds(static_cast<long>(dt_us))),
                                            timer(timer_io, interval)
    {
    
        __init_buffers();

    }

    /**
     * @brief Construct a new ODrive object and create the serial object
     * 
     * @param devname (string) serial port name (default: /dev/ttyACM0)
     * @param control_rate (float) frequency of the control loop [Hz]
     */
    ODrive_base(const std::string& devname, const float control_rate) : devname(devname), 
                                                                        serial_port(devname, 115200), 
                                                                        control_rate(control_rate), 
                                                                        dt(1.0/control_rate), 
                                                                        dt_us(1000000*dt), 
                                                                        interval(boost::posix_time::microseconds(static_cast<long>(dt_us))),
                                                                        timer(timer_io, interval)
    {

        __init_buffers();

    }

    /**
     * @brief Destroy the ODrive object and close the serial port
     * 
     */
    virtual ~ODrive_base();

    // -----------------------------------------------------
    // Parameters
    // -----------------------------------------------------
    // Reduction ratio
    float reduction_ratio = 30.0f;
    // Motor torque constant
    float K_t = 0.02506f;

    // -----------------------------------------------------
    // Serial port
    // -----------------------------------------------------
    // Serial port name
    std::string devname = "/dev/ttyACM0";
    // Serial port object - using boost::serial_port_base
    BufferedAsyncSerial serial_port;
    // Serial receive buffer
    uint8_t rx_buf[512] = {0x00};
    // Number of bytes received from the serial port
    volatile ssize_t rx_bytes = 0;
    // Flag that indicates a pending asynchronous read operation
    // TODO implement this or remove this flag
    volatile bool async_property_read_pending = false; 

    // -----------------------------------------------------
    // System
    // -----------------------------------------------------
    // Battery voltage
    float vbat = 0.0f;
    // System error code
    uint8_t error = 0x00u;
    // Per-axis controller error code (from odrv.axis.controller)
    uint8_t controller_error[2] = {0x00u};
    // Frequency of the control loop
    float control_rate = 1000.0f;
    // Control rate expressed in microseconds
    uint64_t control_rate_us = 1000000/control_rate;
    // Control time steps [s] and [us]
    float dt, dt_us;
    // Loop counter (32-bit modulo)
    volatile uint32_t loop_counter = 0;
    // Store ODrive's n_evt_control_loop
    uint32_t evt = 0;
    // Count the number of valid packets received from serial; this can compared to the number of loops for benchmarking
    uint32_t valid_packets = 0;
    // Flag for the infinite loop: while(run), stopped by ODrive::stop()
    volatile bool run = true;
    // Stop the infinite loop after the specified time [s]
    float t_stop = 120; // default to INFINITY ?
    // Dry-run flag: does not set closed-loop control
    bool DRY_RUN = false;
    // Safe-mode: ignore every torque command (always set 0.0f)
    bool SAFE_MODE = false;
    // Benchmarking flag: compute loop times and do some stats
    bool benchmark = true;
    // Logging to file flag
    bool logging = false;
    // Debug flag
    bool debug = false;
    // SEA Torque Sensor flag
    bool sea_torque_enable = false;
    // Reboot status flag
    bool reboot_ok = false;
    // Watchdog enable flag: ALWAYS ENABLE FOR SAFETY!
    bool watchdog_enable[2] = {false, false};

    // -----------------------------------------------------
    // Controller
    // -----------------------------------------------------
    // Actual position vector for the 2 axis (motor side, [turn])
    float pos[2] = {0.0f};
    // Actual velocity vector for the 2 axis
    float vel[2] = {0.0f};
    // Actual current measurement from ODrive's current control for the 2 axis
    float Iqm[2] = {0.0f};
    // Actual SEA torque estimate
    float sea[2] = {0.0f};
    // Observer Position
   // float qobs[2] = {0.0f};
    // Commanded torque
    float torque_cmd[2] = {0.0f,0.0f};
    // Torque limits
    float torque_limits[2] = {0.5f, 0.5f};
    // Flags set to true by ODrive::enable() if successful
    bool axes_enable[2] = {axis0_enabled, axis1_enabled};
    // Mid-level working mode for the controller
    volatile uint8_t working_mode = ZERO_TORQUE_MODE;
    // Flags to enable friction compensation
    volatile bool fcomp[2] = {false, false};
    // Flags to enable reducer compensation
    volatile bool rcomp[2] = {false, false}; 
    // Position Controller parameters
    struct poscontrol_params {
        float K_p = 1.0f;       // proportional gain
        float K_d = 0.0f;       // derivative gain
        float K_i = 0.0f;       // integral gain
        float theta_ref = 0.0f;
        float qobs = 0.0f;
        float err_tor = 0.02f;
        float t_obs = 0.0f;
        float ext_torque = 0.0f;
        float filtered_ext_torque = 0.0f;
        float vel_ref = 0.0f;
        float filtered_vel = 0.0f;
        float startpos = 0.0f;
        // Set-point
    } position_controller[2];
    float assistance = 0.0f;
    float mode  = 0.0f;
    float cmode = 2.0f;
    float error_integral[2] = {0.0f};
    float error_pos[2] ={0.0f};
    int steady_state[2] = {0, 0};
    bool error_integral_reset[2] = {false, false};
    bool antiwindup[2] = {false, false};
    
    // Actual load cell (TRT) reading from ODrive's ADC
    float trt = 0.0;

    // -----------------------------------------------------
    // Logging
    // -----------------------------------------------------
    // Logfile object for logging to file (if necessary)
    std::ofstream logfile;
    // Logging vectors for the variables of interest
    std::vector<float> assistance_log, torque_cmd_log[2], pos_log[2], vel_log[2], torque_log[2], theta_ref_log[2],qobs_log[2],err_tor_log[2], obs_tor_log[2],error_integral_log[2], current_log[2], ext_tor_log[2], filt_ext_tor_log[2], vel_ref_log[2], filt_vel_log[2];
    std::vector<uint32_t> evt_log;

// NOTE 'protected' methods and variables are effectively private but accessible via class inheritance (by upper classes)
protected:
    // -----------------------------------------------------
    // System
    // -----------------------------------------------------
    // Benchmarking time-point vector
    std::vector<std::chrono::high_resolution_clock::time_point> t_loops_benchmark;
    // Thread that runs ODrive_base::get_feedback() at 4 kHz (lower-priority, higher-frequency thread)
    std::thread* t_feedback;
    // Thread for the deadline timer
    boost::thread* worker;
    
    // -----------------------------------------------------
    // Controller
    // -----------------------------------------------------
    // Axes enable flags, accessible via the array axes_enable[2] (public)
    bool axis0_enabled = false, axis1_enabled = false;
    // Circular buffers for online filtering
    boost::circular_buffer<float> b_pos[2];  // Position 
    boost::circular_buffer<float> b_vel[2];  // Velocity  
    boost::circular_buffer<float> b_Iqm[2];  // Current  
    boost::circular_buffer<float> b_sea[2];  // SEA TorqueSensor 
    // Control loop implementation with boost::asio
    boost::asio::io_service timer_io;
    boost::shared_ptr<boost::asio::io_service::work> work = boost::make_shared<boost::asio::io_service::work>(timer_io);
    boost::posix_time::microseconds interval;  
    // Boost deadline_timer
    boost::asio::deadline_timer timer;
    // Friction compensation
    struct fc_params{
        const float vel_limit = 0.15;           // roughly 30 round/s at the input
        const float vel_dead_band = 0.0018;      
        const float vel_pos = +6.0f*vel_dead_band;
        const float vel_neg = -6.0f*vel_dead_band;
        float vel_upper_th = vel_dead_band, vel_lower_th = -vel_dead_band;
        const float Cp = 0.030, Dp = 0.001, Sp = 0.055, Ws_p = 0.010;
        float k_f = 1.00, k_t = 0.9;
        float ivel = 0.0f;
        bool flag = false;
        uint8_t steady_state = 0;
    } friction_comp_params;


private:
    // Private vars


public:
    /** 
     * Send a command to ODrive over serial using the ASCII protocol.
     * The function returns immediately and does not echo ODrive's response (if any)
     * \param command: (string) command to be sent; 
    */
    void send_command(const std::string& command);

    /**
     * Read a property from ODrive using the ASCII protocol. 
     * The function asks ODrive for the specified property and returns the reply as 
     * an std::string. The reply is also echoed if verbose > 0
     * This function waits 10 ms for the TX/RX communication, so it should *NOT*
     * be used inside the control loop, only during the init phase. For the control loop,
     * use ODrive::async_read_property() instead.
     * \param property: (string) property to read from ODrive 
     * \return value from ODrive 
    */
    virtual std::string read_property(const std::string& property);

    /**
     * Read a property from ODrive using the ASCII protocol asynchronously.
     * This function asks ODrive for the specified property and returns immediately.
     * The result is stored in this->rx_msg and may be accessed later, before any other
     * request is sent to ODrive (otherwise, the reply will be lost).
     * \param property: (string) property to read from ODrive
    */
    virtual void async_read_property(const std::string& property);

    /**
     * @brief Init ODrive specifying serial port and axes
     * 
     * @param dev_name serial port name 
     * @param reboot flag to reboot ODrive
     */
    virtual void init(std::string& dev_name, bool reboot=false);

    /**
     * @brief Init ODrive using the serial port specified in the constructor
     * 
     * @param reboot flag to reboot ODrive
     */
    virtual void init(bool reboot=false);

    /**
     * @brief Enable the specified axis with the desired control mode.
     * 
     * @param control_mode: control mode the for axis: TORQUE_CONTROL (0), VELOCITY_CONTROL (1), Or POSITION_CONTROL (2)
     * @param axis: axis to enable: 0 for axis0, 1 for axis1, 2 for both axes
     * 
     * The enable() function is actually just a wrapper for the private method ODrive::__do_enable(), that creates
     * a real-time thread that handles the boost::deadline_timer object, to run the control loop. This method also
     * sets the thread priority to 99 (real time).
     * 
     */
    virtual void enable(uint8_t control_mode, uint8_t axis);

    /**
     * Stop ODrive and go to idle. Both axis are stopped.
     * TODO: add stopping for only the specified axis
    */
    virtual void stop();

    /**
     * @brief Read the VBUS voltage from ODrive using ODrive::read_property()
     * 
     * @return float reading of VBUS voltage converted to Volts
     */
    float read_vbus_voltage(void);

    /**
     * Control loop function. This function is executed at every interrupt of the timer object.
     * This is a virtual function that needs to be implemented at user level.
     * Define somewhere in your application code the following: 
     *   void ODrive::control_loop()
    */    
    virtual void control_loop() = 0;  // pure virtual function

    /**
     * Send a torque command to a single axis
     * \param axis: target motor axis
     * \param torque_setpoint: desired torque at the motor side [Nm] 
    */
    void torque_command(uint8_t axis, float torque_setpoint);

    /**
     * @brief Dual-axis torque command request to ODrive
     * \param torque_setpoint: desired torque values at the motor side [Nm]
     * 
     */
    void torque_command(float* torque_setpoint);

    /**
     * Send a velocity command to ODrive. 
     * \param axis: target motor axis
     * \param pos_setpoint: desired velocity at the motor side [turn/s]
    */
    void vel_command(uint8_t axis, float vel_setpoint);

    /**
     * @brief Dual-axis velocity command request to ODrive
     * 
     * @param vel_setpoint: desired velocity values at the motor side [turn/s]
     */
    void vel_command(float* vel_setpoint);

    /**
     * Send a position command to ODrive. 
     * \param axis: target motor axis
     * \param pos_setpoint: desired position at the motor side [turns]
    */
    void pos_command(uint8_t axis, float pos_setpoint);

    /**
     * @brief Dual-axis position command request to ODrive
     * 
     * @param pos_setpoint: desired position at the motor side [turns] 
     */
    void pos_command(float* pos_setpoint);

    /**
     * TODO: update the documentation of this function
     * Read the feedback that is automatically sent in response to a torque command.
     * This requires the customized ASCII protocol firmware. The feedback is a 12-byte
     * array with encoder position (4) + encoder velocity (4) + measured motor current (4)
     * plus a stop byte ('\n').
     * Data is also updated in the circular buffers for movmean filtering; the filtered values
     * are stored in this->pos[axis], this->vel[axis], and this->Iqm[axis]
     * \param axis: target motor axis
     * \param data: float pointer to a user-defined buffer: it must be 12-bytes or more!
     * 
    */
    void get_feedback();

    /**
     * @brief Get encoder feedback (pos and vel) for the specified axis
     * 
     * @param axis 
     * @param data 
     */
    void get_encoder_feedback(uint8_t axis, float* data);

    /**
     * @brief Read the encoder position for the specified axis
     * 
     * @param axis 
     * @return float 
     */
    float read_encoder_pos(uint8_t axis);
    
    /**
     * @brief Read the encoder velocity for the specified axis
     * 
     * @param axis 
     * @return float 
     */
    float read_encoder_vel(uint8_t axis);

    /**
     * @brief Read the measured motor current for the specified axis
     * 
     * @param axis 
     * @return float 
     */
    float read_motor_current(uint8_t axis);

    /**
     * @brief Position control : Computed Torque = Kp*(pos_ref - pos) - Kd*vel + Ki*(pos_ref - pos)*dt
     * 
     * @param axis : axis 0 or 1
     * @return float : torque setpoint
     */
    virtual float position_control(uint8_t axis);
    
    /**
     * @brief Compute the friction compensation term for the torque command 
     * 
     * @param axis : axis 0 or 1
     * @return float : feedforward compensation torque [Nm]
     */
    virtual float friction_comp(uint8_t axis);

    /**
     * @brief Compute the reducer compensation term for the torque command 
     * 
     * @param axis : axis 0 or 1
     * @return float : feedforward compensation torque [Nm]
     */
    virtual float reducer_comp(uint8_t axis);

    /**
     * @brief Save the logfile if logging = true
     * 
     */
    virtual void save_logfile(std::string fname = "");


protected:
    // Initialization subroutine common to all ODrive::init() implementations
    virtual void __do_init(bool reboot);

    // Actual enable function that never exits after launching a new thread for the boost::io service
    void __do_enable(uint8_t control_mode, uint8_t axis);
    
    // Compute benchmark for the control loops
    virtual void compute_benchmark(void);

    // Init the circular buffers
    void __init_buffers(void);

    // Open the serial port
    void __do_open_serial(void);

    // Reboot function
    void __do_reboot(bool reboot);

    /**
     * @brief Wrapped function that runs at the deadline of the deadline_timer and calls the user-defined ODrive::control_loop()
     * 
     */
    void __control_loop(void);

private:
    // Update filter the control variables with the circular buffer
    void update_filter_pos(uint8_t axis, float value);
    void update_filter_vel(uint8_t axis, float value);
    void update_filter_Iqm(uint8_t axis, float value);
    void update_filter_sea(uint8_t axis, float value);

};  
