#pragma once

#include "BufferedAsyncSerial.h"
#include "ascii85.h"
#include "logging.h"
#include "shmem.h"

#include "ODrive_base.h"

#define SHMEM_RX_LEN            4
#define SHMEM_RX_PACKET_SIZE    SHMEM_RX_LEN*sizeof(float)
#define SHMEM_TX_LEN            2
#define SHMEM_TX_PACKET_SIZE    SHMEM_TX_LEN*sizeof(float)
#define SHMEM_RX_OFFSET         0
#define SHMEM_TX_OFFSET         SHMEM_RX_PACKET_SIZE

const char opt[] = "[k] Position      [z] Zero Torque    [f] Friction comp  [r] Reducer comp ";

/**
 * \class ODrive
 * \brief C++ interface for ODrive (dual-axis motor driver and controller) for Linux developed for ELIO (Low-Back Exo)
 * 
 * This class is based on ODrive_base and expands its functionality (e.g., shared-memory, console logging, etc)
 * 
 */
class ODrive : public ODrive_base
{

    using ODrive_base::ODrive_base;

public:
    virtual ~ODrive();
    /** @brief Console logger class instance */
    logger console_log;
    // CHIRP values
    float chirp_f0 = 1.00;
    float chirp_f1 = 10.00;
    float chirp_tf = 120;
    float chirp_Ai = 0.5;
    float chirp_A0 = 0.0;
    /** @brief Flag for shared memory */
    bool use_shmem = false;
    /** @brief Semaphore to sync with higher-level control software */
    sem_t* sem_sync;
    /** @brief Inverted direction flags for ELIO */
    int8_t inverted[2] = {-1, +1};
    


private:
    /** @brief Verbosity level */
    uint8_t verbose = 0;
    /** @brief Variable to store user input from the terminal (modality selection) */
    volatile char in;
    /** @brief Flag to update the screen */
    volatile char update_screen = ERR;
    /** @brief Console logger thread */
    std::thread* t_logger;
    /** @brief User-input thread */
    std::thread* t_input;
    /** @brief Shared memory instance */
    shared_memory shmem;
    /** @brief Shared-memory RX thread */
    std::thread* t_shmem_rx;
    /** @brief Shared-memory TX thread */
    std::thread* t_shmem_tx;
    /** @brief Semaphores to be used with shared memory to implement critical sections */
    std::vector<sem_t*> sems;
    /** @brief Flag for the "input" channel of shared memory */
    volatile bool rx_ready = false;
    /** @brief Flag for the "output" channel of shared memory */
    volatile bool tx_ready = false;


public:
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
    std::string read_property(const std::string& property) override;

    /**
     * Read a property from ODrive using the ASCII protocol asynchronously.
     * This function asks ODrive for the specified property and returns immediately.
     * The result is stored in this->rx_msg and may be accessed later, before any other
     * request is sent to ODrive (otherwise, the reply will be lost).
     * \param property: (string) property to read from ODrive
    */
    void async_read_property(const std::string& property) override;

    /**
     * @brief Init ODrive specifying serial port and axes
     * 
     * @param dev_name serial port name 
     * @param reboot flag to reboot ODrive
     */
    void init(std::string& dev_name, bool reboot=false) override;

    /**
     * @brief Init ODrive using the serial port specified in the constructor
     * 
     * @param reboot flag to reboot ODrive
     */
    void init(bool reboot=false) override;

    /**
     * Checks the battery voltage and stops if it is below the safety threshold (i.e., 13 V)
    */
    void battery_check(void);

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
    void enable(uint8_t control_mode, uint8_t axis) override;

    /**
     * Control loop function. This function is executed at every interrupt of the timer object.
     * This is a virtual function that needs to be implemented at user level.
     * Define somewhere in your application code the following: 
     *   void ODrive::control_loop()
    */    
    void control_loop() override;

    /**
     * @brief Compute the torque for the CHIRP test for the selected axis
     * 
     * @param axis : axis 0 or 1
     * @return float : computed torque to be used as command
     */
    float chirp(uint8_t axis);

    /**
     * Set verbosity level (0 for non verbose)
     * \param verbose_level: level of verbosity
     * \return the value set (or the old value if setting was illegal)
    */
    uint8_t set_verbose(uint8_t verbose_level);

    /**
     * Stop ODrive and go to idle. Both axis are stopped.
     * TODO: add stopping for only the specified axis
    */
    void stop() override;

    /**
     * @brief Compute the reducer compensation term for the torque command 
     * 
     * @param axis : axis 0 or 1
     * @return float : feedforward compensation torque [Nm]
     */
    float reducer_comp(uint8_t axis) override;

    /**
     * @brief Save the logfile if logging = true
     * 
     */
    void save_logfile(std::string fname = "") override;

    /**
     * @brief Wait for user input before starting closed-loop control
     * 
     */
    void wait_for_start();

    /**
     * @brief Quit and exit with the specified exit code
     * 
     * @param exit_code exit code for 'exit' system function
     */
    void quit(uint8_t exit_code = 0);

    /**
     * @brief Init the named shared_memory instance
     * 
     * @param name 
     */
    bool shmem_init(const std::string name);


private:    
    /**
     * @brief Callback function run by the t_logger thread (\see ODrive::t_logger)
     * 
     */
    void f_consoleLogging();
    /**
     * @brief Callback function run by the t_input thread (\see ODrive::t_input)
     * 
     */
    void f_getUserInput();
    /**
     * @brief Callback function run by the t_shmem_rx thread for shared memory management; \see ODrive::t_shmem_rx
     * 
     */
    void f_shmem_rx();
    /**
     * @brief Callback function run by the t_shmem_tx thread for shared memory management; \see ODrive::t_shmem_txfor
     * 
     */
    void f_shmem_tx();
    /**
     * @brief Wrapped initialization function called by the polymorphic function \see ODrive::init
     * 
     * @param reboot 
     */
    void __do_init(bool reboot) override;
    /**
     * @brief Compute and show benchmark for serial communication and loop frequency accuracy (if enabled setting \see ODrive_base::benchmark)
     * 
     */
    void compute_benchmark() override;
    

};  