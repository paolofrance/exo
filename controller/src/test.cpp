#include "controller/project.h"
#include "controller/control_parameters.h"

#include <shmem.hpp>
#include <unistd.h>
#include <assert.h>
#include <math.h>
#include <cmath>
#include <Eigen/Dense>
#include <iomanip>
#include <vector>
#include <deque>
#include <iostream>

#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>

using namespace Eigen;

struct timestampstrc
{
    int year, month, day, hour, minute;
    float second;
};

#define SHM_KEY 12345                                   // shared memory key
#define SHM_SIZE (sizeof(bool) + sizeof(timestampstrc)) // shared memory size

volatile bool run_ = true;

// Variable set by the main thread at the specified control frequency
volatile bool feedback_available_ = false;
// Variable set by the shared-memory handler thread to notify when there's feedback data available from ODrive
volatile bool impedance_update_ = false;

volatile uint8_t verbose_ = true;

// Initial position reference (theta_ref_)
float theta_ref_[2] = {0.0f, 0.0f};
float vel_ref_[2] = {0.0f, 0.0f}; // Added this Velocity Reference just in case
// Measured position (theta_m) from the encoders
float theta_meas_[2];
float start_pos_[2];

// Measured velocity from the encoders
float vel_meas_[2];
float filtered_vel_meas_[2];
// Estimated motor torque (tau_m) from the ODrive (estimated as Kt * measured motor current)
float tau_motor_meas_[2];
// observed torque by EKF (motor torque + external torque)
float tau_hat_[2];
// external torque applied by the subject
float ext_torque_[2];
float filtered_ext_torque_[2];

// Commanded Torque from PID
float tau_cmd_[2] = {0.0f, 0.0f};
// float tau_hat_[2] = {0.0f};
float err_tor[1] = {0.0f};
float e[1] = {0.0f};
float assistance_level_[1];
float home_pos_[2] = {0.0f, 0.0f}; // rest position
float mode_[1] = {0.0f};       // mode 1 - Predefined Trajectory
float cmode_[1] = {2.0f};      // mode of the impedance control
bool first_loop_ = true;           // boolean used to save the staring position

/// @brief  Handler function for the t_impedance thread; "output" channel
void f_impedance(shared_memory &shmem)
{
    float temp[17 * sizeof(float)];
    while (run_)
    {
        if (impedance_update_)
        {
            impedance_update_ = false;
            memcpy(&temp[0], ext_torque_, 2 * sizeof(float));
            memcpy(&temp[2], theta_ref_, 2 * sizeof(float));
            memcpy(&temp[4], tau_hat_, 2 * sizeof(float));
            memcpy(&temp[6], filtered_ext_torque_, 2 * sizeof(float));
            memcpy(&temp[8], vel_ref_, 2 * sizeof(float));
            memcpy(&temp[10], assistance_level_, sizeof(float));
            memcpy(&temp[11], filtered_vel_meas_, 2 * sizeof(float));
            memcpy(&temp[13], mode_, sizeof(float));
            memcpy(&temp[14], start_pos_, 2 * sizeof(float));
            memcpy(&temp[16], cmode_, sizeof(float));

            //** CRITICAL SECTION **//
            sem_wait(sem_client_impedance);
            shmem.write((uint8_t *)temp, 17 * sizeof(float), 0);
            sem_post(sem_master_impedance);
            //** CRITICAL SECTION **//
        }
        std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
}

/// @brief  Handler function for the t_feedback thread; "input" channel
void f_feedback(shared_memory &shmem)
{
    float temp[8];
    while (run_)
    {
        //** CRITICAL SECTION **//
        sem_wait(sem_master_feedback);
        shmem.read((uint8_t *)temp, 8 * sizeof(float), 17 * sizeof(float));
        sem_post(sem_client_feedback);
        //** CRITICAL SECTION **//

        memcpy(theta_meas_, &temp[0], 2 * sizeof(float));
        memcpy(tau_motor_meas_, &temp[2], 2 * sizeof(float));
        memcpy(vel_meas_, &temp[4], 2 * sizeof(float));
        memcpy(tau_cmd_, &temp[6], 2 * sizeof(float));

        feedback_available_ = true;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(10));
}

/// @brief  SIGINT handler
void h_sigint(int sig) { run_ = false; }

// mechanical system parameters
float inertia = 0.013475;         // motor-side
float friction_coeff = 0.25081;   // motor-side
float friction_coeff1 = 0.084305; // motor-side

// EKF -------------------------------------------------------------
VectorXd predict(const VectorXd &x, double dt)
{
    double theta = x(0);
    double theta_dot = x(1);
    double taau = x(2);

    double theta_ddot = (taau - friction_coeff * theta_dot - friction_coeff1 * (theta_dot >= 0 ? 1 : -1)) / inertia;

    VectorXd x_next(3);
    x_next << theta + theta_dot * dt,
        theta_dot + theta_ddot * dt,
        taau;

    return x_next;
}

MatrixXd jacobianA(const VectorXd &x, double dt)
{
    double theta_dot = x(1);
    double d_theta_ddot_d_theta_dot = -friction_coeff / inertia;
    double d_theta_ddot_d_tau = 1 / inertia;

    MatrixXd A1(3, 3);

    A1 << 1, dt, 0,
        0, 1 + d_theta_ddot_d_theta_dot * dt, d_theta_ddot_d_tau * dt,
        0, 0, 1;

    return A1;
}



double kalmanStep(
    VectorXd& x_hat,       // State estimate
    MatrixXd& P,           // Estimate covariance
    const double dt,       // Time step
    const VectorXd& theta_meas,  // Measurement: theta
    const VectorXd& vel_meas,    // Measurement: velocity
    const MatrixXd& Q,     // Process noise covariance
    const MatrixXd& R,     // Measurement noise covariance
    const MatrixXd& H,     // Observation model
    double& tau_hat        // Output: estimated torque
) {
    // Predict step
    VectorXd x_hat_pred = predict(x_hat, dt);
    MatrixXd A = jacobianA(x_hat, dt);
    P = A * P * A.transpose() + Q;

    // Measurement vector
    VectorXd y(2);
    y << theta_meas(0), vel_meas(0);

    // Kalman gain
    MatrixXd S = H * P * H.transpose() + R;
    MatrixXd K = P * H.transpose() * S.inverse();

    // Update step
    VectorXd y_hat = H * x_hat_pred;
    x_hat = x_hat_pred + K * (y - y_hat);
    P = P - K * H * P;

    // Extract estimated torque (or any specific state)
    return x_hat(2);
}


class LowPassFilter
{
public:
    // constructor
    LowPassFilter(double cutoff_frequency, double sample_time)
        : cutoff_frequency(cutoff_frequency), sample_time(sample_time), initialized(false)
    {
        // compute the alphavalue based on the cutoff frequency and sample time
        double RC = 1.0 / (2 * M_PI * cutoff_frequency);
        alpha1 = sample_time / (RC + sample_time);
        previous_output = 0; // This can be set to 0 initially or adjusted below
    }

    // Method to apply the filter on a new input value
    float filter(double input)
    {
        if (!initialized)
        {
            // on the first iterationm set previous_output to the first input to avoid bias
            previous_output = input;
            initialized = true;
        }
        float output = previous_output + alpha1 * (input - previous_output);
        previous_output = output;
        return output;
    }
private:
    double cutoff_frequency; // cutoff frequency (Hz)
    double sample_time;      // Sampling time (seconds)
    double alpha1;           // Filter coefficient
    double previous_output;  // Last filtered value
    bool initialized;        // To track if the filter has been initialized
};

int main(int argc, char **argv)
{
    struct sigaction s_action;
    s_action.sa_handler = h_sigint;
    s_action.sa_flags = 0;
    sigemptyset(&s_action.sa_mask);
    sigaction(SIGINT, &s_action, NULL);

    int shmid;
    void *shmaddr;

    // Create shared memory (master)
    shared_memory shmem("/usr/local/include/shmem.hpp", 4096, true);
    // Init
    shmem_init();

    // Wait for the client...
    std::cout << "Init ok\nWaiting for the ODrive-Master...\n";
    sem_wait(sem_sync);

    std::cout << "Started\n\n";

    std::thread t_impedance(f_impedance, std::ref(shmem));
    std::thread t_feedback(f_feedback, std::ref(shmem));

    // Get the shared memory segment
    shmid = shmget(SHM_KEY, SHM_SIZE, 0666 | IPC_CREAT);
    if (shmid == -1)
    {
        perror("shmget");
        return 1;
    }

    // Attach the shared memory segment to the process's address space
    shmaddr = shmat(shmid, nullptr, 0);
    if (shmaddr == (void *)-1)
    {
        perror("shmat");
        return 1;
    }

    // Counter updated at 1 kHz thanks to the feedback loop (ODrive-Master's frequency)
    volatile uint32_t loop_counter = 0;

    // Trajectory Parameters Declaration
    float dt = 0.001f;
    float t = 0.0f;
    float h = 1.57f;
    float eps = 0.0f;
    float dwell[2] = {theta_meas_[0], theta_meas_[1]};

    // Initial Error Conditions
    float ed_new = 0.0f;
    float edd_new = 0.0f;

    float q = 0.0f;
    float qv = 0.0f;

    float pi = 3.14159265f;

    bool first_time;
    bool assisting;
    bool assisstance;
    bool starttrigger = 0;

    VectorXd x_hat(3);
    x_hat << 0, 0, 0;

    MatrixXd P = MatrixXd::Identity(3, 3);

    MatrixXd Q = MatrixXd::Identity(3, 3);

    MatrixXd R(2, 2);
    R << 1.6e-9, 0,
        0, 1.6e-3;

    MatrixXd H(2, 3);

    H << 1, 0, 0,
        0, 1, 0;

    LowPassFilter lpFilter(2.0, 0.001);
    LowPassFilter velfilter(1.8, 0.001);
    LowPassFilter total_torque_filt(2.0, 0.001);

    while (run_)
    {
        // Local variables
        float _pos, _ref, _vel;

        // read the boolean varuables and structure
        bool *detectiontosend = (bool *)shmaddr;
        bool detection = *detectiontosend;

        timestampstrc *frametimestamp = (timestampstrc *)((char *)shmaddr + sizeof(bool));
        timestampstrc frametimemoment = *frametimestamp;

        if (feedback_available_)
        {
            feedback_available_ = false;

            // Use the 1000 Hz update rate from ODrive to generate a lower frequency loop here
            loop_counter++;

            if ((first_loop_))
            {
                if ((theta_meas_[0] != 0) && (theta_meas_[1] != 0))
                {
                    first_loop_ = false;
                    start_pos_[0] = theta_meas_[0];
                    start_pos_[1] = theta_meas_[1];
                    if (verbose_)
                        std::cout << "\n\n\033[32m start_pos_ initialized !  \033[0m\n\n" << start_pos_[0] <<" "<< start_pos_[1] << std::endl;
                }
                else
                {
                    if (verbose_)
                    {
                        std::cout << "\033[31m start_pos_ not initialized  \033[0m\n\n" << std::endl;
                        std::cout << "theta_meas_ " << theta_meas_[0] <<" "<< theta_meas_[1] << std::endl;
                    }
                    continue;
                }

                theta_ref_[0] = theta_meas_[0];
                theta_ref_[1] = theta_meas_[1];
                vel_ref_[0] = vel_meas_[0];
                vel_ref_[1] = vel_meas_[1];
            }
                
            // EKF torque calculation

            VectorXd x_hat_pred = predict(x_hat, dt);
            MatrixXd A = jacobianA(x_hat, dt);
            P = A * P * A.transpose() + Q;
            VectorXd y(2);
            y << theta_meas_[0], vel_meas_[0];

            MatrixXd S = H * P * H.transpose() + R;
            MatrixXd K = P * H.transpose() * S.inverse();
            VectorXd y_hat = H * x_hat_pred;
            x_hat = x_hat_pred + K * (y - y_hat);
            P = P - K * H * P;
            
            tau_hat_[0] = x_hat(2);

            // calculating the external torque and filtering it
            // ext_torque_ = total toque - motor_torque
            ext_torque_[0] = tau_hat_[0] + tau_motor_meas_[0]; // the plus sign is because of the fact that the sign of torque for axis 0 is the opposite of its position

            // Filtering some data
            filtered_ext_torque_[0] = lpFilter.filter(ext_torque_[0]);
            filtered_vel_meas_[0] = velfilter.filter(vel_meas_[0]);

            // Signal the thread that we can update the shared memory

            theta_ref_[0]=start_pos_[0]+0.5;
            theta_ref_[1]=start_pos_[1]+0.5;

            impedance_update_ = true;


            // Log to console (if enabled)
            if (verbose_ && loop_counter % 100 == 0)
            {
                std::cout << "theta_ref_ " << theta_ref_[0] <<" "<< theta_ref_[1] << std::endl;
                std::cout << "theta_meas_ " << theta_meas_[0] <<" "<< theta_meas_[1] << std::endl;
                // std::cout << "mode: " << mode_[0] <<" cmode: "<< cmode_[0] << std::endl;
                // std::cout << "filtered ext torque " << filtered_ext_torque_[0] <<" "<< filtered_ext_torque_[1] << std::endl;
                // std::cout << "tau hat " << tau_hat_[0] <<" "<< tau_hat_[1] << std::endl;
                // std::cout << "tau motor meas " << tau_motor_meas_[0] <<" "<< tau_motor_meas_[1] << std::endl;
                // std::cout << "start trigger: " << starttrigger << std::endl;

                std::cout << "ext_torque_[0] = t_hat + t_meas : " << ext_torque_[0] <<" = "<< tau_hat_[0] <<" + "<<tau_motor_meas_[0] << std::endl;

            }
        }
    }

    std::cout << "StartPos0: " << start_pos_[0] << std::endl;
    std::cout << "StartPos1: " << start_pos_[1] << std::endl;

    t_feedback.join();
    t_impedance.join();

    // Stop the client
    sem_post(sem_sync);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    while (sem_trywait(sem_sync) != 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Dump the shmem object
    if (verbose_)
        shmem.dump(10);

    // Close and free the shared memory
    shmem.destroy();
    if (shmdt(shmaddr) == -1)
    {
        perror("shmdt");
        return 1;
    }

    int r = 0;
    r += sem_close(sem_master_impedance);
    r += sem_close(sem_client_impedance);
    r += sem_close(sem_master_feedback);
    r += sem_close(sem_client_feedback);
    r += sem_close(sem_sync);
    if (r != 0)
        std::cout << "sem_close() error!\n";

    std::cout << r;
    return r;
}

void shmem_init()
{

    // Unlink any pre-existing semaphores
    sem_unlink("/sem_master_impedance");
    sem_unlink("/sem_client_impedance");
    sem_unlink("/sem_master_feedback");
    sem_unlink("/sem_client_feedback");
    sem_unlink("/sem_elio_sync");

    // Open or create the semaphores and the corresponding files (O_CREAT flag)
    // They are created in /dev/shm/ with permission 0660
    sem_master_impedance = sem_open("/sem_master_impedance", O_CREAT | IPC_CREAT, 0660, 0);
    if (sem_master_impedance == SEM_FAILED)
    {
        perror("sem_open master_impedance");
        exit(-1);
    }
    sem_client_impedance = sem_open("/sem_client_impedance", O_CREAT | IPC_CREAT, 0660, 1);
    if (sem_client_impedance == SEM_FAILED)
    {
        perror("sem_open client_impedance");
        exit(-1);
    }

    sem_master_feedback = sem_open("/sem_master_feedback", O_CREAT | IPC_CREAT, 0660, 0);
    if (sem_master_feedback == SEM_FAILED)
    {
        perror("sem_open master_feedback");
        exit(-1);
    }
    sem_client_feedback = sem_open("/sem_client_feedback", O_CREAT | IPC_CREAT, 0660, 1);
    if (sem_client_feedback == SEM_FAILED)
    {
        perror("sem_open client_feedback");
        exit(-1);
    }

    sem_sync = sem_open("/sem_elio_sync", O_CREAT | IPC_CREAT, 0660, 0);
    if (sem_client_feedback == SEM_FAILED)
    {
        perror("sem_open elio_sync");
        exit(-1);
    }
}
