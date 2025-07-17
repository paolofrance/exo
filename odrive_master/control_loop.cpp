#include "ODrive.h"


void ODrive::control_loop() {

    // Higher-level torque command
    float t[2] = {0.0f,0.0f};

    if( working_mode == ZERO_TORQUE_MODE ) {
        t[0] = 0.0f;
        t[1] = 0.0f;
    }
    else if( working_mode == POSCONTROL_MODE ) {
        t[0] = position_control(0);//-0.1*friction_comp(0);
        t[1] = position_control(1);
        //t[1]= -position_control(0);
    }    

    // For logging
    memcpy(torque_cmd, t, sizeof(t));

    // Send torque command
    torque_command(t); 

}