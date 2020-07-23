#include "walking_state.hpp"
#include "balanceFeedback.hpp"

/* 
  target angles for each state
  l_hip_pitch, l_hip_roll, r_hip_pitch, r_hip_roll
*/
jointValue joint_value[5] = {
                {-0.4, 0.0, -0.7, 0.0},
                {0.1, 0.0, -0.7, 0.0},
                {0.4, 0.0, 1.1, 0.0},
                {0.3, 0.0, -0.1, 0.0},
                {-0.7, 0.0, -0.7, 0.0}
};

static double (*state_func[5])(double,double,std::string) = {state_zero, state_one, state_two, state_three, initial_state_zero};
static double balance_tunning[5] = {joint_value[0].l_hip_pitch, 
                                    joint_value[1].l_hip_pitch, 
                                    joint_value[2].r_hip_pitch,
                                    joint_value[3].r_hip_pitch,
                                    joint_value[4].r_hip_pitch};

double walk_position(std::string joint_name){
    if(joint_name == "head_pan"){
        return 0.00012694722852124585;
    }
    if(joint_name == "head_tilt"){
        return -0.17728819249344063;
    }
    
    if(joint_name == "l_ank_pitch"){
        return 0.646665613180418;
    }
    if(joint_name == "l_ank_roll"){
        return -0.03291861562932219;
    }
    if(joint_name == "l_el"){
        return -0.7855104106774036;
    }
    if(joint_name == "l_hip_pitch"){
        return -0.5525954118408896;
    }
    if(joint_name == "l_hip_roll"){
        return -0.027487078840958468;
    }
    if(joint_name == "l_hip_yaw"){
        return -0.001885896914339824;
    }
    if(joint_name == "l_knee"){
        return 1.0821164232041829;
    }
    if(joint_name == "l_sho_pitch"){
        return -0.08764888603198617;
    }
    if(joint_name == "l_sho_roll"){
        return 0.7936567609779654;
    }

    if(joint_name == "r_ank_pitch"){
        return -0.6479910051425213;
    }
    if(joint_name == "r_ank_roll"){
        return 0.03261413078383324;
    }
    if(joint_name == "r_el"){
        return 0.7855119917861186;
        
    }
    if(joint_name == "r_hip_pitch"){
        return 0.5507269288268102;
    }
    if(joint_name == "r_hip_roll"){
        return 0.027803371628569273;
    }
    if(joint_name == "r_hip_yaw"){
        return 0.0016847376209758735;
        
    }
    if(joint_name == "r_knee"){
        return -1.0820369647580703;
    }
    if(joint_name == "r_sho_pitch"){
        return 0.08744525835782824;
    }
    if(joint_name == "r_sho_roll"){
        return -0.7935969445137809;
    }
}

// Calculating Torques in each state

double initial_state_zero(double cur_pos, double cur_vel, std::string joint_name){
   if(joint_name == "r_hip_pitch"){   // stance hip
        return calTorque(-cur_pos, joint_value[4].r_hip_pitch, -cur_vel, 0.0);
    }
    else if(joint_name == "l_hip_pitch"){    // swing hip
        return calTorque(cur_pos, joint_value[4].l_hip_pitch, cur_vel, 0.0);
    }
    else if(joint_name == "l_knee"){         // swubg knee
        return calTorque(cur_pos, 1.4, cur_vel, 0.0);
    }
    else if(joint_name == "r_knee"){         // swubg knee
        return calTorque(cur_pos, -0.05, cur_vel, 0.0);
    }
    else if(joint_name == "l_ank_pitch"){    // swing ank
        return calTorque(cur_pos, 0.6, cur_vel, 0.0);
    }
    else if(joint_name == "r_ank_pitch"){    // stance ank
        return calTorque(cur_pos, -0.8, cur_vel, 0.0);
    }


    else if(joint_name == "l_hip_roll"){     // swing hip lat
         return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else if(joint_name == "l_hip_yaw"){      // swing hip lat
         return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else if(joint_name == "r_hip_roll"){    // stance hip lat
        return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else if(joint_name == "r_hip_yaw"){     // stance hip lat
        return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else if(joint_name == "l_ank_roll"){     // swing ank lat
        return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else if(joint_name == "r_ank_roll"){     // stance ank lat
        return calTorque(cur_pos, 0.0, cur_vel, 0.0);   
    }
    else{
        return calTorque(cur_pos, walk_position(joint_name), cur_vel, 0.0);
    }
}

double state_zero(double cur_pos, double cur_vel, std::string joint_name){
   if(joint_name == "r_hip_pitch"){   // stance hip
        return calTorque(-cur_pos, joint_value[0].r_hip_pitch, -cur_vel, 0.0);
    }
    else if(joint_name == "l_hip_pitch"){    // swing hip
        return calTorque(cur_pos, joint_value[0].l_hip_pitch, cur_vel, 0.0);
    }
    else if(joint_name == "l_knee"){         // swubg knee
        return calTorque(cur_pos, 1.4, cur_vel, 0.0);
    }
    else if(joint_name == "r_knee"){         // swubg knee
        return calTorque(cur_pos, -0.0, cur_vel, 0.0);
    }
    else if(joint_name == "l_ank_pitch"){    // swing ank
        return calTorque(cur_pos, 0.6, cur_vel, 0.0);
    }
    else if(joint_name == "r_ank_pitch"){    // stance ank
        return calTorque(cur_pos, -0.3, cur_vel, 0.0);
    }


    else if(joint_name == "l_hip_roll"){     // swing hip lat
         return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else if(joint_name == "l_hip_yaw"){      // swing hip lat
         return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else if(joint_name == "r_hip_roll"){    // stance hip lat
        return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else if(joint_name == "r_hip_yaw"){     // stance hip lat
        return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else if(joint_name == "l_ank_roll"){     // swing ank lat
        return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else if(joint_name == "r_ank_roll"){     // stance ank lat
        return calTorque(cur_pos, 0.0, cur_vel, 0.0)*2;   
    }
    else{
        return calTorque(cur_pos, walk_position(joint_name), cur_vel, 0.0);
    }
}

double state_one(double cur_pos, double cur_vel, std::string joint_name){
    if(joint_name == "r_hip_pitch"){   // stance hip
        return calTorque(-cur_pos, joint_value[1].r_hip_pitch, -cur_vel, 0.0);
    }
    else if(joint_name == "l_hip_pitch"){    // swing hip
        return calTorque(cur_pos, joint_value[1].l_hip_pitch, cur_vel, 0.0);
    }
    else if(joint_name == "l_knee"){         // swubg knee
        return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else if(joint_name == "r_knee"){         // swubg knee
        return calTorque(cur_pos, -0.1, cur_vel, 0.0);
    }
    else if(joint_name == "l_ank_pitch"){    // swing ank
        return calTorque(cur_pos, 0.10, cur_vel, 0.0);
    }
    else if(joint_name == "r_ank_pitch"){    // stance ank
        return calTorque(cur_pos, -0.3, cur_vel, 0.0);
    }

    else if(joint_name == "l_ank_roll"){     // swing ank lat
        return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else if(joint_name == "l_hip_roll"){     // swing hip lat
         return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else if(joint_name == "l_hip_yaw"){      // swing hip lat
         return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else if(joint_name == "r_ank_roll"){     // stance ank lat
        return calTorque(cur_pos, 0.0, cur_vel, 0.0)*2;   
    }
    else if(joint_name == "r_hip_roll"){    // stance hip lat
        return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else if(joint_name == "r_hip_yaw"){     // stance hip lat
        return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else{
        return calTorque(cur_pos, walk_position(joint_name), cur_vel, 0.0);
    }
}

double state_two(double cur_pos, double cur_vel, std::string joint_name){
    if(joint_name == "r_hip_pitch"){   // swing hip
        return calTorque(cur_pos, joint_value[2].r_hip_pitch, cur_vel, 0.0);
    }
    else if(joint_name == "l_hip_pitch"){    // stance hip
        return calTorque(-cur_pos, joint_value[2].l_hip_pitch, -cur_vel, 0.0);
    }
    else if(joint_name == "l_knee"){         // swubg knee
        return calTorque(cur_pos, -0.2, cur_vel, 0.0)*1.5;
    }
    else if(joint_name == "r_knee"){         // swubg knee
        return calTorque(cur_pos, -1.2, cur_vel, 0.0);
    }
    else if(joint_name == "l_ank_pitch"){    // swing ank
        return calTorque(cur_pos, 1.1, cur_vel, 0.0);
    }
    else if(joint_name == "r_ank_pitch"){    // stance ank
        return calTorque(cur_pos, -0.6, cur_vel, 0.0);
    }

    else if(joint_name == "l_ank_roll"){     // swing ank lat
        return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else if(joint_name == "l_hip_roll"){     // swing hip lat
         return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else if(joint_name == "l_hip_yaw"){      // swing hip lat
         return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else if(joint_name == "r_ank_roll"){     // stance ank lat
        return calTorque(cur_pos, 0.0, cur_vel, 0.0);   
    }
    else if(joint_name == "r_hip_roll"){    // stance hip lat
        return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else if(joint_name == "r_hip_yaw"){     // stance hip lat
        return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else{
        return calTorque(cur_pos, walk_position(joint_name), cur_vel, 0.0);
    }
}

double state_three(double cur_pos, double cur_vel, std::string joint_name){
    if(joint_name == "r_hip_pitch"){   // stance hip
        return calTorque(cur_pos, joint_value[3].r_hip_pitch, cur_vel, 0.0);
    }
    else if(joint_name == "l_hip_pitch"){    // swing hip
        return calTorque(-cur_pos, joint_value[3].l_hip_pitch, -cur_vel, 0.0);
    }
    else if(joint_name == "l_knee"){         // swubg knee
        return calTorque(cur_pos, 0.0, cur_vel, 0.0)*1.5;
    }
    else if(joint_name == "r_knee"){         // swubg knee
        return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else if(joint_name == "l_ank_pitch"){    // swing ank
        return calTorque(cur_pos, 0.9, cur_vel, 0.0);
    }
    else if(joint_name == "r_ank_pitch"){    // stance ank
        return calTorque(cur_pos, -0.15, cur_vel, 0.0);
    }

    else if(joint_name == "l_ank_roll"){     // swing ank lat
        return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else if(joint_name == "l_hip_roll"){     // swing hip lat
         return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else if(joint_name == "l_hip_yaw"){      // swing hip lat
         return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else if(joint_name == "l_knee"){         // swubg knee
        return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else if(joint_name == "r_ank_roll"){     // stance ank lat
        return calTorque(cur_pos, 0.0, cur_vel, 0.0);   
    }
    else if(joint_name == "r_hip_roll"){    // stance hip lat
        return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else if(joint_name == "r_hip_yaw"){     // stance hip lat
        return calTorque(cur_pos, 0.0, cur_vel, 0.0);
    }
    else{
        return calTorque(cur_pos, walk_position(joint_name), cur_vel, 0.0);
    }
}

sensor_msgs::JointState make_msg(int state, const sensor_msgs::JointState::ConstPtr& msg){
    // side -> stance_leg
    std::string stance_leg;
    
    if(state == 0 || state == 1 || state == 4){
        stance_leg = "r";
    }
    else{
        stance_leg = "l";
    }

    sensor_msgs::JointState write_msg;
    write_msg.header = msg->header;

    double swing_hip_pitch = 0.0;
    double swing_hip_roll = 0.0;

    for(int ix = 0; ix <msg->name.size(); ix++){
        std::string joint_name = msg->name[ix];
        saveAngles(stance_leg, joint_name, msg->position[ix]);
        // swing hip angle save
        if(stance_leg == "r"){
            if(joint_name == "l_hip_pitch") 
                swing_hip_pitch = calTorque(msg->position[ix], joint_value[state].l_hip_pitch, msg->velocity[ix], 0.0);
            else if(joint_name == "l_hip_roll")
                swing_hip_roll = calTorque(msg->position[ix], joint_value[state].l_hip_roll, msg->velocity[ix], 0.0);
            else
                continue;
        }
        else{
            if(joint_name == "r_hip_pitch") 
                swing_hip_pitch = calTorque(msg->position[ix], joint_value[state].r_hip_pitch, msg->velocity[ix], 0.0);
            else if(joint_name == "r_hip_roll")
                swing_hip_roll = calTorque(msg->position[ix], joint_value[state].r_hip_roll, msg->velocity[ix], 0.0);
            else
                continue;
        }
    }
    toAnkleDistance(stance_leg);

    double (*temp_func)(double,double,std::string) = state_func[state];
    cout <<fixed;
    cout.precision(2);
    for(int ix = 0; ix< msg->name.size(); ix++){
        std::string joint_name = msg->name[ix];
        double torque = temp_func(msg->position[ix], msg->velocity[ix], joint_name);
        if(stance_leg == "r"){
            //stance hip torque
            if(joint_name == "r_hip_pitch")
                torque = (- swing_hip_pitch - torque);
            else if(joint_name == "r_hip_roll"){
                torque = (swing_hip_roll + torque) * 1.8;
                
            }
            
            else if(joint_name == "r_ank_roll"){
                torque = torque*1.5;
                
            }
            
 
            //balance feedback
            else if(joint_name == "l_hip_pitch")
                torque = calTorque(msg->position[ix], balanceFeedback("x", balance_tunning[state], "l"), msg->velocity[ix], 0.0);
            else if(joint_name == "l_hip_roll")
                torque = calTorque(msg->position[ix], balanceFeedback("y", 0.0, "l"), msg->velocity[ix], 0.0);

            
            else ;

        }
        else{
            //stance hip torque
            if(joint_name == "l_hip_pitch")
                torque =  (- swing_hip_pitch - torque);
            else if(joint_name == "l_hip_roll"){
                torque = (swing_hip_roll + torque) * 1.8;
            }
            
            else if(joint_name == "l_ank_roll"){
                torque = torque*1.5;
                
            }
            
           

            //balance feedback
            else if(joint_name == "r_hip_pitch")
                torque = calTorque(msg->position[ix], balanceFeedback("x", balance_tunning[state], "r"), msg->velocity[ix], 0.0);
            else if(joint_name == "r_hip_roll")
                torque = calTorque(msg->position[ix], balanceFeedback("y", 0.0, "r"), msg->velocity[ix], 0.0);
            else ;

        }
        
        write_msg.name.push_back(joint_name);
        write_msg.effort.push_back(torque);
    }
    //cout << " ----------------------------------------"<< endl;

    return write_msg;
}

// Calculate torque with PD controller equation
// torque = -Kp * (cur_pos - des_pos) - Kd * (cur_vel -des_vel)
double calTorque(double cur_pos, double des_pos, double cur_vel, double des_vel){
    //double p_gain = 2.64;
    //double p_gain = 2.8;
    //double p_gain = 3.2;  //6/30
    
    //double p_gain = 2.8;  //7/7
    //double p_gain = 2.6;
    double p_gain = 1.5;


    double d_gain = 0.05;
    double torque = 0.0;

    torque = -p_gain * (cur_pos - des_pos) - d_gain * (cur_vel - des_vel);
    
    if(torque > 4.0){
        torque = 4.0;
    }
    else if(torque <-4.0){
        torque = -4.0;
    }

    return torque;
}


