/**
 * @file inspire_hand.cpp
 * @brief This is an example of how to control the Unitree H1 (Inspire) Hand using unitree_sdk2.
 */

// Inspire Hand Topic IDL Types
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
// DDS Channel
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/thread/thread.hpp>

#include <eigen3/Eigen/Dense>
#include <unordered_map>
#include <array>

/* H1 gamepad example */
#include <mutex>

#include "unitree/idl/go2/LowState_.hpp"
#include "unitree/common/thread/thread.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"

#define TOPIC_LOWSTATE "rt/lowstate"

using namespace unitree::common;
using namespace unitree::robot;

typedef union {
  struct {
    uint8_t R1 : 1;
    uint8_t L1 : 1;
    uint8_t start : 1;
    uint8_t select : 1;
    uint8_t R2 : 1;
    uint8_t L2 : 1;
    uint8_t F1 : 1;
    uint8_t F2 : 1;
    uint8_t A : 1;
    uint8_t B : 1;
    uint8_t X : 1;
    uint8_t Y : 1;
    uint8_t up : 1;
    uint8_t right : 1;
    uint8_t down : 1;
    uint8_t left : 1;
  } components;
  uint16_t value;
} xKeySwitchUnion;

typedef struct {
  uint8_t head[2];
  xKeySwitchUnion btn;
  float lx;
  float rx;
  float ry;
  float L2;
  float ly;

  uint8_t idle[16];
} xRockerBtnDataStruct;
xRockerBtnDataStruct remote_key_data;    



/* define the ins_hand L && R */
uint8_t L_palm=1;
uint8_t L_Thumb=2;
uint8_t L_Forefinger=3;
uint8_t L_Middlefinger=4;
uint8_t L_Ringfinger=5;
uint8_t L_Littlefinger=6;
uint8_t R_palm=7;
uint8_t R_Thumb=8;
uint8_t R_Forefinger=9;
uint8_t R_Middlefinger=10;
uint8_t R_Ringfinger=11;
uint8_t R_Littlefinger=12;

uint8_t gamepad_state=0;                    //gamepad press state
uint8_t min_size,max_size;                  //the rocker leaning max && min
static uint8_t choose_hand=0;                //choose which one hand

std::array<float, 13> fingers_buff{};       //buffer to save the fingers angle
 

/**
 * @brief Unitree H1 Hand Controller
 * The user can subscribe to "rt/inspire/state" to get the current state of the hand and publish to "rt/inspire/cmd" to control the hand.
 * 
 *                  IDL Types
 * user ---(unitree_go::msg::dds_::MotorCmds_)---> "rt/inspire/cmd"
 * user <--(unitree_go::msg::dds_::MotorStates_)-- "rt/inspire/state"
 * 
 * @attention Currently the hand only supports position control, which means only the `q` field in idl is used.
 */
class H1HandController
{
public:
    H1HandController()
    {
        this->InitDDS_();
    }

    /**
     * @brief Control the hand to a specific label
     */
    void ctrl(std::string label)
    {
        if(labels.find(label) != labels.end())
        {
            this->ctrl(labels[label], labels[label]);
        }
        else
        {
            std::cout << "Invalid label: " << label << std::endl;
        }
    }

    /**
     * @brief Move the fingers to the specified angles
     * 
     * @note The angles should be in the range [0, 1]
     *       0: close  1: open
     */
    void ctrl(
        const Eigen::Matrix<float, 6, 1>& right_angles, 
        const Eigen::Matrix<float, 6, 1>& left_angles)
    {
        for(size_t i(min_size); i<max_size; i++)
        {
            if(choose_hand==1){
            cmd.cmds()[i+6].q() = left_angles(i);      
            }                           //choose the left hand 
            if(choose_hand==2){
            cmd.cmds()[i].q() = right_angles(i);
            }                           //choose the right hand 
            if(choose_hand==3){
            cmd.cmds()[i].q() = right_angles(i); 
            cmd.cmds()[i+6].q() = left_angles(i);
            }                           //choose the left && right hand 
        }
        handcmd->Write(cmd);
    }

    /**
     * @brief Get the right hand angles
     * 
     * Joint order: [pinky, ring, middle, index, thumb_bend, thumb_rotation]
     */
    Eigen::Matrix<float, 6, 1> getRightQ()
    {
        std::lock_guard<std::mutex> lock(mtx);
        Eigen::Matrix<float, 6, 1> q;
        for(size_t i(0); i<6; i++)
        {
            q(i) = state.states()[i].q();
        }
        return q;
    }

    /**
     * @brief Get the left hand angles
     * 
     * Joint order: [pinky, ring, middle, index, thumb_bend, thumb_rotation]
     */
    Eigen::Matrix<float, 6, 1> getLeftQ()
    {
        std::lock_guard<std::mutex> lock(mtx);
        Eigen::Matrix<float, 6, 1> q;
        for(size_t i(0); i<6; i++)
        {
            q(i) = state.states()[i+6].q();
        }
        return q;
    }

    unitree_go::msg::dds_::MotorCmds_ cmd;
    unitree_go::msg::dds_::MotorStates_ state;
private:
    void InitDDS_()
    {
        handcmd = std::make_shared<unitree::robot::ChannelPublisher<unitree_go::msg::dds_::MotorCmds_>>(
            "rt/inspire/cmd");
        handcmd->InitChannel();
        cmd.cmds().resize(12);
        handstate = std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::MotorStates_>>(
            "rt/inspire/state");
        handstate->InitChannel([this](const void *message){
            std::lock_guard<std::mutex> lock(mtx);
            state = *(unitree_go::msg::dds_::MotorStates_*)message;
        });
        state.states().resize(12);
    }

    // DDS parameters
    std::mutex mtx;
    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::MotorCmds_> handcmd;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::MotorStates_> handstate;

    // Saved labels
    std::unordered_map<std::string, Eigen::Matrix<float, 6, 1>> labels = {
        {"open",   Eigen::Matrix<float, 6, 1>::Ones()},
        {"close",  Eigen::Matrix<float, 6, 1>::Zero()},
        {"half",   Eigen::Matrix<float, 6, 1>::Constant(0.5)},
    };
};



/*gamepad conctrl*/
class GamepadExample
{
public:
    GamepadExample() {}

    // setup dds model
    void InitDdsModel(const std::string &networkInterface = "")
    {
        ChannelFactory::Instance()->Init(0, networkInterface);
        lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));

        lowstate_subscriber->InitChannel(std::bind(&GamepadExample::MessageHandler, this, std::placeholders::_1), 1);
    }

    // callback function to save joystick message
    void MessageHandler(const void *message)
    {
        std::lock_guard<std::mutex> lock(joystick_mutex);
        lowstate_msg = *(unitree_go::msg::dds_::LowState_ *)message;
    }

    // work thread
    void Step()
    {
        {
            std::lock_guard<std::mutex> lock(joystick_mutex);
            memcpy(&remote_key_data, &lowstate_msg.wireless_remote()[0], 40);
        }
        // some operations
        if(!(remote_key_data.btn.components.L2&&remote_key_data.btn.components.L1&&remote_key_data.btn.components.R2&&remote_key_data.btn.components.R1&&remote_key_data.btn.components.left
        &&remote_key_data.btn.components.up&&remote_key_data.btn.components.right&&remote_key_data.btn.components.down&&remote_key_data.btn.components.X&&remote_key_data.btn.components.Y
        &&remote_key_data.btn.components.A&&remote_key_data.btn.components.B)&&(remote_key_data.btn.components.F1))
            gamepad_state=0; 
        if (remote_key_data.btn.components.L2){
            gamepad_state=L_palm;
        }
        if (remote_key_data.btn.components.L1){
            gamepad_state=L_Thumb;
        }
        if (remote_key_data.btn.components.up){
            gamepad_state=L_Forefinger;
        }
        if (remote_key_data.btn.components.left){
            gamepad_state=L_Middlefinger;
        }
        if (remote_key_data.btn.components.down){
            gamepad_state=L_Ringfinger;
        }
        if (remote_key_data.btn.components.right){
            gamepad_state=L_Littlefinger;
        }
        if (remote_key_data.btn.components.R2){
            gamepad_state=R_palm;
        }
        if (remote_key_data.btn.components.R1){
            gamepad_state=R_Thumb;
        }
         if (remote_key_data.btn.components.Y){
            gamepad_state=R_Forefinger;
        }
        if (remote_key_data.btn.components.X){
            gamepad_state=R_Middlefinger;
        }
        if (remote_key_data.btn.components.A){
            gamepad_state=R_Ringfinger;
        }
        if (remote_key_data.btn.components.B){
            gamepad_state=R_Littlefinger;
        }     
        
        if(gamepad_state<=6){choose_hand=1;}
        if(gamepad_state>6) {choose_hand=2;}
        if(gamepad_state==0){choose_hand=3;}
        // print gamepad state
        // std::cout << "ry: \n\n" << value_ry << std::endl;
         }

    // start the work thread
    void Start()
    {
        control_thread_ptr = CreateRecurrentThreadEx("nn_ctrl", UT_CPU_ID_NONE, 40000, &GamepadExample::Step, this);
    }

protected:
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
    unitree_go::msg::dds_::LowState_ lowstate_msg;

 

    ThreadPtr control_thread_ptr;

    std::mutex joystick_mutex;

    int press_count = 0;
};



/**
 * Main Function
 */
int main(int argc, char** argv)
{
    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     H1 Hand              \n\n";

    // Initialize the DDS Channel
    std::string networkInterface = argc > 1 ? argv[1] : "";
    unitree::robot::ChannelFactory::Instance()->Init(0, networkInterface);

    // Create the H1 Hand Controller
    auto h1hand = std::make_shared<H1HandController>();
    // the full hand max && min finger size
    int min_full_size=0,max_full_size;

    static float value_ry = 0.0f;   //gamepad ry vaule
    static bool active = false;     //gamepad ry action
    static bool is_upward = false;  //gamepa ry direction
    static uint8_t buff_cnt=0;      //hand angle cnt;
    float speed = 0;                //move the gamepad speed
    float ry = remote_key_data.ry;
    const float threshold = 0.05f;  //anti ry shake
    Eigen::Matrix<float, 6, 1> lq, rq;
    

    GamepadExample example;

    // start program
    example.InitDdsModel("eth0");
    

    while (true)
    {  
        example.Start();            //gamepad
     
            

        /*gamepad activtes*/
        if(fabs(ry)<threshold){
            active = false;
        }
        if(remote_key_data.ry >= threshold){
            active=true;
            is_upward=true;
        }
        if(remote_key_data.ry <= -threshold){
            active=true;
            is_upward=false;
        }

        if(active){
            if(is_upward&&remote_key_data.ry>value_ry&&value_ry<=1){
                speed = remote_key_data.ry*remote_key_data.ry*remote_key_data.ry;
                value_ry+=speed;
            }else if(!is_upward&&remote_key_data.ry<value_ry&&value_ry>=-1){
                speed = remote_key_data.ry*remote_key_data.ry*remote_key_data.ry;
                value_ry-=0.02f;
            }
            fingers_buff[gamepad_state-1] = value_ry; 
        }

       switch(gamepad_state){
            case 0:    min_size=0;max_size=6;
                        for(min_full_size=0;min_full_size<6;min_full_size++){
                            for(max_full_size=6;max_full_size>1;max_full_size--){
                            min_size=min_full_size;max_size=max_full_size; 
                            h1hand->ctrl(lq, rq); // Control the hand
                            lq.fill(value_ry);rq.fill(value_ry); 
                            }
                        }
           break;
            case 1:    min_size=5;max_size=6;rq.fill(fingers_buff[gamepad_state-1]);break;
            case 2:    min_size=4;max_size=5;rq.fill(fingers_buff[gamepad_state-1]);break;
            case 3:    min_size=3;max_size=4;rq.fill(fingers_buff[gamepad_state-1]);break;
            case 4:    min_size=2;max_size=3;rq.fill(fingers_buff[gamepad_state-1]);break;
            case 5:    min_size=1;max_size=2;rq.fill(fingers_buff[gamepad_state-1]);break;
            case 6:    min_size=0;max_size=1;rq.fill(fingers_buff[gamepad_state-1]);break;
            case 7:    min_size=5;max_size=6;lq.fill(fingers_buff[gamepad_state-1]);break;
            case 8:    min_size=4;max_size=5;lq.fill(fingers_buff[gamepad_state-1]);break;
            case 9:    min_size=3;max_size=4;lq.fill(fingers_buff[gamepad_state-1]);break;
            case 10:   min_size=2;max_size=3;lq.fill(fingers_buff[gamepad_state-1]);break;
            case 11:   min_size=1;max_size=2;lq.fill(fingers_buff[gamepad_state-1]);break;
            case 12:   min_size=0;max_size=1;lq.fill(fingers_buff[gamepad_state-1]);break;
       }



            h1hand->ctrl(lq, rq); // Control the hand
        usleep(100000);
            // lq = Eigen::Matrix<float, 6, 1>::Ones() - lq;
            // rq = Eigen::Matrix<float, 6, 1>::Ones() - rq;

        std::cout << "-- Hand State --\n";
        std::cout << " R: " << h1hand->getRightQ().transpose() << std::endl;
        std::cout << " L: " << h1hand->getLeftQ().transpose() << std::endl;
        std::cout << "\033[3A"; // Move cursor up 3 lines
        
    }

    return 0;
}
