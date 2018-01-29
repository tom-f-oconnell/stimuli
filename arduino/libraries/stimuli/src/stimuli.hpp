
#include <ros.h>

// TODO careful not to double namespace, if macro used w/in
// gets a string identifier for the function pair
#define REGISTER_FUNCTION_PAIR(f_on, f_off)\
    stim::register_function_pair(#f_on "&" #f_on, f_on, f_off);

namespace stim {
    // should this go in the namespace?
    typedef void (*func_ptr)();
    // typdef?
    struct func_pair {
        func_ptr f_on;
        func_ptr f_off;
    }

    const uint8_t max_num_pins = 12;

    void load_next_sequence(const stimuli::LoadSequenceRequest &req, \
      stimuli::LoadSequenceResponse &res);

    // TODO to expose or not the ROS internals... might be nice to have access 
    // to host configurable stuff in user functions... and could abstract?
    // how many type signatures?
    // TODO maybe replace w/ c++ templates?
    bool get_param(const char* n, int* param, const int* def, \
      bool required = false, int len = 1, int timeout = 1000);

    bool get_param(const char* n, float* param, const float* def, \
      bool required = false, int len = 1, int timeout = 1000);

    bool get_param(const char *n, char **param, char** def, \
      bool required = false, int len = 1, int timeout = 1000);

    // is this syntax with default args correct?
    bool get_param(const char* n, int* param, \
      bool required = false, int len = 1, int timeout = 1000);

    bool get_param(const char* n, float* param, \
      bool required = false, int len = 1, int timeout = 1000);

    bool get_param(const char *n, char **param, \
      bool required = false, int len = 1, int timeout = 1000);

    /*
    void signal_pin(unsigned char pin);
    void signal_pins();

    unsigned long to_millis(ros::Time t);
    unsigned long to_micros(ros::Time t);
    */

    // TODO document differences / only expose former
    void update_pwm_pinstates();
    void update_pulses_ros();
    void set_non_pwm_pins();
    void end_sequence();
    void update_pulses_blocking();

    void init();
    void update();

    ros::NodeHandle * get_nodehandle();

    void register_function_pair(char * id, func_ptr f_on, func_ptr f_off);
    void while_idle(func_ptr f);
}
