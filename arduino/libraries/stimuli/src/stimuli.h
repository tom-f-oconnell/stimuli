
// TODO include appropriate generated type stuff

namespace stim {
    const uint8_t max_num_pins = 12;

    void load_next_sequence(const stimuli::LoadSequenceRequest &req, \
      stimuli::LoadSequenceResponse &res);

    // TODO to expose or not the ROS internals... might be nice to have access 
    // to host configurable stuff in user functions... and could abstract?
    // how many type signatures?
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

    // TODO functions to register function pointers / pairs of them
}
