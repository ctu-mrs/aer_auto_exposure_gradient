#ifndef EXP_ROS_NODE_H_
#define EXP_ROS_NODE_H_

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>
#include <libgen.h>
#include <mutex>
#include <sys/time.h>
#include <string>
#include <sstream>
#include <cstdlib>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.hpp>

// #include <dynamic_reconfigure/BoolParameter.h>
// #include <dynamic_reconfigure/DoubleParameter.h>
// #include <dynamic_reconfigure/IntParameter.h>
// #include <dynamic_reconfigure/StrParameter.h>
// #include <dynamic_reconfigure/GroupState.h>
// #include <dynamic_reconfigure/Reconfigure.h>
// #include <dynamic_reconfigure/Config.h>

#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>

#ifdef WITH_PLOTTER
#include <plotter_ros2/plotter.hpp>
#endif

//#include <aer_auto_exposure_gradient/Dehaze.h>

#define POLYNOME_DEGREE 2

namespace exp_node {

class ExpNode : public rclcpp::Node {
 public:

  explicit ExpNode(const rclcpp::NodeOptions & options);

 private:

  void CameraCb(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
  void optimizerCb();
  void optimizeGradient();
  void optimizeSimple();
  void optimizeShim();
  double image_gradient_gamma(cv::Mat &src_img, int j);
  void ChangeParam (double exposure_level);
  void shutterLimitCb(const std_msgs::msg::Int32::ConstSharedPtr &msg);
  
  std::array<double, 3> curveFitLogQuadratic(const std::vector<double>& x, const std::vector<double>& y);
  double findRoots1(double a[3]);
  void generate_LUT ();

  std::vector<double> gamma_;
  std::vector<double> metric_;
  double gamma_range_;
  int gamma_num_points_;
  int gamma_neutral_index_;
  std::vector<cv::Mat> gamma_luts_;
  cv::Mat lut_metric_;
  double max_gamma;
  double alpha;
  double expNew;
  double expCur;

  // Normalized optimizer state [0, 1]: 0 = minimum exposure, 1 = maximum exposure
  double exposure_level_cur_;
  double exposure_level_new_;
  double exposure_level_max_;

  // Actuator configuration.
  // Each actuator owns a contiguous slice of the normalized [0,1] optimizer output.
  // The order and width of the slices are fully configurable via ROS parameters:
  //   actuator_order: ["shutter", "gain", "led"]   (any permutation, subset, or repetition)
  //   shutter_portion / shutter_max_ms
  //   gain_portion    / gain_max
  //   led_portion     / led_max
  double shutter_portion_;
  double shutter_max_s_;           // set from shutter_max_us parameter (÷ 1 000 000)
  double gain_portion_;
  double gain_max_;
  double led_portion_;             // 0 = LED disabled
  double led_max_;                 // Watts

  // Runtime-ordered list built from actuator_order parameter.
  // ChangeParam iterates this vector in sequence, assigning each actuator its slice.
  struct ActuatorSlice {
    enum class Type { SHUTTER, GAIN, LED } type;
    double portion;    // fraction of [0,1] this actuator owns
    double max_value;  // physical upper limit (s for shutter, dB for gain, W for LED)
  };
  std::vector<ActuatorSlice> actuator_slices_;

  double simple_step_size_;        // step size for the "simple" optimizer in [0,1] units

  int startup_delay;
  double kp;
  double R;  // parameter used in the nonlinear function in Shim's 2018 paper
  int gamma_index; // index to record the location of the optimum gamma value
  std::string shutter_update_method;
  std::string shim_update_function;
  //std::string service_call ="camera/spinnaker_camera_nodelet/set_parameters";
  //std::string exp_param_call = "camera/spinnaker_camera_nodelet/exposure_time";
  //std::string gain_param_call = "camera/spinnaker_camera_nodelet/gain";

  double grad_k;
  double gamma_x_offset_;

  // Parameters that correlated to Shim's Gradient Metric
  double met_act_thresh = 0.06;
  double lamda = 1000.0; // The lamda value used in Shim's 2014 paper as a control parameter to adjust the mapping tendency (larger->steeper)

  // Soft percentile weighting (Zhang et al. 2017)
  bool softperc_weighting_;
  double softperc_percentile_;
  double softperc_k_;                     // exponent sharpening the bell-shaped weight curve (paper uses 5; <1 flattens)
  // bool softperc_integer_hist_;            // use histogram sort (fast) vs float sort
  // bool softperc_trapezoid_approx_;        // trapezoidal weight approximation inside histogram path
  std::vector<double> softperc_weights_;  // precomputed, unnormalized

  bool do_sweep;

  //ros::NodeHandle nh_;
  image_transport::Subscriber sub_camera_;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr shutter_speed_us_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gain_db_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr led_pub_;  // optional, null if disabled
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr shutter_limit_sub_; // dynamic shutter upper limit (µs)
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gamma_est_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gradient_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gradient_clipped_pub_;

  std::shared_ptr<rclcpp::Time> callback_start_time;
  std::shared_ptr<rclcpp::Time> zeroing_duration;
  std::shared_ptr<rclcpp::Time> last_camera_process_time_;

  int    test_sweep_step_;
  int    sweep_steps_;
  double true_best_exposure_level_ = 0.0;
  double metric_tmp;

#ifdef WITH_PLOTTER
  bool enable_plotter;
  std::shared_ptr<plotter_ros2::Plotter> plotter_gamma;
  std::shared_ptr<plotter_ros2::Plotter> plotter_sweep;
  std::vector<double> sweep_levels_;
  std::vector<double> sweep_metrics_;
  rclcpp::TimerBase::SharedPtr sweep_republish_timer_;
#endif

  int img_proc_loop_hz_;
  int img_proc_width_;
  int img_proc_height_;

  // Optimizer timer state
  double coeff_[POLYNOME_DEGREE + 1];           // curve-fit coefficients shared with optimizerCb
  double exposure_level_at_camera_;            // normalized exposure level when last image captured
  bool   new_camera_data_          = false;    // flag: new curve-fit data available
  int optimizer_loop_hz_;
  std::mutex optimizer_mutex_;
  std::mutex actuator_mutex_;  // protects actuator_slices_, shutter_max_s_, shutter_portion_
  rclcpp::TimerBase::SharedPtr optimizer_timer_;

  // Separate callback group for the optimizer timer so it can run concurrently
  // with the image callback when using component_container_mt.
  rclcpp::CallbackGroup::SharedPtr optimizer_cb_group_;

  // Gradient optimizer state (optimizer thread only, no mutex needed)
  double gamma_est_ = 1.0;  // current gamma estimate, resets to 1.0 on each new image
};

}

#endif
