
#ifndef GAZEBO_ROS_OPENNI_KINECT_NOISY_HH
#define GAZEBO_ROS_OPENNI_KINECT_NOISY_HH
#include <gazebo_plugins/gazebo_ros_openni_kinect.h>
#include <sensor_msgs/PointCloud2.h>

namespace gazebo
{
  class GazeboRosOpenniKinectNoisy : public GazeboRosOpenniKinect
  {
	/// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: GazeboRosOpenniKinectNoisy();

    /// \brief Destructor
    public: ~GazeboRosOpenniKinectNoisy();
    
	private: bool FillPointCloudHelper(sensor_msgs::PointCloud2 &point_cloud_msg,
							  uint32_t rows_arg, uint32_t cols_arg,
							  uint32_t step_arg, void* data_arg);
	  
	private: double mean = 0.; // [cm]
	private: double sigma = 0.01; // [cm]
	/// \brief depth noise added to each pixel in depth cloud
    private: std::default_random_engine depth_noise_generator;
    private: std::normal_distribution<double> depth_noise;
  };
}

#endif
