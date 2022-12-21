#include <eigen_conversions/eigen_msg.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/expressions.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/gtsam_wrappers.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <utils/conversions.h>
#include <utils/utils.h>
#include <visualization_msgs/MarkerArray.h>
//#include <stalker/Range.h>
#include <stonefish_ros/BeaconInfo.h>

using namespace graph_slam_3d;
typedef gtsam::BearingRange<gtsam::Pose3, gtsam::Point3> BearingRange3D;
typedef gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3> Range3D;
typedef gtsam::Range<gtsam::Pose3, int> Radius;

class AcousticLoc
{
   private:
	// ROS variables
	ros::NodeHandle nh_;
	ros::Subscriber subscriber_robot_pose_;
	ros::Subscriber subscriber_range_;
	ros::Publisher publisher_robot_optimized_path_;
	ros::Publisher publisher_landmarks_;

	// ROS topics
	std::string ros_topic_odometry_;
	std::string ros_topic_range_;
	std::string publisher_robot_optimized_path_topic_;
	std::string publisher_landmarks_topic_;

	// GTSAM variables

	// ExpressionFactorGraph is a NonLinearFactorGraph that can
	// *additionally* use expressions as factors. This is useful
	// for us because it allows us to work with BearingRange
	// expressions
	gtsam::NonlinearFactorGraph graph_;

	gtsam::Values initial_estimate_;
	bool use_isam_ = false;
	gtsam::ISAM2* isam2_ = nullptr;

	// SLAM variables
	int robot_pose_counter_ = -1;
	std::vector<int> landmark_id_vector_;
	double last_ping_stamp_ = -1;

	//
	std::string rviz_ins_frame_ = "world_ned";

	// Node options
	int n_cameras_ = 1;
	bool debug_ = false;
	bool calibrate_extrinsics_camera_ = false;
	bool constrain_z_cam_ = false;

	// MRPT variables
	mrpt::poses::CPose3DPDFGaussian newest_ins_pose_;
	mrpt::poses::CPose3DPDFGaussian last_ins_pose_;

	

	// Functions
	void getStaticConfig();
	void insCallback(const nav_msgs::Odometry& msg);
	void rangeCallback(const stonefish_ros::BeaconInfo& range_info);

   public:
	AcousticLoc(const ros::NodeHandle& nh);
	~AcousticLoc();
};

AcousticLoc::AcousticLoc(const ros::NodeHandle& nh) : nh_(nh)
{
	getStaticConfig();

	if (use_isam_)
	{
		printf("Using ISAM2\n");
		gtsam::ISAM2Params parameters;
		// parameters.relinearizeThreshold = 0.01;
		// parameters.relinearizeSkip = 1;
		isam2_ = new gtsam::ISAM2(parameters);
	}
	else { printf("Using Levenberg Marquardt Optimizer\n"); }

	//graph_= gtsam::NonlinearFactorGraph();
	//initial_estimate_= gtsam::Values();

	// Initialize ROS subscribers and publishers
	subscriber_robot_pose_ =
		nh_.subscribe(ros_topic_odometry_, 2, &AcousticLoc::insCallback, this);
	
	subscriber_range_ = nh_.subscribe(
		ros_topic_range_, 2, &AcousticLoc::rangeCallback, this);
		
	publisher_robot_optimized_path_ = nh_.advertise<nav_msgs::Path>(
		publisher_robot_optimized_path_topic_, 2, true);
	publisher_landmarks_ = nh_.advertise<visualization_msgs::Marker>(
		publisher_landmarks_topic_, 2, true);
}

AcousticLoc::~AcousticLoc() {}

void AcousticLoc::getStaticConfig()
{
	bool ok = true;

	// ROS
	// Topics

	ok &= utils::getParamAndCheck(nh_, "n_cameras", n_cameras_);
	ok &= utils::getParamAndCheck(
		nh_, "ros/topics/subscribers/odometry", ros_topic_odometry_);

	ok &= utils::getParamAndCheck(
		nh_, "ros/topics/subscribers/range_info",
		ros_topic_range_);

	ok &= utils::getParamAndCheck(
		nh_, "ros/topics/publishers/robot_optimized_path",
		publisher_robot_optimized_path_topic_);
	ok &= utils::getParamAndCheck(
		nh_, "ros/topics/publishers/landmarks", publisher_landmarks_topic_);

	// Node options
	ok &= utils::getParamAndCheck(
		nh_, "calibrate_extrinsics_camera", calibrate_extrinsics_camera_);
	ok &= utils::getParamAndCheck(nh_, "debug", debug_);
	ok &= utils::getParamAndCheck(nh_, "use_isam", use_isam_);

	// T_ins_camera
	if (!ok)
	{
		ROS_FATAL("Shutdown due to invalid connection parameters!");
		ros::shutdown();
	}

	return;
}

// Store the last INS reading as a CPose3DPDFGaussian
// Note: We are assuming that INS readings happen at a very high frequency.
//       Therefore, interpolation between INS poses is not needed.
void AcousticLoc::insCallback(const nav_msgs::Odometry& msg)
{
	newest_ins_pose_ = conversions::odometry_ros2mrpt(msg);
	if (robot_pose_counter_ == -1)
	{
		robot_pose_counter_++;
		gtsam::Pose3 pose;
		gtsam::Matrix6 cov;
		// conversions::pose_mrpt2gtsam(newest_ins_pose_, pose, cov);
		mrpt::gtsam_wrappers::to_gtsam_se3_cov6(newest_ins_pose_, pose, cov);

		graph_.addExpressionFactor(
			gtsam::noiseModel::Gaussian::Covariance(cov), pose,
			gtsam::Pose3_('x', robot_pose_counter_));
		initial_estimate_.insert(
			gtsam::Symbol('x', robot_pose_counter_), pose);
		last_ins_pose_ = newest_ins_pose_;
	}
}

// This is the main callback of the node.
// When we receive a message with the transformation of an aruco wrt the camera,
// we add to the graph:
//    - A new node with the current robot pose
//    - If the aruco had not been previously seen, a node with the aruco pose
//    - A "between" factor (odometry) between the current and last robot poses
//    (computed with MRPT)
//    - A "bearingRange" expression factor between the current robot pose node
//    and the observed landmark node
// Finally, the graph is optimized after every observation and the results
// published
void AcousticLoc::rangeCallback(
	const stonefish_ros::BeaconInfo& range_info)
{
	if (range_info.range == -1) return;
	if (robot_pose_counter_ == -1) return;
    printf("New Ping registered!\n");
	double new_ping_stamp = range_info.header.stamp.toSec();
	if ((last_ping_stamp_ > 0) and
		(new_ping_stamp - last_ping_stamp_ < 0.05))
		return;

	std::string ping_frame = range_info.header.frame_id;
	char ping_index_string = ping_frame[ping_frame.length() - 1];
	int ping_index = ping_index_string - '1';

	last_ping_stamp_ = new_ping_stamp;

	robot_pose_counter_++;

	// The position of the fiducial relative to the camera.
	// We are currently not using the orientation of the aruco but
	// should be easy to implement if needed (betweenfactor).
		int landmark_id = range_info.beacon_id;
		bool new_landmark = false;
        
		// If landmark already seen
		if (std::find(
				landmark_id_vector_.begin(), landmark_id_vector_.end(),
				landmark_id) != landmark_id_vector_.end())
		{
			new_landmark = false;
		}
		else
		{
			new_landmark = true;
			landmark_id_vector_.push_back(landmark_id);
		}
		double measurement =range_info.range;
				//conversions::point_mrpt2gtsam(point_wrt_modem)));

		double sq_distance_to_landmark = range_info.range;
        
		// TODO: Which noise to add? Proportional to squared distance?
		gtsam::noiseModel::Gaussian::shared_ptr 
        measurement_noise = gtsam::noiseModel::Isotropic::Sigma(1, 0.1 * sq_distance_to_landmark);
		
		// Create landmark factor
		
       
		graph_.emplace_shared<gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3>>(
          gtsam::Symbol('x', robot_pose_counter_), gtsam::Symbol('l', landmark_id), measurement, measurement_noise);
		if (new_landmark)
			initial_estimate_.insert(
				gtsam::Symbol('l', landmark_id), gtsam::Point3());
		

	// Create odometry node and factor

	mrpt::poses::CPose3DPDFGaussian displacement_mrpt =
		conversions::relativeDisplacementDiagCov(
			last_ins_pose_, newest_ins_pose_);

	gtsam::Pose3 displacement;
	gtsam::Matrix6 cov_displacement;
	mrpt::gtsam_wrappers::to_gtsam_se3_cov6(
		displacement_mrpt, displacement, cov_displacement);

	gtsam::noiseModel::Gaussian::shared_ptr displacement_uncertainty =
		gtsam::noiseModel::Gaussian::Covariance(cov_displacement);
	//printf("displacement uncertainty!");
    //printf(displacement_uncertainty);
	graph_.addExpressionFactor(
		displacement_uncertainty, displacement,
		gtsam::between(
			gtsam::Pose3_('x', robot_pose_counter_ - 1),
			gtsam::Pose3_('x', robot_pose_counter_)));
	initial_estimate_.insert(
		gtsam::Symbol('x', robot_pose_counter_),
		mrpt::gtsam_wrappers::toPose3(newest_ins_pose_.mean));

	// Update variables for next iteration
	last_ins_pose_ = newest_ins_pose_;

	if (debug_) graph_.print("Graph");
    
	gtsam::Values result;
    
	if (use_isam_)
	{   
        
		isam2_->update(graph_, initial_estimate_);
		isam2_->update();
		result = isam2_->calculateEstimate();
        
		// reset the graph
		graph_.resize(0);
		initial_estimate_.clear();
	}
	else
	{   
		gtsam::LevenbergMarquardtOptimizer optimizer(graph_, initial_estimate_);
		result = optimizer.optimize();
	}

	std::cout << "************* Landmark poses *************" << std::endl;
	for (size_t i = 0; i < landmark_id_vector_.size(); i++)
		result.at(gtsam::Symbol('l', landmark_id_vector_[i]))
			.print(std::to_string(landmark_id_vector_[i]));
	std::cout << "******************************************" << std::endl;

	// Calculate and print marginal covariances for all variables
	/*if (robot_pose_counter_>4){
		gtsam::Marginals marginals(graph_, result);
	}*/
	// Publish to visualize on RVIZ
	//   Robot poses
	nav_msgs::Path poses;
	for (size_t i = 0; i < robot_pose_counter_ + 1; i++)
	{
		gtsam::Pose3 pose_robot_i =
			result.at(gtsam::Symbol('x', i)).cast<gtsam::Pose3>();
		geometry_msgs::PoseStamped p;
		p.header.stamp = ros::Time::now();
		p.header.frame_id = rviz_ins_frame_;
		p.header.seq = i;
        std::cout << pose_robot_i.translation()[0]<<" "<<pose_robot_i.translation()[1]<<" "<<pose_robot_i.translation()[2]<<std::endl;
		Eigen::Isometry3d eigen_pose;
		eigen_pose.setIdentity();
		eigen_pose.translate(Eigen::Vector3d(
			pose_robot_i.translation()[0], pose_robot_i.translation()[1],
			pose_robot_i.translation()[2]));
		eigen_pose.rotate(pose_robot_i.rotation().matrix());
		tf::poseEigenToMsg(eigen_pose, p.pose);
		poses.poses.push_back(p);
	}
	poses.header.stamp = ros::Time::now();
	poses.header.frame_id = rviz_ins_frame_;
	publisher_robot_optimized_path_.publish(poses);

	//   Landmark poses
	visualization_msgs::Marker m;
	m.type = m.SPHERE_LIST;
	m.header.stamp = ros::Time::now();
	m.header.frame_id = rviz_ins_frame_;

	m.color.a = 1.0;
	m.color.r = 1.0;
	m.scale.x = 0.2;
	m.scale.y = 0.2;
	m.scale.z = 0.02;
	m.action = m.ADD;
	for (size_t i = 0; i < landmark_id_vector_.size(); i++)
	{
		gtsam::Point3 landmark_i =
			result.at(gtsam::Symbol('l', landmark_id_vector_[i]))
				.cast<gtsam::Point3>();
		geometry_msgs::Point gp;
		gp.x = landmark_i.x();
		gp.y = landmark_i.y();
		gp.z = landmark_i.z();
		m.points.push_back(gp);
	}
	publisher_landmarks_.publish(m);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "acoustic_loc");
	ros::NodeHandle nh("~");

	AcousticLoc node(nh);

	ros::spin();

	return 0;
}
