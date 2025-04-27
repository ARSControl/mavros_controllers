#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <controller_msgs/FlatTarget.h>

#include <chrono>
#include <signal.h>
#include <Eigen/Dense>
#include <Eigen/Core>
// #include <Eigen/MatrixFunctions>

#define M_PI   3.14159265358979323846  /*pi*/

// using namespace std::chrono_literals;
using std::placeholders::_1;

sig_atomic_t volatile node_shutdown_request = 0;    //signal manually generated when ctrl+c is pressed

class TrajectoryPub
{
    public:
    TrajectoryPub(): nh_priv_("~")
    {
        nh_priv_.getParam("altitude", z0);
        std::cout << "Desired Altitude: " << z0 << std::endl;
        traj_pub_ = nh_.advertise<nav_msgs::Path>("/trajectory_publisher/trajectory", 10);
        flatreferencePub_ = nh_.advertise<controller_msgs::FlatTarget>("/reference/flatsetpoint", 10);
        primitives_pub_ = nh_.advertise<nav_msgs::Path>("/trajectory_publisher/primitiveset", 10);
        timer_ = nh_.createTimer(ros::Duration(0.01), std::bind(&TrajectoryPub::timer_cb, this));
        trajectory_timer_ = nh_.createTimer(ros::Duration(0.1), std::bind(&TrajectoryPub::publishTrajectory, this));

	    odom_sub = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, std::bind(&TrajectoryPub::odom_cb, this, std::placeholders::_1));
        a_x.resize(3);
        a_y.resize(3);
        a_z.resize(3);
    }
    ~TrajectoryPub() {
        ROS_INFO("TrajectoryPub Destructor called.");
    }

    void stop();
    void timer_cb();
    void publishTrajectory();
    void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
    Eigen::Vector3d getPosition(double t);
    Eigen::Vector3d getVelocity(double t);
    Eigen::Vector3d getAcceleration(double t);


private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    ros::Timer timer_;
    ros::Timer trajectory_timer_;
    nav_msgs::Path traj_msg_;
    ros::Publisher traj_pub_;
    ros::Publisher primitives_pub_;
    ros::Publisher flatreferencePub_;
    ros::Subscriber odom_sub;
    nav_msgs::Odometry odom;

    Eigen::VectorXd a_x;
    Eigen::VectorXd a_y;
    Eigen::VectorXd a_z;
    double exec_time = 15.0;
    double wait_time = 10.0;
    double z0 = 2.0;
    double t_start_;
    bool odom_received = false;
    bool just_started = true;
};

void TrajectoryPub::stop()
{
    ROS_INFO("Shutdown required. Closing trajectory publisher...");
    ros::Duration(0.1).sleep();
    ros::shutdown();
}

void TrajectoryPub::odom_cb(const nav_msgs::Odometry::ConstPtr& msg) {
	odom = *msg;
	if (odom.pose.pose.position.z != 0.0) {
		odom_received = true;
	}
}

Eigen::Vector3d TrajectoryPub::getPosition(double t)
{
    Eigen::Vector3d ref_pos;
    double x = a_x(0) + a_x(1) * t + a_x(2) * t * t;
    double y = a_y(0) + a_y(1) * t + a_y(2) * t * t;
    double z = a_z(0) + a_z(1) * t + a_z(2) * t * t;
    ref_pos << x, y, z;
    return ref_pos;
}

Eigen::Vector3d TrajectoryPub::getVelocity(double t)
{
    Eigen::Vector3d ref_vel;
    double x = a_x(1) + 2 * a_x(2) * t;
    double y = a_y(1) + 2 * a_y(2) * t;
    double z = a_z(1) + 2 * a_z(2) * t;
    ref_vel << x, y, z;
    return ref_vel;
}

Eigen::Vector3d TrajectoryPub::getAcceleration(double t)
{
    Eigen::Vector3d ref_acc;
    ref_acc << 2 * a_x(2), 2 * a_y(2), 2 * a_z(2);
    return ref_acc;
}

void TrajectoryPub::publishTrajectory()
{
    auto time_now = ros::Time::now();
    int num_steps = 200;
    traj_msg_.header.frame_id = "map";
    traj_msg_.header.stamp = time_now;
    for (int i = 0; i < num_steps; i++) {
        geometry_msgs::PoseStamped p;
        Eigen::Vector3d pos = getPosition(i / exec_time);
        p.header.stamp = time_now;
        p.header.frame_id = "map";
        p.pose.position.x = pos(0);
        p.pose.position.y = pos(1);
        p.pose.position.z = pos(2);
        traj_msg_.poses.push_back(p);
    }
    traj_pub_.publish(traj_msg_);
    primitives_pub_.publish(traj_msg_);

}

void TrajectoryPub::timer_cb()
{
	if(!odom_received) {
                ROS_INFO("Waiting 4 odom ...");
        	return;
	}

	if(just_started) {
        // trajectory params
        Eigen::Vector3d x0;
        x0 << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z + z0;
        Eigen::Vector3d xf;
        xf << odom.pose.pose.position.x + 5.0, odom.pose.pose.position.y, odom.pose.pose.position.z + z0;
        double y_peak = odom.pose.pose.position.y + 1.5;

        std::cout << "Starting position: " << x0.transpose() << std::endl;
        std::cout << "Final position: " << xf.transpose() << std::endl;

        double t0 = 0.0;
        double tm = exec_time / 2;
        double tf = exec_time;

        Eigen::Matrix3d A;
        A << 1, t0, t0 * t0,
            1, tm, tm * tm,
            1, tf, tf * tf;
        Eigen::Vector3d b;
        b << x0(1), y_peak, xf(1);

        // get coefficients
        a_x << x0(0), (xf(0) - x0(0)) / exec_time, 0.0;
        a_y = A.colPivHouseholderQr().solve(b);
        a_z << odom.pose.pose.position.z + z0, 0.0, 0.0;
	std::cout << "ax: " << a_x.transpose() << std::endl;
	just_started = false;
	t_start_ = ros::Time::now().toSec();
	}


    double t_now = ros::Time::now().toSec() - t_start_;
    controller_msgs::FlatTarget target_msg;
    target_msg.header.frame_id = "map";
    target_msg.header.stamp = ros::Time::now();
    if (t_now < wait_time) {
	    std::cout << "waiting... t: " << t_now << std::endl;
        target_msg.type_mask = 4;
        target_msg.position.z = z0;
        // flatreferencePub_.publish(target_msg);
    } else if (t_now > wait_time && t_now - wait_time < exec_time) {
	    // std::cout << "t : " << t_now << std::endl;
        Eigen::Vector3d refpos = getPosition(t_now-wait_time);
        Eigen::Vector3d refvel = getVelocity(t_now-wait_time);
        Eigen::Vector3d refacc = getAcceleration(t_now-wait_time);
        target_msg.type_mask = 2;       // ignore snap and jerk
        target_msg.position.x = refpos(0);
        target_msg.position.y = refpos(1);
        target_msg.position.z = refpos(2);
	std::cout << "Reference pos: " << refpos.transpose() << std::endl;
        target_msg.velocity.x = refvel(0);
        target_msg.velocity.y = refvel(1);
        target_msg.velocity.z = refvel(2);
        target_msg.acceleration.x = refacc(0);
        target_msg.acceleration.y = refacc(1);
        target_msg.acceleration.z = refacc(2);
        flatreferencePub_.publish(target_msg);
    } else {
	std::cout << "too late: t = " << t_now << std::endl;
    }

    


}


void nodeobj_wrapper_function(int){
    ROS_WARN("signal handler function CALLED");
    node_shutdown_request = 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_publisher", ros::init_options::NoSigintHandler);
    signal(SIGINT, nodeobj_wrapper_function);

    //Controller node_controller;
    auto node_controller = std::make_shared<TrajectoryPub>();

    while (!node_shutdown_request){
        ros::spinOnce();
    }
    node_controller->stop();

    //ros::spin();
    //do pre-shutdown tasks
    if (ros::ok())
    {
        ROS_WARN("ROS HAS NOT BEEN PROPERLY SHUTDOWN, it is being shutdown again.");
        ros::shutdown();
    }

    

    

    return 0;
}
