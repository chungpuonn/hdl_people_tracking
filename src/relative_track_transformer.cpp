#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <hdl_people_tracking/TrackArray.h>
#include <hdl_people_tracking/Track.h>
#include <hdl_people_tracking/Cluster.h>
#include <hdl_people_tracking/ExtendedTrackArray.h>
#include <hdl_people_tracking/ExtendedTrack.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <boost/array.hpp>
#include <cmath>

class RelativeTrackTransformer {
public:
    RelativeTrackTransformer() {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        pnh.param<std::string>("tracks_topic", tracks_topic_, "/hdl_people_tracking_nodelet/tracks");
        pnh.param<std::string>("odom_topic", odom_topic_, "/odom");
        pnh.param<std::string>("output_topic", output_topic_, "/hdl_people_tracking_nodelet/tracks_relative");

        tracks_sub_ = nh.subscribe(tracks_topic_, 10, &RelativeTrackTransformer::tracksCallback, this);
        odom_sub_   = nh.subscribe(odom_topic_, 10, &RelativeTrackTransformer::odomCallback, this);

        tracks_pub_ = nh.advertise<hdl_people_tracking::ExtendedTrackArray>(output_topic_, 10);

        odom_received_ = false;
    }

private:
    ros::Subscriber tracks_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher  tracks_pub_;

    std::string tracks_topic_;
    std::string odom_topic_;
    std::string output_topic_;

    geometry_msgs::Point robot_position_;
    double robot_yaw_;
    bool odom_received_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        robot_position_ = msg->pose.pose.position;
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, robot_yaw_);
        odom_received_ = true;
    }

    template<typename Array9>
    Array9 rotateCovariance(const Array9& cov, double cos_theta, double sin_theta) {
        Array9 rotated_cov;
        double R[3][3] = {
            {cos_theta, -sin_theta, 0},
            {sin_theta,  cos_theta, 0},
            {0,          0,         1}
        };
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                double sum = 0.0;
                for (int k = 0; k < 3; ++k) {
                    for (int l = 0; l < 3; ++l) {
                        sum += R[i][k] * cov[k*3 + l] * R[j][l];
                    }
                }
                rotated_cov[i*3 + j] = sum;
            }
        }
        return rotated_cov;
    }

    void tracksCallback(const hdl_people_tracking::TrackArray::ConstPtr& msg) {
        if (!odom_received_) {
            ROS_WARN_THROTTLE(5.0, "Waiting for odometry data...");
            return;
        }

        hdl_people_tracking::ExtendedTrackArray relative_tracks;
        relative_tracks.header = msg->header;

        double cos_yaw = std::cos(-robot_yaw_);
        double sin_yaw = std::sin(-robot_yaw_);

        for (const auto& track : msg->tracks) {
            hdl_people_tracking::ExtendedTrack relative;

            // Transform and assign position
            double dx = track.pos.x - robot_position_.x;
            double dy = track.pos.y - robot_position_.y;
            double dz = track.pos.z - robot_position_.z;
            relative.pos.x = dx * cos_yaw - dy * sin_yaw;
            relative.pos.y = dx * sin_yaw + dy * cos_yaw;
            relative.pos.z = dz;

            // Transform and assign velocity
            double vx = track.vel.x;
            double vy = track.vel.y;
            double vz = track.vel.z;
            relative.vel.x = vx * cos_yaw - vy * sin_yaw;
            relative.vel.y = vx * sin_yaw + vy * cos_yaw;
            relative.vel.z = vz;

            // Rotate covariances
            relative.pos_cov = rotateCovariance(track.pos_cov, cos_yaw, sin_yaw);
            relative.vel_cov = rotateCovariance(track.vel_cov, cos_yaw, sin_yaw);

            // Copy other fields
            relative.id        = track.id;
            relative.age       = track.age;
            relative.associated = track.associated;

            // Compute and assign bearing
            relative.bearing = std::atan2(relative.pos.y, relative.pos.x);

            relative_tracks.tracks.push_back(relative);
        }

        tracks_pub_.publish(relative_tracks);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "relative_track_transformer");
    RelativeTrackTransformer transformer;
    ros::spin();
    return 0;
}
