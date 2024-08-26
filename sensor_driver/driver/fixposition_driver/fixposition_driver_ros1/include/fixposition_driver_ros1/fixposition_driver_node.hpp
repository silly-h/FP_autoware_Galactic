/**
 *  @file
 *  @brief Declaration of FixpositionDriver ROS1 Node
 *
 * \verbatim
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 * \endverbatim
 *
 */

#ifndef __FIXPOSITION_DRIVER_ROS1_FIXPOSITION_DRIVER_NODE_
#define __FIXPOSITION_DRIVER_ROS1_FIXPOSITION_DRIVER_NODE_

/* SYSTEM / STL */
#include <termios.h>

#include <unordered_map>

/* ROS */
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

/* FIXPOSITION */
#include <fixposition_driver_lib/fixposition_driver.hpp>

/* PACKAGE */
#include <fixposition_driver_ros1/Speed.h>
#include <fixposition_driver_ros1/VRTK.h>

namespace fixposition {
class FixpositionDriverNode : public FixpositionDriver {
   public:
    /**
     * @brief Construct a new Fixposition Driver Node object
     *
     * @param[in] params
     */
    FixpositionDriverNode(const FixpositionDriverParams& params);

    void Run();

    void RegisterObservers();

    void WsCallback(const fixposition_driver_ros1::SpeedConstPtr& msg);

    struct NmeaMessage {
        GpggaData gpgga;
        GpzdaData gpzda;
        GprmcData gprmc;
        
        /**
         * @brief Construct a new Fixposition Driver Node object
         */
        bool checkEpoch() {
            if (gpgga.valid && gpgga.valid && gpgga.valid) {
                if ((gpgga.time.compare(gpzda.time) == 0) && (gpgga.time.compare(gprmc.time) == 0)) {
                    return true;
                } else {
                    return false;
                }
            } else {
                return false;
            }
        }  
    };

   private:
    /**
     * @brief Observer Functions to publish NavSatFix from BestGnssPos
     *
     * @param[in] header
     * @param[in] payload
     */
    void BestGnssPosToPublishNavSatFix(const Oem7MessageHeaderMem* header, const BESTGNSSPOSMem* payload);

    /**
     * @brief Observer Function to publish NMEA message from GPGGA, GPRMC, and GPZDA once the GNSS epoch transmission is complete
     *
     * @param[in] data
     */
    void PublishNmea(NmeaMessage data);

    ros::NodeHandle nh_;
    ros::Subscriber ws_sub_;  //!< wheelspeed message subscriber

    ros::Publisher rawimu_pub_;
    ros::Publisher corrimu_pub_;
    ros::Publisher navsatfix_pub_;
    ros::Publisher navsatfix_gnss1_pub_;
    ros::Publisher navsatfix_gnss2_pub_;
    ros::Publisher nmea_pub_;
    ros::Publisher odometry_pub_;        //!< ECEF Odometry
    ros::Publisher odometry_smooth_pub_; //!< ECEF Smooth Odometry
    ros::Publisher poiimu_pub_;          //!< Bias corrected IMU
    ros::Publisher vrtk_pub_;            //!< VRTK message
    ros::Publisher odometry_enu0_pub_;   //!< ENU0 Odometry
    ros::Publisher eul_pub_;             //!< Euler angles Yaw-Pitch-Roll in local ENU
    ros::Publisher eul_imu_pub_;         //!< Euler angles Pitch-Roll as estimated from the IMU in local horizontal

    NmeaMessage nmea_message_;

    tf2_ros::TransformBroadcaster br_;
    tf2_ros::StaticTransformBroadcaster static_br_;
};

}  // namespace fixposition

#endif
