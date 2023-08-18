#include <ahrs_driver.h>
namespace RobotSwitch
{
  RobotSwitchBringup::RobotSwitchBringup() : frist_sn_(false)
  {
    ros::NodeHandle pravite_nh("~");

    // topic_name & frame_id  加载参数服务器
    pravite_nh.param("debug", if_debug_, false);
    pravite_nh.param("device_type", device_type_, 1); // default: single imu
    pravite_nh.param("imu_topic", imu_topic_, std::string("/imu"));
    pravite_nh.param("imu_frame", imu_frame_id_, std::string("imu"));
    pravite_nh.param("mag_pose_2d_topic", mag_pose_2d_topic_, std::string("/mag_pose_2d"));
    pravite_nh.param("Euler_angles_pub_", Euler_angles_topic_, std::string("/euler_angles"));
    pravite_nh.param("Magnetic_pub_", Magnetic_topic_, std::string("/magnetic"));
    pravite_nh.param("gps_topic_", gps_topic_, std::string("/gps/fix"));
    pravite_nh.param("twist_topic_", twist_topic_, std::string("/system_speed"));
    pravite_nh.param("NED_odom_topic_", NED_odom_topic_, std::string("/NED_odometry"));

    // serial
    // ahrs
    pravite_nh.param("ahrs_port", ahrs_serial_port_, std::string("/dev/ttyUSB0"));
    pravite_nh.param("ahrs_baud", ahrs_serial_baud_, 921600);

    // publisher  创建发布对象
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic_.c_str(), 10);
    mag_pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>(mag_pose_2d_topic_.c_str(), 10);
    Euler_angles_pub_ = nh_.advertise<geometry_msgs::Vector3>(Euler_angles_topic_.c_str(), 10);
    Magnetic_pub_ = nh_.advertise<geometry_msgs::Vector3>(Magnetic_topic_.c_str(), 10);
    gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>(gps_topic_.c_str(), 10);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>(twist_topic_.c_str(), 10);
    NED_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(NED_odom_topic_.c_str(), 10);

    imu_velocity_publisher = nh_.advertise<geometry_msgs::Twist>("/ahrs_velocity", 10);
    // qtn_publisher = nh_.advertise<geometry_msgs::PoseStamped>("/qtn_pose", 10);
    qtn_publisher = nh_.advertise<geometry_msgs::PoseStamped>("/cartesian_impedance_controller/desired_pose", 10);

    // qtn_publisher = nh_.advertise<geometry_msgs::Pose>("/hiro_panda/goto_pose", 10);

    // setp up serial  设置串口参数并打开串口
     try
    {
        serial_init(&ahrs_serial_, ahrs_serial_port_, ahrs_serial_baud_, ahrs_serial_timeout_);
    }
     catch (serial::IOException &e)  // 抓取异常
     {
       ROS_ERROR_STREAM("Unable to open port ");
       exit(0);
     }
    processLoop();
  }

  RobotSwitchBringup::~RobotSwitchBringup()
  {
    if (ahrs_serial_.isOpen())
      ahrs_serial_.close();
  }

  void RobotSwitchBringup::processLoop()
  {
    static bool initialized = false;

    ROS_INFO("RobotSwitchBringup::processLoop: start");
    while (ros::ok())
    {
      if (!ahrs_serial_.isOpen())
      {
        ROS_WARN("serial unopen");
      }
      // check head start  检查起始 数据帧头
      uint8_t check_head[1] = {0xff};
      size_t head_s = ahrs_serial_.read(check_head, 1);
      if (if_debug_)
      {
        if (head_s != 1)
        {
          ROS_ERROR("Read serial port time out! can't read pack head.");
        }
        std::cout << std::endl;
        std::cout << "check_head: " << std::hex << (int)check_head[0] << std::dec << std::endl;
      }
      if (check_head[0] != FRAME_HEAD)
      {
        continue;
      }
      // check head type   检查数据类型
      uint8_t head_type[1] = {0xff};
      size_t type_s = ahrs_serial_.read(head_type, 1);
      if (if_debug_)
      {
        std::cout << "head_type:  " << std::hex << (int)head_type[0] << std::dec << std::endl;
      }
      if (head_type[0] != TYPE_IMU && head_type[0] != TYPE_AHRS && head_type[0] != TYPE_INSGPS && head_type[0] != TYPE_GEODETIC_POS && head_type[0] != 0x50 && head_type[0] != TYPE_GROUND && head_type[0] != 0xff)
      {
        ROS_WARN("head_type error: %02X", head_type[0]);
        continue;
      }
      // check head length  检查对应数据类型的长度是否符合
      uint8_t check_len[1] = {0xff};
      size_t len_s = ahrs_serial_.read(check_len, 1);
      if (if_debug_)
      {
        std::cout << "check_len: " << std::dec << (int)check_len[0] << std::endl;
      }
      if (head_type[0] == TYPE_IMU && check_len[0] != IMU_LEN)
      {
        ROS_WARN("head_len error (imu)");
        continue;
      }
      else if (head_type[0] == TYPE_AHRS && check_len[0] != AHRS_LEN)
      {
        ROS_WARN("head_len error (ahrs)");
        continue;
      }
      else if (head_type[0] == TYPE_INSGPS && check_len[0] != INSGPS_LEN)
      {
        ROS_WARN("head_len error (insgps)");
        continue;
      }
      else if (head_type[0] == TYPE_GEODETIC_POS && check_len[0] != GEODETIC_POS_LEN)
      {
        ROS_WARN("head_len error (GEODETIC_POS)");
        continue;
      }
      else if (head_type[0] == TYPE_GROUND || head_type[0] == 0x50) // 未知数据，防止记录失败
      {
        uint8_t ground_sn[1];
        size_t ground_sn_s = ahrs_serial_.read(ground_sn, 1);
        if (++read_sn_ != ground_sn[0])
        {
          if (ground_sn[0] < read_sn_)
          {
            if (if_debug_)
            {
              ROS_WARN("detected sn lost.");
            }
            sn_lost_ += 256 - (int)(read_sn_ - ground_sn[0]);
            read_sn_ = ground_sn[0];
            // continue;
          }
          else
          {
            if (if_debug_)
            {
              ROS_WARN("detected sn lost.");
            }
            sn_lost_ += (int)(ground_sn[0] - read_sn_);
            read_sn_ = ground_sn[0];
            // continue;
          }
        }
        uint8_t ground_ignore[500];
        size_t ground_ignore_s = ahrs_serial_.read(ground_ignore, (check_len[0] + 4));
        continue;
      }
      // read head sn  检查sn 流水序号
      uint8_t check_sn[1] = {0xff};
      size_t sn_s = ahrs_serial_.read(check_sn, 1);

      uint8_t head_crc8[1] = {0xff};
      size_t crc8_s = ahrs_serial_.read(head_crc8, 1);

      uint8_t head_crc16_H[1] = {0xff};
      uint8_t head_crc16_L[1] = {0xff};
      size_t crc16_H_s = ahrs_serial_.read(head_crc16_H, 1);
      size_t crc16_L_s = ahrs_serial_.read(head_crc16_L, 1);

      if (if_debug_)
      {
        std::cout << "check_sn: " << std::hex << (int)check_sn[0] << std::dec << std::endl;
        std::cout << "head_crc8: " << std::hex << (int)head_crc8[0] << std::dec << std::endl;
        std::cout << "head_crc16_H: " << std::hex << (int)head_crc16_H[0] << std::dec << std::endl;
        std::cout << "head_crc16_L: " << std::hex << (int)head_crc16_L[0] << std::dec << std::endl;
      }
      // put header & check crc8 & count sn lost
      // check crc8 进行crc8数据校验
      if (head_type[0] == TYPE_IMU)
      {
        imu_frame_.frame.header.header_start = check_head[0];
        imu_frame_.frame.header.data_type = head_type[0];
        imu_frame_.frame.header.data_size = check_len[0];
        imu_frame_.frame.header.serial_num = check_sn[0];
        imu_frame_.frame.header.header_crc8 = head_crc8[0];
        imu_frame_.frame.header.header_crc16_h = head_crc16_H[0];
        imu_frame_.frame.header.header_crc16_l = head_crc16_L[0];

        uint8_t CRC8 = CRC8_Table(imu_frame_.read_buf.frame_header, 4);
        if (CRC8 != imu_frame_.frame.header.header_crc8)
        {
          ROS_WARN("header_crc8 error");
          continue;
        }
        if (!frist_sn_)
        {
          read_sn_ = imu_frame_.frame.header.serial_num - 1;
          frist_sn_ = true;
        }
        // check sn
        RobotSwitchBringup::ahrs_checkSN(TYPE_IMU);
      }
      else if (head_type[0] == TYPE_AHRS)
      {
        ahrs_frame_.frame.header.header_start = check_head[0];
        ahrs_frame_.frame.header.data_type = head_type[0];
        ahrs_frame_.frame.header.data_size = check_len[0];
        ahrs_frame_.frame.header.serial_num = check_sn[0];
        ahrs_frame_.frame.header.header_crc8 = head_crc8[0];
        ahrs_frame_.frame.header.header_crc16_h = head_crc16_H[0];
        ahrs_frame_.frame.header.header_crc16_l = head_crc16_L[0];
        uint8_t CRC8 = CRC8_Table(ahrs_frame_.read_buf.frame_header, 4);
        if (CRC8 != ahrs_frame_.frame.header.header_crc8)
        {
          ROS_WARN("header_crc8 error");
          continue;
        }
        if (!frist_sn_)
        {
          read_sn_ = ahrs_frame_.frame.header.serial_num - 1;
          frist_sn_ = true;
        }
        // check sn
        RobotSwitchBringup::ahrs_checkSN(TYPE_AHRS);
      }
      else if (head_type[0] == TYPE_INSGPS)
      {
        insgps_frame_.frame.header.header_start = check_head[0];
        insgps_frame_.frame.header.data_type = head_type[0];
        insgps_frame_.frame.header.data_size = check_len[0];
        insgps_frame_.frame.header.serial_num = check_sn[0];
        insgps_frame_.frame.header.header_crc8 = head_crc8[0];
        insgps_frame_.frame.header.header_crc16_h = head_crc16_H[0];
        insgps_frame_.frame.header.header_crc16_l = head_crc16_L[0];
        uint8_t CRC8 = CRC8_Table(insgps_frame_.read_buf.frame_header, 4);
        if (CRC8 != insgps_frame_.frame.header.header_crc8)
        {
          ROS_WARN("header_crc8 error");
          continue;
        }
        else if (if_debug_)
        {
          std::cout << "header_crc8 matched." << std::endl;
        }

        RobotSwitchBringup::ahrs_checkSN(TYPE_INSGPS);
      }
      else if (head_type[0] == TYPE_GEODETIC_POS)
      {
        Geodetic_Position_frame_.frame.header.header_start = check_head[0];
        Geodetic_Position_frame_.frame.header.data_type = head_type[0];
        Geodetic_Position_frame_.frame.header.data_size = check_len[0];
        Geodetic_Position_frame_.frame.header.serial_num = check_sn[0];
        Geodetic_Position_frame_.frame.header.header_crc8 = head_crc8[0];
        Geodetic_Position_frame_.frame.header.header_crc16_h = head_crc16_H[0];
        Geodetic_Position_frame_.frame.header.header_crc16_l = head_crc16_L[0];
        uint8_t CRC8 = CRC8_Table(Geodetic_Position_frame_.read_buf.frame_header, 4);
        if (CRC8 != Geodetic_Position_frame_.frame.header.header_crc8)
        {
          ROS_WARN("header_crc8 error");
          continue;
        }
        if (!frist_sn_)
        {
          read_sn_ = Geodetic_Position_frame_.frame.header.serial_num - 1;
          frist_sn_ = true;
        }

        RobotSwitchBringup::ahrs_checkSN(TYPE_GEODETIC_POS);
      }
      // check crc16 进行crc16数据校验
      if (head_type[0] == TYPE_IMU)
      {
        uint16_t head_crc16_l = imu_frame_.frame.header.header_crc16_l;
        uint16_t head_crc16_h = imu_frame_.frame.header.header_crc16_h;
        uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
        size_t data_s = ahrs_serial_.read(imu_frame_.read_buf.read_msg, (IMU_LEN + 1)); // 48+1
        // if (if_debug_){
        //   for (size_t i = 0; i < (IMU_LEN + 1); i++)
        //   {
        //     std::cout << std::hex << (int)imu_frame_.read_buf.read_msg[i] << " ";
        //   }
        //   std::cout << std::dec << std::endl;
        // }
        uint16_t CRC16 = CRC16_Table(imu_frame_.frame.data.data_buff, IMU_LEN);
        if (if_debug_)
        {
          std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
          std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
          std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
          std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
          bool if_right = ((int)head_crc16 == (int)CRC16);
          std::cout << "if_right: " << if_right << std::endl;
        }

        if (head_crc16 != CRC16)
        {
          ROS_WARN("check crc16 faild(imu).");
          continue;
        }
        else if (imu_frame_.frame.frame_end != FRAME_END)
        {
          ROS_WARN("check frame end.");
          continue;
        }
      }
      else if (head_type[0] == TYPE_AHRS)
      {
        uint16_t head_crc16_l = ahrs_frame_.frame.header.header_crc16_l;
        uint16_t head_crc16_h = ahrs_frame_.frame.header.header_crc16_h;
        uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
        size_t data_s = ahrs_serial_.read(ahrs_frame_.read_buf.read_msg, (AHRS_LEN + 1)); // 48+1
        // if (if_debug_){
        //   for (size_t i = 0; i < (AHRS_LEN + 1); i++)
        //   {
        //     std::cout << std::hex << (int)ahrs_frame_.read_buf.read_msg[i] << " ";
        //   }
        //   std::cout << std::dec << std::endl;
        // }
        uint16_t CRC16 = CRC16_Table(ahrs_frame_.frame.data.data_buff, AHRS_LEN);
        if (if_debug_)
        {
          std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
          std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
          std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
          std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
          bool if_right = ((int)head_crc16 == (int)CRC16);
          std::cout << "if_right: " << if_right << std::endl;
        }

        if (head_crc16 != CRC16)
        {
          ROS_WARN("check crc16 faild(ahrs).");
          continue;
        }
        else if (ahrs_frame_.frame.frame_end != FRAME_END)
        {
          ROS_WARN("check frame end.");
          continue;
        }
      }
      else if (head_type[0] == TYPE_INSGPS)
      {
        uint16_t head_crc16_l = insgps_frame_.frame.header.header_crc16_l;
        uint16_t head_crc16_h = insgps_frame_.frame.header.header_crc16_h;
        uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
        size_t data_s = ahrs_serial_.read(insgps_frame_.read_buf.read_msg, (INSGPS_LEN + 1)); // 48+1
        // if (if_debug_){
        //   for (size_t i = 0; i < (AHRS_LEN + 1); i++)
        //   {
        //     std::cout << std::hex << (int)ahrs_frame_.read_buf.read_msg[i] << " ";
        //   }
        //   std::cout << std::dec << std::endl;
        // }
        uint16_t CRC16 = CRC16_Table(insgps_frame_.frame.data.data_buff, INSGPS_LEN);
        if (if_debug_)
        {
          std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
          std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
          std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
          std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
          bool if_right = ((int)head_crc16 == (int)CRC16);
          std::cout << "if_right: " << if_right << std::endl;
        }

        if (head_crc16 != CRC16)
        {
          ROS_WARN("check crc16 faild(ahrs).");
          continue;
        }
        else if (insgps_frame_.frame.frame_end != FRAME_END)
        {
          ROS_WARN("check frame end.");
          continue;
        }
      }
      else if (head_type[0] == TYPE_GEODETIC_POS)
      {
        uint16_t head_crc16_l = Geodetic_Position_frame_.frame.header.header_crc16_l;
        uint16_t head_crc16_h = Geodetic_Position_frame_.frame.header.header_crc16_h;
        uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
        size_t data_s = ahrs_serial_.read(Geodetic_Position_frame_.read_buf.read_msg, (GEODETIC_POS_LEN + 1)); // 24+1
        uint16_t CRC16 = CRC16_Table(Geodetic_Position_frame_.frame.data.data_buff, GEODETIC_POS_LEN);
        if (if_debug_)
        {
          std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
          std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
          std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
          std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
          bool if_right = ((int)head_crc16 == (int)CRC16);
          std::cout << "if_right: " << if_right << std::endl;
        }

        if (head_crc16 != CRC16)
        {
          ROS_WARN("check crc16 faild(gps).");
          continue;
        }
        else if (Geodetic_Position_frame_.frame.frame_end != FRAME_END)
        {
          ROS_WARN("check frame end.");
          continue;
        }
      }
      // 读取IMU数据进行解析，并发布相关话题
      if (head_type[0] == TYPE_IMU)
      {
        // publish imu topic
        sensor_msgs::Imu imu_data;
        imu_data.header.stamp = ros::Time::now();
        imu_data.header.frame_id = imu_frame_id_.c_str();
        Eigen::Quaterniond q_ahrs(ahrs_frame_.frame.data.data_pack.Qw,
                                  ahrs_frame_.frame.data.data_pack.Qx,
                                  ahrs_frame_.frame.data.data_pack.Qy,
                                  ahrs_frame_.frame.data.data_pack.Qz);
        Eigen::Quaterniond q_r =
            Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0.00000, Eigen::Vector3d::UnitX());
        Eigen::Quaterniond q_rr =
            Eigen::AngleAxisd(0.00000, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(0.00000, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitX());
        Eigen::Quaterniond q_xiao_rr =
            Eigen::AngleAxisd(PI / 2, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(0.00000, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitX());
        if (device_type_ == 0) // 未经变换的原始数据
        {
          imu_data.orientation.w = ahrs_frame_.frame.data.data_pack.Qw;
          imu_data.orientation.x = ahrs_frame_.frame.data.data_pack.Qx;
          imu_data.orientation.y = ahrs_frame_.frame.data.data_pack.Qy;
          imu_data.orientation.z = ahrs_frame_.frame.data.data_pack.Qz;
          imu_data.angular_velocity.x = imu_frame_.frame.data.data_pack.gyroscope_x;
          imu_data.angular_velocity.y = imu_frame_.frame.data.data_pack.gyroscope_y;
          imu_data.angular_velocity.z = imu_frame_.frame.data.data_pack.gyroscope_z;
          imu_data.linear_acceleration.x = imu_frame_.frame.data.data_pack.accelerometer_x;
          imu_data.linear_acceleration.y = imu_frame_.frame.data.data_pack.accelerometer_y;
          imu_data.linear_acceleration.z = imu_frame_.frame.data.data_pack.accelerometer_z;
        }
        else if (device_type_ == 1) // imu单品ROS标准下的坐标变换
        {

          Eigen::Quaterniond q_out = q_r * q_ahrs * q_rr;
          imu_data.orientation.w = q_out.w();
          imu_data.orientation.x = q_out.x();
          imu_data.orientation.y = q_out.y();
          imu_data.orientation.z = q_out.z();
          imu_data.angular_velocity.x = imu_frame_.frame.data.data_pack.gyroscope_x;
          imu_data.angular_velocity.y = -imu_frame_.frame.data.data_pack.gyroscope_y;
          imu_data.angular_velocity.z = -imu_frame_.frame.data.data_pack.gyroscope_z;
          imu_data.linear_acceleration.x = imu_frame_.frame.data.data_pack.accelerometer_x;
          imu_data.linear_acceleration.y = -imu_frame_.frame.data.data_pack.accelerometer_y;
          imu_data.linear_acceleration.z = -imu_frame_.frame.data.data_pack.accelerometer_z;
        }
        imu_pub_.publish(imu_data);
        //   机械臂四元数:
        // w: -4.92282e-05
        // x: 0.999998
        // y: -0.000260781
        // z: -0.000265499
        Eigen::Quaterniond controller_quaternion(imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z);

        if (!initialized) {
            Eigen::Quaterniond robot_initial_quaternion(0, 1, 0, 0);
            calibration_quaternion = robot_initial_quaternion * controller_quaternion.conjugate();
            initialized = true;
        }
        Eigen::Quaterniond robot_quaternion = calibration_quaternion * controller_quaternion;
        // clang
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.pose.position.x = 0.3069;
        pose_msg.pose.position.y = 0.0;
        pose_msg.pose.position.z = 0.4866;

        // pose_msg.pose.position.x = 0.4;
        // pose_msg.pose.position.y = 0.0;
        // pose_msg.pose.position.z = 0.6;

        pose_msg.pose.orientation.x = robot_quaternion.x();
        pose_msg.pose.orientation.y = robot_quaternion.y();
        pose_msg.pose.orientation.z = robot_quaternion.z();
        pose_msg.pose.orientation.w = robot_quaternion.w();
        qtn_publisher.publish(pose_msg);
      }
    }
    ros::waitForShutdown();
  }
          // geometry_msgs::Twist fk_cartesian_msg;
          // fk_cartesian_msg.angular.x = imu_data.angular_velocity.x * 0.5f;
          // fk_cartesian_msg.angular.y = imu_data.angular_velocity.y * 0.5f;
          // fk_cartesian_msg.angular.z = imu_data.angular_velocity.z * 0.5f;
          // imu_velocity_publisher.publish(fk_cartesian_msg);

  void RobotSwitchBringup::ahrs_magCalculateYaw(double roll, double pitch, double &magyaw, double magx, double magy, double magz)
  {
    double temp1 = magy * cos(roll) + magz * sin(roll);
    double temp2 = magx * cos(pitch) + magy * sin(pitch) * sin(roll) - magz * sin(pitch) * cos(roll);
    magyaw = atan2(-temp1, temp2);
    if (magyaw < 0)
    {
      magyaw = magyaw + 2 * PI;
    }
    // return magyaw;
  }

  void RobotSwitchBringup::ahrs_checkSN(int type)
  {
    switch (type)
    {
    case TYPE_IMU:
      if (++read_sn_ != imu_frame_.frame.header.serial_num)
      {
        if (imu_frame_.frame.header.serial_num < read_sn_)
        {
          sn_lost_ += 256 - (int)(read_sn_ - imu_frame_.frame.header.serial_num);
          if (if_debug_)
          {
            ROS_WARN("detected sn lost.");
          }
        }
        else
        {
          sn_lost_ += (int)(imu_frame_.frame.header.serial_num - read_sn_);
          if (if_debug_)
          {
            ROS_WARN("detected sn lost.");
          }
        }
      }
      read_sn_ = imu_frame_.frame.header.serial_num;
      break;

    case TYPE_AHRS:
      if (++read_sn_ != ahrs_frame_.frame.header.serial_num)
      {
        if (ahrs_frame_.frame.header.serial_num < read_sn_)
        {
          sn_lost_ += 256 - (int)(read_sn_ - ahrs_frame_.frame.header.serial_num);
          if (if_debug_)
          {
            ROS_WARN("detected sn lost.");
          }
        }
        else
        {
          sn_lost_ += (int)(ahrs_frame_.frame.header.serial_num - read_sn_);
          if (if_debug_)
          {
            ROS_WARN("detected sn lost.");
          }
        }
      }
      read_sn_ = ahrs_frame_.frame.header.serial_num;
      break;

    case TYPE_INSGPS:
      if (++read_sn_ != insgps_frame_.frame.header.serial_num)
      {
        if (insgps_frame_.frame.header.serial_num < read_sn_)
        {
          sn_lost_ += 256 - (int)(read_sn_ - insgps_frame_.frame.header.serial_num);
          if (if_debug_)
          {
            ROS_WARN("detected sn lost.");
          }
        }
        else
        {
          sn_lost_ += (int)(insgps_frame_.frame.header.serial_num - read_sn_);
          if (if_debug_)
          {
            ROS_WARN("detected sn lost.");
          }
        }
      }
      read_sn_ = insgps_frame_.frame.header.serial_num;
      break;

    case TYPE_GEODETIC_POS:
      if (++read_sn_ != Geodetic_Position_frame_.frame.header.serial_num)
      {
        if (Geodetic_Position_frame_.frame.header.serial_num < read_sn_)
        {
          sn_lost_ += 256 - (int)(read_sn_ - Geodetic_Position_frame_.frame.header.serial_num);
          if (if_debug_)
          {
            ROS_WARN("detected sn lost.");
          }
        }
        else
        {
          sn_lost_ += (int)(Geodetic_Position_frame_.frame.header.serial_num - read_sn_);
          if (if_debug_)
          {
            ROS_WARN("detected sn lost.");
          }
        }
      }
      read_sn_ = Geodetic_Position_frame_.frame.header.serial_num;
      break;

    default:
      break;
    }
  }

  void RobotSwitchBringup::serial_init(serial::Serial *serial_, std::string _port_, int _baud_, int _timeout_)
  {
    serial_->setPort(_port_);
    serial_->setBaudrate(_baud_);
    serial_->setFlowcontrol(serial::flowcontrol_none);
    serial_->setParity(serial::parity_none); // default is parity_none
    serial_->setStopbits(serial::stopbits_one);
    serial_->setBytesize(serial::eightbits);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(_timeout_);
    serial_->setTimeout(time_out);
    serial_->open();
  }

} // namespace RobotSwitch

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robotswitch_bringup");
  RobotSwitch::RobotSwitchBringup bp;

  return 0;
}
