#include <yaml-cpp/yaml.h>
#include "lidarodom.h"

namespace zjloc
{
#define USE_ANALYTICAL_DERIVATE 1

     lidarodom::lidarodom(/* args */)
     {
          laser_point_cov = 0.001;
          current_state = new state(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

          CT_ICP::LidarPlaneNormFactor::sqrt_info = sqrt(1 / laser_point_cov);

          CT_ICP::CTLidarPlaneNormFactor::sqrt_info = sqrt(1 / laser_point_cov);

          index_frame = 1;
          points_world.reset(new pcl::PointCloud<pcl::PointXYZI>());

          R_align = computeGravityAlignment(g_odom, g_imu);
     }

     lidarodom::~lidarodom()
     {
     }

     void lidarodom::loadOptions()
     {
          auto yaml = YAML::LoadFile(config_yaml_);
          options_.surf_res = yaml["odometry"]["surf_res"].as<double>();
          options_.log_print = yaml["odometry"]["log_print"].as<bool>();
          options_.max_num_iteration = yaml["odometry"]["max_num_iteration"].as<int>();

          options_.size_voxel_map = yaml["odometry"]["size_voxel_map"].as<double>();
          options_.min_distance_points = yaml["odometry"]["min_distance_points"].as<double>();
          options_.max_num_points_in_voxel = yaml["odometry"]["max_num_points_in_voxel"].as<int>();
          options_.max_distance = yaml["odometry"]["max_distance"].as<double>();
          options_.weight_alpha = yaml["odometry"]["weight_alpha"].as<double>();
          options_.weight_neighborhood = yaml["odometry"]["weight_neighborhood"].as<double>();
          options_.max_dist_to_plane_icp = yaml["odometry"]["max_dist_to_plane_icp"].as<double>();
          options_.init_num_frames = yaml["odometry"]["init_num_frames"].as<int>();
          options_.voxel_neighborhood = yaml["odometry"]["voxel_neighborhood"].as<int>();
          options_.max_number_neighbors = yaml["odometry"]["max_number_neighbors"].as<int>();
          options_.threshold_voxel_occupancy = yaml["odometry"]["threshold_voxel_occupancy"].as<int>();
          options_.estimate_normal_from_neighborhood = yaml["odometry"]["estimate_normal_from_neighborhood"].as<bool>();
          options_.min_number_neighbors = yaml["odometry"]["min_number_neighbors"].as<int>();
          options_.power_planarity = yaml["odometry"]["power_planarity"].as<double>();
          options_.num_closest_neighbors = yaml["odometry"]["num_closest_neighbors"].as<int>();

          options_.sampling_rate = yaml["odometry"]["sampling_rate"].as<double>();
          options_.ratio_of_nonground = yaml["odometry"]["ratio_of_nonground"].as<double>();
          options_.max_num_residuals = yaml["odometry"]["max_num_residuals"].as<int>();
          std::string str_motion_compensation = yaml["odometry"]["motion_compensation"].as<std::string>();
          if (str_motion_compensation == "NONE")
               options_.motion_compensation = MotionCompensation::NONE;
          else if (str_motion_compensation == "CONSTANT_VELOCITY")
               options_.motion_compensation = MotionCompensation::CONSTANT_VELOCITY;
          else if (str_motion_compensation == "ITERATIVE")
               options_.motion_compensation = MotionCompensation::ITERATIVE;
          else if (str_motion_compensation == "CONTINUOUS")
               options_.motion_compensation = MotionCompensation::CONTINUOUS;
          else
               std::cout << "The `motion_compensation` " << str_motion_compensation << " is not supported." << std::endl;

          std::string str_icpmodel = yaml["odometry"]["icpmodel"].as<std::string>();
          if (str_icpmodel == "POINT_TO_PLANE")
               options_.icpmodel = POINT_TO_PLANE;
          else if (str_icpmodel == "CT_POINT_TO_PLANE")
               options_.icpmodel = CT_POINT_TO_PLANE;
          else
               std::cout << "The `icp_residual` " << str_icpmodel << " is not supported." << std::endl;

          options_.beta_location_consistency = yaml["odometry"]["beta_location_consistency"].as<double>();
          options_.beta_orientation_consistency = yaml["odometry"]["beta_orientation_consistency"].as<double>();
          options_.beta_constant_velocity = yaml["odometry"]["beta_constant_velocity"].as<double>();
          options_.beta_small_velocity = yaml["odometry"]["beta_small_velocity"].as<double>();
          options_.thres_orientation_norm = yaml["odometry"]["thres_orientation_norm"].as<double>();
          options_.thres_translation_norm = yaml["odometry"]["thres_translation_norm"].as<double>();
     }

     bool lidarodom::init(const std::string &config_yaml)
     {
          config_yaml_ = config_yaml;
          StaticIMUInit::Options imu_init_options;
          imu_init_options.use_speed_for_static_checking_ = false;
          imu_init_ = StaticIMUInit(imu_init_options);

          text = "No Degeneracy";

          auto yaml = YAML::LoadFile(config_yaml_);
          delay_time_ = yaml["delay_time"].as<double>();

          std::vector<double> ext_t = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
          std::vector<double> ext_r = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();
          Vec3d lidar_T_wrt_IMU = math::VecFromArray(ext_t);
          Mat3d lidar_R_wrt_IMU = math::MatFromArray(ext_r);
          std::cout << yaml["mapping"]["extrinsic_R"] << std::endl;
          // std::cout << "lidar_R_wrt_IMU1:\n"
          //           << lidar_R_wrt_IMU << std::endl;

          Eigen::Quaterniond q_IL(lidar_R_wrt_IMU);
          // std::cout << "q_IL : [w, x, y, z] = [" 
          // << q_IL.w() << ", " 
          // << q_IL.x() << ", " 
          // << q_IL.y() << ", " 
          // << q_IL.z() << "]" << std::endl;

          q_IL.normalized();
          // std::cout << "q_IL (normalized): [w, x, y, z] = [" 
          // << q_IL.w() << ", " 
          // << q_IL.x() << ", " 
          // << q_IL.y() << ", " 
          // << q_IL.z() << "]" << std::endl;

          lidar_R_wrt_IMU = q_IL;
          // lidar_R_wrt_IMU = q_IL.toRotationMatrix();
          // lidar_R_wrt_IMU.transposeInPlace();
          // lidar_T_wrt_IMU = -lidar_R_wrt_IMU * lidar_T_wrt_IMU;

          // Eigen::Quaterniond q_IL(lidar_R_wrt_IMU);
          // q_IL.normalized();

          // std::cout << "lidar_R_wrt_IMU2:\n"
          //           << lidar_R_wrt_IMU << std::endl;

          // init TIL
          TIL_ = SE3(q_IL, lidar_T_wrt_IMU);
          // TIL_ = SE3(lidar_R_wrt_IMU, lidar_T_wrt_IMU);
          
          R_imu_lidar = lidar_R_wrt_IMU;
          t_imu_lidar = lidar_T_wrt_IMU;
          std::cout << "RIL:\n"
                    << R_imu_lidar << std::endl;
          std::cout << "tIL:" << t_imu_lidar.transpose() << std::endl;

          CT_ICP::LidarPlaneNormFactor::t_il = t_imu_lidar;
          CT_ICP::LidarPlaneNormFactor::q_il = TIL_.rotationMatrix();
          CT_ICP::CTLidarPlaneNormFactor::t_il = t_imu_lidar;
          CT_ICP::CTLidarPlaneNormFactor::q_il = TIL_.rotationMatrix();

          loadOptions();
          switch (options_.motion_compensation)
          {
          case NONE:
          case CONSTANT_VELOCITY:
               options_.point_to_plane_with_distortion = false;
               options_.icpmodel = POINT_TO_PLANE;
               break;
          case ITERATIVE:
               options_.point_to_plane_with_distortion = true;
               options_.icpmodel = POINT_TO_PLANE;
               break;
          case CONTINUOUS:
               options_.point_to_plane_with_distortion = true;
               options_.icpmodel = CT_POINT_TO_PLANE;
               break;
          }
          LOG(WARNING) << "motion_compensation:" << options_.motion_compensation << ", model: " << options_.icpmodel;

          return true;
     }

     void lidarodom::pushData(std::vector<point3D> msg, std::pair<double, double> data)
     {
          if (data.first < last_timestamp_lidar_)
          {
               LOG(ERROR) << "lidar loop back, clear buffer";
               lidar_buffer_.clear();
               time_buffer_.clear();
          }

          mtx_buf.lock();
          lidar_buffer_.push_back(msg);
          time_buffer_.push_back(data);
          last_timestamp_lidar_ = data.first;
          mtx_buf.unlock();
          cond.notify_one();
     }
     void lidarodom::pushData(IMUPtr imu)
     {
          double timestamp = imu->timestamp_;
          if (timestamp < last_timestamp_imu_)
          {
               LOG(WARNING) << "imu loop back, clear buffer";
               imu_buffer_.clear();
          }

          last_timestamp_imu_ = timestamp;

          mtx_buf.lock();
          imu_buffer_.emplace_back(imu);
          mtx_buf.unlock();
          cond.notify_one();
     }
     void lidarodom::pushData(const nav_msgs::Odometry::ConstPtr& odometryMsg)
     {
       std::lock_guard<std::mutex> lock2(odoLock);
       odomQueue.push_back(*odometryMsg);

     //  std::cout << "\033[2K\rCurrent odomQueue size: " << odomQueue.size() << std::flush;
     }

     void lidarodom::pushData(cv::Mat img, double timestamp)
     {
          if(timestamp < last_timestamp_img_)
          {
               LOG(WARNING) << "imu loop back, clear buffer";
               img_buffer_.clear();
          }
          last_timestamp_img_ = timestamp;

          mtx_buf.lock();
          img_buffer_.emplace_back(std::move(img));
          img_time_buffer_.emplace_back(timestamp);
          mtx_buf.unlock();
          cond.notify_one();
     }

     void lidarodom::run()
     {
          while (true)
          {
               std::vector<MeasureGroup> measurements;
               std::unique_lock<std::mutex> lk(mtx_buf);
               // std::cout << "Before waiting for condition..." << std::endl;
               cond.wait(lk, [&]
                         { return (measurements = getMeasureMents()).size() != 0; });
                         // std::cout << "Condition met, measurements size: " << measurements.size() << std::endl;
               lk.unlock();

               for (auto &m : measurements)
               {
                    // std::cout << "Processing frame: " << std::endl;
                    zjloc::common::Timer::Evaluate([&]()
                                                   { ProcessMeasurements(m); },
                                                   "processMeasurement");

                    {
                         auto real_time = std::chrono::high_resolution_clock::now();
                         static std::chrono::system_clock::time_point prev_real_time = real_time;

                         if (real_time - prev_real_time > std::chrono::seconds(5))
                         {
                              auto data_time = m.lidar_end_time_;
                              static double prev_data_time = data_time;
                              auto delta_real = std::chrono::duration_cast<std::chrono::milliseconds>(real_time - prev_real_time).count() * 0.001;
                              auto delta_sim = data_time - prev_data_time;
                              // printf("Processing the rosbag at %.1fX speed.", delta_sim / delta_real);

                              prev_data_time = data_time;
                              prev_real_time = real_time;
                         }
                    }
               }
          }
     }

     void lidarodom::ProcessMeasurements(MeasureGroup &meas)
     {
          measures_ = meas;

          if (imu_need_init_)
          {
               TryInitIMU();
               return;
          }

          // std::cout << ANSI_DELETE_LAST_LINE;
          // std::cout << ANSI_COLOR_GREEN << "============== process frame: "
          //           << index_frame << ANSI_COLOR_RESET << std::endl;
          imu_states_.clear(); //   need clear here

          zjloc::common::Timer::Evaluate([&]()
                                         { Predict(); },
                                         "predict");
          //
          zjloc::common::Timer::Evaluate([&]()
                                         { stateInitialization(); },
                                         "state init");

          std::vector<point3D> const_surf;
          const_surf.insert(const_surf.end(), meas.lidar_.begin(), meas.lidar_.end());
          // const_surf.assign(meas.lidar_.begin(), meas.lidar_.end());

          cloudFrame *p_frame;
          // zjloc::common::Timer::Evaluate([&]()
          //                                { p_frame = buildFrame(const_surf, current_state,
          //                                                       meas.lidar_begin_time_,
          //                                                       meas.lidar_end_time_); },
          //                                "build frame");
          
          zjloc::common::Timer::Evaluate([&]()
                                         { p_frame = buildFrame(const_surf, meas.img_, current_state,
                                                                meas.lidar_begin_time_,
                                                                meas.lidar_end_time_); },
                                         "build frame");
          

          //   lio
          zjloc::common::Timer::Evaluate([&]()
                                         { poseEstimation(p_frame); },
                                         "poseEstimate");

          SE3 pose_of_lo_ = SE3(current_state->rotation, current_state->translation);
          // std::cout << "obs: " << current_state->translation.transpose() << ", " << current_state->rotation.transpose() << std::endl;
          // SE3 pred_pose = eskf_.GetNominalSE3();
          // std::cout << "pred: " << pred_pose.translation().transpose() << ", " << pred_pose.so3().log().transpose() << std::endl;
          zjloc::common::Timer::Evaluate([&]()
                                         { eskf_.ObserveSE3(pose_of_lo_, 1e-2, 1e-2); },
                                         "eskf_obs");

          zjloc::common::Timer::Evaluate([&]()
                                         {
               std::string laser_topic = "laser";
               std::string laser_topic2 = "orin_laser";
               std::string laser_topic3 = "orin_vins";
               static double z_axis_start = 0.0; 
               static bool z_axis_recorded = false; 

               if (!odomQueue.empty()){
                    nav_msgs::Odometry externalOdom = getClosestOdom(meas.lidar_end_time_);
     
                    tf::Quaternion orientation;
                    tf::quaternionMsgToTF(externalOdom.pose.pose.orientation, orientation);
                    Eigen::Quaterniond odom_quat(orientation.w(), orientation.x(), orientation.y(), orientation.z());
                    Eigen::Vector3d odom_trans(
                              externalOdom.pose.pose.position.x,
                              externalOdom.pose.pose.position.y,
                              externalOdom.pose.pose.position.z
                         );
     
                    external_pose = SE3(Eigen::Quaterniond(R_align * odom_quat.toRotationMatrix()), R_align * odom_trans);
               }

               if (first_is_degenerate && is_degenerate) {

                    if (first_degenerate) {   
                         text = "Switch to VIO";
                         VIO_to_LIO = external_pose.inverse() * pose_of_lo_;
                         first_degenerate = false;
                    }

                    fused_pose =  external_pose * VIO_to_LIO;
                    pub_pose_to_ros(laser_topic, fused_pose, meas.lidar_end_time_);
               } else if (first_is_degenerate && !is_degenerate) {

                    if (first_exit_degenerate) {
                         
                         text = "Switch to LIO";
                         LIO_to_Fused = Last_pose_of_lo_.inverse() * fused_pose;

                         first_exit_degenerate = false;
                    }
                         fused_pose = SE3(LIO_to_Fused.rotationMatrix() * pose_of_lo_.rotationMatrix(),
                                           pose_of_lo_.translation() + LIO_to_Fused.translation());
                         pub_pose_to_ros(laser_topic, fused_pose, meas.lidar_end_time_);
               }

               if (!first_is_degenerate && is_degenerate) {


                    if (first_degenerate) {

                         text = "Switch to VIO";
                         // VIO_to_Fused = Last_external_pose.inverse() * fused_pose;

                         Eigen::Vector3d last_translation = Last_external_pose.translation();
                         Eigen::Matrix3d last_rotation = Last_external_pose.rotationMatrix();

                         Eigen::Vector3d fused_translation = fused_pose.translation();
                         Eigen::Matrix3d fused_rotation = fused_pose.rotationMatrix();

                         Eigen::Vector3d translation_offset = fused_translation - last_translation;
                         Eigen::Matrix3d rotation_offset =  last_rotation.inverse() * fused_rotation;

                         VIO_to_Fused = SE3(rotation_offset, translation_offset);
                         first_degenerate = false;
                         
                    }
                         // fused_pose =  external_pose * VIO_to_Fused;
                         fused_pose = SE3(external_pose.rotationMatrix() * VIO_to_Fused.rotationMatrix(),
                                           external_pose.translation() + VIO_to_Fused.translation());
                         pub_pose_to_ros(laser_topic, fused_pose, meas.lidar_end_time_);
               } else if (!first_is_degenerate && !is_degenerate && has_entered_degenerate) {

                    if (first_exit_degenerate) {
                             text = "Switch to LIO";
                             
                         //     LIO_to_Fused = Last_pose_of_lo_.inverse() * fused_pose;

                             Eigen::Vector3d last_translation = Last_pose_of_lo_.translation();
                             Eigen::Matrix3d last_rotation = Last_pose_of_lo_.rotationMatrix();

                             Eigen::Vector3d fused_translation = fused_pose.translation();
                             Eigen::Matrix3d fused_rotation = fused_pose.rotationMatrix();

                             Eigen::Vector3d translation_offset = fused_translation - last_translation;
                             Eigen::Matrix3d rotation_offset =  last_rotation.inverse() * fused_rotation;

                             LIO_to_Fused = SE3(rotation_offset, translation_offset);

                         first_exit_degenerate = false;
                    }
                         // fused_pose = pose_of_lo_ * LIO_to_Fused;
                         fused_pose = SE3(pose_of_lo_.rotationMatrix() * LIO_to_Fused.rotationMatrix(),
                                           pose_of_lo_.translation() + LIO_to_Fused.translation());
                         pub_pose_to_ros(laser_topic, fused_pose, meas.lidar_end_time_);
               }

               Last_pose_of_lo_ = pose_of_lo_;
               Last_external_pose = external_pose;
               // prev_is_degenerate = is_degenerate;

               if (!entered_degenerate) pub_pose_to_ros(laser_topic, pose_of_lo_, meas.lidar_end_time_);
               pub_pose_to_ros(laser_topic2, pose_of_lo_, meas.lidar_end_time_);
               pub_pose_to_ros(laser_topic3, external_pose, meas.lidar_end_time_);
               laser_topic = "velocity";
               SE3 pred_pose = eskf_.GetNominalSE3();
               Eigen::Vector3d vel_world = eskf_.GetNominalVel();
               Eigen::Vector3d vel_base = pred_pose.rotationMatrix().inverse()*vel_world;
               pub_data_to_ros(laser_topic, vel_base.x(), text);
               laser_topic = "text";
               // std::cout << "Text content: " << text << std::endl;
               pub_data_to_ros(laser_topic, 0, text);
               if(index_frame%8==0)
               {
                    laser_topic = "dist";
                    static Eigen::Vector3d last_t = Eigen::Vector3d::Zero();
                    Eigen::Vector3d t = pred_pose.translation();
                    static double dist = 0;
                    dist += (t - last_t).norm();
                    last_t = t;
                    pub_data_to_ros(laser_topic, dist, text);
                    // std::cout << eskf_.GetGravity().transpose() << std::endl;
               } },
                                         "pub cloud");

          p_frame->p_state = new state(current_state, true);
          state *tmp_state = new state(current_state, true);
          all_state_frame.push_back(tmp_state);
          current_state = new state(current_state, false);

          index_frame++;
          p_frame->release();
          std::vector<point3D>().swap(meas.lidar_);
          std::vector<point3D>().swap(const_surf);
     }

     void lidarodom::poseEstimation(cloudFrame *p_frame)
     {
          if (index_frame > 1)
          {
               zjloc::common::Timer::Evaluate([&]()
                                              { optimize(p_frame); },
                                              "optimize");
          }

          bool add_points = true;
          if (add_points)
          { //   update map here
               zjloc::common::Timer::Evaluate([&]()
                                              { map_incremental(p_frame); },
                                              "map update");
          }

          zjloc::common::Timer::Evaluate([&]()
                                         { lasermap_fov_segment(); },
                                         "fov segment");
     }

     void lidarodom::optimize(cloudFrame *p_frame)
     {

          state *previous_state = nullptr;
          Eigen::Vector3d previous_translation = Eigen::Vector3d::Zero();
          Eigen::Vector3d previous_velocity = Eigen::Vector3d::Zero();
          Eigen::Quaterniond previous_orientation = Eigen::Quaterniond::Identity();

          state *curr_state = p_frame->p_state;
          Eigen::Quaterniond begin_quat = Eigen::Quaterniond(curr_state->rotation_begin);
          Eigen::Quaterniond end_quat = Eigen::Quaterniond(curr_state->rotation);
          Eigen::Vector3d begin_t = curr_state->translation_begin;
          Eigen::Vector3d end_t = curr_state->translation;

          if (p_frame->frame_id > 1)
          {
               if (options_.log_print)
                    std::cout << "all_cloud_frame.size():" << all_state_frame.size() << ", " << p_frame->frame_id << std::endl;
               // previous_state = all_cloud_frame[p_frame->frame_id - 2]->p_state;
               previous_state = all_state_frame[p_frame->frame_id - 2];
               previous_translation = previous_state->translation;
               previous_velocity = previous_state->translation - previous_state->translation_begin;
               previous_orientation = previous_state->rotation;
          }
          if (options_.log_print)
          {
               std::cout << "prev end: " << previous_translation.transpose() << std::endl;
               std::cout << "curr begin: " << p_frame->p_state->translation_begin.transpose()
                         << "\ncurr end: " << p_frame->p_state->translation.transpose() << std::endl;
          }

          std::vector<point3D> surf_keypoints;
          gridSampling(p_frame->point_surf, surf_keypoints,
                       options_.sampling_rate * options_.surf_res);

          size_t num_size = p_frame->point_surf.size();

          auto transformKeypoints = [&](std::vector<point3D> &point_frame)
          {
               Eigen::Matrix3d R;
               Eigen::Vector3d t;
               for (auto &keypoint : point_frame)
               {
                    if (options_.point_to_plane_with_distortion ||
                        options_.icpmodel == IcpModel::CT_POINT_TO_PLANE)
                    {
                         double alpha_time = keypoint.alpha_time;

                         Eigen::Quaterniond q = begin_quat.slerp(alpha_time, end_quat);
                         q.normalize();
                         R = q.toRotationMatrix();
                         t = (1.0 - alpha_time) * begin_t + alpha_time * end_t;
                    }
                    else
                    {
                         R = end_quat.normalized().toRotationMatrix();
                         t = end_t;
                    }
                    keypoint.point = R * (TIL_ * keypoint.raw_point) + t;
               }
          };

          for (int iter(0); iter < options_.max_num_iteration; iter++)
          {
               transformKeypoints(surf_keypoints);

               // ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1 / (1.5e-3));
               ceres::LossFunction *loss_function = new ceres::HuberLoss(0.5);
               ceres::Problem::Options problem_options;
               ceres::Problem problem(problem_options);
#ifdef USE_ANALYTICAL_DERIVATE
               ceres::LocalParameterization *parameterization = new RotationParameterization();
#else
               auto *parameterization = new ceres::EigenQuaternionParameterization();
#endif

               switch (options_.icpmodel)
               {
               case IcpModel::CT_POINT_TO_PLANE:
                    problem.AddParameterBlock(&begin_quat.x(), 4, parameterization);
                    problem.AddParameterBlock(&end_quat.x(), 4, parameterization);
                    problem.AddParameterBlock(&begin_t.x(), 3);
                    problem.AddParameterBlock(&end_t.x(), 3);
                    break;
               case IcpModel::POINT_TO_PLANE:
                    problem.AddParameterBlock(&end_quat.x(), 4, parameterization);
                    problem.AddParameterBlock(&end_t.x(), 3);
                    break;
               }

               std::vector<ceres::CostFunction *> surfFactor;
               std::vector<Eigen::Vector3d> normalVec;
               addSurfCostFactor(surfFactor, normalVec, surf_keypoints, p_frame);

               bool is_last_iteration = (iter == options_.max_num_iteration - 1);
               bool is_exit_condition_met = false;

               int surf_num = 0;
               if (options_.log_print)
                    std::cout << "get factor: " << surfFactor.size() << std::endl;
               for (auto &e : surfFactor)
               {
                    surf_num++;
                    switch (options_.icpmodel)
                    {
                    case IcpModel::CT_POINT_TO_PLANE:
                         problem.AddResidualBlock(e, loss_function, &begin_t.x(), &begin_quat.x(), &end_t.x(), &end_quat.x());
                         break;
                    case IcpModel::POINT_TO_PLANE:
                         problem.AddResidualBlock(e, loss_function, &end_t.x(), &end_quat.x());
                         break;
                    }
                    // if (surf_num > options_.max_num_residuals)
                    //      break;
               }
                         
               //   release
               // std::vector<Eigen::Vector3d>().swap(normalVec);
               std::vector<ceres::CostFunction *>().swap(surfFactor);

               if (options_.icpmodel == IcpModel::CT_POINT_TO_PLANE)
               {
                    if (options_.beta_location_consistency > 0.) //   location consistency
                    {
#ifdef USE_ANALYTICAL_DERIVATE
                         CT_ICP::LocationConsistencyFactor *cost_location_consistency =
                             new CT_ICP::LocationConsistencyFactor(previous_translation, sqrt(surf_num * options_.beta_location_consistency * laser_point_cov));
#else
                         auto *cost_location_consistency =
                             CT_ICP::LocationConsistencyFunctor::Create(previous_translation, sqrt(surf_num * options_.beta_location_consistency));
#endif
                         problem.AddResidualBlock(cost_location_consistency, nullptr, &begin_t.x());
                    }

                    if (options_.beta_orientation_consistency > 0.) // orientation consistency
                    {
#ifdef USE_ANALYTICAL_DERIVATE
                         CT_ICP::RotationConsistencyFactor *cost_rotation_consistency =
                             new CT_ICP::RotationConsistencyFactor(previous_orientation, sqrt(surf_num * options_.beta_orientation_consistency * laser_point_cov));
#else
                         auto *cost_rotation_consistency =
                             CT_ICP::OrientationConsistencyFunctor::Create(previous_orientation, sqrt(surf_num * options_.beta_orientation_consistency));
#endif
                         problem.AddResidualBlock(cost_rotation_consistency, nullptr, &begin_quat.x());
                    }

                    if (options_.beta_small_velocity > 0.) //     small velocity
                    {
#ifdef USE_ANALYTICAL_DERIVATE
                         CT_ICP::SmallVelocityFactor *cost_small_velocity =
                             new CT_ICP::SmallVelocityFactor(sqrt(surf_num * options_.beta_small_velocity * laser_point_cov));
#else
                         auto *cost_small_velocity =
                             CT_ICP::SmallVelocityFunctor::Create(sqrt(surf_num * options_.beta_small_velocity));
#endif
                         problem.AddResidualBlock(cost_small_velocity, nullptr, &begin_t.x(), &end_t.x());
                    }

                    // if (options_.beta_constant_velocity > 0.) //  const velocity
                    // {
                    //      CT_ICP::VelocityConsistencyFactor2 *cost_velocity_consistency =
                    //          new CT_ICP::VelocityConsistencyFactor2(previous_velocity, sqrt(surf_num * options_.beta_constant_velocity * laser_point_cov));
                    //      problem.AddResidualBlock(cost_velocity_consistency, nullptr, PR_begin, PR_end);
                    // }
               }

               if (surf_num < options_.min_num_residuals)
               {
                    std::stringstream ss_out;
                    ss_out << "[Optimization] Error : not enough keypoints selected in ct-icp !" << std::endl;
                    ss_out << "[Optimization] number_of_residuals : " << surf_num << std::endl;
                    std::cout << "ERROR: " << ss_out.str();
               }

               ceres::Solver::Options options;
               options.max_num_iterations = 5;
               options.num_threads = 3;
               options.minimizer_progress_to_stdout = false;
               options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;

               // ceres::Solver::Options options;
               // options.linear_solver_type = ceres::DENSE_SCHUR;
               // options.trust_region_strategy_type = ceres::DOGLEG;
               // options.max_num_iterations = 10;
               // options.minimizer_progress_to_stdout = false;
               // options.num_threads = 6;

               ceres::Solver::Summary summary;

               ceres::Solve(options, &problem, &summary);

               if (!summary.IsSolutionUsable())
               {
                    std::cout << summary.FullReport() << std::endl;
                    throw std::runtime_error("Error During Optimization");
               }

               begin_quat.normalize();
               end_quat.normalize();

               double diff_trans = 0, diff_rot = 0;
               diff_trans += (current_state->translation_begin - begin_t).norm();
               diff_rot += AngularDistance(current_state->rotation_begin, begin_quat);

               diff_trans += (current_state->translation - end_t).norm();
               diff_rot += AngularDistance(current_state->rotation, end_quat);

               if (diff_rot < options_.thres_orientation_norm &&
                    diff_trans < options_.thres_translation_norm)
               {
                     // if (options_.log_print)
                     //      std::cout << "Optimization: Finished with N=" << iter << " ICP iterations" << std::endl;
                     // break;
                     is_exit_condition_met = true;
               }

               if (is_last_iteration || is_exit_condition_met) {
                    // if (checkLocalizability(surf_keypoints, normalVec) == 1)                
                    if (checkLocalizability(normalVec) == 1) 
                    {
                      is_degenerate = true;
                      entered_degenerate = true;
                      // if (!prev_is_degenerate) {
                      if (!prev_is_degenerate) {
                           if (!has_entered_degenerate) {
                                first_is_degenerate = true;
                                has_entered_degenerate = true;
                              } else first_is_degenerate = false;
                           first_degenerate = true;
                          // std::cout << "Setting first_degenerate to true: "
                          //           << "prev_is_degenerate: " << prev_is_degenerate
                          //           << ", has_entered_degenerate: " << has_entered_degenerate
                          //           << std::endl;
                         }
                    }
                    else {
                       is_degenerate = false;
                       if (prev_is_degenerate) {
                         first_exit_degenerate = true;
                       }
                    }
                    prev_is_degenerate = is_degenerate;
               }
               
               //release
               std::vector<Eigen::Vector3d>().swap(normalVec);

               if (options_.icpmodel == IcpModel::CT_POINT_TO_PLANE)
               {
                    p_frame->p_state->translation_begin = begin_t;
                    p_frame->p_state->rotation_begin = begin_quat;
                    p_frame->p_state->translation = end_t;
                    p_frame->p_state->rotation = end_quat;

                    current_state->translation_begin = begin_t;
                    current_state->translation = end_t;
                    current_state->rotation_begin = begin_quat;
                    current_state->rotation = end_quat;
               }
               if (options_.icpmodel == IcpModel::POINT_TO_PLANE)
               {
                    p_frame->p_state->translation = end_t;
                    p_frame->p_state->rotation = end_quat;

                    current_state->translation = end_t;
                    current_state->rotation = end_quat;
               }

               // if (diff_rot < options_.thres_orientation_norm &&
               //     diff_trans < options_.thres_translation_norm){
               if (is_exit_condition_met) {
                    if (options_.log_print)
                         std::cout << "Optimization: Finished with N=" << iter << " ICP iterations" << std::endl;
                    break;
                    // is_exit_condition_met = true;
               }
          }     

          std::vector<point3D>().swap(surf_keypoints);
          if (options_.log_print)
          {
               std::cout << "opt: " << p_frame->p_state->translation_begin.transpose()
                         << ",end: " << p_frame->p_state->translation.transpose() << std::endl;
          }

          //   transpose point before added
          transformKeypoints(p_frame->point_surf);
     }

     nav_msgs::Odometry lidarodom::getClosestOdom(double timestamp) {
          // std::lock_guard<std::mutex> lock(odoLock);
      
          nav_msgs::Odometry closestOdom;
          double minTimeDiff = std::numeric_limits<double>::max();
      
          // for (const auto& odom : odomQueue) {
          //     double timeDiff = std::abs(odom.header.stamp.toSec() - timestamp);
          //     if (timeDiff < minTimeDiff) {
          //         minTimeDiff = timeDiff;
          //         closestOdom = odom;
          //     }
          // }

          if (odomQueue.empty()) {
               throw std::runtime_error("odomQueue is empty");
          }
       
          auto it = std::lower_bound(odomQueue.begin(), odomQueue.end(), timestamp,
                                      [](const nav_msgs::Odometry& odom, double time) {
                                          return odom.header.stamp.toSec() < time;
                                      });
       
          if (it == odomQueue.end()) {
               closestOdom = odomQueue.back(); 
          }
          if (it == odomQueue.begin()) {
               closestOdom = odomQueue.front(); 
          }else {
               auto prev_it = std::prev(it);
               if (std::abs(prev_it->header.stamp.toSec() - timestamp) <
                   std::abs(it->header.stamp.toSec() - timestamp)) {
                   closestOdom = *prev_it;
               } else {
                   closestOdom = *it;
               }
          }

          return closestOdom;
     }

     Eigen::Matrix3d lidarodom::computeGravityAlignment(const Eigen::Vector3d& g_odom, const Eigen::Vector3d& g_imu) const {

          Eigen::Vector3d axis = g_imu.cross(g_odom).normalized();
          double angle = acos(g_imu.dot(g_odom) / (g_imu.norm() * g_odom.norm()));
      
          Eigen::AngleAxisd rotation_vector(angle, axis);
          return rotation_vector.toRotationMatrix();
     }

     double lidarodom::checkLocalizability(std::vector<Eigen::Vector3d> planeNormals)
     {
          static bool permanently_degenerate = false; 

          if (permanently_degenerate)
          {
             return 1;
          }
          
          Eigen::MatrixXd mat;
          static int stable_degenerate_count = 0;
          static int stable_exit_degenerate_count = 0; 
          
          if (planeNormals.size() > 10)
          {
               mat.setZero(planeNormals.size(), 3);
               for (int i = 0; i < planeNormals.size(); i++)
               {
                    mat(i, 0) = planeNormals[i].x();
                    mat(i, 1) = planeNormals[i].y();
                    mat(i, 2) = planeNormals[i].z();
               }
               Eigen::JacobiSVD<Eigen::MatrixXd> svd(planeNormals.size(), 3);
               svd.compute(mat);

               double degeneracyIndex = (svd.singularValues().x() - svd.singularValues().z()) / svd.singularValues().y();
               double SparseIndex = (svd.singularValues().x() + svd.singularValues().z() + svd.singularValues().y())/3;

               // if (svd.singularValues().z() < 10)
               // if (degeneracyIndex > 0.9 ){
               //    std::cout << ANSI_COLOR_YELLOW << "Low convincing result -> singular values:"
               //                << svd.singularValues().x() << ", " << svd.singularValues().y() << ", "
               //                << svd.singularValues().z() << ANSI_COLOR_RESET << std::endl;  
               // }
               // if (SparseIndex < 10 || degeneracyIndex > 0.90){ //mid360 1.5 - 3.55  //0.45 
 
               if (SparseIndex < 10 || svd.singularValues().z() < 7 ){         
               // if (SparseIndex < 10 ){
               // if (svd.singularValues().z() < 4){  //avia
               // if (SparseIndex < 10){   
                    // std::cout << ANSI_COLOR_YELLOW << "Low convincing result -> singular values:"
                    //           << svd.singularValues().x() << ", " << svd.singularValues().y() << ", "
                    //           << svd.singularValues().z() << ANSI_COLOR_RESET << std::endl;
                    // std::cout << "Degenerate scene detected. Using external odometry." << std::endl;
                    // std::cout << "\033[2K\rDegenerate scene detected. Using external odometry." << std::flush;
                    // stable_degenerate_count++;
                    // stable_exit_degenerate_count = 0;
                    // if (stable_degenerate_count > 2) {
                    //      return 1;
                    // }
                    // else return 0;
                    return 1;
               } else return 0;
               // }
               // else if(has_entered_degenerate){

               //      stable_degenerate_count = 0;
               //      stable_exit_degenerate_count++;

               //      if (stable_exit_degenerate_count > 2)
               //      {
               //         return 0;
               //      }
               //      else
               //      {
               //         return 1;
               //      }
               // }
          }
          else
          {
               std::cout << ANSI_COLOR_RED << "Too few normal vector received -> " << planeNormals.size() << ANSI_COLOR_RESET << std::endl;
               permanently_degenerate = true;
               return 1;
          }

     }

     Neighborhood lidarodom::computeNeighborhoodDistribution(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &points)
     {
          Neighborhood neighborhood;
          // Compute the normals
          Eigen::Vector3d barycenter(Eigen::Vector3d(0, 0, 0));
          for (auto &point : points)
          {
               barycenter += point;
          }

          barycenter /= (double)points.size();
          neighborhood.center = barycenter;

          Eigen::Matrix3d covariance_Matrix(Eigen::Matrix3d::Zero());
          for (auto &point : points)
          {
               for (int k = 0; k < 3; ++k)
                    for (int l = k; l < 3; ++l)
                         covariance_Matrix(k, l) += (point(k) - barycenter(k)) *
                                                    (point(l) - barycenter(l));
          }
          covariance_Matrix(1, 0) = covariance_Matrix(0, 1);
          covariance_Matrix(2, 0) = covariance_Matrix(0, 2);
          covariance_Matrix(2, 1) = covariance_Matrix(1, 2);
          neighborhood.covariance = covariance_Matrix;
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(covariance_Matrix);
          Eigen::Vector3d normal(es.eigenvectors().col(0).normalized());
          neighborhood.normal = normal;

          double sigma_1 = sqrt(std::abs(es.eigenvalues()[2]));
          double sigma_2 = sqrt(std::abs(es.eigenvalues()[1]));
          double sigma_3 = sqrt(std::abs(es.eigenvalues()[0]));
          neighborhood.a2D = (sigma_2 - sigma_3) / sigma_1;

          if (neighborhood.a2D != neighborhood.a2D)
          {
               throw std::runtime_error("error");
          }

          return neighborhood;
     }

     void lidarodom::addSurfCostFactor(std::vector<ceres::CostFunction *> &surf, std::vector<Eigen::Vector3d> &normals,
                                       std::vector<point3D> &keypoints, const cloudFrame *p_frame)
     {
          auto estimatePointNeighborhood = [&](const std::vector<Eigen::Vector3d,
               Eigen::aligned_allocator<Eigen::Vector3d>> &vector_neighbors,
               const Eigen::Vector3d                                &location,
               double                                               &planarity_weight)
           {
             auto neighborhood = computeNeighborhoodDistribution(vector_neighbors);
             planarity_weight = std::pow(neighborhood.a2D, options_.power_planarity);
             if (neighborhood.normal.dot(p_frame->p_state->translation_begin - location) < 0)
               neighborhood.normal = -neighborhood.normal;
             return neighborhood;
           };

         double lambda_weight       = std::abs(options_.weight_alpha);
         double lambda_neighborhood = std::abs(options_.weight_neighborhood);
         const double sum           = lambda_weight + lambda_neighborhood;
         lambda_weight /= sum;
         lambda_neighborhood /= sum;

         const short nb_voxels_visited = (p_frame->frame_id < options_.init_num_frames)
                                          ? 2
                                          : options_.voxel_neighborhood;
         const int kThresholdCapacity   = (p_frame->frame_id < options_.init_num_frames)
                                          ? 1
                                          : options_.threshold_voxel_occupancy;
         const double kMaxPointToPlane  = options_.max_dist_to_plane_icp;

         size_t max_possible =
           std::min<size_t>(keypoints.size() * options_.num_closest_neighbors,
                            options_.max_num_residuals);
         surf.reserve(max_possible);
         normals.reserve(max_possible);
         std::vector<point3D> valid_keypoints;
         valid_keypoints.reserve(max_possible);

         int num_residuals = 0;
         const size_t N    = keypoints.size();

         for (size_t k = 0; k < N; ++k)
         {
             const auto &kp        = keypoints[k];
             const auto &raw_point = kp.raw_point;

             std::vector<voxel> voxels;
             auto vector_neighbors = searchNeighbors(
               voxel_map,
               kp.point,
               nb_voxels_visited,
               options_.size_voxel_map,
               options_.max_number_neighbors,
               kThresholdCapacity,
               options_.estimate_normal_from_neighborhood ? nullptr : &voxels);

             if (vector_neighbors.size() < options_.min_number_neighbors)
                 continue;

             double planarity_w = 0;
             Eigen::Vector3d location = TIL_ * raw_point;
             auto neighborhood = estimatePointNeighborhood(vector_neighbors,
                                                          location,
                                                          planarity_w);
             double weight = lambda_weight * planarity_w
                           + lambda_neighborhood
                             * std::exp(
                                 - (vector_neighbors[0] - kp.point).norm()
                                 / (kMaxPointToPlane * options_.min_number_neighbors)
                               );

             for (int i = 0;
                  i < options_.num_closest_neighbors
                  && size_t(i) < vector_neighbors.size();
                  ++i)
             {
                 double dist =
                   std::abs((kp.point - vector_neighbors[i]).transpose()
                            * neighborhood.normal);
                 if (dist >= options_.max_dist_to_plane_icp)
                     continue;

                 ++num_residuals;

                 Eigen::Vector3d nvec = neighborhood.normal.normalized();
                 double offset = -nvec.dot(vector_neighbors[i]);

                 ceres::CostFunction *cost_f = nullptr;
                 switch (options_.icpmodel)
                 {
                   case IcpModel::CT_POINT_TO_PLANE:
                   {
         #ifdef USE_ANALYTICAL_DERIVATE
                     cost_f = new CT_ICP::CTLidarPlaneNormFactor(
                                raw_point,
                                nvec,
                                offset,
                                kp.alpha_time,
                                weight);
         #else
                     cost_f = CT_ICP::CTPointToPlaneFunctor::Create(
                                vector_neighbors[i],
                                raw_point,
                                nvec,
                                kp.alpha_time,
                                weight);
         #endif
                     break;
                   }
                   case IcpModel::POINT_TO_PLANE:
                   {
                     Eigen::Vector3d point_end =
                       p_frame->p_state->rotation.inverse()  * kp.point
                       - p_frame->p_state->rotation.inverse() * p_frame->p_state->translation;
         #ifdef USE_ANALYTICAL_DERIVATE
                     cost_f = new CT_ICP::LidarPlaneNormFactor(
                                point_end,
                                nvec,
                                offset,
                                weight);
         #else
                     cost_f = CT_ICP::PointToPlaneFunctor::Create(
                                vector_neighbors[i],
                                point_end,
                                nvec,
                                weight);
         #endif
                     break;
                   }
                 }

                 surf.push_back(cost_f);
                 normals.push_back(nvec);
                 valid_keypoints.push_back(kp);

                 if (num_residuals >= options_.max_num_residuals) break;
             }

             if (num_residuals >= options_.max_num_residuals) break;
         }

         keypoints = std::move(valid_keypoints);
         
    }

     ///  ===================  for search neighbor  ===================================================
     using pair_distance_t = std::tuple<double, Eigen::Vector3d, voxel>;

     struct comparator
     {
          bool operator()(const pair_distance_t &left, const pair_distance_t &right) const
          {
               return std::get<0>(left) < std::get<0>(right);
          }
     };

     using priority_queue_t = std::priority_queue<pair_distance_t, std::vector<pair_distance_t>, comparator>;

     std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
     lidarodom::searchNeighbors(const voxelHashMap &map, const Eigen::Vector3d &point,
                                int nb_voxels_visited, double size_voxel_map,
                                int max_num_neighbors, int threshold_voxel_capacity,
                                std::vector<voxel> *voxels)
     {

          if (voxels != nullptr)
               voxels->reserve(max_num_neighbors);

          short kx = static_cast<short>(point[0] / size_voxel_map);
          short ky = static_cast<short>(point[1] / size_voxel_map);
          short kz = static_cast<short>(point[2] / size_voxel_map);

          priority_queue_t priority_queue;

          int max_iterations = 200; 
          int iteration_count = 0;

          voxel voxel_temp(kx, ky, kz);
          for (short kxx = kx - nb_voxels_visited; kxx < kx + nb_voxels_visited + 1; ++kxx)
          {
               for (short kyy = ky - nb_voxels_visited; kyy < ky + nb_voxels_visited + 1; ++kyy)
               {
                    for (short kzz = kz - nb_voxels_visited; kzz < kz + nb_voxels_visited + 1; ++kzz)
                    {
                         if (++iteration_count > max_iterations)
                         {
                            std::cerr << "Warning: Exceeded maximum iterations, exiting loop." << std::endl;
                            is_degenerate = true;
                            goto exit_loops; 
                         }
                         voxel_temp.x = kxx;
                         voxel_temp.y = kyy;
                         voxel_temp.z = kzz;

                         auto search = map.find(voxel_temp);
                         if (search != map.end())
                         {
                              const auto &voxel_block = search.value();
                              if (voxel_block.NumPoints() < threshold_voxel_capacity)
                                   continue;
                              for (int i(0); i < voxel_block.NumPoints(); ++i)
                              {
                                   auto &neighbor = voxel_block.points[i];
                                   double distance = (neighbor - point).norm();
                                   if (priority_queue.size() == max_num_neighbors)
                                   {
                                        if (distance < std::get<0>(priority_queue.top()))
                                        {
                                             priority_queue.pop();
                                             priority_queue.emplace(distance, neighbor, voxel_temp);
                                        }
                                   }
                                   else
                                        priority_queue.emplace(distance, neighbor, voxel_temp);
                              }
                         }
                    }
               }
          }

          exit_loops:

          auto size = priority_queue.size();
          std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> closest_neighbors(size);
          if (voxels != nullptr)
          {
               voxels->resize(size);
          }
          for (auto i = 0; i < size; ++i)
          {
               closest_neighbors[size - 1 - i] = std::get<1>(priority_queue.top());
               if (voxels != nullptr)
                    (*voxels)[size - 1 - i] = std::get<2>(priority_queue.top());
               priority_queue.pop();
          }

          return closest_neighbors;
     }

     void lidarodom::addPointToMap(voxelHashMap &map, const Eigen::Vector3d &point,
                                   const double &intensity, double voxel_size,
                                   int max_num_points_in_voxel, double min_distance_points,
                                   int min_num_points, cloudFrame *p_frame)
     {
          short kx = static_cast<short>(point[0] / voxel_size);
          short ky = static_cast<short>(point[1] / voxel_size);
          short kz = static_cast<short>(point[2] / voxel_size);

          voxelHashMap::iterator search = map.find(voxel(kx, ky, kz));

          if (search != map.end())
          {
               auto &voxel_block = (search.value());

               if (!voxel_block.IsFull())
               {
                    double sq_dist_min_to_points = 10 * voxel_size * voxel_size;
                    for (int i(0); i < voxel_block.NumPoints(); ++i)
                    {
                         auto &_point = voxel_block.points[i];
                         double sq_dist = (_point - point).squaredNorm();
                         if (sq_dist < sq_dist_min_to_points)
                         {
                              sq_dist_min_to_points = sq_dist;
                         }
                    }
                    if (sq_dist_min_to_points > (min_distance_points * min_distance_points))
                    {
                         if (min_num_points <= 0 || voxel_block.NumPoints() >= min_num_points)
                         {
                              voxel_block.AddPoint(point);
                              // addPointToPcl(points_world, point, intensity, p_frame);
                         }
                    }
               }
          }
          else
          {
               if (min_num_points <= 0)
               {
                    voxelBlock block(max_num_points_in_voxel);
                    block.AddPoint(point);
                    map[voxel(kx, ky, kz)] = std::move(block);
               }
          }
          addPointToPcl(points_world, point, intensity, p_frame);
     }

     void lidarodom::addPointToPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_points, const Eigen::Vector3d &point, const double &intensity, cloudFrame *p_frame)
     {
          pcl::PointXYZI cloudTemp;

          cloudTemp.x = point.x();
          cloudTemp.y = point.y();
          cloudTemp.z = point.z();
          cloudTemp.intensity = intensity;
          // cloudTemp.intensity = 50 * (point.z() - p_frame->p_state->translation.z());
          pcl_points->points.push_back(cloudTemp);
     }
     

     void lidarodom::map_incremental(cloudFrame *p_frame, int min_num_points)
     {
          int count = 0;
          for (const auto &point : p_frame->point_surf)
          {
               addPointToMap(voxel_map, point.point, point.intensity,
                             options_.size_voxel_map, options_.max_num_points_in_voxel,
                             options_.min_distance_points, min_num_points, p_frame);

               ++count;
               if (count % 2 == 0)
                    addPointToPcl(points_world, point.point, point.intensity, p_frame);

          }

          {
               if (!points_world->empty())
               {
                    std::string laser_topic = "laser";
                    pub_cloud_to_ros(laser_topic, points_world, p_frame->time_frame_end);
               }
               else
               {
                    LOG(INFO) << "No laser points provided.";
               }

               if (!p_frame->img_.empty())
               {
                    std::string img_topic = "camera";
                    pub_image_to_ros(img_topic, p_frame->img_, p_frame->time_frame_end );
               }
               else
               {
                    LOG(INFO) << "No image provided.";
               }
          }
          points_world->clear();
     }

     void lidarodom::lasermap_fov_segment()
     {
          //   use predict pose here
          Eigen::Vector3d location = current_state->translation;
          std::vector<voxel> voxels_to_erase;
          for (auto &pair : voxel_map)
          {
               Eigen::Vector3d pt = pair.second.points[0];
               if ((pt - location).squaredNorm() > (options_.max_distance * options_.max_distance))
               {
                    voxels_to_erase.push_back(pair.first);
               }
          }
          for (auto &vox : voxels_to_erase)
               voxel_map.erase(vox);
          std::vector<voxel>().swap(voxels_to_erase);
     }

     cloudFrame *lidarodom::buildFrame(std::vector<point3D> &const_surf, state *cur_state,
                                       double timestamp_begin, double timestamp_end)
     {
          std::vector<point3D> frame_surf(const_surf);
          if (index_frame < 2)
          {
               for (auto &point_temp : frame_surf)
               {
                    point_temp.alpha_time = 1.0; //  alpha reset 0
               }
          }

          if (options_.motion_compensation == CONSTANT_VELOCITY)
               Undistort(frame_surf);

          for (auto &point_temp : frame_surf)
               transformPoint(options_.motion_compensation, point_temp, cur_state->rotation_begin,
                              cur_state->rotation, cur_state->translation_begin, cur_state->translation,
                              R_imu_lidar, t_imu_lidar);

          cloudFrame *p_frame = new cloudFrame(frame_surf, const_surf, cur_state);

          p_frame->time_frame_begin = timestamp_begin;
          p_frame->time_frame_end = timestamp_end;

          p_frame->dt_offset = 0;

          p_frame->frame_id = index_frame;

          return p_frame;
     }

     cloudFrame *lidarodom::buildFrame(std::vector<point3D> &const_surf, cv::Mat& img,  state *cur_state,
                                            double timestamp_begin, double timestamp_end)
     {
          std::vector<point3D> frame_surf(const_surf);
          if (index_frame < 2)
          {
               for (auto &point_temp : frame_surf)
               {
                    point_temp.alpha_time = 1.0; //  alpha reset 0
               }
          }

          if (options_.motion_compensation == CONSTANT_VELOCITY)
               Undistort(frame_surf);

          for (auto &point_temp : frame_surf)
               transformPoint(options_.motion_compensation, point_temp, cur_state->rotation_begin,
                              cur_state->rotation, cur_state->translation_begin, cur_state->translation,
                              R_imu_lidar, t_imu_lidar);

          cloudFrame *p_frame = new cloudFrame(frame_surf, const_surf, cur_state);

          p_frame->time_frame_begin = timestamp_begin;
          p_frame->time_frame_end = timestamp_end;

          p_frame->dt_offset = 0;

          p_frame->frame_id = index_frame;

          p_frame->img_ = img;

          return p_frame;
     }
     
     void lidarodom::stateInitialization()
     {
          // Eigen::Matrix3d R_align = computeGravityAlignment(g_odom, g_imu);

          if (index_frame < 2) //   only first frame 2 //50-3.53
          {
               if (!odomQueue.empty()){        
                    if(!odomQueue.front().header.stamp.toSec() > time_begin){
                         auto it = std::lower_bound(
                                        odomQueue.begin(), odomQueue.end(), time_begin,
                                        [](const nav_msgs::Odometry &odom, double time) {
                                        return odom.header.stamp.toSec() < time;});
                    
                         if (it != odomQueue.end()) {
                              startOdomMsg = *it;
                       
                              tf::Quaternion orientation;
                              tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);
                              double roll, pitch, yaw;
                              tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

                              Eigen::Quaterniond odom_quat(orientation.w(), orientation.x(), orientation.y(), orientation.z());
                              Eigen::Vector3d odom_trans(
                                                   startOdomMsg.pose.pose.position.x,
                                                   startOdomMsg.pose.pose.position.y,
                                                   startOdomMsg.pose.pose.position.z);
                              
                              current_state->rotation_begin = Eigen::Quaterniond(R_align * odom_quat.toRotationMatrix());
                              current_state->translation_begin = R_align * odom_trans;
                         }
                    }
                    if (!odomQueue.back().header.stamp.toSec() < time_curr){
                         auto it = std::lower_bound(odomQueue.begin(), odomQueue.end(), time_curr,
                           [](const nav_msgs::Odometry &odom, double time) {
                               return odom.header.stamp.toSec() < time;
                           });

                         if (it != odomQueue.end()) {
                            endOdomMsg = *it;

                            tf::Quaternion orientation;
                            tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
                            Eigen::Quaterniond odom_quat(orientation.w(), orientation.x(), orientation.y(), orientation.z());
                            Eigen::Vector3d odom_trans(
                                    endOdomMsg.pose.pose.position.x,
                                    endOdomMsg.pose.pose.position.y,
                                    endOdomMsg.pose.pose.position.z);
                         
                            current_state->rotation = Eigen::Quaterniond(R_align * odom_quat.toRotationMatrix());
                            current_state->translation = R_align * odom_trans;
                         }
                    }
                    if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0]))){
                         current_state->rotation_begin = Eigen::Quaterniond(imu_states_.front().R_.matrix());
                         current_state->translation_begin = imu_states_.front().p_;
                         current_state->rotation = Eigen::Quaterniond(imu_states_.back().R_.matrix());
                         current_state->translation = imu_states_.back().p_; 
                    }
               }
               else{
                       current_state->rotation_begin = Eigen::Quaterniond(imu_states_.front().R_.matrix());
                       current_state->translation_begin = imu_states_.front().p_;
                       current_state->rotation = Eigen::Quaterniond(imu_states_.back().R_.matrix());
                       current_state->translation = imu_states_.back().p_;
               }
          }
          else
          {
               //   use last pose
               current_state->rotation_begin = all_state_frame[all_state_frame.size() - 1]->rotation;
               current_state->translation_begin = all_state_frame[all_state_frame.size() - 1]->translation;
               // current_state->rotation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;
               // current_state->translation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation;
               //   use imu predict
               current_state->rotation = Eigen::Quaterniond(imu_states_.back().R_.matrix());
               current_state->translation = imu_states_.back().p_;
               // current_state->rotation = q_next_end;
               // current_state->translation = t_next_end;
          }
     }

     std::vector<MeasureGroup> lidarodom::getMeasureMents()
     {
          std::vector<MeasureGroup> measurements;
          while (true)
          {
               // if (imu_buffer_.empty())
               //      return measurements;

               // if (lidar_buffer_.empty())
               //      return measurements;

               // if (imu_buffer_.back()->timestamp_ - time_curr < delay_time_)
               //      return measurements;
               
               if (imu_buffer_.empty() || lidar_buffer_.empty() || img_buffer_.empty())
                    return measurements;

               if (imu_buffer_.back()->timestamp_ - time_curr < delay_time_)
                    return measurements;

               if (imu_buffer_.back()->timestamp_ < img_time_buffer_.front())
                    return measurements;
               
               auto curr_lidar = lidar_buffer_.front();
               auto lidar = last_lidar_;
               lidar.reserve(lidar.size() + curr_lidar.size());
               lidar.insert(lidar.end(), curr_lidar.begin(), curr_lidar.end());
               lidar_buffer_.pop_front();
               time_buffer_.pop_front();
               double begin_time = lidar.front().timestamp;
               double end_time = lidar.back().timestamp;

               if (!odomQueue.empty() && odomQueue.front().header.stamp.toSec() < time_begin - delay_time_)
                    odomQueue.pop_front();

               std::vector<double> times;
               std::vector<cv::Mat> images;
               int count = 0;
               while (!img_time_buffer_.empty() && !img_buffer_.empty() && img_time_buffer_.front() <= end_time)
               {
                    ++count;
                    if (count > 2 && img_time_buffer_.front() >= begin_time)
                    {
                         images.emplace_back(img_buffer_.front());
                         times.emplace_back(img_time_buffer_.front());
                    }
                    img_buffer_.pop_front();
                    img_time_buffer_.pop_front();
               }
               
               if (times.empty())
               {
                    last_lidar_ = lidar;
                    return measurements;
               }

               
               std::vector<std::vector<point3D>> points;
               std::vector<std::deque<IMUPtr>> imus;

               size_t p = 0; 
               // double left = begin_time;
               for (size_t k = 0; k < times.size(); ++k)
               {
                    double right = times[k];
                    
                    size_t p_left = p;
                    while (p < lidar.size() && lidar[p].timestamp <= right)
                         ++p;
                    points.emplace_back(lidar.begin() + p_left, lidar.begin() + p);
                    // left = right;
               }

               
               last_lidar_.clear();
               last_lidar_.insert(last_lidar_.end(), lidar.begin() + p, lidar.end());

               
               double prev_time = begin_time;
               for (size_t k = 0; k < times.size(); ++k) {
                    double curr_time = times[k];
                    std::deque<IMUPtr> curr_imu;
                    while (!imu_buffer_.empty() && imu_buffer_.front()->timestamp_ <= curr_time)
                    {
                         if (imu_buffer_.front()->timestamp_ > prev_time)
                         {
                              curr_imu.push_back(imu_buffer_.front());
                         }
                         imu_buffer_.pop_front();
                    }
                    imus.emplace_back(curr_imu);
                    prev_time = curr_time;
               }

               
               for (size_t k = 0; k < times.size(); ++k)
               {
                    
                    MeasureGroup meas;
                    meas.lidar_ = points[k];
                    meas.lidar_begin_time_ = k == 0 ? begin_time : times[k-1];
                    meas.lidar_end_time_ = times[k];
                    meas.imu_ = imus[k];
                    meas.img_ = images[k];
                    measurements.emplace_back(meas);
               }
               time_curr = end_time;
          
          }
     }

     void lidarodom::Predict()
     {
          imu_states_.emplace_back(eskf_.GetNominalState());

          
          double time_current = measures_.lidar_end_time_;
          Vec3d last_gyr, last_acc;
          for (auto &imu : measures_.imu_)
          {
               double time_imu = imu->timestamp_;
               if (imu->timestamp_ <= time_current)
               {
                    if (last_imu_ == nullptr)
                         last_imu_ = imu;
                    eskf_.Predict(*imu);
                    imu_states_.emplace_back(eskf_.GetNominalState());
                    last_imu_ = imu;
               }
               else
               {
                    double dt_1 = time_imu - time_current;
                    double dt_2 = time_current - last_imu_->timestamp_;
                    double w1 = dt_1 / (dt_1 + dt_2);
                    double w2 = dt_2 / (dt_1 + dt_2);
                    Eigen::Vector3d acc_temp = w1 * last_imu_->acce_ + w2 * imu->acce_;
                    Eigen::Vector3d gyr_temp = w1 * last_imu_->gyro_ + w2 * imu->gyro_;
                    IMUPtr imu_temp = std::make_shared<zjloc::IMU>(time_current, gyr_temp, acc_temp);
                    eskf_.Predict(*imu_temp);
                    imu_states_.emplace_back(eskf_.GetNominalState());
                    last_imu_ = imu_temp;
               }
          }
     }

     void lidarodom::Undistort(std::vector<point3D> &points)
     {
          // auto &cloud = measures_.lidar_;
          auto imu_state = eskf_.GetNominalState(); 
          // std::cout << __FUNCTION__ << ", " << imu_state.timestamp_ << std::endl;
          SE3 T_end = SE3(imu_state.R_, imu_state.p_);

          
          for (auto &pt : points)
          {
               SE3 Ti = T_end;
               NavStated match;

               math::PoseInterp<NavStated>(
                   pt.timestamp, imu_states_, [](const NavStated &s)
                   { return s.timestamp_; },
                   [](const NavStated &s)
                   { return s.GetSE3(); },
                   Ti, match);

               pt.raw_point = TIL_.inverse() * T_end.inverse() * Ti * TIL_ * pt.raw_point;
          }
     }

     void lidarodom::TryInitIMU()
     {
          for (auto imu : measures_.imu_)
          {
               imu_init_.AddIMU(*imu);
          }

          if (imu_init_.InitSuccess())
          {
               
               zjloc::ESKFD::Options options;
               // options.gyro_var_ = sqrt(imu_init_.GetCovGyro()[0]);
               // options.acce_var_ = sqrt(imu_init_.GetCovAcce()[0]);
               // options.update_bias_acce_ = false;
               // options.update_bias_gyro_ = false;
               eskf_.SetInitialConditions(options, imu_init_.GetInitBg(), imu_init_.GetInitBa(), imu_init_.GetGravity());
               imu_need_init_ = false;

               std::cout << ANSI_COLOR_GREEN_BOLD << "IMU init" << ANSI_COLOR_RESET << std::endl;
          }
     }

     
}
