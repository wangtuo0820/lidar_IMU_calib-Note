/*
 * LI_Calib: An Open Platform for LiDAR-IMU Calibration
 * Copyright (C) 2020 Jiajun Lv
 * Copyright (C) 2020 Kewei Hu
 * Copyright (C) 2020 Jinhong Xu
 * Copyright (C) 2020 LI_Calib Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include <ui/calib_helper.h>
#include <core/scan_undistortion.h>
#include <utils/tic_toc.h>

#include <boost/filesystem.hpp>
#include <memory>
#include <sstream>

namespace licalib {

CalibrHelper::CalibrHelper(ros::NodeHandle& nh)
        : calib_step_(Start),
          iteration_step_(0),
          opt_time_offset_(false),
          plane_lambda_(0.6),
          ndt_resolution_(0.5),
          associated_radius_(0.05) {
  std::string topic_lidar;
  double bag_start, bag_durr;
  double scan4map;
  double knot_distance;
  double time_offset_padding;

  nh.param<std::string>("path_bag", bag_path_, "V1_01_easy.bag");
  nh.param<std::string>("topic_imu", topic_imu_, "/imu0");
  nh.param<std::string>("topic_lidar", topic_lidar, "/velodyne_packets");
  nh.param<double>("bag_start", bag_start, 0);
  nh.param<double>("bag_durr", bag_durr, -1);
  nh.param<double>("scan4map", scan4map, 15); // 用来建图的时间长度
  nh.param<double>("ndtResolution", ndt_resolution_, 0.5);
  nh.param<double>("time_offset_padding", time_offset_padding, 0.015);
  nh.param<double>("knot_distance", knot_distance, 0.02);

  if (!createCacheFolder(bag_path_)) {
    calib_step_ = Error;
  }

  {
    std::string lidar_model;
    nh.param<std::string>("LidarModel", lidar_model, "VLP_16");
    IO::LidarModelType lidar_model_type = IO::LidarModelType::VLP_16;
    if (lidar_model == "VLP_16") {
      lidar_model_type = IO::LidarModelType::VLP_16;
    } else {
      calib_step_ = Error;
      ROS_WARN("LiDAR model %s not support yet.", lidar_model.c_str());
    }
    /// read dataset
    std::cout << "\nLoad dataset from " << bag_path_ << std::endl;
    IO::LioDataset lio_dataset_temp(lidar_model_type);

    // TODO 这里读取数据
    lio_dataset_temp.read(bag_path_, topic_imu_, topic_lidar, bag_start, bag_durr);

    dataset_reader_ = lio_dataset_temp.get_data();
    dataset_reader_->adjustDataset(); // 将不在目标时间范围内的数据去掉
  }

  // 获取数据的起止时间
  map_time_ = dataset_reader_->get_start_time();
  scan4map_time_ = map_time_ + scan4map;
  double end_time = dataset_reader_->get_end_time();


  // 构建一个接收传感器构建航迹的类，封装的kontiki, kontiki内部会调用ceres
  traj_manager_ = std::make_shared<TrajectoryManager>( 
          map_time_, end_time, knot_distance, time_offset_padding);

  // 用于激光雷达去畸变
  scan_undistortion_ = std::make_shared<ScanUndistortion>(
          traj_manager_, dataset_reader_);

  // 激光雷达里程计 include/core/lidar_odometry.h 
  lidar_odom_ = std::make_shared<LiDAROdometry>(ndt_resolution_);

  rotation_initializer_ = std::make_shared<InertialInitializer>();

  surfel_association_ = std::make_shared<SurfelAssociation>(
          associated_radius_, plane_lambda_);
}

bool CalibrHelper::createCacheFolder(const std::string& bag_path) {
  boost::filesystem::path p(bag_path);
  if (p.extension() != ".bag") {
    return false;
  }
  cache_path_ = p.parent_path().string() + "/" + p.stem().string();
  boost::filesystem::create_directory(cache_path_);
  return true;
}

// Step 1
// 读取所有数据
// 使用kontiki优化控制点，拟合B样条
// 激光雷达NDT建图
// 使用SVD求解超定方程初始化imu和lidar的初始姿态角
void CalibrHelper::Initialization() {
  if (Start != calib_step_) {
    ROS_WARN("[Initialization] Need status: Start.");
    return;
  }

  // 遍历所有imu数据
  for (const auto& imu_data: dataset_reader_->get_imu_data()) {
    traj_manager_->feedIMUData(imu_data);
  }

  std::cout << "imu_data size: " << dataset_reader_->get_imu_data().size() << std::endl;

  // TODO 这里假设最初是水平放置的
  traj_manager_->initialSO3TrajWithGyro(); // 只用陀螺仪的观测初始化旋转姿态(通过ceres优化得到)

  // 遍历所有lidar数据
  for(const TPointCloud& raw_scan: dataset_reader_->get_scan_data()) {
    VPointCloud::Ptr cloud(new VPointCloud);
    TPointCloud2VPointCloud(raw_scan.makeShared(), cloud);
    double scan_timestamp = pcl_conversions::fromPCL(raw_scan.header.stamp).toSec();

    lidar_odom_->feedScan(scan_timestamp, cloud); // 加入激光雷达里程计，更新激光雷达里程计

    if (lidar_odom_->get_odom_data().size() < 30
        || (lidar_odom_->get_odom_data().size() % 10 != 0))
      continue;

    // 估计imu和lidar的初始旋转角
    if (rotation_initializer_->EstimateRotation(traj_manager_, 
                                                lidar_odom_->get_odom_data())) { // 如果超定方程有解，初始化结束
      Eigen::Quaterniond qItoLidar = rotation_initializer_->getQ_ItoS(); // 初始化得到imu到lidar的旋转四元数
      traj_manager_->getCalibParamManager()->set_q_LtoI(qItoLidar.conjugate()); // 共轭就是求逆

      Eigen::Vector3d euler_ItoL = qItoLidar.toRotationMatrix().eulerAngles(0,1,2);
      std::cout << "[Initialization] Done. Euler_ItoL initial degree: "
                << (euler_ItoL*180.0/M_PI).transpose() << std::endl;
      calib_step_ = InitializationDone;
      break;
    }
  }


  if (calib_step_ != InitializationDone)
    ROS_WARN("[Initialization] fails.");
}

// Step 2
// 首先根据imu里程计（连续的B样条）去畸变，之后根据去畸变后的点云重新构建ndt
// 根据重新构建的ndt，计算surfel_planes，区分三种不同方向的平面
// 在对原始点云中每个点进行匹配，匹配到相近的平面上
// 这一步会比较耗时，但是这里只进行面元计算和去畸变，没有进行优化
void CalibrHelper::DataAssociation() {
  std::cout << "[Association] start ...." << std::endl;
  TicToc timer;
  timer.tic();

  /// set surfel pap
  if (InitializationDone == calib_step_ ) {
    Mapping(); // 根据imu里程计去畸变，根据去畸变后的激光雷达重新构建里程计和lidar_odometry中的map_cloud_
    scan_undistortion_->undistortScanInMap(lidar_odom_->get_odom_data()); // 更新scan_undistortion_中的map_cloud_

    // TODO 根据ndt计算的VoxelGrid的特征向量和特征值，判断每个VoxelGrid的平面方向，将所有平面上的点加入到surfel_planes_中
    surfel_association_->setSurfelMap(lidar_odom_->getNDTPtr(), map_time_); // map_time_是数据集起始时间 
  } else if (BatchOptimizationDone == calib_step_ || RefineDone == calib_step_) {
    scan_undistortion_->undistortScanInMap();

    plane_lambda_ = 0.7;
    surfel_association_->setPlaneLambda(plane_lambda_);
    auto ndt_omp = LiDAROdometry::ndtInit(ndt_resolution_);
    ndt_omp->setInputTarget(scan_undistortion_->get_map_cloud());
    surfel_association_->setSurfelMap(ndt_omp, map_time_);
  } else {
      ROS_WARN("[DataAssociation] Please follow the step.");
      return;
  }

  /// get association
  for (auto const &scan_raw : dataset_reader_->get_scan_data()) { // 遍历每个时刻的点云
    auto iter = scan_undistortion_->get_scan_data_in_map().find( // 在地图中std::map<pcl::uint64_t, VPointCloud::Ptr>查找去畸变后的点云
            scan_raw.header.stamp);
    if (iter == scan_undistortion_->get_scan_data_in_map().end()) {
      continue;
    }
    surfel_association_->getAssociation(iter->second, scan_raw.makeShared(), 2); // 获取每个原始点对应的去畸变点
  }
  surfel_association_->averageTimeDownSmaple(); // 对每个原始点匹配到的去畸变点进行下采样
  std::cout << "Surfel point number: "
            << surfel_association_->get_surfel_points().size() << std::endl;
  std::cout<<GREEN<<"[Association] "<<timer.toc()<<" ms"<<RESET<<std::endl;

  if (surfel_association_->get_surfel_points().size() > 10){
    calib_step_ = DataAssociationDone;
  } else {
    ROS_WARN("[DataAssociation] fails.");
  }
}

// step3 
// 由于第一次不知道imu和lidar的位移误差，所有不是很准
// 这里根据imu和lidar surfel的误差项进行优化，得到imu内参和imu和lidar的外参
void CalibrHelper::BatchOptimization() {
  if (DataAssociationDone != calib_step_) {
    ROS_WARN("[BatchOptimization] Need status: DataAssociationDone.");
    return;
  }
  std::cout << "\n================ Iteration " << iteration_step_ << " ==================\n";

  TicToc timer;
  timer.tic();
  traj_manager_->trajInitFromSurfel(surfel_association_, opt_time_offset_); // 这里加入优化项，拟合的是SE3（包括位移和旋转）

  calib_step_ = BatchOptimizationDone;
  saveCalibResult(cache_path_ + "/calib_result.csv");
  std::cout<<GREEN<<"[BatchOptimization] "<<timer.toc()<<" ms"<<RESET<<std::endl;
}

// step4 
// 再次进行step3的batch optimization
// 有了step3得到的位移作为初始化，误差项会更小
// 论文中说进行4次优化后误差项会收敛
void CalibrHelper::Refinement() {
  if (BatchOptimizationDone > calib_step_) {
    ROS_WARN("[Refinement] Need status: BatchOptimizationDone.");
    return;
  }
  iteration_step_++;
  std::cout << "\n================ Iteration " << iteration_step_ << " ==================\n";

  // TODO 这里果然还要进行一次step2
  DataAssociation(); // 根据上一步优化后的标定参数重新进行一次，去畸变和面元计算

  if (DataAssociationDone != calib_step_) {
    ROS_WARN("[Refinement] Need status: DataAssociationDone.");
    return;
  }
  TicToc timer;
  timer.tic();

  traj_manager_->trajInitFromSurfel(surfel_association_, opt_time_offset_);
  calib_step_ = RefineDone;
  saveCalibResult(cache_path_ + "/calib_result.csv");

  std::cout<<GREEN<<"[Refinement] "<<timer.toc()<<" ms"<<RESET<<std::endl;
}

// DataAssociation中第一步
// 根据imu里程计去畸变，根据去畸变后的激光雷达重新构建里程计和全局地图
void CalibrHelper::Mapping(bool relocalization) { // relocalization默认是false
  bool update_map = true; // TODO 这里的update_map如果为false的话，ndt就没法构建了
  if (relocalization) {
    lidar_odom_->clearOdomData();
    update_map = false;
  } else { // 默认执行这里
    scan_undistortion_->undistortScan(); // 根据imu里程计去畸变 
    lidar_odom_ = std::make_shared<LiDAROdometry>(ndt_resolution_); // 构建了一个ndt激光雷达里程计
  }

  double last_scan_t = 0;
  // 根据去畸变后的激光雷达重新构建里程计和全局地图
  for (const auto& scan_raw: dataset_reader_->get_scan_data()) { // 遍历所有激光雷达 
    double scan_t = pcl_conversions::fromPCL(scan_raw.header.stamp).toSec();
    if (scan_t > scan4map_time_)
      update_map = false;
    auto iter = scan_undistortion_->get_scan_data().find(scan_raw.header.stamp);
    if (iter != scan_undistortion_->get_scan_data().end()) {
      Eigen::Matrix4d pose_predict = Eigen::Matrix4d::Identity();
      Eigen::Quaterniond q_L2toL1 = Eigen::Quaterniond::Identity();
      if (last_scan_t > 0 &&
          traj_manager_->evaluateLidarRelativeRotation(last_scan_t, scan_t, q_L2toL1)) {
        pose_predict.block<3,3>(0,0) = q_L2toL1.toRotationMatrix();
      }
      lidar_odom_->feedScan(scan_t, iter->second, pose_predict, update_map); // 更新地图
      last_scan_t = scan_t;
    }
  }
}


void CalibrHelper::saveCalibResult(const std::string& calib_result_file) const {
  if (!boost::filesystem::exists(calib_result_file)) {
    std::ofstream outfile;
    outfile.open(calib_result_file, std::ios::app);
    outfile << "bag_path" << ","
            << "imu_topic" << "," << "map_time" << "," << "iteration_step" << ","
            << "p_IinL.x" << "," << "p_IinL.y" << "," << "p_IinL.z" << ","
            << "q_ItoL.x" << "," << "q_ItoL.y" << "," << "q_ItoL" << ","
            << "q_ItoL.w" << ","
            << "time_offset" << ","
            << "gravity.x" << "," << "gravity.y" << "," << "gravity.z" << ","
            << "gyro_bias.x" << "," << "gyro_bias.y" << "," <<"gyro_bias.z" << ","
            << "acce_bias.z" << "," << "acce_bias.y" << "," <<"acce_bias.z" << "\n";
    outfile.close();
  }

  std::stringstream ss;
  ss << bag_path_;
  ss << "," << topic_imu_;
  ss << "," << map_time_;
  ss << "," << iteration_step_;
  std::string info;
  ss >> info;

  traj_manager_->getCalibParamManager()->save_result(calib_result_file, info);
}

// 将ndt地图和surfel地图保存起来
void CalibrHelper::saveMap() const {
  if (calib_step_ <= Start)
    return;
  std::string NDT_target_map_path = cache_path_ + "/NDT_target_map.pcd";
  lidar_odom_->saveTargetMap(NDT_target_map_path);

  std::string surfel_map_path = cache_path_ + "/surfel_map.pcd";
  surfel_association_->saveSurfelsMap(surfel_map_path);

  // 如果进行过refine，还会保存去畸变的地图
  if (RefineDone == calib_step_) {
    std::string refined_map_path = cache_path_ + "/refined_map.pcd";
    std::cout << "Save refined map to " << refined_map_path << "; size: "
              << scan_undistortion_->get_map_cloud()->size() << std::endl;
    pcl::io::savePCDFileASCII(refined_map_path, *scan_undistortion_->get_map_cloud());
  }
}

}
