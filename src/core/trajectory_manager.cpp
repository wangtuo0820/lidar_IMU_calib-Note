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
#include <core/trajectory_manager.h>
#include <utils/math_utils.h>
#include <utils/eigen_utils.hpp>
#include <utils/ceres_callbacks.h>

#include <memory>

namespace licalib {
using namespace kontiki::trajectories;

void TrajectoryManager::initialTrajTo(double max_time) {
  Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
  Eigen::Vector3d p0(0,0,0);
  traj_->R3Spline()->ExtendTo (max_time, p0);
  traj_->SO3Spline()->ExtendTo(max_time, q0);
}

// 向`imu_data_`添加数据
/*  struct IMUData {
 *    double timestamp;
 *    Eigen::Matrix<double, 3, 1> gyro;
 *    Eigen::Matrix<double, 3, 1> accel;
 *    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 *  };
 */

void TrajectoryManager::feedIMUData(const IO::IMUData& data) {
  imu_data_.emplace_back(data);
}

// 根据`SO3TrajEstimator`求解，初始化初始化旋转轨迹
void TrajectoryManager::initialSO3TrajWithGyro() {
  assert(imu_data_.size() > 0 &&
         "[initialSO3TrajWithGyro]: There's NO imu data for initialization.");

  // 创建了一个SO3TrajEstimator本质是TrajectoryEstimator，内部管理一个ceres::Problem
  std::shared_ptr<SO3TrajEstimator> estimator_SO3; 
  estimator_SO3 = std::make_shared<SO3TrajEstimator>(traj_->SO3Spline()); // traj_->SO3Spline()是trajectory_

  std::cout << "===================== Before addGyroscopeMeasurements ========================" << std::endl;
  std::cout << "NumKnots: " << estimator_SO3->trajectory()->NumKnots() << std::endl;
  std::cout << "NumSegs: " << estimator_SO3->trajectory()->NumSegs() << std::endl;
  printf("MinTime: %.5f\n", traj_->SO3Spline()->MinTime());
  printf("MaxTime: %.5f\n", traj_->SO3Spline()->MaxTime());
  printf("t0: %.5f\n", traj_->SO3Spline()->t0());
  std::cout << "dt: " << estimator_SO3->trajectory()->dt() << std::endl;

  // estimator_SO3在include/kontiki/trajectory_estimator.h中定义
  // 内部封装了ceres::Problem
  addGyroscopeMeasurements(estimator_SO3); // 将imu_data_加入gyro_list_，

  std::cout << "===================== After addGyroscopeMeasurements ========================" << std::endl;
  std::cout << "NumKnots: " << estimator_SO3->trajectory()->NumKnots() << std::endl;
  std::cout << "NumSegs: " << estimator_SO3->trajectory()->NumSegs() << std::endl;
  printf("MinTime: %.5f\n", traj_->SO3Spline()->MinTime());
  printf("MaxTime: %.5f\n", traj_->SO3Spline()->MaxTime());
  printf("t0: %.5f\n", traj_->SO3Spline()->t0());
  std::cout << "dt: " << estimator_SO3->trajectory()->dt() << std::endl;

  /// fix the initial pose of trajectory
  double weight_t0 = calib_param_manager->global_opt_gyro_weight;
  double t0 = traj_->SO3Spline()->MinTime(); // 轨迹的初始时间

  // TODO 这里好像没有影响啊？？？
  //Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
  Eigen::AngleAxisd rotation_vector(0.0001, Eigen::Vector3d(0,0,1)); // 定义一个初始放置的位姿
  Eigen::Quaterniond q0 = Eigen::Quaterniond (rotation_vector.matrix());
  auto m_q0 = std::make_shared<OrientationMeasurement>(t0, q0, weight_t0);
  estimator_SO3->AddMeasurement<OrientationMeasurement>(m_q0);

  ceres::Solver::Summary summary = estimator_SO3->Solve(30, false);
  std::cout << summary.BriefReport() << std::endl;

  std::cout << "===================== After BA ========================" << std::endl;
  std::cout << "NumKnots: " << estimator_SO3->trajectory()->NumKnots() << std::endl;
  std::cout << "NumSegs: " << estimator_SO3->trajectory()->NumSegs() << std::endl;
  std::cout << "MinTime: " << estimator_SO3->trajectory()->MinTime() << std::endl;
  std::cout << "MaxTime: " << estimator_SO3->trajectory()->MaxTime() << std::endl;
  std::cout << "t0: " << estimator_SO3->trajectory()->t0() << std::endl;
  std::cout << "dt: " << estimator_SO3->trajectory()->dt() << std::endl;
}

// 这里使用的是SplitTrajEstimator轨迹，而initialSO3TrajWithGyro使用的是SO3TrajEstimator轨迹 
// SplitTrajEstimator轨迹同时优化旋转和平移
void TrajectoryManager::trajInitFromSurfel(
        SurfelAssociation::Ptr surfels_association,
        bool opt_time_offset_) {
  lidar_->set_relative_orientation(calib_param_manager->q_LtoI);
  lidar_->set_relative_position(calib_param_manager->p_LinI); // 初始的相对位置估计为0 TODO 后期可以这里给一个合理的初值

  lidar_->LockRelativeOrientation(false);
  lidar_->LockRelativePosition(false);

  // 这里默认opt_time_offset_是false
  // 也就是说相信硬件的时间，否则会额外增加待优化的控制点个数
  if (opt_time_offset_ && time_offset_padding_ > 0) {
    lidar_->LockTimeOffset(false);
    lidar_->set_max_time_offset(time_offset_padding_);
  }
  else {
    lidar_->LockTimeOffset(true);
  }
  imu_->LockGyroscopeBias(false);
  imu_->LockAccelerometerBias(false);

  std::shared_ptr<SplitTrajEstimator> estimator_split;
  estimator_split = std::make_shared<SplitTrajEstimator>(traj_);

  // add constraints
  // 添加gyro，加速度计，和面元的约束
  addGyroscopeMeasurements(estimator_split);
  addAccelerometerMeasurement(estimator_split);
  addSurfMeasurement(estimator_split, surfels_association); // 面元的重投影约束

  // addCallback(estimator_split);

  //printErrorStatistics("Before optimization");
  ceres::Solver::Summary summary = estimator_split->Solve(30, false);
  std::cout << summary.BriefReport() << std::endl;
  printErrorStatistics("After optimization");

  calib_param_manager->set_p_LinI(lidar_->relative_position());
  calib_param_manager->set_q_LtoI(lidar_->relative_orientation());
  calib_param_manager->set_time_offset(lidar_->time_offset());
  calib_param_manager->set_gravity(imu_->refined_gravity());
  calib_param_manager->set_gyro_bias(imu_->gyroscope_bias());
  calib_param_manager->set_acce_bias(imu_->accelerometer_bias());
  calib_param_manager->showStates();
}

bool TrajectoryManager::evaluateIMUPose(double imu_time, int flags,
                                        Result &result) const {
  if (traj_->MinTime() > imu_time || traj_->MaxTime() <= imu_time)
    return false;
  result = traj_->Evaluate(imu_time, flags);
  return true;
}

bool TrajectoryManager::evaluateLidarPose(double lidar_time,
                                          Eigen::Quaterniond &q_LtoG,
                                          Eigen::Vector3d &p_LinG) const {
  double traj_time = lidar_time + lidar_->time_offset();
  if (traj_->MinTime() > traj_time || traj_->MaxTime() <= traj_time)
    return false;
  Result result = traj_->Evaluate( traj_time, EvalOrientation | EvalPosition);
  q_LtoG = result->orientation * calib_param_manager->q_LtoI;
  p_LinG = result->orientation * calib_param_manager->p_LinI + result->position;
  return true;
}

bool TrajectoryManager::evaluateLidarRelativeRotation(double lidar_time1,
        double lidar_time2, Eigen::Quaterniond &q_L2toL1) const {
  assert(lidar_time1 <= lidar_time2
         && "[evaluateRelativeRotation] : lidar_time1 > lidar_time2");

  double traj_time1 = lidar_time1 + lidar_->time_offset();
  double traj_time2 = lidar_time2 + lidar_->time_offset();

  if (traj_->MinTime() > traj_time1 || traj_->MaxTime() <= traj_time2)
    return false;

  Result result1 = traj_->Evaluate(traj_time1, EvalOrientation);
  Result result2 = traj_->Evaluate(traj_time2, EvalOrientation);
  Eigen::Quaterniond q_I2toI1 = result1->orientation.conjugate()*result2->orientation;

  q_L2toL1 = calib_param_manager->q_LtoI.conjugate() * q_I2toI1 * calib_param_manager->q_LtoI;
  return true;
}

// 将imu_data_中的陀螺仪观测加入到传入的estimator中
template <typename TrajectoryModel> // TrajectoryModel is SO3TrajEstimator
void TrajectoryManager::addGyroscopeMeasurements(
        std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator) {
  gyro_list_.clear();

  double weight = calib_param_manager->global_opt_gyro_weight;
  const double min_time = estimator->trajectory()->MinTime();
  const double max_time = estimator->trajectory()->MaxTime();

  for (const auto &v : imu_data_) {
    if ( min_time > v.timestamp || max_time <= v.timestamp) {
      continue;
    }
    // 这里将imu_data_中的IMUData的gyro(Eigen::Matrix<double, 3, 1>)转为GyroMeasurement
    // imu_是IMUSensor = kontiki::sensors::ConstantBiasImu
    // GyroMeasurement是kontiki::measurements::GyroscopeMeasurement<IMUSensor>
    // GyroscopeMeasurement中包含一个IMUSensor
    auto mg = std::make_shared<GyroMeasurement>(imu_, v.timestamp, v.gyro, weight); // v.gyro是观测，用来计算误差
    gyro_list_.push_back(mg);

    estimator->template AddMeasurement<GyroMeasurement>(mg); // 这里将陀螺仪的测量加入estimator构造的ceres::Problem中 // 这里也会创建四个控制点，这四个点作为优化项

    // TODO 下面是kontiki中AddMeasurement做的事情
    /*  ================== include/kontiki/trajectory_estimator.h ======================
     *  template<typename MeasurementType>
     *  void AddMeasurement(std::shared_ptr<MeasurementType> m) {
     *    m->AddToEstimator(*this);
     *  }
     *
     *  =============== include/kontiki/measurements/gyroscope_measurement.h =========== TODO 变量mg的类型
     *  struct Residual { // TODO 残差项调用measurement进行误差计算
     *      Residual(const GyroscopeMeasurement &m) : measurement(m) {};
     *  
     *      template <typename T>
     *      bool operator()(T const* const* params, T* residual) const {
     *        size_t offset = 0;
     *        const auto trajectory = entity::Map<TrajectoryModel, T>(&params[offset], trajectory_meta);
     *        offset += trajectory_meta.NumParameters();
     *        const auto imu = entity::Map<ImuModel, T>(&params[offset], imu_meta);
     *  
     *        Eigen::Map<Eigen::Matrix<T,3,1>> r(residual);
     *        r = measurement.Error<TrajectoryModel, T>(imu, trajectory); 
     *        return true;
     *      }
     *  
     *      const GyroscopeMeasurement& measurement;
     *      typename ImuModel::Meta imu_meta;
     *      typename TrajectoryModel::Meta trajectory_meta;
     *    }; // Residual;
     *
     *
     *    template<typename TrajectoryModel>
     *    void AddToEstimator(kontiki::TrajectoryEstimator<TrajectoryModel>& estimator) {
     *      using ResidualImpl = Residual<TrajectoryModel>;
     *      auto residual = new ResidualImpl(*this);
     *      auto cost_function = new ceres::DynamicAutoDiffCostFunction<ResidualImpl>(residual); // TODO 这个residual用来计算观测与预测的残差
     *      std::vector<entity::ParameterInfo<double>> parameter_info;
     *  
     *      // Add trajectory to problem
     *      double tmin, tmax;
     *      if (this->imu_->TimeOffsetIsLocked()) {
     *        tmin = t;
     *        tmax = t;
     *      }
     *      else {
     *        tmin = t - this->imu_->max_time_offset();
     *        tmax = t + this->imu_->max_time_offset();
     *      }
     *      estimator.AddTrajectoryForTimes({{tmin, tmax}}, residual->trajectory_meta, parameter_info); // TODO 调用estimator.AddTrajectoryForTimes --> trajectory_->AddToProblem(problem_, times, meta, parameter_info);
     *  
     *      // Add IMU to problem
     *      imu_->AddToProblem(estimator.problem(), {{tmin, tmax}}, residual->imu_meta, parameter_info); // TODO imu_是ConstantBiasImu类型
     *  
     *      // Let cost function know about the number and sizes of parameters dynamically added
     *      for (auto& pi : parameter_info) {
     *        cost_function->AddParameterBlock(pi.size);
     *      }
     *  
     *      // Add measurement
     *      cost_function->SetNumResiduals(3);
     *      estimator.problem().AddResidualBlock(cost_function, nullptr, entity::ParameterInfo<double>::ToParameterBlocks(parameter_info));
     *    }
     *
     *
     *  ======================= include/kontiki/sensors/constant_bias_imu.h ====================
     *  void AddToProblem(ceres::Problem &problem,
     *              time_init_t times,
     *              MetaType &meta,
     *              std::vector<entity::ParameterInfo<double>> &parameters) const override {
     *       Base::AddToProblem(problem, times, meta, parameters); // TODO 调用的是include/kontiki/sensors/sensors.h中的AddToProblem

     *       auto p_ab = this->pstore_->Parameter(this->PARAM_ABIAS);
     *       problem.AddParameterBlock(p_ab.data, p_ab.size, p_ab.parameterization);
     *       parameters.push_back(p_ab);

     *       if (acc_bias_locked_)
     *         problem.SetParameterBlockConstant(p_ab.data);

     *       auto p_gb = this->pstore_->Parameter(this->PARAM_GBIAS);
     *       problem.AddParameterBlock(p_gb.data, p_gb.size, p_gb.parameterization);
     *       parameters.push_back(p_gb);

     *       if (gyro_bias_locked_)
     *         problem.SetParameterBlockConstant(p_gb.data);
     *     }
     *
     *  ========================== include/kontiki/sensors/sensors.h =========================
     *  void AddToProblem(ceres::Problem &problem,
     *              time_init_t times,
     *              MetaType &meta,
     *              std::vector<entity::ParameterInfo<double>> &parameters) const override {
     *       auto pi_qct = this->pstore_->Parameter(0);
     *       auto pi_pct = this->pstore_->Parameter(1);
     *       auto pi_offset = this->pstore_->Parameter(2);
     *   
     *       // Relative orientation q_ct
     *       problem.AddParameterBlock(pi_qct.data, pi_qct.size, pi_qct.parameterization);
     *       parameters.push_back(pi_qct);
     *   
     *       if (relative_orientation_locked_)
     *         problem.SetParameterBlockConstant(pi_qct.data);
     *   
     *       // Relative translation p_ct
     *       problem.AddParameterBlock(pi_pct.data, pi_pct.size, pi_pct.parameterization);
     *       parameters.push_back(pi_pct);
     *   
     *       if (relative_position_locked_)
     *         problem.SetParameterBlockConstant(pi_pct.data);
     *   
     *       // Time offset is constrained to (-d, d)
     *       problem.AddParameterBlock(pi_offset.data, pi_offset.size, pi_offset.parameterization);
     *       problem.SetParameterLowerBound(pi_offset.data, 0, -this->max_time_offset());
     *       problem.SetParameterUpperBound(pi_offset.data, 0, this->max_time_offset());
     *       parameters.push_back(pi_offset);
     *   
     *       if (time_offset_locked_)
     *         problem.SetParameterBlockConstant(pi_offset.data);
     *     }
     *    
     *    
     * ================= Kontiki/include/kontiki/trajectories/spline_base.h =========================
     * void AddToProblem(ceres::Problem &problem,
     *              time_init_t times,
     *              SplineMeta &meta,
     *              std::vector<entity::ParameterInfo<double>> &parameters) const override {
     *      double master_dt = segment_entity_->dt();
     *      double master_t0 = segment_entity_->t0();
     *      int current_segment_start = 0;
     *      int current_segment_end = -1; // Negative signals no segment created yet
     *  
     *      // Times are guaranteed to be sorted correctly and t2 >= t1
     *      for (auto tt : times) {
     *  
     *        int i1, i2;
     *        double u_notused;
     *        segment_entity_->CalculateIndexAndInterpolationAmount(tt.first, i1, u_notused);
     *        segment_entity_->CalculateIndexAndInterpolationAmount(tt.second, i2, u_notused);
     *  
     *        // Create new segment, or extend the current one
     *        if (i1 > current_segment_end) {
     *          double segment_t0 = master_t0 + master_dt * i1;
     *          meta.segments.push_back(SplineSegmentMeta(master_dt, segment_t0));
     *          current_segment_start = i1;
     *        }
     *        else {
     *          i1 = current_segment_end + 1;
     *        }
     *  
     *        auto& current_segment_meta = meta.segments.back();
     *  
     *        // Add parameters and update currently active segment meta
     *        for (int i=i1; i < (i2 + 4); ++i) {
     *          auto pi = this->segment_entity_->Parameter(i);
     *          parameters.push_back(pi);
     *          problem.AddParameterBlock(pi.data, pi.size, pi.parameterization);
     *  
     *          if (this->IsLocked())
     *            problem.SetParameterBlockConstant(pi.data);
     *  
     *          current_segment_meta.n += 1;
     *        }
     *  
     *        current_segment_end = current_segment_start + current_segment_meta.n - 1;
     *      } // for times
     *    }


     */ 
  }
}

template <typename TrajectoryModel>
void TrajectoryManager::addAccelerometerMeasurement(
        std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator) {
  accel_list_.clear();

  const double weight = calib_param_manager->global_opt_acce_weight;
  const double min_time = estimator->trajectory()->MinTime();
  const double max_time = estimator->trajectory()->MaxTime();

  for (auto const &v : imu_data_) {
    if ( min_time > v.timestamp || max_time <= v.timestamp) {
      continue;
    }
    auto ma = std::make_shared<AccelMeasurement>(imu_, v.timestamp, v.accel, weight);
    accel_list_.push_back(ma);
    estimator->template AddMeasurement<AccelMeasurement>(ma);
  }
}

// 将surf观测加入误差项
template <typename TrajectoryModel>
void TrajectoryManager::addSurfMeasurement(
        std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator,
        const SurfelAssociation::Ptr surfel_association) {
  const double weight = calib_param_manager->global_opt_lidar_weight;
  surfelpoint_list_.clear();
  closest_point_vec_.clear();
  // 获取所有的平面
  for (auto const& v: surfel_association->get_surfel_planes()) {
    closest_point_vec_.push_back(v.Pi);
  }

  map_time_ = surfel_association->get_maptime();
  // 将每个平面点加入到观测方程，计算旋转后到原平面的重投影误差作为优化项
  for (auto const &spoint : surfel_association->get_surfel_points()) {
    double time = spoint.timestamp;
    size_t plane_id = spoint.plane_id;

    auto msp = std::make_shared<SurfMeasurement> (lidar_, spoint.point,
                                                  closest_point_vec_.at(plane_id).data(), time, map_time_, 5.0, weight);
    surfelpoint_list_.push_back(msp);
    estimator->template AddMeasurement<SurfMeasurement>(msp);
  }
}

template <typename TrajectoryModel>
void TrajectoryManager::addCallback(
        std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator) {
  // Add callback for debug
  std::unique_ptr<CheckStateCallback> cb  = std::make_unique<CheckStateCallback>();
  cb->addCheckState("q_LtoI     :", 4, lidar_->relative_orientation().coeffs().data());
  cb->addCheckState("p_LinI     :", 3, lidar_->relative_position().data());
  cb->addCheckState("time_offset:", 1, &lidar_->time_offset());
  cb->addCheckState("g_roll     :", 1, &imu_->gravity_orientation_roll());
  cb->addCheckState("g_pitch    :", 1, &imu_->gravity_orientation_pitch());
  estimator->AddCallback(std::move(cb), true);
}

void TrajectoryManager::printErrorStatistics(const std::string& intro, bool show_gyro,
                                             bool show_accel, bool show_lidar) const {
  std::cout << "\n============== " << intro << " ================" << std::endl;

  if (show_gyro && !gyro_list_.empty()) {
    Eigen::Vector3d error_sum;
    for(auto const& m : gyro_list_) {
      error_sum += m->ErrorRaw<SplitTrajectory> (*traj_).cwiseAbs();
    }
    std::cout << "[Gyro]  Error size, average: " << gyro_list_.size()
              << "; " << (error_sum/gyro_list_.size()).transpose() << std::endl;
  }

  if (show_accel && !accel_list_.empty()) {
    Eigen::Vector3d error_sum;
    for(auto const& m : accel_list_) {
      error_sum += m->ErrorRaw<SplitTrajectory> (*traj_).cwiseAbs();
    }
    std::cout << "[Accel] Error size, average: " << accel_list_.size()
              << ";  " << (error_sum/accel_list_.size()).transpose() << std::endl;
  }

  if (show_lidar && !surfelpoint_list_.empty()) {
    Eigen::Matrix<double,1,1>  error_sum;
    for (auto const &m : surfelpoint_list_) {
      error_sum += m->point2plane<SplitTrajectory>(*traj_).cwiseAbs();
    }
    std::cout << "[LiDAR] Error size, average: " << surfelpoint_list_.size()
              << "; " << (error_sum/surfelpoint_list_.size()).transpose() << std::endl;
  }

  std::cout << std::endl;
}

}
