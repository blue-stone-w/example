#ifndef POSESTATE
#define POSESTATE

#include <Eigen/Core>
// tips: positon, orientation, velocity are included and dimension is 9;
class PoseState
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // variable's position 状态的相应分量在噪声向量、协方差矩阵和状态向量中的起始位置
  static constexpr unsigned int DIM_OF_STATE = 9; // dimension of state
  static constexpr unsigned int DIM_OF_NOISE = 9; // dimension of noise
  static constexpr unsigned int POS = 0;  // positipon: x, y, z
  static constexpr unsigned int ROT = 3;  // attitude : roll, pitch, yaw
  static constexpr unsigned int VEL = 6;  // velocity : vx, vy, vz
  static constexpr unsigned int ACC = 9;  // acceleration
  static constexpr unsigned int GYR = 12; // gyroscope / angular velocity

  // constructor and destructor
  PoseState()
  {
    initialize();
  }

  void initialize()
  {
    stateVec.setZero();
    noiseVec.fill(FLT_MAX);
    covariance = noiseVec.asDiagonal();
  }

  PoseState(const std_msgs::Header headerIn, 
            const Eigen::Matrix<double, DIM_OF_STATE, 1> &stateIn, 
            const Eigen::Matrix<double, DIM_OF_NOISE, 1> &noiseIn)
  {
    header = headerIn;
    time = headerIn.stamp.toSec();
    setState(stateIn);
    setNoise(noiseIn);
  }

  ~PoseState() {}

  // set member varibles
  void setState(const Eigen::Matrix<double, DIM_OF_STATE, 1> &stateIn)
  {
    stateVec = stateIn;
    Eigen::Matrix<double, 3, 1> tm = stateVec.block<3,1>(ROT,0);
    quat = rpy2Quat(stateVec.block<3,1>(ROT,0));
  }

  setPose(const Eigen::Matrix<double, 3, 1> &poseIn)
  {
    stateVec.block<3,1>(POS,0) = poseIn;
  }

  setRotation(const Eigen::Matrix<double, 3, 1> &rotIn)
  {
    stateVec.block<3,1>(ROT,0) = rotIn;
    quat = rpy2Quat(rotIn);
  }

  void setNoise(const Eigen::Matrix<double, DIM_OF_NOISE, 1> &noiseIn)
  {
    noiseVec = noiseIn;
    covariance = noiseVec.asDiagonal();
  }

  void setCovariance(const Eigen::Matrix<double, DIM_OF_NOISE, DIM_OF_NOISE> &covarianceIn)
  {
    covariance = covarianceIn;
  }

  void setHeader(const std_msgs::Header headerIn)
  {
    header = headerIn;
    time = headerIn.stamp.toSec();
  }

  void setTime(const double timeIn)
  {
    time = timeIn;
  }

  void setTime(const std_msgs::Header::stamp stampIn)
  {
    time = stampIn.toSec();
    header.stamp = stampIn;
  }

  // get member varible
  Eigen::Matrix<double, 3, 1> getPose()
  {
    return stateVec.block<3,1>(POS,0);
  }

  Eigen::Matrix<double, 3, 1> getRotation()
  {
    return stateVec.block<3,1>(ROT,0);
  }

  Eigen::Quaternion<double> getQuaternion()
  {
    return quat;
  }


  // overload operator
  PoseState& operator=(const PoseState& other)
  { 
    if (this == &other) {
      return *this;
    }

    this->stateVec = other.stateVec;
    this->time = other.time;
    this->header = other.header;
    this->noiseVec = other.noiseVec;
    this->covariance = other.covariance;

    return *this;
  }

  // plus and minus only state; return other information in front object
  // front object shouldn't be edited;
  PoseState operator+(const PoseState& pose1) 
  {
    PoseState t_poseState(*this);
    t_poseState.stateVec = t_poseState.stateVec + pose1.stateVec;
    return t_poseState;
  }
  
  PoseState operator-(const PoseState& pose1) 
  {
    PoseState t_poseState(*this);
    t_poseState.stateVec = t_poseState.stateVec - pose1.stateVec;
    return t_poseState;
  }

 private:  
  double time;
  std_msgs::Header header;
  Eigen::Matrix<double, DIM_OF_STATE, 1> stateVec;
  Eigen::Matrix<double, DIM_OF_NOISE, 1> noiseVec;
  Eigen::Matrix<double, DIM_OF_NOISE, DIM_OF_NOISE> covariance;
  Eigen::Quaternion<double> quat;
};

#endif
