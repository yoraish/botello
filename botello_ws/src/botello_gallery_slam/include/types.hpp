#include <functional>
#include <istream>
#include <map>
#include <string>
#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

/*
Structs that are used in this package.
Many of the structs here were borrowed from the Ceres examples. https://ceres-solver.googlesource.com/ceres-solver/+/master/examples
*/

namespace botello_gallery_slam
{

struct LandmarkObservation
{
  int frameId;
  int landmarkId;
  std::vector<cv::Point2f> corners;
  double confidence;
};

inline std::ofstream& operator<<(std::ofstream& outfile, LandmarkObservation& landmarkObs) {
    outfile << "EDGE_UV " <<
    landmarkObs.frameId << " " <<
    landmarkObs.landmarkId << " ";

    for (cv::Point2f p : landmarkObs.corners)
    {
        outfile << p.x << " " << p.y << " ";
    }
    return outfile;
}

// Below, all structs are from the Ceres examples, as mentioned above.
struct Pose3d {
  Eigen::Vector3d p;
  Eigen::Quaterniond q;
  unsigned int frameId;

  // The name of the data type in the g2o file format.
  static std::string name() { return "VERTEX_SE3:QUAT"; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline std::ofstream& operator<<(std::ofstream& outfile, Pose3d& pose) {
    outfile << "VERTEX_SE3:QUAT " <<
    pose.frameId << " " <<
    pose.p.x() << " " <<
    pose.p.y() << " " <<
    pose.p.z() << " " <<
    pose.q.x() << " " <<  // TODO(yoraish): is this the correct ordering of quaternion according to g2o?
    pose.q.y() << " " << 
    pose.q.z() << " " << 
    pose.q.w();
    return outfile;
}


using MapOfPoses =
    std::map<int,
             Pose3d,
             std::less<int>,
             Eigen::aligned_allocator<std::pair<const int, Pose3d>>>;

// The constraint between two vertices in the pose graph. The constraint is the
// transformation from vertex id_begin to vertex id_end.
struct Constraint3d {
  int id_begin;
  int id_end;

  // The transformation that represents the pose of the end frame E w.r.t. the
  // begin frame B. In other words, it transforms a vector in the E frame to
  // the B frame.
  Pose3d t_be;

  // The inverse of the covariance matrix for the measurement. The order of the
  // entries are x, y, z, delta orientation.
  Eigen::Matrix<double, 6, 6> information;

  // The name of the data type in the g2o file format.
  static std::string name() { return "EDGE_SE3:QUAT"; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


} // End namespace botello_gallery_slam
