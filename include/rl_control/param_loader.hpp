#pragma once
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <XmlRpcValue.h>
#include <vector>


using namespace Eigen;

// inline void loadMatrix(ros::NodeHandle& nh, const std::string& name, MatrixXd& mat) {
//   XmlRpc::XmlRpcValue param;
//   if (!nh.getParam(name, param)) {
//     ROS_ERROR_STREAM("Failed to load matrix: " << name);
//     return;
//   }
//   if (param.hasMember("rows") && param.hasMember("cols") && param.hasMember("data")) {
//     int rows = static_cast<int>(param["rows"]);
//     int cols = static_cast<int>(param["cols"]);
//     std::vector<double> data;
//     for (int i = 0; i < param["data"].size(); ++i)
//       data.push_back(static_cast<double>(param["data"][i]));
//     using RowMajorMat = Matrix<double, Dynamic, Dynamic, RowMajor>;
//     const Map<const RowMajorMat> M(data.data(), rows, cols);
//     mat = M;
//   }
// }


// template <typename Derived>
// inline bool loadMatrix(ros::NodeHandle& nh,
//                        const std::string& name,
//                        Eigen::MatrixBase<Derived>& mat)
// {
//     XmlRpc::XmlRpcValue param;
//     if (!nh.getParam(name, param)) {
//         ROS_ERROR_STREAM("Failed to load parameter: " << name);
//         return false;
//     }

//     if (!(param.hasMember("rows") && param.hasMember("cols") && param.hasMember("data"))) {
//         ROS_ERROR_STREAM("Invalid format for parameter: " << name
//                          << ". Expected {rows, cols, data}.");
//         return false;
//     }

//     int rows = static_cast<int>(param["rows"]);
//     int cols = static_cast<int>(param["cols"]);

//     std::vector<double> data;
//     for (int i = 0; i < param["data"].size(); ++i)
//         data.push_back(static_cast<double>(param["data"][i]));

//     if ((int)data.size() != rows * cols) {
//         ROS_ERROR_STREAM("Parameter " << name << " has " << data.size()
//                          << " elements, expected " << rows * cols);
//         return false;
//     }

//     // Interpreta os dados em row-major (ordem natural no YAML)
//     typedef Matrix<double, Dynamic, Dynamic, RowMajor> RowMajorMatrix;
//     const Map<const RowMajorMatrix> M(data.data(), rows, cols);

//     // Agora adapta o tipo automaticamente
//     if (rows == 1) {
//         // Vetor linha
//         typedef Matrix<double, 1, Dynamic, RowMajor> RowVec;
//         mat.derived() = Map<const RowVec>(data.data(), 1, cols);
//     }
//     else if (cols == 1) {
//         // Vetor coluna
//         mat.derived() = Map<const VectorXd>(data.data(), rows);
//     }
//     else {
//         // Matriz normal
//         mat.derived() = M;
//     }

//     return true;
// }

template <typename Derived>
inline bool loadMatrix(ros::NodeHandle& nh,
                       const std::string& name,
                       Eigen::MatrixBase<Derived>& mat)
{
    XmlRpc::XmlRpcValue param;
    if (!nh.getParam(name, param)) {
        ROS_ERROR_STREAM("Failed to load parameter: " << name);
        return false;
    }

    if (!(param.hasMember("rows") && param.hasMember("cols") && param.hasMember("data"))) {
        ROS_ERROR_STREAM("Invalid format for parameter: " << name
                         << ". Expected {rows, cols, data}.");
        return false;
    }

    int rows = static_cast<int>(param["rows"]);
    int cols = static_cast<int>(param["cols"]);

    std::vector<double> data;
    for (int i = 0; i < param["data"].size(); ++i)
        data.push_back(static_cast<double>(param["data"][i]));

    if ((int)data.size() != rows * cols) {
        ROS_ERROR_STREAM("Parameter " << name << " has " << data.size()
                         << " elements, expected " << rows * cols);
        return false;
    }

    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMajorMatrix;
    const Eigen::Map<const RowMajorMatrix> M(data.data(), rows, cols);

    mat.derived().resize(rows, cols);

    if (rows == 1)
        mat.derived() = Eigen::Map<const Eigen::RowVectorXd>(data.data(), cols);
    else if (cols == 1)
        mat.derived() = Eigen::Map<const Eigen::VectorXd>(data.data(), rows);
    else
        mat.derived() = M;

    ROS_DEBUG_STREAM("Loaded matrix: " << name << " (" << rows << "x" << cols << ")");
    return true;
}


inline void loadVector(ros::NodeHandle& nh, const std::string& name, VectorXd& vec) {
  std::vector<double> data;
  if (!nh.getParam(name, data)) {
    ROS_ERROR_STREAM("Failed to load vector: " << name);
    return;
  }
  vec = Map<VectorXd>(data.data(), data.size());
}

