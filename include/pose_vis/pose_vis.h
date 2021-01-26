#ifndef POSE_VIS_H
#define POSE_VIS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

#include <cstdint>

// std
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>
#include <fstream>

typedef Eigen::Matrix<float, 2, 1> Vector2f;
typedef Eigen::Matrix<float, 3, 1> Vector3f;
typedef Eigen::Matrix<float, 4, 1> Vector4f;

typedef Eigen::Matrix<float, 3, 3> Matrix3f;
typedef Eigen::Matrix<float, 4, 4> Matrix4f;


#pragma pack(push, 1)
typedef struct
{
    uint16_t POS_Status;
    uint16_t POS_Type;
    uint16_t Reserved;
    uint16_t Week;
    double Time;
    double latitude;
    double longitude;
    double altitue;
    double n_posistion_rmse;
    double e_posistion_rmse;
    double d_posistion_rmse;
    double qw;
    double qx;
    double qy;
    double qz;
    double roll_rmes;
    double pitch_rmse;
    double heading_rmse;

} fc_bpos_recoder_t;
#pragma pop

typedef struct Pose
{
    float latitude;
    float longitude;
    float altitue;
    float qw;
    float qx;
    float qy;
    float qz;
    double Time;

    Matrix3f R;
    Vector3f T;

    Pose()
        : latitude(0), longitude(0), altitue(0), qw(0), qx(0), qy(0), qz(0), Time(0)
    {
        R << 1, 0, 0,
             0, 1, 0,
             0, 0, 1;
        T << 0, 0, 0;
    }

    Pose(double x, double y, double z, double _qw, double _qx, double _qy, double _qz, double timestamp_in)
        : latitude(x), longitude(y), altitue(z), qw(_qw), qx(_qx), qy(_qy), qz(_qz), Time(timestamp_in)
    {
        R << 1 - 2*qy*qy - 2*qz*qz,   2*qx*qy - 2*qz*qw,   2*qx*qz + 2*qy*qw,
             2*qx*qy + 2*qz*qw,   1 - 2*qx*qx - 2*qz*qz,   2*qy*qz - 2*qx*qw,
             2*qx*qz - 2*qy*qw,   2*qy*qz + 2*qx*qw,   1 - 2*qx*qx - 2*qy*qy;
        T << latitude, longitude, altitue;
    }
} Pose;

class PoseVis
{
public:
    PoseVis() {}
    ~PoseVis() {}

    void CreatePoseFromData(std::string bpos_data_path);

    const std::vector<Pose>& get_fc_pos_vec() const {return _fc_pos_vec;}

private:
    std::vector<Pose> _fc_pos_vec;
};


#endif