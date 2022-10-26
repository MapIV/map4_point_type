#ifndef MAP4_POINT_TYPE_DOUBLE_HPP
#define MAP4_POINT_TYPE_DOUBLE_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct PointDXYZT
{
  double x;
  double y;
  double z;
  std::uint32_t sec;
  std::uint32_t nsec;
};

struct PointDXYZ
{
  double x, y, z;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointDXYZ, (double, x, x)(double, y, y)(double, z, z))

struct PointDXYZI
{
  double x, y, z;
  PCL_ADD_INTENSITY;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointDXYZI, (double, x, x)(double, y, y)(double, z, z)(float, intensity, intensity))

struct PointDXYZRGB
{
  double x, y, z;
  PCL_ADD_RGB;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointDXYZRGB, (double, x, x)(double, y, y)(double, z, z)(float, rgb, rgb))

struct PointDXYZIRT
{
  double x, y, z;
  PCL_ADD_INTENSITY;
  std::uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointDXYZIRT, (double, x, x)(double, y, y)(double, z, z)(float, intensity, intensity)(
                                                   std::uint16_t, ring, ring)(float, time, time))

struct PointDXYZIR
{
  double x, y, z;
  PCL_ADD_INTENSITY;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointDXYZIR, (double, x, x)(double, y, y)(double, z, z)(float, intensity,
                                                                                      intensity)(std::uint16_t, ring, ring))

struct PointDXYZIS
{
  double x, y, z;
  PCL_ADD_INTENSITY;
  double stamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointDXYZIS,
                                  (double, x, x)(double, y, y)(double, z, z)(float, intensity,
                                                                          intensity)(double, stamp, stamp))

struct PointDXYZISC
{
  double x, y, z;
  PCL_ADD_INTENSITY;
  double stamp;
  unsigned char classification;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointDXYZISC,
                                  (double, x, x)(double, y, y)(double, z, z)(float, intensity,
                                                                          intensity)(double, stamp, stamp)(unsigned char, classification, classification))

struct PointDXYZRGBS
{
  double x, y, z;
  PCL_ADD_RGB;
  double stamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointDXYZRGBS, (double, x, x)(double, y, y)(double, z, z)(float, rgb, rgb)(double, stamp, stamp))

struct PointDXYZIRGBS
{
  double x, y, z;
  PCL_ADD_INTENSITY;
  PCL_ADD_RGB;
  double stamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointDXYZIRGBS, (double, x, x)(double, y, y)(double, z, z)(float, intensity, intensity)(float, rgb, rgb)(double, stamp, stamp))

struct PointDXYZIRGBSC
{
  double x, y, z;
  PCL_ADD_INTENSITY;
  PCL_ADD_RGB;
  double stamp;
  unsigned char classification;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointDXYZIRGBSC, (double, x, x)(double, y, y)(double, z, z)(float, intensity, intensity)(float, rgb, rgb)(double, stamp, stamp)(unsigned char, classification, classification))

struct PointDXYZIT
{
  double x, y, z;
  PCL_ADD_INTENSITY;
  std::uint16_t ring;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointDXYZIT, (double, x, x)(double, y, y)(double, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(double, timestamp, timestamp))

#endif
