#ifndef IRT_POINT_TYPES_H
#define IRT_POINT_TYPES_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct PointXYZT
{
  float x;
  float y;
  float z;
  std::uint32_t sec;
  std::uint32_t nsec;
};

struct PointXYZIRT
{
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  std::uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                                   std::uint16_t, ring, ring)(float, time, time))

struct PointXYZIR
{
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t,
                                                                                                       ring, ring))

struct PointXYZIS
{
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  double stamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIS, (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                                      intensity)(double, stamp, stamp))

struct PointXYZISC
{
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  double stamp;
  unsigned char classification;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZISC, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                                   double, stamp, stamp)(unsigned char, classification, classification))

struct PointXYZRGBS
{
  PCL_ADD_POINT4D;
  PCL_ADD_RGB;
  double stamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBS,
                                  (float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)(double, stamp, stamp))

struct PointXYZIRGBS
{
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  PCL_ADD_RGB;
  double stamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRGBS, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                                     float, rgb, rgb)(double, stamp, stamp))

struct PointXYZIRGBSC
{
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  PCL_ADD_RGB;
  double stamp;
  unsigned char classification;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRGBSC,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, rgb, rgb)(
                                      double, stamp, stamp)(unsigned char, classification, classification))

struct PointXYZIT
{
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  std::uint16_t ring;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                                  std::uint16_t, ring, ring)(double, timestamp, timestamp))

struct PointOUSTER
{
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  uint32_t t;
  uint16_t reflectivity;
  uint8_t ring;
  uint16_t ambient;
  uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointOUSTER, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                                   std::uint32_t, t, t)(std::uint16_t, reflectivity, reflectivity)(
                                                   std::uint8_t, ring, ring)(std::uint16_t, ambient,
                                                                             ambient)(std::uint32_t, range, range))

struct PointXYZIRADT
{
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  float azimuth;
  float distance;
  uint8_t return_type;
  double time_stamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIRADT, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(
                       float, azimuth, azimuth)(float, distance, distance)(std::uint8_t, return_type,
                                                                           return_type)(double, time_stamp, time_stamp))

struct PointXYZIL
{
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  std::uint32_t label;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIL,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint32_t,
                                                                                                       label, label))

#endif
