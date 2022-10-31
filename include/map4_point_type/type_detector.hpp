#ifndef TYPE_DETECTOR_HPP
#define TYPE_DETECTOR_HPP

#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>

namespace map4_pcl_extensions
{
enum class PointType
{
  // Float
  PointXYZ = 1,
  PointXYZI = 2,
  PointXYZIR = 4,
  PointXYZIRT = 8,
  PointXYZRGB = 9,
  PointXYZIS = 18,
  PointXYZISC = 50,
  PointXYZRGBS = 25,
  PointXYZIRGBS = 26,
  PointXYZIRGBSC = 58,
  PointXYZIT = 68,
  PointOUSTER = 132,
  PointRIEGL = 262,

  // Double
  PointDXYZ = -1,
  PointDXYZI = -2,
  PointDXYZIR = -4,
  PointDXYZIRT = -8,
  PointDXYZRGB = -9,
  PointDXYZIS = -18,
  PointDXYZISC = -50,
  PointDXYZRGBS = -25,
  PointDXYZIRGBS = -26,
  PointDXYZIRGBSC = -58,
  PointDXYZIT = -68,
};

PointType detectType(const std::string& file_name);
PointType detectType(const std::vector<pcl::PCLPointField>& fields);
int checkFieldName(const pcl::PCLPointField& field);
inline PointType castPointType(const int& i)
{
  switch (i)
  {
    // Float
    case static_cast<int>(PointType::PointXYZ):
      return PointType::PointXYZ;
    case static_cast<int>(PointType::PointXYZI):
      return PointType::PointXYZI;
    case static_cast<int>(PointType::PointXYZIR):
      return PointType::PointXYZIR;
    case static_cast<int>(PointType::PointXYZIRT):
      return PointType::PointXYZIRT;
    case static_cast<int>(PointType::PointXYZRGB):
      return PointType::PointXYZRGB;
    case static_cast<int>(PointType::PointXYZIS):
      return PointType::PointXYZIS;
    case static_cast<int>(PointType::PointXYZISC):
      return PointType::PointXYZISC;
    case static_cast<int>(PointType::PointXYZRGBS):
      return PointType::PointXYZRGBS;
    case static_cast<int>(PointType::PointXYZIRGBS):
      return PointType::PointXYZIRGBS;
    case static_cast<int>(PointType::PointXYZIRGBSC):
      return PointType::PointXYZIRGBSC;
    case static_cast<int>(PointType::PointXYZIT):
      return PointType::PointXYZIT;
    case static_cast<int>(PointType::PointOUSTER):
      return PointType::PointOUSTER;
    case static_cast<int>(PointType::PointRIEGL):
      return PointType::PointRIEGL;

    // Double
    case static_cast<int>(PointType::PointDXYZ):
      return PointType::PointDXYZ;
    case static_cast<int>(PointType::PointDXYZI):
      return PointType::PointDXYZI;
    case static_cast<int>(PointType::PointDXYZIR):
      return PointType::PointDXYZIR;
    case static_cast<int>(PointType::PointDXYZIRT):
      return PointType::PointDXYZIRT;
    case static_cast<int>(PointType::PointDXYZRGB):
      return PointType::PointDXYZRGB;
    case static_cast<int>(PointType::PointDXYZIS):
      return PointType::PointDXYZIS;
    case static_cast<int>(PointType::PointDXYZISC):
      return PointType::PointDXYZISC;
    case static_cast<int>(PointType::PointDXYZRGBS):
      return PointType::PointDXYZRGBS;
    case static_cast<int>(PointType::PointDXYZIRGBS):
      return PointType::PointDXYZIRGBS;
    case static_cast<int>(PointType::PointDXYZIRGBSC):
      return PointType::PointDXYZIRGBSC;
    case static_cast<int>(PointType::PointDXYZIT):
      return PointType::PointDXYZIT;
    default:
      std::cerr << "\033[31;1mError: Undefined PointType is detected: " << i << "\033[m" << std::endl;
      std::cerr << "Supported types are below:" << std::endl;
      // Float
      std::cerr << "\tPointXYZ" << std::endl;
      std::cerr << "\tPointXYZI" << std::endl;
      std::cerr << "\tPointXYZIR" << std::endl;
      std::cerr << "\tPointXYZIRT" << std::endl;
      std::cerr << "\tPointXYZRGB" << std::endl;
      std::cerr << "\tPointXYZIS" << std::endl;
      std::cerr << "\tPointXYZISC" << std::endl;
      std::cerr << "\tPointXYZRGBS" << std::endl;
      std::cerr << "\tPointXYZIRGBSC" << std::endl;
      std::cerr << "\tPointXYZIT" << std::endl;
      std::cerr << "\tPointOUSTER" << std::endl;
      // Double
      std::cerr << "\tPointDXYZ" << std::endl;
      std::cerr << "\tPointDXYZI" << std::endl;
      std::cerr << "\tPointDXYZIR" << std::endl;
      std::cerr << "\tPointDXYZIRT" << std::endl;
      std::cerr << "\tPointDXYZRGB" << std::endl;
      std::cerr << "\tPointDXYZIS" << std::endl;
      std::cerr << "\tPointDXYZISC" << std::endl;
      std::cerr << "\tPointDXYZRGBS" << std::endl;
      std::cerr << "\tPointDXYZIRGBSC" << std::endl;
      std::cerr << "\tPointDXYZIT" << std::endl;
      std::cerr << "\033[m" << std::flush;
      exit(4);
  }
}
inline PointType configCastPointType(const int& i)
{
  switch (i)
  {
    case 0:
      return PointType::PointXYZ;
    case 1:
      return PointType::PointXYZI;
    case 2:
      return PointType::PointXYZIR;
    case 3:
      return PointType::PointXYZIRT;
    case 4:
      return PointType::PointXYZRGB;
    case 5:
      return PointType::PointXYZIS;
    case 6:
      return PointType::PointXYZISC;
    case 7:
      return PointType::PointXYZRGBS;
    case 8:
      return PointType::PointXYZIRGBS;
    case 9:
      return PointType::PointXYZIRGBSC;
    case 10:
      return PointType::PointXYZIT;
    case 11:
      return PointType::PointOUSTER;
    case -1:
      return PointType::PointDXYZ;
    case -2:
      return PointType::PointDXYZI;
    case -5:
      return PointType::PointDXYZRGB;
    case -7:
      return PointType::PointDXYZISC;
    case -10:
      return PointType::PointDXYZIRGBSC;
    default:
      std::cerr << "\033[31;1mError: Undefined PointType is detected: " << i << "\033[m" << std::endl;
      exit(4);
  }
}
}  // namespace map4_pcl_extensions

#endif
