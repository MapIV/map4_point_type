#ifndef TYPE_DETECTOR_HPP
#define TYPE_DETECTOR_HPP

#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>

namespace map4_pcl_extensions
{
enum class PointType
{
  PointXYZ = 0,
  PointXYZI = 1,
  PointXYZIR = 3,
  PointXYZIRT = 7,
  PointXYZRGB = 8,
  PointXYZIS = 17,
  PointXYZISC = 49,
  PointXYZRGBS = 24,
  PointXYZIRGBS = 25,
  PointXYZIRGBSC = 57,
  PointXYZIT = 67,
  PointOUSTER = 131,
};

PointType detectType(const std::string& file_name);
PointType detectType(const std::vector<std::string>& fields);
int checkFieldName(const std::string& field_name);
inline PointType castPointType(const int& i)
{
  switch (i)
  {
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
    default:
      std::cerr << "\033[31;1mError: Undefined PointType is detected: " << i << "\033[m" << std::endl;
      std::cerr << "Supported types are below:" << std::endl;
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
    default:
      std::cerr << "\033[31;1mError: Undefined PointType is detected: " << i << "\033[m" << std::endl;
      exit(4);
  }
}
}  // namespace map4_pcl_extensions

#endif
