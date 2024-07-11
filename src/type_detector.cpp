#include <map4_point_type/type_detector.hpp>

namespace map4_pcl_extensions
{
PointType detectType(const std::string& file_name)
{
  pcl::PCDReader reader;
  pcl::PCLPointCloud2 cloud;
  reader.readHeader(file_name, cloud);

  return detectType(cloud.fields);
}

PointType detectType(const std::vector<pcl::PCLPointField>& fields)
{
  int type_num = 1, sign = 1;
  for (int i = 0; i < fields.size(); i++)
  {
    int ret = checkFieldName(fields[i]);
    if (ret < 0)
      sign = -1;
    else
      type_num += ret;
  }
  return castPointType(sign * type_num);
}

int checkFieldName(const pcl::PCLPointField& field)
{
  std::string field_name = field.name;
  uint8_t datatype = field.datatype;
  if ((field_name == "x" || field_name == "y" || field_name == "z"))
  {
    if (datatype == 7)
      return 0;
    else if (datatype == 8)
      return -1;
  }
  else if (field_name == "intensity")
  {
    return 1;
  }
  else if (field_name == "ring")
  {
    return 2;
  }
  else if (field_name == "time")
  {
    return 4;
  }
  else if (field_name == "rgb")
  {
    return 8;
  }
  else if (field_name == "stamp")
  {
    return 16;
  }
  else if (field_name == "classification")
  {
    return 32;
  }
  else if (field_name == "timestamp")
  {
    return 64;
  }
  else if (field_name == "ambient")
  {
    return 128;
  }
  else if (field_name == "amplitude")
  {
    return 256;
  }
  else if (field_name == "label")
  {
    return 512;
  }
  else
  {
    return 0;
  }
}
}  // namespace map4_pcl_extensions
