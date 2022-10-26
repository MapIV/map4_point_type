#include <map4_point_type/type_detector.hpp>

namespace map4_pcl_extensions
{
PointType detectType(const std::string& file_name)
{
  pcl::PCDReader reader;
  pcl::PCLPointCloud2 cloud;
  reader.readHeader(file_name, cloud);

  std::vector<std::string> field_vec;
  int type_num = 0;
  for (int i = 0; i < cloud.fields.size(); i++)
  {
    type_num += checkFieldName(cloud.fields[i].name);
  }

  return castPointType(type_num);
}

PointType detectType(const std::vector<std::string>& fields)
{
  int type_num = 0;
  for (int i = 0; i < fields.size(); i++)
  {
    type_num += checkFieldName(fields[i]);
  }
  return castPointType(type_num);
}

int checkFieldName(const std::string& field_name)
{
  if (field_name == "x" || field_name == "y" || field_name == "z")
  {
    return 0;
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
  else
  {
    return 0;
  }
}
}  // namespace map4_pcl_extensions
