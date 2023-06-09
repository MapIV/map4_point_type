#include <iostream>
#include <map4_point_type/map4_point_type.h>

int main()
{
    PointXYZIR test_p;

    fprintf(stderr, "[%s, %d] %s::type of test_p = %s\n",
            __FILE__, __LINE__, __func__, typeid(test_p).name());

    return 0;
}