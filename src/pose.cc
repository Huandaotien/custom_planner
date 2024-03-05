#include <custom_planner/pose.h>

namespace custom_planner
{

bool Pose::SavePoseAsFile(const string& file_name)
{
    bool result = false;
    ofstream file(file_name);
    if (file.is_open())
    {
        file << x_ << " " << y_ << " "<< yaw_ << "\n";
        file.close();
        result = true;
    }
    else
    {
        result = false;
    }
    return result;
}

bool Pose::LoadPoseFromFile(const string& file_name)
{
    bool result = false;
    ifstream file(file_name);
    if (file.is_open())
    {
        double x, y, yaw;
        file >> x >> y >> yaw;
        setPose(x, y, yaw);
        file.close();
        result = true;
    }
    else
    {
       result = false;
    }
    return result;
}

}