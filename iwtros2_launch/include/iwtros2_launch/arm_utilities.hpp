#ifndef ARM_UTILITIES_HPP
#define ARM_UTILITIES_HPP

#include <iostream>
#include <string>

namespace iwtros2
{

/**
 * @brief Robot Basic Parameters
 * 
 */
class robot_config
{
  public:
    std::string ARM_GROUP_NAME;
    std::string ARM_REFERENCE_FRAME;
    std::string ARM_END_EFFECTOR;
    std::string PIPELINE_ID;
    std::string PTP_PLANNER_ID;
    std::string LIN_PLANNER_ID;

    robot_config()
    {
        ARM_GROUP_NAME = "iiwa_arm";
        ARM_REFERENCE_FRAME = "iiwa7_link_0";
        ARM_END_EFFECTOR = "iiwa7_link_7";
        PIPELINE_ID = "pilz";
        PTP_PLANNER_ID = "PTP";
        LIN_PLANNER_ID = "LIN";
    }
};

} // namespace iwtros2

#endif // ARM_UTILITIES_HPP
