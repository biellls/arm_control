
#include <iostream>

#include <boost/program_options.hpp>

#include "melfa/melfa.h"
#include "melfa/exceptions.h"
#include "melfa/joint_state.h"
#include "melfa/tool_pose.h"

namespace po = boost::program_options;

int main(int argc, char* argv[])
{
    // Declare the supported options.
    po::options_description desc("Options");
    desc.add_options()
        ("device,D", po::value<std::string>()->default_value("/dev/ttyUSB0"), "robot device name")
        ("j", "move joint mode, values in radiants [J1 J2 J3 J4 J5 J6]")
        ("J", "move joint mode, values in degrees [J1 J2 J3 J4 J5 J6]")
        ("t", "move tool mode, values in meters and radiants [X Y Z R P Y]")
        ("T", "move tool mode, values in millimeters and degrees [X Y Z R P Y]")
        ;
    po::options_description hidden("Hidden options");
    hidden.add_options()
        ("coordinates", po::value<std::vector<double> >(), "coordinates")
        ;

    po::positional_options_description p;
    p.add("coordinates", 6);

    po::options_description all_desc("All options");
    all_desc.add(desc).add(hidden);

    po::variables_map vm;
    try
    {
        po::store(po::command_line_parser(argc, argv).
                      options(all_desc).positional(p).run(), vm);
        po::notify(vm);    
    } catch (const po::error& error)
    {
        std::cerr << "Error parsing program options: " << std::endl;
        std::cerr << "  " << error.what() << std::endl;
        std::cerr << desc << std::endl;
        return -1;
    }

    if (vm.count("j") + vm.count("J") + vm.count("t") + vm.count("T") != 1)
    {
        std::cerr << "Please specify exactly one movement mode." << std::endl;
        return -1;
    }

    if (vm.count("coordinates") != 1)
    {
        std::cerr << "Please specify the coordinates." << std::endl;
        return -1;
    }

    std::string device_name = vm["device"].as<std::string>();
    std::vector<double> coordinates = vm["coordinates"].as<std::vector<double> >();
    if (coordinates.size() != 6)
    {
        std::cerr << "Invalid number of coordinates (" 
                  << coordinates.size() << "), must be 6." << std::endl;
        return -1;
    }


    melfa::Melfa::ConfigParams params;
    params.device = std::string(argv[1]);

    melfa::Melfa melfa(params);
    try
    {
        melfa.connect();
        std::cout << "Robot connected." << std::endl;
        if (vm.count("j"))
        {
            melfa::JointState joint_state;
            joint_state.j1 = coordinates[0];
            joint_state.j2 = coordinates[1];
            joint_state.j3 = coordinates[2];
            joint_state.j4 = coordinates[3];
            joint_state.j5 = coordinates[4];
            joint_state.j6 = coordinates[5];
            melfa.moveJoints(joint_state);
        }
        else if (vm.count("J"))
        {
            melfa::JointState joint_state;
            joint_state.j1 = coordinates[0] / 180.0 * M_PI;
            joint_state.j2 = coordinates[1] / 180.0 * M_PI;
            joint_state.j3 = coordinates[2] / 180.0 * M_PI;
            joint_state.j4 = coordinates[3] / 180.0 * M_PI;
            joint_state.j5 = coordinates[4] / 180.0 * M_PI;
            joint_state.j6 = coordinates[5] / 180.0 * M_PI;
            melfa.moveJoints(joint_state);
        }
        else if (vm.count("t"))
        {
            melfa::ToolPose tool_pose;
            tool_pose.x = coordinates[0];
            tool_pose.y = coordinates[1];
            tool_pose.z = coordinates[2];
            tool_pose.roll = coordinates[3];
            tool_pose.pitch = coordinates[4];
            tool_pose.yaw = coordinates[5];
            melfa.moveTool(tool_pose);
        }
        else if (vm.count("T"))
        {
            melfa::ToolPose tool_pose;
            tool_pose.x = coordinates[0] / 1000.0;
            tool_pose.y = coordinates[1] / 1000.0;
            tool_pose.z = coordinates[2] / 1000.0;
            tool_pose.roll = coordinates[3] / 180.0 * M_PI;
            tool_pose.pitch = coordinates[4] / 180.0 * M_PI;
            tool_pose.yaw = coordinates[5] / 180.0 * M_PI;
            melfa.moveTool(tool_pose);
         }
    }
    catch (melfa::SerialConnectionError& err)
    {
        std::cerr << "Serial Connection error: " << err.what() << std::endl;
    }
    catch (melfa::RobotError& err)
    {
        std::cerr << "Robot error: " << err.what() << std::endl;
    }

    return 0;
}

