//
// Created by androst on 06.06.19.
//

#include <iostream>
#include <chrono>
#include <thread>

#include "romocc/Robot.h"

using namespace romocc;

int main(int argc, char *argv[])
{
    auto robotIP = "192.168.153.131";
    if (argc >= 2) {
        robotIP = argv[1];
    }

    std::cout << "Robot IP: " << robotIP << std::endl;

    auto print_message = []()
    {
        std::cout << "State updated" << "\n";
    };

    auto ur3e = Robot::New();
    ur3e->configure(Manipulator(ManipulatorType::UR3e), robotIP, 30003);

    if(ur3e->connect())
    {
        ur3e->addUpdateSubscription(print_message);

        auto initialJointConfig = ur3e->getCurrentState()->getJointConfig();
        std::cout << initialJointConfig.transpose() << std::endl;

        double previousTime = 0;
        bool stop = false;
        std::thread processing_thread([&](){
            while(!stop){
                Vector6d jointConfig = ur3e->getCurrentState()->getJointConfig();
                double currentTime = ur3e->getCurrentState()->getTimestamp();
                Transform3d m_bM1 = ur3e->getCurrentState()->getTransformToJoint(1);
                Transform3d m_bM2 = ur3e->getCurrentState()->getTransformToJoint(2);
                Transform3d m_bM3 = ur3e->getCurrentState()->getTransformToJoint(3);
                Transform3d m_bM4 = ur3e->getCurrentState()->getTransformToJoint(4);
                Transform3d m_bM5 = ur3e->getCurrentState()->getTransformToJoint(5);
                Transform3d m_bM6 = ur3e->getCurrentState()->getTransformToJoint(6);

                if(currentTime > previousTime)
                {
                    std::cout << jointConfig.transpose() << std::endl;
                    previousTime = ur3e->getCurrentState()->getTimestamp();
                }
            }
        });

        double targetConfig[] = {190,-400, 150, 1.0,-3.0, 0};
        ur3e->move(romocc::MotionType::movep, targetConfig, 100, 50);
        std::this_thread::sleep_for(std::chrono::seconds(3));

        std::cout << ur3e->getCurrentState()->getJointConfig().transpose() << std::endl;
        ur3e->move(romocc::MotionType::movej, initialJointConfig, 50, 25);
        std::this_thread::sleep_for(std::chrono::seconds(3));

        std::cout << ur3e->getCurrentState()->getJointConfig().transpose() << std::endl;
        stop = true;
        processing_thread.join();
        ur3e->disconnect();
        std::this_thread::sleep_for(std::chrono::seconds(1));

        ur3e->connect();
        ur3e->addUpdateSubscription(print_message);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        ur3e->disconnect();
    }
}