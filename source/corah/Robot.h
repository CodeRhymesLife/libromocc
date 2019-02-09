//
// Created by androst on 29.06.18.
//

#ifndef CORAH_ROBOT_H
#define CORAH_ROBOT_H

#include "corah/robotics/RobotState.h"
#include "corah/robotics/RobotMotion.h"
#include "corah/communication/CommunicationInterface.h"

namespace corah
{

class CORAH_EXPORT Robot : public QObject
{
    Q_OBJECT

    public:
        Robot();
        ~Robot();

        void configure(Manipulator manipulator, QString ip_address, int port);
        bool start();
        bool isConnected();
        bool disconnectFromRobot();
        void shutdown();

        RobotState getCurrentState();

        template <class Target>
        void move(MotionType type, Target target, double acc, double vel, double t=0, double rad=0);
        void stopMove(MotionType type, double acc);

        Transform3d get_rMt();
        Transform3d get_rMb();
        Transform3d get_eeMt();

        void set_eeMt(Eigen::Affine3d eeMt);
        void set_rMb(Eigen::Affine3d rMb);

        void runMotionQueue(MotionQueue queue);
        void stopRunMotionQueue();

    signals:
        void stateUpdated();

    private:
        void updateCurrentState(JointState state);
        void waitForMove();

        CommunicationInterface mCommunicationInterface;
        RobotState mCurrentState;
        MotionQueue mMotionQueue;

        Transform3d eeMt, rMb;

        Vector6d calculateJointVelocity(RobotMotion target);
};

template <class Target>
void Robot::move(MotionType type, Target target, double acc, double vel, double t, double rad)
{
    mCommunicationInterface.move(type, target, acc, vel, t, rad);
};

}

#endif //CORAH_ROBOT_H