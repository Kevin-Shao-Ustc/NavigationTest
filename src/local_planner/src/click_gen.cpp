#include "local_planner/visualizer.hpp"
#include "local_planner/trajectory.hpp"

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include <cmath>
#include <iostream>
#include <vector>

struct Config
{
    std::string targetTopic;
    double clickHeight;
    std::vector<double> initialVel;
    std::vector<double> initialAcc;
    std::vector<double> terminalVel;
    std::vector<double> terminalAcc;
    double allocationSpeed;
    double allocationAcc;
    int maxPieceNum;

    Config(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("TargetTopic", targetTopic);
        nh_priv.getParam("ClickHeight", clickHeight);
        nh_priv.getParam("InitialVel", initialVel);
        nh_priv.getParam("InitialAcc", initialAcc);
        nh_priv.getParam("TerminalVel", terminalVel);
        nh_priv.getParam("TerminalAcc", terminalAcc);
        nh_priv.getParam("AllocationSpeed", allocationSpeed);
        nh_priv.getParam("AllocationAcc", allocationAcc);
        nh_priv.getParam("MaxPieceNum", maxPieceNum);
    }
};

double timeTrapzVel(const double dist,
                    const double vel,
                    const double acc)
{
    const double t = vel / acc;
    const double d = 0.5 * acc * t * t;

    if (dist < d + d)
    {
        return 2.0 * sqrt(dist / acc);
    }
    else
    {
        return 2.0 * t + (dist - 2.0 * d) / vel;
    }
}

void minimumJerkTrajGen(
    // Inputs:
    const int pieceNum,
    const Eigen::Vector3d &initialPos,
    const Eigen::Vector3d &initialVel,
    const Eigen::Vector3d &initialAcc,
    const Eigen::Vector3d &terminalPos,
    const Eigen::Vector3d &terminalVel,
    const Eigen::Vector3d &terminalAcc,
    const Eigen::Matrix3Xd &intermediatePositions,
    const Eigen::VectorXd &timeAllocationVector,
    // Outputs:
    Eigen::MatrixX3d &coefficientMatrix)
{
    // coefficientMatrix is a matrix with 6*piece num rows and 3 columes
    // As for a polynomial c0+c1*t+c2*t^2+c3*t^3+c4*t^4+c5*t^5,
    // each 6*3 sub-block of coefficientMatrix is
    // --              --
    // | c0_x c0_y c0_z |
    // | c1_x c1_y c1_z |
    // | c2_x c2_y c2_z |
    // | c3_x c3_y c3_z |
    // | c4_x c4_y c4_z |
    // | c5_x c5_y c5_z |
    // --              --
    // Please computed coefficientMatrix of the minimum-jerk trajectory
    // in this function

    // ------------------------ Put your solution below ------------------------
    //  print to indicate the start of the function
    //  for the position constraints

    //  - A matrix
    Eigen::MatrixXd A_pos = Eigen::MatrixXd::Zero(2 * pieceNum, 6 * pieceNum);

    for (int i = 0; i < pieceNum; i++)
    {
        A_pos(2 * i, 6 * i) = 1;
        A_pos(2 * i + 1, 6 * i) = 1;
        A_pos(2 * i + 1, 6 * i + 1) = timeAllocationVector(i);
        A_pos(2 * i + 1, 6 * i + 2) = pow(timeAllocationVector(i), 2);
        A_pos(2 * i + 1, 6 * i + 3) = pow(timeAllocationVector(i), 3);
        A_pos(2 * i + 1, 6 * i + 4) = pow(timeAllocationVector(i), 4);
        A_pos(2 * i + 1, 6 * i + 5) = pow(timeAllocationVector(i), 5);
    }

    //  - b matrix
    Eigen::MatrixXd b_pos = Eigen::MatrixXd::Zero(2 * pieceNum, 3);

    b_pos.row(0).noalias() = initialPos.transpose();
    b_pos.row(2 * pieceNum - 1).noalias() = terminalPos.transpose();
    for (int i = 0; i < pieceNum - 1; i++)
    {
        b_pos.row(2 * i + 1).noalias() = intermediatePositions.col(i).transpose();
        b_pos.row(2 * i + 2).noalias() = intermediatePositions.col(i).transpose();
    }
    //  for the velocity constraints

    //  - A matrix
    Eigen::MatrixXd A_vel = Eigen::MatrixXd::Zero(pieceNum + 1, 6 * pieceNum);

    A_vel(0, 1) = 1;

    A_vel(pieceNum, 6 * pieceNum - 5) = 1;
    A_vel(pieceNum, 6 * pieceNum - 4) = 2 * timeAllocationVector(pieceNum - 1);
    A_vel(pieceNum, 6 * pieceNum - 3) = 3 * pow(timeAllocationVector(pieceNum - 1), 2);
    A_vel(pieceNum, 6 * pieceNum - 2) = 4 * pow(timeAllocationVector(pieceNum - 1), 3);
    A_vel(pieceNum, 6 * pieceNum - 1) = 5 * pow(timeAllocationVector(pieceNum - 1), 4);

    for (int i = 0; i < pieceNum - 1; i++)
    {
        A_vel(i + 1, 6 * i + 1) = 1;
        A_vel(i + 1, 6 * i + 2) = 2 * timeAllocationVector(i);
        A_vel(i + 1, 6 * i + 3) = 3 * pow(timeAllocationVector(i), 2);
        A_vel(i + 1, 6 * i + 4) = 4 * pow(timeAllocationVector(i), 3);
        A_vel(i + 1, 6 * i + 5) = 5 * pow(timeAllocationVector(i), 4);
        A_vel(i + 1, 6 * i + 7) = -1;
    }

    // - b matrix
    Eigen::MatrixXd b_vel = Eigen::MatrixXd::Zero(pieceNum + 1, 3);

    b_vel.row(0).noalias() = initialVel.transpose();
    b_vel.row(pieceNum).noalias() = terminalVel.transpose();

    //  for the acceleration constraints

    //  - A matrix
    Eigen::MatrixXd A_acc = Eigen::MatrixXd::Zero(pieceNum + 1, 6 * pieceNum);

    A_acc(0, 2) = 2;

    A_acc(pieceNum, 6 * pieceNum - 4) = 2;
    A_acc(pieceNum, 6 * pieceNum - 3) = 6 * timeAllocationVector(pieceNum - 1);
    A_acc(pieceNum, 6 * pieceNum - 2) = 12 * pow(timeAllocationVector(pieceNum - 1), 2);
    A_acc(pieceNum, 6 * pieceNum - 1) = 20 * pow(timeAllocationVector(pieceNum - 1), 3);

    for (int i = 0; i < pieceNum - 1; i++)
    {
        A_acc(i + 1, 6 * i + 2) = 2;
        A_acc(i + 1, 6 * i + 3) = 6 * timeAllocationVector(i);
        A_acc(i + 1, 6 * i + 4) = 12 * pow(timeAllocationVector(i), 2);
        A_acc(i + 1, 6 * i + 5) = 20 * pow(timeAllocationVector(i), 3);
        A_acc(i + 1, 6 * i + 8) = -2;
    }

    // - b matrix
    Eigen::MatrixXd b_acc = Eigen::MatrixXd::Zero(pieceNum + 1, 3);

    b_acc.row(0).noalias() = initialAcc.transpose();
    b_acc.row(pieceNum).noalias() = terminalAcc.transpose();
    
    //  quadratic programming
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6 * pieceNum, 6 * pieceNum);

    for (int i = 0; i < pieceNum; i++)
    {
        Q(6 * i + 3, 6 * i + 3) = 36 * timeAllocationVector(i) * 2;
        Q(6 * i + 4, 6 * i + 3) = 72 * pow(timeAllocationVector(i), 2) * 2;
        Q(6 * i + 3, 6 * i + 4) = 72 * pow(timeAllocationVector(i), 2) * 2;
        Q(6 * i + 4, 6 * i + 4) = 192 * pow(timeAllocationVector(i), 3) * 2;
        Q(6 * i + 5, 6 * i + 3) = 120 * pow(timeAllocationVector(i), 3) * 2;
        Q(6 * i + 3, 6 * i + 5) = 120 * pow(timeAllocationVector(i), 3) * 2;
        Q(6 * i + 4, 6 * i + 5) = 380 * pow(timeAllocationVector(i), 4) * 2;
        Q(6 * i + 5, 6 * i + 4) = 380 * pow(timeAllocationVector(i), 4) * 2;
        Q(6 * i + 5, 6 * i + 5) = 720 * pow(timeAllocationVector(i), 5) * 2;
    }


    //  - problem description
    //  -- objective function
    //  --- 0.5 * x^T * Q * x
    //  -- constraints
    //  --- A * x = b
    //  -- Lagrangian function
    //  --- L(x, lambda) = 0.5 * x^T * Q * x + lambda^T * (A * x - b)
    //  -- KKT conditions
    //  --- Q * x + A^T * lambda = 0
    //  --- A * x = b
    //  -- formulate into single matrix equation
    //  --- P * [x; lambda] = q
    //  --- where P = [Q A^T; A 0], q = [0; b]
    //  -- solve for [x; lambda] by PLU decomposition
    //  --- [x; lambda] = P^-1 * q

    //  -- A and b matrix for the constraints
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(4 * pieceNum + 2, 6 * pieceNum);
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(4 * pieceNum + 2, 3);

    A.block(0, 0, 2 * pieceNum, 6 * pieceNum) = A_pos;
    A.block(2 * pieceNum, 0, pieceNum + 1, 6 * pieceNum) = A_vel;
    A.block(3 * pieceNum + 1, 0, pieceNum + 1, 6 * pieceNum) = A_acc;

    b.block(0, 0, 2 * pieceNum, 3) = b_pos;
    b.block(2 * pieceNum, 0, pieceNum + 1, 3) = b_vel;
    b.block(3 * pieceNum + 1, 0, pieceNum + 1, 3) = b_acc;

    //  -- P and q matrix for the KKT conditions
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(10 * pieceNum + 2, 10 * pieceNum + 2);
    Eigen::MatrixXd q = Eigen::MatrixXd::Zero(10 * pieceNum + 2, 3);

    P.block(0, 0, 6 * pieceNum, 6 * pieceNum) = Q;
    P.block(0, 6 * pieceNum, 6 * pieceNum, 4 * pieceNum + 2) = A.transpose();
    P.block(6 * pieceNum, 0, 4 * pieceNum + 2, 6 * pieceNum) = A;
    q.block(6 * pieceNum, 0, 4 * pieceNum + 2, 3) = b;
    //  -- solve for [x; lambda] by partial PLU decomposition

    Eigen::PartialPivLU<Eigen::MatrixXd> lu(P);
    Eigen::MatrixXd x_lambda = lu.solve(q);
    
    coefficientMatrix = x_lambda.block(0, 0, 6 * pieceNum, 3);
    // ------------------------ Put your solution above ------------------------
}

class ClickGen
{
private:
    Config config;

    ros::NodeHandle nh;
    ros::Subscriber targetSub;

    Visualizer visualizer;

    Eigen::Matrix3Xd positions;
    Eigen::VectorXd times;
    int positionNum;
    Trajectory<5> traj;

public:
    ClickGen(const Config &conf,
             ros::NodeHandle &nh_)
        : config(conf),
          nh(nh_),
          visualizer(nh),
          positions(3, config.maxPieceNum + 1),
          times(config.maxPieceNum),
          positionNum(0)
    {
        targetSub = nh.subscribe(config.targetTopic, 1,
                                 &ClickGen::targetCallBack, this,
                                 ros::TransportHints().tcpNoDelay());
    }

    void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (positionNum > config.maxPieceNum)
        {
            positionNum = 0;
            traj.clear();
        }

        positions(0, positionNum) = msg->pose.position.x;
        positions(1, positionNum) = msg->pose.position.y;
        positions(2, positionNum) = std::fabs(msg->pose.orientation.z) * config.clickHeight;

        if (positionNum > 0)
        {
            const double dist = (positions.col(positionNum) - positions.col(positionNum - 1)).norm();
            times(positionNum - 1) = timeTrapzVel(dist, config.allocationSpeed, config.allocationAcc);
        }

        ++positionNum;

        if (positionNum > 1)
        {
            const int pieceNum = positionNum - 1;
            const Eigen::Vector3d initialPos = positions.col(0);
            const Eigen::Vector3d initialVel(config.initialVel[0], config.initialVel[1], config.initialVel[2]);
            const Eigen::Vector3d initialAcc(config.initialAcc[0], config.initialAcc[1], config.initialAcc[2]);
            const Eigen::Vector3d terminalPos = positions.col(pieceNum);
            const Eigen::Vector3d terminalVel(config.terminalVel[0], config.terminalVel[1], config.terminalVel[2]);
            const Eigen::Vector3d terminalAcc(config.terminalAcc[0], config.terminalAcc[1], config.terminalAcc[2]);
            const Eigen::Matrix3Xd intermediatePositions = positions.middleCols(1, pieceNum - 1);
            const Eigen::VectorXd timeAllocationVector = times.head(pieceNum);

            Eigen::MatrixX3d coefficientMatrix = Eigen::MatrixXd::Zero(6 * pieceNum, 3);

            minimumJerkTrajGen(pieceNum,
                               initialPos, initialVel, initialAcc,
                               terminalPos, terminalVel, terminalAcc,
                               intermediatePositions,
                               timeAllocationVector,
                               coefficientMatrix);

            traj.clear();
            traj.reserve(pieceNum);
            for (int i = 0; i < pieceNum; i++)
            {
                traj.emplace_back(timeAllocationVector(i),
                                  coefficientMatrix.block<6, 3>(6 * i, 0).transpose().rowwise().reverse());
            }
        }

        visualizer.visualize(traj, positions.leftCols(positionNum));

        return;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "click_gen_node");
    ros::NodeHandle nh_;
    ClickGen clickGen(Config(ros::NodeHandle("~")), nh_);
    ros::spin();
    return 0;
}
