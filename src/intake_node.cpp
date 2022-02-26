#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>
#include "ck_utilities/Motor.hpp"
#include "rio_control_node/Robot_Status.h"
#include "hmi_agent_node/HMI_Signals.h"

#define FRONT_ROLLER_CAN_ID 7
#define BACK_ROLLER_CAN_ID 8
#define FRONT_BELT_CAN_ID 9
#define BACK_BELT_CAN_ID 10
#define OUTAKE_CAN_ID 11

ros::NodeHandle* node;

static ros::Subscriber hmi_subscriber;
static ros::Subscriber robot_state_subscriber;

static Motor * front_roller;
static Motor * back_roller;
static Motor * front_belt;
static Motor * back_belt;
static Motor * uptake;


enum class RobotState
{
	DISABLED,
	AUTONOMUS,
	TELEOP
};

static RobotState robot_state = RobotState::DISABLED;

enum class IntakeStates
{
	IDLE,
	INTAKE_ROLLERS,
	UPTAKE_BALL,
	EJECT_BALL
};

enum class DeployedDirection
{
	FRONT,
	BACK
};

static DeployedDirection deployed_direction = DeployedDirection::FRONT;
static IntakeStates intake_state = IntakeStates::IDLE;

static bool intake_rollers;
static bool retract_intake;
static bool manual_intake;
static bool manual_outake;
static float drivetrain_fwd_back;

void HMISignalCallback(const hmi_agent_node::HMI_Signals& msg)
{
	intake_rollers = msg.intake_rollers;
	retract_intake = msg.retract_intake;
	manual_intake = msg.manual_intake;
	manual_outake = msg.manual_outake;
	drivetrain_fwd_back = msg.drivetrain_fwd_back;
} 

void stateMachineStep()
{
	switch (intake_state)
	{
		case IntakeStates::IDLE:
		{
			//Turn Off Belts
			front_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
			back_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
			//Turn Off Rollers
			front_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
			back_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
		}
		break;

		case IntakeStates::INTAKE_ROLLERS:
		{
			if(deployed_direction == DeployedDirection::FRONT)
			{
				front_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 0.5, 0);
				front_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 0.5, 0);
				back_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
				back_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
			}
			else
			{
				back_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 0.5, 0);
				back_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 0.5, 0);
				front_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
				front_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
			}
		}
		break;

		case IntakeStates::UPTAKE_BALL:
		{
			//Put Ball Into Uptake
			uptake->set(Motor::Control_Mode::PERCENT_OUTPUT, 0.5, 0);
			front_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
			back_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
			front_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
			back_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
		}
		break;

		case IntakeStates::EJECT_BALL:
		{
			//Lob The Ball Out
			if(deployed_direction == DeployedDirection::FRONT)
			{
				front_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 0.5, 0);
				front_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 0.5, 0);
				back_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, -0.5, 0);
				back_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, -0.5, 0);
			}
			else
			{
				back_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 0.5, 0);
				back_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 0.5, 0);
				front_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, -0.5, 0);
				front_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, -0.5, 0);
			}
		}
		break;
	}
}

void DetermineDeployDirection()
{
	constexpr float accumulator_cap = 50;
	constexpr float threshold = 20;
	//Figure out deployment direction
	if(fabs(drivetrain_fwd_back) > 0.1)
	{
		static float drivetrain_accumulator = 0;
		drivetrain_accumulator += drivetrain_fwd_back;
		if(drivetrain_accumulator > accumulator_cap)
		{
			drivetrain_accumulator = accumulator_cap;
		}
		if(drivetrain_accumulator < -accumulator_cap)
		{
			drivetrain_accumulator = -accumulator_cap;
		}
		if(drivetrain_accumulator > threshold)
		{
			deployed_direction = DeployedDirection::FRONT;
		}
		if(drivetrain_accumulator < -threshold)
		{
			deployed_direction = DeployedDirection::BACK;
		}
	}
}

void RobotStatusCallback(const rio_control_node::Robot_Status& msg)
{
	if(msg.robot_state == rio_control_node::Robot_Status::AUTONOMOUS)
	{
		robot_state = RobotState::AUTONOMUS;
	}

	if(msg.robot_state == rio_control_node::Robot_Status::DISABLED)
	{
		robot_state = RobotState::DISABLED;
	}

	if(msg.robot_state == rio_control_node::Robot_Status::TELEOP)
	{
		robot_state = RobotState::TELEOP;
	}
}

void MotorConfiguration(void)
{
	front_roller = new Motor(FRONT_ROLLER_CAN_ID, Motor::Motor_Type::TALON_FX);
	front_roller->config().set_stator_current_limit(true, 10, 10, 0);
	front_roller->config().set_supply_current_limit(true, 10, 10, 0);
	front_roller->config().apply();

	back_roller = new Motor(BACK_ROLLER_CAN_ID, Motor::Motor_Type::TALON_FX);
	back_roller->config().set_stator_current_limit(true, 10, 10, 0);
	back_roller->config().set_supply_current_limit(true, 10, 10, 0);
	back_roller->config().apply();

	front_belt = new Motor(FRONT_BELT_CAN_ID, Motor::Motor_Type::TALON_FX);
	front_belt->config().set_stator_current_limit(true, 10, 10, 0);
	front_belt->config().set_supply_current_limit(true, 10, 10, 0);
	front_belt->config().apply();
	
	back_belt = new Motor(BACK_BELT_CAN_ID, Motor::Motor_Type::TALON_FX);
	back_belt->config().set_stator_current_limit(true, 10, 10, 0);
	back_belt->config().set_supply_current_limit(true, 10, 10, 0);
	back_belt->config().apply();

	uptake = new Motor(OUTAKE_CAN_ID, Motor::Motor_Type::TALON_FX);
	uptake->config().set_stator_current_limit(true, 10, 10, 0);
	uptake->config().set_supply_current_limit(true, 10, 10, 0);
	uptake->config().apply();
}

int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "intake_node");

	ros::NodeHandle n;

	node = &n;

	robot_state_subscriber = node->subscribe("/RobotStatus", 1, RobotStatusCallback);
	hmi_subscriber = node->subscribe("/HMISignals", 1, HMISignalCallback);

	while(ros::ok())
	{
		ros::spinOnce();
		if(robot_state == RobotState::DISABLED)
		{

		}
		else
		{
			DetermineDeployDirection();
			if(retract_intake || manual_intake || manual_outake)
			{

			}
			else
			{
				stateMachineStep();
			}
		}
	}

	return 0;
}