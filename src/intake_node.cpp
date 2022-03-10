#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>
#include <map>
#include "ck_utilities/Motor.hpp"
#include "rio_control_node/Robot_Status.h"
#include "rio_control_node/Motor_Status.h"
#include "rio_control_node/Motor_Info.h"

#include "intake_node/Intake_Control.h"
#include "intake_node/Intake_Status.h"

#include "hmi_agent_node/HMI_Signals.h"
#include "ck_utilities/Solenoid.hpp"

#define FRONT_ROLLER_CAN_ID 7
#define BACK_ROLLER_CAN_ID 8
#define FRONT_BELT_CAN_ID 9
#define BACK_BELT_CAN_ID 10
#define UPTAKE_CAN_ID 11
#define FRONT_SOLENOID_ID 8
#define BACK_SOLENOID_ID 9

#define PIXY_SIGNAL_CAN_ID 12

ros::NodeHandle *node;

static ros::Subscriber hmi_subscriber;
static ros::Subscriber robot_state_subscriber;
static ros::Subscriber motor_status_subscriber;

static Motor *front_roller;
static Motor *back_roller;
static Motor *front_belt;
static Motor *back_belt;
static Motor *uptake;

static Solenoid *front_intake_solenoid;
static Solenoid *back_intake_solenoid;

static std::map<uint16_t, rio_control_node::Motor_Info> motor_status_map;

enum class Alliance
{
	RED,
	BLUE
};

enum class RobotState
{
	DISABLED,
	AUTONOMUS,
	TELEOP
};

static RobotState robot_state = RobotState::DISABLED;
static Alliance alliance = Alliance::RED;

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
static IntakeStates next_intake_state = IntakeStates::IDLE;

static bool intake_rollers = false;
static bool retract_intake = false;
static bool manual_intake = false;
static bool manual_outake = false;
static float drivetrain_fwd_back = 0;

static bool red_ball_present = false;
static bool blue_ball_present = false;
static bool uptake_complete = false;
static bool eject_complete = false;
static ros::Time time_roller_last_active = ros::Time(0);

void hmiSignalCallback(const hmi_agent_node::HMI_Signals &msg)
{
	intake_rollers = msg.intake_rollers;
	if (intake_rollers)
	{
		time_roller_last_active = ros::Time::now();
	}
	retract_intake = msg.retract_intake;
	manual_intake = msg.manual_intake;
	manual_outake = msg.manual_outake;
	drivetrain_fwd_back = msg.drivetrain_fwd_back;
}

bool command_shoot = false;
void intake_control_callback(const intake_node::Intake_Control &msg)
{
	(void)msg;
	command_shoot = msg.command_shoot;
}

static bool has_a_ball = false;
static bool uptake_reverse = false;
static bool uptake_normal = false;
static bool uptake_shoot = false;
void stateMachineStep()
{
	static ros::Time time_state_entered = ros::Time::now();
	if (intake_state != next_intake_state)
	{
		time_state_entered = ros::Time::now();
	}
	ros::Duration time_in_state = ros::Time::now() - time_state_entered;
	intake_state = next_intake_state;

	static ros::Publisher intakeStatusPublisher = node->advertise<intake_node::Intake_Status>("/IntakeStatus", 1);
	intake_node::Intake_Status statusMsg;
	if (command_shoot)
	{
		statusMsg.readyToShoot = true;
	}
	else
	{
		statusMsg.readyToShoot = false;
	}

	intakeStatusPublisher.publish(statusMsg);

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
		if (deployed_direction == DeployedDirection::FRONT)
		{
			front_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 1, 0);
			front_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 1, 0);
			back_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 1, 0);
			back_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 1, 0);
		}
		else
		{
			back_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 1, 0);
			back_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 1, 0);
			front_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 1, 0);
			front_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 1, 0);
		}
	}
	break;

	case IntakeStates::UPTAKE_BALL:
	{
		//Put Ball Into Uptake
		uptake_normal = true;
		front_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
		back_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
		front_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
		back_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
		uptake_complete = time_in_state > ros::Duration(0.5);
	}
	break;

	case IntakeStates::EJECT_BALL:
	{
		//Lob The Ball Out
		if (deployed_direction == DeployedDirection::FRONT)
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
		eject_complete = time_in_state > ros::Duration(0.5);
	}
	break;
	}
	switch (intake_state)
	{
	case IntakeStates::IDLE:
	{
		if (intake_rollers)
		{
			next_intake_state = IntakeStates::INTAKE_ROLLERS;
		}
		else
		{
			next_intake_state = IntakeStates::IDLE;
		}
	}
	break;

	case IntakeStates::INTAKE_ROLLERS:
	{
		if ((alliance == Alliance::RED && red_ball_present) || (alliance == Alliance::BLUE && blue_ball_present))
		{
			next_intake_state = IntakeStates::UPTAKE_BALL;
		}
		else if ((alliance == Alliance::RED && blue_ball_present) || (alliance == Alliance::BLUE && red_ball_present))
		{
			next_intake_state = IntakeStates::EJECT_BALL;
		}
		else if (intake_rollers || ros::Time::now() - time_roller_last_active < ros::Duration(1))
		{
			next_intake_state = IntakeStates::INTAKE_ROLLERS;
		}
		else
		{
			next_intake_state = IntakeStates::IDLE;
		}
	}
	break;

	case IntakeStates::UPTAKE_BALL:
	{
		if (uptake_complete)
		{
			next_intake_state = IntakeStates::INTAKE_ROLLERS;
			has_a_ball = true;
			uptake_complete = false;
		}
		else
		{
			next_intake_state = IntakeStates::UPTAKE_BALL;
		}
	}
	break;

	case IntakeStates::EJECT_BALL:
	{
		if (eject_complete)
		{
			next_intake_state = IntakeStates::INTAKE_ROLLERS;
			eject_complete = false;
		}
		else
		{
			next_intake_state = IntakeStates::EJECT_BALL;
		}
	}
	break;
	}
	static ros::Time time_command_started = ros::Time::now();
	if (command_shoot)
	{
		uptake_shoot = true;
		if ((ros::Time::now() - time_command_started) > ros::Duration(0.75))
		{
			has_a_ball = false;
		}
	}
	else
	{
		time_command_started = ros::Time::now();
	}

	if (uptake_reverse)
	{
		uptake->set(Motor::Control_Mode::PERCENT_OUTPUT, -0.5, 0);
	}
	else if (uptake_normal || uptake_shoot)
	{
		uptake->set(Motor::Control_Mode::PERCENT_OUTPUT, 0.5, 0);
	}
	else
	{
		uptake->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
	}
}

void determineDeployDirection()
{
	constexpr float accumulator_cap = 10;
	constexpr float threshold = 5;
	static float drivetrain_accumulator = 0;
	ROS_INFO("Accumulator %f", drivetrain_accumulator);
	//Figure out deployment direction
	if (fabs(drivetrain_fwd_back) > 0.1)
	{
		drivetrain_accumulator += drivetrain_fwd_back;
		if (drivetrain_accumulator > accumulator_cap)
		{
			drivetrain_accumulator = accumulator_cap;
		}
		if (drivetrain_accumulator < -accumulator_cap)
		{
			drivetrain_accumulator = -accumulator_cap;
		}
		if (drivetrain_accumulator > threshold)
		{
			deployed_direction = DeployedDirection::FRONT;
		}
		if (drivetrain_accumulator < -threshold)
		{
			deployed_direction = DeployedDirection::BACK;
		}
	}
}

void motorStatusCallback(const rio_control_node::Motor_Status &msg)
{
	for (const rio_control_node::Motor_Info &motorInfo : msg.motors)
	{
		motor_status_map[motorInfo.id] = motorInfo;
	}

	if (motor_status_map.count(PIXY_SIGNAL_CAN_ID))
	{
		red_ball_present = motor_status_map[PIXY_SIGNAL_CAN_ID].forward_limit_closed;
		blue_ball_present = motor_status_map[PIXY_SIGNAL_CAN_ID].reverse_limit_closed;
	}
	else
	{
		red_ball_present = false;
		blue_ball_present = false;
	}
}

void robotStatusCallback(const rio_control_node::Robot_Status &msg)
{

	static std::map<int8_t, RobotState> robotStateLookupMap =
		{
			{rio_control_node::Robot_Status::AUTONOMOUS, RobotState::AUTONOMUS},
			{rio_control_node::Robot_Status::DISABLED, RobotState::DISABLED},
			{rio_control_node::Robot_Status::TELEOP, RobotState::TELEOP}};

	static std::map<int8_t, Alliance> allianceLookupMap =
		{
			{rio_control_node::Robot_Status::RED, Alliance::RED},
			{rio_control_node::Robot_Status::BLUE, Alliance::BLUE}};

	robot_state = robotStateLookupMap[msg.robot_state];
	alliance = allianceLookupMap[msg.alliance];
}

void motorConfiguration(void)
{
	front_roller = new Motor(FRONT_ROLLER_CAN_ID, Motor::Motor_Type::TALON_FX);
	front_roller->config().set_supply_current_limit(true, 10, 0, 0);
	front_roller->config().apply();

	back_roller = new Motor(BACK_ROLLER_CAN_ID, Motor::Motor_Type::TALON_FX);
	back_roller->config().set_supply_current_limit(true, 10, 0, 0);
	back_roller->config().apply();

	front_belt = new Motor(FRONT_BELT_CAN_ID, Motor::Motor_Type::TALON_FX);
	front_belt->config().set_supply_current_limit(true, 10, 0, 0);
	front_belt->config().set_inverted(true);
	front_belt->config().apply();

	back_belt = new Motor(BACK_BELT_CAN_ID, Motor::Motor_Type::TALON_FX);
	back_belt->config().set_supply_current_limit(true, 10, 0, 0);
	back_belt->config().apply();

	uptake = new Motor(UPTAKE_CAN_ID, Motor::Motor_Type::TALON_FX);
	uptake->config().set_supply_current_limit(true, 10, 0, 0);
	uptake->config().set_forward_limit_switch(MotorConfig::LimitSwitchSource::Deactivated, MotorConfig::LimitSwitchNormal::Disabled);
	uptake->config().set_reverse_limit_switch(MotorConfig::LimitSwitchSource::Deactivated, MotorConfig::LimitSwitchNormal::Disabled);
	uptake->config().apply();

	front_intake_solenoid = new Solenoid(FRONT_SOLENOID_ID, Solenoid::SolenoidType::SINGLE);
	back_intake_solenoid = new Solenoid(BACK_SOLENOID_ID, Solenoid::SolenoidType::SINGLE);
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

	robot_state_subscriber = node->subscribe("/RobotStatus", 1, robotStatusCallback);
	hmi_subscriber = node->subscribe("/HMISignals", 1, hmiSignalCallback);
	motor_status_subscriber = node->subscribe("/MotorStatus", 1, motorStatusCallback);

	motorConfiguration();

	ros::Rate rate(100);

	while (ros::ok())
	{
		ros::spinOnce();

		determineDeployDirection();
		if (retract_intake || manual_intake || manual_outake)
		{
			if (retract_intake)
			{
				front_intake_solenoid->set(Solenoid::SolenoidState::OFF);
				back_intake_solenoid->set(Solenoid::SolenoidState::OFF);
			}
			if (manual_intake)
			{
				front_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 1.0, 0);
				front_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 1.0, 0);
				back_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 1.0, 0);
				back_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 1.0, 0);
				uptake->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
			}
			if (manual_outake)
			{
				front_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, -1.0, 0);
				front_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, -1.0, 0);
				back_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, -1.0, 0);
				back_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, -1.0, 0);
				uptake->set(Motor::Control_Mode::PERCENT_OUTPUT, -1.0, 0);
			}
		}
		else
		{
			stateMachineStep();
			if (deployed_direction == DeployedDirection::FRONT)
			{
				ROS_INFO("Deploy direction front");
				front_intake_solenoid->set(Solenoid::SolenoidState::ON);
				back_intake_solenoid->set(Solenoid::SolenoidState::ON);
			}
			if (deployed_direction == DeployedDirection::BACK)
			{
				ROS_INFO("Deploy direction back");
				front_intake_solenoid->set(Solenoid::SolenoidState::ON);
				back_intake_solenoid->set(Solenoid::SolenoidState::ON);
			}
		}
		rate.sleep();
	}

	return 0;
}