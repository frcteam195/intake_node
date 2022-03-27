#include "ros/ros.h"
#include "std_msgs/String.h"

#include <atomic>
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
#include "intake_node/intake_diagnostics.h"

#include "hmi_agent_node/HMI_Signals.h"
#include "ck_utilities/Solenoid.hpp"
#include <network_tables_node/NTSetBool.h>
#include <climber_node/Climber_Status.h>

// #define REAR_INTAKE_ENABLED

#define FRONT_ROLLER_CAN_ID 7
#define FRONT_BELT_CAN_ID 9
#define BACK_BELT_CAN_ID 10
#define UPTAKE_CAN_ID 11
#define FRONT_SOLENOID_ID 4

#define PIXY_SIGNAL_CAN_ID 12

#define UPTAKE_POWER_FORWARD 1
#define UPTAKE_POWER_REVERSE -1
#define UPTAKE_LOADING_DISTANCE 4.5
#define UPTAKE_DURATION_S 0.02
#define UPTAKE_SHOOT_DURATION_S 0.5
#define EJECT_TIME 2
#define INTAKE_TIME 1



ros::NodeHandle *node;

static ros::Subscriber hmi_subscriber;
static ros::Subscriber robot_state_subscriber;
static ros::Subscriber motor_status_subscriber;
static ros::Subscriber intake_control_subscriber;
static ros::Subscriber climber_status_subscriber;

static Motor *front_roller;
static Motor *front_belt;
static Motor *back_belt;
static Motor *uptake;

static Solenoid *front_intake_solenoid;

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
	TELEOP,
	TEST
};

static RobotState robot_state = RobotState::DISABLED;
static Alliance alliance = Alliance::RED;

enum class IntakeStates
{
	IDLE,
	INTAKE_ROLLERS,
	UPTAKE_BALL,
	EJECT_BALL,
	SHOOTING_BALL
};

static IntakeStates intake_state = IntakeStates::IDLE;
static IntakeStates next_intake_state = IntakeStates::IDLE;

static bool intake_rollers = false;
static bool retract_intake = false;
static bool manual_intake = false;
static bool manual_outake = false;
static float drivetrain_fwd_back = 0;
static double uptake_position = 0;

static bool red_ball_present = false;
static bool blue_ball_present = false;
static bool hooks_deployed = false;
static ros::Time time_roller_last_active = ros::Time(0);

ros::ServiceClient nt_setbool_client;

ros::ServiceClient& getNTSetBoolSrv()
{
	if (!nt_setbool_client)
	{
		nt_setbool_client = node->serviceClient<network_tables_node::NTSetBool>("nt_setbool", true);
	}
	return nt_setbool_client;
}

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
	if (!hooks_deployed)
    {
        hooks_deployed = msg.deploy_hooks;
    }
}

static bool command_shoot = false;
void intake_control_callback(const intake_node::Intake_Control &msg)
{
	command_shoot = msg.command_shoot;
}

static bool climber_retract_intake = false;
void climber_status_callback(const climber_node::Climber_Status &msg)
{
	climber_retract_intake = msg.climber_retract_intake;
}


static std::atomic_bool has_a_ball {false};
static double uptake_target = 0;
static int uptake_command = 0;
void stateMachineStep()
{
	static ros::Time time_state_entered = ros::Time::now();
	static ros::Publisher intakeStatusPublisher = node->advertise<intake_node::Intake_Status>("/IntakeStatus", 1);
	static double uptake_at_start_of_state = 0;
	if(next_intake_state != intake_state)
	{
		uptake_at_start_of_state = uptake_position;
	}


	intake_node::Intake_Status statusMsg;
	if (command_shoot)
	{
		next_intake_state = IntakeStates::SHOOTING_BALL;
	}

	if (has_a_ball)
	{
		statusMsg.readyToShoot = true;
	}
	else
	{
		statusMsg.readyToShoot = false;
	}

	if (intake_state != next_intake_state)
	{
		time_state_entered = ros::Time::now();
	}
	ros::Duration time_in_state = ros::Time::now() - time_state_entered;
	intake_state = next_intake_state;

	// ROS_INFO("Intake State: %d", (int)intake_state);

	intakeStatusPublisher.publish(statusMsg);

	switch (intake_state)
	{
	case IntakeStates::IDLE:
	{
		// Turn Off Belts
		front_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
		back_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
		// Turn Off Rollers
		front_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
		uptake_command = 0;
	}
	break;

	case IntakeStates::INTAKE_ROLLERS:
	{
		front_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 1, 0);
		front_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 1, 0);
		back_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
		uptake_command = 0;
	}
	break;

	case IntakeStates::UPTAKE_BALL:
	{
		// Put Ball Into Uptake
		if (!has_a_ball)
		{
			uptake_target = uptake_at_start_of_state + UPTAKE_LOADING_DISTANCE;
			uptake->set(Motor::Control_Mode::MOTION_MAGIC, uptake_target, 0);
		}
		front_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
		back_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
		front_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
	}
	break;

	case IntakeStates::EJECT_BALL:
	{
		// Lob The Ball Out
		front_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 1, 0);
		front_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 1, 0);
		back_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, -1, 0);

		if ((alliance == Alliance::RED && red_ball_present) || (alliance == Alliance::BLUE && blue_ball_present))
		{
			uptake_command = 1;
		}
		else
		{
			uptake_command = 0;
		}
		
	}
	break;

	case IntakeStates::SHOOTING_BALL:
	{
		uptake_command = 1;
		break;
	}

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
		if (!has_a_ball && ((alliance == Alliance::RED && red_ball_present) || (alliance == Alliance::BLUE && blue_ball_present)))
		{
			next_intake_state = IntakeStates::UPTAKE_BALL;
		}
		else if ((alliance == Alliance::RED && blue_ball_present) || (alliance == Alliance::BLUE && red_ball_present))
		{
			next_intake_state = IntakeStates::EJECT_BALL;
		}
		else if (intake_rollers || ros::Time::now() - time_roller_last_active < ros::Duration(INTAKE_TIME))
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
		static ros::Time begin_transition_time = ros::Time::now();
		if (uptake_position >= uptake_target - 0.2 || has_a_ball)
		{
			
			if(!has_a_ball)
			{
				begin_transition_time = ros::Time::now();
			}
			has_a_ball = true;
		}
		else
		{
			next_intake_state = IntakeStates::UPTAKE_BALL;
		}
		if(has_a_ball && begin_transition_time < ros::Time::now() - ros::Duration(0.5))
		{
			next_intake_state = IntakeStates::INTAKE_ROLLERS;
		}
	}
	break;

	case IntakeStates::EJECT_BALL:
	{
		if (time_in_state > ros::Duration(EJECT_TIME))
		{
			next_intake_state = IntakeStates::INTAKE_ROLLERS;
		}
		else
		{
			next_intake_state = IntakeStates::EJECT_BALL;
		}
	}
	break;

	case IntakeStates::SHOOTING_BALL:
	{
		if (time_in_state > ros::Duration(UPTAKE_SHOOT_DURATION_S))
		{
			next_intake_state = IntakeStates::IDLE;
		}
		has_a_ball = false;
		break;
	}
	}

	if(intake_state != IntakeStates::UPTAKE_BALL)
	{
		if (uptake_command > 0)
		{
			uptake->set(Motor::Control_Mode::PERCENT_OUTPUT, UPTAKE_POWER_FORWARD, 0);
		}
		else if (uptake_command < 0)
		{
			uptake->set(Motor::Control_Mode::PERCENT_OUTPUT, UPTAKE_POWER_REVERSE, 0);
		}
		else
		{
			uptake->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
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
	if (motor_status_map.find(UPTAKE_CAN_ID) != motor_status_map.end())
	{
		uptake_position = motor_status_map[UPTAKE_CAN_ID].sensor_position;
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
	front_roller->config().set_supply_current_limit(true, 20, 0, 0);
	front_roller->config().set_neutral_mode(MotorConfig::NeutralMode::COAST);
	front_roller->config().apply();

	front_belt = new Motor(FRONT_BELT_CAN_ID, Motor::Motor_Type::TALON_FX);
	front_belt->config().set_supply_current_limit(true, 20, 0, 0);
	front_belt->config().set_inverted(true);
	front_belt->config().set_neutral_mode(MotorConfig::NeutralMode::COAST);
	front_belt->config().apply();

	back_belt = new Motor(BACK_BELT_CAN_ID, Motor::Motor_Type::TALON_FX);
	back_belt->config().set_supply_current_limit(true, 20, 0, 0);
	back_belt->config().set_neutral_mode(MotorConfig::NeutralMode::COAST);
	back_belt->config().apply();

	uptake = new Motor(UPTAKE_CAN_ID, Motor::Motor_Type::TALON_FX);
	uptake->config().set_inverted(true);
	uptake->config().set_supply_current_limit(true, 20, 0, 0);
	uptake->config().set_neutral_mode(MotorConfig::NeutralMode::BRAKE);
    uptake->config().set_kP(0.07);
    uptake->config().set_kI(0.0);
    uptake->config().set_kD(0.05);
    uptake->config().set_kF(0.047651);
    uptake->config().set_motion_cruise_velocity(16000);
    uptake->config().set_motion_acceleration(32000);
    uptake->config().set_motion_s_curve_strength(5);
	uptake->config().apply();

	front_intake_solenoid = new Solenoid(FRONT_SOLENOID_ID, Solenoid::SolenoidType::SINGLE);
}



std::string intake_state_to_string(IntakeStates state)
{
    switch (state)
    {
    case IntakeStates::IDLE:
    {
        return "IDLE";
        break;
    }
    case IntakeStates::INTAKE_ROLLERS:
    {
        return "INTAKE_ROLLERS";
        break;
    }
	case IntakeStates::UPTAKE_BALL:
    {
        return "UPTAKE_BALL";
        break;
    }
	case IntakeStates::EJECT_BALL:
    {
        return "EJECT_BALL";
        break;
    }
	case IntakeStates::SHOOTING_BALL:
	{
		return "SHOOTING_BALL";
		break;
	}
    }
    return "INVALID";
}

void publish_diagnostic_data()
{
	static ros::Publisher diagnostic_publisher = node->advertise<intake_node::intake_diagnostics>("/IntakeNodeDiagnostics", 1);
	intake_node::intake_diagnostics diagnostics;

	diagnostics.intake_state = intake_state_to_string(intake_state);
	diagnostics.next_intake_state = intake_state_to_string(next_intake_state);
	diagnostics.intake_rollers = intake_rollers;
	diagnostics.retract_intake = retract_intake;
	diagnostics.manual_intake = manual_intake;
	diagnostics.manual_outake = manual_outake;
	diagnostics.drivetrain_fwd_back = drivetrain_fwd_back;
	diagnostics.red_ball_present = red_ball_present;
	diagnostics.blue_ball_present = blue_ball_present;
	diagnostics.has_a_ball = has_a_ball;
	diagnostics.uptake_target = uptake_target;
	diagnostics.uptake_actual = uptake_position;
	diagnostic_publisher.publish(diagnostics);
}

void nt_publish()
{
	ros::Rate rate(10);
	while(ros::ok())
	{
		ros::ServiceClient& nt_setbool_localclient = getNTSetBoolSrv();
		if (nt_setbool_localclient)
		{
			bool setSuccess = true;
			network_tables_node::NTSetBool ntmsg;
			ntmsg.request.table_name = "dashboard_data";
			ntmsg.request.entry_name = "has_ball";
			ntmsg.request.value = has_a_ball;
			setSuccess &= nt_setbool_client.call(ntmsg);
			if (!setSuccess)
			{
				// ROS_WARN("Failed to set values for limelight: %s", ll.name.c_str());
			}

		}
		rate.sleep();
	}

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
	intake_control_subscriber = node->subscribe("/IntakeControl", 1, intake_control_callback);
	climber_status_subscriber = node->subscribe("/ClimberStatus", 1, climber_status_callback);

	std::thread nt_publish_thread(nt_publish);

	motorConfiguration();

	ros::Rate rate(100);

	while (ros::ok())
	{
		ros::spinOnce();

		static RobotState last_robot_state = RobotState::DISABLED;

		if ((robot_state == RobotState::AUTONOMUS || robot_state == RobotState::TELEOP) && last_robot_state == RobotState::DISABLED)
		{
			has_a_ball = false;
			next_intake_state = IntakeStates::IDLE;
			next_intake_state = IntakeStates::IDLE;
		}

		last_robot_state = robot_state;

		// Decided at WNE to require retract intake to be held down to extend intakes
		// this should be cleaned up FIXME TBD MGT
		if (!retract_intake || manual_intake || manual_outake)
		{
			if ((!retract_intake && !hooks_deployed) || climber_retract_intake)
			{
				front_intake_solenoid->set(Solenoid::SolenoidState::OFF);
			}
			else if (hooks_deployed && !climber_retract_intake)
			{
				front_intake_solenoid->set(Solenoid::SolenoidState::OFF);
			}
			if (manual_outake)
			{
				has_a_ball = false;
				intake_state = IntakeStates::IDLE;
				next_intake_state = IntakeStates::IDLE;
				front_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, -1.0, 0);
				front_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, -1.0, 0);
				back_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, -1.0, 0);
				uptake->set(Motor::Control_Mode::PERCENT_OUTPUT, -1.0, 0);
			}
			else
			{
				//TBD MGT FIXME TODO this has to be cleaned up after WNE
				stateMachineStep();
			}
		}
		else
		{
			stateMachineStep();
			front_intake_solenoid->set(Solenoid::SolenoidState::ON);
		}
		publish_diagnostic_data();
		rate.sleep();
	}

	nt_publish_thread.join();

	return 0;
}