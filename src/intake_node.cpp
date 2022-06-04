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
#include "ck_utilities/NTHelper.hpp"
#include <climber_node/Climber_Status.h>

// #define REAR_INTAKE_ENABLED

#define FRONT_ROLLER_CAN_ID 7
#define FRONT_BELT_CAN_ID 9
#define UPTAKE_CAN_ID 11
#define FRONT_SOLENOID_ID 3

#define LOWER_HARDSTOP_SOLENOID_ID 0
#define UPPER_HARDSTOP_SOLENOID_ID 5


#define PIXY_SIGNAL_CAN_ID 12

#define UPTAKE_POWER_FORWARD 1
#define UPTAKE_POWER_REVERSE -1
#define UPTAKE_LOADING_DISTANCE 4.5
#define UPTAKE_DURATION_S 0.02
#define UPTAKE_SHOOT_DURATION_S 0.9
#define UPTAKE_SENSOR_CAN_ID 18
#define EJECT_PISTON_OFFSET_TIME 0.100
#define EJECT_TIME 0.3 //changed from 0.5
#define INTAKE_TIME 0.5
#define UPTAKE_DURATION 0.8



ros::NodeHandle *node;

static ros::Subscriber hmi_subscriber;
static ros::Subscriber robot_state_subscriber;
static ros::Subscriber motor_status_subscriber;
static ros::Subscriber intake_control_subscriber;
static ros::Subscriber climber_status_subscriber;

static Motor *front_roller;
static Motor *front_belt;
static Motor *uptake;

static Solenoid *front_intake_solenoid;

static Solenoid *lower_hardstop_solenoid;
static Solenoid *upper_hardstop_solenoid;


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
static bool manual_outake_back = false;
static bool manual_outake_front = false;
static float drivetrain_fwd_back = 0;
static double uptake_position = 0;
static bool uptake_sensor_has_ball = false;

static bool red_ball_present = false;
static bool blue_ball_present = false;
static bool hooks_deployed = false;
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
	manual_outake_back = msg.manual_outake_back;
	manual_outake_front = msg.manual_outake_front;
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
static std::atomic_bool has_a_second_ball {false};
static double uptake_target = 0;
static float uptake_command = 0;
void stateMachineStep()
{
	static ros::Time time_state_entered = ros::Time::now();
	static ros::Publisher intakeStatusPublisher = node->advertise<intake_node::Intake_Status>("/IntakeStatus", 1);

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
		// Turn Off Rollers
		front_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
		uptake_command = 0;
	}
	break;

	case IntakeStates::INTAKE_ROLLERS:
	{
		if(intake_rollers)
		{
			front_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 1, 0);
		}
		else
		{
			front_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
		}
		if(!has_a_second_ball)
		{
			front_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 1, 0);
		}
		else
		{
			front_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
		}
		uptake_command = 0;
	}
	break;

	case IntakeStates::UPTAKE_BALL:
	{
		// Put Ball Into Uptake
		uptake->set(Motor::Control_Mode::PERCENT_OUTPUT, 1.0, 0);
		front_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 1.0, 0);
		if(intake_rollers)
		{
			front_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 1, 0);
		}
		else
		{
			front_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
		}
	}
	break;

	case IntakeStates::EJECT_BALL:
	{
		// if(time_in_state > ros::Duration(EJECT_PISTON_OFFSET_TIME))
		// {
		// 	// Lob The Ball Out
		// 	front_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 1, 0);
		// }
		// // else
		// // {
		// // 	front_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
		// // }
		// else 
		if(intake_rollers)
		{
			front_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 1, 0);
		}
		else
		{
			front_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
		}
		uptake_command = 0;
	}
	break;

	case IntakeStates::SHOOTING_BALL:
	{
		uptake_command = 1.0;
		front_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, 0, 0);
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
		if(!has_a_ball)
		{
			begin_transition_time = ros::Time::now();
			has_a_ball = true;
		}

		if(ros::Time::now() > begin_transition_time + ros::Duration(UPTAKE_DURATION) || uptake_sensor_has_ball)
		{
			next_intake_state = IntakeStates::INTAKE_ROLLERS;
		}
		else
		{
			next_intake_state = IntakeStates::UPTAKE_BALL;
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
		uptake->set(Motor::Control_Mode::VELOCITY, uptake_command * 1520.0, 0);	
	}

	static ros::Time second_ball_time = ros::Time::now();
	static bool had_second_ball_last_time = false;

	if (intake_state != IntakeStates::UPTAKE_BALL && has_a_ball && ((alliance == Alliance::RED && red_ball_present) || (alliance == Alliance::BLUE && blue_ball_present)))
	{
		if(!had_second_ball_last_time)
		{
			second_ball_time = ros::Time::now();
		}
		had_second_ball_last_time = true;
	}
	else
	{
		had_second_ball_last_time = false;
	}

	if(had_second_ball_last_time && ros::Time::now() > second_ball_time + ros::Duration(0.1))
	{
		has_a_second_ball = true;
	}
	else
	{
		has_a_second_ball = false;
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
	if(motor_status_map.find(UPTAKE_SENSOR_CAN_ID) != motor_status_map.end())
	{
		uptake_sensor_has_ball = motor_status_map[UPTAKE_SENSOR_CAN_ID].forward_limit_closed;
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
	front_roller->config().set_inverted(true);
	front_roller->config().set_neutral_mode(MotorConfig::NeutralMode::COAST);
	front_roller->config().apply();

	front_belt = new Motor(FRONT_BELT_CAN_ID, Motor::Motor_Type::TALON_FX);
	front_belt->config().set_supply_current_limit(true, 20, 0, 0);
	front_belt->config().set_inverted(true);
	front_belt->config().set_neutral_mode(MotorConfig::NeutralMode::COAST);
	front_belt->config().apply();

	uptake = new Motor(UPTAKE_CAN_ID, Motor::Motor_Type::TALON_FX);
	uptake->config().set_inverted(true);
	uptake->config().set_supply_current_limit(true, 20, 0, 0);
	uptake->config().set_neutral_mode(MotorConfig::NeutralMode::BRAKE);
    uptake->config().set_kP(0.012);
    uptake->config().set_kI(0.0);
    uptake->config().set_kD(0.03);
    uptake->config().set_kF(0.047651);
    uptake->config().set_motion_cruise_velocity(16000);
    uptake->config().set_motion_acceleration(32000);
    uptake->config().set_motion_s_curve_strength(5);
	uptake->config().apply();

	front_intake_solenoid = new Solenoid(FRONT_SOLENOID_ID, Solenoid::SolenoidType::SINGLE);
	lower_hardstop_solenoid = new Solenoid(LOWER_HARDSTOP_SOLENOID_ID, Solenoid::SolenoidType::SINGLE);
	upper_hardstop_solenoid = new Solenoid(UPPER_HARDSTOP_SOLENOID_ID, Solenoid::SolenoidType::SINGLE);

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
	diagnostics.manual_outake_back = manual_outake_back;
	diagnostics.manual_outake_front = manual_outake_front;
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
		bool local_has_a_ball = has_a_ball;
		ck::nt::set("dashboard_data", "has_ball", local_has_a_ball);

		bool local_has_a_second_ball = has_a_second_ball;
		ck::nt::set("dashboard_data", "has_second_ball", local_has_a_second_ball);
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

		if (robot_state == RobotState::AUTONOMUS && last_robot_state == RobotState::DISABLED)
		{
			has_a_ball = false;
			has_a_second_ball = false;
			next_intake_state = IntakeStates::IDLE;
			next_intake_state = IntakeStates::IDLE;
		}

		if (robot_state == RobotState::TELEOP && last_robot_state == RobotState::DISABLED)
		{
			next_intake_state = IntakeStates::IDLE;
			next_intake_state = IntakeStates::IDLE;
		}


		last_robot_state = robot_state;

		// Decided at WNE to require retract intake to be held down to extend intakes
		// this should be cleaned up FIXME TBD MGT
		if (!retract_intake || manual_intake || manual_outake_back || manual_outake_front)
		{
			if (manual_outake_front || manual_outake_back)
			{
				front_intake_solenoid->set(Solenoid::SolenoidState::ON);
			}
			else if ((!retract_intake && !hooks_deployed) || climber_retract_intake)
			{
				front_intake_solenoid->set(Solenoid::SolenoidState::OFF);
			}
			else if (hooks_deployed && !climber_retract_intake)
			{
				front_intake_solenoid->set(Solenoid::SolenoidState::OFF);
			}
			
			if (manual_outake_back)
			{
				has_a_ball = false;
				intake_state = IntakeStates::IDLE;
				next_intake_state = IntakeStates::IDLE;
				front_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, 1.0, 0);
				front_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, -1.0, 0);
				uptake->set(Motor::Control_Mode::PERCENT_OUTPUT, -0.6, 0);
			}
			else if (manual_outake_front)
			{
				has_a_ball = false;
				intake_state = IntakeStates::IDLE;
				next_intake_state = IntakeStates::IDLE;
				front_belt->set(Motor::Control_Mode::PERCENT_OUTPUT, -1.0, 0);
				front_roller->set(Motor::Control_Mode::PERCENT_OUTPUT, -1.0, 0);
				uptake->set(Motor::Control_Mode::PERCENT_OUTPUT, -0.6, 0);
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

		if (intake_state == IntakeStates::EJECT_BALL || manual_outake_back)
		{
			lower_hardstop_solenoid->set(Solenoid::SolenoidState::ON);
		}
		else
		{
			lower_hardstop_solenoid->set(Solenoid::SolenoidState::OFF);
		}

		if (has_a_second_ball || intake_state == IntakeStates::SHOOTING_BALL || manual_outake_back || manual_outake_front)
		{
			upper_hardstop_solenoid->set(Solenoid::SolenoidState::ON);
		}
		else
		{
			upper_hardstop_solenoid->set(Solenoid::SolenoidState::OFF);
		}

		publish_diagnostic_data();
		rate.sleep();
	}

	nt_publish_thread.join();

	return 0;
}