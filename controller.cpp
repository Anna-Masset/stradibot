/**
 * @file controller.cpp
 * @brief Controller with simple state machine (BOWING + placeholder MOVE_TO_STRING)
 */

#include <SaiModel.h>
#include "SaiPrimitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

using namespace std;
using namespace Eigen;
using namespace SaiPrimitives;

#include <signal.h>
bool runloop = false;
void sighandler(int) { runloop = false; }

#include "redis_keys.h"

// ============================
// STATE MACHINE
// ============================

enum class ControllerState
{
	CALIBRATION,
	MOVE_TO_STRING,
	BOWING
};

// ============================
// FUNCTION: CALIBRATION
// ============================

void calibration(Vector3d &ee_pos_desired,
				 Vector3d &ee_force_desired,
				 Matrix3d &ee_ori_desired,
				 Vector3d &ee_moment_desired,
				 int &key_pressed,
				 const shared_ptr<MotionForceTask> &general_task,
				 SaiCommon::RedisClient &redis_client,
				 vector<Vector3d> &string_positions,
				 vector<Matrix3d> &string_orientations,
				 const Matrix3d &initial_orientation,
				 const Vector3d &initial_position)
{
	general_task->parametrizeForceMotionSpaces(3);
	general_task->parametrizeMomentRotMotionSpaces(3);

	ee_pos_desired = initial_position;
	ee_ori_desired = initial_orientation;
	ee_force_desired.setZero();
	ee_moment_desired.setZero();

	static int prev_key_pressed = 0;

	int current_key = (int)redis_client.getDouble(KEYBOARD_INPUT_KEY);
	bool key_once = (current_key != 0 && prev_key_pressed == 0);

	prev_key_pressed = current_key;

	if (key_once)
	{

		key_pressed = redis_client.getDouble(KEYBOARD_INPUT_KEY);
		string_positions[key_pressed - 1] = general_task->getCurrentPosition();
		string_orientations[key_pressed - 1] = general_task->getCurrentOrientation();
		cout << "string " << key_pressed << " position: " << string_positions[key_pressed - 1].transpose() << "\n";
		cout << "string " << key_pressed << " orientation: \n"
			 << string_orientations[key_pressed - 1] << "\n";
	}
}

// ============================
// FUNCTION: MOVE TO STRING
// ============================

bool moveToString(Vector3d &ee_pos_desired,
				  Vector3d &ee_force_desired,
				  const Vector3d &string_position)
{
	ee_pos_desired = string_position;
	ee_force_desired.setZero();

	// return true when motion is complete
	return false;
}

// ============================
// FUNCTION: BOWING MOTION
// ============================

void bowingMotion(Vector3d &ee_pos_desired,
				  Matrix3d &ee_ori_desired,
				  Vector3d &ee_force_desired,
				  const shared_ptr<MotionForceTask> &general_task,
				  const Vector3d &string_position,
				  const Matrix3d &string_orientation_world,
				  const Vector3d &bowing_dir,
				  double time)
{
	general_task->parametrizeForceMotionSpaces(1, Vector3d::UnitY()); // switch to force control
	general_task->parametrizeMomentRotMotionSpaces(0);				  // no moment control

	ee_pos_desired = string_position + 0.60 / 2 * sin(2 * M_PI * 0.1 * time) * bowing_dir;
	ee_force_desired << 0.0, -1.0, 0.0;
	ee_ori_desired = string_orientation_world;
}

// ============================
// MAIN
// ============================

int main()
{
	// Robot model
	static const string robot_file =
		string(STRADIBOT_FOLDER) + "/urdf_models/flexiv_violin/flexiv.urdf";
	const string robot_name = "flexiv";

	// Redis
	auto redis_client = SaiCommon::RedisClient();
	redis_client.connect();

	// Signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// Robot model
	auto robot = std::make_shared<SaiModel::SaiModel>(robot_file, false);
	robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
	robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
	robot->updateModel();

	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// ============================
	// TASK SETUP
	// ============================

	const string control_link = "bow";
	const Vector3d control_point(0.0, 0.05, 0.0);

	Affine3d compliant_frame = Affine3d::Identity();
	compliant_frame.translation() = control_point;

	auto general_task = std::make_shared<MotionForceTask>(
		robot, control_link, compliant_frame, "general_task", true);

	general_task->disableInternalOtg(); // not necessary when there is contact

	general_task->parametrizeForceMotionSpaces(1, Vector3d::UnitY());
	general_task->parametrizeMomentRotMotionSpaces(0);

	general_task->setPosControlGains(400, 40, 0);
	general_task->setOriControlGains(400, 40, 0);
	general_task->setForceControlGains(0.7, 10.0, 1.3);

	// Joint task
	auto joint_task = std::make_shared<JointTask>(robot);
	joint_task->setGains(400, 40, 0);

	VectorXd q_desired = robot->q();
	joint_task->setGoalPosition(q_desired);

	// ============================
	// INITIALIZATION & DATABASE
	// ============================
	const Vector3d initial_position = general_task->getCurrentPosition();
	const Matrix3d initial_orientation = general_task->getCurrentOrientation();
	int key_pressed = 0;

	vector<Vector3d> string_positions(4);
	vector<Matrix3d> string_orientations(4);

	// ============================
	// INPUTS (orientation/position will come from calibration, number of string from user input)
	// ============================

	Vector3d string_position(0.9215, 0.0, 0.0);

	Matrix3d string_orientation_world = Matrix3d::Identity();
	// string_orientation_world.col(0) << 0, -cos(11 * M_PI / 180), sin(11 * M_PI / 180);
	// string_orientation_world.col(1) << 0, sin(11 * M_PI / 180), cos(11 * M_PI / 180);
	// string_orientation_world.col(2) << -1, 0, 0;
	Vector3d bowing_dir = string_orientation_world.col(2);

	// ============================
	// CONTROL VARIABLES
	// ============================

	Vector3d ee_pos_desired;
	Vector3d ee_force_desired;
	Matrix3d ee_ori_desired = string_orientation_world;
	Vector3d ee_moment_desired;

	Vector3d control_position = general_task->getCurrentPosition();

	// ============================
	// STATE MACHINE INIT
	// ============================

	ControllerState state = ControllerState::CALIBRATION; // start directly in bowing
	double state_start_time = 0.0;

	// ============================
	// LOOP
	// ============================

	runloop = true;
	double control_freq = 1000;
	SaiCommon::LoopTimer timer(control_freq, 1e6);

	cout << "Calibration: Press 1, 2, 3, or 4 to save the position and orientation of the corresponding string. Press 9 when done with calibration." << endl;
	while (runloop)
	{
		timer.waitForNextLoop();

		double time = timer.elapsedSimTime();
		double time_in_state = time - state_start_time;

		// Update robot
		robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
		robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
		robot->updateModel();

		// ============================
		// STATE MACHINE
		// ============================

		switch (state)
		{
		case ControllerState::CALIBRATION:
		{
			calibration(ee_pos_desired,
						ee_force_desired,
						ee_ori_desired,
						ee_moment_desired,
						key_pressed,
						general_task,
						redis_client,
						string_positions,
						string_orientations,
						initial_orientation,
						initial_position);

			if (key_pressed == 9) // if 9 key is pressed, move to next state
			{
				cout << "9 key pressed, ending calibration" << endl;
				cout << "String positions and orientations: " << endl;
				for (int i = 0; i < 4; i++)
				{
					cout << "String " << i + 1 << ": position " << string_positions[i].transpose() << ", orientation:\n"
						 << string_orientations[i] << "\n";
				}
				state = ControllerState::BOWING;
				state_start_time = time;
			}
			break;
		}
		case ControllerState::MOVE_TO_STRING:
		{
			bool reached = moveToString(ee_pos_desired,
										ee_force_desired,
										string_position);

			if (reached)
			{
				state = ControllerState::BOWING;
				state_start_time = time;
			}
			break;
		}

		case ControllerState::BOWING:
		{
			static int prev_key_pressed = 0;

			int current_key = (int)redis_client.getDouble(KEYBOARD_INPUT_KEY);
			bool key_once = (current_key != 0 && prev_key_pressed == 0);

			prev_key_pressed = current_key;

			if (key_once)
			{

				key_pressed = redis_client.getDouble(KEYBOARD_INPUT_KEY);
				if (key_pressed == 9)
				{
					string_position = string_positions[0]; // default to string 1 if 9 is pressed during bowing
					string_orientation_world = string_orientations[0];
				}
				else
				{
					string_position = string_positions[key_pressed - 1];
					string_orientation_world = string_orientations[key_pressed - 1];
				}
				bowing_dir = string_orientation_world.col(2);
			}

			// cout << "bowing dir: " << bowing_dir.transpose() << "\n";

			bowingMotion(ee_pos_desired,
						 ee_ori_desired,
						 ee_force_desired,
						 general_task,
						 string_position,
						 string_orientation_world,
						 bowing_dir,
						 time_in_state);
			break;
		}
		}

		// ============================
		// APPLY TASKS
		// ============================

		general_task->setGoalPosition(ee_pos_desired);
		general_task->setGoalOrientation(ee_ori_desired);
		general_task->setGoalForce(ee_force_desired);
		general_task->setGoalMoment(ee_moment_desired);

		control_position = general_task->getCurrentPosition();

		N_prec.setIdentity();
		general_task->updateTaskModel(N_prec);
		joint_task->updateTaskModel(general_task->getTaskAndPreviousNullspace());

		command_torques =
			general_task->computeTorques() +
			joint_task->computeTorques();

		// Redis write
		redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		redis_client.setEigen(CONTROL_POSITION_KEY, control_position);
	}

	// ============================
	// CLEAN EXIT
	// ============================

	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();

	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY,
						  VectorXd::Zero(dof));

	return 0;
}