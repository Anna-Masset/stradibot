/**
 * @file controller.cpp
 * @brief Controller with state machine: CALIBRATION, MOVE_TO_STRING, BOWING
 */

#include <SaiModel.h>
#include "SaiPrimitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>
#include <cmath>

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

enum class ControllerState {
	CALIBRATION,
	MOVE_TO_STRING,
	BOWING
};

enum class MoveToStringPhase {
	ADJACENT,
	LIFTING,
	MOVING,
	LOWERING
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

	if (key_once) {
		key_pressed = (int)redis_client.getDouble(KEYBOARD_INPUT_KEY);
		if (key_pressed >= 1 && key_pressed <= 4) {
			string_positions[key_pressed - 1] = general_task->getCurrentPosition();
			string_orientations[key_pressed - 1] = general_task->getCurrentOrientation();
			cout << "String " << key_pressed << " position: "
				 << string_positions[key_pressed - 1].transpose() << "\n";
			cout << "String " << key_pressed << " orientation:\n"
				 << string_orientations[key_pressed - 1] << "\n";
		}
	}
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
				  double &bow_displacement,
				  double &bow_velocity_dir,
				  double control_freq,
				  double bow_speed,
				  double max_displacement)
{
	general_task->parametrizeForceMotionSpaces(1, Vector3d::UnitY());
	general_task->parametrizeMomentRotMotionSpaces(0);

	bow_displacement += bow_velocity_dir * bow_speed / control_freq;

	if (bow_displacement >= max_displacement) {
		bow_displacement = max_displacement;
		bow_velocity_dir = -1.0;
	} else if (bow_displacement <= -max_displacement) {
		bow_displacement = -max_displacement;
		bow_velocity_dir = 1.0;
	}

	ee_pos_desired = string_position + bow_displacement * bowing_dir;
	ee_force_desired << 0.0, -1.0, 0.0;
	ee_ori_desired = string_orientation_world;
}

// ============================
// FUNCTION: MOVE TO STRING
// Renvoie true quand la transition est terminée
// ============================

bool moveToString(Vector3d &ee_pos_desired,
				  Vector3d &ee_force_desired,
				  Matrix3d &ee_ori_desired,
				  const shared_ptr<MotionForceTask> &general_task,
				  const Vector3d &target_string_position,
				  const Matrix3d &target_string_orientation,
				  const Vector3d &transition_start_pos,
				  MoveToStringPhase &phase,
				  double lift_height,
				  double position_threshold,
				  double control_freq,
				  double transition_speed)
{
	general_task->parametrizeForceMotionSpaces(0);
	general_task->parametrizeMomentRotMotionSpaces(0);
	ee_force_desired.setZero();

	const double step = transition_speed / control_freq;
	Vector3d current_pos = general_task->getCurrentPosition();

	auto move_toward = [&](const Vector3d &waypoint) {
		Vector3d error = waypoint - ee_pos_desired;
		double dist = error.norm();
		if (dist > step) ee_pos_desired += error.normalized() * step;
		else             ee_pos_desired = waypoint;
	};

	// Adjacent strings: direct move
	if (phase == MoveToStringPhase::ADJACENT) {
		ee_ori_desired = target_string_orientation;
		move_toward(target_string_position);
		return (current_pos - target_string_position).norm() < position_threshold;
	}

	// Non-adjacent: lift, move laterally, lower
	Vector3d lifted_start  = transition_start_pos + Vector3d(0, 0, lift_height);
	Vector3d lifted_target = target_string_position + Vector3d(0, 0, lift_height);

	switch (phase) {
	case MoveToStringPhase::LIFTING:
		move_toward(lifted_start);
		if ((current_pos - lifted_start).norm() < position_threshold) {
			phase = MoveToStringPhase::MOVING;
			cout << "Transition: lifting done, moving laterally\n";
		}
		break;

	case MoveToStringPhase::MOVING:
		ee_ori_desired = target_string_orientation;
		move_toward(lifted_target);
		if ((current_pos - lifted_target).norm() < position_threshold) {
			phase = MoveToStringPhase::LOWERING;
			cout << "Transition: lateral move done, lowering\n";
		}
		break;

	case MoveToStringPhase::LOWERING:
		move_toward(target_string_position);
		if ((current_pos - target_string_position).norm() < position_threshold) {
			return true;
		}
		break;

	default:
		return true;
	}

	return false;
}

// ============================
// MAIN
// ============================

int main()
{
	static const string robot_file =
		string(STRADIBOT_FOLDER) + "/urdf_models/flexiv_violin/flexiv.urdf";
	const string robot_name = "flexiv";

	auto redis_client = SaiCommon::RedisClient();
	redis_client.connect();

	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

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

	general_task->disableInternalOtg();
	general_task->enableVelocitySaturation(0.3, M_PI / 3);

	general_task->parametrizeForceMotionSpaces(1, Vector3d::UnitY());
	general_task->parametrizeMomentRotMotionSpaces(0);

	general_task->setPosControlGains(400, 40, 0);
	general_task->setOriControlGains(400, 40, 0);
	general_task->setForceControlGains(0.7, 10.0, 1.3);

	auto joint_task = std::make_shared<JointTask>(robot);
	joint_task->setGains(400, 40, 0);

	VectorXd q_desired = robot->q();
	joint_task->setGoalPosition(q_desired);

	// ============================
	// BOWING PARAMETERS
	// ============================

	const double bow_speed        = 0.06;
	const double max_displacement = 0.15; // m, bow reverses at ±15cm
	const double near_end_ratio   = 0.75; // reverse bow direction early when changing strings near end-of-stroke

	// ============================
	// TRANSITION PARAMETERS
	// ============================

	const double lift_height          = 0.05;
	const double position_threshold   = 0.01;
	const double transition_speed     = 0.15; // m/s for goal interpolation during string changes

	// ============================
	// STATE VARIABLES
	// ============================

	const Vector3d initial_position    = general_task->getCurrentPosition();
	const Matrix3d initial_orientation = general_task->getCurrentOrientation();

	int key_pressed = 0;
	int prev_key    = 0;

	vector<Vector3d> string_positions(4);
	vector<Matrix3d> string_orientations(4);

	int current_string = 0;
	int target_string  = 0;

	double bow_displacement  = 0.0;
	double bow_velocity_dir  = 1.0;

	MoveToStringPhase move_phase = MoveToStringPhase::ADJACENT;
	Vector3d transition_start_pos = Vector3d::Zero();

	Vector3d string_position         = Vector3d(0.9215, 0.0, 0.0);
	Matrix3d string_orientation_world = Matrix3d::Identity();
	Vector3d bowing_dir              = string_orientation_world.col(2);

	Vector3d ee_pos_desired;
	Vector3d ee_force_desired;
	Matrix3d ee_ori_desired    = string_orientation_world;
	Vector3d ee_moment_desired = Vector3d::Zero();

	Vector3d control_position = general_task->getCurrentPosition();

	ControllerState state    = ControllerState::CALIBRATION;
	double state_start_time  = 0.0;

	// ============================
	// CONTROL LOOP
	// ============================

	runloop = true;
	double control_freq = 1000;
	SaiCommon::LoopTimer timer(control_freq, 1e6);

	cout << "Calibration: press 1-4 to save each string position, press 9 when done.\n";

	while (runloop)
	{
		timer.waitForNextLoop();

		double time          = timer.elapsedSimTime();
		double time_in_state = time - state_start_time;
		(void)time_in_state;

		robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
		robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
		robot->updateModel();

		// Read keyboard (one event per press)
		int current_key = (int)redis_client.getDouble(KEYBOARD_INPUT_KEY);
		bool key_once   = (current_key != 0 && prev_key == 0);
		prev_key        = current_key;

		// ============================
		// STATE MACHINE
		// ============================

		switch (state)
		{

		// ---- CALIBRATION ----
		case ControllerState::CALIBRATION:
		{
			calibration(ee_pos_desired, ee_force_desired, ee_ori_desired,
						ee_moment_desired, key_pressed, general_task,
						redis_client, string_positions, string_orientations,
						initial_orientation, initial_position);

			if (key_pressed == 9) {
				cout << "Calibration done. Starting on string 1.\n";
				current_string         = 0;
				target_string          = 0;
				string_position        = string_positions[0];
				string_orientation_world = string_orientations[0];
				bowing_dir             = string_orientation_world.col(2);
				bow_displacement       = 0.0;
				bow_velocity_dir       = 1.0;
				general_task->disableInternalOtg();
				state           = ControllerState::BOWING;
				state_start_time = time;
			}
			break;
		}

		// ---- BOWING ----
		case ControllerState::BOWING:
		{
			if (key_once && current_key >= 1 && current_key <= 4) {
				int requested = current_key - 1;
				if (requested != current_string) {
					target_string = requested;

					double ratio = bow_displacement / max_displacement;
					if (abs(ratio) >= near_end_ratio) {
						bow_velocity_dir *= -1.0;
					}

					transition_start_pos = general_task->getCurrentPosition();

					if (abs(target_string - current_string) == 1) {
						move_phase = MoveToStringPhase::ADJACENT;
					} else {
						move_phase = MoveToStringPhase::LIFTING;
					}

					cout << "String change: " << current_string + 1
						 << " -> " << target_string + 1 << "\n";

					state           = ControllerState::MOVE_TO_STRING;
					state_start_time = time;
					break; // skip bowingMotion this frame
				}
			}

			bowingMotion(ee_pos_desired, ee_ori_desired, ee_force_desired,
						 general_task, string_position, string_orientation_world,
						 bowing_dir, bow_displacement, bow_velocity_dir,
						 control_freq, bow_speed, max_displacement);
			break;
		}

		// ---- MOVE TO STRING ----
		case ControllerState::MOVE_TO_STRING:
		{
			bool done = moveToString(
				ee_pos_desired, ee_force_desired, ee_ori_desired,
				general_task,
				string_positions[target_string],
				string_orientations[target_string],
				transition_start_pos,
				move_phase,
				lift_height,
				position_threshold,
				control_freq,
				transition_speed);

			if (done) {
				current_string           = target_string;
				string_position          = string_positions[current_string];
				string_orientation_world = string_orientations[current_string];
				bowing_dir               = string_orientation_world.col(2);
				bow_displacement         = 0.0;

				cout << "Now bowing on string " << current_string + 1 << "\n";
				state           = ControllerState::BOWING;
				state_start_time = time;
			}
			break;
		}

		} // end switch

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

		redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		redis_client.setEigen(CONTROL_POSITION_KEY, control_position);

		// publish bow pose for KF
		redis_client.setEigen(BOW_FROG_POSITION_KEY,
			robot->position("bow") + robot->rotation("bow") * Vector3d(0.0, 0.0, 0.376));
		redis_client.setEigen(BOW_DIRECTION_KEY, -robot->rotation("bow").col(2));
		redis_client.setEigen(SENSOR_ROTATION_KEY, robot->rotation("bow"));
		redis_client.setEigen(SENSOR_POSITION_KEY, robot->position("bow"));
		redis_client.setDouble(TARGET_STRING_KEY, key_pressed - 1);
	}
	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();

	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, VectorXd::Zero(dof));

	return 0;
}
