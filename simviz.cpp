/**
 * @file simviz.cpp
 * @brief Simulation and visualization of panda robot with 1 DOF gripper
 *
 */

#include <math.h>
#include <algorithm>
#include <array>
#include <signal.h>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <fstream>
#include <filesystem>
#include <vector>
#include <typeinfo>
#include <random>
#include <GLFW/glfw3.h>

#include "SaiGraphics.h"
#include "SaiModel.h"
#include "SaiSimulation.h"
#include "SaiPrimitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "logger/Logger.h"
#include "chai3d.h"

bool fSimulationRunning = true;
void sighandler(int) { fSimulationRunning = false; }

#include "redis_keys.h"

using namespace Eigen;
using namespace std;
using namespace chai3d;

// --- adding the sounds into the simulation ---
// declaring the variables for the sound
// cStereoMode stereoMode = C_STEREO_ACTIVE;
// cAudioDevice *audioDevice;
// cAudioSource *violinToneSources[4];
// std::array<bool, 4> audio_source_active = {false, false, false, false};
// std::array<int, 4> audio_source_cooldown_frames = {0, 0, 0, 0};
// std::array<std::vector<unsigned char>, 4> raw_audio_storage;

// struct AudioState
// {
// 	double frequency_hz = 440.0;
// 	double gain = 0.0;
// 	Eigen::Vector3d position = Eigen::Vector3d::Zero();
// 	bool in_contact = false;
// };

// mutex mutex_audio;

// // Simplified constraints for debugging
// static constexpr int kRetriggerDebounceFrames = 8;
// static constexpr double kConstantTestVolume = 0.15; // Safe volume to prevent clipping
// static constexpr double kMinAudibleFrequencyHz = 80.0;
// static constexpr double kMaxAudibleFrequencyHz = 1200.0;
// static constexpr double kFrequencyMatchToleranceHz = 120.0;
// static constexpr double kTangentialForceThresholdN = 0.05;

// struct StringAudioConfig
// {
// 	std::string link_name;
// 	double open_string_frequency_hz;
// };

// static const std::array<StringAudioConfig, 4> kStringConfigs = {{
// 	{"string1", 196.00}, // G3
// 	{"string2", 293.66}, // D4
// 	{"string3", 440.00}, // A4
// 	{"string4", 659.25}, // E5
// }};

// std::array<AudioState, 4> audio_states;

// mutex and globals
VectorXd ui_torques;
mutex mutex_torques, mutex_update;

// specify urdf and robots
static const string robot_name = "flexiv";
static const string violin_name = "violin";
static const string camera_name = "camera_fixed";
static const string sensor_link_name = "link7";
static const string contact_link_name = "bow";

// dynamic objects information
const vector<std::string> object_names = {};
vector<Affine3d> object_poses;
vector<VectorXd> object_velocities;
const int n_objects = object_names.size();

// --- Audio Helper Functions Declarations ---
// void initializeAudio(std::shared_ptr<SaiGraphics::SaiGraphics> graphics);
// void updateAudioPlayback();
// void updateAudioPhysics(std::shared_ptr<SaiSimulation::SaiSimulation> sim);
// void cleanupAudio();

// simulation thread
void simulation(std::shared_ptr<SaiSimulation::SaiSimulation> sim);

unsigned long long simulation_counter = 0;

int main()
{
	SaiModel::URDF_FOLDERS["STRADIBOT_FOLDER"] = string(STRADIBOT_FOLDER);

	static const string robot_file = string(STRADIBOT_FOLDER) + "/urdf_models/flexiv_violin/flexiv.urdf";
	static const string world_file = string(STRADIBOT_FOLDER) + "/urdf_models/world_stradibot.urdf";
	static const string violin_file = string(STRADIBOT_FOLDER) + "/urdf_models/flexiv_violin/violin.urdf";
	std::cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	auto redis_client = SaiCommon::RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = std::make_shared<SaiGraphics::SaiGraphics>(world_file, camera_name, false);

	// load robots
	auto robot = std::make_shared<SaiModel::SaiModel>(robot_file, false);
	auto violin = std::make_shared<SaiModel::SaiModel>(violin_file, false);
	robot->updateModel();
	violin->updateModel();
	ui_torques = VectorXd::Zero(robot->dof());

	// enable ui force interaction
	graphics->addUIForceInteraction(robot_name);

	// load simulation world
	auto sim = std::make_shared<SaiSimulation::SaiSimulation>(world_file, false);
	sim->setJointPositions(robot_name, robot->q());
	sim->setJointVelocities(robot_name, robot->dq());
	Vector3d control_position = redis_client.getEigen(CONTROL_POSITION_KEY);
	Affine3d control_pose = Affine3d::Identity();
	control_pose.translation() = control_position;

	// fill in object information
	for (int i = 0; i < n_objects; ++i)
	{
		object_poses.push_back(sim->getObjectPose(object_names[i]));
		object_velocities.push_back(sim->getObjectVelocity(object_names[i]));
	}

	// --- FORCE SENSOR AT EE ---
	Eigen::Affine3d T_link_sensor = Eigen::Affine3d::Identity();
	double sensor_filter_cutoff_freq = 0.5;
	sim->addSimulatedForceSensor(robot_name, sensor_link_name, T_link_sensor,
								 sensor_filter_cutoff_freq);

	// add sensor display in the graphics
	for (auto sensor_data : sim->getAllForceSensorData())
	{
		graphics->addForceSensorDisplay(sensor_data);
	}

	// ---- CONTACT FORCE ---
	// set simulation parameters
	sim->setCollisionRestitution(0.5);
	sim->setCoeffFrictionStatic(0.0);
	sim->setCoeffFrictionDynamic(0.0);

	/*------- Set up visualization -------*/
	Vector3d contact_pos = Vector3d::Zero();
	Vector3d contact_force = Vector3d::Zero();
	auto contact_list = sim->getContactList(robot_name, contact_link_name);
	if (!contact_list.empty())
	{
		contact_pos = contact_list[0].first;
		contact_force = contact_list[0].second;
	}

	// ---- REDIS KEYS ----
	int key_pressed = 0;

	redis_client.setEigen(JOINT_ANGLES_KEY, robot->q());
	redis_client.setEigen(JOINT_VELOCITIES_KEY, robot->dq());
	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * robot->q());
	redis_client.setEigen(CONTACT_POINT_FORCE_KEY, contact_force);
	redis_client.setEigen(CONTACT_POINT_POSITION_KEY, contact_pos);
	redis_client.setDouble(KEYBOARD_INPUT_KEY, key_pressed);

	//--------------------------------------------------------------------------
	// ADD SOUND
	//--------------------------------------------------------------------------
	// initializeAudio(graphics);

	// chai3d::cShapeSphere *control_sphere = graphics->createGoalSphere(control_position, 0.01, chai3d::cColorf(1.0, 0.65, 0.0), true);

	// start simulation thread
	thread sim_thread(simulation, sim);

	// while window is open:
	while (graphics->isWindowOpen() && fSimulationRunning)
	{
		control_position = redis_client.getEigen(CONTROL_POSITION_KEY);
		control_pose = Affine3d::Identity();
		control_pose.translation() = control_position;
		// graphics->updateGoalSphere(control_sphere, control_position, true, false, Matrix3d::Identity());
		graphics->updateRobotGraphics(robot_name, redis_client.getEigen(JOINT_ANGLES_KEY));

		if (graphics->isKeyPressed(49)) // 1 key
		{
			key_pressed = 1;
			cout << "1 key pressed" << endl;
		}
		else if (graphics->isKeyPressed(50)) // 2 key
		{
			key_pressed = 2;
			cout << "2 key pressed" << endl;
		}
		else if (graphics->isKeyPressed(51)) // 3 key
		{
			key_pressed = 3;
			cout << "3 key pressed" << endl;
		}
		else if (graphics->isKeyPressed(52)) // 4 key
		{
			key_pressed = 4;
			cout << "4 key pressed" << endl;
		}
		else if (graphics->isKeyPressed(57)) // 9 key
		{
			key_pressed = 9;
			cout << "9 key pressed" << endl;
		}
		else
		{
			key_pressed = 0;
		}
		redis_client.setDouble(KEYBOARD_INPUT_KEY, key_pressed);

		for (const auto sensor_data : sim->getAllForceSensorData())
		{
			graphics->updateDisplayedForceSensor(sensor_data);
		}
		{
			lock_guard<mutex> lock(mutex_update);
			for (int i = 0; i < n_objects; ++i)
			{
				graphics->updateObjectGraphics(object_names[i], object_poses[i]);
			}
		}

		graphics->renderGraphicsWorld();
		{
			lock_guard<mutex> lock(mutex_torques);
			ui_torques = graphics->getUITorques(robot_name);
		}

		//--------------------------------------------------------------------------
		// SOUND PLAYBACK UPDATE
		//--------------------------------------------------------------------------
		// updateAudioPlayback();

		// gets the sensor measurements
		// 	if (simulation_counter % 100 == 0)
		// 	{
		// 		cout << "force local frame:\t"
		// 			 << sim->getSensedForce(robot_name, sensor_link_name).transpose()
		// 			 << endl;
		// 		cout << "force world frame:\t"
		// 			 << sim->getSensedForce(robot_name, sensor_link_name, false).transpose()
		// 			 << endl;
		// 		cout << "moment local frame:\t"
		// 			 << sim->getSensedMoment(robot_name, sensor_link_name).transpose()
		// 			 << endl;
		// 		cout << "moment world frame:\t"
		// 			 << sim->getSensedMoment(robot_name, sensor_link_name, false).transpose()
		// 			 << endl;
		// 		cout << endl;
		// 	}

		simulation_counter++;
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	// cleanupAudio();

	return 0;
}

//------------------------------------------------------------------------------
void simulation(std::shared_ptr<SaiSimulation::SaiSimulation> sim)
{
	// create redis client
	auto redis_client = SaiCommon::RedisClient();
	redis_client.connect();

	// create a timer
	double sim_freq = 2000;
	SaiCommon::LoopTimer timer(sim_freq);

	sim->setTimestep(1.0 / sim_freq);
	sim->enableGravityCompensation(true);
	sim->enableJointLimits(robot_name);

	while (fSimulationRunning)
	{
		timer.waitForNextLoop();

		VectorXd control_torques = redis_client.getEigen(JOINT_TORQUES_COMMANDED_KEY);
		{
			lock_guard<mutex> lock(mutex_torques);
			sim->setJointTorques(robot_name, control_torques + ui_torques);
		}
		sim->integrate();
		redis_client.setEigen(JOINT_ANGLES_KEY, sim->getJointPositions(robot_name));
		redis_client.setEigen(JOINT_VELOCITIES_KEY, sim->getJointVelocities(robot_name));

		//--------------------------------------------------------------------------
		// SOUND PHYSICS UPDATE
		//--------------------------------------------------------------------------
		// updateAudioPhysics(sim);

		// update object information
		{
			lock_guard<mutex> lock(mutex_update);
			for (int i = 0; i < n_objects; ++i)
			{
				object_poses[i] = sim->getObjectPose(object_names[i]);
				object_velocities[i] = sim->getObjectVelocity(object_names[i]);
			}
		}
	}
	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
}

//==============================================================================
// AUDIO IMPLEMENTATION DETAILS
//==============================================================================

// void initializeAudio(std::shared_ptr<SaiGraphics::SaiGraphics> graphics)
// {
// 	audioDevice = new cAudioDevice();
// 	graphics->attachAudioDeviceToCamera(camera_name, audioDevice);

// 	std::string base_path = string(STRADIBOT_FOLDER) + "/sounds/";

// 	std::string file_paths[4] = {
// 		base_path + "string1_g3_clean_16.raw",
// 		base_path + "string2_d4_clean_16.raw",
// 		base_path + "string3_a4_clean_16.raw",
// 		base_path + "string4_e5_clean_16.raw"};

// 	for (int i = 0; i < static_cast<int>(kStringConfigs.size()); ++i)
// 	{
// 		cAudioBuffer *buffer = new cAudioBuffer();
// 		std::ifstream file(file_paths[i], std::ios::binary | std::ios::ate);
// 		bool buffer_loaded = false;
// 		if (file.is_open())
// 		{
// 			const std::streamsize size = file.tellg();
// 			file.seekg(0, std::ios::beg);
// 			raw_audio_storage[i].resize(static_cast<size_t>(size));
// 			if (file.read(reinterpret_cast<char *>(raw_audio_storage[i].data()), size))
// 			{
// 				buffer_loaded = buffer->setup(raw_audio_storage[i].data(),
// 											  static_cast<unsigned int>(size),
// 											  48000,
// 											  false,
// 											  16);
// 			}
// 		}

// 		violinToneSources[i] = new cAudioSource();
// 		if (buffer_loaded)
// 		{
// 			violinToneSources[i]->setAudioBuffer(buffer);
// 			violinToneSources[i]->setLoop(true); // Loop forever
// 			violinToneSources[i]->setPitch(1.0);
// 			violinToneSources[i]->setGain(0.0); // Start silent
// 			violinToneSources[i]->play();		// 🛑 Start playing immediately!
// 			std::cout << "✅ SUCCESS: Audio playing silently for " << kStringConfigs[i].link_name << std::endl;
// 		}
// 		else
// 		{
// 			std::cout << "❌ ERROR: Could not load RAW PCM file: " << file_paths[i] << std::endl;
// 			delete violinToneSources[i];
// 			violinToneSources[i] = nullptr;
// 			delete buffer;
// 		}
// 	}
// }

// void updateAudioPhysics(std::shared_ptr<SaiSimulation::SaiSimulation> sim)
// {
// 	std::array<AudioState, 4> next_audio_states;

// 	// 🛑 The Physics Debouncer: This memory array survives between loops
// 	static std::array<int, 4> contact_memory = {0, 0, 0, 0};
// 	static std::array<double, 4> tangential_force_lowpass = {0.0, 0.0, 0.0, 0.0};
// 	static std::array<double, 4> previous_force_signal = {0.0, 0.0, 0.0, 0.0};
// 	static std::array<unsigned long long, 4> last_zero_crossing_step = {0, 0, 0, 0};
// 	static std::array<double, 4> estimated_frequency_hz = {
// 		kStringConfigs[0].open_string_frequency_hz,
// 		kStringConfigs[1].open_string_frequency_hz,
// 		kStringConfigs[2].open_string_frequency_hz,
// 		kStringConfigs[3].open_string_frequency_hz};
// 	static unsigned long long step_counter = 0;
// 	static constexpr double sim_freq = 2000.0;

// 	for (int i = 0; i < static_cast<int>(kStringConfigs.size()); ++i)
// 	{
// 		next_audio_states[i] = AudioState();
// 		const auto contact_list = sim->getContactList(violin_name, kStringConfigs[i].link_name);

// 		if (!contact_list.empty())
// 		{
// 			const Eigen::Vector3d &contact_force = contact_list[0].second;
// 			const double tangential_force = contact_force.x();
// 			tangential_force_lowpass[i] =
// 				0.995 * tangential_force_lowpass[i] + 0.005 * tangential_force;
// 			const double force_signal = tangential_force - tangential_force_lowpass[i];

// 			if ((previous_force_signal[i] <= 0.0) && (force_signal > 0.0))
// 			{
// 				if (last_zero_crossing_step[i] > 0)
// 				{
// 					const double period_steps =
// 						static_cast<double>(step_counter - last_zero_crossing_step[i]);
// 					if (period_steps > 1.0)
// 					{
// 						const double measured_frequency = sim_freq / period_steps;
// 						if (measured_frequency >= kMinAudibleFrequencyHz &&
// 							measured_frequency <= kMaxAudibleFrequencyHz)
// 						{
// 							estimated_frequency_hz[i] =
// 								0.95 * estimated_frequency_hz[i] + 0.05 * measured_frequency;
// 						}
// 					}
// 				}
// 				last_zero_crossing_step[i] = step_counter;
// 			}
// 			previous_force_signal[i] = force_signal;

// 			// If it touches, fill the memory with 200 physics frames (0.1 seconds at 2000Hz)
// 			contact_memory[i] = 200;
// 		}
// 		else if (contact_memory[i] > 0)
// 		{
// 			// If the bow micro-bounces off, slowly drain the memory instead of instantly stopping
// 			contact_memory[i]--;
// 			previous_force_signal[i] = 0.0;
// 		}

// 		// As long as there is memory remaining, we treat it as a perfectly steady contact
// 		next_audio_states[i].frequency_hz = estimated_frequency_hz[i];
// 		if (contact_memory[i] > 0)
// 		{
// 			next_audio_states[i].in_contact = true;
// 			next_audio_states[i].gain = kConstantTestVolume;
// 		}
// 		else
// 		{
// 			next_audio_states[i].in_contact = false;
// 			next_audio_states[i].gain = 0.0;
// 		}
// 	}

// 	{
// 		lock_guard<mutex> lock(mutex_audio);
// 		audio_states = next_audio_states;
// 	}

// 	step_counter++;
// }

// void updateAudioPlayback()
// {
// 	// 🛑 Track volume in C++ to prevent OpenAL read-delays from causing oscillations
// 	static std::array<double, 4> smooth_gain_tracker = {0.0, 0.0, 0.0, 0.0};
// 	static std::array<bool, 4> was_active_last_update = {false, false, false, false};

// 	lock_guard<mutex> lock(mutex_audio);
// 	for (int i = 0; i < static_cast<int>(kStringConfigs.size()); ++i)
// 	{
// 		if (!violinToneSources[i])
// 			continue;

// 		const double measured_frequency = std::clamp(
// 			audio_states[i].frequency_hz, kMinAudibleFrequencyHz,
// 			kMaxAudibleFrequencyHz);
// 		const double target_frequency = kStringConfigs[i].open_string_frequency_hz;
// 		const double frequency_error =
// 			std::abs(measured_frequency - target_frequency);
// 		const double frequency_match_strength = std::clamp(
// 			1.0 - (frequency_error / kFrequencyMatchToleranceHz), 0.0, 1.0);
// 		double target_gain = audio_states[i].in_contact
// 								 ? kConstantTestVolume *
// 									   (0.15 + 0.85 * frequency_match_strength)
// 								 : 0.0;
// 		const bool string_active = target_gain > 0.005;

// 		// Smoothly glide the volume up and down (acting like a shock absorber)
// 		smooth_gain_tracker[i] = (0.9 * smooth_gain_tracker[i]) + (0.1 * target_gain);

// 		// Only send the command to OpenAL if the volume changed meaningfully
// 		if (std::abs(violinToneSources[i]->getGain() - smooth_gain_tracker[i]) > 0.001)
// 		{
// 			violinToneSources[i]->setGain(smooth_gain_tracker[i]);
// 		}

// 		if (string_active && !was_active_last_update[i])
// 		{
// 			std::cout << "[audio] activated " << kStringConfigs[i].link_name
// 					  << " | estimated freq: " << measured_frequency
// 					  << " Hz | target: " << target_frequency << " Hz"
// 					  << std::endl;
// 		}
// 		else if (!string_active && was_active_last_update[i])
// 		{
// 			std::cout << "[audio] deactivated " << kStringConfigs[i].link_name
// 					  << std::endl;
// 		}

// 		was_active_last_update[i] = string_active;
// 	}
// }

// void cleanupAudio()
// {
// 	for (int i = 0; i < static_cast<int>(kStringConfigs.size()); ++i)
// 	{
// 		if (violinToneSources[i])
// 		{
// 			cAudioBuffer *buffer = violinToneSources[i]->getAudioBuffer();
// 			delete violinToneSources[i];
// 			if (buffer)
// 				delete buffer;
// 		}
// 	}
// 	if (audioDevice)
// 		delete audioDevice;
// }
