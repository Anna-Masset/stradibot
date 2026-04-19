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
#include <iostream>
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
cStereoMode stereoMode = C_STEREO_ACTIVE;
cAudioDevice *audioDevice; // NB: could add audiobuffer to save the sound
cAudioBuffer *violinToneBuffer;
cAudioSource *violinToneSources[4];
std::vector<short> violinToneSamples;

struct AudioState
{
	double frequency_hz = 440.0;
	double gain = 0.0;
	Eigen::Vector3d position = Eigen::Vector3d::Zero();
	bool in_contact = false;
};

mutex mutex_audio;

static constexpr double kAudioSampleRate = 48000.0;
static constexpr double kWaveformBaseFrequencyHz = 440.0;
static constexpr double kMinAudibleFrequencyHz = 80.0;
static constexpr double kMaxAudibleFrequencyHz = 2000.0;
static constexpr double kTangentialForceThresholdN = 0.15;
static constexpr double kGainForceScale = 0.08;
static constexpr double kAudioGainMax = 0.35;

struct StringAudioConfig
{
	std::string link_name;
	double open_string_frequency_hz;
};

static const std::array<StringAudioConfig, 4> kStringConfigs = {{
	{"cord1", 196.00}, // G3
	{"cord2", 293.66}, // D4
	{"cord3", 440.00}, // A4
	{"cord4", 659.25}, // E5
}};

std::array<AudioState, 4> audio_states;

static std::vector<short> generateSineWaveSamples(double sample_rate,
												  double frequency_hz,
												  double duration_sec)
{
	const int sample_count = static_cast<int>(sample_rate * duration_sec);
	std::vector<short> samples(sample_count);
	for (int i = 0; i < sample_count; ++i)
	{
		const double time = static_cast<double>(i) / sample_rate;
		const double value = sin(2.0 * M_PI * frequency_hz * time);
		samples[i] = static_cast<short>(32767.0 * 0.35 * value);
	}
	return samples;
}

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
const vector<std::string>
	object_names = {};
vector<Affine3d> object_poses;
vector<VectorXd> object_velocities;
const int n_objects = object_names.size();

// simulation thread
void simulation(std::shared_ptr<SaiSimulation::SaiSimulation> sim);

unsigned long long simulation_counter = 0;

int main()
{

	// SaiModel::URDF_FOLDERS["CS225A_URDF_FOLDER"] = string(CS225A_URDF_FOLDER);
	// static const string robot_file = string(CS225A_URDF_FOLDER) + "/panda_violin/panda_arm.urdf";
	// static const string world_file = string(STRADIBOT_FOLDER) + "/world_stradibot.urdf";
	// std::cout << "Loading URDF world model file: " << world_file << endl;

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
	// robot->setQ();
	// robot->setDq();
	robot->updateModel();
	violin
	ui_torques = VectorXd::Zero(robot->dof());

	// enable ui force interaction
	graphics->addUIForceInteraction(robot_name);

	// load simulation world
	auto sim = std::make_shared<SaiSimulation::SaiSimulation>(world_file, false);
	sim->setJointPositions(robot_name, robot->q());
	sim->setJointVelocities(robot_name, robot->dq());

	// fill in object information
	for (int i = 0; i < n_objects; ++i)
	{
		object_poses.push_back(sim->getObjectPose(object_names[i]));
		object_velocities.push_back(sim->getObjectVelocity(object_names[i]));
	}

	// --- FORCE SENSOR AT EE ---
	// add simulated force sensor
	Eigen::Affine3d T_link_sensor = Eigen::Affine3d::Identity();
	// T_link_sensor.translate(Eigen::Vector3d(0.0, 0.0, 0.0));
	// T_link_sensor.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
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
	// init redis client values
	Vector3d contact_pos = Vector3d::Zero();
	Vector3d contact_force = Vector3d::Zero();
	auto contact_list = sim->getContactList(robot_name, contact_link_name);
	if (!contact_list.empty())
	{
		contact_pos = contact_list[0].first;
		contact_force = contact_list[0].second;
	}

	redis_client.setEigen(JOINT_ANGLES_KEY, robot->q());
	redis_client.setEigen(JOINT_VELOCITIES_KEY, robot->dq());
	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * robot->q());
	redis_client.setEigen(CONTACT_POINT_FORCE_KEY, contact_force);
	redis_client.setEigen(CONTACT_POINT_POSITION_KEY, contact_pos);

	//--------------------------------------------------------------------------
	// TRYING TO ADD SOUND - START
	//--------------------------------------------------------------------------
	audioDevice = new cAudioDevice();
	graphics->attachAudioDeviceToCamera(camera_name, audioDevice);
	violinToneSamples =
		generateSineWaveSamples(kAudioSampleRate, kWaveformBaseFrequencyHz, 1.0);
	violinToneBuffer = new cAudioBuffer();
	violinToneBuffer->setup(
		reinterpret_cast<unsigned char *>(violinToneSamples.data()),
		static_cast<unsigned int>(violinToneSamples.size() * sizeof(short)),
		static_cast<int>(kAudioSampleRate), false, 16);
	for (int i = 0; i < static_cast<int>(kStringConfigs.size()); ++i)
	{
		violinToneSources[i] = new cAudioSource();
		violinToneSources[i]->setAudioBuffer(violinToneBuffer);
		violinToneSources[i]->setLoop(true);
		violinToneSources[i]->setGain(0.0);
		violinToneSources[i]->setPitch(
			kStringConfigs[i].open_string_frequency_hz / kWaveformBaseFrequencyHz);
		violinToneSources[i]->play();
	}

	//--------------------------------------------------------------------------
	// TRYING TO ADD SOUND - END
	//--------------------------------------------------------------------------

	// start simulation thread
	thread sim_thread(simulation, sim);

	// while window is open:
	while (graphics->isWindowOpen() && fSimulationRunning)
	{
		graphics->updateRobotGraphics(robot_name, redis_client.getEigen(JOINT_ANGLES_KEY));
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
		{
			lock_guard<mutex> lock(mutex_audio);
			for (int i = 0; i < static_cast<int>(kStringConfigs.size()); ++i)
			{
				const double clamped_frequency =
					std::clamp(audio_states[i].frequency_hz, kMinAudibleFrequencyHz,
							   kMaxAudibleFrequencyHz);
				violinToneSources[i]->setPitch(
					clamped_frequency / kWaveformBaseFrequencyHz);
				violinToneSources[i]->setGain(
					audio_states[i].in_contact ? audio_states[i].gain : 0.0);
				violinToneSources[i]->setSourcePos(cVector3d(
					audio_states[i].position.x(), audio_states[i].position.y(),
					audio_states[i].position.z()));
				violinToneSources[i]->setSourceVel(cVector3d(0.0, 0.0, 0.0));
			}
		}

		// gets the sensor measurements
		if (simulation_counter % 100 == 0)
		{
			cout << "force local frame:\t"
				 << sim->getSensedForce(robot_name, sensor_link_name).transpose()
				 << endl;
			cout
				<< "force world frame:\t"
				<< sim->getSensedForce(robot_name, sensor_link_name, false).transpose()
				<< endl;
			cout << "moment local frame:\t"
				 << sim->getSensedMoment(robot_name, sensor_link_name).transpose()
				 << endl;
			cout << "moment world frame:\t"
				 << sim->getSensedMoment(robot_name, sensor_link_name, false)
						.transpose()
				 << endl;
			cout << endl;
		}

		// gets the contact forces
		if (simulation_counter % 100 == 0)
		{
			contact_list = sim->getContactList(robot_name, contact_link_name);
			if (!contact_list.empty())
			{
				contact_pos = contact_list[0].first;
				contact_force = contact_list[0].second;
				redis_client.setEigen(CONTACT_POINT_POSITION_KEY, contact_pos);
				redis_client.setEigen(CONTACT_POINT_FORCE_KEY, contact_force);
				std::cout << "contact at " << robot_name << " " << contact_link_name
						  << std::endl;
				int n = contact_list.size();
				for (int i = 0; i < n; i++)
				{
					std::cout << "contact point " << i << " : "
							  << contact_list[i].first.transpose() << "\n";
					std::cout << "contact force " << i << " : "
							  << contact_list[i].second.transpose() << "\n";
				}
				std::cout << endl;
			}
		}

		simulation_counter++;
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	for (int i = 0; i < static_cast<int>(kStringConfigs.size()); ++i)
	{
		delete violinToneSources[i];
	}
	delete violinToneBuffer;
	delete audioDevice;

	return 0;
}

//------------------------------------------------------------------------------
void simulation(std::shared_ptr<SaiSimulation::SaiSimulation> sim)
{
	// fSimulationRunning = true;

	// create redis client
	auto redis_client = SaiCommon::RedisClient();
	redis_client.connect();

	// create a timer
	double sim_freq = 2000;
	SaiCommon::LoopTimer timer(sim_freq);

	sim->setTimestep(1.0 / sim_freq);
	sim->enableGravityCompensation(true);
	sim->enableJointLimits(robot_name);

	std::array<double, 4> tangential_force_lowpass = {0.0, 0.0, 0.0, 0.0};
	std::array<double, 4> previous_force_signal = {0.0, 0.0, 0.0, 0.0};
	std::array<unsigned long long, 4> last_zero_crossing_step = {0, 0, 0, 0};
	std::array<double, 4> estimated_frequency_hz = {
		kStringConfigs[0].open_string_frequency_hz,
		kStringConfigs[1].open_string_frequency_hz,
		kStringConfigs[2].open_string_frequency_hz,
		kStringConfigs[3].open_string_frequency_hz,
	};
	std::array<AudioState, 4> next_audio_states;
	unsigned long long step_counter = 0;

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

		for (int i = 0; i < static_cast<int>(kStringConfigs.size()); ++i)
		{
			next_audio_states[i] = AudioState();
			next_audio_states[i].frequency_hz = estimated_frequency_hz[i];

			const auto contact_list =
				sim->getContactList(violin_name, kStringConfigs[i].link_name);
			if (contact_list.empty())
			{
				previous_force_signal[i] = 0.0;
				continue;
			}

			const Eigen::Vector3d &contact_position = contact_list[0].first;
			const Eigen::Vector3d &contact_force = contact_list[0].second;

			// First approximation: bow motion is mostly along world X.
			const double tangential_force = contact_force.x();
			tangential_force_lowpass[i] =
				0.995 * tangential_force_lowpass[i] + 0.005 * tangential_force;
			const double force_signal =
				tangential_force - tangential_force_lowpass[i];

			if ((previous_force_signal[i] <= 0.0) && (force_signal > 0.0))
			{
				if (last_zero_crossing_step[i] > 0)
				{
					const double period_steps =
						static_cast<double>(step_counter - last_zero_crossing_step[i]);
					if (period_steps > 1.0)
					{
						const double measured_frequency = sim_freq / period_steps;
						if (measured_frequency >= kMinAudibleFrequencyHz &&
							measured_frequency <= kMaxAudibleFrequencyHz)
						{
							estimated_frequency_hz[i] =
								0.92 * estimated_frequency_hz[i] +
								0.08 * measured_frequency;
						}
					}
				}
				last_zero_crossing_step[i] = step_counter;
			}
			previous_force_signal[i] = force_signal;

			next_audio_states[i].frequency_hz = estimated_frequency_hz[i];
			next_audio_states[i].gain = std::clamp(
				kGainForceScale * std::abs(tangential_force), 0.0, kAudioGainMax);
			next_audio_states[i].position = contact_position;
			next_audio_states[i].in_contact =
				(std::abs(tangential_force) > kTangentialForceThresholdN);
		}

		{
			lock_guard<mutex> lock(mutex_audio);
			audio_states = next_audio_states;
		}

		// update object information
		{
			lock_guard<mutex> lock(mutex_update);
			for (int i = 0; i < n_objects; ++i)
			{
				object_poses[i] = sim->getObjectPose(object_names[i]);
				object_velocities[i] = sim->getObjectVelocity(object_names[i]);
			}
		}

		step_counter++;
	}
	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
}
