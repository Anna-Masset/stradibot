/**
 * @file redis_keys.h
 * @brief Contains all redis keys for simulation and control.
 *
 */

const std::string JOINT_ANGLES_KEY = "sai::sim::flexiv::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai::sim::flexiv::sensors::dq";
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai::sim::flexiv::actuators::fgc";
const std::string CONTROLLER_RUNNING_KEY = "sai::sim::flexiv::controller";

const std::string CONTACT_POINT_POSITION_KEY = "sai::sim::bow::contact::position";
const std::string CONTACT_POINT_FORCE_KEY = "sai::sim::bow::contact::force";

const std::string BOW_FORCE_SENSOR_WORLD_KEY = "sai::sim::bow::sensor::force::world";
const std::string BOW_FORCE_SENSOR_LOCAL_KEY = "sai::sim::bow::sensor::force::local";

const std::string CONTROL_POSITION_KEY = "sai::sim::bow::control::position";

const std::string KEYBOARD_INPUT_KEY = "sai::sim::string::keyboard_input";

const std::string FINGER_TARGET_KEY = "sai::sim::bow::finger::target";
const std::string FINGER_STATE_KEY = "sai::sim::bow::finger::state";

const std::string BOW_FORCE_CONTROLLED_KEY = "sai::control::bow::force_controlled";
const std::string BOW_SPEED_CONTROLLED_KEY = "sai::control::bow::speed_controlled";