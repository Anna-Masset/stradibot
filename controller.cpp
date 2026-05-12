/**
 * @file controller.cpp
 * @brief Stradibot controller — MIDI FSM + manual bowing mode
 *
 * Usage:
 *   ./controller_stradibot                          → manual bowing
 *   ./controller_stradibot file.mid                → MIDI playback
 *   ./controller_stradibot file.mid /dev/ttyACM0   → MIDI + Arduino (Linux)
 *   ./controller_stradibot file.mid /dev/cu.usbmodemXXXX → MIDI + Arduino (Mac)
 */

#include <SaiModel.h>
#include "SaiPrimitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <cmath>
#include <climits>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace std;
using namespace Eigen;
using namespace SaiPrimitives;

#include <signal.h>
bool runloop = false;
static int serial_fd = -1;

void sighandler(int) {
    runloop = false;
    if (serial_fd >= 0) { uint8_t z = 0; [[maybe_unused]] auto r = write(serial_fd, &z, 1); }
}

// Mapping fret -> solenoid number (1-8), 0 = open string (no solenoid)
// Each solenoid is a curved bar that presses all 4 strings at once.
// The bow selects the string; the solenoid selects the fret.
static const int FRET_SOLENOID[9] = {
    0,  // fret 0: open string
    1,  // fret 1
    2,  // fret 2
    3,  // fret 3
    4,  // fret 4
    5,  // fret 5
    6,  // fret 6
    7,  // fret 7
    8,  // fret 8
};

static int last_sent_solenoid = -1;

static void serialSend(int solenoid) {
    if (serial_fd < 0 || solenoid == last_sent_solenoid) return;
    uint8_t byte = (uint8_t)solenoid;
    [[maybe_unused]] auto r = write(serial_fd, &byte, 1);
    last_sent_solenoid = solenoid;
}

static int openSerialPort(const string& port) {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        cerr << "Warning: cannot open serial port " << port << " — Arduino disabled\n";
        return -1;
    }
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    tcgetattr(fd, &tty);
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_lflag = 0;
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tcsetattr(fd, TCSANOW, &tty);
    cout << "Serial port " << port << " opened (115200 baud).\n";
    return fd;
}

#include "redis_keys.h"

// ============================================================
// MIDI PARSER
// ============================================================

static const int OPEN_NOTES[4] = {55, 62, 69, 76}; // G3, D4, A4, E5

struct NoteEvent {
    int    string_idx;
    int    fret;
    double start_time;  // seconds from start of piece
    double duration;    // seconds
};

static pair<int,int> midiToViolin(int note)
{
    for (int s = 0; s < 4; s++) {
        int diff = note - OPEN_NOTES[s];
        if (diff >= 0 && diff <= 4) return {s, diff};
    }
    // Out of range: clamp to nearest string/fret
    int best_s = 0, best_dist = INT_MAX;
    for (int s = 0; s < 4; s++) {
        int f    = max(0, min(4, note - OPEN_NOTES[s]));
        int dist = abs(note - (OPEN_NOTES[s] + f));
        if (dist < best_dist) { best_dist = dist; best_s = s; }
    }
    return {best_s, max(0, min(4, note - OPEN_NOTES[best_s]))};
}

static uint32_t readVLQ(ifstream &f)
{
    uint32_t val = 0;
    uint8_t b;
    do {
        f.read(reinterpret_cast<char*>(&b), 1);
        val = (val << 7) | (b & 0x7F);
    } while (b & 0x80);
    return val;
}

static uint32_t readBE32(ifstream &f)
{
    uint8_t buf[4];
    f.read(reinterpret_cast<char*>(buf), 4);
    return ((uint32_t)buf[0]<<24)|((uint32_t)buf[1]<<16)|((uint32_t)buf[2]<<8)|buf[3];
}

static uint16_t readBE16(ifstream &f)
{
    uint8_t buf[2];
    f.read(reinterpret_cast<char*>(buf), 2);
    return ((uint16_t)buf[0]<<8)|buf[1];
}

static vector<NoteEvent> parseMidi(const string &filename)
{
    ifstream f(filename, ios::binary);
    if (!f) { cerr << "Cannot open MIDI file: " << filename << "\n"; return {}; }

    // --- Header ---
    char magic[4];
    f.read(magic, 4);
    if (string(magic, 4) != "MThd") { cerr << "Not a valid MIDI file\n"; return {}; }
    readBE32(f);                        // length (always 6)
    uint16_t format   = readBE16(f);
    uint16_t ntracks  = readBE16(f);
    uint16_t division = readBE16(f);
    (void)format;

    // Bit15=0 → ticks per beat; bit15=1 → SMPTE (not handled here)
    uint32_t ticks_per_beat = division & 0x7FFF;
    if (ticks_per_beat == 0) ticks_per_beat = 480;

    // --- Tempo map ---
    struct TempoChange { uint32_t tick; uint32_t tempo_us; };
    vector<TempoChange> tempo_map = {{0, 500000}};

    // --- Raw notes collected across all tracks ---
    struct RawNote { int midi_note; uint32_t on_tick; uint32_t off_tick; };
    vector<RawNote> raw_notes;
    map<int, uint32_t> note_on_ticks;

    for (int t = 0; t < ntracks; t++) {
        char mtrk[4];
        f.read(mtrk, 4);
        if (!f || string(mtrk, 4) != "MTrk") break;

        uint32_t chunk_len   = readBE32(f);
        streampos chunk_end  = f.tellg() + (streampos)chunk_len;

        uint32_t abs_tick      = 0;
        uint8_t  running_status = 0;

        while (f.tellg() < chunk_end && f.good()) {
            abs_tick += readVLQ(f);

            uint8_t b;
            f.read(reinterpret_cast<char*>(&b), 1);
            if (!f) break;

            // --- System / meta events are NOT channel messages ---
            if (b == 0xFF) {
                uint8_t  meta_type;
                f.read(reinterpret_cast<char*>(&meta_type), 1);
                uint32_t meta_len = readVLQ(f);
                if (meta_type == 0x51 && meta_len == 3) {
                    uint8_t buf[3];
                    f.read(reinterpret_cast<char*>(buf), 3);
                    uint32_t new_tempo = ((uint32_t)buf[0]<<16)|((uint32_t)buf[1]<<8)|buf[2];
                    tempo_map.push_back({abs_tick, new_tempo});
                } else if (meta_type == 0x2F) {
                    break; // End of Track
                } else {
                    f.seekg(meta_len, ios::cur);
                }
                continue;
            }

            if (b == 0xF0 || b == 0xF7) {
                uint32_t len = readVLQ(f);
                f.seekg(len, ios::cur);
                continue;
            }

            // --- Channel message ---
            if (b & 0x80) {
                running_status = b;
            } else {
                f.seekg(-1, ios::cur); // data byte → running status applies
            }

            uint8_t type = running_status & 0xF0;

            if (type == 0x80 || type == 0x90) {
                uint8_t note, vel;
                f.read(reinterpret_cast<char*>(&note), 1);
                f.read(reinterpret_cast<char*>(&vel),  1);
                if (type == 0x90 && vel > 0) {
                    note_on_ticks[note] = abs_tick;
                } else {
                    auto it = note_on_ticks.find(note);
                    if (it != note_on_ticks.end()) {
                        raw_notes.push_back({(int)note, it->second, abs_tick});
                        note_on_ticks.erase(it);
                    }
                }
            } else if (type == 0xA0 || type == 0xB0 || type == 0xE0) {
                f.seekg(2, ios::cur);
            } else if (type == 0xC0 || type == 0xD0) {
                f.seekg(1, ios::cur);
            }
        }

        f.seekg(chunk_end);
    }

    // --- Convert ticks → seconds ---
    auto ticksToSecs = [&](uint32_t tick) -> double {
        double   secs      = 0.0;
        uint32_t prev_tick = 0;
        uint32_t cur_tempo = 500000;
        for (const auto &tc : tempo_map) {
            if (tc.tick >= tick) break;
            secs      += (double)(tc.tick - prev_tick) * cur_tempo / (ticks_per_beat * 1e6);
            prev_tick  = tc.tick;
            cur_tempo  = tc.tempo_us;
        }
        secs += (double)(tick - prev_tick) * cur_tempo / (ticks_per_beat * 1e6);
        return secs;
    };

    // --- Build NoteEvent list ---
    vector<NoteEvent> events;
    events.reserve(raw_notes.size());
    for (auto &rn : raw_notes) {
        auto [s, fret] = midiToViolin(rn.midi_note);
        double start = ticksToSecs(rn.on_tick);
        double dur   = ticksToSecs(rn.off_tick) - start;
        if (dur > 0.0) events.push_back({s, fret, start, dur});
    }

    sort(events.begin(), events.end(),
         [](const NoteEvent &a, const NoteEvent &b){ return a.start_time < b.start_time; });

    cout << "Parsed " << events.size() << " notes from " << filename << "\n";
    return events;
}

// ============================================================
// STATE MACHINE ENUMS
// ============================================================

enum class ControllerState {
    CALIBRATION,
    MANUAL_BOWING,
    LOAD_MIDI,
    IDLE,
    SET_FRET,
    MOVE_TO_STRING,
    BOW_NOTE,
    NEXT_NOTE,
    WAIT_SILENCE,
    END
};

enum class MoveToStringPhase {
    ADJACENT,
    LIFTING,
    MOVING,
    LOWERING
};

// ============================================================
// HELPER: BOWING MOTION
// ============================================================

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

    ee_pos_desired   = string_position + bow_displacement * bowing_dir;
    ee_force_desired << 0.0, -1.0, 0.0;
    ee_ori_desired   = string_orientation_world;
}

// ============================================================
// HELPER: CALIBRATION
// ============================================================

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
    bool key_once   = (current_key != 0 && prev_key_pressed == 0);
    prev_key_pressed = current_key;

    if (key_once) {
        key_pressed = current_key;
        if (key_pressed >= 1 && key_pressed <= 4) {
            string_positions[key_pressed - 1]    = general_task->getCurrentPosition();
            string_orientations[key_pressed - 1] = general_task->getCurrentOrientation();
            cout << "String " << key_pressed << " position: "
                 << string_positions[key_pressed - 1].transpose() << "\n";
        }
    }
}

// ============================================================
// HELPER: MOVE TO STRING  — returns true when done
// ============================================================

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

    const double step        = transition_speed / control_freq;
    Vector3d current_pos     = general_task->getCurrentPosition();

    auto move_toward = [&](const Vector3d &waypoint) {
        Vector3d error = waypoint - ee_pos_desired;
        double dist = error.norm();
        if (dist > step) ee_pos_desired += error.normalized() * step;
        else             ee_pos_desired  = waypoint;
    };

    // Adjacent strings: direct move
    if (phase == MoveToStringPhase::ADJACENT) {
        ee_ori_desired = target_string_orientation;
        move_toward(target_string_position);
        return (current_pos - target_string_position).norm() < position_threshold;
    }

    // Non-adjacent: lift / move laterally / lower
    Vector3d lifted_start  = transition_start_pos   + Vector3d(0, 0, lift_height);
    Vector3d lifted_target = target_string_position + Vector3d(0, 0, lift_height);

    switch (phase) {
    case MoveToStringPhase::LIFTING:
        move_toward(lifted_start);
        if ((current_pos - lifted_start).norm() < position_threshold) {
            phase = MoveToStringPhase::MOVING;
            cout << "Transition: lift done, moving laterally\n";
        }
        break;
    case MoveToStringPhase::MOVING:
        ee_ori_desired = target_string_orientation;
        move_toward(lifted_target);
        if ((current_pos - lifted_target).norm() < position_threshold) {
            phase = MoveToStringPhase::LOWERING;
            cout << "Transition: lateral done, lowering\n";
        }
        break;
    case MoveToStringPhase::LOWERING:
        move_toward(target_string_position);
        if ((current_pos - target_string_position).norm() < position_threshold)
            return true;
        break;
    default:
        return true;
    }
    return false;
}

// ============================================================
// MAIN
// ============================================================

int main(int argc, char *argv[])
{
    static const string robot_file =
        string(STRADIBOT_FOLDER) + "/urdf_models/flexiv_violin/flexiv.urdf";

    // argv[1] = optional MIDI file path, argv[2] = optional serial port
    const string midi_file   = (argc > 1) ? string(argv[1]) : "";
    const bool   midi_mode   = !midi_file.empty();
    const string serial_port = (argc > 2) ? string(argv[2]) : "";

    auto redis_client = SaiCommon::RedisClient();
    redis_client.connect();

    if (!serial_port.empty())
        serial_fd = openSerialPort(serial_port);
    serialSend(0); // release all solenoids at startup

    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT,  &sighandler);

    auto robot = std::make_shared<SaiModel::SaiModel>(robot_file, false);
    robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
    robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
    robot->updateModel();

    int dof = robot->dof();
    VectorXd command_torques = VectorXd::Zero(dof);
    MatrixXd N_prec          = MatrixXd::Identity(dof, dof);

    // ---- Task setup ----
    const string  control_link  = "bow";
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

    // ---- Constants ----
    const double control_freq = 1000.0;

    // MIDI bowing
    const double BOW_AMPLITUDE  = 0.25;  // m — half-stroke range
    const double MAX_BOW_SPEED  = 0.50;  // m/s
    const double LIFT_HEIGHT    = 0.03;  // m — MIDI string changes
    const double SILENCE_THRESH = 0.05;  // s — gaps shorter than this are ignored
    const double MIN_STROKE     = 0.05;  // m — minimum stroke guaranteed after string change

    // Manual bowing (original values)
    const double manual_bow_speed   = 0.06;
    const double manual_max_disp    = 0.15;
    const double manual_lift_height = 0.05;
    const double near_end_ratio     = 0.75;

    // Shared
    const double position_threshold = 0.01; // m
    const double transition_speed   = 0.15; // m/s

    // ---- State variables ----
    const Vector3d initial_position    = general_task->getCurrentPosition();
    const Matrix3d initial_orientation = general_task->getCurrentOrientation();

    int key_pressed = 0;
    int prev_key    = 0;

    vector<Vector3d> string_positions(4);
    vector<Matrix3d> string_orientations(4);

    int current_string = 0;
    int target_string  = 0;

    double bow_displacement = 0.0;
    double bow_velocity_dir = 1.0; // manual mode: +1 down-bow, -1 up-bow

    MoveToStringPhase move_phase         = MoveToStringPhase::ADJACENT;
    Vector3d          transition_start_pos = Vector3d::Zero();

    Vector3d string_position          = Vector3d(0.9215, 0.0, 0.0);
    Matrix3d string_orientation_world = Matrix3d::Identity();
    Vector3d bowing_dir               = string_orientation_world.col(2);

    Vector3d ee_pos_desired    = string_position;
    Vector3d ee_force_desired  = Vector3d::Zero();
    Matrix3d ee_ori_desired    = string_orientation_world;
    Vector3d ee_moment_desired = Vector3d::Zero();
    Vector3d control_position  = general_task->getCurrentPosition();

    // MIDI playback state
    vector<NoteEvent> notes;
    int    note_idx              = 0;
    int    midi_bow_dir          = 1;    // +1 down-bow, -1 up-bow; alternates per note
    double bow_offset_note_start = 0.0;  // bow_displacement when current note started
    double midi_bow_speed_       = 0.0;  // computed on entry of each BOW_NOTE
    bool   bow_note_initialized  = false;

    double   silence_duration  = 0.0;
    Vector3d silence_bow_pos   = Vector3d::Zero();
    bool     silence_pos_set   = false;

    Vector3d move_to_string_target = Vector3d::Zero(); // actual target point on new string

    ControllerState state           = ControllerState::CALIBRATION;
    ControllerState post_move_state = ControllerState::MANUAL_BOWING;
    double          state_start_time = 0.0;

    // ---- Control loop ----
    runloop = true;
    SaiCommon::LoopTimer timer(control_freq, 1e6);

    cout << "Calibration: press 1-4 to save each string position, press 9 when done.\n";
    if (midi_mode) cout << "MIDI file: " << midi_file << "\n";

    while (runloop)
    {
        timer.waitForNextLoop();
        double time          = timer.elapsedSimTime();
        double time_in_state = time - state_start_time;

        robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
        robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
        robot->updateModel();

        int  current_key = (int)redis_client.getDouble(KEYBOARD_INPUT_KEY);
        bool key_once    = (current_key != 0 && prev_key == 0);
        prev_key         = current_key;

        // ================================================================
        // FSM
        // ================================================================
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
                key_pressed          = 0;
                current_string       = 0;
                target_string        = 0;
                string_position      = string_positions[0];
                string_orientation_world = string_orientations[0];
                bowing_dir           = string_orientation_world.col(2);
                bow_displacement     = 0.0;
                bow_velocity_dir     = 1.0;
                general_task->disableInternalOtg();
                state_start_time     = time;

                if (midi_mode) {
                    cout << "Calibration done. Loading MIDI...\n";
                    state = ControllerState::LOAD_MIDI;
                } else {
                    cout << "Calibration done. Starting manual bowing on string 1.\n";
                    state = ControllerState::MANUAL_BOWING;
                }
            }
            break;
        }

        // ---- MANUAL_BOWING ----
        case ControllerState::MANUAL_BOWING:
        {
            if (key_once && current_key >= 1 && current_key <= 4) {
                int requested = current_key - 1;
                if (requested != current_string) {
                    target_string = requested;

                    // Reverse early if near end of stroke in current direction
                    if (abs(bow_displacement / manual_max_disp) >= near_end_ratio)
                        bow_velocity_dir *= -1.0;

                    // Target: preserve bow_displacement on the new string
                    Vector3d new_bowing_dir = string_orientations[target_string].col(2);
                    move_to_string_target   = string_positions[target_string]
                                             + bow_displacement * new_bowing_dir;

                    transition_start_pos = general_task->getCurrentPosition();
                    move_phase = (abs(target_string - current_string) == 1)
                                 ? MoveToStringPhase::ADJACENT
                                 : MoveToStringPhase::LIFTING;

                    cout << "String change: " << current_string+1
                         << " -> " << target_string+1
                         << "  bow_disp=" << bow_displacement << "m\n";
                    post_move_state  = ControllerState::MANUAL_BOWING;
                    state            = ControllerState::MOVE_TO_STRING;
                    state_start_time = time;
                    break;
                }
            }

            // Continuous bowing with auto-reversal at ±manual_max_disp
            general_task->parametrizeForceMotionSpaces(1, Vector3d::UnitY());
            general_task->parametrizeMomentRotMotionSpaces(0);

            bow_displacement += bow_velocity_dir * manual_bow_speed / control_freq;
            if (bow_displacement >= manual_max_disp) {
                bow_displacement = manual_max_disp;
                bow_velocity_dir = -1.0;
            } else if (bow_displacement <= -manual_max_disp) {
                bow_displacement = -manual_max_disp;
                bow_velocity_dir =  1.0;
            }

            ee_pos_desired   = string_position + bow_displacement * bowing_dir;
            ee_force_desired = Vector3d(0.0, -1.0, 0.0);
            ee_ori_desired   = string_orientation_world;
            break;
        }

        // ---- LOAD_MIDI ----
        case ControllerState::LOAD_MIDI:
        {
            notes    = parseMidi(midi_file);
            note_idx = 0;
            cout << "Ready. Press 9 to start playback.\n";
            state            = ControllerState::IDLE;
            state_start_time = time;
            break;
        }

        // ---- IDLE ----
        case ControllerState::IDLE:
        {
            // Full position hold
            general_task->parametrizeForceMotionSpaces(0);
            general_task->parametrizeMomentRotMotionSpaces(0);
            ee_force_desired.setZero();

            if (key_once && current_key == 9) {
                if (notes.empty()) {
                    cout << "No notes to play.\n";
                    state = ControllerState::END;
                } else {
                    note_idx             = 0;
                    midi_bow_dir         = 1;
                    bow_note_initialized = false;
                    cout << "Starting MIDI playback (" << notes.size() << " notes).\n";
                    state            = ControllerState::SET_FRET;
                    state_start_time = time;
                }
            }
            break;
        }

        // ---- SET_FRET ----
        case ControllerState::SET_FRET:
        {
            const NoteEvent &note = notes[note_idx];

            // Publish finger target (Redis for monitoring, serial for Arduino)
            redis_client.set(FINGER_TARGET_KEY,
                             to_string(note.string_idx) + ":" + to_string(note.fret));
            serialSend(FRET_SOLENOID[note.fret]);

            bow_note_initialized = false; // BOW_NOTE will re-init on entry

            if (note.string_idx != current_string) {
                target_string = note.string_idx;
                Vector3d new_bowing_dir = string_orientations[target_string].col(2);

                // Symmetric clamp: guarantee MIN_STROKE in both directions.
                // Requires calibration at the center of the bow stroke.
                bow_displacement = max(-(BOW_AMPLITUDE - MIN_STROKE),
                                   min(  BOW_AMPLITUDE - MIN_STROKE, bow_displacement));

                // Flip direction if not enough stroke remaining in current direction
                double avail = (midi_bow_dir > 0)
                    ? (BOW_AMPLITUDE - bow_displacement)
                    : (bow_displacement + BOW_AMPLITUDE);
                if (avail < MIN_STROKE) {
                    midi_bow_dir *= -1;
                    cout << "  [bow dir flipped: not enough stroke]\n";
                }

                move_to_string_target = string_positions[target_string]
                                        + bow_displacement * new_bowing_dir;

                transition_start_pos = general_task->getCurrentPosition();
                move_phase = (abs(target_string - current_string) == 1)
                             ? MoveToStringPhase::ADJACENT
                             : MoveToStringPhase::LIFTING;

                cout << "MIDI: string change " << current_string+1
                     << " -> " << target_string+1
                     << "  bow_disp=" << bow_displacement << "m\n";
                post_move_state = ControllerState::BOW_NOTE;
                state           = ControllerState::MOVE_TO_STRING;
            } else {
                state = ControllerState::BOW_NOTE;
            }
            state_start_time = time;
            break;
        }

        // ---- MOVE_TO_STRING ----
        case ControllerState::MOVE_TO_STRING:
        {
            double lh = (post_move_state == ControllerState::MANUAL_BOWING)
                        ? manual_lift_height : LIFT_HEIGHT;

            bool done = moveToString(
                ee_pos_desired, ee_force_desired, ee_ori_desired,
                general_task,
                move_to_string_target,
                string_orientations[target_string],
                transition_start_pos,
                move_phase, lh, position_threshold, control_freq, transition_speed);

            // Safety timeout: if the target is unreachable, don't block forever
            if (!done && time_in_state > 8.0) {
                cout << "MOVE_TO_STRING timeout — target may be unreachable, proceeding\n";
                done = true;
            }

            if (done) {
                current_string           = target_string;
                string_position          = string_positions[current_string];
                string_orientation_world = string_orientations[current_string];
                bowing_dir               = string_orientation_world.col(2);
                general_task->disableInternalOtg();

                // bow_displacement is preserved — target was already string_pos + bow_disp * dir
                if (post_move_state == ControllerState::MANUAL_BOWING)
                    cout << "Now bowing on string " << current_string+1 << "\n";

                state            = post_move_state;
                state_start_time = time;
            }
            break;
        }

        // ---- BOW_NOTE ----
        case ControllerState::BOW_NOTE:
        {
            const NoteEvent &note = notes[note_idx];

            // Entry: compute bow speed for this note's duration
            if (!bow_note_initialized) {
                bow_note_initialized  = true;
                bow_offset_note_start = bow_displacement;

                double available = (midi_bow_dir > 0)
                    ? (BOW_AMPLITUDE - bow_offset_note_start)
                    : (bow_offset_note_start + BOW_AMPLITUDE);
                available = max(available, 0.01); // guard against zero stroke

                midi_bow_speed_ = min(available / note.duration, MAX_BOW_SPEED);

                cout << "Note " << note_idx+1 << "/" << notes.size()
                     << "  string=" << note.string_idx
                     << "  fret=" << note.fret
                     << "  dur=" << note.duration << "s"
                     << "  speed=" << midi_bow_speed_ << "m/s\n";
            }

            general_task->parametrizeForceMotionSpaces(1, Vector3d::UnitY());
            general_task->parametrizeMomentRotMotionSpaces(0);

            bow_displacement = max(-BOW_AMPLITUDE, min(BOW_AMPLITUDE,
                bow_offset_note_start + midi_bow_dir * midi_bow_speed_ * time_in_state));

            ee_pos_desired   = string_position + bow_displacement * bowing_dir;
            ee_force_desired = Vector3d(0.0, -1.0, 0.0);
            ee_ori_desired   = string_orientation_world;

            if (time_in_state >= note.duration) {
                midi_bow_dir        *= -1; // alternate direction for next note
                state                = ControllerState::NEXT_NOTE;
                state_start_time     = time;
            }
            break;
        }

        // ---- NEXT_NOTE ----
        case ControllerState::NEXT_NOTE:
        {
            serialSend(0); // release solenoid between notes
            note_idx++;
            if (note_idx >= (int)notes.size()) {
                cout << "MIDI playback complete.\n";
                state            = ControllerState::END;
                state_start_time = time;
                break;
            }

            double prev_end    = notes[note_idx-1].start_time + notes[note_idx-1].duration;
            double next_start  = notes[note_idx].start_time;
            double gap         = next_start - prev_end;

            if (gap > SILENCE_THRESH) {
                silence_duration = gap;
                silence_pos_set  = false;
                cout << "Silence: " << gap << "s\n";
                state = ControllerState::WAIT_SILENCE;
            } else {
                state = ControllerState::SET_FRET;
            }
            state_start_time = time;
            break;
        }

        // ---- WAIT_SILENCE ----
        case ControllerState::WAIT_SILENCE:
        {
            // Capture bow position at start of silence and hold it
            if (!silence_pos_set) {
                silence_bow_pos = general_task->getCurrentPosition();
                silence_pos_set = true;
            }

            general_task->parametrizeForceMotionSpaces(0);
            general_task->parametrizeMomentRotMotionSpaces(0);
            ee_pos_desired   = silence_bow_pos;
            ee_force_desired.setZero();

            if (time_in_state >= silence_duration) {
                state            = ControllerState::SET_FRET;
                state_start_time = time;
            }
            break;
        }

        // ---- END ----
        case ControllerState::END:
        {
            serialSend(0);
            general_task->parametrizeForceMotionSpaces(0);
            general_task->parametrizeMomentRotMotionSpaces(0);
            ee_force_desired.setZero();
            // Hold position until SIGINT
            break;
        }

        } // end switch

        // ================================================================
        // APPLY TASKS
        // ================================================================

        general_task->setGoalPosition(ee_pos_desired);
        general_task->setGoalOrientation(ee_ori_desired);
        general_task->setGoalForce(ee_force_desired);
        general_task->setGoalMoment(ee_moment_desired);

        control_position = general_task->getCurrentPosition();

        N_prec.setIdentity();
        general_task->updateTaskModel(N_prec);
        joint_task->updateTaskModel(general_task->getTaskAndPreviousNullspace());

        command_torques = general_task->computeTorques() + joint_task->computeTorques();

        redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, command_torques);
        redis_client.setEigen(CONTROL_POSITION_KEY, control_position);
    }

    timer.stop();
    cout << "\nSimulation loop timer stats:\n";
    timer.printInfoPostRun();
    redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, VectorXd::Zero(dof));
    serialSend(0);
    if (serial_fd >= 0) close(serial_fd);
    return 0;
}
