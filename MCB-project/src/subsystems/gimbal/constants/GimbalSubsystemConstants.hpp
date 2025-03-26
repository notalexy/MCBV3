/*
Constants file for all robots subsystem constants
*/
namespace subsystems {

constexpr static float PI_CONST = 3.14159;
constexpr static int YAW_MOTOR_MAX_VOLTAGE = 24000;  // Should be the voltage of the battery. Unless the motor maxes out below that.
static constexpr float dt = 0.002f;
// for sysid
static constexpr int YAW_DIST_RANGE = 18000;
static constexpr int PITCH_DIST_RANGE = 0;

#if defined(HERO)
constexpr static int YAW_MOTOR_MAX_SPEED = 1000;  // TODO: Make this value relevent
                                                  // //TODO: Check the datasheets

// standard looks down 17 degrees, 15 is safe
static constexpr float MAX_PITCH_UP = PI_CONST / 180 * 15;
// looks up 20, 18 is safe
static constexpr float MAX_PITCH_DOWN = PI_CONST / 180 * 18;

static constexpr float YAW_OFFSET = 3 * PI_CONST / 4;

static constexpr float PITCH_OFFSET = -0.48 * PI_CONST;  // to make gimbal horizontal when told to go to 0

static constexpr float YAW_TOTAL_RATIO = 32319.0f/748.0f;  // unitless, ratio of encoder counts to degrees of rotation


#elif defined(SENTRY)
constexpr static int YAW_MOTOR_MAX_SPEED = 1000;  // TODO: Make this value relevent
                                                  // //TODO: Check the datasheets

// standard looks down 17 degrees, 15 is safe
static constexpr float MAX_PITCH_UP = PI_CONST / 180 * 15;
// looks up 20, 18 is safe
static constexpr float MAX_PITCH_DOWN = PI_CONST / 180 * 18;

static constexpr float YAW_OFFSET = 3 * PI_CONST / 4;

static constexpr float PITCH_OFFSET = -0.48 * PI_CONST;  // to make gimbal horizontal when told to go to 0

static constexpr float YAW_TOTAL_RATIO = 32319.0f/748.0f;  // unitless, ratio of encoder counts to degrees of rotation


#elif defined(INFANTRY)
constexpr static int YAW_MOTOR_MAX_SPEED = 1000;  // TODO: Make this value relevent
                                                  // //TODO: Check the datasheets

// standard looks down 17 degrees, 15 is safe
static constexpr float MAX_PITCH_UP = PI_CONST / 180 * 15;
// looks up 20, 18 is safe
static constexpr float MAX_PITCH_DOWN = PI_CONST / 180 * 18;

static constexpr float YAW_OFFSET = 0;// 3 * PI_CONST / 4;

static constexpr float PITCH_OFFSET = -1 * PI_CONST;  // to make gimbal horizontal when told to go to 0

static constexpr float YAW_TOTAL_RATIO = 32319.0f/748.0f;  // unitless, ratio of encoder counts to degrees of rotation


#else
constexpr static int YAW_MOTOR_MAX_SPEED = 1000;  // TODO: Make this value relevent
                                                  // //TODO: Check the datasheets

// standard looks down 17 degrees, 15 is safe
static constexpr float MAX_PITCH_UP = PI_CONST / 180 * 15;
// looks up 20, 18 is safe
static constexpr float MAX_PITCH_DOWN = PI_CONST / 180 * 18;

static constexpr float YAW_OFFSET = 3 * PI_CONST / 4;

static constexpr float YAW_TOTAL_RATIO = 1.0f;  // unitless, ratio of encoder counts to degrees of rotation

static constexpr float PITCH_OFFSET = -0.48 * PI_CONST;  // to make gimbal horizontal when told to go to 0

#endif

}  // namespace subsystems