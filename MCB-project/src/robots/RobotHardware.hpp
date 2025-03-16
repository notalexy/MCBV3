

//gaslights the compiler to think that RobotHardware is one of these three based on the defines
#if defined(HERO)
#include "robots/hero/HeroHardware.hpp"
using RobotHardware = robots::HeroHardware;

#elif defined(SENTRY)
#include "robots/sentry/SentryHardware.hpp"
using RobotHardware = robots::SentryHardware;

#elif defined(INFANTRY)
#include "robots/infantry/InfantryHardware.hpp"
using RobotHardware = robots::InfantryHardware;

#else //for standard
#include "robots/infantry/InfantryHardware.hpp"
using RobotHardware = robots::InfantryHardware;

#endif