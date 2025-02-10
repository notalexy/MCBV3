
#if defined(hero)
#include "hero/HeroControl.hpp"
using RobotControl = HeroControl;

#elif defined(sentry)
#include "sentry/SentryControl.hpp"
using RobotControl = SentryControl;

#else //for standard
#include "standard/StandardControl.hpp"
using RobotControl = StandardControl;


#endif