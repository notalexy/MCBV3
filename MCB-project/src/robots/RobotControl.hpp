#if defined(standard)
#include "standard/StandardControl.hpp"
using RobotControl = StandardControl;

#elif defined(hero)
#include "hero/HeroControl.hpp"
using RobotControl = HeroControl;

#elif defined(sentry)
#include "sentry/SentryControl.hpp"
using RobotControl = SentryControl;

#endif