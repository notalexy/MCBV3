

namespace subsystems
{
class YawController
{
public:
    YawController();
    //~YawController();
    float calculate(float currentPosition, float currentVelocity, float currentDrivetrainVelocity, float targetPosition, float inputVelocity, float deltaT);

    void clearBuildup() { buildup = 0; };

private:
    // START getters and setters
    float buildup = 0;
    float pastTargetVelocity = 0;
    float pastOutput = 0;
    int signum(float num) { return (num > 0) ? 1 : ((num < 0) ? -1 : 0); }

public:
    // Physical constants
 };
}  // namespace subsystems
