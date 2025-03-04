#include <util/Orientation2d.hpp>
#include <util/Vector2d.hpp>

class Pose2d : public Vector2d, public Orientation2d {
public:
    Pose2d(float x, float y, float r) : Vector2d(x, y), Orientation2d(r) {}

    ~Pose2d() {}

    Pose2d() {}

    Pose2d(float vec[3]) : Vector2d(vec), Orientation2d(vec[2]) {}

    Pose2d(const Vector2d& vec) : Vector2d(vec), Orientation2d(0.0f) {}

    Pose2d(const Pose2d& other) : Vector2d(other.x, other.y), Orientation2d(other.rotation) {}

    Vector2d& vec() { return *this; }

    Orientation2d& orientation() { return *this; }

    Pose2d rotate(float amt) { return Pose2d(magnitude() * std::cos(amt + angle()), magnitude() * std::sin(amt + angle()), rotation); }

    Pose2d addRotation(float rot) { return Pose2d(x, y, rotation + rot); }

    operator float*() { return new float[3]{x, y, rotation}; }

    // Overload + operator (Pose2d addition)
    Pose2d operator+(const Vector2d& other) const { return Pose2d(x + other.getX(), y + other.getY(), rotation); }

    // Overload - operator (Pose2d subtraction)
    Pose2d operator-(const Vector2d& other) const { return Pose2d(x - other.getX(), y - other.getY(), rotation); }

    // Overload * operator (scalar multiplication)
    Pose2d operator*(float scalar) const { return Pose2d(x * scalar, y * scalar, rotation * scalar); }

    // Overload * operator (dot product)
    Pose2d operator*(const Pose2d& other) const { return Pose2d(x * other.x, y * other.y, rotation * other.rotation); }

    // Overload += operator (Pose2d addition and assignment)
    Pose2d& operator+=(const Vector2d& other) {
        x += other.getX();
        y += other.getY();
        return *this;
    }

    // Overload -= operator (Pose2d subtraction and assignment)
    Pose2d& operator-=(const Vector2d& other) {
        x -= other.getX();
        y -= other.getY();
        return *this;
    }

    // Overload *= operator (scalar multiplication and assignment)
    Pose2d& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        rotation *= scalar;
        return *this;
    }

    // Overload == operator (Pose2d equality check)
    bool operator==(const Pose2d& other) const {
        constexpr float EPSILON = 1e-4;  // Threshold for floating-point comparison
        return (std::fabs(x - other.x) < EPSILON) && (std::fabs(y - other.y) < EPSILON) && (std::fabs(rotation - other.rotation) < EPSILON);
    }

    // Overload = operator (Pose2d assignment)
    Pose2d& operator=(const Pose2d& other) {
        if (this != &other) {  // Prevent self-assignment
            x = other.x;
            y = other.y;
            rotation = other.rotation;
        }
        return *this;
    }

    // Overload = operator (Pose2d assignment)
    Pose2d& operator=(const Vector2d& other) {
        if (this != &other) {  // Prevent self-assignment
            x = other.getX();
            y = other.getY();
        }
        return *this;
    }

    void print() const {
        std::cout << "Pose2d(" << x << ", " << y << ", " << rotation << ")\n";
    }
};