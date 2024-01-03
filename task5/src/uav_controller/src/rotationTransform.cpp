#include "rotationTransform.hpp"

EulerAngles			quaternionToEuler(const double &w, const double &x, const double &y, const double &z)
{
	EulerAngles result;
	result.roll = std::atan2(2 * (w * x + y * z),
					1 - 2 * (q.x * q.x + q.y * q.y));

	double sinp = 2 * (q.w * q.y - q.z * q.x);
	if (std::abs(sinp) >= 1)
        result.pitch = std::copysign(M_PI / 2, sinp);
    else
        result.pitch = std::asin(sinp);

	result.yaw = std::atan2(2 * (w * z + x * y),
				 1 - 2 * (y * y + z * z));
				 
	return result;
}

Quaternion			eulerToQuat(const double &roll, const double &pitch, const double &yaw)
{
	Quaternion result;
	double cosYaw = std::cos(yaw / 2);
    double sinYaw = std::sin(yaw / 2);
    double cosPitch = std::cos(pitch / 2);
    double sinPitch = std::sin(pitch / 2);
    double cosRoll = std::cos(roll / 2);
    double sinRoll = std::sin(roll / 2);

	result.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
    result.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    result.y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
    result.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
	
	return result;
}	