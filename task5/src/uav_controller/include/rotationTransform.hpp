#ifndef rotationTransform_HPP
#define rotationTransform_HPP

struct EulerAngles
{
	double roll;
	double pitch;
	double yaw;
};

struct Quaternion
{
	double w;
	double x;
	double y;
	double z;
};

EulerAngles			quaternionToEuler(const double &w, const double &x, const double &y, const double &z);
Quaternion			eulerToQuat(const double &roll, const double &pitch, const double &yaw);		


#endif