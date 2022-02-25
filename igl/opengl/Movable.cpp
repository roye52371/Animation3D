#include "Movable.h"
#include <iostream>
Movable::Movable()
{
	Tout = Eigen::Affine3d::Identity();
	Tin = Eigen::Affine3d::Identity();
}

Movable::Movable(const Movable& mov)
{
	Tout = mov.Tout;
	Tin = mov.Tin;
}

Eigen::Matrix4f Movable::MakeTransScale()
{
	return (Tout.matrix() * Tin.matrix()).cast<float>();
}

Eigen::Matrix4d Movable::MakeTransScaled()
{
	return (Tout.matrix() * Tin.matrix());
}

Eigen::Matrix4d Movable::MakeTransd()
{
	Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
	mat.col(3) << Tin.translation(), 1;

	return (Tout.matrix() * mat);
}

void Movable::MyTranslate(Eigen::Vector3d amt, bool preRotation)
{

	if (preRotation)
		Tout.pretranslate(amt);
	else
		Tout.translate(amt);
}
//angle in radians
void Movable::MyRotate(Eigen::Vector3d rotAxis, double angle)
{
	Tout.rotate(Eigen::AngleAxisd(angle, rotAxis.normalized()));
}

void Movable::MyRotate(const Eigen::Matrix3d& rot)
{
	Tout.rotate(rot);
}

//Ass 3 comment
void Movable::MyRotate(Eigen::Vector3d rotAxis, double angle, bool zAxis)
{
	Eigen::Matrix3d mat = Tout.rotation().matrix();
	mat.transposeInPlace();
	if (zAxis)
		Tout.rotate(Eigen::AngleAxisd(angle, mat * rotAxis));
	else
		Tout.rotate(Eigen::AngleAxisd(angle, rotAxis));
}
//end Ass3 comment


void Movable::MyScale(Eigen::Vector3d amt)
{
	Tin.scale(amt);
}

void Movable::SetCenterOfRotation(Eigen::Vector3d amt)
{
	Tout.pretranslate(amt);
	Tin.pretranslate(-amt);

}

void Movable::MyTranslateInSystem(Eigen::Matrix3d rot, Eigen::Vector3d amt)
{
	Tout.pretranslate(rot.transpose() * amt);
}

void Movable::RotateInSystem(Eigen::Vector3d rotAxis, double angle)
{
	Tout.rotate(Eigen::AngleAxisd(angle, Tout.rotation().transpose() *
		rotAxis.normalized())); //we will multiply the vector with the rotate matrix

}

// --------------------------------------------------Project--------------------------------------------------------------------

Eigen::Quaterniond Movable::GetRotationQ()
{
	return Eigen::Quaterniond(Tout.rotation());
}

void Movable::RotateInSystem(Eigen::Matrix4d Mat, Eigen::Vector3d rotAxis, double angle)
{
	Eigen::Vector3d v = Mat.transpose().block<3, 3>(0, 0) * rotAxis; //transpose instead of inverse
	MyRotate(v.normalized(), angle);
}
// ----------------------------------------------------Project-----------------------------------------------------------------------

//Project comment

void Movable::MyRotate(const Eigen::Quaterniond rot)
{
	Tout.rotate(rot);
}

void Movable::RotateInSystem(const Eigen::Matrix3d& mat, const Eigen::Quaterniond rot)
{
	Eigen::Quaterniond q = Eigen::Quaterniond(mat);
	MyRotate((q * rot.conjugate()) * q.conjugate());
}

//maybe to delete this
void Movable::SetTranslation(Eigen::Vector3d position)
{
	Eigen::Vector3d old_position = (Tout * Tin).matrix().block(0, 3, 3, 1);
	Tout.pretranslate(-old_position + position);
}

void Movable::LookAt(Eigen::Vector3d forward)
{
	Eigen::Vector3d pos = (Tout * Tin).matrix().block(0, 3, 3, 1);
	Tin = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
	Tout = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
	Tout.pretranslate(pos);

	Eigen::Vector3d right = Eigen::Vector3d(0, 1, 0).cross(forward);
	Eigen::Vector3d up = forward.cross(right);

	Eigen::Matrix3d R;

	R << right[0], up[0], forward[0],
		right[1], up[1], forward[1],
		right[2], up[2], forward[2];

	double angleX = 0;
	double angleY = 0;
	double angleZ = 0;

	if (R.row(0)[2] < 1) {
		if (R.row(0)[2] > -1) {
			angleY = asinf(R.row(0)[2]);
			angleX = atan2f(-R.row(1)[2], R.row(2)[2]);
			angleZ = atan2f(-R.row(0)[1], R.row(0)[0]);
		}
		else {
			angleY = -M_PI / 2;
			angleX = -atan2f(-R.row(1)[0], R.row(1)[1]);
			angleZ = 0;
		}
	}
	else {
		angleY = M_PI / 2;
		angleX = atan2f(R.row(1)[0], R.row(1)[1]);
		angleZ = 0;

	}

	Tout.rotate(Eigen::AngleAxisd(angleX, Eigen::Vector3d(1, 0, 0)));
	Tout.rotate(Eigen::AngleAxisd(angleY, Eigen::Vector3d(0, 1, 0)));
	Tout.rotate(Eigen::AngleAxisd(angleZ, Eigen::Vector3d(0, 0, 1)));

}
//end comment maybe to delete this

void Movable::resetTranslation()
{
	Tout = Eigen::Affine3d::Identity();
	Tin = Eigen::Affine3d::Identity();
}
//end Project comment
























