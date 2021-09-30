#include "cameraPath.h"
#include <iostream>
#include <array>

using namespace FW;


Mat4f FW::cameraPath::GetOrientation(float t)
{
	if (orientationMode)
	{
		int count = orientationPoints.size();
		int start = int(t * count);

		std::array<Vec4f, 4> controlPoints = orientationPoints[start % orientationPoints.size()];

		// YOUR CODE HERE (extra)
		// Use the De Casteljau construction with spherical interpolation (slerp) to interpolate between the orientation control point
		// quaternions in the points array, and convert the interpolated quaternion to an orientation matrix.

		return Mat4f();
	}
	else
	{
		Mat4f orientation;
		int i = int(t * positionPath.size());

		orientation.setCol(0, -Vec4f(positionPath[i].B, 0));
		orientation.setCol(1, -Vec4f(positionPath[i].N, 0));
		orientation.setCol(2, -Vec4f(positionPath[i].T, .0f));
		return orientation.transposed();
	}
}

Mat4f FW::cameraPath::GetTranslation(float t)
{
	int i = int(t * positionPath.size());

	Vec3f pos = positionPath[i].V;
	return Mat4f::translate(-pos);
}

Mat4f FW::cameraPath::GetWorldToCam(float t)
{
	return GetOrientation(t) * GetTranslation(t);
}

void FW::cameraPath::Draw(float t, GLContext* gl, Mat4f projection)
{
	mesh->draw(gl, GetWorldToCam(t), projection);
	glUseProgram(0);
}
