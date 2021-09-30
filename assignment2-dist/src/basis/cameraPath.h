#pragma once

#include "base/Math.hpp"
#include "curve.h"
#include "3d/Mesh.hpp"

#include <memory>
#include <vector>
#include <array>

namespace FW
{

	class cameraPath
	{
	private:
		Mat4f GetOrientation(float t);
		Mat4f GetTranslation(float t);

	public:
		Mat4f GetWorldToCam(float t);
		void  Draw(float t, GLContext* gl, Mat4f projection);

		Curve positionPath;
		std::vector<std::array<Vec4f,4>> orientationPoints;
		std::unique_ptr<Mesh<VertexPNTC>> mesh;

		bool loaded = false;
		bool orientationMode = true;
	};

}
