#pragma once

#include "gui/Window.hpp"
#include "gui/CommonControls.hpp"
#include "3d/CameraControls.hpp"
#include "gpu/Buffer.hpp"

#include <map>

namespace FW {

// This class converts a regular mesh into a form suitable for performing subdivision.
// In particular, it computes neighbor information that determines, for each triangle,
// which other triangles are adjacent to it in the mesh.
struct MeshWithConnectivity
{
	void fromMesh			(const Mesh<VertexPNC>& mesh);
	void toMesh				(Mesh<VertexPNC>& dest);

	void LoopSubdivision	();

	void computeConnectivity();

	// Runs a debug version of the subdivision pass
	std::vector<Vec3f> debugHighlight(Vec2f mousePos, Mat4f worldToClip);

	// vertex data
	std::vector<Vec3f>	positions;
	std::vector<Vec3f>	normals;
	std::vector<Vec3f>	colors;

	// Debug highlight data. Contains indices for vertices that need to be highlighted
	std::vector<int> highlightIndices;
	bool debugPass = false; // Whether our current LoopSubdivision call is a debug call
	Vec2i debugVertexIdx;

	// index data
	// (for each triangle, a triplet of indices into the above vertex data arrays)
	std::vector<Vec3i>	indices;

	// connectivity information
	std::vector<Vec3i>	neighborTris;
	std::vector<Vec3i>	neighborEdges;

};

} // namespace FW
