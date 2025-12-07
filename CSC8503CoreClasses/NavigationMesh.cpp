#include "NavigationMesh.h"
#include "Assets.h"
#include "Maths.h"
#include <fstream>
#include <limits>
#include <algorithm>
using namespace NCL;
using namespace CSC8503;
using namespace std;

NavigationMesh::NavigationMesh()
{
}

NavigationMesh::NavigationMesh(const std::string&filename)
{
	ifstream file(Assets::DATADIR + filename);

	int numVertices = 0;
	int numIndices	= 0;

	file >> numVertices;
	file >> numIndices;

	for (int i = 0; i < numVertices; ++i) {
		Vector3 vert;
		file >> vert.x;
		file >> vert.y;
		file >> vert.z;

		allVerts.emplace_back(vert);
	}

	allTris.resize(numIndices / 3);

	for (int i = 0; i < allTris.size(); ++i) {
		NavTri* tri = &allTris[i];
		file >> tri->indices[0];
		file >> tri->indices[1];
		file >> tri->indices[2];

		tri->centroid = allVerts[tri->indices[0]] +
			allVerts[tri->indices[1]] +
			allVerts[tri->indices[2]];

		tri->centroid = allTris[i].centroid / 3.0f;

		tri->triPlane = Plane::PlaneFromTri(allVerts[tri->indices[0]],
			allVerts[tri->indices[1]],
			allVerts[tri->indices[2]]);

		tri->area = Maths::AreaofTri3D(allVerts[tri->indices[0]], allVerts[tri->indices[1]], allVerts[tri->indices[2]]);
	}
	for (int i = 0; i < allTris.size(); ++i) {
		NavTri* tri = &allTris[i];
		for (int j = 0; j < 3; ++j) {
			int index = 0;
			file >> index;
			if (index != -1) {
				tri->neighbours[j] = &allTris[index];
			}
		}
	}
}

NavigationMesh::~NavigationMesh()
{
}

bool NavigationMesh::FindPath(const Vector3& from, const Vector3& to, NavigationPath& outPath) {
	outPath.Clear();

	const NavTri* startTri	= GetTriForPosition(from);
	const NavTri* endTri	= GetTriForPosition(to);

	if (!startTri || !endTri || allTris.empty()) {
		return false;
	}

	const int triCount = static_cast<int>(allTris.size());
	auto triIndex = [&](const NavTri* t) -> int {
		return static_cast<int>(t - &allTris[0]);
	};

	const int startIdx = triIndex(startTri);
	const int endIdx   = triIndex(endTri);

	std::vector<float> g(triCount, std::numeric_limits<float>::infinity());
	std::vector<float> f(triCount, std::numeric_limits<float>::infinity());
	std::vector<int> parent(triCount, -1);
	std::vector<int> openSet;
	std::vector<bool> inOpen(triCount, false);
	std::vector<bool> closed(triCount, false);

	auto heuristic = [&](int idx) {
		return Vector::Length(allTris[idx].centroid - allTris[endIdx].centroid);
	};

	g[startIdx] = 0.0f;
	f[startIdx] = heuristic(startIdx);
	openSet.push_back(startIdx);
	inOpen[startIdx] = true;

	auto popBest = [&]() -> int {
		int best = openSet.front();
		size_t bestPos = 0;
		for (size_t i = 1; i < openSet.size(); ++i) {
			int idx = openSet[i];
			if (f[idx] < f[best]) {
				best = idx;
				bestPos = i;
			}
		}
		openSet.erase(openSet.begin() + bestPos);
		inOpen[best] = false;
		return best;
	};

	while (!openSet.empty()) {
		int current = popBest();
		if (current == endIdx) {
			std::vector<int> pathIdx;
			for (int n = endIdx; n != -1; n = parent[n]) {
				pathIdx.push_back(n);
			}
			for (int idx : pathIdx) {
				outPath.PushWaypoint(allTris[idx].centroid);
			}
			return true;
		}
		closed[current] = true;

		for (int i = 0; i < 3; ++i) {
			NavTri* neigh = allTris[current].neighbours[i];
			if (!neigh) {
				continue;
			}
			int ni = triIndex(neigh);
			if (closed[ni]) {
				continue;
			}
			float edgeCost = Vector::Length(allTris[current].centroid - neigh->centroid);
			float tentativeG = g[current] + edgeCost;
			if (tentativeG < g[ni]) {
				parent[ni] = current;
				g[ni] = tentativeG;
				f[ni] = tentativeG + heuristic(ni);
				if (!inOpen[ni]) {
					openSet.push_back(ni);
					inOpen[ni] = true;
				}
			}
		}
	}
	return false;
}

/*
If you have triangles on top of triangles in a full 3D environment, you'll need to change this slightly,
as it is currently ignoring height. You might find tri/plane raycasting is handy.
*/

const NavigationMesh::NavTri* NavigationMesh::GetTriForPosition(const Vector3& pos) const {
	for (const NavTri& t : allTris) {
		Vector3 planePoint = t.triPlane.ProjectPointOntoPlane(pos);

		float ta = Maths::AreaofTri3D(allVerts[t.indices[0]], allVerts[t.indices[1]], planePoint);
		float tb = Maths::AreaofTri3D(allVerts[t.indices[1]], allVerts[t.indices[2]], planePoint);
		float tc = Maths::AreaofTri3D(allVerts[t.indices[2]], allVerts[t.indices[0]], planePoint);

		float areaSum = ta + tb + tc;

		if (abs(areaSum - t.area)  > 0.001f) { //floating points are annoying! Are we more or less inside the triangle?
			continue;
		}
		return &t;
	}
	return nullptr;
}
