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
	return FindPath(from, to, outPath, true);
}

bool NavigationMesh::FindPath(const Vector3& from, const Vector3& to, NavigationPath& outPath, bool useFunnel) {
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

	// A*: collect triangle path
	while (!openSet.empty()) {
		int current = popBest();
		if (current == endIdx) {
			std::vector<int> triPath;
			for (int n = endIdx; n != -1; n = parent[n]) {
				triPath.push_back(n);
			}
			std::reverse(triPath.begin(), triPath.end());
			
			// DEBUG: Print triPath size
			// std::cout << "FindPath TriPath Size: " << triPath.size() << " UseFunnel: " << useFunnel << std::endl;

			if (useFunnel) {
				// Funnel smoothing on triangle strip
				// Build portals (shared edges)
				struct Portal { Vector3 left; Vector3 right; };
				std::vector<Portal> portals;
				portals.reserve(triPath.size());

				Vector3 startPos = from;
				Vector3 endPos = to;

				// helper to find shared edge
				auto sharedEdge = [&](int a, int b, Vector3& v0, Vector3& v1)->bool {
					int shared[2] = { -1, -1 };
					int count = 0;
					for (int i = 0; i < 3; ++i) {
						int idxA = allTris[a].indices[i];
						for (int j = 0; j < 3; ++j) {
							if (idxA == allTris[b].indices[j]) {
								shared[count++] = idxA;
								if (count == 2) break;
							}
						}
						if (count == 2) break;
					}
					if (count < 2) return false;
					v0 = allVerts[shared[0]];
					v1 = allVerts[shared[1]];
					return true;
				};

				// Build portal list
				portals.push_back({ startPos, startPos });
				for (size_t i = 0; i + 1 < triPath.size(); ++i) {
					Vector3 a, b;
					if (sharedEdge(triPath[i], triPath[i + 1], a, b)) {
						// order edge so that left/right is consistent w.r.t movement direction
						Vector3 dir = allTris[triPath.back()].centroid - startPos;
						Vector3 edgeDir = b - a;
						Vector3 cross = Vector::Cross(edgeDir, dir);
						if (cross.y < 0.0f) {
							portals.push_back({ b, a });
						}
						else {
							portals.push_back({ a, b });
						}
					}
				}
				portals.push_back({ endPos, endPos });

				// Funnel algorithm
				std::vector<Vector3> smooth;
				size_t apexIndex = 0, leftIndex = 0, rightIndex = 0;
				Vector3 apex = portals[0].left;
				Vector3 left = portals[0].left;
				Vector3 right = portals[0].right;
				auto triArea2 = [](const Vector3& a, const Vector3& b, const Vector3& c) {
					return (b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x);
				};
				auto samePoint = [](const Vector3& a, const Vector3& b) {
					return Vector::Length(a - b) < 1e-4f;
				};
				for (size_t i = 1; i < portals.size(); ++i) {
					Vector3 pLeft = portals[i].left;
					Vector3 pRight = portals[i].right;
					// update right
					if (triArea2(apex, right, pRight) <= 0.0f) {
						if (samePoint(apex, right) || triArea2(apex, left, pRight) > 0.0f) {
							right = pRight;
							rightIndex = i;
						}
						else {
							smooth.push_back(left);
							apex = left;
							apexIndex = leftIndex;
							left = apex;
							right = apex;
							leftIndex = apexIndex;
							rightIndex = apexIndex;
							i = apexIndex;
							continue;
						}
					}
					// update left
					if (triArea2(apex, left, pLeft) >= 0.0f) {
						if (samePoint(apex, left) || triArea2(apex, right, pLeft) < 0.0f) {
							left = pLeft;
							leftIndex = i;
						}
						else {
							smooth.push_back(right);
							apex = right;
							apexIndex = rightIndex;
							left = apex;
							right = apex;
							leftIndex = apexIndex;
							rightIndex = apexIndex;
							i = apexIndex;
							continue;
						}
					}
				}
				smooth.push_back(endPos);

				for (const auto& wp : smooth) {
					outPath.PushWaypoint(wp);
				}
			}
			else {
				// No Funnel: just use centroids
				outPath.PushWaypoint(from);
				for (int i : triPath) {
					outPath.PushWaypoint(allTris[i].centroid);
				}
				outPath.PushWaypoint(to);
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
