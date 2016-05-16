#include "stdafx.h"
#include "lighttree.h"
#include <queue>

PointLightTree::PointLightTree(const vector<PointLightNodeData> &d) {
	nNodes = d.size();
	nextFreeNode = 1;
	nodes = AllocAligned<PointLightNode>(nNodes * 4);
	nodeData = AllocAligned<PointLightNodeData>(nNodes * 4);
	vector<const PointLightNodeData *> buildNodes(nNodes, NULL);
	for (uint32_t i = 0; i < nNodes; ++i)
		buildNodes[i] = &d[i];
	// Begin the KdTree building process
	recursiveBuild(0, 0, nNodes, &buildNodes[0]);

	// Set parameters
	MAX_CUT_SIZE = 300;
	MAX_ERROR_RATIO = 0.01f;

	count = 0.f;
	avgCutSize = 0.f;
}


void PointLightTree::recursiveBuild(uint32_t nodeNum, int start, int end,
		const PointLightNodeData **buildNodes) {
	// Create leaf node of kd-tree if we've reached the bottom
	if (start + 1 == end) {
		nodes[nodeNum].initLeaf();
		nodeData[nodeNum] = *buildNodes[start];
		nodeData[nodeNum].bound = BBox(buildNodes[start]->lightPos);
		return;
	}

	// Choose split direction and partition data

	// Compute bounds of data from _start_ to _end_
	BBox bound;
	
	// Choose representative light from all point lights in the cluster
	// Alternative: use child's representative light
	vector<float> lightIntensity(end - start, 0.f);
	for (int i = start; i < end; ++i) {
		bound = Union(bound, buildNodes[i]->lightPos);
		lightIntensity[i - start] = buildNodes[i]->Intensity.y();
	}
	Distribution1D lightDistribution(&lightIntensity[0], end - start);
	float lightPdf;
	int ln = lightDistribution.SampleDiscrete(rng.RandomFloat(), &lightPdf);
	nodeData[nodeNum].lightPos = buildNodes[start + ln]->lightPos;
	nodeData[nodeNum].lightN = buildNodes[start + ln]->lightN;
	nodeData[nodeNum].rayEpsilon = buildNodes[start + ln]->rayEpsilon;

	int splitAxis = bound.MaximumExtent();
	int splitPos = (start + end) / 2;
	std::nth_element(&buildNodes[start], &buildNodes[splitPos],
		&buildNodes[end], ComparePointLightNode(splitAxis));

	// Allocate kd-tree node and continue recursively
	nodes[nodeNum].init(buildNodes[splitPos]->lightPos[splitAxis], splitAxis);
	nodeData[nodeNum].bound = bound;
	nodeData[nodeNum].Intensity = Spectrum(0.f);

	if (start < splitPos) {
		nodes[nodeNum].hasLeftChild = 1;
		uint32_t childNum = nextFreeNode++;
		recursiveBuild(childNum, start, splitPos, buildNodes);
		//nodeData[nodeNum].lightPos = nodeData[childNum].lightPos;
		nodeData[nodeNum].Intensity = nodeData[childNum].Intensity;
	}
	if (splitPos < end) {
		nodes[nodeNum].rightChild = nextFreeNode++;
		uint32_t childNum = nodes[nodeNum].rightChild;
		recursiveBuild(nodes[nodeNum].rightChild, splitPos,
			end, buildNodes);
		if (nodeData[nodeNum].Intensity.y() < 1e-6) {
			//nodeData[nodeNum].lightPos = nodeData[childNum].lightPos;
			nodeData[nodeNum].Intensity = nodeData[childNum].Intensity;
		}
		else {
			// Choose representative light with probability proportional to light intensity
			/*
			float intensity_ratio = nodeData[nodeNum].Intensity.y() /
				(nodeData[nodeNum].Intensity.y() + nodeData[childNum].Intensity.y());
			if (rng.RandomFloat() > intensity_ratio) {
				nodeData[nodeNum].lightPos = nodeData[childNum].lightPos;
			}
			*/
			nodeData[nodeNum].Intensity += nodeData[childNum].Intensity;
		}
	}
}


Spectrum EstimateNodeIllumination(const Scene *scene,
		const Renderer *renderer, MemoryArena &arena,
		const RayDifferential &ray, const Intersection &isect,
		RNG &rng, const Point &p, const Normal &n, const Vector &wo,
		BSDF *bsdf, float gLimit, float rrThreshold, const PointLightNodeData &vl) {
	float d2 = DistanceSquared(p, vl.lightPos);
	Vector wi = Normalize(vl.lightPos - p);
	float G = AbsDot(wi, n) * AbsDot(wi, vl.lightN) / d2;
	//float G = AbsDot(wi, n) * ClampDot(-wi, vl.lightN) / d2;
	G = min(G, gLimit);
	Spectrum f = bsdf->f(wo, wi);
	if (G == 0.f || f.IsBlack()) return Spectrum(0.f);
	Spectrum Llight = f * G * vl.Intensity;
	RayDifferential connectRay(p, wi, ray, isect.rayEpsilon,
		sqrtf(d2) * (1.f - vl.rayEpsilon));
	Llight *= renderer->Transmittance(scene, connectRay, NULL, rng, arena);

	// Possibly skip virtual light shadow ray with Russian roulette
	/*
	if (Llight.y() < rrThreshold) {
		float continueProbability = .1f;
		if (rng.RandomFloat() > continueProbability)
			return Spectrum(0.f);
		Llight /= continueProbability;
	}
	*/

	// Add contribution from _VirtualLight_ _vl_
	if (!scene->IntersectP(connectRay))
		return Llight;
	else
		return Spectrum(0.f);
}


Spectrum ErrorBound(const Point &p, const Normal &n, const Vector &wo,
		BSDF *bsdf, const PointLightNodeData &vl) {
	float errBound = 1.f;

	// distance term
	const BBox &bound = vl.bound;
	bool inBBox = true;
	float dist2 = 0.f;
	for (int i = 0; i < 3; i++) {
		if (p[i] < bound.pMin[i]) {
			dist2 += (p[i] - bound.pMin[i]) * (p[i] - bound.pMin[i]);
			inBBox = false;
		}
		else if (p[i] > bound.pMax[i]) {
			dist2 += (p[i] - bound.pMax[i]) * (p[i] - bound.pMax[i]);
			inBBox = false;
		}
	}

	if (inBBox || dist2 < 1e-5f) return Spectrum(1e8f);
	errBound /= dist2;
	
	// cosine term: 1

	// brdf term: Lambertian
	//Vector wi = Normalize(vl.lightPos - p);
	Spectrum brdf = bsdf->f(wo, wo);

	Spectrum res = vl.Intensity * brdf * errBound;
	return res;
}


Spectrum PointLightTree::refineLightcuts(const Scene *scene,
		const Renderer *renderer, MemoryArena &arena,
		const RayDifferential &ray, const Intersection &isect,
		RNG &rng, const Point &p, const Normal &n, const Vector &wo,
		BSDF *bsdf, float gLimit, float rrThreshold) {
	Spectrum contrib = EstimateNodeIllumination(scene, renderer, arena, ray, isect,
		rng, p, n, wo, bsdf, gLimit, rrThreshold, nodeData[0]);
	
	std::priority_queue<CutNodeData> q;
	Spectrum err = ErrorBound(p, n, wo, bsdf, nodeData[0]);
	Spectrum L = contrib;
	q.push(CutNodeData(err, contrib, 0));
	int nLeafNodes = 0;
	while (!q.empty() && q.size() + nLeafNodes < MAX_CUT_SIZE) {
		CutNodeData d = q.top();
		err = d.errBound;
		int now = d.lightIdx;
		if (err.x(0) < L.x(0) * MAX_ERROR_RATIO && err.x(1) < L.x(1) * MAX_ERROR_RATIO &&
				err.x(2) < L.x(2) * MAX_ERROR_RATIO && d.errBoundValue < L.y() * MAX_ERROR_RATIO) {
			//Warning("should not be here!\n");
			//Warning("vl = (%.6f, %.6f, %.6f)\n", nodeData[now].Intensity.x(0),
			//	nodeData[now].Intensity.x(1), nodeData[now].Intensity.x(2));
			//Warning("err = (%.6f, %.6f, %.6f)\n", err.x(0), err.x(1), err.x(2));
			//Log("cut size = %d, error = (%.6f, %.6f, %.6f)\n", q.size() + nLeafNodes,
			//	err.x(0), err.x(1), err.x(2));
			break;
		}
		else {
			q.pop();
			if (nodes[now].splitAxis == 3) {
				// leaf
				nLeafNodes++;
				continue;
			}
			else {
				L = L - d.contrib;
				int leftChild = now + 1;
				int rightChild = nodes[now].rightChild;

				contrib = EstimateNodeIllumination(scene, renderer, arena, ray, isect,
					rng, p, n, wo, bsdf, gLimit, rrThreshold, nodeData[leftChild]);
				L += contrib;
				err = ErrorBound(p, n, wo, bsdf, nodeData[leftChild]);
				q.push(CutNodeData(err, contrib, leftChild));

				contrib = EstimateNodeIllumination(scene, renderer, arena, ray, isect,
					rng, p, n, wo, bsdf, gLimit, rrThreshold, nodeData[rightChild]);
				L += contrib;
				err = ErrorBound(p, n, wo, bsdf, nodeData[rightChild]);
				q.push(CutNodeData(err, contrib, rightChild));
			}
		}
	}

	count += 1.f;
	avgCutSize += q.size() + nLeafNodes;
	
	return L;
	//float rgb[3] = { max(L.x(0), 0.f), max(L.x(1), 0.f), max(L.x(2), 0.f) };
	//Spectrum res = Spectrum::FromRGB(rgb);
	//return res;
}