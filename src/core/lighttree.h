/*
	Created by Lifan: adapt from kdtree.h
*/

#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_CORE_LIGHTTREE_H
#define PBRT_CORE_LIGHTTREE_H

// core/paramset.h*
#include "pbrt.h"
#include "geometry.h"
#include "primitive.h"
#include "spectrum.h"
#include "light.h"
#include "reflection.h"
#include "sampler.h"
#include "material.h"
#include "probes.h"
#include "renderer.h"
#include "../lights/point.h"

struct PointLightNode {
	void init(float p, uint32_t a) {
		splitPos = p;
		splitAxis = a;
		rightChild = (1 << 29) - 1;
		hasLeftChild = 0;
	}
	void initLeaf() {
		splitAxis = 3;
		rightChild = (1 << 29) - 1;
		hasLeftChild = 0;
	}

	float splitPos;
	uint32_t splitAxis : 2;
	uint32_t hasLeftChild : 1, rightChild : 29;
};

struct PointLightNodeData {
	// representative light info
	Point lightPos;
	Normal lightN;
	Spectrum Intensity;
	float rayEpsilon;
	
	BBox bound;
};


struct CutNodeData {
	// index of light tree node
	int lightIdx;
	// maximum error bound of the node
	Spectrum errBound;
	float errBoundValue;

	Spectrum contrib;

	CutNodeData() {}
	CutNodeData(Spectrum _err, Spectrum _contrib, int _idx) : lightIdx(_idx), 
			errBound(_err), contrib(_contrib) {
		errBoundValue = errBound.y();
	}

	bool operator <(const CutNodeData &d) const {
		if (errBoundValue == d.errBoundValue) return lightIdx > d.lightIdx;
		else return errBoundValue < d.errBoundValue;
	}
};


class PointLightTree {
public:
	PointLightTree(const vector<PointLightNodeData> &data);
	~PointLightTree() {
		FreeAligned(nodes);
		FreeAligned(nodeData);
	}

	void recursiveBuild(uint32_t nodeNum, int start, int end,
		const PointLightNodeData **buildNodes);

	Spectrum refineLightcuts(const Scene *scene,
		const Renderer *renderer, MemoryArena &arena, 
		const RayDifferential &ray, const Intersection &isect,
		RNG &rng, const Point &p, const Normal &n, const Vector &wo, 
		BSDF *bsdf, float gLimit, float rrThreshold);

	void test();

	PointLightNode *nodes;
	PointLightNodeData *nodeData;
	uint32_t nNodes, nextFreeNode;
	RNG rng;
	
	int MAX_CUT_SIZE;
	float MAX_ERROR_RATIO;
};


struct ComparePointLightNode {
	ComparePointLightNode(int a) { axis = a; }
	int axis;
	bool operator()(const PointLightNodeData *d1, const PointLightNodeData *d2) const {
		return d1->lightPos[axis] == d2->lightPos[axis] ? (d1 < d2) :
			d1->lightPos[axis] < d2->lightPos[axis];
	}
};

Spectrum ErrorBound(const Point &p, const Normal &n, const Vector &wo,
	BSDF *bsdf, const PointLightNodeData &vl);

Spectrum EstimateNodeIllumination(const Scene *scene,
	const Renderer *renderer, MemoryArena &arena,
	const RayDifferential &ray, const Intersection &isect,
	RNG &rng, const Point &p, const Normal &n, const Vector &wo,
	BSDF *bsdf, float gLimit, float rrThreshold, const PointLightNodeData &vl);

#endif
