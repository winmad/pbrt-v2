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
	
	BBox bound;
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

	void test();

	PointLightNode *nodes;
	PointLightNodeData *nodeData;
	uint32_t nNodes, nextFreeNode;
	RNG rng;
};


struct ComparePointLightNode {
	ComparePointLightNode(int a) { axis = a; }
	int axis;
	bool operator()(const PointLightNodeData *d1, const PointLightNodeData *d2) const {
		return d1->lightPos[axis] == d2->lightPos[axis] ? (d1 < d2) :
			d1->lightPos[axis] < d2->lightPos[axis];
	}
};


#endif
