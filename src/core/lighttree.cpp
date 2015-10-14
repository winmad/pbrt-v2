#include "stdafx.h"
#include "lighttree.h"

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

