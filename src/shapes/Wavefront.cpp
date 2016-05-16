
/*
    pbrt source code Copyright(c) 1998-2010 Matt Pharr and Greg Humphreys.

    This file is part of pbrt.

    pbrt is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.  Note that the text contents of
    the book "Physically Based Rendering" are *not* licensed under the
    GNU GPL.

    pbrt is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 */

// shapes/Wavefront.cpp*
#include "stdafx.h"
#include "shapes/Wavefront.h"
#include "texture.h"
#include "textures/constant.h"
#include "paramset.h"
#include "montecarlo.h"

// line buffer size determines at compile time how large the input
// buffer should be for the file input lines
const int LINE_BUFFER_SIZE = 1024;

// GETNUM just gets the next number from a line of input in an OBJ file
#ifndef GETNUM
#define GETNUM(lineBuffer, numBuffer, lindex, nindex, tval)  \
	nindex=0;\
	while ((lineBuffer[lindex] == ' ') || lineBuffer[lindex] == '/') lindex++;\
	while ((lineBuffer[lindex] != ' ') && (lineBuffer[lindex] != '/') && \
		   (lineBuffer[lindex] != '\0') && (lineBuffer[lindex] != '\n') && (lindex != LINE_BUFFER_SIZE)) { \
		numBuffer[nindex] = lineBuffer[lindex]; \
		nindex++; \
		lindex++; \
	} \
	numBuffer[nindex] = '\0'; \
	tval = atoi(numBuffer);
#endif

// constructor / parser
void
Wavefront::ParseOBJfile(string filename,int **vi,int *nvi,Point ** P,int * npi,Normal ** N,int * nni,float ** uvs,int *nuvi) {
	FILE* fin;
	fin = fopen(filename.c_str(), "r");
	if (!fin) return;
	
	// temporary input buffers
	vector<Point> points;
	vector<int> verts;
	vector<int> normalIndex;
	vector<int> uvIndex;
	
	vector<Normal> file_normals;
	vector<float> file_uvvector;
		
	Point ptmp;
	Normal ntmp;
	float uv1, uv2;
	
	char lineBuffer[LINE_BUFFER_SIZE];
	char numBuffer[256];
	int lindex=0;
	int nindex=0;
	int ival, uvval, nval;
	ntris=0;
	
	// parse the data in
	while (fgets(lineBuffer, LINE_BUFFER_SIZE, fin)) {
		switch (lineBuffer[0]) {
			case 'v':
				// case vertex information
				if (lineBuffer[1] == ' ') {
					// regular vertex point
					sscanf(&lineBuffer[2], "%f %f %f", &ptmp.x, &ptmp.y, &ptmp.z);
					points.push_back(ptmp);
				} else if (lineBuffer[1] == 't') {
					// texture coordinates
					sscanf(&lineBuffer[3], "%f %f", &uv1, &uv2);
					file_uvvector.push_back(uv1);
					file_uvvector.push_back(uv2);
				} else if (lineBuffer[1] == 'n') {
					// normal vector
					sscanf(&lineBuffer[2], "%f %f %f", &ntmp.x, &ntmp.y, &ntmp.z);
					file_normals.push_back(ntmp);
				}
				break;
			case 'f':
				// case face information
				lindex = 2;
				ntris++;
				for (int i=0; i < 3; i++) {
					
					GETNUM(lineBuffer, numBuffer, lindex, nindex, ival)
					
					// obj files go from 1..n, this just allows me to access the memory
					// directly by droping the index value to 0...(n-1)
					ival--;
					verts.push_back(ival);
										
					if (lineBuffer[lindex] == '/') {
						lindex++;
						GETNUM(lineBuffer, numBuffer, lindex, nindex, uvval)
						uvIndex.push_back(uvval-1);
					}
					
					if (lineBuffer[lindex] == '/') {
						lindex++;
						GETNUM(lineBuffer, numBuffer, lindex, nindex, nval)
						normalIndex.push_back(nval-1);
					}
					lindex++;
				}
				break;
			case 'g':
				// not really caring about faces or materials now
				// so just making life easier, I'll ignoring it
				break;
		}
	}
		
	fclose(fin);
	
	// merge everything back into one index array instead of multiple arrays
	MergeIndicies(points, file_normals, file_uvvector, verts, normalIndex, uvIndex);
		
	*vi = this->vertexIndex;
	*nvi = this->nvi;

	*P = this->p;
	*npi = *nvi;

	*N = this->n;
	*nni = *nvi;

	*uvs = this->uvs;
	*nuvi = *nvi;
	
	points.clear();
	file_normals.clear();
	file_uvvector.clear();
	verts.clear();
	normalIndex.clear();
	uvIndex.clear();
		

	Warning("Found %d vertices (%d)\n",*nvi,vi);
	Warning("Found %d points (%d)\n",*nvi,P);

	if (n) Warning("Used normal\n");
	if (uvs) Warning("Used UVs\n");
	
}

void Wavefront::MergeIndicies(vector<Point> &points, vector<Normal> &normals, vector<float> &uvVec, vector<int> &vIndex, vector<int> &normalIndex, vector<int> &uvIndex) {

	bool useNormals = !normals.empty();
	bool useUVs = !uvVec.empty();
	
	
	if (!useNormals && !useUVs) { 
		Warning("Copying points\n");
		// just copy the points into the array
		nverts = vIndex.size();
		p = new Point[points.size()];
		nvi = points.size();
		for (unsigned int i=0; i < points.size(); i++)
			p[i] = points[i];
		vertexIndex = new int[nverts];
		for (int i=0; i < nverts; i++)
			vertexIndex[i] = vIndex[i];
		return;
	}
		
	// assumes that vertexIndex = normalIndex = uvIndex	
	nvi = nverts = vIndex.size();				// FIX: Dec, 3
	vertexIndex = new int[nverts];
	
	p = new Point[nverts];
	if (useNormals) n = new Normal[nverts];
	if (useUVs) uvs = new float[nverts*2];
	
	for (int i=0; i < nverts; i++) {
		p[i] = points[vIndex[i]];
		if (useNormals) n[i] = normals[normalIndex[i]];
		if (useUVs) { 
			uvs[i*2] = uvVec[uvIndex[i]*2];
			uvs[i*2+1] = uvVec[uvIndex[i]*2 + 1];
		}
		vertexIndex[i] = i;
	}	
}

TriangleMesh *CreateWavefrontShape(const Transform *o2w, const Transform *w2o,
				   bool reverseOrientation, const ParamSet &params,
				   map<string, Reference<Texture<float> > > *floatTextures) {
  string filename = params.FindOneString("filename","");
  bool discardDegnerateUVs = params.FindOneBool("discarddegenerateUVs", false);
  // XXX should complain if uvs aren't an array of 2...
  
  Wavefront * wfs = new Wavefront();
  int nvi =0, npi = 0, nuvi = 0, nsi = 0, nni = 0;
  int *vi    = NULL;
  Point *P   = NULL;
  float *uvs = NULL;
  Normal * N = NULL;
  Vector * S = NULL;
  
  wfs->ParseOBJfile(filename,&vi,&nvi,&P,&npi,&N,&nni,&uvs,&nuvi);
  
  if (uvs) {
    if (nuvi < 2 * npi) {
      Error("Not enough of \"uv\"s for triangle mesh.  Expencted %d, "
	    "found %d.  Discarding.\n", 2*npi, nuvi);
      uvs = NULL;
    }
    else if (nuvi > 2 * npi)
      Warning("More \"uv\"s provided than will be used for triangle "
	      "mesh.  (%d expcted, %d found)\n", 2*npi, nuvi);
  }
  if (!vi || !P) {
    Warning("No vertices (%d) or points (%d) found. TriangleMesh not created.",vi,P);
    return NULL;
  }

  if (S && nsi != npi) {
    Error("Number of \"S\"s for triangle mesh must match \"P\"s");
    S = NULL;
  }
  if (N && nni != npi) {
    Error("Number of \"N\"s for triangle mesh must match \"P\"s");
    N = NULL;
  }
  
  if (discardDegnerateUVs && uvs && N) {
    // if there are normals, check for bad uv's that
    // give degenerate mappings; discard them if so
    const int *vp = vi;
    for (int i = 0; i < nvi; i += 3, vp += 3) {
      float area = .5f * Cross(P[vp[0]]-P[vp[1]], P[vp[2]]-P[vp[1]]).Length();
      if (area < 1e-7) continue; // ignore degenerate tris.
      if ((uvs[2*vp[0]] == uvs[2*vp[1]] &&
	   uvs[2*vp[0]+1] == uvs[2*vp[1]+1]) ||
	  (uvs[2*vp[1]] == uvs[2*vp[2]] &&
	   uvs[2*vp[1]+1] == uvs[2*vp[2]+1]) ||
                (uvs[2*vp[2]] == uvs[2*vp[0]] &&
		 uvs[2*vp[2]+1] == uvs[2*vp[0]+1])) {
	Warning("Degenerate uv coordinates in triangle mesh.  Discarding all uvs.");
	uvs = NULL;
	break;
      }
    }
  }
  for (int i = 0; i < nvi; ++i)
    if (vi[i] >= npi) {
      Error("trianglemesh has out of-bounds vertex index %d (%d \"P\" values were given",
	    vi[i], npi);
      return NULL;
        }
  
  Reference<Texture<float> > alphaTex = NULL;
  string alphaTexName = params.FindTexture("alpha");
  if (alphaTexName != "") {
    if (floatTextures->find(alphaTexName) != floatTextures->end())
      alphaTex = (*floatTextures)[alphaTexName];
    else
      Error("Couldn't find float texture \"%s\" for \"alpha\" parameter",
	    alphaTexName.c_str());
  }
  else if (params.FindOneFloat("alpha", 1.f) == 0.f)
    alphaTex = new ConstantTexture<float>(0.f);

  Warning("OBJ Parsing done creating Triangle mesh with %d vertices ... ",nvi);

  return new TriangleMesh(o2w, w2o, reverseOrientation, nvi/3, npi, vi, P,
			    N, S, uvs, alphaTex);
}
