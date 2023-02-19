#include "Primitive.h"
#include "threading.hpp"
#include "Mesh.h"
#include "Memory.h"

struct OctTree : IntersectionAccelerator {
	struct Node {
		BBox box;
		Node *children[8] = {nullptr, };
		std::vector<Intersectable*> primitives;
		bool isLeaf() const {
			return children[0] == nullptr;
		}
	};

	std::vector<Intersectable*> allPrimitives;
	Node *root = nullptr;
	int depth = 0;
	int leafSize = 0;
	int nodes = 0;
	int MAX_DEPTH = 35;
	int MIN_PRIMITIVES = 10;

	void clear(Node *n) {
		if (!n) {
			return;
		}

		for (int c = 0; c < 8; c++) {
			clear(n->children[c]);
			delete n->children[c];
		}
	}

	void clear() {
		clear(root);
		allPrimitives.clear();
	}

	void addPrimitive(Intersectable* prim) override {
		allPrimitives.push_back(prim);
	}

	void build(Node *n, int currentDepth = 0) {
		if (currentDepth >= MAX_DEPTH || n->primitives.size() <= MIN_PRIMITIVES) {
			leafSize = std::max(leafSize, int(n->primitives.size()));
			return;
		}

		depth = std::max(depth, currentDepth);
		BBox childBoxes[8];
		n->box.octSplit(childBoxes);

		for (int c = 0; c < 8; c++) {
			Node *& child = n->children[c];
			child = new Node;
			nodes++;
			memset(child->children, 0, sizeof(child->children));
			child->box = childBoxes[c];
			for (int r = 0; r < n->primitives.size(); r++) {
				if (n->primitives[r]->boxIntersect(child->box)) {
					child->primitives.push_back(n->primitives[r]);
				}
			}
			if (child->primitives.size() == n->primitives.size()) {
				build(child, MAX_DEPTH + 1);
			} else {
				build(child, currentDepth + 1);
			}
		}
		n->primitives.clear();
	}

	void build(Purpose purpose) override {
		const char *treePurpose = "";
		if (purpose == Purpose::Instances) {
			MAX_DEPTH = 5;
			MIN_PRIMITIVES = 4;
			treePurpose = " instances";
		} else if (purpose == Purpose::Mesh) {
			MAX_DEPTH = 35;
			MIN_PRIMITIVES = 20;
			treePurpose = " mesh";
		}

		if (root) {
			clear(root);
			delete root;
		}

		printf("Building%s oct tree with %d primitives... ", treePurpose, int(allPrimitives.size()));
		Timer timer;
		nodes = leafSize = depth = 0;
		root = new Node();
		root->primitives.swap(allPrimitives);
		for (int c = 0; c < root->primitives.size(); c++) {  
			root->primitives[c]->expandBox(root->box);
		}

		build(root);
		printf(" done in %lldms, nodes %d, depth %d, %d leaf size\n", timer.toMs(timer.elapsedNs()), nodes, depth, leafSize);
	}

	bool intersect(Node *n, const Ray& ray, float tMin, float &tMax, Intersection& intersection) {
		bool hasHit = false;

		if (n->isLeaf()) {
			for (int c = 0; c < n->primitives.size(); c++) {
				if (n->primitives[c]->intersect(ray, tMin, tMax, intersection)) {
					tMax = intersection.t;
					hasHit = true;
				}
			}
		} else {
			for (int c = 0; c < 8; c++) {
				if (n->children[c]->box.testIntersect(ray)) {
					if (intersect(n->children[c], ray, tMin, tMax, intersection)) {
						tMax = intersection.t;
						hasHit = true;
					}
				}
			}
		}

		return hasHit;
	}

	bool intersect(const Ray& ray, float tMin, float tMax, Intersection& intersection) override {
		return intersect(root, ray, tMin, tMax, intersection);
	}

	bool isBuilt() const override {
		return root != nullptr;
	}

	~OctTree() override {
		clear();
	}
};



/// TODO: Implement one/both or any other acceleration structure and change makeDefaultAccelerator to create it
struct KDTree : IntersectionAccelerator { 
	enum class EdgeType { Start, End };
	struct BoundEdge {
		BoundEdge() {}
		BoundEdge(float t, int primNum, bool starting) : t(t), primNum(primNum) {
			type = starting ? EdgeType::Start : EdgeType::End;
		}
		float t;
		int primNum;
		EdgeType type;
	};

	struct Node {
		void initLeaf(int* primNums, int np, std::vector<int>* primitiveIndices) {
			flags = 3;
			nPrims |= (np << 2);
			if(np == 0) {
				onePrimitive = 0;
			}
			else if (np == 1) {
				onePrimitive = primNums[0];
			} else {
				primitiveIndicesOffset = primitiveIndices->size();
				for(int i =0; i < np; i++) {
					primitiveIndices->push_back(primNums[i]);
				}
			}
		}

		void initInterior(int axis, int ac, float splitPosition) {
			this->splitPosition = splitPosition;
			flags = axis;
			aboveChild |= (ac << 2);
		}
		
		float splitPos() const { return splitPosition; }
		int nPrimitives() const { return nPrims >> 2; }
		int splitAxis() const { return flags & 3; }
		bool isLeaf() const { return (flags & 3) == 3; }
		int AboveChild() const { return aboveChild >> 2; }
		
		union {
			float splitPosition;
			int onePrimitive;
			int primitiveIndicesOffset;
		};

		private:
			union {
				int flags;
				int nPrims;
				int aboveChild;
			};
	};

	struct KdToDo {
		const Node* node;
		float tMin, tMax;
	};

	~KDTree() {
		if(nodes) 
			Release(nodes);
	}	

private:
	Node* nodes;
	BBox bounds;
	float emptyBonus;
	std::vector<Intersectable*> allPrimitives;
	std::vector<int> primitiveIndices;

	int MAX_DEPTH = 35;
	int MAX_PRIMITIVES = 10;
	int nAllocedNodes, nextFreeNode;
	int totalNodes = 0;
	int leafSize = 0;
	int depth = 0;

	void addPrimitive(Intersectable *prim) override {
		allPrimitives.push_back(prim);
	}
	
	void clear(Node *n) {
		if(n)
			Release(n);
	}

	void clear() {
		clear(nodes);
		allPrimitives.clear();
	}

public:
	void build(Purpose purpose) override {
		const char *treePurpose = "";
		if (purpose == Purpose::Instances) {
			MAX_DEPTH = 5;
			MAX_PRIMITIVES = 4;
			treePurpose = " instances";
		} else if (purpose == Purpose::Mesh) {
			MAX_DEPTH = 35;
			MAX_PRIMITIVES = 20;
			treePurpose = " mesh";
		}

		if (nodes) {
			clear(nodes);
			delete nodes;
		}

		printf("Building%s kd tree with %d primitives... ", treePurpose, int(allPrimitives.size()));
		Timer timer;
		
		nAllocedNodes = nextFreeNode = 0;
		emptyBonus = 0.5f;
		std::vector<BBox> primBounds;
		primBounds.reserve(allPrimitives.size());

		for(auto prim : allPrimitives) {
			BBox box;
			prim->expandBox(box);
			primBounds.push_back(box);
			
			prim->expandBox(bounds);
		}

		std::unique_ptr<BoundEdge[]> edges[3];
    	for (int i = 0; i < 3; ++i) {
        	edges[i].reset(new BoundEdge[2 * allPrimitives.size()]);
		}
		
		std::unique_ptr<int[]> prims0(new int[allPrimitives.size()]);
		std::unique_ptr<int[]> prims1(new int[(MAX_DEPTH + 1) * allPrimitives.size()]);

    	std::unique_ptr<int[]> primNums(new int[allPrimitives.size()]);
    	for (size_t i = 0; i < allPrimitives.size(); ++i) {
			primNums[i] = i;
		}

		buildTree(0, bounds, primBounds, primNums.get(), allPrimitives.size(), edges, prims0.get(), prims1.get(), 0);
		printf(" done in %lldms, nodes %d, depth %d, %d leaf size\n", timer.toMs(timer.elapsedNs()), totalNodes, depth, leafSize); 
	}

	void buildTree(	int nodeNum, const BBox& nodeBounds, const std::vector<BBox>& primBounds, int* primNums, 
					int nPrims, const std::unique_ptr<BoundEdge[]> edges[3], int* prims0, int* prims1, int currentDepth, int badRefines = 0) {
		if (nextFreeNode == nAllocedNodes) {
			int nNewAllocNodes = std::max(2 * nAllocedNodes, 512);
			Node* n = AllocateAllined<Node>(nNewAllocNodes);
			if (nAllocedNodes > 0) {
				memcpy(n, nodes, nAllocedNodes * sizeof(Node));
				Release(nodes);
			}
			nodes = n;
			nAllocedNodes = nNewAllocNodes;
    	}
    	++nextFreeNode;	

		if(nPrims <= MAX_PRIMITIVES || currentDepth >= MAX_DEPTH) {
			nodes[nodeNum].initLeaf(primNums, nPrims, &primitiveIndices);
			totalNodes++;
			leafSize = std::max(leafSize, int(nPrims));
			depth = std::max(depth, currentDepth);

			return;
		}	

		// SAH
		int bestAxis = -1;
		int bestOffset = -1;

		float bestCost = INFINITY;
		float oldCost = nPrims;
		float totalSA = nodeBounds.surfaceArea();
		float invTotalSA = 1 / totalSA;
		vec3 d = nodeBounds.max - nodeBounds.min;

		int axis = nodeBounds.maximumExtent();

		for(int retries = 0; retries < 2; retries++) {
			for(int i = 0; i < nPrims; i++) {
				int pn = primNums[i];
				const BBox& bounds = primBounds[pn];

				edges[axis][2 * i] = BoundEdge(bounds.min[axis], pn, true);
				edges[axis][2 * i + 1] = BoundEdge(bounds.max[axis], pn, false);
			}

			std::sort(&edges[axis][0], &edges[axis][2 * nPrims],
				[](const BoundEdge &e0, const BoundEdge &e1) -> bool {
					if (e0.t == e1.t)
						return (int)e0.type < (int)e1.type;
					else
						return e0.t < e1.t;
				});

			int nBelow = 0;
			int nAbove = nPrims;

			for(int i =0; i < 2 * nPrims; i++) {
				if(edges[axis][i].type == EdgeType::End) --nAbove;
				float edgeT = edges[axis][i].t;
				if(edgeT > nodeBounds.min[axis] && edgeT < nodeBounds.max[axis]) {
					int otherAxis0 = (axis + 1) % 3;
					int otherAxis1 = (axis + 2) % 3;

					float belowSA = 2 * (d[otherAxis0] * d[otherAxis1] +
									(edgeT - nodeBounds.min[axis]) *
										(d[otherAxis0] + d[otherAxis1]));
										
					float aboveSA = 2 * (d[otherAxis0] * d[otherAxis1] +
									(nodeBounds.max[axis] - edgeT) *
										(d[otherAxis0] + d[otherAxis1]));

					float pBelow = belowSA * invTotalSA;
					float pAbove = aboveSA * invTotalSA;

					float bonus = (nAbove == 0 || nBelow == 0) ? emptyBonus : 0;
					// assume traversal and intersection cost == 1
					float cost = (1 - bonus) * (pBelow * nBelow + pAbove * nAbove);

					if(cost < bestCost) {
						bestCost = cost;
						bestAxis = axis;
						bestOffset = i;
					}
				}
				if (edges[axis][i].type == EdgeType::Start) ++nBelow;
			}

			if(bestAxis == -1) {
				axis = (axis + 1) % 3;
				continue;
			} else {
				break;
			}
		}
			 
		if(bestCost > oldCost) ++badRefines;
		if((bestCost > 4 * oldCost && nPrims < 16) || bestAxis == -1 || badRefines == 3) {
			nodes[nodeNum].initLeaf(primNums, nPrims, &primitiveIndices);
			return;
		}

		int n0 = 0;
		int n1 = 0;

		for(int i = 0; i < bestOffset; i++) {
			if(edges[bestAxis][i].type == EdgeType::Start) {
				prims0[n0++] = edges[bestAxis][i].primNum;
			}
		}

		for(int i = bestOffset + 1; i < 2 * nPrims; i++) {
			if(edges[bestAxis][i].type == EdgeType::End) {
				prims1[n1++] = edges[bestAxis][i].primNum;
			}
		}

		float split = edges[bestAxis][bestOffset].t;
		BBox bounds0 = nodeBounds;
		BBox bounds1 = nodeBounds;

		bounds0.max[bestAxis] = bounds1.min[bestAxis] = split;
		buildTree(nodeNum + 1, bounds0, primBounds, prims0, n0, edges, prims0, prims1 + nPrims, currentDepth + 1, badRefines);
		
		int aboveChild = nextFreeNode;
		nodes[nodeNum].initInterior(bestAxis, aboveChild, split);

		buildTree(aboveChild, bounds1, primBounds, prims1, n1, edges, prims0, prims1 + nPrims, currentDepth + 1, badRefines);
	}

	bool isBuilt() const override { 
		return nodes != nullptr; 
	}

	bool kdIntersect(const Ray& ray, float tMin, float& tMax, Intersection& info) {
		if(!bounds.intersect(ray, tMin, tMax)) {
			return false;
		}

		vec3 invDir(1 / ray.dir.x, 1 / ray.dir.y, 1 / ray.dir.z);
		const int maxToDo = 64;

		KdToDo todo[maxToDo];
		int todoPos = 0;

		bool hit = false;
		const Node* node = &nodes[0];

		while(node != nullptr) {
			if(!node->isLeaf()) {
				int axis = node->splitAxis();
				float tPlane = (node->splitPosition - ray.origin[axis]) * invDir[axis];

				const Node* firstChild;
				const Node* secondChild;

				int belowFirst = (ray.origin[axis] < node->splitPosition) || 
								(ray.origin[axis] == node->splitPosition && ray.dir[axis] <= 0);
				if(belowFirst) {
					firstChild = node + 1;
					secondChild = &nodes[node->AboveChild()];
				}
				else {
					firstChild = &nodes[node->AboveChild()];
					secondChild = node + 1;
				}

				if(tPlane > tMax || tPlane <= 0) {
					node = firstChild;
				}
				else if(tPlane < tMin) {
					node = secondChild;
				}
				else {
					todo[todoPos].node = secondChild;
					todo[todoPos].tMin = tPlane;
					todo[todoPos].tMax = tMax;
					++todoPos;
					node = firstChild;
					tMax = tPlane;
				}
			} else {
				int nPrims = node->nPrimitives();
				if(nPrims == 1) {
					Intersectable* p = allPrimitives[node->onePrimitive];
                
                	if (p->intersect(ray, tMin, tMax, info)) {
						hit = true;
						tMax = info.t;
					}
						
				} else {
					for(int i = 0; i < nPrims; i++) {
						int index = primitiveIndices[node->primitiveIndicesOffset + i];
						auto prim = allPrimitives[index];
						if(prim->intersect(ray, tMin, tMax, info)) {
							hit = true;
							tMax = info.t;
						}
					}
				}

				if(todoPos > 0) {
					--todoPos;
					node = todo[todoPos].node;
					tMin = todo[todoPos].tMin;
					tMax = todo[todoPos].tMax;
				} else {
					break;
				}

			}
		}

	return hit;
}

	bool intersect(const Ray& ray, float tMin, float tMax, Intersection& intersection) override {
		return kdIntersect(ray, tMin, tMax, intersection);
	}
};

struct BVHTree : IntersectionAccelerator {
	void addPrimitive(Intersectable *prim) override {}
	void clear() override {}
	void build(Purpose purpose) override {}
	bool isBuilt() const override { return false; }
	bool intersect(const Ray &ray, float tMin, float tMax, Intersection &intersection) override { return false; }
};

AcceleratorPtr makeDefaultAccelerator() {
	// TODO: uncomment or add the acceleration structure you have implemented
	//  return AcceleratorPtr(new OctTree());
	return AcceleratorPtr(new KDTree());
}

