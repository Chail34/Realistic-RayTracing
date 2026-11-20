#ifndef BVH_H
#define BVH_H

#include <cfloat>
#include <vector>
#include "AABB.h"
#include "Triangle.h"

// Noeud BVH
struct Node
{
    AABB aabb ;
    int left  = -1 ; // Fils gauche
    int right = -1 ; // Fils droit
    unsigned int firstTriangle = 0 ;
    unsigned int numberOfTriangles = 0 ;
    bool isLeaf() const {return left < 0 ;}
};

// Collision BVH
struct Collision 
{
    bool exists = false ;
    float t = FLT_MAX ;
    unsigned int triangleIndex = 0 ;
    // Coordonnées barycentriques du point d'impact sur le triangle
    float w0 = 0.0f ;
    float w1 = 0.0f ; // u
    float w2 = 0.0f ; // v
};

// Squelette BVH
class BVH {
    
    public:
        // Nombre de triangles maximal par feuille
        static const unsigned int MAX_TRIANGLE = 5 ;

        // Attributs de l'arbre
        std::vector<Triangle> triangles ;
        std::vector<unsigned int> trianglesIndex ;
        std::vector<Node> nodes ;

        // Méthodes de création et de parcours de l'arbre
        void build(std::vector<Triangle> & tri);
        Collision intersect(const Ray & ray) const ;

    private:
        void updateBox(int nodeIndex);
        void subdivide(int nodeIndex);
        // Méthodes des différentes intersections
        bool intersectAABB(const Ray & ray, const AABB & aabb, float tMax) const ;
        void intersectNode(Ray const & ray, int nodeIndex, Collision & outCollision) const ;
        void intersectTriangle(const Ray & ray, unsigned int triangleIndex, Collision & outCollision) const ;
    };

#endif