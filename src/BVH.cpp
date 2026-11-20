#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <algorithm>

#include "BVH.h"
#include "Triangle.h"

/*
Fonction d'initialisation de l'arbre BVH

- tri : tableau de triangles
*/
void BVH::build(std::vector<Triangle> & tri)
{
    // Initialisation
    triangles = tri ;
    unsigned int nbTriangles = triangles.size() ;
    nodes.clear();
    trianglesIndex.resize(nbTriangles);
    if(triangles.empty()) return ; // Si aucun triangles fin de la fonction

    // Calcul des centroid des triangles
    for(unsigned int i = 0 ; i < nbTriangles ; i++){
        Triangle t = triangles[i] ;
        trianglesIndex[i] = i ;
        triangles[i].centroid = (t.m_c[0] + t.m_c[1] + t.m_c[2]) / 3.0f ;
        triangles[i].updateAreaAndNormal();
    }

    // Création de la racine dans l'arbre
    nodes.reserve(2 * nbTriangles); // Taille max d'un arbre binaire
    Node root ;
    root.firstTriangle = 0 ;
    root.numberOfTriangles = nbTriangles ;
    nodes.push_back(root);

    // Remplissage récursif de l'arbre
    updateBox(0);
    subdivide(0);
}


/*
Fonction de mise à jour de la boite englobante

- nodeIndex : indice du noeud dont la boite doit être recalculée
*/
void BVH::updateBox(int nodeIndex)
{
    Node & node = nodes[nodeIndex];
    node.aabb = AABB();
    for(unsigned int i = node.firstTriangle ; i < node.firstTriangle + node.numberOfTriangles ; i++){
        Triangle t = triangles[trianglesIndex[i]] ;
        node.aabb.expand(t.m_c[0]);
        node.aabb.expand(t.m_c[1]);
        node.aabb.expand(t.m_c[2]);
    }
}


/*
Fonction de subdivision d'un noeud

- nodeIndex : indice du noeud a diviser en 2
*/
void BVH::subdivide(int nodeIndex)
{
    unsigned int nbTriangles = nodes[nodeIndex].numberOfTriangles ;
    unsigned int firstTriangle = nodes[nodeIndex].firstTriangle ;

    // On vérifie qu'on doit encore subdiviser
    if(nbTriangles <= MAX_TRIANGLE) return ;

    // On coupe selon le plan le plus long de la boite des centroid, pas des triangles
    AABB centroidBox ;
    for(unsigned int i = firstTriangle ; i < firstTriangle + nbTriangles ; i++)
        centroidBox.expand(triangles[trianglesIndex[i]].centroid);
    int axis = centroidBox.longestAxis() ;
    float split = (centroidBox.min[axis] + centroidBox.max[axis]) * 0.5f ;

    // Réarrangement des triangles
    unsigned int a = firstTriangle ;
    unsigned int b = firstTriangle + nbTriangles ;
    while(a < b){
        if(triangles[trianglesIndex[a]].centroid[axis] < split) a++ ;
        else std::swap(trianglesIndex[a], trianglesIndex[--b]);
    }

    // Si les triangles sont regroupé et qu'on arrive pas a subdiviser
    unsigned int leftSideCount = a - firstTriangle ;
    if(leftSideCount == 0 || leftSideCount == nbTriangles) return ;

    // Création des noeuds enfants
    Node left, right ;
    left.firstTriangle = firstTriangle ;
    left.numberOfTriangles = leftSideCount ;
    right.firstTriangle = a ;
    right.numberOfTriangles = nbTriangles - leftSideCount ;

    int leftIndex = nodes.size() ;
    nodes.push_back(left);
    int rightIndex = nodes.size() ;
    nodes.push_back(right);

    nodes[nodeIndex].left = leftIndex ;
    nodes[nodeIndex].right = rightIndex ;
    nodes[nodeIndex].numberOfTriangles = 0 ; // Seuls les feuilles contiennent des triangles

    // Mise à jour + récursion
    updateBox(leftIndex);
    updateBox(rightIndex);
    subdivide(leftIndex);
    subdivide(rightIndex);
}


/*
Fonction qui dit si le rayon 'ray' entre en contact avec la boite englobante 'aabb'

- ray : rayon incident
- aabb : boite dont l'on veut vérifier si elle est sur notre route 
- tMax : distance maximal d'où on cherche l'intersection, on regarde pas plus loin
*/
bool BVH::intersectAABB(const Ray & ray, const AABB & aabb, float tMax) const
{
    Vec3 rayOrigin = ray.origin() ;
    Vec3 rayDirection = ray.direction() ;
    float tmin = 0.0f ;
    float tmax = tMax ;
    float all_tmin[3] ;
    float all_tmax[3] ;

    // Calcul des tmin et tmax
    for(int i = 0 ; i < 3 ; i++){
        float t1 = (aabb.min[i] - rayOrigin[i]) / rayDirection[i] ;
        float t2 = (aabb.max[i] - rayOrigin[i]) / rayDirection[i] ;

        tmin = std::min(t1, t2);
        tmax = std::min(std::max(t1,t2), tmax);

        all_tmin[i] = tmin ;
        all_tmax[i] = tmax ;
    }

    // Calcul et vérification de l'intersection
    float firstIntersection  = std::max( std::max(all_tmin[0], all_tmin[1]), all_tmin[2] ) ;
    float secondIntersection = std::min( std::min(all_tmax[0], all_tmax[1]), all_tmax[2] ) ;

    return ((firstIntersection <= secondIntersection) && (secondIntersection > 0.0f));
}

/*
Fonction qui vérifie l'intersection entre le rayon et le triangle

- ray : rayon incident
- aabb : boite dont l'on veut vérifier si elle est sur notre route 
- outIntersection : information de l'intersection avec le triangle
*/
void BVH::intersectTriangle(const Ray & ray, unsigned int triangleIndex, Collision & outCollision) const
{
    const Triangle & triangle = triangles[triangleIndex] ;
    RayTriangleIntersection intersection = triangle.getIntersection(ray);

    if(intersection.intersectionExists && intersection.t < outCollision.t){
        outCollision.exists = true ;
        outCollision.t = intersection.t ;
        outCollision.triangleIndex = triangleIndex ;
        outCollision.w0 = intersection.w0 ;
        outCollision.w1 = intersection.w1 ;
        outCollision.w2 = intersection.w2 ;
    }
}


/*
Fonction qui vérifie que le rayon 'ray' parcours bien le noeud 'nodeIndex'

- ray : rayon incident
- aabb : boite dont l'on veut vérifier si elle est sur notre route 
- outIntersection : information de l'intersection avec le triangle
*/
void BVH::intersectNode(const Ray & ray, int nodeIndex, Collision & outCollision) const
{
    const Node & node = nodes[nodeIndex] ;
    if(!intersectAABB(ray, node.aabb, outCollision.t)) return ;

    unsigned int firstIndex = node.firstTriangle ;
    unsigned int nbTriangles = node.numberOfTriangles ;
    if(node.isLeaf()){
        for(unsigned i = firstIndex ; i < firstIndex + nbTriangles ; i++)
            intersectTriangle(ray, trianglesIndex[i], outCollision);
        return ;
    }

    intersectNode(ray, node.left, outCollision);
    intersectNode(ray, node.right, outCollision);
}

/*
Fonction qui dit où le rayon touche

- ray : rayon incident
*/
Collision BVH::intersect(const Ray & ray) const
{
    Collision collision ;
    if(!nodes.empty()) intersectNode(ray, 0, collision);
    return collision;
}