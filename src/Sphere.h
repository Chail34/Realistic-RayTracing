#ifndef Sphere_H
#define Sphere_H
#include "Vec3.h"
#include <vector>
#include "Mesh.h"
#include <cmath>

struct RaySphereIntersection{
    bool intersectionExists;
    float t;
    float theta,phi;
    Vec3 intersection;
    Vec3 secondintersection;
    Vec3 normal;
};

static
Vec3 SphericalCoordinatesToEuclidean( Vec3 ThetaPhiR ) {
    return ThetaPhiR[2] * Vec3( cos(ThetaPhiR[0]) * cos(ThetaPhiR[1]) , sin(ThetaPhiR[0]) * cos(ThetaPhiR[1]) , sin(ThetaPhiR[1]) );
}
static
Vec3 SphericalCoordinatesToEuclidean( float theta , float phi ) {
    return Vec3( cos(theta) * cos(phi) , sin(theta) * cos(phi) , sin(phi) );
}

static
Vec3 EuclideanCoordinatesToSpherical( Vec3 xyz ) {
    float R = xyz.length();
    float phi = asin( xyz[2] / R );
    float theta = atan2( xyz[1] , xyz[0] );
    return Vec3( theta , phi , R );
}



class Sphere : public Mesh {
public:
    Vec3 m_center;
    float m_radius;

    Sphere() : Mesh() {}
    Sphere(Vec3 c , float r) : Mesh() , m_center(c) , m_radius(r) {}

    void build_arrays(){
        unsigned int nTheta = 20 , nPhi = 20;
        positions_array.resize(3 * nTheta * nPhi );
        normalsArray.resize(3 * nTheta * nPhi );
        uvs_array.resize(2 * nTheta * nPhi );
        for( unsigned int thetaIt = 0 ; thetaIt < nTheta ; ++thetaIt ) {
            float u = (float)(thetaIt) / (float)(nTheta-1);
            float theta = u * 2 * M_PI;
            for( unsigned int phiIt = 0 ; phiIt < nPhi ; ++phiIt ) {
                unsigned int vertexIndex = thetaIt + phiIt * nTheta;
                float v = (float)(phiIt) / (float)(nPhi-1);
                float phi = - M_PI/2.0 + v * M_PI;
                Vec3 xyz = SphericalCoordinatesToEuclidean( theta , phi );
                positions_array[ 3 * vertexIndex + 0 ] = m_center[0] + m_radius * xyz[0];
                positions_array[ 3 * vertexIndex + 1 ] = m_center[1] + m_radius * xyz[1];
                positions_array[ 3 * vertexIndex + 2 ] = m_center[2] + m_radius * xyz[2];
                normalsArray[ 3 * vertexIndex + 0 ] = xyz[0];
                normalsArray[ 3 * vertexIndex + 1 ] = xyz[1];
                normalsArray[ 3 * vertexIndex + 2 ] = xyz[2];
                uvs_array[ 2 * vertexIndex + 0 ] = u;
                uvs_array[ 2 * vertexIndex + 1 ] = v;
            }
        }
        triangles_array.clear();
        for( unsigned int thetaIt = 0 ; thetaIt < nTheta - 1 ; ++thetaIt ) {
            for( unsigned int phiIt = 0 ; phiIt < nPhi - 1 ; ++phiIt ) {
                unsigned int vertexuv = thetaIt + phiIt * nTheta;
                unsigned int vertexUv = thetaIt + 1 + phiIt * nTheta;
                unsigned int vertexuV = thetaIt + (phiIt+1) * nTheta;
                unsigned int vertexUV = thetaIt + 1 + (phiIt+1) * nTheta;
                triangles_array.push_back( vertexuv );
                triangles_array.push_back( vertexUv );
                triangles_array.push_back( vertexUV );
                triangles_array.push_back( vertexuv );
                triangles_array.push_back( vertexUV );
                triangles_array.push_back( vertexuV );
            }
        }
    }


    RaySphereIntersection intersect(const Ray &ray) const {
        RaySphereIntersection intersection;
        intersection.intersectionExists = false ; 

        // On récupère les informations de la sphère et du rayon
        Vec3 rayDirection = ray.direction() ;
        Vec3 rayOrigin = ray.origin() ;
        Vec3 center = m_center ;
        float radius = m_radius ;
        Vec3 centerToRay = rayOrigin - center ;

        // Équation de l'intersection : t2 d.d + 2t d.(o-c) + ||o-c||² – r² = 0
        float a = Vec3::dot(rayDirection, rayDirection);
        float b = 2 * Vec3::dot(rayDirection, centerToRay);
        float c = centerToRay.squareLength() - radius*radius ;
        // Calcul du discriminant (si négatif, pas d'intersection)
        float delta = b*b - 4.0f*a*c ;
        if(delta < 0) return intersection ;

        // Calcul des points d'intersection entre le rayon et la sphère
        float root1 = (-b-sqrt(delta))/(2.0f*a) ;
        float root2 = (-b+sqrt(delta))/(2.0f*a) ;
        float nearRoot = (root1 < root2) ? root1 : root2 ;
        float farRoot = (root1 > root2) ? root1 : root2 ;

        // On vérifie que les racines soient positives
        float t = -1.0f ;
        if(nearRoot > 0.0f){
            t = nearRoot ;
        }else if(farRoot > 0.0f){
            t = farRoot ;
        }else{
            return intersection ;
        }
        intersection.t = t ;
        intersection.intersectionExists = true ;

        // Calcul des coordonnées 3D des points d'intersection
        Vec3 nearPoint = rayOrigin + intersection.t * rayDirection ;
        Vec3 farPoint = rayOrigin + farRoot * rayDirection ;
        intersection.intersection = nearPoint ;
        intersection.secondintersection = farPoint ;

        // Calcul de phi et theta
        Vec3 sphericalCoordinates = EuclideanCoordinatesToSpherical(intersection.intersection);
        intersection.theta = sphericalCoordinates[0];
        intersection.phi = sphericalCoordinates[1];

        // Calcul de la normale
        Vec3 normal = intersection.intersection - center ;
        normal.normalize() ;
        intersection.normal = normal ;

        return intersection;
    }
};
#endif
