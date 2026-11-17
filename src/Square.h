#ifndef SQUARE_H
#define SQUARE_H
#include "Vec3.h"
#include <vector>
#include "Mesh.h"
#include <cmath>

struct RaySquareIntersection{
    bool intersectionExists;
    float t;
    float u,v;
    Vec3 intersection;
    Vec3 normal;
};


class Square : public Mesh {
public:
    Vec3 m_normal;
    Vec3 m_bottom_left;
    Vec3 m_right_vector;
    Vec3 m_up_vector;

    Square() : Mesh() {}
    Square(Vec3 const & bottomLeft , Vec3 const & rightVector , Vec3 const & upVector , float width=1. , float height=1. ,
           float uMin = 0.f , float uMax = 1.f , float vMin = 0.f , float vMax = 1.f) : Mesh() {
        setQuad(bottomLeft, rightVector, upVector, width, height, uMin, uMax, vMin, vMax);
    }

    void setQuad( Vec3 const & bottomLeft , Vec3 const & rightVector , Vec3 const & upVector , float width=1. , float height=1. ,
                  float uMin = 0.f , float uMax = 1.f , float vMin = 0.f , float vMax = 1.f) {
        m_right_vector = rightVector;
        m_up_vector = upVector;
        m_normal = Vec3::cross(rightVector , upVector);
        m_bottom_left = bottomLeft;

        m_normal.normalize();
        m_right_vector.normalize();
        m_up_vector.normalize();

        m_right_vector = m_right_vector*width;
        m_up_vector = m_up_vector*height;

        vertices.clear();
        vertices.resize(4);
        vertices[0].position = bottomLeft;                                      vertices[0].u = uMin; vertices[0].v = vMin;
        vertices[1].position = bottomLeft + m_right_vector;                     vertices[1].u = uMax; vertices[1].v = vMin;
        vertices[2].position = bottomLeft + m_right_vector + m_up_vector;       vertices[2].u = uMax; vertices[2].v = vMax;
        vertices[3].position = bottomLeft + m_up_vector;                        vertices[3].u = uMin; vertices[3].v = vMax;
        vertices[0].normal = vertices[1].normal = vertices[2].normal = vertices[3].normal = m_normal;
        triangles.clear();
        triangles.resize(2);
        triangles[0][0] = 0;
        triangles[0][1] = 1;
        triangles[0][2] = 2;
        triangles[1][0] = 0;
        triangles[1][1] = 2;
        triangles[1][2] = 3;


    }

    RaySquareIntersection intersect(const Ray &ray) const {
        RaySquareIntersection intersection;
        intersection.intersectionExists = false ; 
        
        // Trouver sur le discord Imagine
        // en fait vu que la prof fait des transfos sur les planes sans modifier les variables qu'on utilise pour la collision ça fait des trucs bizarres
        // du coup là je redéfinis toutes les variables en prenant les points du mesh au lieu des trucs qu'on a de base
        Vec3 m_bottom_left = vertices[0].position;
        Vec3 m_right_vector = vertices[1].position - vertices[0].position;
        Vec3 m_up_vector = vertices[3].position - vertices[0].position;
        Vec3 m_normal = Vec3::cross(m_right_vector, m_up_vector);
        m_normal.normalize();

        // On récupère les informations du plan et du rayon
        Vec3 rayDirection = ray.direction();
        Vec3 rayOrigin = ray.origin() ;
        Vec3 normal = m_normal ;
        Vec3 a = m_bottom_left ; // Un point du plan
        float dist = Vec3::dot(a, normal) ;

        // Calcul de la distance au plan
        float denominator = Vec3::dot(rayDirection, normal);
        if(fabs(denominator) < 1e-4){ return intersection ;}
        float t = (dist - Vec3::dot(rayOrigin, normal))/denominator ;
        if(t < 1e-4f){ return intersection ;}
        intersection.t = t ;

        // Calcul des coordonnées 3D du point d'intersection
        Vec3 intersectionPoint = rayOrigin + t * rayDirection ;
        intersection.intersection = intersectionPoint ;

        // Calcul des coordonnées barycentriques locales (u et v)
        Vec3 planeCoordinates = intersectionPoint - m_bottom_left ;
        float u = Vec3::dot(planeCoordinates, m_right_vector) / Vec3::dot(m_right_vector,m_right_vector) ;
        float v = Vec3::dot(planeCoordinates, m_up_vector) / Vec3::dot(m_up_vector,m_up_vector) ;
        if (u < 0.0f || u > 1.0f || v < 0.0f || v > 1.0f){ return intersection ; }

        intersection.intersectionExists = true ;
        intersection.u = u ;
        intersection.v = v ;

        // Calcul de la normale
        intersection.normal = m_normal ;

        return intersection;
    }
};
#endif // SQUARE_H
