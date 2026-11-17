#ifndef TRIANGLE_H
#define TRIANGLE_H
#include "Vec3.h"
#include "Ray.h"
#include "Plane.h"

struct RayTriangleIntersection{
    bool intersectionExists;
    float t;
    float w0,w1,w2;
    unsigned int tIndex;
    Vec3 intersection;
    Vec3 normal;
};

class Triangle {
private:
    Vec3 m_c[3] , m_normal;
    float area;
public:
    Triangle() {}
    Triangle( Vec3 const & c0 , Vec3 const & c1 , Vec3 const & c2 ) {
        m_c[0] = c0;
        m_c[1] = c1;
        m_c[2] = c2;
        updateAreaAndNormal();
    }
    void updateAreaAndNormal() {
        Vec3 nNotNormalized = Vec3::cross( m_c[1] - m_c[0] , m_c[2] - m_c[0] );
        float norm = nNotNormalized.length();
        m_normal = nNotNormalized / norm;
        area = norm / 2.f;
    }
    void setC0( Vec3 const & c0 ) { m_c[0] = c0; } // remember to update the area and normal afterwards!
    void setC1( Vec3 const & c1 ) { m_c[1] = c1; } // remember to update the area and normal afterwards!
    void setC2( Vec3 const & c2 ) { m_c[2] = c2; } // remember to update the area and normal afterwards!
    Vec3 const & normal() const { return m_normal; }
    Vec3 projectOnSupportPlane( Vec3 const & p ) const {
        Vec3 result;
        
        Vec3 triangleToPoint = p - m_c[0] ;

        float distance = Vec3::dot(triangleToPoint, m_normal) / m_normal.squareLength() ;
        result = p - distance * m_normal ;

        return result;
    }
    float squareDistanceToSupportPlane( Vec3 const & p ) const {
        float result;
        
        Vec3 pProjection = projectOnSupportPlane(p) ;
        Vec3 projectionToPoint = p - pProjection ;
        result = projectionToPoint.squareLength() ;

        return result;
    }
    float distanceToSupportPlane( Vec3 const & p ) const { return sqrt( squareDistanceToSupportPlane(p) ); }
    bool isParallelTo( Line const & L ) const {
        bool result;
        
        result = fabs(Vec3::dot(L.direction(), m_normal)) < 1e-6 ;;

        return result;
    }
    Vec3 getIntersectionPointWithSupportPlane( Line const & L ) const {
        Vec3 result;
        
        if(isParallelTo(L)){return result ;}

        Vec3 lineDirection = L.direction() ;
        Vec3 lineOrigin = L.origin() ;
        Vec3 lineToCorner = m_c[0] - lineOrigin ;

        // Calcul de la distance au plan
        float denominator = Vec3::dot(lineDirection, m_normal);
        if(fabs(denominator) < 1e-4){ return result ;}
        float t = Vec3::dot(lineToCorner, m_normal)/denominator ;
        result = lineOrigin + t * lineDirection ;

        return result;
    }
    void computeBarycentricCoordinates( Vec3 const & p , float & alpha , float & beta , float & gamma ) const {
        
        Vec3 pProjection = projectOnSupportPlane(p) ;

        // Calcul des vecteurs
        Vec3 v0 = m_c[1] - m_c[0] ; // Arête c0-c1
        Vec3 v1 = m_c[2] - m_c[0] ; // Arête c0-c2
        Vec3 v2 = pProjection - m_c[0] ; // Vecteur c0-pointProjeté

        // Calcul des distances
        float dot00 = Vec3::dot(v0,v0) ;
        float dot01 = Vec3::dot(v0,v1) ;
        float dot11 = Vec3::dot(v1,v1) ;
        float dot20 = Vec3::dot(v2,v0) ;
        float dot21 = Vec3::dot(v2,v1) ;

        // Calcul des coordonnées barycentriques
        float denominator = (dot00 * dot11) - (dot01 * dot01) ;
        beta  = (dot11 * dot20 - dot01 * dot21) / denominator ;
        gamma = (dot00 * dot21 - dot01 * dot20) / denominator ;
        alpha = 1.0f - beta - gamma ;
    }

    RayTriangleIntersection getIntersection( Ray const & ray ) const {

        RayTriangleIntersection result;
        Vec3 rayDirection = ray.direction() ;
        Vec3 rayOrigin = ray.origin() ;
        const float epsilon = 0.001f ;

        // 1) check that the ray is not parallel to the triangle:

        if(isParallelTo(ray)){
            result.intersectionExists = false ;
            return result ;
        }

        // 2) check that the triangle is "in front of" the ray:

        Vec3 rayToTriangle = m_c[0] - rayOrigin ;
        float t = Vec3::dot(rayToTriangle, m_normal) / Vec3::dot(rayDirection, m_normal);
        if(t < epsilon){
            result.intersectionExists = false ;
            return result ;
        }

        // 3) check that the intersection point is inside the triangle:
        // CONVENTION: compute u,v such that p = w0*c0 + w1*c1 + w2*c2, check that 0 <= w0,w1,w2 <= 1

        Vec3 intersectionPoint = getIntersectionPointWithSupportPlane(ray) ;
        float alpha, beta, gamma ;
        computeBarycentricCoordinates(intersectionPoint, alpha, beta, gamma);

        if(alpha < epsilon || beta < epsilon || gamma < epsilon){
            result.intersectionExists = false ;
            return result ;
        }

        // 4) Finally, if all conditions were met, then there is an intersection! :

        result.intersectionExists = true ;
        result.t = t ;
        result.intersection = intersectionPoint ;
        result.normal = m_normal ;
        result.w0 = alpha ;
        result.w1 = beta ;
        result.w2 = gamma ;

        return result;
    }
};
#endif
