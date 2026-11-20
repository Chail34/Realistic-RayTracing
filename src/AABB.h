#ifndef AABB_H
#define AABB_H

#include "Vec3.h"
#include <cfloat>

struct AABB {

    // Points de la boite englobante
    Vec3 min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX) ;
    Vec3 max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX) ;

    // Aggrandir la boite englobante
    void expand(const Vec3 & point){
        for(int i = 0 ; i < 3 ; i++){
            min[i] = std::min(min[i], point[i]) ;
            max[i] = std::max(max[i], point[i]) ;
        }
    }

    // Récupérer l'axe le plus long
    // x = 0 ; y = 1 ; z = 2
    int longestAxis() const {
        Vec3 dist = max - min ;
        if(dist[0] > dist[1] && dist[0] > dist[2]) return 0 ;
        if(dist[1] > dist[2]) return 1 ;
        return 2 ;
    }

};

#endif