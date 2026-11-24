#ifndef SCENE_H
#define SCENE_H

#include <vector>
#include <string>
#include "Mesh.h"
#include "Sphere.h"
#include "Square.h"


#include <GL/glut.h>


enum LightType {
    LightType_Spherical,
    LightType_Quad
};


struct Light {
    Vec3 material;
    bool isInCamSpace;
    LightType type;

    Vec3 pos;
    float radius;

    Mesh quad;

    float powerCorrection;

    Light() : powerCorrection(1.0) {}

};

struct RaySceneIntersection{
    bool intersectionExists;
    unsigned int typeOfIntersectedObject; 
    unsigned int objectIndex;
    float t; // "Distance" depuis la caméra
    RayTriangleIntersection rayTriangleIntersection;
    RaySphereIntersection raySphereIntersection;
    RaySquareIntersection raySquareIntersection;
    RaySceneIntersection() : intersectionExists(false) , t(FLT_MAX) {}
};
// typeOfIntersectedObject :
// - 0 : Mesh 
// - 1 : Sphère 
// - 2 : Carré 
// - 3 : Lumière



class Scene {
    std::vector< Mesh > meshes;
    std::vector< Sphere > spheres;
    std::vector< Square > squares;
    std::vector< Light > lights;

public:


    Scene() {
    }

    void draw() {
        // iterer sur l'ensemble des objets, et faire leur rendu :
        for( unsigned int It = 0 ; It < meshes.size() ; ++It ) {
            Mesh const & mesh = meshes[It];
            mesh.draw();
        }
        for( unsigned int It = 0 ; It < spheres.size() ; ++It ) {
            Sphere const & sphere = spheres[It];
            sphere.draw();
        }
        for( unsigned int It = 0 ; It < squares.size() ; ++It ) {
            Square const & square = squares[It];
            square.draw();
        }
    }

// ========== Méthode de calcul de rayon ==========

    RaySceneIntersection computeIntersection(Ray const & ray) 
    {
        RaySceneIntersection result;

        // On récupère le nombre d'objet de la scène
        unsigned int nb_mesh = meshes.size() ;
        unsigned int nb_sphere = spheres.size() ;
        unsigned int nb_square = squares.size() ;

        // Début de la détection de l'intersection
        result.intersectionExists = false ;
        result.t = FLT_MAX ;

        // Parcours de tous les maillages
        for(unsigned int i = 0 ; i < nb_mesh ; i++){
           RayTriangleIntersection triangleIntersection = meshes[i].intersect(ray) ;
           if(triangleIntersection.intersectionExists && (triangleIntersection.t < result.t)){
            result.intersectionExists = true ;
            result.typeOfIntersectedObject = 0 ;
            result.objectIndex = i ;
            result.t = triangleIntersection.t ;
            result.rayTriangleIntersection = triangleIntersection ;
           }
        }

        // Parcours de toutes les sphères
        for(unsigned int i = 0 ; i < nb_sphere ; i++){
           RaySphereIntersection sphereIntersection = spheres[i].intersect(ray) ;
           if(sphereIntersection.intersectionExists && (sphereIntersection.t < result.t)){
            result.intersectionExists = true ;
            result.typeOfIntersectedObject = 1 ;
            result.objectIndex = i ;
            result.t = sphereIntersection.t ;
            result.raySphereIntersection = sphereIntersection ;
           }
        }
        // Parcours de tous les plans
        for(unsigned int i = 0 ; i < nb_square ; i++){
           RaySquareIntersection squareIntersection = squares[i].intersect(ray) ;
           if(squareIntersection.intersectionExists && (squareIntersection.t < result.t)){
            result.intersectionExists = true ;
            result.typeOfIntersectedObject = 2 ;
            result.objectIndex = i ;
            result.t = squareIntersection.t ;
            result.raySquareIntersection = squareIntersection ;
           }
        }

        return result;
    }


    Vec3 rayTraceRecursive( Ray ray , int NRemainingBounces )
    {    
        if(NRemainingBounces > 0){
            Vec3 color = Vec3(1.0f,1.0f,1.0f) ;
            RaySceneIntersection intersection = computeIntersection(ray);
            const float epsilon = 0.001f ;

            // Paramètres de lumière et d'ombre
            Material objectMaterial ;
            Vec3 normal ;
            Vec3 intersectionPoint ;
            Vec3 viewDirection = ray.direction() ;
            Vec3 lightPosition = lights[0].pos ;
            float n1 = 1.0f ;

            if(intersection.intersectionExists){

                unsigned int typeOfObject = intersection.typeOfIntersectedObject ;
                unsigned int objectIndex = intersection.objectIndex ;

                if(typeOfObject == 0){
                    objectMaterial = meshes[objectIndex].material ;
                    normal = intersection.rayTriangleIntersection.normal ;
                    intersectionPoint = intersection.rayTriangleIntersection.intersection ;
                    
                    color = lightIntensity(intersectionPoint, lightPosition, normal) * phongLight(objectMaterial, normal, intersectionPoint, viewDirection);
                }

                if(typeOfObject == 1){
                    objectMaterial = spheres[objectIndex].material ;
                    normal = intersection.raySphereIntersection.normal ;
                    intersectionPoint = intersection.raySphereIntersection.intersection ;
                    
                    color = lightIntensity(intersectionPoint, lightPosition, normal) * phongLight(objectMaterial, normal, intersectionPoint, viewDirection);
                
                    // Cas Miroir
                    if(objectMaterial.type == Material_Mirror && NRemainingBounces > 1){
                        Vec3 reflectionDirection = reflection(ray, normal);
                        Vec3 newRayOrigin = intersectionPoint + normal * epsilon ;
                        Ray reflectionRay = Ray(newRayOrigin, reflectionDirection) ;
                        Vec3 reflectionColor = rayTraceRecursive(reflectionRay, NRemainingBounces-1);
                        color = reflectionColor ;
                    }

                    // Cas Verre
                    if(objectMaterial.type == Material_Glass && NRemainingBounces > 1){
                        Vec3 refractionDirection = refraction(ray, normal, n1, objectMaterial.index_medium);
                        // Si la refraction est dans le bon sens on l'applique
                        if(refractionDirection.squareLength() > 0.0f){
                            Vec3 refractionPoint = intersectionPoint + refractionDirection * epsilon ;
                            Ray refractionRay = Ray(refractionPoint, refractionDirection);
                            Vec3 refractionColor = rayTraceRecursive(refractionRay, NRemainingBounces - 1);
                            color = refractionColor ; 
                        }
                    }
                
                }
                if(typeOfObject == 2){
                    objectMaterial = squares[objectIndex].material ;
                    normal = intersection.raySquareIntersection.normal ;
                    intersectionPoint = intersection.raySquareIntersection.intersection ;

                    color = lightIntensity(intersectionPoint, lightPosition, normal) * phongLight(objectMaterial, normal, intersectionPoint, viewDirection);
                
                    // Cas Miroir
                    if(objectMaterial.type == Material_Mirror && NRemainingBounces > 1){
                        Vec3 reflectionDirection = reflection(ray, normal);
                        Vec3 newRayOrigin = intersectionPoint + normal * epsilon ;
                        Ray reflectionRay = Ray(newRayOrigin, reflectionDirection) ;
                        Vec3 reflectionColor = rayTraceRecursive(reflectionRay, NRemainingBounces-1);
                        color = reflectionColor ;
                    }

                    // Cas Verre
                    if(objectMaterial.type == Material_Glass && NRemainingBounces > 1){
                        Vec3 refractionDirection = refraction(ray, normal, n1, objectMaterial.index_medium);
                        // Si la refraction est dans le bon sens on l'applique
                        if(refractionDirection.squareLength() > 0.0f){
                            Vec3 refractionPoint = intersectionPoint + refractionDirection * epsilon ;
                            Ray refractionRay = Ray(refractionPoint, refractionDirection);
                            Vec3 refractionColor = rayTraceRecursive(refractionRay, NRemainingBounces - 1);
                            color = refractionColor ; 
                        }
                    }
                }

                return color;
            }else{
                return Vec3(0.0f, 0.0f, 0.0f) ;
            }

        }else{
            return Vec3(0.0f, 0.0f, 0.0f) ;
        }
    }

    Vec3 rayTrace( Ray const & rayStart ) 
    {
        Vec3 color;
        color = rayTraceRecursive(rayStart, 4) ;
        return color;
    }

// ========== Méthodes de calcul de lumière ==========

    Vec3 phongLight(Material material, Vec3 normal, Vec3 intersectionPoint, Vec3 viewDirection)
    {

        // Coefficients du matériau
        Vec3 Kambiant  = material.ambient_material ;
        Vec3 Kdiffuse  = material.diffuse_material ;
        Vec3 Kspecular = material.specular_material ;
        double shininess = material.shininess ;

        // Direction du rayon vers la caméra
        Vec3 viewDirectionToCamera = -1.0f * viewDirection ;
        viewDirectionToCamera.normalize() ;

        // Lumière ambiante
        Vec3 Isa = lights[0].material ;
        Vec3 Ia = Vec3::compProduct(Isa, Kambiant) ; // Lumière réfléchie

        // Lumière diffuse et spéculaire
        Vec3 Id = Vec3(0.0f, 0.0f, 0.0f);
        Vec3 Is = Vec3(0.0f, 0.0f, 0.0f);
        for(Light light : lights){ // Parcours de toutes les lumières de la scène
            Vec3 lightMaterial = light.material ;
            Vec3 lightVector = light.pos - intersectionPoint ;
            lightVector.normalize() ;
            
            // Calcul du produit scalaire
            float dotProduct = Vec3::dot(lightVector, normal) ;

            // Lumière diffuse
            Id += Vec3::compProduct(lightMaterial, Kdiffuse) * dotProduct ;

            // Lumière spéculaire
            Vec3 lightreflection = 2 * dotProduct * normal - lightVector ;
            Is += Vec3::compProduct(lightMaterial, Kspecular) * pow(Vec3::dot(lightreflection, viewDirectionToCamera), shininess) ;
        }

        return Ia + Id + Is ;
    }

    float lightIntensity(Vec3 intersectionPoint, Vec3 lightPosition, Vec3 normal)
    {
        Vec3 lightDirection = intersectionPoint - lightPosition ;
        lightDirection.normalize() ;
        unsigned int maxBounce = 10 ;

        // On va lancer plusieurs rayons à partir de notre source lumineuse pour calculer l'ombre
        RaySceneIntersection lightIntersection ;
        const float epsilon = 0.001f ;
        const float glassTransmission = 0.9f ;
        unsigned int nbRayLaunched = 10 ;
        Vec3 newRayOrigin = intersectionPoint + normal * epsilon ;
        float lightTransmission = 0.0f ;

        // Echantillonnage du carré lumineux pour calculer l'ombrage
        for(unsigned int i = 0 ; i < nbRayLaunched ; i++){
            // Offset pour obtenir les ombres douces
            Vec3 rayOffset = Vec3(float(std::rand()%100)/100.f - 0.5f,
                                  float(std::rand()%100)/100.f - 0.5f,
                                  float(std::rand()%100)/100.f - 0.5f) ;
            Vec3 rayPosition = lightPosition + rayOffset ;
            Vec3 rayDirection = rayPosition - intersectionPoint ;
            float rayDistance = rayDirection.length() ;
            rayDirection.normalize() ;

            // Suivi du rayon d'ombre (ombre -> lumière)
            float transmission = 1.0f ;
            Vec3 origin = newRayOrigin ;
            float remainingDistance = rayDistance ;

            // Si l'objet rencontré est en verre, on génère une ombre plus légère
            for(unsigned int j = 0 ; j < maxBounce ; j++){

                // Calcul de l'intersection du i-ème rayon
                Ray newRayLight = Ray(origin, rayDirection) ;
                lightIntersection = computeIntersection(newRayLight) ;

                // Verre déjà passé
                if(!lightIntersection.intersectionExists || lightIntersection.t >= remainingDistance) break ;
            
                // Obstacle pas en verre
                if(materialOf(lightIntersection).type != Material_Glass){
                    transmission = 0.0f ;
                    break ;
                }

                // Gestion passage dans le verre
                transmission *= glassTransmission ;
                float distanceInGlass = lightIntersection.t + epsilon ;
                origin = origin + distanceInGlass * rayDirection ;
                remainingDistance -= distanceInGlass ;
            }

            lightTransmission += transmission ;

        }

        return lightTransmission / float(nbRayLaunched) ;
    }

    Vec3 reflection(Ray ray, Vec3 normal){
        Vec3 rayDirection = ray.direction() ;
        float reflectionAngle = Vec3::dot(rayDirection, normal);
        Vec3 reflectionDirection = rayDirection - 2 * reflectionAngle * normal ;
        return reflectionDirection ;
    }

    Vec3 refraction(Ray ray, Vec3 normal, float n1, float n2){

        Vec3 rayDirection = ray.direction() ;
        rayDirection.normalize();
        normal.normalize() ;

        // Snell-Descartes
        float cosRayDirection = Vec3::dot(rayDirection, normal);
        float eta = n1 / n2 ;

        // On vérifie la direction du rayon
        if(cosRayDirection > 0.0f){
            normal = Vec3(0,0,0) - normal ;
            eta = n2 / n1 ;
        }else{
            cosRayDirection = -cosRayDirection ;
        }

        float k = 1.0f - eta * eta * (1.0f - cosRayDirection * cosRayDirection);

        // Réflexion total interne
        if(k < 0.0f){return Vec3(0,0,0) ;}

        Vec3 refractionDirection = eta * rayDirection + (eta * cosRayDirection - sqrt(k)) * normal ;
        refractionDirection.normalize();
        return refractionDirection ;
    }

// ========== Helpers ==========

Material materialOf(RaySceneIntersection raySceneIntersection)
{
    switch (raySceneIntersection.typeOfIntersectedObject){
    case 0: return meshes[raySceneIntersection.objectIndex].material ;
    case 1: return spheres[raySceneIntersection.objectIndex].material ;
    default: return squares[raySceneIntersection.objectIndex].material ;
    }
}

// ========== Définitions des scènes ==========

    void setup_single_sphere() {
        meshes.clear();
        spheres.clear();
        squares.clear();
        lights.clear();

        {
            lights.resize( lights.size() + 1 );
            Light & light = lights[lights.size() - 1];
            light.pos = Vec3(-5,5,5);
            light.radius = 2.5f;
            light.powerCorrection = 2.f;
            light.type = LightType_Spherical;
            light.material = Vec3(1,1,1);
            light.isInCamSpace = false;
        }
        {
            spheres.resize( spheres.size() + 1 );
            Sphere & s = spheres[spheres.size() - 1];
            s.m_center = Vec3(1. , 0. , 0.);
            s.m_radius = 1.f;
            s.build_arrays();
            s.material.type = Material_Diffuse_Blinn_Phong;
            s.material.diffuse_material = Vec3( 1.,0.,0 );
            s.material.specular_material = Vec3( 0.2,0.2,0.2 );
            s.material.shininess = 20;
        }
        {
            spheres.resize( spheres.size() + 1 );
            Sphere & s = spheres[spheres.size() - 1];
            s.m_center = Vec3(-1. , 0. , 0.);
            s.m_radius = 0.5f;
            s.build_arrays();
            s.material.type = Material_Diffuse_Blinn_Phong;
            s.material.diffuse_material = Vec3( 0.,1.,0 );
            s.material.specular_material = Vec3( 0.2,0.2,0.2 );
            s.material.shininess = 20;
        }
    }

    void setup_single_square() {
        meshes.clear();
        spheres.clear();
        squares.clear();
        lights.clear();

        {
            lights.resize( lights.size() + 1 );
            Light & light = lights[lights.size() - 1];
            light.pos = Vec3(-5,5,5);
            light.radius = 2.5f;
            light.powerCorrection = 2.f;
            light.type = LightType_Spherical;
            light.material = Vec3(1,1,1);
            light.isInCamSpace = false;
        }

        {
            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 0., 1., 1. );
            s.material.specular_material = Vec3( 0.8,0.8,0.8 );
            s.material.shininess = 20;
        }
    }

    void setup_cornell_box(){
        meshes.clear();
        spheres.clear();
        squares.clear();
        lights.clear();

        {
            lights.resize( lights.size() + 1 );
            Light & light = lights[lights.size() - 1];
            light.pos = Vec3( 0.0, 1.5, 0.0 );
            light.radius = 2.5f;
            light.powerCorrection = 2.f;
            light.type = LightType_Spherical;
            light.material = Vec3(1,1,1);
            light.isInCamSpace = false;
        }

        { //Back Wall
            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.scale(Vec3(2., 2., 1.));
            s.translate(Vec3(0., 0., -2.));
            s.build_arrays();
            s.material.diffuse_material = Vec3( 1.,1.,1. );
            s.material.specular_material = Vec3( 1.,1.,1. );
            s.material.shininess = 16;
            // s.material.type = Material_Mirror ;
        }

        { //Left Wall

            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.scale(Vec3(2., 2., 1.));
            s.translate(Vec3(0., 0., -2.));
            s.rotate_y(90);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 1.,0.,0. );
            s.material.specular_material = Vec3( 1.,0.,0. );
            s.material.shininess = 16;
        }

        { //Right Wall
            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(2., 2., 1.));
            s.rotate_y(-90);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 0.0,1.0,0.0 );
            s.material.specular_material = Vec3( 0.0,1.0,0.0 );
            s.material.shininess = 16;
        }

        { //Floor
            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(2., 2., 1.));
            s.rotate_x(-90);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 1.0,1.0,1.0 );
            s.material.specular_material = Vec3( 1.0,1.0,1.0 );
            s.material.shininess = 16;
        }

        { //Ceiling
            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(2., 2., 1.));
            s.rotate_x(90);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 1.0,1.0,1.0 );
            s.material.specular_material = Vec3( 1.0,1.0,1.0 );
            s.material.shininess = 16;
        }

        // { //Front Wall
        //     squares.resize( squares.size() + 1 );
        //     Square & s = squares[squares.size() - 1];
        //     s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
        //     s.translate(Vec3(0., 0., -2.));
        //     s.scale(Vec3(2., 2., 1.));
        //     s.rotate_y(180);
        //     s.build_arrays();
        //     s.material.diffuse_material = Vec3( 1.0,1.0,1.0 );
        //     s.material.specular_material = Vec3( 1.0,1.0,1.0 );
        //     s.material.shininess = 16;
        // }


        { //GLASS Sphere

            spheres.resize( spheres.size() + 1 );
            Sphere & s = spheres[spheres.size() - 1];
            s.m_center = Vec3(1.0, 1.25, 0.5);
            s.m_radius = 0.75f;
            s.build_arrays();
            s.material.type = Material_Glass;
            s.material.diffuse_material = Vec3( 1.,0.,0. );
            s.material.specular_material = Vec3( 1.,0.,0. );
            s.material.shininess = 16;
            s.material.transparency = 1.0;
            s.material.index_medium = 1.4;
        }


        { //MIRRORED Sphere
            spheres.resize( spheres.size() + 1 );
            Sphere & s = spheres[spheres.size() - 1];
            s.m_center = Vec3(-1.0, 0.25, -0.5);
            s.m_radius = 0.75f;
            s.build_arrays();
            s.material.type = Material_Mirror;
            s.material.diffuse_material = Vec3( 1.,1.,1. );
            s.material.specular_material = Vec3(  1.,1.,1. );
            s.material.shininess = 16;
            s.material.transparency = 0.;
            s.material.index_medium = 0.;
        }

    }

    void setup_mesh(){
        meshes.clear();
        spheres.clear();
        squares.clear();
        lights.clear();
    
        {
            lights.resize( lights.size() + 1 );
            Light & light = lights[lights.size() - 1];
            light.pos = Vec3( 0.0, 1.5, 1.0 );
            light.radius = 2.5f;
            light.powerCorrection = 2.f;
            light.type = LightType_Spherical;
            light.material = Vec3(1,1,1);
            light.isInCamSpace = false;
        }

        {
            meshes.resize( meshes.size() + 1 );
            Mesh & mesh = meshes[meshes.size() - 1];
            mesh.loadOFF( "data/mario.off" );
            mesh.centerAndScaleToUnit();
            mesh.scale(Vec3(1.5f, 1.5f, 1.5f));
            mesh.translate(Vec3(0.0f, -0.6f, 0.0f));
            mesh.build_arrays();
            mesh.material.diffuse_material = Vec3( 0.8,0.8,0.8 );
            mesh.material.specular_material = Vec3( 0.8,0.8,0.8 );
            mesh.material.ambient_material = Vec3(1.0, 0.0, 0.0);
            mesh.material.shininess = 20;
        }

        { //Back Wall
            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.scale(Vec3(2., 2., 1.));
            s.translate(Vec3(0., 0., -2.));
            s.build_arrays();
            s.material.diffuse_material = Vec3( 1.,1.,1. );
            s.material.specular_material = Vec3( 1.,1.,1. );
            s.material.shininess = 16;
            // s.material.type = Material_Mirror ;
        }

        { //Left Wall

            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.scale(Vec3(2., 2., 1.));
            s.translate(Vec3(0., 0., -2.));
            s.rotate_y(90);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 1.,0.,0. );
            s.material.specular_material = Vec3( 1.,0.,0. );
            s.material.shininess = 16;
        }

        { //Right Wall
            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(2., 2., 1.));
            s.rotate_y(-90);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 0.0,1.0,0.0 );
            s.material.specular_material = Vec3( 0.0,1.0,0.0 );
            s.material.shininess = 16;
        }

        { //Floor
            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(2., 2., 1.));
            s.rotate_x(-90);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 1.0,1.0,1.0 );
            s.material.specular_material = Vec3( 1.0,1.0,1.0 );
            s.material.shininess = 16;
        }

        { //Ceiling
            squares.resize( squares.size() + 1 );
            Square & s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(2., 2., 1.));
            s.rotate_x(90);
            s.build_arrays();
            s.material.diffuse_material = Vec3( 1.0,1.0,1.0 );
            s.material.specular_material = Vec3( 1.0,1.0,1.0 );
            s.material.shininess = 16;
        }

    }

};



#endif
