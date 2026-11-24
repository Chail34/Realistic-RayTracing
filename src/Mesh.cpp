#include "Mesh.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

// Lit la prochaine ligne utile du fichier : ignore les lignes vides et les
// commentaires (# ...), et retire les fins de ligne Windows (\r).
static bool nextDataLine (std::istream & in, std::string & line) {
    while (std::getline (in, line)) {
        std::size_t comment = line.find ('#');
        if (comment != std::string::npos)
            line.erase (comment);
        line.erase (std::remove (line.begin (), line.end (), '\r'), line.end ());
        if (line.find_first_not_of (" \t") != std::string::npos)
            return true;
    }
    return false;
}

// Chargement d'un fichier OFF quelconque :
// - en-tête OFF / COFF / NOFF / CNOFF / STOFF..., avec les nombres sur la même
//   ligne ou sur la suivante ;
// - sommets suivis de données supplémentaires (normales, couleurs, uv) : seules
//   les 3 premières valeurs sont lues ;
// - faces à n sommets (triangles, quads, polygones) triangulées en éventail ;
// - couleurs de face éventuelles en fin de ligne ignorées.
void Mesh::loadOFF (const std::string & filename) {
    std::ifstream in (filename.c_str ());
    if (!in) {
        std::cerr << "loadOFF : impossible d'ouvrir " << filename << std::endl;
        exit (EXIT_FAILURE);
    }

    vertices.clear ();
    triangles.clear ();

    std::string line;
    if (!nextDataLine (in, line)) {
        std::cerr << "loadOFF : fichier vide " << filename << std::endl;
        exit (EXIT_FAILURE);
    }

    // En-tête
    std::istringstream header (line);
    std::string keyword;
    header >> keyword;
    if (keyword.size () < 3 || keyword.compare (keyword.size () - 3, 3, "OFF") != 0) {
        std::cerr << "loadOFF : en-tête OFF invalide dans " << filename << std::endl;
        exit (EXIT_FAILURE);
    }
    if (keyword.find ('4') != std::string::npos || keyword.find ('n') != std::string::npos) {
        std::cerr << "loadOFF : variante " << keyword << " (dimension != 3) non supportée" << std::endl;
        exit (EXIT_FAILURE);
    }

    long sizeV = -1, sizeF = -1;
    if (!(header >> sizeV >> sizeF)) {
        if (!nextDataLine (in, line)) {
            std::cerr << "loadOFF : nombres de sommets/faces manquants dans " << filename << std::endl;
            exit (EXIT_FAILURE);
        }
        std::istringstream counts (line);
        counts >> sizeV >> sizeF;
    }
    if (sizeV <= 0 || sizeF < 0) {
        std::cerr << "loadOFF : nombres de sommets/faces invalides dans " << filename << std::endl;
        exit (EXIT_FAILURE);
    }

    // Sommets
    vertices.resize (sizeV);
    for (long i = 0; i < sizeV; i++) {
        if (!nextDataLine (in, line)) {
            std::cerr << "loadOFF : fichier tronqué (sommet " << i << ") dans " << filename << std::endl;
            exit (EXIT_FAILURE);
        }
        std::istringstream vertexLine (line);
        vertexLine >> vertices[i].position;
    }

    // Faces
    triangles.reserve (sizeF);
    unsigned int skipped = 0, degenerate = 0;
    std::vector<long> face;
    for (long i = 0; i < sizeF; i++) {
        if (!nextDataLine (in, line)) {
            std::cerr << "loadOFF : fichier tronqué (face " << i << ") dans " << filename << std::endl;
            break;
        }
        std::istringstream faceLine (line);
        long n = 0;
        faceLine >> n;
        face.resize (n > 0 ? n : 0);
        bool valid = n >= 3;
        for (long j = 0; j < n && valid; j++)
            valid = (faceLine >> face[j]) && face[j] >= 0 && face[j] < sizeV;
        if (!valid) {
            skipped++;
            continue;
        }
        // Triangulation en éventail (v0, vj, vj+1)
        for (long j = 1; j + 1 < n; j++) {
            if (face[0] == face[j] || face[j] == face[j + 1] || face[0] == face[j + 1])
                continue; // triangle dégénéré (sommet répété)
            Vec3 e1 = vertices[face[j]].position - vertices[face[0]].position;
            Vec3 e2 = vertices[face[j + 1]].position - vertices[face[0]].position;
            // Test relatif (indépendant de l'échelle) : sin²(angle entre e1 et e2)
            // quasi nul => triangle plat, dont la normale deviendrait NaN après
            // centerAndScaleToUnit à cause des arrondis.
            float area2 = Vec3::cross (e1, e2).squareLength ();
            if (area2 <= 1e-8f * e1.squareLength () * e2.squareLength ()) {
                degenerate++;
                continue;
            }
            triangles.push_back (MeshTriangle (face[0], face[j], face[j + 1]));
        }
    }
    in.close ();

    if (skipped > 0)
        std::cerr << "loadOFF : " << skipped << " face(s) invalide(s) ignorée(s) dans " << filename << std::endl;
    if (degenerate > 0)
        std::cerr << "loadOFF : " << degenerate << " triangle(s) d'aire nulle ignoré(s) dans " << filename << std::endl;
}

void Mesh::recomputeNormals () {
    for (unsigned int i = 0; i < vertices.size (); i++)
        vertices[i].normal = Vec3 (0.0, 0.0, 0.0);
    for (unsigned int i = 0; i < triangles.size (); i++) {
        Vec3 e01 = vertices[triangles[i].v[1]].position -  vertices[triangles[i].v[0]].position;
        Vec3 e02 = vertices[triangles[i].v[2]].position -  vertices[triangles[i].v[0]].position;
        Vec3 n = Vec3::cross (e01, e02);
        n.normalize ();
        for (unsigned int j = 0; j < 3; j++)
            vertices[triangles[i].v[j]].normal += n;
    }
    for (unsigned int i = 0; i < vertices.size (); i++)
        vertices[i].normal.normalize ();
}

void Mesh::centerAndScaleToUnit () {
    Vec3 c(0,0,0);
    for  (unsigned int i = 0; i < vertices.size (); i++)
        c += vertices[i].position;
    c /= vertices.size ();
    float maxD = (vertices[0].position - c).length();
    for (unsigned int i = 0; i < vertices.size (); i++){
        float m = (vertices[i].position - c).length();
        if (m > maxD)
            maxD = m;
    }
    for  (unsigned int i = 0; i < vertices.size (); i++)
        vertices[i].position = (vertices[i].position - c) / maxD;
}
