#ifndef TRIANGLE_H
#define TRIANGLE_H


#include "Homogeneous4.h"
#include "Ray.h"
#include "Material.h"
#include "RGBAImage.h"
#include "RenderParameters.h"


class Triangle
{


public:
    int triangle_id;
    Homogeneous4 verts[3];
    Homogeneous4 normals[3];
    Homogeneous4 colors[3];
    Cartesian3 uvs[3];

    Material *shared_material;

    Triangle();
    void validate(int id);
    bool isValid(); 
    float intersect(Ray r);
    Cartesian3 baricentric(Cartesian3 o);
    Homogeneous4 Blinn_PhongShading(Cartesian3 intersection, RenderParameters* renderParameters, Matrix4 matrix);
    Homogeneous4 Blinn_PhongShading1(Cartesian3 intersection, Light* light, Matrix4 matrix);

};

#endif // TRIANGLE_H
