#include "Triangle.h"
#include <math.h>
#include <algorithm>
#include <cmath>

const double PI = 3.141592653589793;

Triangle::Triangle()
{
    triangle_id = -1;
    shared_material= nullptr;
}


void Triangle::validate(int id){
    triangle_id = id;
}

bool Triangle::isValid(){
    return triangle_id != -1;
}


float Triangle::intersect(Ray r) {
    

    // use Möller-Trumbore
    float result = -1;
    float epsilon = 1e-4;
    Cartesian3 AB = verts[1].Point() - verts[0].Point();
    Cartesian3 AC = verts[2].Point() - verts[0].Point();
    Cartesian3 AO = r.origin - verts[0].Point();
    Cartesian3 vec1 = r.direction.cross(AC);
    Cartesian3 vec2 = AO.cross(AB);
    float se = 1.0f / vec1.dot(AB);
    float t = vec2.dot(AC) * se;
    float b1 = vec1.dot(AO) * se;
    float b2 = vec2.dot(r.direction) * se;
    if (t + epsilon > 0 && b1 + epsilon > 0 && b2 + epsilon > 0 && 1 - b1 - b2 + epsilon > 0)
        return t;
    return result;


}

Cartesian3 Triangle::baricentric(Cartesian3 o)
{
	//TODO: Input is the intersection between the ray and the triangle.

	Cartesian3 ab = verts[1].Point() - verts[0].Point();
	Cartesian3 bc = verts[2].Point() - verts[1].Point();
	Cartesian3 ca = verts[0].Point() - verts[2].Point();

	Cartesian3 ap = o - verts[0].Point();
	Cartesian3 bp = o - verts[1].Point();
	Cartesian3 cp = o - verts[2].Point();

	float areaabc = ab.cross(ca).length();



	float alpha = bc.cross(bp).length() / areaabc;
	float beta = cp.cross(ca).length() / areaabc;
	float gamma = 1 - alpha - beta;


	return Cartesian3(alpha, beta, gamma);

}

// I_total = I_specular + I_diffuse + I_ambient + I_emitted

Homogeneous4 Triangle::Blinn_PhongShading(Cartesian3 intersection, RenderParameters* renderParameters, Matrix4 matrix)
{
    Cartesian3 color;
    std::vector<Light*> lights = renderParameters->lights;
    Cartesian3 baricentri = baricentric(intersection);
    // normal
    Cartesian3 intersection_normal = (normals[0].Vector() * baricentri.x
        + normals[1].Vector() * baricentri.y
        + normals[2].Vector() * baricentri.z).unit();

    // I_emitted
    color = color + shared_material->emissive;

    // I_ambient
    color = color + shared_material->ambient;


    for (Light* light : lights)
    {
        Cartesian3 tolight = ((matrix * light->GetPositionCenter()).Point() - intersection).unit();
        Cartesian3 toeye = (-1 * intersection).unit();


        float diffuse_cos = std::max(0.0f, tolight.dot(intersection_normal));
        //float diffuse_cos = tolight.dot(intersection_normal);

        // I_diffuse
        color = color + diffuse_cos * light->GetColor().modulate(Homogeneous4(shared_material->diffuse)).Vector();

        // I_specular
        Cartesian3 bisector = ((tolight + toeye) * 0.5).unit();
        float specular_cos = intersection_normal.dot(bisector);
        // Fresnel approximation
        Cartesian3 f = shared_material->specular + (Cartesian3(1, 1, 1) - shared_material->specular) * std::pow(1 - bisector.dot(tolight), 5);
        color = color + (shared_material->shininess + 2) * diffuse_cos * std::pow(specular_cos, shared_material->shininess) * (light->GetColor().modulate(f)).Vector() / (2 * PI);

    }

    // avoid overflow
    color.x = std::min(1.0f, color.x);
    color.y = std::min(1.0f, color.y);
    color.z = std::min(1.0f, color.z);
    return Homogeneous4(color.x, color.y, color.z, 1);
}

Homogeneous4 Triangle::Blinn_PhongShading1(Cartesian3 intersection, Light* light, Matrix4 matrix)
{
    Cartesian3 color;
    //std::vector<Light*> lights = renderParameters->lights;
    Cartesian3 baricentri = baricentric(intersection);
    // normal
    Cartesian3 intersection_normal = (normals[0].Vector() * baricentri.x
        + normals[1].Vector() * baricentri.y
        + normals[2].Vector() * baricentri.z).unit();

    //// I_emitted
    //color = color + shared_material->emissive;

    //// I_ambient
    //color = color + shared_material->ambient;

    Cartesian3 tolight = ((matrix * light->GetPositionCenter()).Point() - intersection).unit();
    Cartesian3 toeye = (-1 * intersection).unit();


    float diffuse_cos = std::max(0.0f, tolight.dot(intersection_normal));
    //float diffuse_cos = tolight.dot(intersection_normal);

    // I_diffuse
    color = color + diffuse_cos * light->GetColor().modulate(Homogeneous4(shared_material->diffuse)).Vector();

    // I_specular
    Cartesian3 bisector = ((tolight + toeye) * 0.5).unit();
    float specular_cos = intersection_normal.dot(bisector);
    // Fresnel approximation
    Cartesian3 f = shared_material->specular + (Cartesian3(1, 1, 1) - shared_material->specular) * std::pow(1 - bisector.dot(tolight), 5);
    color = color + (shared_material->shininess + 2) * diffuse_cos * std::pow(specular_cos, shared_material->shininess) * (light->GetColor().modulate(f)).Vector() / (2 * PI);

    return Homogeneous4(color.x, color.y, color.z, 1);

}


