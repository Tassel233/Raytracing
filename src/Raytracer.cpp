//////////////////////////////////////////////////////////////////////
//
//  University of Leeds
//  COMP 5892M Advanced Rendering
//  User Interface for Coursework
////////////////////////////////////////////////////////////////////////


#include <math.h>
#include <thread>
#include <random>
#include <omp.h>
#include <algorithm>
// include the header file
#include "Raytracer.h"

#define N_THREADS 16
#define N_LOOPS 600
#define N_BOUNCES 10
#define TERMINATION_FACTOR 0.35f

// constructor
Raytracer::Raytracer(std::vector<ThreeDModel> *newTexturedObject, RenderParameters *newRenderParameters):
    texturedObjects(newTexturedObject),
    renderParameters(newRenderParameters),
    raytraceScene(texturedObjects,renderParameters)
    { 
        std::srand(static_cast<unsigned int>(std::time(nullptr)));
        restartRaytrace = false;
        raytracingRunning = false;
    }


Raytracer::~Raytracer()
    { 
    // all of our pointers are to data owned by another class
    // so we have no responsibility for destruction
    }                                                                  


// called every time the widget is resized
void Raytracer::resize(int w, int h)
{ // RaytraceRenderWidget::resizeGL()
    // resize the render image
	frameBuffer.Resize(w, h);
} // RaytraceRenderWidget::resizeGL()
    
void Raytracer::stopRaytracer() {
    restartRaytrace = true;
    while (raytracingRunning) {
        std::chrono::milliseconds timespan(10);
        std::this_thread::sleep_for(timespan);
    }
    restartRaytrace = false;
}

inline
float linear_from_srgb(std::uint8_t aValue) noexcept
{
    float const fvalue = float(aValue) / 255.f;

    if (fvalue < 0.04045f)
        return (1.f / 12.92f) * fvalue;

    return std::pow((1.f / 1.055f) * (fvalue + 0.055f), 2.4f);

}

inline
std::uint8_t linear_to_srgb(float aValue) noexcept
{
    if (aValue < 0.0031308f)
        return std::uint8_t(255.f * 12.92f * aValue + 0.5f);
    return std::uint8_t(255.f * (1.055f * std::pow(aValue, 1.f / 2.4f) - 0.055f) + 0.5f);
}

void Raytracer::RaytraceThread()
{
    //raytraceScene.test_triangle();
    for (int j = 0; j < frameBuffer.height; j++) {
#pragma omp parallel for schedule(dynamic) 
        for (int i = 0; i < frameBuffer.width; i++) {

            // cast a ray
            Ray ray = calculateRay(i, j, !renderParameters->orthoProjection);

            // find the nearest collision triangle
            Scene::CollisionInfo collision_info = raytraceScene.closestTriangle(ray);

            // calculate intersection vertex 
            Cartesian3 intersection = ray.direction * collision_info.t + ray.origin;

            // get barycentric interplation
            Cartesian3 barycentric_point = collision_info.tri.baricentric(intersection);


            Homogeneous4 color;

            if (collision_info.t > 0)
            {
                Cartesian3 ambient_color;
                Cartesian3 emit_color = collision_info.tri.shared_material->emissive;
                for (Light* light : renderParameters->lights)
                {
                    ambient_color = ambient_color + light->GetColor().modulate(Homogeneous4(collision_info.tri.shared_material->ambient)).Vector();
                }


                if (renderParameters->interpolationRendering)
                {
                    Cartesian3 normal = (barycentric_point.x * collision_info.tri.normals[0].Vector()
                        + barycentric_point.y * collision_info.tri.normals[1].Vector()
                        + barycentric_point.z * collision_info.tri.normals[2].Vector()).unit();
                    color = Homogeneous4(abs(normal.x), abs(normal.y), abs(normal.z), 1);
                    frameBuffer[j][i] = RGBAValue(linear_to_srgb(color.x), linear_to_srgb(color.y), linear_to_srgb(color.z), 255);
                    continue;
                }

                if (renderParameters->phongEnabled)
                {
                
                    for (Light* light : renderParameters->lights)
                    {
                        color = color + collision_info.tri.Blinn_PhongShading1(intersection, light, raytraceScene.getModelview());
                    }
                }

                if (renderParameters->shadowsEnabled)
                {
                    Matrix4 matrix = raytraceScene.getModelview();

                    // avoid shadow acne
					float bias = 0.001f;
					Cartesian3 normal = (collision_info.tri.normals[0].Vector() * barycentric_point.x +
						collision_info.tri.normals[1].Vector() * barycentric_point.y +
						collision_info.tri.normals[2].Vector() * barycentric_point.z).unit();

					// iterate each light
					for (Light* light : renderParameters->lights)
					{
						Ray shadow_ray(intersection + bias * normal, ((matrix * light->GetPositionCenter()).Point() - intersection).unit(), Ray::secondary);
						// get the t from intersection to light position
						float t2light = ((matrix * light->GetPositionCenter()).Point() - intersection).length();
						Scene::CollisionInfo shadow_collision = raytraceScene.closestTriangle(shadow_ray);

						if (shadow_collision.t > 0)
						{
							if (!shadow_collision.tri.shared_material->isLight())
							{
								// generate shadow
								continue;
							}
							else
							{
								color = color + collision_info.tri.Blinn_PhongShading1(intersection, light, raytraceScene.getModelview());
							}
						}
					}

				}

                if (renderParameters->reflectionEnabled)
                {
                    Homogeneous4 r_color = TraceAndShadeWithRay(ray, N_BOUNCES, 1.0);
                    frameBuffer[j][i] = RGBAValue(linear_to_srgb(r_color.x), linear_to_srgb(r_color.y), linear_to_srgb(r_color.z), 255);
                    continue;
                }

				color = color + emit_color + ambient_color;
				color.x = std::min(1.0f, color.x);
				color.y = std::min(1.0f, color.y);
				color.z = std::min(1.0f, color.z);
				frameBuffer[j][i] = RGBAValue(linear_to_srgb(color.x), linear_to_srgb(color.y), linear_to_srgb(color.z), 255);
			}
		}
	}
	if (restartRaytrace) 
    {
		raytracingRunning = false;
		return;
	}
	raytracingRunning = false;
}

    // routine that generates the image
void Raytracer::Raytrace()
{ // RaytraceRenderWidget::Raytrace()
    stopRaytracer();
    //To make our lifes easier, lets calculate things on VCS.
    //So we need to process our scene to get a triangle soup in VCS.
    raytraceScene.updateScene();
    frameBuffer.clear(RGBAValue(0.0f, 0.0f, 0.0f,1.0f));
    std::thread raytracingThread(&Raytracer::RaytraceThread,this);
    raytracingThread.detach();
    raytracingRunning = true;
} // RaytraceRenderWidget::Raytrace()
    


Ray Raytracer::calculateRay(int pixelx, int pixely, bool perspective)
{
    //cast a ray in vcs

    Cartesian3 origin;
    Cartesian3 dir;
    float width = frameBuffer.width;
    float height = frameBuffer.height;
    float tanfov2 = tanf(renderParameters->fov / 2.0f);

    float aspect = width / height;

    float ndcs_x = ((pixelx / width) - 0.5) * 2;
    float ndcs_y = ((pixely / height) - 0.5) * 2;
    float x, y;
 

    if (perspective)
    {
        x = ndcs_x * aspect;
        y = ndcs_y;
        x = x * tanfov2;
        y = y * tanfov2;

        origin = Cartesian3();
        dir = Cartesian3(x, y, 1) - origin;
        return Ray(origin, dir.unit(), Ray::primary);
    }
    else 
    {
		if (aspect > 1.0f) {
            x = ndcs_x * aspect;
            y = ndcs_y;
		}
		else if (aspect < 1.0f) {
			y = ndcs_y / aspect;
			x = ndcs_x;
		}

        origin = Cartesian3(x, y, 0);
        dir = Cartesian3(0, 0, 1);
        return Ray(origin, dir, Ray::primary);
    }
}

Ray Raytracer::reflectRay(Ray r, Cartesian3 normal, Cartesian3 hitPoint) 
{
    //avoid acnes
    float epsilon = 1e-4;
    
    Cartesian3 dir = r.direction - 2 * r.direction.dot(normal) * normal;

    return Ray(hitPoint + epsilon * normal, dir.unit(), Ray::secondary);
}

Homogeneous4 Raytracer::TraceAndShadeWithRay(Ray r, int n_bounces, float mirror)
{
    Homogeneous4 color;

    Scene::CollisionInfo collision_info = raytraceScene.closestTriangle(r);

    // intersect valid triangle
    if (collision_info.t > 0) {

        /////////////////////////
        //blinn phong with shadow
        /////////////////////////

        // calculate intersection vertex
        Cartesian3 intersection = r.direction * collision_info.t + r.origin;

        // get barycentric interplation
        Cartesian3 barycentric_point = collision_info.tri.baricentric(intersection);

        Matrix4 matrix = raytraceScene.getModelview();

        Cartesian3 normal = (collision_info.tri.normals[0].Vector() * barycentric_point.x +
            collision_info.tri.normals[1].Vector() * barycentric_point.y +
            collision_info.tri.normals[2].Vector() * barycentric_point.z).unit();

        Cartesian3 ambient_color = collision_info.tri.shared_material->ambient;
        Cartesian3 emit_color = collision_info.tri.shared_material->emissive;

        color = ambient_color + emit_color;

        // avoid shadow acne
        float bias = 0.001f;

        // iterate each light
        for (Light* light : renderParameters->lights)
        {
            Ray shadow_ray(intersection + bias * normal, ((matrix * light->GetPositionCenter()).Point() - intersection).unit(), Ray::secondary);
            // get the t from intersection to light position
            float t2light = ((matrix * light->GetPositionCenter()).Point() - intersection).length();
            Scene::CollisionInfo shadow_collision = raytraceScene.closestTriangle(shadow_ray);

            if (shadow_collision.t > 0)
            {
                if (!shadow_collision.tri.shared_material->isLight())
                {
                    // generate shadow
                    continue;
                }
                else
                {
                    color = color + collision_info.tri.Blinn_PhongShading1(intersection, light, raytraceScene.getModelview());
                }
            }
        }

        color = color * (1 - collision_info.tri.shared_material->reflectivity);


        ///////////////////////
        //recursive reflection
        //////////////////////
        if (n_bounces > 0 && collision_info.tri.shared_material->reflectivity != 0)
        {
            if (collision_info.tri.shared_material->reflectivity == 0) return color;
            Cartesian3 ref_color;
            ref_color = TraceAndShadeWithRay(reflectRay(r, normal, intersection), n_bounces - 1, 1.0f).Vector();
            color =  color + ref_color * collision_info.tri.shared_material->reflectivity;
        }
        color.x = std::min(1.0f, color.x);
        color.y = std::min(1.0f, color.y);
        color.z = std::min(1.0f, color.z);
        return color;
    }
    return Homogeneous4(0, 0, 0, 1.0f);
}
