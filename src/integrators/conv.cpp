// integrators/conv.cpp
#include "integrators/conv.h"
#include "scene.h"
#include "camera.h"
#include "paramset.h"

namespace pbrt {

Spectrum ConventionalIntegrator::Li(const RayDifferential& ray, const Scene& scene,
    Sampler& sampler, MemoryArena& arena, int depth) const
{
    Spectrum L(0.);
    SurfaceInteraction isect;

    if (!scene.Intersect(ray, &isect))
    {
        return L;
    }

    Normal3f n = isect.shading.n;
    Vector3f wo = isect.wo;

    SurfaceInteraction isecttmp;

    for (const auto& light : scene.lights) {
        // Lichtposition
        Point3f lightPos = light->LightToWorld(Point3f(0, 0, 0));  // falls LightToWorld nicht klappt, von
                                // protected auf public setzen
        Ray visibilityRay = isect.SpawnRayTo(lightPos);
        Ray vRay = Ray(isect.p, Vector3f(lightPos - isect.p));
        
        bool isVisible = !scene.IntersectP(vRay);

        if (isVisible) {
            Spectrum li = light->Power() / (4 * Pi * DistanceSquared(lightPos, isect.p));  // Abstandsgesetz
            Vector3f l = Normalize(Vector3f(lightPos - isect.p));  // Vector von hit zu Lichtquelle
            L += li * AbsDot(n, l);             // Skalarprodukt
        }
    }
    return L;
}


ConventionalIntegrator *CreateConventionalIntegrator(const ParamSet& params, 
                                                     std::shared_ptr<Sampler> sampler,
                                                     std::shared_ptr<const Camera> camera) 
{
    int maxDepth = params.FindOneInt("maxdepth", 5);
    int np;
    const int* pb = params.FindInt("pixelbounds", &np);
    Bounds2i pixelBounds = camera->film->GetSampleBounds();
    if (pb) {
        if (np != 4)
            Error("Expected four values for \"pixelbounds\" parameter. Got %d.",
                np);
        else {
            pixelBounds = Intersect(pixelBounds,
                Bounds2i{ {pb[0], pb[2]}, {pb[1], pb[3]} });
            if (pixelBounds.Area() == 0)
                Error("Degenerate \"pixelbounds\" specified.");
        }
    }
    return new ConventionalIntegrator(maxDepth, camera, sampler, pixelBounds);
}

}  // namespace pbrt