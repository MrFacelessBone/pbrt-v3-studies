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

    if (!scene.Intersect(ray, & isect))
    {
        return L;
    }

    Spectrum L2(1.0);
    return L2;
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