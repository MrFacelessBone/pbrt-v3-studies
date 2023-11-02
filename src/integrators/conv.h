#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_INTEGRATORS_CONV_H
#define PBRT_INTEGRATORS_CONV_H

// integrators/conv.h*
#include "pbrt.h"
#include "integrator.h"
#include "scene.h"

namespace pbrt {
class ConventionalIntegrator :
    public SamplerIntegrator
{
public:
    ConventionalIntegrator(int maxDepth, std::shared_ptr<const Camera> camera,
        std::shared_ptr<Sampler> sampler,
        const Bounds2i& pixelBounds)
        : SamplerIntegrator(camera, sampler, pixelBounds), maxDepth(maxDepth) {}
    Spectrum Li(const RayDifferential& ray, const Scene& scene,
        Sampler& sampler, MemoryArena& arena, int depth) const;

private:
    // WhittedIntegrator Private Data
    const int maxDepth; // brauchen wir das überhaupt?
};

ConventionalIntegrator* CreateConventionalIntegrator(
    const ParamSet& params, std::shared_ptr<Sampler> sampler,
    std::shared_ptr<const Camera> camera);

}  // namespace pbrt

#endif  // PBRT_INTEGRATORS_CONV_H
