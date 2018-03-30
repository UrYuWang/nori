#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>



NORI_NAMESPACE_BEGIN

class PathMatsIntegrator : public Integrator {
public:
	PathMatsIntegrator(const PropertyList &props) {
		/* No parameters this time */
	}

	Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
		// Initial radiance and throughput
		Color3f Li = 0, t = 1;
		Ray3f rayR = ray;
		float prob = 1;

		while (true) {
			Intersection its;

			if (!scene->rayIntersect(rayR, its))
				return Li;

			// emitted
			if (its.mesh->isEmitter()) {
				EmitterQueryRecord lRecE(rayR.o, its.p, its.shFrame.n);
				Li += t*its.mesh->getEmitter()->eval(lRecE);
			}

			// Russian roulette
			prob = std::min(t[0], .99f);
			if (sampler->next1D() >= prob)
				return Li;

			t /= prob;

			// BSDF
			BSDFQueryRecord bRec(its.shFrame.toLocal(-rayR.d));
			Color3f f = its.mesh->getBSDF()->sample(bRec, sampler->next2D());

			t *= f;

			rayR = Ray3f(its.p, its.toWorld(bRec.wo));
		}
		return Li;
	}   

	std::string toString() const {
		return "PathMatsIntegrator[]";
	}
};

NORI_REGISTER_CLASS(PathMatsIntegrator, "path_mats");
NORI_NAMESPACE_END