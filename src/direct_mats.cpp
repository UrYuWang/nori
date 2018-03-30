#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>



NORI_NAMESPACE_BEGIN

class DirectMatsIntegrator : public Integrator {
public:
	DirectMatsIntegrator(const PropertyList &props) {
		/* No parameters this time */
	}

	Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
		//Find the surface that is visible in the requested direction 
		Intersection its;
		if (!scene->rayIntersect(ray, its)){
			if (scene->getEnvLight() == nullptr) {
				return Color3f(0,0,0);
			} else {
				EmitterQueryRecord lRec;
				lRec.wi = ray.d;
				return scene->getEnvLight()->eval(lRec);
			}
		}

		// emitted
		Color3f Le(0,0,0);
		if (its.mesh->isEmitter()) {
			EmitterQueryRecord lRecE(ray.o, its.p, its.shFrame.n);
			Le = its.mesh->getEmitter()->eval(lRecE);
		}

		// BSDF
		BSDFQueryRecord bRec(its.shFrame.toLocal(-ray.d));
		bRec.uv = its.uv;
		Color3f f = its.mesh->getBSDF()->sample(bRec, sampler->next2D());

		// relflected
		Color3f Li(0,0,0);

		Ray3f rayR = Ray3f(its.p, its.toWorld(bRec.wo));
		Intersection itsR;
		if (scene->rayIntersect(rayR, itsR)) {
			if (itsR.mesh->isEmitter()) {
				EmitterQueryRecord lRecR = EmitterQueryRecord(its.p, itsR.p, itsR.shFrame.n);
				Li = itsR.mesh->getEmitter()->eval(lRecR);
			}
		} else if (scene->getEnvLight() != nullptr) {
			EmitterQueryRecord lRecR;
			lRecR.wi = rayR.d;
			Li =  scene->getEnvLight()->eval(lRecR);
		}

		// return local illumination (case where reflected ray doesn't intersect an emitter)
		return Le + Li * f;
	}

	std::string toString() const {
		return "DirectMatsIntegrator[]";
	}
};

NORI_REGISTER_CLASS(DirectMatsIntegrator, "direct_mats");
NORI_NAMESPACE_END
