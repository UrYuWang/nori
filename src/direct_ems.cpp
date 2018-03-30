#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>



NORI_NAMESPACE_BEGIN

class DirectEmsIntegrator : public Integrator {
public:
	DirectEmsIntegrator(const PropertyList &props) {
		/* No parameters this time */
	}

	Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
		
		//Find the surface that is visible in the requested direction 
		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return Color3f(0.0f);

		// get a random emitter
		const Emitter * emitter = scene->getRandomEmitter(sampler->next1D());
		
		// reflected
		EmitterQueryRecord lRecR;
		lRecR.ref = its.p;
		Color3f Li = emitter->sample(lRecR, sampler->next2D())*scene->getLights().size();

		// cosine term 
		float cosTheta = Frame::cosTheta(its.shFrame.toLocal(lRecR.wi));
		
		// BSDF
		BSDFQueryRecord bRec(its.shFrame.toLocal(-ray.d), its.shFrame.toLocal(lRecR.wi), ESolidAngle);
		bRec.uv = its.uv;
		Color3f f = its.mesh->getBSDF()->eval(bRec);

		// emitted
		Color3f Le = 0;
		if (its.mesh->isEmitter()) {
			EmitterQueryRecord lRecE(ray.o, its.p, its.shFrame.n);
			Le = its.mesh->getEmitter()->eval(lRecE);
		}

		// check if shadow ray is occluded
		if (scene->rayIntersect(lRecR.shadowRay, its))
			Li = 0;

		// return local illumination
		return Le + Li * f * std::max(0.f, cosTheta);
	}

	std::string toString() const {
		return "DirectEmsIntegrator[]";
	}
};

NORI_REGISTER_CLASS(DirectEmsIntegrator, "direct_ems");
NORI_NAMESPACE_END