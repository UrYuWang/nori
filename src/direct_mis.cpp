#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>



NORI_NAMESPACE_BEGIN

class DirectMisIntegrator : public Integrator {
public:
	DirectMisIntegrator(const PropertyList &props) {
		/* No parameters this time */
	}

	Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
		//Find the surface that is visible in the requested direction 
		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return Color3f(0.0f);

		// -----------------------------------emitted------------------------------------- //
		Color3f Le = 0;
		if (its.mesh->isEmitter()) {
			EmitterQueryRecord lRecE(ray.o, its.p, its.shFrame.n);
			Le = its.mesh->getEmitter()->eval(lRecE);
		}

		// -------------------------------------EMS--------------------------------------- //
		Color3f L_ems = 0;

		// get a random emitter
		const Emitter * emitter = scene->getRandomEmitter(sampler->next1D());

		// reflected
		EmitterQueryRecord lRec_ems;
		lRec_ems.ref = its.p;
		Color3f Li_ems = emitter->sample(lRec_ems, sampler->next2D())*scene->getLights().size();
		float w_ems = emitter->pdf(lRec_ems);

		// BSDF
		BSDFQueryRecord bRec_ems(its.shFrame.toLocal(-ray.d), its.shFrame.toLocal(lRec_ems.wi), ESolidAngle);
		bRec_ems.uv = its.uv;
		Color3f f_ems = its.mesh->getBSDF()->eval(bRec_ems);
		if (w_ems + its.mesh->getBSDF()->pdf(bRec_ems) != 0)
			w_ems /= (w_ems + its.mesh->getBSDF()->pdf(bRec_ems));

		// check if shadow ray is occluded
		Intersection its_ems;
		if (!scene->rayIntersect(lRec_ems.shadowRay, its_ems))
			L_ems = f_ems * Li_ems * std::max(0.f, Frame::cosTheta(its.shFrame.toLocal(lRec_ems.wi)));

		// -------------------------------------MATS--------------------------------------- //
		Color3f L_mats = 0;

		// BSDF
		BSDFQueryRecord bRec_mats(its.shFrame.toLocal(-ray.d));
		bRec_mats.uv = its.uv;
		Color3f f_mats = its.mesh->getBSDF()->sample(bRec_mats, sampler->next2D());
		float w_mats = its.mesh->getBSDF()->pdf(bRec_mats);

		// relflected
		Ray3f rayR = Ray3f(its.p, its.toWorld(bRec_mats.wo));
		Intersection itsR;
		if (scene->rayIntersect(rayR, itsR)) {
			if (itsR.mesh->isEmitter()) {
				EmitterQueryRecord lRec_mats = EmitterQueryRecord(its.p, itsR.p, itsR.shFrame.n);
				Color3f Li_mats = itsR.mesh->getEmitter()->eval(lRec_mats);
				if (w_mats + itsR.mesh->getEmitter()->pdf(lRec_mats) != 0)
					w_mats /= (w_mats + itsR.mesh->getEmitter()->pdf(lRec_mats));
				//local mats illumination (case where reflected ray interesects an emitter)
				L_mats = Li_mats * f_mats /** std::max(0.f, Frame::cosTheta(its.shFrame.toLocal(lRec_mats.wi)))*/;
			}
		}

		// -------------------------------------MIS--------------------------------------- //
		return Le + w_ems * L_ems + w_mats * L_mats;
	}

	std::string toString() const {
		return "DirectMisIntegrator[]";
	}
};

NORI_REGISTER_CLASS(DirectMisIntegrator, "direct_mis");
NORI_NAMESPACE_END