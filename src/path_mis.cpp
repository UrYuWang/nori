
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>



NORI_NAMESPACE_BEGIN

class PathMisIntegrator : public Integrator {
public:
	PathMisIntegrator(const PropertyList &props) {
		/* No parameters this time */
	}

	Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
		// Initial radiance and throughput
		Color3f Li = 0, t = 1;
		Ray3f rayR = ray;
		float prob = 1, w_mats = 1, w_ems = 1;
		Color3f f(1,1,1);

		while (true) {
			Intersection its;

			if (!scene->rayIntersect(rayR, its)) {
				if (scene->getEnvLight() == nullptr) {
					return Li;
				} else {
					EmitterQueryRecord lRec;
					lRec.wi = rayR.d;
					return Li + w_mats * t * scene->getEnvLight()->eval(lRec);
				}
			}

			// -------------------------------------Emitted--------------------------------------- //
			Color3f Le = 0;
			if (its.mesh->isEmitter()) {
				EmitterQueryRecord lRecE(rayR.o, its.p, its.shFrame.n);
				Le = its.mesh->getEmitter()->eval(lRecE);
			}
			Li += t * w_mats * Le;

			// -------------------------------------Russian roulette--------------------------------------- //
			prob = std::min(t.maxCoeff(), .99f);
			if (sampler->next1D() >= prob)
				return Li;

			t /= prob;

			// -------------------------------------EMS--------------------------------------- //
			Color3f L_ems = 0;

			// get a random emitter
			const Emitter * emitter = scene->getRandomEmitter(sampler->next1D());

			// reflected
			EmitterQueryRecord lRec_ems;
			lRec_ems.ref = its.p;
			Color3f Li_ems = emitter->sample(lRec_ems, sampler->next2D())*scene->getLights().size();
			float pdf_ems = emitter->pdf(lRec_ems);

			// BSDF
			BSDFQueryRecord bRec_ems(its.shFrame.toLocal(-rayR.d), its.shFrame.toLocal(lRec_ems.wi), ESolidAngle);
			bRec_ems.uv = its.uv;
			Color3f f_ems = its.mesh->getBSDF()->eval(bRec_ems);
			if (pdf_ems + its.mesh->getBSDF()->pdf(bRec_ems) != 0)
				w_ems = pdf_ems / (pdf_ems + its.mesh->getBSDF()->pdf(bRec_ems));

			// check if shadow ray is occluded
			Intersection its_ems;
			if (!scene->rayIntersect(lRec_ems.shadowRay, its_ems))
				L_ems = f_ems * Li_ems * std::max(0.f, Frame::cosTheta(its.shFrame.toLocal(lRec_ems.wi)));

			Li += t * w_ems * L_ems;

			// -------------------------------------BSDF sampling--------------------------------------- //
			BSDFQueryRecord bRec(its.shFrame.toLocal(-rayR.d));
			Color3f f = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
			t *= f;
			// shoot next ray
			rayR = Ray3f(its.p, its.toWorld(bRec.wo));

			// ----------------------------------MATS weight for next Le----------------------------------- //
			float pdf_mats = its.mesh->getBSDF()->pdf(bRec);

			Intersection itsR;
			if (scene->rayIntersect(rayR, itsR)) {
				if (itsR.mesh->isEmitter()) {
					EmitterQueryRecord lRec_mats = EmitterQueryRecord(its.p, itsR.p, itsR.shFrame.n);
					if (pdf_mats + itsR.mesh->getEmitter()->pdf(lRec_mats) != 0)
						w_mats = pdf_mats / (pdf_mats + itsR.mesh->getEmitter()->pdf(lRec_mats));
				}
			} else if (scene->getEnvLight() != nullptr) {
				EmitterQueryRecord lRec_mats;
				lRec_mats.wi = rayR.d;
				if (pdf_mats + scene->getEnvLight()->pdf(lRec_mats) != 0)
					w_mats = pdf_mats / (pdf_mats + scene->getEnvLight()->pdf(lRec_mats));
			}

			if (bRec.measure == EDiscrete) {
				w_mats = 1;
				w_ems = 0;
			}
		}
	} 

	std::string toString() const {
		return "PathMisIntegrator[]";
	}
};

NORI_REGISTER_CLASS(PathMisIntegrator, "path_mis");
NORI_NAMESPACE_END
