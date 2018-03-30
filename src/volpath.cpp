#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>
#include <nori/warp.h>


NORI_NAMESPACE_BEGIN

class VolPathIntegrator : public Integrator {
public:
	VolPathIntegrator(const PropertyList &props) {
		/* No parameters this time */
	}

	Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {

		// Initial radiance and throughput
		Color3f Li = 0, t = 1;
		Ray3f rayR = ray;
		float prob = 1;
		// medium
		const HomogeneousMedium* medium = scene->getMedium();

		while (true) {
			Intersection its;

			float tmax;
			if(scene->rayIntersect(rayR, its))
				tmax = (its.p - rayR.o).norm();
			else
				tmax =its.t;

			// sample free path
			float td = -log(1 - sampler->next1D()) / medium->getSt().maxCoeff();

			// volume interaction
			if (td < tmax) {
				rayR = Ray3f(rayR.o + td * rayR.d.normalized(), Warp::squareToUniformSphere(sampler->next2D()));

				// reflected
				const Emitter* emitter = scene->getRandomEmitter(sampler->next1D());
				EmitterQueryRecord lRecE;
				lRecE.ref = rayR.o;
				Color3f Le = emitter->sample(lRecE, sampler->next2D())*scene->getLights().size();

				// check if shadow ray is occluded
				Intersection itsE;
				if (scene->rayIntersect(lRecE.shadowRay, itsE))
					Le = Color3f(0, 0, 0);

				t *= medium->getAlbedo();
				Li += t * medium->tr(rayR.o, lRecE.p) * medium->getPhaseFunction() * Le;
			} else {
				// surface interaction
				if (its.mesh->isEmitter()) {
					EmitterQueryRecord lRecE(rayR.o, its.p, its.shFrame.n);
					Color3f Le = its.mesh->getEmitter()->eval(lRecE);
					Li += t * medium->tr(rayR.o, its.p) * Le;
				} else {
					// reflected
					const Emitter* emitter = scene->getRandomEmitter(sampler->next1D());
					EmitterQueryRecord lRecE;
					lRecE.ref = its.p;
					Color3f Le = emitter->sample(lRecE, sampler->next2D())*scene->getLights().size();

					// check if shadow ray is occluded
					Intersection itsE;
					if (scene->rayIntersect(lRecE.shadowRay, itsE))
						Le = Color3f(0, 0, 0);

					// BSDF
					BSDFQueryRecord bRecE(its.shFrame.toLocal(-rayR.d), its.shFrame.toLocal(lRecE.wi), ESolidAngle);
					bRecE.uv = its.uv;
					Color3f fE = its.mesh->getBSDF()->eval(bRecE);

					// result
					Li += t * fE * medium->tr(rayR.o, lRecE.p) * Le;
				}


				// BSDF sampling
				BSDFQueryRecord bRec(its.shFrame.toLocal(-rayR.d));
				Color3f f = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
				t *= f;
				// shoot next ray
				rayR = Ray3f(its.p, its.toWorld(bRec.wo));
			}
			// russian roulette
			prob = std::min(t.maxCoeff(), .99f);
			if (sampler->next1D() >= prob)
				return Li;

			t /= prob;
		}
	}

	std::string toString() const {
		return "VolPathIntegrator[]";
	}
};

NORI_REGISTER_CLASS(VolPathIntegrator, "volpath");
NORI_NAMESPACE_END
