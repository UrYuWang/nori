#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>


NORI_NAMESPACE_BEGIN

class DirectIntegrator : public Integrator {
public:
	DirectIntegrator(const PropertyList &props) {
		/* No parameters this time */
	}

	Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
		/* Find the surface that is visible in the requested direction */
		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return Color3f(0.0f);

		// get all light emitters in scene 
		std::vector<Emitter *> emitters = scene->getLights();
		
		// normal of shading point
		Normal3f n = its.shFrame.n;
		
		// structure for query of light sample function
		EmitterQueryRecord lRec;
		
		// point of shading point
		lRec.ref = its.p;
		
		// sample point not used but necessary for calling the emitters sample function
		Point2f sample;
		
		//initialize color
		Color3f color = 0;

		// add color for every light emitter
		for (int i; i < emitters.size(); ++i) {
			// term of emitter
			Color3f eColor = emitters[i]->sample(lRec, sample);
			
			// angle between ray of light and normal of shading point
			float cosTheta = Frame::cosTheta(its.shFrame.toLocal(lRec.wi));
			
			// query for the bsdf evaluation
			BSDFQueryRecord bRec(its.shFrame.toLocal(lRec.wi), its.shFrame.toLocal(- ray.d), ESolidAngle);

			bRec.uv = its.uv;
			
			// all the terms together
			color += its.mesh->getBSDF()->eval(bRec) * !scene->rayIntersect(lRec.shadowRay, its) * (cosTheta + sqrt(pow(cosTheta, 2))) / 2 * eColor;
		}

		return color;
	}

	std::string toString() const {
		return "DirectIntegrator[]";
	}
};

NORI_REGISTER_CLASS(DirectIntegrator, "direct");
NORI_NAMESPACE_END