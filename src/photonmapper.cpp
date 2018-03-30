/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/integrator.h>
#include <nori/sampler.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/scene.h>
#include <nori/photon.h>

NORI_NAMESPACE_BEGIN

class PhotonMapper : public Integrator {
public:
    /// Photon map data structure
    typedef PointKDTree<Photon> PhotonMap;

    PhotonMapper(const PropertyList &props) {
        /* Lookup parameters */
		m_photonCount = props.getInteger("photonCount", 1000000);
        m_photonRadius = props.getFloat("photonRadius", 0.0f /* Default: automatic */);
    }

    virtual void preprocess(const Scene *scene) override {
        cout << "Gathering " << m_photonCount << " photons .. ";
        cout.flush();

        /* Create a sample generator for the preprocess step */
        Sampler *sampler = static_cast<Sampler *>(
            NoriObjectFactory::createInstance("independent", PropertyList()));

        /* Allocate memory for the photon map */
        m_photonMap = std::unique_ptr<PhotonMap>(new PhotonMap());
        m_photonMap->reserve(m_photonCount);

		/* Estimate a default photon radius */
		if (m_photonRadius == 0)
			m_photonRadius = scene->getBoundingBox().getExtents().norm() / 500.0f;

	

		/* How to add a photon?
		 * m_photonMap->push_back(Photon(
		 *	Point3f(0, 0, 0),  // Position
		 *	Vector3f(0, 0, 1), // Direction
		 *	Color3f(1, 2, 3)   // Power
		 * ));
		 */

		// put your code to trace photons here

		for (int p = 0; p < m_photonCount; ++p) {
			const Emitter* emitter = scene->getRandomEmitter(sampler->next1D());
			
			Ray3f ray;
			Color3f power = emitter->samplePhoton(ray, sampler->next2D(), sampler->next2D()) * scene->getLights().size();
			
			tracePhoton(scene, sampler, ray, power);
		}

        m_photonMap->build();
    }

	void tracePhoton(const Scene *scene, Sampler *sampler, Ray3f &ray, Color3f &power) {
		float prob = 1;
		
		while (true) {
			Intersection its;

			if (!scene->rayIntersect(ray, its))
				break;

			Photon photon(its.p, -ray.d, power);

			if (its.mesh->getBSDF()->isDiffuse())
				m_photonMap->push_back(photon);

			// Russian roulette
			prob = std::min(power[0], .99f);
			if (sampler->next1D() >= prob)
				break;

			power /= prob;

			BSDFQueryRecord bRec(its.toLocal(-ray.d));
			Color3f f = its.mesh->getBSDF()->sample(bRec, sampler->next2D());

			power *= f;

			ray = Ray3f(its.p, its.toWorld(bRec.wo));
		}
	}

    virtual Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &_ray) const override {
    	
		/* How to find photons?
		 * std::vector<uint32_t> results;
		 * m_photonMap->search(Point3f(0, 0, 0), // lookup position
		 *                     m_photonRadius,   // search radius
		 *                     results);
		 *
		 * for (uint32_t i : results) {
		 *    const Photon &photon = (*m_photonMap)[i];
		 *    cout << "Found photon!" << endl;
		 *    cout << " Position  : " << photon.getPosition().toString() << endl;
		 *    cout << " Power     : " << photon.getPower().toString() << endl;
		 *    cout << " Direction : " << photon.getDirection().toString() << endl;
		 * }
		 */

		// put your code for path tracing with photon gathering here
		// Initial radiance and throughput
		Color3f Li = 0, t = 1;
		Ray3f rayR = _ray;
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

			// photons
			if (its.mesh->getBSDF()->isDiffuse())
				return Li + t * photonDensityEstimation(its, rayR);

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

	Color3f photonDensityEstimation(const Intersection &its, const Ray3f &ray) const {
		Color3f Lr = 0;

		std::vector<Photon::IndexType> scope_photons;
		m_photonMap->search(its.p, m_photonRadius, scope_photons);

		for (int i = 0; i < scope_photons.size(); ++i) {
			Photon photon = (*m_photonMap)[scope_photons[i]];
			BSDFQueryRecord bRec(its.shFrame.toLocal(-ray.d), its.shFrame.toLocal(photon.getDirection()), ESolidAngle);			
			Lr += its.mesh->getBSDF()->eval(bRec) * photon.getPower();
		}
		return Lr * INV_PI / (pow(m_photonRadius, 2) * m_photonCount);
	}

    virtual std::string toString() const override {
        return tfm::format(
            "PhotonMapper[\n"
            "  photonCount = %i,\n"
            "  photonRadius = %f\n"
            "]",
            m_photonCount,
            m_photonRadius
        );
    }
private:
    int m_photonCount;
    float m_photonRadius;
    std::unique_ptr<PhotonMap> m_photonMap;
};

NORI_REGISTER_CLASS(PhotonMapper, "photonmapper");
NORI_NAMESPACE_END
