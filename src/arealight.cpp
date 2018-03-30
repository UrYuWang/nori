/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Romain Prévost

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

#include <nori/emitter.h>
#include <nori/warp.h>
#include <nori/shape.h>

NORI_NAMESPACE_BEGIN

class AreaEmitter : public Emitter {
public:
    AreaEmitter(const PropertyList &props) {
        m_radiance = props.getColor("radiance");
    }

    virtual std::string toString() const override {
        return tfm::format(
                "AreaLight[\n"
                "  radiance = %s,\n"
                "]",
                m_radiance.toString());
    }

    virtual Color3f eval(const EmitterQueryRecord & lRec) const override {
        if(!m_shape)
            throw NoriException("There is no shape attached to this Area light!");

		if ((-lRec.wi).dot(lRec.n) >= 0)
			return m_radiance;
		else
			return 0;
    }

    virtual Color3f sample(EmitterQueryRecord & lRec, const Point2f & sample) const override {
        if(!m_shape)
            throw NoriException("There is no shape attached to this Area light!");
		ShapeQueryRecord sRec;
		m_shape->sampleSurface(sRec, sample);
		
		// set the emitter query record
		lRec.p = sRec.p;
		lRec.n = sRec.n;
		lRec.wi = (lRec.p - lRec.ref).normalized();
		lRec.shadowRay = Ray3f(lRec.ref, lRec.wi, Epsilon, (lRec.p - lRec.ref).norm() - Epsilon);

		float pdf_val = pdf(lRec);
		if (pdf_val == 0)
			return 0;

		return eval(lRec) / pdf_val;
		//return m_radiance / pdf(lRec);
    }

    virtual float pdf(const EmitterQueryRecord &lRec) const override {
        if(!m_shape)
            throw NoriException("There is no shape attached to this Area light!");

		ShapeQueryRecord sRec(lRec.ref, lRec.p);
		sRec.n = lRec.n;
		float cosTheta = (-lRec.wi).dot(lRec.n);

		if (cosTheta <= 0)
			return 0;

		return  m_shape->pdfSurface(sRec) * (lRec.p - lRec.ref).squaredNorm() / cosTheta;
    }

    virtual Color3f samplePhoton(Ray3f &ray, const Point2f &sample1, const Point2f &sample2) const override {
		
		ShapeQueryRecord sRec(Point3f(0.0f));
		m_shape->sampleSurface(sRec, sample1);

		Vector3f d = Warp::squareToCosineHemisphere(sample2);
		d = Frame(sRec.n).toWorld(d);

		//photon ray
		ray = Ray3f(sRec.p, d);

		EmitterQueryRecord lRec(sRec.p + d, sRec.p, sRec.n);

		if (sRec.pdf == 0)
			throw NoriException("You are trying to divide by zero");

		return eval(lRec) * M_PI / sRec.pdf;
    }


protected:
    Color3f m_radiance;
};

NORI_REGISTER_CLASS(AreaEmitter, "area")
NORI_NAMESPACE_END
