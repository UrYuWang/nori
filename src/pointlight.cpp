#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

class PointLight : public Emitter {
public:
	PointLight(const PropertyList &propList) {
		m_position = propList.getPoint3("position", Point3f());
		m_power = propList.getColor("power", Color3f());
	}

	virtual Color3f sample(EmitterQueryRecord &lRec, const Point2f &sample) const {
		lRec.p = m_position;
		lRec.wi = (m_position - lRec.ref).normalized();
		// shadowray term to check that sampled point and light source are mutually visible
		lRec.shadowRay = Ray3f(lRec.ref, lRec.wi, Epsilon, (lRec.p-lRec.ref).norm());

		return m_power / (4 * M_PI * (lRec.ref - lRec.p).squaredNorm());
	}

	virtual Color3f eval(const EmitterQueryRecord &lRec) const {
		return m_power;
	}

	virtual float pdf(const EmitterQueryRecord &lRec) const {
		return 1;
	}

	virtual std::string toString() const {
		return "PointLight[]";
	}

private:
	Color3f m_power;
	Point3f m_position;
};

NORI_REGISTER_CLASS(PointLight, "point");
NORI_NAMESPACE_END