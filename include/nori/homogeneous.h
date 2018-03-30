#if !defined(__NORI_HOMOGENEOUSMEDIUM_H)
#define __NORI_HOMOGENEOUSMEDIUM_H

#include <nori/object.h>

NORI_NAMESPACE_BEGIN

/*
struct MediumQueryRecord {
    Point3f ref;
    Point3f p;
    Normal3f n;
    Vector3f wi;
    float pdf;
    Ray3f shadowRay;
};
*/

class HomogeneousMedium : public NoriObject {
public:
	HomogeneousMedium(const PropertyList& props);

	Color3f tr(const Point3f &a, const Point3f &b) const;
	//Color3f tr(const Ray3f &ray) const;
	//Color3f sample(const Ray3f &ray, const Point2f &sample) const;
	float phaseIsotropic() { return 0.25 * INV_PI; }
	Color3f getSa() const { return m_sa; }
	Color3f getSs() const { return m_ss; }
	Color3f getSt() const { return m_st; }
	Color3f getAlbedo() const { return m_albedo;}
	float getPhaseFunction() const { return 0.25 * INV_PI; }


	EClassType getClassType() const { return EMedium; }
	//virtual std::string toString() const override;
	virtual std::string toString() const override {
		return tfm::format(
				"HomogeneousMedium[\n"
				"  sigma absorption = \n%s,\n"
				"  sigma scattering = \n%s,\n"
				"  sigma extinction = \n%s,\n"
				"  albedo = \n%s \n"
				"]",
				m_sa,
				m_ss,
				m_st,
				m_albedo
				);
	}

private:
	Color3f m_sa, m_ss, m_st, m_albedo;
};

NORI_NAMESPACE_END

#endif /* __NORI_HOMOGENEOUSMEDIUM_H */
