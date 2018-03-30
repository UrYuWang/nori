#include "nori/homogeneous.h"

NORI_NAMESPACE_BEGIN

HomogeneousMedium::HomogeneousMedium(const PropertyList &props) {
	m_sa = props.getColor("sigma_a", Color3f(0.2,0.2,0.2));
	m_ss = props.getColor("sigma_s", Color3f(0.2,0.2,0.2));
	m_st = m_sa + m_ss;
	if (m_st.isZero())
			std::cerr << "Unvalid sigma values for medium \n";
	m_albedo = m_sa / m_st;
}

Color3f HomogeneousMedium::tr(const Point3f &a, const Point3f &b) const {
	return Color3f(std::exp(- m_st.x() * (a - b).norm()),
			       std::exp(- m_st.y() * (a - b).norm()),
			       std::exp(- m_st.z() * (a - b).norm()));
}

/*
Color3f HomogeneousMedium::tr(const Ray3f &ray) const {
	return ray.maxt * Color3f(std::exp(- m_st.x() * ray.d.norm()),
			                  std::exp(- m_st.y() * ray.d.norm()),
			                  std::exp(- m_st.z() * ray.d.norm()));
}
*/

NORI_REGISTER_CLASS(HomogeneousMedium, "homogeneous")
NORI_NAMESPACE_END
