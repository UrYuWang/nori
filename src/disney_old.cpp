#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>
#include <nori/texture.h>

NORI_NAMESPACE_BEGIN
/*
float clamp(float x, float minVal, float maxVal) {
	minVal = std::min(x, minVal);
	return std::max(minVal, maxVal);
}
*/


Color3f lerp(float t, Color3f v1,Color3f v2) {
	//return Color3f(lerp(a.x(), b.x(), fac), lerp(a.y(), b.y(), fac), lerp(a.z(), b.z(), fac));
	return (1 - t) * v1 + t * v2;
}

float SchlickFresnel(float x) {
	x = clamp(1-x, 0.f, 1.f);
	return pow(x, 5);
}

float GTR1(float cT, float a) {
	if (a >= 1)
		return INV_PI;
	return (pow(a, 2) - 1)*INV_PI/(2*std::log(a) * (1 + pow(cT, 2) * (pow(a, 2) - 1)));
}

float GTR2(float cT, float a) {
	return pow(a, 2) * INV_PI / pow(1 + pow(cT, 2) * (pow(a, 2) - 1), 2);
}

/*
float GTR2aniso(float hn, float hx, float hy, float ax, float ay) {
	return INV_PI / ax * ay * pow(pow(hx/ax, 2) + pow(hy/ay, 2) + pow(hn, 2), 2);
}
*/

float SmithGGX(float cT, float a) {
	return 1 / (cT + sqrt(pow(cT, 2) + pow(a, 2) - pow(cT * a, 2)));
}

/*
float SmithGGXaniso(float nv, float vx, float vy, float ax, float ay) {
	return 1 / (nv + sqrt(pow(vx * ax, 2) + pow(vy * ay, 2) + pow(nv, 2)));
}
*/

class DisneyBRDF : public BSDF {
public:
	DisneyBRDF(const PropertyList &propList) {
		// the surface color, usually supplied by texture maps
		m_baseColor = propList.getColor("baseColor", Color3f(0,0,0));

		// controls diffuse shape using a subsurface approximation
		m_subsurface = propList.getFloat("subsurface", 0.5);

		// the metallic-ness (0 = dielectric, 1 = metallic)
		m_metallic = propList.getFloat("metallic", 0.5);

		// incident specular amount
		m_specular = propList.getFloat("specular", 0.5);

		// a concession for artistic control that tints incident specular towards the base color
		m_specularTint = propList.getFloat("specularTint", 0.5);

		// surface roughness, controls both diffuse and specular response
		m_roughness = propList.getFloat("roughness", 0.5);

		// degree of anisotropy.  This controls the aspect ratio of the specular highlight. (0 =isotropic, 1 = maximally anisotropic)
		m_anisotropic = propList.getFloat("anisotropic", 0.5);

		// an additional grazing component, primarily intended for cloth
		m_sheen = propList.getFloat("sheen", 0.5);

		// amount to tint sheen towards base color
		m_sheenTint = propList.getFloat("sheenTint", 0.5);

		// a second, special-purpose specular lobe
		m_clearcoat = propList.getFloat("clearcoat", 0.5);

		// controls clearcoat glossiness (0 = a “satin” appearance, 1 = a “gloss” appearance)
		m_clearcoatGloss = propList.getFloat("clearcoatGloss", 0.5);
	}

	virtual Color3f eval(const BSDFQueryRecord &bRec) const override {
		if (Frame::cosTheta(bRec.wi) < 0 || Frame::cosTheta(bRec.wo) < 0)
			return Color3f(0.0f);

		//vectors
		Vector3f l = bRec.wo;
		Vector3f v = bRec.wi;
		Vector3f n(0,0,1);
		Vector3f h = (bRec.wi + bRec.wo).normalized();
		//tangent and bitangent
		//Vector3f x(1,0,0);
		//Vector3f y(0,1,0);

		// angles of incidence
		float cosTheta_l = Frame::cosTheta(l);
		float cosTheta_v = Frame::cosTheta(v);

		// angle between the normal and the half vector
		float cosTheta_h = Frame::cosTheta(h);

		// difference angle between l and the half vector
		//float cosTheta_d = bRec.wi.dot(h) / (bRec.wi.norm() * h.norm());

		// ---------- Color
		Color3f Cdlin = m_baseColor.pow(2.2);
		//approximate luminance
		float Cdlum = .3 * Cdlin.x() + .6*Cdlin.y() + .1*Cdlin.z();
		//normalize
		Color3f Ctint = (Cdlum > 0) ? Color3f(Cdlin.x() / Cdlum, Cdlin.y() / Cdlum, Cdlin.z() / Cdlum) : Color3f(1,1,1);
		Color3f Ctintmix = .08 * m_specular * lerp(m_specularTint, Color3f(1,1,1), Ctint);
		Color3f Cspec = lerp(m_metallic, Ctintmix, Cdlin);

		// ---------- Diffuse Model
		// Fresnel
		float FD90 = 0.5 + 2 * pow(l.dot(h), 2) * m_roughness;
		float FL = SchlickFresnel(cosTheta_l);
		float FV = SchlickFresnel(cosTheta_v);
		float fd = lerp(FL, 1, FD90) * lerp(FV, FD90, 1);
		// Hanrahan-Krueger subsurface BRDF approximation
		float Fss90 = pow(l.dot(h), 2) * m_roughness;
		float Fss = lerp(FL, 1.0, Fss90) * lerp(FV, 1.0, Fss90);
		float ss = 1.25 * (Fss * (1 / (cosTheta_l + cosTheta_v) - .5) + .5);

		// ---------- Specular
		/*
		float aspect = sqrt(1 - .9 * m_anisotropic);
		float ax = std::max(.001, pow(m_roughness, 2)/aspect);
		float ay = std::max(.001, pow(m_roughness, 2)*aspect);
		float Ds = GTR2aniso(h.dot(n), h.dot(x), h.dot(y), ax, ay);
		float FH = SchlickFresnel(l.dot(h));
		Color3f Fs = lerp(FH, Cspec, Color3f(1,1,1));
		float Gs = SmithGGXaniso(n.dot(l), l.dot(x), l.dot(y), ax, ay)
				* SmithGGXaniso(n.dot(v), v.dot(x), v.dot(y), ax, ay);
		*/
		float alpha = std::max(0.01, pow(m_roughness, 2));
		float Ds = GTR2(cosTheta_h, alpha);
		//float Ds = Warp::squareToGTR2Pdf(h, alpha);
		float FH = SchlickFresnel(l.dot(h));
		Color3f Fs = lerp(FH, Cspec, Color3f(1,1,1));
		float Gs = SmithGGX(cosTheta_l, alpha)
				* SmithGGX(cosTheta_v, alpha);

		// ---------- Sheen
		Color3f Csheen = lerp(m_sheenTint, Color3f(1, 1, 1), Ctint);
		Color3f Fsheen = FH * m_sheen * Csheen;

		// ---------- Clearcoat
		float Dr = GTR1(cosTheta_h, lerp(m_clearcoatGloss, .1, .001));
		//float Dr = Warp::squareToGTR1Pdf(h, lerp(m_clearcoatGloss, .1, .001));
		float Fr = lerp(FH, .04, 1);
		float Gr = SmithGGX(cosTheta_l, .25) * SmithGGX(cosTheta_v, .25);

		return (1 - m_metallic) * (INV_PI * lerp(m_subsurface, fd, ss) * Cdlin + Fsheen)
				+ Gs * Fs * Ds  + .25 * m_clearcoat * Gr * Fr * Dr;
	}


	virtual float pdf(const BSDFQueryRecord &bRec) const override {
		Vector3f h = (bRec.wi + bRec.wo).normalized();
		if (Frame::cosTheta(bRec.wo) <= 0)
			return 0.0f;

		float diffuseGTR2 = 1 / (1 + m_clearcoat);
		float diffuseR = (1 - m_metallic) / 2;

		float alpha = std::max(0.01, pow(m_roughness, 2));

		return diffuseR * Frame::cosTheta(bRec.wo) * INV_PI + (1 - diffuseR) * ((diffuseGTR2) * Warp::squareToGTR2Pdf(h, alpha) + (1 - diffuseGTR2) * Warp::squareToGTR1Pdf(h, lerp(m_clearcoatGloss, .1, .001)));
	}

	virtual Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const override {
		/// Sample the BRDF
		if (Frame::cosTheta(bRec.wi) <= 0)
			return Color3f(0.0f);

		float diffuseR = (1 - m_metallic) / 2;

		if (_sample.x() < diffuseR) {
			Point2f sample(_sample.x() / diffuseR, _sample.y());
			bRec.wo = Warp::squareToCosineHemisphere(sample);
		} else {
			Point2f sample((_sample.x() - diffuseR) / (1 - diffuseR), _sample.y());
			float diffuseGTR2 = 1 / (1 + m_clearcoat);
			if (sample.x() < diffuseGTR2) {
				Point2f sample(_sample.x() / diffuseGTR2, _sample.y());
				float alpha = pow(m_roughness, 2);
				Vector3f h = Warp::squareToGTR2(sample, alpha);
				bRec.wo = ((2.f * h.dot(bRec.wi) * h) - bRec.wi).normalized();
			} else {
				Point2f sample((sample.x() - diffuseGTR2) / (1 - diffuseGTR2), _sample.y());
				Vector3f h = Warp::squareToGTR1(sample, lerp(m_clearcoatGloss, .1, .001));
				bRec.wo = ((2.f * h.dot(bRec.wi) * h) - bRec.wi).normalized();
			}
		}

		if (Frame::cosTheta(bRec.wo) <= 0)
			return Color3f(0.0f);

		return eval(bRec) / pdf(bRec) * Frame::cosTheta(bRec.wo);
	}

	virtual std::string toString() const override {
		return tfm::format(
			"DisneyBRDF[\n"
			"  baseColor = %f %f %f,\n"
			"  subsurface = %f\n"
			"  metallic = %f\n"
			"  specular = %f\n"
			"  specularTint = %f\n"
			"  roughness = %f\n"
			"  anisotropic = %f\n"
			"  sheen = %f\n"
			"  shenTint = %f\n"
			"  clearcoat = %f\n"
			"  clearcoatGloss = %f\n"
			"]",
			m_baseColor.x(), m_baseColor.y(), m_baseColor.z(), m_subsurface, m_metallic, m_specular, m_specularTint, m_roughness, m_anisotropic, m_sheen, m_sheenTint, m_clearcoat, m_clearcoatGloss);
	}
private:
	Color3f m_baseColor;
	float m_subsurface, m_metallic, m_specular, m_specularTint, m_roughness, m_anisotropic, m_sheen, m_sheenTint, m_clearcoat, m_clearcoatGloss;
};

NORI_REGISTER_CLASS(DisneyBRDF, "disney");
NORI_NAMESPACE_END

