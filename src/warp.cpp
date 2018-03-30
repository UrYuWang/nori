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

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Vector3f Warp::sampleUniformHemisphere(Sampler *sampler, const Normal3f &pole) {
    // Naive implementation using rejection sampling
    Vector3f v;
    do {
        v.x() = 1.f - 2.f * sampler->next1D();
        v.y() = 1.f - 2.f * sampler->next1D();
        v.z() = 1.f - 2.f * sampler->next1D();
    } while (v.squaredNorm() > 1.f);

    if (v.dot(pole) < 0.f)
        v = -v;
    v /= v.norm();

    return v;
}

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
	return Point2f(sqrt(sample.x())*cos(2*M_PI*sample.y()), sqrt(sample.x())*sin(2 * M_PI*sample.y()));
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
	return (p.norm() < 1)*INV_PI;
}

Vector3f Warp::squareToUniformCylinder(const Point2f &sample) {
	return Vector3f(cos(2*M_PI*sample.y()), sin(2 * M_PI*sample.y()), 2*sample.x() - 1);
}

Vector3f Warp::squareToUniformSphereCap(const Point2f &sample, float cosThetaMax) {
	Vector3f c = squareToUniformCylinder(sample);
	float z = c.z()*(1 - cosThetaMax) / 2 + 0.5 + cosThetaMax / 2;
	return Vector3f(c.x()*sqrt(1 - pow(z, 2)), c.y()*sqrt(1 - pow(z, 2)), z);
}


float Warp::squareToUniformSphereCapPdf(const Vector3f &v, float cosThetaMax) {
	return (abs(v.norm() - 1) < Epsilon && v.z() > cosThetaMax) * INV_PI * .5 / (1 -cosThetaMax) ;
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
	Vector3f c = squareToUniformCylinder(sample);
	return Vector3f(sqrt(1 - pow(c.z(), 2))*c.x(), sqrt(1 - pow(c.z(), 2))*c.y(), c.z());
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
	return (abs(v.norm() - 1) < Epsilon) * INV_PI * .25;
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
	Vector3f s = squareToUniformSphere(sample);
	return Vector3f(s.x(), s.y(), abs(s.z()));
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
	return (abs(v.norm() - 1) < Epsilon && v.z() >= 0) * INV_PI * .5;
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
	float phi = 2 * M_PI * sample[0];
	float theta = acos(sqrt(sample[1]));
	return Vector3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
	return (abs(v.norm() - 1) < Epsilon && v.z() >= 0) * INV_PI * v.z() / v.norm();
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
	float phi = 2 * M_PI * sample[0];
	float theta = atan(alpha * sqrt(log(1/(1-sample[1]))));
	return Vector3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
	float theta = acos(m.z() / m.norm());
	return (abs(m.norm() - 1) < Epsilon && m.z() >= 0) * exp(-pow(tan(theta), 2)/ pow(alpha, 2)) / (M_PI * pow(alpha, 2) * pow(cos(theta), 3));
}

Vector3f Warp::squareToUniformTriangle(const Point2f &sample) {
    float su1 = sqrtf(sample.x());
    float u = 1.f - su1, v = sample.y() * su1;
    return Vector3f(u,v,1.f-u-v);
}

Vector3f Warp::squareToGTR1(const Point2f & sample, float alpha)
{
	float phi = 2 * M_PI * sample[0];
	float theta = 0;
	if (alpha < 1)
		theta = acos(sqrt((1 - pow(pow(alpha, 2), 1 - sample[1])) / (1 - pow(alpha, 2))));
	return Vector3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
}

float Warp::squareToGTR1Pdf(const Vector3f & m, float alpha)
{
	float cosTheta = m.z() / m.norm();
	if (alpha >= 1)
		return INV_PI;
	return (abs(m.norm() - 1) < Epsilon && m.z() >= 0) * cosTheta * (pow(alpha, 2) - 1)*INV_PI/(2*std::log(alpha) * (1 + pow(cosTheta, 2) * (pow(alpha, 2) - 1)));
}

Vector3f Warp::squareToGTR2(const Point2f & sample, float alpha)
{
	float phi = 2 * M_PI * sample[0];
	float theta = acos(sqrt((1 - sample[1]) / (1 + (pow(alpha, 2) - 1) * sample[1])));
	return Vector3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
}

float Warp::squareToGTR2Pdf(const Vector3f & m, float alpha)
{
	float cosTheta = m.z() / m.norm();
	return (abs(m.norm() - 1) < Epsilon && m.z() >= 0) * pow(alpha, 2) * cosTheta * INV_PI / pow(1 + pow(cosTheta, 2) * (pow(alpha, 2) - 1), 2);
}



NORI_NAMESPACE_END
