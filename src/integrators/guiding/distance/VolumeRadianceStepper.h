#pragma once

#include <openpgl/cpp/OpenPGL.h>

MTS_NAMESPACE_BEGIN

namespace Guiding
{

class VolumeRadianceStepper{
	//bool init;

protected:
	const openpgl::cpp::Field* m_guidingField;

	Ray m_ray;
	Float m_t;
	Float m_T;
	//cache
	// Should this called model ?
	openpgl::cpp::VolumeSamplingDistribution* m_estimate;
	//Point cp;
	Float m_ct;
	Float m_cT;
	Spectrum m_Li;
	Spectrum m_L;

//	TPhaseRepresentation m_phaseRep;
	const PhaseFunction *m_phase;
	//Float m_avgCosine;

	int m_count;

public:


	virtual void setGuidingField(const openpgl::cpp::Field* guidingField, int wId) {
		this->m_guidingField = guidingField;
		this->m_estimate = new openpgl::cpp::VolumeSamplingDistribution(this->m_guidingField);
	}

	virtual void init(Ray ray, Float t,  const PhaseFunction *phase) = 0;


	virtual Float step(Float step, Float stepT, const Float &sigmaT, const Float &sigmaS, Float &Li, Float  &L) = 0;


	virtual std::string toString()const = 0;

	int getCount()const{
		return m_count;
	}

};

class VolumeRadianceStepperEMA: public VolumeRadianceStepper{
private:
	Float m_alpha;


public:

	VolumeRadianceStepperEMA(const Properties &props =  Properties()){
		//this->m_estimate->reset();
		this->m_alpha = props.getFloat("ema.alpha", 0.25);
		this->m_count = 0;

	}

	VolumeRadianceStepperEMA(const VolumeRadianceStepperEMA &stepper){
			this->m_estimate = stepper.m_estimate;
			this->m_alpha = stepper.m_alpha;
			//init = false;
			this->m_count = 0;
		}


	VolumeRadianceStepperEMA(Float alpha){
		//this->m_estimate->reset();
		this->m_alpha = alpha;
		this->m_count = 0;

	}

	~VolumeRadianceStepperEMA(){

//		avgCacheLookUps.incrementBase();
//		avgCacheLookUps += m_count;

	}

	void init(Ray ray, Float t/*, const TPhaseRepresentation &phaseRep*/, const PhaseFunction *phase){

		this->m_count = 0;
//		this->m_phaseRep = phaseRep;
		this->m_phase = phase;
		//this->m_avgCosine = avgCosine;
		this->m_t = t;
		this->m_ray = ray;

		Point p = ray(t);
		Vector wiWorld = ray.d;
		Vector woWorld = -ray.d;

		pgl_vec3f pglWi = openpgl::cpp::Vector3(wiWorld[0], wiWorld[1], wiWorld[2]);
		pgl_vec3f pglWo = openpgl::cpp::Vector3(woWorld[0], woWorld[1], woWorld[2]);

		//this->m_estimate->reset();
		AABB bound;
		float sample1D = -1.f;
//		uint32_t dataIdx = 0;
		//this->m_estimate->reset();
		pgl_vec3f pglP = openpgl::cpp::Point3(p[0], p[1], p[2]);
		if(this->m_estimate->Init(this->m_guidingField, pglP, sample1D)){
			Float meanCosine = this->m_phase->getMeanCosine();
			//this->m_estimate->ApplySingleLobeHenyeyGreensteinProduct(pglWo, meanCosine);
//		if(this->m_guidingField->findFieldIdx(p, dataIdx, bound)){
//			this->m_estimate->initialize(this->m_guidingField->getField(dataIdx), this->m_phaseRep, false, this->m_ray.d, false);
		//this->m_estimate = this->m_guidingField->getDistribution(p, buffer);
			this->m_count ++;
			this->m_T = 1.0;

			//if(this->m_estimate->isInitialized()){
			this->m_ct = t+this->m_estimate->GetRadius();
			pgl_vec3f pglLi = this->m_estimate->InscatteredRadiance(pglWo, meanCosine);
			this->m_Li[0] = pglLi.x;
			this->m_Li[1] = pglLi.y;
			this->m_Li[2] = pglLi.z;

			pgl_vec3f pglL = this->m_estimate->IncomingRadiance(pglWi);
			this->m_L[0] = pglL.x;
			this->m_L[1] = pglL.y;
			this->m_L[2] = pglL.z;
			this->m_cT = 1.0;
			//}else{

			//}
		}
		//SLog(EInfo, "init: m_ct: %f \t m_t: %f",m_ct, m_t);

	}

	Float step(Float step, Float stepT, const Float &sigmaT, const Float &sigmaS, Float &Li, Float  &L){
		this->m_t += step;
		if(this->m_t > this->m_ct){

			Point p = this->m_ray(this->m_t);
			Vector wiWorld = this->m_ray.d;
			Vector woWorld = -this->m_ray.d;
			pgl_vec3f pglP = openpgl::cpp::Point3(p[0], p[1], p[2]);
			pgl_vec3f pglWi = openpgl::cpp::Vector3(wiWorld[0], wiWorld[1], wiWorld[2]);
			pgl_vec3f pglWo = openpgl::cpp::Vector3(woWorld[0], woWorld[1], woWorld[2]);
			AABB bound;
			float sample1D = -1.f;
//       	uint32_t dataIdx = 0;
			//this->m_estimate->reset();
			if(this->m_estimate->Init(this->m_guidingField, pglP, sample1D)){
				Float meanCosine = this->m_phase->getMeanCosine();
				//this->m_estimate->ApplySingleLobeHenyeyGreensteinProduct(pglWo, meanCosine);
//			if(this->m_guidingField->findFieldIdx(p, dataIdx, bound)){
//				this->m_estimate->initialize(this->m_guidingField->getField(dataIdx), this->m_phaseRep, false, this->m_ray.d, false);
				// this->m_estimate = this->m_guidingField->getDistribution(p, buffer);
				this->m_count++;
				//if(this->m_estimate->isInitialized()){
					this->m_ct = this->m_t+this->m_estimate->GetRadius();
					pgl_vec3f pglLi = this->m_estimate->InscatteredRadiance(pglWo, meanCosine);
					Spectrum cLi;
					cLi[0] = pglLi.x;
					cLi[1] = pglLi.y;
					cLi[2] = pglLi.z;
					pgl_vec3f pglL = this->m_estimate->IncomingRadiance(pglWi);
					Spectrum cL;
					cL[0] = pglL.x;
					cL[1] = pglL.y;
					cL[2] = pglL.z;
					this->m_cT = 1.0;

					this->m_Li = (1.0-m_alpha)*this->m_Li + this->m_alpha* cLi;
					this->m_L = (1.0-m_alpha)*this->m_L + this->m_alpha* cL;

				//}else{

				//}
			}

		}

		Li = sigmaS*this->m_Li.average();
		L = this->m_cT*this->m_L.average();
		this->m_cT *= stepT;
		this->m_T *= stepT;

		//SLog(EInfo, "step: m_ct: %f \t m_t: %f",m_ct, m_t);

		return this->m_ct-this->m_t;
	}

	std::string toString()const{
		std::ostringstream oss;
		oss << "VolumeRadianceStepperEMA[" << endl;
		oss << "	alpha = " << this->m_alpha << endl;
		oss << "	t = " << this->m_t << endl;
		oss << "	ray = " << this->m_ray.toString() << endl;

		oss << "	T = " << this->m_T << endl;
		oss << "	L = " << this->m_L.toString() << endl;
		oss << "	Li = " << this->m_Li.toString() << endl;

		//oss << "	guidingField = " << this->m_guidingField->toString() << endl;
		oss << "	count = " << this->m_count << endl;
		oss << "]" << endl;

		return oss.str();
	}

};

}

MTS_NAMESPACE_END