#pragma once

#include <mitsuba/mitsuba.h>
#include <mitsuba/core/platform.h>

#include <mitsuba/render/phase.h>
#include <mitsuba/render/medium.h>
#include <mitsuba/render/sampler.h>

#include <openpgl/cpp/OpenPGL.h>

MTS_NAMESPACE_BEGIN

namespace guiding
{

class GuidedPhaseFunction: public PhaseFunction{

public:

	GuidedPhaseFunction(const openpgl::cpp::Field* field
						, bool enableVolumeGuiding
						, Float phaseFunctionSamplingProbability
						, bool usePhaseFunctionProduct = false)
		: PhaseFunction(Properties())
		, m_enableVolumeGuiding(enableVolumeGuiding)
		, m_phaseFunctionSamplingProbability(phaseFunctionSamplingProbability)
		, m_usePhaseFunctionProduct(usePhaseFunctionProduct)
	{
		m_guidingSamplingDistribution = new openpgl::cpp::VolumeSamplingDistribution(field);
	}

	~GuidedPhaseFunction()
	{
		delete m_guidingSamplingDistribution;
	}

	void configure() override
	{
        SAssert(m_phase);
	}

	Float eval(const PhaseFunctionSamplingRecord &pRec) const override{
		return m_phase->eval(pRec);
	}


	virtual Float sample(PhaseFunctionSamplingRecord &pRec,
			Sampler *sampler) const override{
		Float pdf = 0.0f;
		Float rrCorrect = 1.0f;
		return sample(pRec, pdf, rrCorrect, sampler);
	}

	virtual Float sample(PhaseFunctionSamplingRecord &pRec,
			Float &pdf, Sampler *sampler) const override{
		Float rrCorrect = 1.0f;
		return sample(pRec, pdf, rrCorrect, sampler);
	}

	virtual Float sample(PhaseFunctionSamplingRecord &pRec,
			Float &pdf, Float &rrCorrect, Sampler *sampler) const {
		rrCorrect = 1.0f;
		// check if we have a valid guiding distribution
		bool sampleOnlyPhase = !m_enableVolumeGuiding || !m_guidingSamplingDistribution->Validate();
		
		// select between phase function sampling and guiding distribution sampling
		bool usePhase = sampleOnlyPhase || sampler->next1D() < m_phaseFunctionSamplingProbability ;

		pdf = 0.0f;
		Float out = 0.0f;

		//bool valid = false;
		if (!usePhase){

			// we choose to use the guiding dist for sampling

			// sampling based on the guiding distribution
            //Vector3 wo = m_guidingData.sample(sampler->next2D());
        	Point2 sample = sampler->next2D();
			pgl_point2f pglSample = openpgl::cpp::Point2(sample[0], sample[1]);
            pgl_vec3f pglWo = m_guidingSamplingDistribution->Sample(pglSample);
            Vector3 wo = Vector3(pglWo.x, pglWo.y, pglWo.z);
			//SAssert(Guiding::isValid(wo));
			//SLog(EInfo, "Guided dist sampling failed at !");
			pRec.wo = wo;

			// get the result of the phase function evaluation for the 
			// sampled direction
			out = m_phase->eval(pRec);

			// calcualte the MIS pdf
			//Float spdf = m_guidingData.pdf(pRec.wo);
			Float spdf = m_guidingSamplingDistribution->PDF(pglWo);

			pdf += (1.f - m_phaseFunctionSamplingProbability)* spdf;

			Float pPdf = m_phase->pdf(pRec);
			pdf += m_phaseFunctionSamplingProbability* pPdf;
			//valid = true;
			if(pPdf > 1e-4f)
				rrCorrect = pdf/pPdf;

		}

		if(usePhase ){

			// we decided to use phase function sampling
			Float pPdf = 0.0f;

			// sample using the phase function
			out = m_phase->sample(pRec, pPdf, sampler);
			//SAssert(Guiding::isValid(pRec.wo));
			// correct the sample result, since it is already
			// multiplied by the inverse pdf
			out *= pPdf;

			// if we where  only sampling the phase function
			// we do not need to add the guiding dist pdf
			if (!sampleOnlyPhase) {

				// if it would be also possible to sample the guiding dist
				// we need to adjust the pdf using MIS

				pdf = pPdf*(m_phaseFunctionSamplingProbability);
				//Float iPdf = m_guidingData.pdf(pRec.wo);
				pgl_vec3f pglWo = openpgl::cpp::Vector3(pRec.wo[0], pRec.wo[1], pRec.wo[2]);
				Float iPdf = m_guidingSamplingDistribution->PDF(pglWo);

				pdf += iPdf*(1.f-m_phaseFunctionSamplingProbability);
				if(pPdf > 1e-4f)
					rrCorrect = pdf/pPdf;
			}
			else{
				pdf = pPdf;
			}
		}

		//SAssert(Guiding::isValid(out));
		//SAssert(Guiding::isValid(pdf));

		if (pdf != 0.0f) {
			return out / pdf;
		}
		else {
			return 0.0f;
		}
	}


	virtual Float pdf(const PhaseFunctionSamplingRecord &pRec) const override{
		
		Float pdf = 0.0f;
		
		// calculate the phase function pdf
		Float pPdf = m_phase->pdf(pRec);
		pdf += pPdf;

		// check if we would even consider the guidance distribution
		bool phaseOnly = !m_enableVolumeGuiding || !m_guidingSamplingDistribution->Validate();

		// if we consider the guidance distribution adjust the pdf using MIS
		if (!phaseOnly){
			pdf *= m_phaseFunctionSamplingProbability;
			//Float iPdf = m_guidingData.pdf(pRec.wo);
			pgl_vec3f pglWo = openpgl::cpp::Vector3(pRec.wo[0], pRec.wo[1], pRec.wo[2]);
			Float iPdf = m_guidingSamplingDistribution->PDF(pglWo);


			pdf += (1.0f - m_phaseFunctionSamplingProbability)*iPdf;
		}

		return pdf;
	}


#ifdef OPENPGL_EF_RADIANCE_CACHES
    Spectrum incomingRadiance(const Vector3 &woWorld) const {
        Spectrum spec(0.f);
        if(m_init)
        {
            pgl_vec3f pglWo = openpgl::cpp::Vector3(woWorld[0], woWorld[1], woWorld[2]);
            pgl_vec3f pglIncomingRad =  m_guidingSamplingDistribution->IncomingRadiance(pglWo);
            spec[0] = pglIncomingRad.x;
            spec[1] = pglIncomingRad.y;
            spec[2] = pglIncomingRad.z;
        }
        return spec;
    }

    Spectrum outgoingRadiance(const Vector3 &wiWorld) const {
        Spectrum spec(0.f);
        if(m_init)
        {
            pgl_vec3f pglWi = openpgl::cpp::Vector3(wiWorld[0], wiWorld[1], wiWorld[2]);
            pgl_vec3f pglOutgoingRad =  m_guidingSamplingDistribution->OutgoingRadiance(pglWi);
            spec[0] = pglOutgoingRad.x;
            spec[1] = pglOutgoingRad.y;
            spec[2] = pglOutgoingRad.z;
        }
        return spec;
    }

	Spectrum inscatteredRadiance(const Vector3 &wiWorld, const float meanCosine) const {
        Spectrum spec(0.f);
        if(m_init)
        {
            pgl_vec3f pglWi = openpgl::cpp::Vector3(wiWorld[0], wiWorld[1], wiWorld[2]);
            pgl_vec3f pglInscatteredRad =  m_guidingSamplingDistribution->InscatteredRadiance(pglWi, meanCosine);
            spec[0] = pglInscatteredRad.x;
            spec[1] = pglInscatteredRad.y;
            spec[2] = pglInscatteredRad.z;
        }
        return spec;
    }

		Spectrum fluence() const {
        Spectrum spec(0.f);
        if(m_init)
        {
            pgl_vec3f pglFluence =  m_guidingSamplingDistribution->Fluence();
            spec[0] = pglFluence.x;
            spec[1] = pglFluence.y;
            spec[2] = pglFluence.z;
        }
        return spec;
    }
#endif

	/*
		Wrapping standard phase function methods
	*/

	virtual bool needsDirectionallyVaryingCoefficients() const override{
		return m_phase->needsDirectionallyVaryingCoefficients();
	}

	virtual Float sigmaDir(Float cosTheta) const override{
		return m_phase->sigmaDir(cosTheta);
	}

	virtual Float sigmaDirMax() const override{
		return m_phase->sigmaDirMax();
	}

	virtual Float getMeanCosine() const override{
		return m_phase->getMeanCosine();
	}

	/// Return a string representation
	virtual std::string toString() const override{
		std::ostringstream oss;
		oss << "GuidedPhaseFunction[" << endl
			<< "]";
		return oss.str();
	}

	void prepare(const openpgl::cpp::Field* field, const Point &p, const Vector3 &wiWorld, const PhaseFunction *volumePhase, Sampler* sampler)
	{
		m_init = false;
		this->m_phase = volumePhase;

		pgl_vec3f pglP = openpgl::cpp::Point3(p[0], p[1], p[2]);
		float sample1D = sampler->next1D();
		//float sample1D = -1.0f;
		if (m_guidingSamplingDistribution->Init(field, pglP, sample1D))
        {
			pgl_vec3f pglWi = openpgl::cpp::Vector3(-wiWorld[0], -wiWorld[1], -wiWorld[2]);
			Float meanCosine = this->m_phase->getMeanCosine();
			m_guidingSamplingDistribution->ApplySingleLobeHenyeyGreensteinProduct(pglWi, meanCosine);
			m_init = true;
        }
        else
        {
            m_guidingSamplingDistribution->Clear();
        }
        this->configure();

	}

	openpgl::cpp::Region getRegion()const
	{
		return m_guidingSamplingDistribution->GetRegion();
	}

private:

    openpgl::cpp::VolumeSamplingDistribution* m_guidingSamplingDistribution;

	/// phase function at the scattering event
	const PhaseFunction *m_phase;

	bool m_enableVolumeGuiding;

	/// the probability of sampling the phase function instead of the
	Float m_phaseFunctionSamplingProbability;

	bool m_usePhaseFunctionProduct;
	bool m_init {false};
};
}

MTS_NAMESPACE_END
