/*
    This file is part of the implementation of the SIGGRAPH 2020 paper
    "Robust Fitting of Parallax-Aware Mixtures for Path Guiding",
    as well as the updated implementation of the ACM TOG 2019 paper
    "Volume Path Guiding Based on Zero-Variance Random Walk Theory".
    The implementation extends Mitsuba, a physically based rendering system.

    Copyright (c) 2020 Lukas Ruppert, Sebastian Herholz.

    Mitsuba is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Mitsuba is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <mitsuba/mitsuba.h>
#include <mitsuba/core/platform.h>

#include <mitsuba/render/bsdf.h>
#include <mitsuba/render/sampler.h>
#include <mitsuba/render/shape.h>

#include <openpgl/cpp/OpenPGL.h>

MTS_NAMESPACE_BEGIN

namespace guiding
{

class GuidedBSDF : public BSDF {
public:

    enum GuidingTypes{
        EGUIDING_TYPE_PRODUCT = 0,
        EGUIDING_TYPE_RIS = 1,
        EGUIDING_TYPE_ROUGHNESS = 2
    };

public:

    GuidedBSDF(const openpgl::cpp::Field* field, bool enableSurfaceGuiding, Float bsdfSamplingProbability, bool useCosineProduct=true, bool useBSDFProduct=false)
        : BSDF(Properties()),
          m_enableSurfaceGuiding(enableSurfaceGuiding),
          m_bsdfSamplingProbability(bsdfSamplingProbability),
          m_useCosineProduct(useCosineProduct),
          m_useBSDFProduct(useBSDFProduct)
    {
        m_guidingSamplingDistribution = new openpgl::cpp::SurfaceSamplingDistribution(field);
	}

    ~GuidedBSDF()
    {
        delete m_guidingSamplingDistribution;
    }

    void configure() override
    {
        SAssert(m_bsdf);

        // check if we can use guiding
        m_allowGuiding = m_guidingSamplingDistribution->Validate() && bsdfTypeAllowsGuiding();

        m_glossyComponentId = -1;
        m_diffuseComponentId = -1;

        m_combinedType = m_bsdf->getType();
        m_components.resize(m_bsdf->getComponentCount());
        for (int i=0; i<m_bsdf->getComponentCount(); ++i)
        {
            m_components[i] = m_bsdf->getType(i);

            if (m_components[i]&BSDF::EDiffuse)
                m_diffuseComponentId = i;
            else if (m_components[i]&BSDF::EGlossy)
                m_glossyComponentId = i;
        }

        //if (useGuiding)
        //    m_combinedType |= BSDF::EGuiding;
    }

    Spectrum getAlbedo(const Intersection &its) const override
    {
        return m_bsdf->getAlbedo(its);
	}

    Spectrum eval(const BSDFSamplingRecord & bRec, EMeasure measure = ESolidAngle) const override
    {
        return m_bsdf->eval(bRec, measure);
	}

    Spectrum sample(BSDFSamplingRecord &bRec, const Point2 &sample) const override
    {
        if (!canUseGuiding()) {
            return m_bsdf->sample(bRec, sample);
        }

        Float pdf;
        Float rrCorrect;
        return this->sample(bRec, pdf, rrCorrect, sample);
    }

    Spectrum sample(BSDFSamplingRecord &bRec, Float &pdf, const Point2 &sample) const override
    {
        Float rrCorrect;
        return this->sample(bRec, pdf, rrCorrect, sample);
    }
    Spectrum sample(BSDFSamplingRecord &bRec, Float &pdf, Float &rrCorrect, const Point2 &sample) const
    {
        rrCorrect = 1.0f;
        if (!canUseGuiding()) {
            //use only the BSDF for sampling
            return m_bsdf->sample(bRec, pdf, sample);
        }

        const bool hasDelta = m_bsdf->getType()&bRec.typeMask&BSDF::EDelta;
        const Float deltaSamplingRate = hasDelta ? m_bsdf->getDeltaSamplingRate(bRec) : 0.0f;
        const unsigned int bRecTypemask = bRec.typeMask;

        if (hasDelta && bRec.sampler->next1D() < deltaSamplingRate) {
            //sample the delta component
            bRec.typeMask &= BSDF::EDelta;

            Spectrum throughput = m_bsdf->sample(bRec, pdf, sample);

            pdf *= deltaSamplingRate;
            throughput /= deltaSamplingRate;

            bRec.typeMask = bRecTypemask;

            return throughput;
        }

        //from here on, sample only smooth components
        bRec.typeMask &= BSDF::ESmooth;

        // select between BSDF sampling and guiding distribution sampling
        const bool sampleBSDF = bRec.sampler->next1D() < m_bsdfSamplingProbability;

        pdf = 0.0f;
        Spectrum throughput = Spectrum(0.f);
        Float bsdfPDF, guidingPDF;

        if (!sampleBSDF)
        {
            // sample the guiding distribution
            pgl_point2f pglSample = openpgl::cpp::Point2(sample[0], sample[1]);

            pgl_vec3f pglWo;
            guidingPDF = m_guidingSamplingDistribution->SamplePDF(pglSample, pglWo);
            Vector3 wo = Vector3(pglWo.x, pglWo.y, pglWo.z);
            //SAssert(Guiding::isValid(wo));
            bRec.wo = bRec.its.toLocal(wo);

            throughput = m_bsdf->eval(bRec);
            //for invalid samples, we can terminate early
            if (throughput.isZero())
            {
                //bRec.sampledType = BSDF::EGuiding;
                return Spectrum{0.0f};
            }

            bsdfPDF = m_bsdf->pdf(bRec);

            // the change in index of refraction depends upon which side has been sampled
            bRec.eta = Frame::cosTheta(bRec.wi) > 0.0f ? m_bsdf->getEta() : 1.0f/m_bsdf->getEta();

            // set the sampled component and type
            if (m_combinedType&BSDF::EGlossy && m_combinedType&BSDF::EDiffuse)
            {
                const float glossySamplingRate = m_bsdf->getGlossySamplingRate(bRec);
                bRec.typeMask &= BSDF::EGlossy;
                const Float glossyPDF = m_bsdf->pdf(bRec);
                if (glossySamplingRate*glossyPDF >= bsdfPDF*0.5f)
                    bRec.sampledComponent = m_glossyComponentId;
                else
                    bRec.sampledComponent = m_diffuseComponentId;
            }
            else if (m_combinedType&BSDF::EGlossy)
                bRec.sampledComponent = m_glossyComponentId;
            else if (m_combinedType&BSDF::EDiffuse)
                bRec.sampledComponent = m_diffuseComponentId;
            //this should never happen
            else
                SLog(EError, "BSDF has no glossy or diffuse component, yet guiding has been used for sampling.");
            bRec.sampledType = getType(bRec.sampledComponent);//|BSDF::EGuiding;
        }
        else
        {
            // sample the BSDF
            throughput = m_bsdf->sample(bRec, bsdfPDF, sample);
            //for invalid samples, we can terminate early
            if (throughput.isZero())
            {
                bRec.sampledType = m_bsdf->getType();
                return Spectrum{0.0f};
            }
            //SAssert(Guiding::isValid(bRec.wo));
            // correct the sample result, since it is already
            // multiplied by the inverse pdf
            throughput *= bsdfPDF;
            Vector3 wo = bRec.its.toWorld(bRec.wo);
            pgl_vec3f pglWo = openpgl::cpp::Vector3(wo[0], wo[1], wo[2]);
            guidingPDF = m_guidingSamplingDistribution->PDF(pglWo);
        }
        //compute the one-sample MIS PDF
        pdf = guidingPDF+(bsdfPDF-guidingPDF)*m_bsdfSamplingProbability;

        if (hasDelta)
            pdf *= 1.0f-deltaSamplingRate;

        bRec.typeMask = bRecTypemask;

        //if (EXPECT_NOT_TAKEN(!Guiding::isValid(pdf)))
        //    SLog(EWarn, "invalid PDF evaluated for direction %s in GuidedBSDF\n%s", bRec.its.toWorld(bRec.wo).toString().c_str(), toString().c_str());

        SAssert(throughput.isValid());
        //SAssert(Guiding::isValid(pdf));
        if(bsdfPDF > 1e-4f)
            rrCorrect = pdf / bsdfPDF;
        return throughput/pdf;
    }

    Spectrum sampleRIS(BSDFSamplingRecord &bRec, Float &pdf, Float &misPdf, Float &rrCorrect, const Point2 &sample) const
    {
        rrCorrect = 1.0f;
        if (!canUseGuiding()) {
            //use only the BSDF for sampling
            Spectrum bsdfWeight = m_bsdf->sample(bRec, pdf, sample);
            misPdf = pdf;
            return bsdfWeight;
        }

        const bool hasDelta = m_bsdf->getType()&bRec.typeMask&BSDF::EDelta;
        const Float deltaSamplingRate = hasDelta ? m_bsdf->getDeltaSamplingRate(bRec) : 0.0f;
        const unsigned int bRecTypemask = bRec.typeMask;

        if (hasDelta && bRec.sampler->next1D() < deltaSamplingRate) {
            //sample the delta component
            bRec.typeMask &= BSDF::EDelta;

            Spectrum throughput = m_bsdf->sample(bRec, pdf, sample);

            pdf *= deltaSamplingRate;
            throughput /= deltaSamplingRate;

            bRec.typeMask = bRecTypemask;
            misPdf = pdf;
            return throughput;
        }

        //from here on, sample only smooth components
        bRec.typeMask &= BSDF::ESmooth;

        // select between BSDF sampling and guiding distribution sampling
        //const bool sampleBSDF = bRec.sampler->next1D() < m_bsdfSamplingProbability;
        const bool sampleBSDF = false;
        Float bsdfPDF = 0.f;
        pdf = 0.0f;
        misPdf = 0.0f;
        Spectrum throughput = Spectrum(0.f);
        //Float bsdfPDF;//, guidingPDF;

        if (!sampleBSDF)
        {
            Point2 sampleRIS[2];
            sampleRIS[0] = bRec.sampler->next2D();
            sampleRIS[1] = bRec.sampler->next2D();

            float bsdfPDFRIS[2] = {0.f, 0.f};
            float guidingPDFRIS[2] = {0.f, 0.f};
            Spectrum bsdfEvalRIS[2];

            Vector3 woRIS[2];
            float weightRIS[2];
            // RIS0 - sample BSDF
            m_bsdf->sample(bRec, bsdfPDFRIS[0], sampleRIS[0]);
            bsdfEvalRIS[0] = m_bsdf->eval(bRec);
            woRIS[0] = bRec.its.toWorld(bRec.wo);
            pgl_vec3f pglWo0 = openpgl::cpp::Vector3(woRIS[0][0], woRIS[0][1], woRIS[0][2]);
            guidingPDFRIS[0] = m_guidingSamplingDistribution->PDF(pglWo0);

            // RIS1
            pgl_point2f pglSample1 = openpgl::cpp::Point2(sampleRIS[1][0], sampleRIS[1][1]);
            pgl_vec3f pglWo1;
            guidingPDFRIS[1] = m_guidingSamplingDistribution->SamplePDF(pglSample1, pglWo1);
            Vector3 wo1 = Vector3(pglWo1.x, pglWo1.y, pglWo1.z);
            //SAssert(Guiding::isValid(wo));
            woRIS[1] = wo1;
            bRec.wo = bRec.its.toLocal(wo1);
            bsdfEvalRIS[1] = m_bsdf->eval(bRec);
            bsdfPDFRIS[1] = m_bsdf->pdf(bRec);

            float sumWeightsRIS = 0.f;
            int numSamplesRIS = 0;
            if(!bsdfEvalRIS[0].isZero())
            {
                weightRIS[0] = (bsdfPDFRIS[0] * (0.5f * ((1.0f/(4.0f * M_PI))) + guidingPDFRIS[0])) / (0.5f * (bsdfPDFRIS[0] + guidingPDFRIS[0]));
                sumWeightsRIS += weightRIS[0];
                numSamplesRIS++;
            } else {
                weightRIS[0] = 0.f;
            }

            if(!bsdfEvalRIS[1].isZero())
            {
                weightRIS[1] = (bsdfPDFRIS[1] * (0.5f * ((1.0f/(4.0f * M_PI))) + guidingPDFRIS[1])) / (0.5f * (bsdfPDFRIS[1] + guidingPDFRIS[1]));
                sumWeightsRIS += weightRIS[1];
                numSamplesRIS++;
            } else {
                weightRIS[1] = 0.f;
            }

            if(numSamplesRIS == 0 || sumWeightsRIS <=0.f)
            {
                bRec.sampledType = m_bsdf->getType();
                return Spectrum{0.0f};
            }

            int idxRIS = 0;
            float sample1DRIS = sumWeightsRIS * bRec.sampler->next1D();
            float sumRis = 0.f;
            for(int i = 0; i < 2; i++)
            {
                sumRis += weightRIS[i];
                if(sample1DRIS <= sumRis)
                {
                    idxRIS = i;
                    break;
                }
            }
            
            bsdfPDF = bsdfPDFRIS[idxRIS];
            pdf = (bsdfPDFRIS[idxRIS] * (0.5f * ((1.0f/(4.0f * M_PI))) + guidingPDFRIS[idxRIS])) * (float(2)/ sumWeightsRIS);
            misPdf = (0.5f * (bsdfPDFRIS[idxRIS] + guidingPDFRIS[idxRIS]));

            throughput = (bsdfEvalRIS[idxRIS]);
            //for invalid samples, we can terminate early
            if (throughput.isZero())
            {
                bRec.sampledType = m_bsdf->getType();
                return Spectrum{0.0f};
            }
            
            //SAssert(Guiding::isValid(wo));
            bRec.wo = bRec.its.toLocal(woRIS[idxRIS]);

            // the change in index of refraction depends upon which side has been sampled
            bRec.eta = Frame::cosTheta(bRec.wi) > 0.0f ? m_bsdf->getEta() : 1.0f/m_bsdf->getEta();

            // set the sampled component and type
            if (m_combinedType&BSDF::EGlossy && m_combinedType&BSDF::EDiffuse)
            {
                const float glossySamplingRate = m_bsdf->getGlossySamplingRate(bRec);
                bRec.typeMask &= BSDF::EGlossy;
                const Float glossyPDF = m_bsdf->pdf(bRec);
                if (glossySamplingRate*glossyPDF >= pdf*0.5f)
                    bRec.sampledComponent = m_glossyComponentId;
                else
                    bRec.sampledComponent = m_diffuseComponentId;
            }
            else if (m_combinedType&BSDF::EGlossy)
                bRec.sampledComponent = m_glossyComponentId;
            else if (m_combinedType&BSDF::EDiffuse)
                bRec.sampledComponent = m_diffuseComponentId;
            //this should never happen
            else
                SLog(EError, "BSDF has no glossy or diffuse component, yet guiding has been used for sampling.");
            bRec.sampledType = getType(bRec.sampledComponent);
        }
        if(bsdfPDF > 1e-4f)
            rrCorrect = std::min(10.0f, pdf / bsdfPDF);
        //pdf = bsdfPDF;
        if (hasDelta){
            pdf *= 1.0f-deltaSamplingRate;
            misPdf *= 1.0f-deltaSamplingRate;
        }

        bRec.typeMask = bRecTypemask;

        //if (EXPECT_NOT_TAKEN(!Guiding::isValid(pdf)))
        //    SLog(EWarn, "invalid PDF evaluated for direction %s in GuidedBSDF\n%s", bRec.its.toWorld(bRec.wo).toString().c_str(), toString().c_str());

        SAssert(throughput.isValid());
        return throughput/pdf;
    }

    Float pdf(const BSDFSamplingRecord &bRec, EMeasure measure = ESolidAngle) const override {
        if (!canUseGuiding()) {
            //use only the BSDF for sampling
            return m_bsdf->pdf(bRec);
        }
        BSDFSamplingRecord bRecCopy(bRec);

        // start with evaluating the pdf for smooth reflectance
        bRecCopy.typeMask &= BSDF::ESmooth;

        const Float bsdfPDFSmooth = m_bsdf->pdf(bRecCopy, measure);
        const Vector woWorld = bRec.its.toWorld(bRec.wo);
        pgl_vec3f pglWoWorld = openpgl::cpp::Vector3(woWorld[0], woWorld[1], woWorld[2]);
        const Float guidedPDF = m_guidingSamplingDistribution->PDF(pglWoWorld);
        Float pdf = guidedPDF+(bsdfPDFSmooth-guidedPDF)*m_bsdfSamplingProbability;

        // if there is a delta component, consider its contribution
        if (m_bsdf->getType()&bRec.typeMask&BSDF::EDelta) {
            const Float deltaReflectanceRate = m_bsdf->getDeltaSamplingRate(bRec);

            bRecCopy.typeMask = bRec.typeMask&BSDF::EDelta;
            const Float bsdfPDFDelta = m_bsdf->pdf(bRecCopy, measure);

            pdf += (bsdfPDFDelta-pdf)*deltaReflectanceRate;
        }

        return pdf;
    }

    FINLINE bool canUseGuiding() const {
        return m_enableSurfaceGuiding && m_allowGuiding;
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

    Spectrum irradiance(const Vector3 &nWorld) const {
        Spectrum spec(0.f);
        if(m_init)
        {
            pgl_vec3f pglN = openpgl::cpp::Vector3(nWorld[0], nWorld[1], nWorld[2]);
            pgl_vec3f pglIrradiance =  m_guidingSamplingDistribution->Irradiance(pglN);
            spec[0] = pglIrradiance.x;
            spec[1] = pglIrradiance.y;
            spec[2] = pglIrradiance.z;
        }
        return spec;
    }
#endif
	/// Return a string representation
    std::string toString() const override {
		std::ostringstream oss;
        oss << "GuidedBSDF[\n"
            << "  bsdf: " << (m_bsdf ? m_bsdf->toString() : "nullptr") << '\n'
//            << "  region: " << (m_region ? m_region->toString() : "nullptr") << '\n'
//            << "  guidingData: " << m_guidingSamplingDistribution.toString() << '\n'
            << "  bsdfSamplingProbability: " << m_bsdfSamplingProbability << '\n'
            << "  useCosineProduct: " << m_useCosineProduct << '\n'
            << "  useBSDFProduct: " << m_useBSDFProduct << '\n'
            << ']';
		return oss.str();
	}

    void prepare(const openpgl::cpp::Field* field, const Intersection &its, const Vector3 &wiWorld, const BSDF *surfaceBSDF, Sampler *sampler)
    {
        SAssert(its.isValid());
        //SAssert(isValid(wiWorld));
        this->m_bsdf = surfaceBSDF;
        m_init = false;
        float sample1D = sampler->next1D();
        //float sample1D = -1.0f;
        pgl_point3f pgl_p = openpgl::cpp::Point3(its.p[0], its.p[1], its.p[2]);
        if (this->bsdfTypeAllowsGuiding())
        {
            if(EXPECT_NOT_TAKEN(this->m_bsdf->hasComponent(BSDF::ETransmission&~BSDF::ENull)))
            {
                m_guidingSamplingDistribution->Clear();
            }
            else if(EXPECT_TAKEN( m_guidingSamplingDistribution->Init(field, pgl_p, sample1D)))
            {
                if(this->m_useCosineProduct)
                {
                    const Vector surfaceNormalTowardsWi = its.shFrame.n*math::signum(Frame::cosTheta(its.wi));
                    pgl_point3f pglSurfaceNormalTowardsWi = openpgl::cpp::Vector3(surfaceNormalTowardsWi[0], surfaceNormalTowardsWi[1], surfaceNormalTowardsWi[2]);
                    m_guidingSamplingDistribution->ApplyCosineProduct(pglSurfaceNormalTowardsWi);
                }
                m_init = true;
            }
            else
            {
                m_guidingSamplingDistribution->Clear();
            }
        }
        else
        {
            m_guidingSamplingDistribution->Clear();
        }
        this->configure();
    }

    //TODO: should implement Mitsuba's RTTI interface
    //MTS_DECLARE_CLASS()

    FINLINE bool bsdfTypeAllowsGuiding() const {
        const int type = m_bsdf->getType();
        //TODO: should also exclude materials with very low roughness (e.g. < 0.01)
        return !(isBSDFPureSpecular(type) || isBSDFTransmissionExcludingNull(type));
        //return !isBSDFPureSpecular(type);
        //return !(isBSDFPureSpecular(type) || isBSDFTransmissionExcludingNull(type));
    }

	/// Returns true if bsdf is purely specular (i.e. only delta transmission and/or reflection)
    FINLINE static constexpr bool isBSDFPureSpecular(const int type) {
        return (type & BSDF::EAll & ~BSDF::EDelta) == 0;
	}

    /// Returns true if the bsdf contains a transmission component which is not a Null pseudo-intersection
    FINLINE static constexpr bool isBSDFTransmissionExcludingNull(const int type) {
        return (type & BSDF::EAll & BSDF::ETransmission & ~BSDF::ENull);
	}

	openpgl::cpp::Region getRegion()const
	{
		return m_guidingSamplingDistribution->GetRegion();
	}

private:

    openpgl::cpp::SurfaceSamplingDistribution* m_guidingSamplingDistribution;
    
    /// bsdf at the current intersection
    const BSDF* m_bsdf;
    int m_glossyComponentId;
    int m_diffuseComponentId;


    bool m_enableSurfaceGuiding;
    /// the probability of sampling the bsdf instead of the
    /// guiding distribution
    Float m_bsdfSamplingProbability;

    bool m_useCosineProduct;
    bool m_useBSDFProduct;
    bool m_allowGuiding;
    bool m_init {false};
};
}
MTS_NAMESPACE_END