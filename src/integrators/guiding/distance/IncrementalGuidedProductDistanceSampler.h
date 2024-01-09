#pragma once
 
#include <mitsuba/mitsuba.h>

#include "DistanceSamplingTechiques.h"

#include "VolumeRadianceStepper.h"

#define NO_POINTERS
#define RGB_TRANS

MTS_NAMESPACE_BEGIN


namespace Guiding
{

inline float sampleSegmentUni(Float range, Float sigmaT, Float sample, Float &pdf){
	Float t = sample * range;
	pdf = 1.0 / range;
	return t;
}

inline Float sampleSegmentExp(Float range, Float sigmaT, Float sample, Float &pdf){
	Float a = 0.0;
	Float b = range;
	Float t = a - math::fastlog(1 - sample*(1 - math::fastexp(-(b - a)*sigmaT))) / sigmaT;
	pdf = sigmaT / (math::fastexp(sigmaT * (t - a)) - math::fastexp(sigmaT * (t - b)));
	return t;
}

class IncrementalGuidedProductDistanceSampler: public DistanceSampler{

public:

    enum SegmentSamplingType{
		EUniform,
		EExponential
	};

    IncrementalGuidedProductDistanceSampler(const Properties &props){
    
    
    #ifdef NO_POINTERS
        Scheduler* sched = Scheduler::getInstance();
        m_nCores = sched->getCoreCount();
        
        SLog(EInfo, "Allocating stepper buffer for %d cores", m_nCores);
        for (int i =0; i<m_nCores; i++){
            VolumeRadianceStepperEMA *irStepper = new VolumeRadianceStepperEMA(props);
            //irStepper->setGuidingField()
            m_volumeRadianceSteppers.push_back(irStepper);
        }

        if(m_nCores==0){
            SLog(EInfo, "create backup stepper");
            VolumeRadianceStepperEMA *irStepper = new VolumeRadianceStepperEMA(props);
            m_volumeRadianceSteppers.push_back(irStepper);
        }
    #endif
    }

    virtual void setGuidingField(const openpgl::cpp::Field* guidingField) override {
        this->m_guidingField = guidingField;
        for (size_t i =0; i<m_volumeRadianceSteppers.size(); i++){
            m_volumeRadianceSteppers[i]->setGuidingField(this->m_guidingField, i);
        }
    }

    bool sampleDistance(const Medium *medium, const Ray &ray,
        MediumSamplingRecord &mRec, Sampler *sampler, int wId) const override {
        SAssert(medium);
        return sampleDistanceCache(medium, ray, mRec, sampler, wId);
    }

    Spectrum evalTransmittance(const Medium *medium, const Ray &ray, Sampler *sampler) const override{
        SAssert(medium);
        return medium->evalTransmittance(ray, sampler);
    }

    std::string toString() const override{
        std::ostringstream oss;
        oss << "IncrementalGuidedProductDistanceSampler[" << endl;
        oss << "]";
        return oss.str();
    }


private:


    bool sampleDistanceCache(const Medium *medium, const Ray &ray,
        MediumSamplingRecord &mRec, Sampler *sampler, int wId) const{

        typedef Vector4 Float4;
        
        //int wId = 0;

        //int debugCount = 0;
        //SLog(EInfo, "ProductTrackingVolumeDistanceSampler::sampleDistanceCache: wId: %d",wId);
        //return false;
        //typedef Importance::Float4 Float4;

        //const Medium *medium = rRec.medium;

        bool sucess = false;

        //check if we are really inside a medium
        /** ---- Part 1/4 checking if the queried ray is valid ---- **/


        //check if we are really inside a medium
        if (medium == NULL){
            SLog(EError, "sampleDistance: medium is NULL");
        }

    #ifdef DEBUG_PRINTS
        if (!m_distrCache->contains(ray.o)){
            SLog(EInfo, "ray origin is not inside volume cache: p: %s \t aabb: %s", ray.o.toString().c_str(), m_distrCache->getAABB().toString().c_str());
        }
    #endif
        // Mitsuba BUG: wrongly estimated cut between AABB and ray (maxt == inf)
        if (std::isinf(ray.maxt) || ray.maxt <= 0.f){
            mRec.t = ray.maxt;
            mRec.p = ray(mRec.t);
            mRec.sigmaS = Spectrum(0.0);
            mRec.sigmaA = Spectrum(0.0);
            mRec.transmittance = Spectrum(1.0);
            mRec.pdfSuccess = 1.f;
            mRec.pdfFailure = 1.0f;
            mRec.time = ray.time;
            mRec.medium = NULL;
            //fillDebug(mRec, 0.0, 0.0, 1.0);

            SAssert(mRec.pdfSuccess >= 0.0f);
            SAssert(mRec.pdfFailure >= 0.0f);

            return false;
        }

        // get max volume extend
        pgl_box3f pglVBBox = this->m_guidingField->GetSceneBounds();
        AABB vAABB = AABB(Point3(pglVBBox.lower.x, pglVBBox.lower.y, pglVBBox.lower.z), Point3(pglVBBox.upper.x, pglVBBox.upper.y, pglVBBox.upper.z));

        Float aabb_mint, aabb_maxt = 0.0f;
        bool intersectAABB = vAABB.rayIntersect(ray, aabb_mint, aabb_maxt);

        // setting min and max distance for the tabulation
        Float minT = std::max(aabb_mint, ray.mint);
        Float maxT = std::min(aabb_maxt, ray.maxt);

        Float maxDist = maxT - minT;

        // abort the distance sampling, if the ray does not intersect the volume ABBB,
        // if the end of the intersection with the AABB lies behind the origin of the ray,
        // or if the distance between beginning and end is too small
        if (!intersectAABB || maxT < 0.0 || maxT == aabb_maxt || maxDist < Epsilon || aabb_mint > 1e-4f){
            mRec.t = 0.f;
            mRec.p = ray(mRec.t);
            mRec.sigmaS = Spectrum(0.0);
            mRec.sigmaA = Spectrum(0.0);
            mRec.transmittance = Spectrum(1.0);
            mRec.pdfSuccess = 0.f;
            mRec.pdfFailure = 1.0f;
            mRec.time = ray.time;
            mRec.medium = NULL;
            //fillDebug(mRec, 0.0, 0.0, 1.0);

            SAssert(mRec.pdfSuccess >= 0.0f);
            SAssert(mRec.pdfFailure >= 0.0f);

            return false;

        }

        SAssert(vAABB.contains(ray.o) || (!vAABB.contains(ray.o) && aabb_mint < 1e-4f));

        // create a new ray with the clipped range
        Ray rayRange = Ray(ray, minT, maxT);

        Float samplingWeight = 1.0f;
        //if (!m_forceInteraction){
        //	samplingWeight = 1.0f - medium->evalTransmittance(ray, sampler).average();
        //}

        SAssert(std::isfinite(samplingWeight));

        // the scale of the medium density/extinsion coefficient
        //Float mScale = medium->getScale();

        //Float avgCosine = medium->getPhaseFunction()->getMeanCosine();
        const PhaseFunction *phase = medium->getPhaseFunction();
//        const PhaseFunctionRepresentation<TVMF> phaseRep = m_pfOracle->getPhaseFunctionRepresentation(phase); 

        // the distance traveled along the ray
        Float t = 0.0;
        // current transmittance
    #ifndef RGB_TRANS
        Float T = 1.0;
    #else
        Float4 T(1.0f);
    #endif
        // integrated density along the ray
        //Float intDensity = 0.0f;

        // get the ray intersector from the media data setvAABB
        //RayIntersector *rayIntersector = medium->getVolumeData()->getRayIntersector(rayRange);

        // the pdf of reaching the current ray segment without generating a scattering event
        Float pdfTLi = 1.0f;
        Float pdfT = 1.0;

        bool sampleTrans = false;
        if(sampler->next1D() < m_transProb){
            sampleTrans = true;
        }

        // if we reached the end of the data structure
        bool end = false;

        // debug stats to count the number of march end DDA steps

        bool initCache = false;
    #ifndef NO_POINTERS
        IncommingRadianceStepper *irStepper = createIncomingRadianceStepper();
        irStepper->setDistributionCache(m_distrCache,0);
    #else
        //mitsuba::LocalWorker *wrk = static_cast<mitsuba::LocalWorker*>(mitsuba::Thread::getThread());
        //int wId = wrk->getID();
        //SAssert(wId<m_nCores);
        VolumeRadianceStepper *irStepper = m_volumeRadianceSteppers[wId];
        //irStepper->setDistributionCache(m_distrCache, wId);
    #endif
        // while ther is still an non-empty density area along the ray
        // preforme some DDA steps and fill the tabs
        //while(rayIntersector->march(ddaT0, ddaT1) && !end){
        if(!initCache){
            //irStepper.init(rayRange, t, buffer);
            SAssert(t<=0.0);
            irStepper->init(rayRange, t, /*phaseRep,*/ phase);
            initCache = true;
        }

        Float dt = 0.0f;
    #ifndef RGB_TRANS
        Float T0 = 1.0f;
    #else
        Float4 T0 = Float4(1.0f);
    #endif
        //SLog(EInfo, "maxDist: %f ",maxDist);
        while(t < (maxDist - 1e-6f) && !end){

            // make sure that we query inside the volume data set
            Float rt = std::max(Epsilon, t);
            rt = std::min(maxDist-Epsilon, rt);

            // get the current position along the ray
            Point p = rayRange(minT + rt);

            Spectrum albedo = medium->getAlbedo(p);
            Spectrum tmpSigmaT = medium->getSigmaT(p);

            Spectrum tmpSigmaS = tmpSigmaT*albedo;


            Float avgSigmaT = tmpSigmaT.average();
            Float avgSigmaS = tmpSigmaS.average();

    #ifdef RGB_TRANS
            Float4 sigmaT = Float4(tmpSigmaT[0], tmpSigmaT[1],tmpSigmaT[2],avgSigmaT);
            Float4 sigmaS = Float4(tmpSigmaS[0], tmpSigmaS[1],tmpSigmaS[2],avgSigmaS);
    #endif
            Float L = 0.0f;
            Float Li = 0.0f;

            //irStepper.step(dt, T0, sigmaT, sigmaS, Li, L, buffer);
    #ifndef RGB_TRANS
            dt = irStepper->step(dt, T0, avgSigmaT, avgSigmaS, Li, L);
    #else
            dt = irStepper->step(dt, T0[3], sigmaT[3], sigmaS[3], Li, L);
    #endif

            if(dt <=0.0f){
                SLog(EInfo, "dt >0.0f: %f");
            }

            SAssert(dt> 0.0f);
            dt *= m_stepMultiplier;

            dt = std::min(dt, maxDist- t);

            SAssert(dt > 0);

            //SLog(EInfo, "dt: %f \t Li: %f \t L: %f",dt, Li, L);

            if (avgSigmaT > 0.0f){


                //SAssert(avgAlbedo > 0.0f);
                SAssert(avgSigmaS > 0.0f);

    #ifndef RGB_TRANS
                Float T0 = math::fastexp(-dt*avgSigmaT);
                Float intT = (1.0f - T0) / avgSigmaT;
    #else
                Float4 tmp = -dt*sigmaT;
                // TODO: MAKE FAST
                Float4 T0 = Float4(std::min(1.0f-1e-4f, math::fastexp(tmp[0])),std::min(1.0f-1e-4f, math::fastexp(tmp[1])),std::min(1.0f-1e-4f, math::fastexp(tmp[2])),std::min(1.0f-1e-4f, math::fastexp(tmp[3])));
                // the transmittance of the tab
                // TODO: MAKE FAST
                Float4 intT = Float4((1.0f - T0[0]) / sigmaT[0],(1.0f - T0[1]) / sigmaT[1],(1.0f - T0[2]) / sigmaT[2],(1.0f - T0[3]) / sigmaT[3]);
    #endif
                    // approximations for the incoming and in-scattered radiance

                //SLog(EInfo, "avgSigmaT: %f \t T0: %f",avgSigmaT, T0);
                    //the transmitted incoming radiance
    #ifndef RGB_TRANS
                Float TL = T*L;
    #else
                Float TL = T[3]*L;
    #endif
                // approximate TL after doing a step of size stepT
                //Float TL0 = (TL - T*(Li*intT));
    #ifndef RGB_TRANS
                Float TLi = T*(Li*intT);
    #else
                Float TLi = T[3]*(Li*intT[3]);
    #endif
                // calculate the probability of scattering in this ray segment
                // baed on the product of transmittance and in-scattered radiance
                //Float PL = TLi/ TL;
                Float PLi = TLi/ (TL);
                //
				// TODO: find reason
				//
                if(!std::isfinite(PLi)){
                    SLog(EInfo, "PLi isfinite: %f \t TLi: %f \t TL: %f \t T0: %f \t dt: %f", PLi, TLi, TL, T0, dt);
                    PLi = 0.1;
                }
                //
				// TODO: find reason
				//
                if(!(PLi>0.0)){
                    SLog(EInfo, "PLi !> 0 : %f \t TLi: %f \t TL: %f \t T0: %f \t dt: %f", PLi, TLi, TL, T0, dt);
                    PLi = 0.1;
                }

                if(m_conservative){
                    // conservative version bounds PLI to 1.0
                    PLi = TLi/ (TLi+TL);
                }


                // check if the transmitted in-scattered radiance is bigger
                // then the incident radiance (!! this can be caused by errors in the estimates !!)
                if(PLi >= 1.0f){
                    //SLog(EInfo, "PLi>= 1.0 : %f", PLi);
                }
                // add a safeguard for the scattering probability to avoid a bias
                PLi = std::min(PLi, 0.90f);

                // calulate the probabiltiy of scattering in this ray saegment
                // based on transmittance
    #ifndef RGB_TRANS
                Float PT = (1.0 - T0);
    #else
                Float PT = (1.0 - T0[3]);

                //
				// TODO: find reason
				//
                if( !(PT> 0.0f)){
                    SLog(EInfo, "PT !> 0.0 : %f \t T0: %f \t dt: %f", PT, T0[3], dt);
                }

    #endif

                SAssert(std::isfinite(PT));
                SAssert(PT >= 0.0f);

                SAssert(std::isfinite(PLi));
                SAssert(PLi >= 0.0f);
                //SAssert(PLi > 0.0f);

                if (PLi<0.0f){
                    SLog(EInfo, "PLi<0.0 : %f", PLi);
                }


                // based in the previous decision at the begining set the decision
                // probability to the product or transmittance prob
                float P = PLi;
                if(sampleTrans){
                    P = PT;
                }

                //SLog(EInfo, "P: %f ",P);

                //SAssert(P > 0.0);
                P = std::max(0.0f, P);

                // decide if we scatter in this ray segment
                if (sampler->next1D() < P){

                    // update the pdfs for scattering in this ray segment
                    pdfT *= PT;
                    pdfTLi *= PLi;
                	//
					// TODO: find reason
					//
                    if (!(pdfT>0.0f)){
    #ifndef RGB_TRANS
                        SLog(EInfo, "pdfT!>0.0 : %f \t PT: %f \t T0: %f \t dt: %f", pdfT, PT, T0, dt);
    #else
                        SLog(EInfo, "pdfT!>0.0 : %f \t PT: %f \t T0: %f \t dt: %f", pdfT, PT, T0[3], dt);
    #endif
                    }

                    SAssert(std::isfinite(pdfT) && pdfT > 0.0f);
                    SAssert(std::isfinite(pdfTLi) && pdfTLi > 0.0f);
                    // sample a positin inside the segment
                    //Float segStep = (stepT*sampler->next1D());
                    // update the pdf for scattering at a given position in this ray segment
                    // TODO: do not scatter uniformly in the segment
                    // use T based sampling for segments
                    //pdf *= 1.0 / stepT;

                    Float spdf = 0.0f;
                    Float sampleSegment = 0.0f;

                    if (m_segSamplingType == EUniform){
    #ifndef RGB_TRANS
                        sampleSegment = sampleSegmentUni(dt, avgSigmaT, sampler->next1D(), spdf);
    #else
                        sampleSegment = sampleSegmentUni(dt, sigmaT[3], sampler->next1D(), spdf);

    #endif
                    }
                    else{
    #ifndef RGB_TRANS
                        sampleSegment = sampleSegmentExp(dt, avgSigmaT, sampler->next1D(), spdf);
    #else
                        sampleSegment = sampleSegmentExp(dt, sigmaT[3], sampler->next1D(), spdf);

    #endif
                    }


                    // calcualtes the distance of the sampled position
                    Float sampleDistance = t + sampleSegment;
                    sampleDistance = std::min(sampleDistance, maxDist);

    #ifndef RGB_TRANS
                    T *= math::fastexp(-sampleSegment*avgSigmaT);
    #else
                    // TODO: make fast
                    T = Float4(T[0]*math::fastexp(-sampleSegment*sigmaT[0]),T[1]*math::fastexp(-sampleSegment*sigmaT[1]),T[2]*math::fastexp(-sampleSegment*sigmaT[2]),T[3]*math::fastexp(-sampleSegment*sigmaT[3]));
    #endif
                    // calculate the MIS PDF of scattering in this ray segment
                    Float pdf = m_transProb*pdfT + (1.0-m_transProb)*pdfTLi;
                    // add the probability to sample this given distance inside the ray segment
                    pdf *= spdf;
                    if(!std::isfinite(pdf) && pdf <= 0.0f){
                    
                        SLog(EInfo, "pdf: %f \t spdf: %f",pdf, spdf);
                        SLog(EInfo, "pdfT: %f \t pdfTLi: %f",pdfT, pdfTLi); 
                        SLog(EInfo, "maxDist: %f \t dt: %f",maxDist, dt); 
                
                    }
                    SAssert(std::isfinite(pdf) && pdf > 0.0f);

                    //TODO: fill in stuff
                    mRec.t = minT + sampleDistance;
                    mRec.p = ray(mRec.t);
                    if (!vAABB.contains(mRec.p)){
                        SLog(EInfo, "generated point is not inside volume cache: p: %s \t aabb: %s", mRec.p.toString().c_str(), vAABB.toString().c_str());
                    }
    #ifndef RGB_TRANS
                    mRec.sigmaS = Spectrum(avgSigmaS);
                    mRec.sigmaA = Spectrum(avgSigmaT-avgSigmaS);
    #else
                    mRec.sigmaS[0] = sigmaS[0];
                    mRec.sigmaS[1] = sigmaS[1];
                    mRec.sigmaS[2] = sigmaS[2];

                    mRec.sigmaA[0] = sigmaT[0]-mRec.sigmaS[0];
                    mRec.sigmaA[1] = sigmaT[1]-mRec.sigmaS[1];
                    mRec.sigmaA[2] = sigmaT[2]-mRec.sigmaS[2];
    #endif
                    mRec.medium = medium;

                    // evaluate the ransmittance for the sampled distance
                    Ray tRay(ray.o, ray.d, minT, mRec.t, 0.0);
                    //T = medium->evalTransmittance(tRay, sampler).average();
    #ifndef RGB_TRANS
                    mRec.transmittance = Spectrum(T);
    #else
                    mRec.transmittance[0] = T[0];
                    mRec.transmittance[1] = T[1];
                    mRec.transmittance[2] = T[2];
    #endif
                    mRec.pdfSuccess = (samplingWeight)*pdf;
                    SAssert(std::isfinite(mRec.pdfSuccess) && mRec.pdfSuccess > 0.0f);
                    mRec.pdfFailure = 1.0 - (pdf*sampleSegment);
                    mRec.time = ray.time;
                    sucess = true;
                    end = true;
    #ifndef RGB_TRANS
                    //fillDebug(mRec, TL, Li, T);
    #else
                    //fillDebug(mRec, TL, Li, T[3]);
    #endif
                    break;

                }// end scatter?

                // update pdf for not scattering before the next segment
                //pdfT *= (1.0f - PT);
                // TODO: make fast
    #ifndef RGB_TRANS
                T = T*T0;
    #else
                T = Float4(T[0]*T0[0],T[1]*T0[1],T[2]*T0[2],T[3]*T0[3]);
    #endif

                // update the PDFs for not scatting until the current ray segment
                // using the different probs
                pdfTLi *= (1.0f - PLi);
                if(pdfTLi <= 0.0f){
                    SLog(EInfo, "PLi: %f \t pdfTLi: %f",PLi, pdfTLi);
                    SLog(EInfo, "PT: %f \t pdfT: %f",PT, pdfT); 
                    SLog(EInfo, "dt: %f \t dt: %f",dt, dt); 
                }
                SAssert(!std::isnan(pdfTLi));
                SAssert(std::isfinite(pdfTLi));
                SAssert(pdfTLi >= 0.0f);
                //SAssert(pdfTLi > 0.0f);

                pdfT *= (1.0f - PT);
                SAssert(std::isfinite(pdfT));
                SAssert(pdfT >= 0.0f);
                //SAssert(pdfT > 0.0f);
            }// end current DDA step
                // step along the ray
            t += dt;
            //}// end DDA stepping
        }// end non-empty space marching
//        avgCacheLookUps2.incrementBase();
//        avgCacheLookUps2 += irStepper->getCount();
    #ifndef NO_POINTERS
        delete irStepper;
    #endif
        //delete rayIntersector;

        if(!sucess){
            // TODO: fill in stuff
            mRec.t = minT + maxDist;
            //mRec.p = ray(mRec.t);
            mRec.sigmaS = Spectrum(0.0);
            mRec.sigmaA = Spectrum(1.0);
            mRec.medium = medium;
            Ray tRay(ray, minT, mRec.t);

            Float pdf = m_transProb*pdfT + (1.0-m_transProb)*pdfTLi;
            mRec.pdfSuccess = (samplingWeight)*pdf;
            // the pdf of not scattering inside the media is
            // 1.0- the integral of scattering inside the media
            //mRec.pdfFailure = 1.0 - intPdf;
            mRec.pdfFailure = (samplingWeight)*pdf + (1.0f - samplingWeight)*1.0f;
            SAssert(mRec.pdfFailure >= 0.0f);

    #ifndef RGB_TRANS
            mRec.transmittance = Spectrum(T);
            //fillDebug(mRec, 0.0, 0.0, T);
    #else
            mRec.transmittance[0] = T[0];
            mRec.transmittance[1] = T[1];
            mRec.transmittance[2] = T[2];
            //fillDebug(mRec, 0.0, 0.0, T[3]);
    #endif
        }

        SAssert(mRec.pdfSuccess >= 0.0f);
        SAssert(mRec.pdfFailure >= 0.0f);
        return sucess;
    }

private:
    Float m_transProb {0.5f};
    Float m_stepMultiplier {1.01f};

    int m_nCores {1};
    bool m_conservative = {false};
    SegmentSamplingType m_segSamplingType {EExponential};

 //   const PhaseFunctionOracleVMF<TVMF> *m_pfOracle;

    //TGuidingField* m_guidingField;
    std::vector<VolumeRadianceStepper*> m_volumeRadianceSteppers;

};

}

MTS_NAMESPACE_END

