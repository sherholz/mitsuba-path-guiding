#pragma once
#include <mitsuba/mitsuba.h>

MTS_NAMESPACE_BEGIN

class GuidedRussianRouletteAndSplittingProbabilities: public Object{

public:


    static Float standardRussianRouletteProbability(const Spectrum &throughput, const Float &eta){
        return std::min(throughput.max() * eta * eta, (Float) 0.95f);
    }


    static Float guidedRussianRouletteProbability(const Spectrum &throughput, const Spectrum &adjoint, const Spectrum &referenceEstimate){

        if(!adjoint.isValid() || !throughput.isValid()|| !referenceEstimate.isValid() ){
            SLog(EInfo, "throughput: %s \t adjoint: %s \t referenceEstimate: %s",throughput.toString().c_str(), adjoint.toString().c_str(),referenceEstimate.toString().c_str());      
        }

        SAssert(throughput.isValid());
        //Guiding2_Assert(adjoint.isValid());
        SAssert(referenceEstimate.isValid());

        Float survivalProb = 1.0f;
        if(adjoint.isValid() && !adjoint.isZero()){
            const Float s = 5.0f;
            // weight window center
            Spectrum Cww = referenceEstimate/adjoint;
            Cww[0] = adjoint[0] > 0.f ? Cww[0]: 0.f;
            Cww[1] = adjoint[1] > 0.f ? Cww[1]: 0.f;
            Cww[2] = adjoint[2] > 0.f ? Cww[2]: 0.f;
            if( !Cww.isValid()){
                SLog(EInfo, "throughput: %s \t adjoint: %s \t referenceEstimate: %s",throughput.toString().c_str(), adjoint.toString().c_str(),referenceEstimate.toString().c_str());
                SLog(EInfo, "Cww: %s", Cww.toString().c_str());
            }
            SAssert(Cww.isValid());
            // weight window lower bound
            Spectrum min = 2.0f*Cww/(1.0f+s);
            // weight window upper bound
            //Spectrum max = s*min;
            Float fthroughput = throughput.average();
            Float fmin = min.average();
            fmin = fmin>0 ? fmin:fthroughput;
            survivalProb = fthroughput/fmin;
            /**/
            if( std::isnan(survivalProb) || !std::isfinite(survivalProb)){
                SLog(EInfo, "throughput: %s \t adjoint: %s \t referenceEstimate: %s",throughput.toString().c_str(), adjoint.toString().c_str(),referenceEstimate.toString().c_str());
                SLog(EInfo, "survivalProb: %f, Cww: %s \t min: %s",survivalProb, Cww.toString().c_str(),min.toString().c_str());
            }
            /**/
        }
        SAssert(std::isfinite(survivalProb) && !std::isnan(survivalProb));
        survivalProb = std::max(0.1f, survivalProb);
        return std::min(survivalProb, 1.0f);
    }

};
MTS_NAMESPACE_END
