#pragma once
 
#include "DistanceSamplingTechiques.h"


MTS_NAMESPACE_BEGIN

namespace Guiding
{

class StandardDistanceSampler: public DistanceSampler{

public:

    bool sampleDistance(const Medium *medium, const Ray &ray,
        MediumSamplingRecord &mRec, Sampler *sampler, int wId) const override {
        SAssert(medium);
        return medium->sampleDistance(ray, mRec, sampler);
    }

    Spectrum evalTransmittance(const Medium *medium, const Ray &ray, Sampler *sampler) const override{
        SAssert(medium);
        return medium->evalTransmittance(ray, sampler);
    }

    std::string toString() const override{
        std::ostringstream oss;
        oss << "StandardDistanceSampler[" << endl;
        oss << "]";
        return oss.str();
    }

};

}

MTS_NAMESPACE_END

