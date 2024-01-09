#pragma once

#include <openpgl/cpp/OpenPGL.h>

MTS_NAMESPACE_BEGIN

namespace Guiding
{

class DistanceSampler{

public:

    virtual bool sampleDistance(const Medium *medium, const Ray &ray, MediumSamplingRecord &mRec, Sampler *sampler, int wId) const = 0;
    
    virtual Spectrum evalTransmittance(const Medium *medium, const Ray &ray, Sampler *sampler) const = 0;

    virtual std::string toString() const = 0;

    virtual void setGuidingField(const openpgl::cpp::Field* guidingField) {
        m_guidingField = guidingField;
    }

protected:
    const openpgl::cpp::Field* m_guidingField;


};


enum DistanceSamplerTypes{
    EStandardDS = 0,
    EIncrementalGuidedProductDS,
    //ENaviGuidedProductDS
};

}

MTS_NAMESPACE_END