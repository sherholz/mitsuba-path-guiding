#pragma once
#if !defined(__MITSUBA_RENDER_DENOISER_H_)
#define __MITSUBA_RENDER_DENOISER_H_

#include <mitsuba/render/common.h>
#include <OpenImageDenoise/oidn.hpp>

MTS_NAMESPACE_BEGIN

struct MTS_EXPORT_RENDER DenoiseBuffer {

    struct Sample{
        Spectrum color {0.f};
        Spectrum albedo {0.f};
        Vector3 normal {0.f, 0.f, -1.f};
    };

    void init(const Vector2i filmSize);
    void denoise();
    void storeBuffers(std::string filename);
    void loadBuffers(std::string filename);
    void add(int pixIdx, const Sample& sample);
    Spectrum get(int pixIdx) const;
private:
    std::unique_ptr<Vector3[]> m_filterColor;
    std::unique_ptr<Vector3[]> m_filterAlbedo;
    std::unique_ptr<Vector3[]> m_filterAlbedoOutput;
    std::unique_ptr<Vector3[]> m_filterNormal;
    std::unique_ptr<Vector3[]> m_filterNormalOutput;
    std::unique_ptr<Vector3[]> m_filterOutput;
    std::unique_ptr<int[]> m_sampleCounts;
    oidn::DeviceRef m_oidnDevice;
    oidn::FilterRef m_oidnAlbedoFilter;
    oidn::FilterRef m_oidnNormalFilter;
    oidn::FilterRef m_oidnFilter;

    Vector2i m_filmSize;
};

MTS_NAMESPACE_END

#endif /* __MITSUBA_RENDER_DENOISER_H_ */