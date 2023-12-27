#pragma once
#if !defined(__MITSUBA_RENDER_DENOISER_H_)
#define __MITSUBA_RENDER_DENOISER_H_

#include <mitsuba/render/common.h>
#include <OpenImageDenoise/oidn.hpp>

MTS_NAMESPACE_BEGIN

struct DenoiseBuffer {

    void init(const Vector2i filmSize);
    void denoise();
    void storeBuffers(std::string filename);
    void add(int pixIdx, const Vector3& color, const Vector3& albedo, const Vector3& normal);
    Spectrum get(int pixIdx) const;
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