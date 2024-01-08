#pragma once
#if !defined(__MITSUBA_RENDER_DENOISER_H_)
#define __MITSUBA_RENDER_DENOISER_H_

#include <mitsuba/render/common.h>
#include <OpenImageDenoise/oidn.hpp>

MTS_NAMESPACE_BEGIN

struct MTS_EXPORT_RENDER Denoiser {

    struct Sample{
        Spectrum color {0.f};
        Spectrum albedo {0.f};
        Vector3 normal {0.f, 0.f, -1.f};
    };

    void init(const Vector2i filmSize, const bool filterFeatures = true);
    void denoise();
    void storeBuffers(std::string filename);
    void loadBuffers(std::string filename);
    void add(int pixIdx, const Sample& sample);
    Spectrum get(int pixIdx) const;
private:
    std::unique_ptr<Vector3[]> m_filterColor;
    std::unique_ptr<Vector3[]> m_filterAlbedo;
    std::unique_ptr<Vector3[]> m_filterNormal;

    std::unique_ptr<Vector3[]> m_filterOutput;

    oidn::DeviceRef m_oidnDevice;

    oidn::BufferRef m_bufferColor;
    oidn::BufferRef m_bufferAlbedo;
    oidn::BufferRef m_bufferAlbedoOutput;
    oidn::BufferRef m_bufferNormal;
    oidn::BufferRef m_bufferNormalOutput;
    oidn::BufferRef m_bufferOutput;
    std::unique_ptr<int[]> m_sampleCounts;
    
    oidn::FilterRef m_oidnAlbedoFilter;
    oidn::FilterRef m_oidnNormalFilter;
    oidn::FilterRef m_oidnFilter;

    Vector2i m_filmSize;
    bool m_filterFeatures {false};
};

MTS_NAMESPACE_END

#endif /* __MITSUBA_RENDER_DENOISER_H_ */