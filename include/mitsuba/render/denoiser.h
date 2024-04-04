#pragma once
#if !defined(__MITSUBA_RENDER_DENOISER_H_)
#define __MITSUBA_RENDER_DENOISER_H_

#include <mitsuba/render/common.h>
#include <OpenImageDenoise/oidn.hpp>

MTS_NAMESPACE_BEGIN

#define DENOISE_SURF_VOL

struct MTS_EXPORT_RENDER Denoiser {

    struct Sample{
        Spectrum color {0.f};
        Spectrum albedo {0.f};
        Vector3 normal {0.f, 0.f, -1.f};

#if defined(DENOISE_SURF_VOL)
        bool isSurface {true};
#endif

    };

    void init(const Vector2i filmSize, const bool filterFeatures = true, const bool filterSufVolume = false);
    void denoise();
    void storeBuffers(std::string filename);
    void loadBuffers(std::string filename);
    void add(int pixIdx, const Sample& sample);
    Spectrum get(int pixIdx) const;
#if defined(DENOISE_SURF_VOL)
    Spectrum getSurf(int pixIdx) const;
    Spectrum getVol(int pixIdx) const;
#endif
private:
    std::unique_ptr<Vector3[]> m_filterColor;
    std::unique_ptr<Vector3[]> m_filterAlbedo;
    std::unique_ptr<Vector3[]> m_filterNormal;
    std::unique_ptr<Vector3[]> m_filterOutput;

#if defined(DENOISE_SURF_VOL)
    std::unique_ptr<Vector3[]> m_filterSurfColor;
    std::unique_ptr<Vector3[]> m_filterSurfAlbedo;
    std::unique_ptr<Vector3[]> m_filterSurfNormal;
    std::unique_ptr<Vector3[]> m_filterSurfOutput;

    std::unique_ptr<Vector3[]> m_filterVolColor;
    std::unique_ptr<Vector3[]> m_filterVolAlbedo;
    std::unique_ptr<Vector3[]> m_filterVolNormal;
    std::unique_ptr<Vector3[]> m_filterVolOutput;
#endif

    oidn::DeviceRef m_oidnDevice;

    oidn::BufferRef m_bufferColor;
    oidn::BufferRef m_bufferAlbedo;
    oidn::BufferRef m_bufferAlbedoOutput;
    oidn::BufferRef m_bufferNormal;
    oidn::BufferRef m_bufferNormalOutput;
    oidn::BufferRef m_bufferOutput;
#if defined(DENOISE_SURF_VOL)
    oidn::BufferRef m_bufferSurfColor;
    oidn::BufferRef m_bufferSurfAlbedo;
    oidn::BufferRef m_bufferSurfAlbedoOutput;
    oidn::BufferRef m_bufferSurfNormal;
    oidn::BufferRef m_bufferSurfNormalOutput;
    oidn::BufferRef m_bufferSurfOutput;

    oidn::BufferRef m_bufferVolColor;
    oidn::BufferRef m_bufferVolAlbedo;
    oidn::BufferRef m_bufferVolAlbedoOutput;
    oidn::BufferRef m_bufferVolNormal;
    oidn::BufferRef m_bufferVolNormalOutput;
    oidn::BufferRef m_bufferVolOutput;
#endif
    std::unique_ptr<int[]> m_sampleCounts;
    
    oidn::FilterRef m_oidnAlbedoFilter;
    oidn::FilterRef m_oidnNormalFilter;
    oidn::FilterRef m_oidnFilter;

#if defined(DENOISE_SURF_VOL)
    oidn::FilterRef m_oidnSurfAlbedoFilter;
    oidn::FilterRef m_oidnSurfNormalFilter;
    oidn::FilterRef m_oidnSurfFilter;
    
    oidn::FilterRef m_oidnVolAlbedoFilter;
    oidn::FilterRef m_oidnVolNormalFilter;
    oidn::FilterRef m_oidnVolFilter;
#endif

    Vector2i m_filmSize;
    bool m_filterFeatures {false};
#if defined(DENOISE_SURF_VOL)
    bool m_filterSurfVol {false};
#endif
};

MTS_NAMESPACE_END

#endif /* __MITSUBA_RENDER_DENOISER_H_ */