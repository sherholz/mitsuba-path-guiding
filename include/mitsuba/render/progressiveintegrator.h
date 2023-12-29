
#pragma once
#if !defined(__MITSUBA_RENDER_PROGINTEGRATOR_H_)
#define __MITSUBA_RENDER_PROGINTEGRATOR_H_

#include <mitsuba/render/integrator.h>
#include <limits>

MTS_NAMESPACE_BEGIN

class MTS_EXPORT_RENDER ProgressiveMonteCarloIntegrator : public MonteCarloIntegrator{

 public:
     /// Serialize this integrator to a binary data stream
     void serialize(Stream *stream, InstanceManager *manager) const;

     MTS_DECLARE_CLASS()

    bool preprocess(const Scene *scene, RenderQueue *queue, const RenderJob *job,
			int sceneResID, int sensorResID, int samplerResID);

	inline int renderSamples(ref<Scheduler> sched, Scene *scene,
			RenderQueue *queue, const RenderJob *job,
			int sceneResID, int sensorResID, int samplerResID, int integratorResID);


	inline int renderTime(ref<Scheduler> sched, Scene *scene,
			RenderQueue *queue, const RenderJob *job,
			int sceneResID, int sensorResID, int samplerResID, int integratorResID);

	virtual bool render(Scene *scene,
			RenderQueue *queue, const RenderJob *job,
			int sceneResID, int sensorResID, int samplerResID);


	void renderBlock(const Scene *scene,
			const Sensor *sensor, Sampler *sampler, ImageBlock *block,
			const bool &stop, const std::vector< TPoint2<uint8_t> > &points) const;

    void cancel();

    virtual void postprocess(const Scene *scene, RenderQueue *queue,
        const RenderJob *job, int sceneResID, int sensorResID,
        int samplerResID);

    virtual void preprogression(RenderQueue *queue, const RenderJob *job,
			int sceneResID, int sensorResID, int samplerResID);

    virtual void postprogression(RenderQueue *queue, const RenderJob *job,
			int sceneResID, int sensorResID, int samplerResID);

    std::string toString() const;

 protected:
     /// Create a integrator
     ProgressiveMonteCarloIntegrator(const Properties &props);
     /// Unserialize an integrator
     ProgressiveMonteCarloIntegrator(Stream *stream, InstanceManager *manager);
    /// Virtual destructor
     virtual ~ProgressiveMonteCarloIntegrator();

protected:

    /// Given time to render the image
    Float m_maxRenderTime;
    //ir::ImageBuffer * m_imageBuffer;
    ref<Timer> m_timer;

    ref<Timer> m_progressionTimer;
    int m_progressionCounter {0};
    /// Should the rendering stop
    volatile bool m_cancel;

    //final stats
    float m_renderTime;
    //int m_iterations;

    int m_spp;
    int m_samplesPerProgression;

    std::vector<ref<Sampler>> m_samplers;
    Vector2i m_filmSize;

    std::vector<ref<BlockedRenderProcess>> m_renderProcesses;
    float m_maxComponentValue {std::numeric_limits<float>::infinity()};
};


MTS_NAMESPACE_END

#endif /* __MITSUBA_RENDER_INTEGRATOR_H_ */
