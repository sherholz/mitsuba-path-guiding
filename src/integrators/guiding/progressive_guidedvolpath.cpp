/*
    This file is part of Mitsuba, a physically based rendering system.

    Copyright (c) 2007-2014 by Wenzel Jakob and others.

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

#include <mitsuba/core/statistics.h>
#include <mitsuba/core/bitmap.h>

#include <mitsuba/core/fresolver.h>

#include <mitsuba/render/denoiser.h>
#include <mitsuba/render/scene.h>
#include <mitsuba/render/progressiveintegrator.h>

// including headers from the guiding libary
#include <openpgl/cpp/OpenPGL.h>

#include "GuidingHelper.h"            // type definitions and Mitsuba Property parser
#include "GuidedBSDF.h"                            // implementation of a guided (MIS-bases) BSDF wrapper
#include "GuidedPhaseFunction.h"    // implementation of a guided (MIS-bases) phase functiom wrapper
#include "GuidedRussianRouletteAndSplitting.h"

#include "distance/DistanceSamplingTechiques.h"
#include "distance/StandardDistanceSampler.h"
#include "distance/IncrementalGuidedProductDistanceSampler.h"

#include <atomic>

#ifdef OPENPGL_EF_RADIANCE_CACHES
#define GUIDING_RR
#endif

#define SCATTER_GUIDING
//#define GUIDING_SIGMA_S

// typedef of the guiding classes
// guided wrapper classes for the BSDF and phase function
typedef mitsuba::guiding::GuidedBSDF GuidedBSDFType;
typedef mitsuba::guiding::GuidedPhaseFunction GuidedPhaseFunctionType;

MTS_NAMESPACE_BEGIN

// conversion of the spectrum type to a scalar float type used
// fitting the guiding distriubtions
inline float SPECTRUM_TO_FLOAT(Spectrum spectrum)
{
    return (spectrum[0] + spectrum[1] + spectrum[2] ) / 3.0f;
}

static StatsCounter avgPathLength("Guided volumetric path tracer RK", "Average path length", EAverage);
static StatsCounter avgPerIterationPathLength("Guided volumetric path tracer RK", "Average per iteration path length", EAverage);

/*!\plugin{volpath}{Extended volumetric path tracer}
 * \order{4}
 * \parameters{
 *     \parameter{maxDepth}{\Integer}{Specifies the longest path depth
 *         in the generated output image (where \code{-1} corresponds to $\infty$).
 *         A value of \code{1} will only render directly visible light sources.
 *         \code{2} will lead to single-bounce (direct-only) illumination,
 *         and so on. \default{\code{-1}}
 *     }
 *     \parameter{rrDepth}{\Integer}{Specifies the minimum path depth, after
 *        which the implementation will start to use the ``russian roulette''
 *        path termination criterion. \default{\code{5}}
 *     }
 *     \parameter{strictNormals}{\Boolean}{Be strict about potential
 *        inconsistencies involving shading normals? See
 *        page~\pageref{sec:strictnormals} for details.
 *        \default{no, i.e. \code{false}}
 *     }
 *     \parameter{hideEmitters}{\Boolean}{Hide directly visible emitters?
 *        See page~\pageref{sec:hideemitters} for details.
 *        \default{no, i.e. \code{false}}
 *     }
 * }
 *
 * This plugin provides a volumetric path tracer that can be used to
 * compute approximate solutions of the radiative transfer equation.
 * Its implementation makes use of multiple importance sampling to
 * combine BSDF and phase function sampling with direct illumination
 * sampling strategies. On surfaces, it behaves exactly
 * like the standard path tracer.
 *
 * This integrator has special support for \emph{index-matched} transmission
 * events (i.e. surface scattering events that do not change the direction
 * of light). As a consequence, participating media enclosed by a stencil shape (see
 * \secref{shapes} for details) are rendered considerably more efficiently when this
 * shape has \emph{no}\footnote{this is what signals to Mitsuba that the boundary is
 * index-matched and does not interact with light in any way. Alternatively,
 * the \pluginref{mask} and \pluginref{thindielectric} BSDF can be used to specify
 * index-matched boundaries that involve some amount of interaction.} BSDF assigned
 * to it (as compared to, say, a \pluginref{dielectric} or \pluginref{roughdielectric} BSDF).
 *
 * \remarks{
 *    \item This integrator will generally perform poorly when rendering
 *      participating media that have a different index of refraction compared
 *      to the surrounding medium.
 *    \item This integrator has poor convergence properties when rendering
 *      caustics and similar effects. In this case, \pluginref{bdpt} or
 *      one of the photon mappers may be preferable.
 * }
 */
class GuidedProgressiveVolumetricPathTracer : public ProgressiveMonteCarloIntegrator {
public:
    GuidedProgressiveVolumetricPathTracer(const Properties &props) : ProgressiveMonteCarloIntegrator(props)
    {

        ///////////////////////////////////////////////
        /// Guiding: setting up the parameters
        /// Note: usually only a small subset is needed
        ///////////////////////////////////////////////

        m_useNee                        = props.getBoolean("useNee", true);

        //guiding settings
        m_useSurfaceGuiding             = props.getBoolean("useSurfaceGuiding", true);
        m_useVolumeGuiding              = props.getBoolean("useVolumeGuiding", true);

        m_guideDirectLight              = props.getBoolean("guideDirectLight", true);
        m_useCosineProduct              = props.getBoolean("useCosineProduct", true);
        //m_useBSDFProduct                = props.getBoolean("useBSDFProduct", false);
        m_bsdfProbability               = props.getFloat("bsdfProbability", 0.5f);
        m_phaseFunctionProbability      = props.getFloat("phaseFunctionProbability", 0.5f);
#if defined(GUIDING_RR)
        m_guidedRR                      = props.getBoolean("guidedRR", false);
#endif
        m_rrCorrection                  = props.getBoolean("useRRCorrection", true);

#if defined(GUIDING_RR)
        m_guidedScatter                      = props.getBoolean("guidedScatter", false);
#endif

        m_surfaceAdjointType            = Guiding::getSurfaceAdjointType(props.getString("surfaceAdjoint","lo"));
        m_volumeAdjointType             = Guiding::getVolumeAdjointType(props.getString("volumeAdjoint","lo"));

        m_guidingType                   = (GuidedBSDFType::GuidingTypes) props.getInteger("guidingType", (int)GuidedBSDFType::EGUIDING_TYPE_PRODUCT);
        std::cout << "m_guidingType = " << m_guidingType << std::endl;

        //training/sample generation
        m_minSamplesToStartFitting      = props.getSize("minSamplesToStartFitting", 128);
        m_deterministic                 = props.getBoolean("deterministic", true);
        m_envmapDistance                = std::numeric_limits<float>::infinity();

        // general field configuration
        m_guidingFieldProps.setString("directionalDistribution", props.getString("directionalDistribution", "PAVMM"));
        m_guidingFieldProps.setBoolean("deterministic", m_deterministic);
        //m_guidingFieldProps.setBoolean("stochasticNN", props.getBoolean("stochasticNN", false));
        //m_guidingFieldProps.setBoolean("isNNLookup", props.getBoolean("isNNLookup", false));

        //kd-tree configuration
        m_guidingFieldProps.setSize("maxSamplesPerLeafNode", props.getSize("maxSamplesPerLeafNode", 32000));
        

        //training spp/time
        m_overallTrainingSamples            = props.getSize("trainingSamples", 32);
        m_trainingSamplesPerIteration       = props.getSize("maxSamplesPerIteration", 1);
        m_samplesPerProgression = m_trainingSamplesPerIteration;
#if defined(GUIDING_RR) || defined(SCATTER_GUIDING)
		m_savePixelEstimate = props.getBoolean("savePixelEstimate", false);
		m_loadPixelEstimate = props.getBoolean("loadPixelEstimate", false);
		m_pixelEstimateFile = props.getString("pixelEstimateFile", "pixEst.exr");
#endif
		m_saveGuidingCaches = props.getBoolean("saveGuidingCaches", false);
		m_loadGuidingCaches = props.getBoolean("loadGuidingCaches", false);
		m_guidingCachesFile = props.getString("guidingCachesFile", "guidingCaches.field");  


        m_distanceSamplerType = (Guiding::DistanceSamplerTypes)props.getInteger("distanceSamplerType", (int)m_distanceSamplerType);

        switch(m_distanceSamplerType){
            case Guiding::EStandardDS:{
                    Log(EInfo, "DistanceSampler = StandardDistanceSampler");
                    m_distanceSampler = new Guiding::StandardDistanceSampler();
                    break;
            }
            case Guiding::EIncrementalGuidedProductDS:{
                    Log(EInfo, "DistanceSampler = IncrementalGuidedProductDistanceSampler");
                    m_distanceSampler = new Guiding::IncrementalGuidedProductDistanceSampler(props);
                    break;
            }
        }

        m_training = false;
        m_canceled = false;

        if(m_loadGuidingCaches)
		{
			fs::path guidingCachesFile = Thread::getThread()->getFileResolver()->resolve(m_guidingCachesFile);

            if (!fs::exists(guidingCachesFile))
                Log(EError, "Guiding caches file \"%s\" could not be found!", guidingCachesFile.string().c_str());

			m_guidingCachesFile = guidingCachesFile.string();
		}	

        m_accountForDirectLightMiWeight = m_useNee;
#if defined(GUIDING_RR) || defined(SCATTER_GUIDING)
        if(m_guidedRR || m_guidedScatter)
        {
            m_rrDepth = 1;
            m_calulatePixelEstimate = true;
        }

        if (m_savePixelEstimate){
            m_calulatePixelEstimate = true;
        }

        if(m_loadPixelEstimate)
		{
			fs::path pixelEstimateFile = Thread::getThread()->getFileResolver()->resolve(m_pixelEstimateFile);

            if (!fs::exists(pixelEstimateFile))
                Log(EError, "Pixel estimate file \"%s\" could not be found!", pixelEstimateFile.string().c_str());
			m_pixelEstimateFile = pixelEstimateFile.string();
            m_calulatePixelEstimate = false;
		}
#endif
    }

    /// Unserialize from a binary data stream
    GuidedProgressiveVolumetricPathTracer(Stream *stream, InstanceManager *manager)
     : ProgressiveMonteCarloIntegrator(stream, manager) {}


    bool preprocess(const Scene *scene, RenderQueue *queue, const RenderJob *job, int sceneResID, int sensorResID, int samplerResID) {
        bool success = ProgressiveMonteCarloIntegrator::preprocess(scene, queue, job, sceneResID, sensorResID, samplerResID);
        Log(EInfo, this->toString().c_str());
        if(success) {
            m_canceled = false;
            m_training = true;

            ///////////////////////////////////////////////
            /// Guiding
            ///////////////////////////////////////////////

            //if (m_useBSDFProduct)
            //    BSDFOracle::initBSDFRepresentations(scene);

            // determine scene dependent information/parameters
            m_sceneSpace = scene->getKDTree()->getAABB();
            m_envmapDistance = scene->getAABB().getBSphere().radius*1024.0f;

            // set up per thread storage of sample data
            ref<Scheduler> sched = Scheduler::getInstance();
            const size_t nCores = sched->getCoreCount();
            m_maxThreadId.store(0);

            // init the storage for the per path segment data
            // -- one instance per thread
            m_perThreadPathSegmentDataStorage.resize(nCores);
            for(int i=0; i<nCores; i++)
            {
#ifdef OPENPGL_EF_RADIANCE_CACHES
                m_perThreadPathSegmentDataStorage[i] = new openpgl::cpp::PathSegmentStorage(true);
#else
                m_perThreadPathSegmentDataStorage[i] = new openpgl::cpp::PathSegmentStorage();
#endif
                m_perThreadPathSegmentDataStorage[i]->Reserve( 4 * (m_maxDepth+2));
            }

            // set up the storage for the training samples
            m_sampleStorage.Reserve(sched->getCoreCount()*1024*1024, sched->getCoreCount()*1024*1024);
            m_sampleStorage.Clear();
            m_guidingDevice = std::unique_ptr<openpgl::cpp::Device>(new openpgl::cpp::Device(PGL_DEVICE_TYPE_CPU_4));
            // Parse the settings for the guiding field and create an empty field
            Guiding::parseFieldProperties(m_guidingFieldProps, m_guidingFieldConfig);
            if(m_loadGuidingCaches && fs::exists(m_guidingCachesFile))
            {
                m_guidingField = std::unique_ptr<openpgl::cpp::Field>(new openpgl::cpp::Field(&*m_guidingDevice, m_guidingCachesFile));  
                m_iteration = m_guidingField->GetIteration();
                m_training = false;
            } else {    
                m_guidingField = std::unique_ptr<openpgl::cpp::Field>(new openpgl::cpp::Field(&*m_guidingDevice, m_guidingFieldConfig));
                pgl_box3f pglSceneBounds = openpgl::cpp::Box3(  m_sceneSpace.min[0], m_sceneSpace.min[1], m_sceneSpace.min[2],
                                                                m_sceneSpace.max[0], m_sceneSpace.max[1], m_sceneSpace.max[2]);
                m_guidingField->SetSceneBounds(pglSceneBounds);
                m_iteration = 0;
            }

            m_distanceSampler->setGuidingField(m_guidingField.get());

            m_perThreadGuidedBSDF.resize(nCores);
            m_perThreadGuidedPhaseFunction.resize(nCores);
            for(int i=0; i<nCores; i++)
            {
                m_perThreadGuidedBSDF[i] = new GuidedBSDFType(&*m_guidingField, m_useSurfaceGuiding, m_bsdfProbability, m_useCosineProduct, false);
                m_perThreadGuidedPhaseFunction[i] = new GuidedPhaseFunctionType(&*m_guidingField, m_useVolumeGuiding, m_phaseFunctionProbability);
            }

            m_fieldUpdateTimer = new Timer();
            m_fieldUpdateTimings.clear();
            m_sampleCounts.clear();
#if defined(GUIDING_RR) || defined(SCATTER_GUIDING)
            m_denoiseTimer = new Timer();
            if(m_loadPixelEstimate && fs::exists(m_pixelEstimateFile))
            {
                m_pixelEstimateDenoiser.loadBuffers(m_pixelEstimateFile);
                m_pixelEstimateReady = true;
            } else {
                ref<Sensor> sensor = static_cast<Sensor *>(sched->getResource(sensorResID));
                ref<Film> film = sensor->getFilm();
                Vector2i filmSize = film->getSize();
                m_pixelEstimateDenoiser.init(filmSize);
                m_pixelEstimateReady = false;
            }
#endif
        }
        return success;
    }

    void postprocess(const Scene *scene, RenderQueue *queue,
        const RenderJob *job, int sceneResID, int sensorResID,
        int samplerResID){

        ProgressiveMonteCarloIntegrator::postprocess(scene, queue,job,sceneResID, sensorResID, samplerResID);
        if(m_saveGuidingCaches)
        {
            m_guidingField->Store(m_guidingCachesFile);
        }
#if defined(GUIDING_RR) || defined(SCATTER_GUIDING)
		if (m_savePixelEstimate)
		//if(true)
        {
			Log( EInfo, "PixelEstimate::store = %s", m_pixelEstimateFile.c_str());
            m_pixelEstimateDenoiser.storeBuffers(m_pixelEstimateFile.c_str());
		}
#endif
        SLog(EInfo, "AvgX. path length: %f (%d/%d)",
            (float)avgPathLength.getValue()/(float)avgPathLength.getBase(),
            avgPathLength.getValue(),
            avgPathLength.getBase());

        std::cout << "fieldUpdateTimings.size: " << m_fieldUpdateTimings.size() << std::endl;

        if (m_fieldUpdateTimings.size() > 0)
        {
            float sumTimings = 0.f;
            float sumSamples = 0.f;
            std::stringstream st;
            std::stringstream ss;
            for ( int i = 0; i < m_fieldUpdateTimings.size(); i++)
            {
                if (i > 0) {
                    st << " , ";
                    ss << " , ";
                }
                st << m_fieldUpdateTimings[i];
                sumTimings += m_fieldUpdateTimings[i];
                ss << m_sampleCounts[i];
                sumSamples += m_sampleCounts[i];
            }

            SLog(EInfo, "Field training times (s): %s",
            st.str().c_str());
            SLog(EInfo, "Total training time (s): %f",
            sumTimings);

            SLog(EInfo, "Field training samples: %s",
            ss.str().c_str());
            SLog(EInfo, "Total number of training samples: %f",
            sumSamples);
        }
    }

	void preprogression(RenderQueue *queue, const RenderJob *job,
			int sceneResID, int sensorResID, int samplerResID) {
        ProgressiveMonteCarloIntegrator::preprogression(queue, job, sceneResID, sensorResID, samplerResID);
	}

	void postprogression(RenderQueue *queue, const RenderJob *job,
			int sceneResID, int sensorResID, int samplerResID) {
        ProgressiveMonteCarloIntegrator::postprogression(queue, job, sceneResID, sensorResID, samplerResID);

        SLog(EInfo, "Avg. path length: %f (%d/%d)",
            (float)avgPerIterationPathLength.getValue()/(float)avgPerIterationPathLength.getBase(),
            avgPerIterationPathLength.getValue(),
            avgPerIterationPathLength.getBase());
        avgPerIterationPathLength.reset();

        ref<Scheduler> sched = Scheduler::getInstance();
        ref<Sensor> sensor = static_cast<Sensor *>(sched->getResource(sensorResID));
        ref<Film> film = sensor->getFilm();

        ///////////////////////////////////////////////
        /// Guiding: Fitting/Updating
        ///////////////////////////////////////////////

        if (m_training){

            SLog(EInfo, "training: %d surface samples: %d \t volume samples: %d \t minSamplesToStartFitting: %d.",
                (int)m_training,
                (int)m_sampleStorage.GetSizeSurface(),
                (int)m_sampleStorage.GetSizeVolume(), m_minSamplesToStartFitting);

#if defined(MTS_OPENMP)
            ref<Scheduler> scheduler = Scheduler::getInstance();
            size_t nCores = scheduler->getCoreCount();
            mitsuba::Thread::initializeOpenMP(nCores);
#endif

            // fit/update guiding cache
            const size_t numValidSamples = m_sampleStorage.GetSizeSurface() + m_sampleStorage.GetSizeVolume();
            if (EXPECT_TAKEN(numValidSamples >= m_minSamplesToStartFitting))
            {
                pgl_box3f bounds;
                pglBox3f(bounds, m_sceneSpace.min.x, m_sceneSpace.min.y, m_sceneSpace.min.z, m_sceneSpace.max.x, m_sceneSpace.max.y, m_sceneSpace.max.z);

                m_guidingField->SetSceneBounds(bounds);
                m_fieldUpdateTimer->reset();
                m_guidingField->Update(m_sampleStorage);
                m_fieldUpdateTimer->stop();
                m_sampleCounts.push_back(numValidSamples);
                m_fieldUpdateTimings.push_back(m_fieldUpdateTimer->getSeconds());
                m_iteration++;
                std::cout << "trainingSmplesPerIteration: " << m_trainingSamplesPerIteration << "\t Iteration: " << m_guidingField->GetIteration() << std::endl;

                if(m_guidingField->GetIteration()*m_trainingSamplesPerIteration >= m_overallTrainingSamples)
                {
                    m_training = false;
                    std::cout << "STOP TRAINING" << std::endl;
/*
                    if (m_resetFBAfterTraining)
                    {
                        film->clear();
                    }
*/
                }
                // clear the sample storage after training is done
                m_sampleStorage.Clear();
            }

#if defined(GUIDING_RR) || defined(SCATTER_GUIDING)
            if (m_calulatePixelEstimate && m_progressionCounter == std::pow(2.0f, m_pixelEstimateWave))
            {
                m_denoiseTimer->reset();
                m_pixelEstimateDenoiser.denoise();
                Log( EInfo, "Filter[%d]: took %fs", m_progressionCounter, m_denoiseTimer->getMilliseconds() * 1e-3f );
                m_pixelEstimateReady = true;
                m_pixelEstimateWave++;
            }
#endif
        }
    }

    Spectrum Li(const RayDifferential &r, RadianceQueryRecord &rRec) const {

	    bool useFWD = true;
#ifdef GUIDING_RR
        Spectrum pixelEstimate = m_pixelEstimateReady ? m_pixelEstimateDenoiser.get(rRec.pixelId) : Spectrum(0.f);
        Denoiser::Sample denoiseSample;
#endif
        // guiding specific thread local instances
	    static thread_local openpgl::cpp::PathSegmentStorage* pathSegmentDataStorage;
        static thread_local GuidedBSDFType* guidedBSDF;
        static thread_local GuidedPhaseFunctionType* guidedPhaseFunction;

        std::vector<Spectrum> Lis;
        std::vector<Spectrum> throughputs;
        Lis.reserve(m_maxDepth*5);
        throughputs.reserve(m_maxDepth*5);
        // initialize thread local instances on first use
        if (EXPECT_NOT_TAKEN(!m_threadLocalInitialized.get()))
        {
            const size_t threadId = m_maxThreadId.fetch_add(1, std::memory_order_relaxed);
            pathSegmentDataStorage = m_perThreadPathSegmentDataStorage.at(threadId);
            guidedBSDF = m_perThreadGuidedBSDF.at(threadId);
            guidedPhaseFunction = m_perThreadGuidedPhaseFunction.at(threadId);
            m_threadLocalInitialized.get() = true;
            m_threadLocalId.get() = threadId;
        }

        int wID = m_threadLocalId.get();

        pathSegmentDataStorage->Clear();

        // storage for the per-path segment data
        openpgl::cpp::PathSegment* nextPathSegmentData = nullptr;
        openpgl::cpp::PathSegment* currentPathSegmentData = nullptr;

        /* Some aliases and local variables */
        const Scene *scene = rRec.scene;
        Intersection &its = rRec.its;
        MediumSamplingRecord mRec;
        RayDifferential ray(r);
        Spectrum Li(0.0f);
        bool scattered = false;

        bool nextHitEmitter = false;
        Spectrum nextHitEmitterEmission = Spectrum(0.f);
        Spectrum nextHitEmitterTransmittance = Spectrum(1.f);
        Spectrum nextHitEmitterScatteredEmission = Spectrum(0.f);
        Float nextHitEmitterMIS = 0.f;

        /* Perform the first ray intersection (or ignore if the
           intersection has already been provided). */
        rRec.rayIntersect(ray);
        ray.mint = Epsilon;

        Spectrum throughput(1.0f);
        //std::cout << "throughputs.pus_back: init" << throughput.toString() << std::endl; 
        throughputs.push_back(throughput);
        Float eta = 1.0f;

        int numDiffuseGlossyInteractions = 0;

        // sample data generated when splitting the ray inside a volume
        // to query the emission form the surface behind the volume using rayIntersectAndLookForEmitter

        pgl_vec3f pglZero = openpgl::cpp::Vector3(0.0f, 0.0f, 0.0f);
        pgl_vec3f pglOne = openpgl::cpp::Vector3(1.0f, 1.0f, 1.0f);

        float rrCorrectionFactor = 1.0f;
#ifdef GUIDING_RR
        Float q = 1.0f;
        bool guideRR = false;
#endif
        Spectrum cameraTransmittance(1.0f);
        Spectrum trackedTransmittanceWeight(1.0f);
        bool maxEventsReached = false;
        //std::cout << "newPath"  << std::endl;
        while (rRec.depth <= m_maxDepth || m_maxDepth < 0) {
#ifdef GUIDING_RR
            q = 1.0f;
            guideRR = false;
#endif
            Spectrum transmittanceWeight(1.0f);

            /* ==================================================================== */
            /*                 Radiative Transfer Equation sampling                 */
            /* ==================================================================== */
            //if (rRec.medium && rRec.medium->sampleDistance(Ray(ray, 0, its.t), mRec, rRec.sampler)) {
            if (rRec.medium && m_distanceSampler->sampleDistance(rRec.medium, Ray(ray, 0, its.t), mRec, rRec.sampler, wID)) {

                // check if the is contribution from behind the volume
                // and generate a radiance sample directly
                
                if(nextHitEmitter && currentPathSegmentData)
                {
                    pgl_vec3f pglContr = openpgl::cpp::Vector3(nextHitEmitterScatteredEmission[0], nextHitEmitterScatteredEmission[1], nextHitEmitterScatteredEmission[2]);
                    openpgl::cpp::AddScatteredContribution(currentPathSegmentData, pglContr);
                    nextHitEmitter = false;
                }
                
                if(!m_sceneSpace.contains(mRec.p))
                {
                    break;
                }

                transmittanceWeight = mRec.transmittance / mRec.pdfSuccess;
#ifdef GUIDING_SIGMA_S
                trackedTransmittanceWeight *= transmittanceWeight;
#else
                trackedTransmittanceWeight *= transmittanceWeight * mRec.sigmaS;
#endif
                if(m_training && currentPathSegmentData)
                {
                    pgl_vec3f pglTransmittanceWeight = openpgl::cpp::Vector3(trackedTransmittanceWeight[0], trackedTransmittanceWeight[1], trackedTransmittanceWeight[2]);
                    //pgl_vec3f pglTransmittanceWeight = openpgl::cpp::Vector3(transmittanceWeight[0], transmittanceWeight[1], transmittanceWeight[2]);
                    openpgl::cpp::SetTransmittanceWeight(currentPathSegmentData, pglTransmittanceWeight);
                }

                if (rRec.depth >= m_maxDepth && m_maxDepth != -1) // No more scattering events allowed
                {
                    maxEventsReached = true;
                    break;
                }
                rRec.depth++;

                if(m_training)
                {   
                    // create a new -volume- path segment
                    pgl_point3f pglP = openpgl::cpp::Point3(mRec.p[0], mRec.p[1], mRec.p[2]);
                    pgl_vec3f pglWi = openpgl::cpp::Vector3(-ray.d[0],-ray.d[1],-ray.d[2]);
                    pgl_vec3f pglNormal = openpgl::cpp::Vector3(0.0f, 0.0f, 1.0f);
                    currentPathSegmentData = pathSegmentDataStorage->NextSegment();
                    trackedTransmittanceWeight = Spectrum(1.0f);
                    openpgl::cpp::SetPosition(currentPathSegmentData, pglP);
                    openpgl::cpp::SetNormal(currentPathSegmentData, pglNormal);
                    openpgl::cpp::SetDirectionOut(currentPathSegmentData, pglWi);
                    openpgl::cpp::SetVolumeScatter(currentPathSegmentData, true);
                    openpgl::cpp::SetScatteredContribution(currentPathSegmentData, pglZero);
                    openpgl::cpp::SetDirectContribution(currentPathSegmentData, pglZero);
                    openpgl::cpp::SetTransmittanceWeight(currentPathSegmentData, pglOne);
                    openpgl::cpp::SetEta(currentPathSegmentData, 1.0);
                }
                /* Sample the integral
                   \int_x^y tau(x, x') [ \sigma_s \int_{S^2} \rho(\omega,\omega') L(x,\omega') d\omega' ] dx'
                */

                const PhaseFunction *volumePhase = mRec.getPhaseFunction();
                const float meanCosine = volumePhase->getMeanCosine();
                guidedPhaseFunction->prepare(&*m_guidingField, mRec.p, -ray.d, volumePhase, rRec.sampler);
                //throughputs.push_back(throughput* transmittanceWeight);
                //std::cout << "throughput = " << throughput.toString() << "\t transmittanceWeight = " << transmittanceWeight.toString()<< std::endl;

//                std::cout << "throughputT = " << (throughput* transmittanceWeight).toString() << std::endl;
#ifdef GUIDING_SIGMA_S
                Spectrum rrThroughput = throughput * transmittanceWeight;
#else
                Spectrum rrThroughput = throughput * transmittanceWeight * mRec.sigmaS;
#endif
                throughput *= transmittanceWeight * mRec.sigmaS;
                //std::cout << "throughputs.pus_back: vol trans*sigmaS " << throughput.toString() << "\t transmittanceWeight = " << (transmittanceWeight * mRec.sigmaS).toString()<< std::endl; 
                throughputs.push_back(throughput);
                if(!scattered)
                {
                    cameraTransmittance = throughput;//*transmittanceWeight;
                }
                //throughputs.push_back(throughput);
//                std::cout << "throughputST = " << throughput.toString() << std::endl;
                
                Spectrum inscatteredRadiance(1.0f);
#ifdef OPENPGL_EF_RADIANCE_CACHES
                if(m_volumeAdjointType == Guiding::EVALo)
                    inscatteredRadiance = guidedPhaseFunction->outgoingRadiance(-ray.d);
                else if(m_volumeAdjointType == Guiding::EVAInscattered)
                    inscatteredRadiance = guidedPhaseFunction->inscatteredRadiance(-ray.d, meanCosine);
                else if(m_volumeAdjointType == Guiding::EVAFluence)
                    inscatteredRadiance = (1.0f/(4.0f*M_PI)) * guidedPhaseFunction->fluence();
                else
                    inscatteredRadiance = guidedPhaseFunction->outgoingRadiance(-ray.d);
#endif  
#ifdef GUIDING_RR
                if(m_guidedRR && m_iteration >= 16)
                {
                    q = GuidedRussianRouletteAndSplittingProbabilities::guidedRussianRouletteProbability(rrThroughput, inscatteredRadiance, pixelEstimate);
                    guideRR = true;
                }
#endif
                /* Adding denoise features */
                if (numDiffuseGlossyInteractions == 0) {
                    denoiseSample.albedo = throughput/**mRec.sigmaS*//(mRec.sigmaS+mRec.sigmaA);
                    denoiseSample.normal = -ray.d;
                }
                numDiffuseGlossyInteractions++;

                /* ==================================================================== */
                /*                          Luminaire sampling                          */
                /* ==================================================================== */

                /* Estimate the single scattering component if this is requested */
                DirectSamplingRecord dRec(mRec.p, mRec.time);
                
                if (m_useNee &&  (rRec.type & RadianceQueryRecord::EDirectMediumRadiance)) {
                    int interactions = m_maxDepth - rRec.depth - 1;
                    //std::cout << "PING: interactions = " << interactions << std::endl;
                    Spectrum value = scene->sampleAttenuatedEmitterDirect(
                            dRec, rRec.medium, interactions,
                            rRec.nextSample2D(), rRec.sampler);

                    if (!value.isZero()) {
                        const Emitter *emitter = static_cast<const Emitter *>(dRec.object);

                        /* Evaluate the phase function */
                        PhaseFunctionSamplingRecord pRec(mRec, -ray.d, dRec.d);
                        Float phaseVal = guidedPhaseFunction->eval(pRec);

                        if (phaseVal != 0) {
                            /* Calculate prob. of having sampled that direction using
                               phase function sampling */
                            Float phasePdf = (emitter->isOnSurface() && dRec.measure == ESolidAngle)
                                    ? guidedPhaseFunction->pdf(pRec) : (Float) 0.0f;

                            /* Weight using the power heuristic */
#ifdef GUIDING_RR
                            Float weight = useFWD ? miWeight(dRec.pdf, q*phasePdf) : 1.0f;
#else
                            Float weight = useFWD ? miWeight(dRec.pdf, phasePdf) : 1.0f;
#endif
                            Li += throughput * value * phaseVal * weight;
                            if(m_training)
                            {
                                // add the NEE contribution to the scattered contribution of
                                // the current path segment
#ifdef GUIDING_SIGMA_S
                                const Spectrum contr = mRec.sigmaS * value * phaseVal * weight;
#else
                                const Spectrum contr = value * phaseVal * weight;
#endif
                                pgl_vec3f pglContr = openpgl::cpp::Vector3(contr[0], contr[1], contr[2]);
                                openpgl::cpp::AddScatteredContribution(currentPathSegmentData, pglContr);
                            }
                        }
                    }
                }

                /* ==================================================================== */
                /*                         Phase function sampling                      */
                /* ==================================================================== */

                Float phasePdf;
                PhaseFunctionSamplingRecord pRec(mRec, -ray.d);
                Float rrCorrect = 1.0f;
                Float phaseWeight = guidedPhaseFunction->sample(pRec, phasePdf, rrCorrect, rRec.sampler);
                scattered = true;
                if(m_rrCorrection)
                    rrCorrectionFactor *= rrCorrect;
                if (phaseWeight == 0)
                    break;

                // transfer the information of the current scattering position inside the volume
                // to the current path segment
                if(m_training)
                {
#ifdef GUIDING_SIGMA_S
                    const Spectrum sigmaSphaseWeight = mRec.sigmaS * Spectrum(phaseWeight);
#else
                    const Spectrum sigmaSphaseWeight = Spectrum(phaseWeight);
#endif
                    pgl_vec3f pglWo2 = openpgl::cpp::Vector3(pRec.wo[0], pRec.wo[1], pRec.wo[2]);
                    pgl_vec3f pglPhaseWeight = openpgl::cpp::Vector3(sigmaSphaseWeight[0], sigmaSphaseWeight[1], sigmaSphaseWeight[2]);
                    openpgl::cpp::SetDirectionIn(currentPathSegmentData, pglWo2);
                    openpgl::cpp::SetPDFDirectionIn(currentPathSegmentData, phasePdf);
                    openpgl::cpp::SetScatteringWeight(currentPathSegmentData, pglPhaseWeight);
                    openpgl::cpp::SetIsDelta(currentPathSegmentData, false);
                    openpgl::cpp::SetEta(currentPathSegmentData, 1.0f);
                    openpgl::cpp::SetRoughness(currentPathSegmentData, 1.0f - guidedPhaseFunction->getMeanCosine());
                }
                throughput *= phaseWeight;
                //std::cout << "throughputs.pus_back: vol phase" << throughput.toString() << std::endl; 
                throughputs.push_back(throughput);
                /* Trace a ray in this direction */
                ray = Ray(mRec.p, pRec.wo, ray.time);
                ray.mint = 0;

                Spectrum value(0.0f);
                Point lightPos;
                Spectrum transmittance(1.0f);
                bool passedMedium = false;
                nextHitEmitter = false;
                nextHitEmitterEmission = Spectrum(0.f);
                nextHitEmitterScatteredEmission = Spectrum(0.f);
                nextHitEmitterMIS = 0.f;
                rayIntersectAndLookForEmitter(scene, rRec.sampler, rRec.medium, passedMedium,
                    m_maxDepth - rRec.depth - 1, ray, its, dRec, value, lightPos, transmittance);

                /* If a luminaire was hit, estimate the local illumination and
                   weight using the power heuristic */
                if (useFWD && !value.isZero() && value.min() > 0.f && (rRec.type & RadianceQueryRecord::EDirectMediumRadiance)) {
                    nextHitEmitter = true;
                    const Float emitterPdf = (m_useNee) ? scene->pdfEmitterDirect(dRec): 0.0f;
#ifdef GUIDING_RR
                    const Float weight = (m_useNee) ? miWeight(q*phasePdf, emitterPdf) : 1.0f;
#else
                    const Float weight = (m_useNee) ? miWeight(phasePdf, emitterPdf) : 1.0f;
#endif
                    Li += throughput * value * weight;
                    // if we found an emitter at the end of the ray we
                    // need to generate a radiance sample for it

                    if(m_training)
                    {
                        nextHitEmitterScatteredEmission = value * phaseWeight* weight;
                        nextHitEmitterEmission = value / transmittance;
                        nextHitEmitterMIS = weight;
                        nextHitEmitterTransmittance = transmittance;
                        //std::cout << "transmittance = " << transmittance.toString() << "\t value = " << value.toString() << "\t emission = " << nextHitEmitterEmission.toString()<< std::endl;
                    }
                }

                /* ==================================================================== */
                /*                         Multiple scattering                          */
                /* ==================================================================== */

                /* Stop if multiple scattering was not requested */
                if (!(rRec.type & RadianceQueryRecord::EIndirectMediumRadiance))
                    break;
                rRec.type = RadianceQueryRecord::ERadianceNoEmission;
            } else {
                /* Sample
                    tau(x, y) (Surface integral). This happens with probability mRec.pdfFailure
                    Account for this and multiply by the proper per-color-channel transmittance.
                */
                transmittanceWeight = Spectrum(1.0f);
                if (rRec.medium){
                    transmittanceWeight = mRec.transmittance / mRec.pdfFailure;
                    throughput *= transmittanceWeight;
                    //std::cout << "throughputs.pus_back: no vol transmittance" << throughput.toString() << "\t transmittanceWeight = " << transmittanceWeight.toString()<< std::endl; 
                    throughputs.push_back(throughput);
                }
                trackedTransmittanceWeight *= transmittanceWeight;
                if(m_training && currentPathSegmentData){
                    pgl_vec3f pglTransmittanceWeight = openpgl::cpp::Vector3(trackedTransmittanceWeight[0], trackedTransmittanceWeight[1], trackedTransmittanceWeight[2]);
                    openpgl::cpp::SetTransmittanceWeight(currentPathSegmentData, pglTransmittanceWeight);
                }

                if(!scattered)
                {
                    cameraTransmittance = throughput;
                }

                if (!its.isValid()) {
                    /* If no intersection could be found, possibly return
                       attenuated radiance from a background luminaire */
                    if ((rRec.type & RadianceQueryRecord::EEmittedRadiance)
                        && (!m_hideEmitters || scattered)) {
                        Spectrum value = throughput * scene->evalEnvironment(ray);
                        if (rRec.medium) {
                            //value *= rRec.medium->evalTransmittance(ray, rRec.sampler);
                            value *= m_distanceSampler->evalTransmittance(rRec.medium, ray, rRec.sampler);
                        }
                        Li += value;

                        /* Adding denoise features */
                        if (numDiffuseGlossyInteractions == 0)
                            denoiseSample.albedo = value;
                    }
                    if (nextHitEmitter) { // Add Envmap if we hit an emitter
                        //std::cout << "PING: depth = " << rRec.depth<< std::endl;
                        //std::cout << "nextHitEmitter 1"<< std::endl;
                        nextHitEmitter = false;
                        Point3 envPos = ray.o + (m_envmapDistance * ray.d);
                        pgl_point3f pglEnvPos = openpgl::cpp::Vector3(envPos[0], envPos[1], envPos[2]);
                        pgl_vec3f pglNormal = openpgl::cpp::Vector3(0.0f, 0.0f, 1.0f);
                        pgl_vec3f pglDirectionOut = openpgl::cpp::Vector3(-ray.d[0],-ray.d[1],-ray.d[2]);
                        pgl_vec3f pglTransmittance = openpgl::cpp::Vector3(nextHitEmitterTransmittance[0], nextHitEmitterTransmittance[1], nextHitEmitterTransmittance[2]);
                        if(currentPathSegmentData)
                            openpgl::cpp::SetTransmittanceWeight(currentPathSegmentData, pglTransmittance);

                        nextPathSegmentData = pathSegmentDataStorage->NextSegment();
                        trackedTransmittanceWeight = Spectrum(1.0f);
                        openpgl::cpp::SetPosition(nextPathSegmentData, pglEnvPos);
                        openpgl::cpp::SetNormal(nextPathSegmentData, pglNormal);
                        openpgl::cpp::SetDirectionOut(nextPathSegmentData, pglDirectionOut);
                        openpgl::cpp::SetDirectContribution(nextPathSegmentData, openpgl::cpp::Vector3(nextHitEmitterEmission[0],nextHitEmitterEmission[1],nextHitEmitterEmission[2]));
                        openpgl::cpp::SetMiWeight(nextPathSegmentData, nextHitEmitterMIS);
                        openpgl::cpp::SetRussianRouletteProbability(nextPathSegmentData,1.0f);
                        //openpgl::cpp::SetTransmittanceWeight(nextPathSegmentData, pglTransmittance);
                        //pathSegmentDataStorage->Print();
                    }
                    break;
                }
                // if we collect samples we want to add a new path segment
                const BSDF *surfaceBSDF = its.getBSDF(ray);

                const int bsdfType = surfaceBSDF->getType();
                bool isNullBSDF = bsdfType == (BSDF::ENull | BSDF::EFrontSide | BSDF::EBackSide);
/*
                if (isNullBSDF){
                    if (!(rRec.type & RadianceQueryRecord::EIndirectSurfaceRadiance))
                        break;
                    rRec.type = scattered ? RadianceQueryRecord::ERadianceNoEmission
                        : RadianceQueryRecord::ERadiance;
                    ray = Ray(its.p, ray.d, ray.time);
                    scene->rayIntersect(ray, its);
#ifdef MTS_IGNORE_NULLBSDF_INTERSECTIONS
                    //rRec.depth++;
#endif
                }
*/
                if(m_training && !isNullBSDF)
                {
                    const Vector3 normal = its.toWorld(Vector3(0.0,0.0,1.0));
                    const Vector3 wi = its.toWorld(its.wi);
                    pgl_point3f pglP = openpgl::cpp::Point3(its.p[0], its.p[1], its.p[2]);
                    pgl_vec3f pglNormal = openpgl::cpp::Vector3(normal[0], normal[1], normal[2]);
                    pgl_vec3f pglWi = openpgl::cpp::Vector3(wi[0], wi[1], wi[2]);
                    nextPathSegmentData = pathSegmentDataStorage->NextSegment();
                    trackedTransmittanceWeight = Spectrum(1.0f);
                    openpgl::cpp::SetPosition(nextPathSegmentData, pglP);
                    openpgl::cpp::SetNormal(nextPathSegmentData, pglNormal);
                    openpgl::cpp::SetDirectionOut(nextPathSegmentData, pglWi);
                    openpgl::cpp::SetVolumeScatter(nextPathSegmentData, false);
                    openpgl::cpp::SetScatteredContribution(nextPathSegmentData, pglZero);
                    openpgl::cpp::SetDirectContribution(nextPathSegmentData, pglZero);
                    openpgl::cpp::SetTransmittanceWeight(nextPathSegmentData, pglOne);
                    openpgl::cpp::SetEta(nextPathSegmentData, 1.0f);
                    openpgl::cpp::SetRussianRouletteProbability(nextPathSegmentData,1.0f);
                }

                if (m_training && !isNullBSDF)
                {
                    currentPathSegmentData = nextPathSegmentData;

                    if (nextHitEmitter)
                    {
                        pgl_vec3f pglNextHitEmitterEmission = openpgl::cpp::Vector3(nextHitEmitterEmission[0], nextHitEmitterEmission[1], nextHitEmitterEmission[2]);
                        openpgl::cpp::SetDirectContribution(currentPathSegmentData, pglNextHitEmitterEmission);
                        openpgl::cpp::SetMiWeight(currentPathSegmentData, nextHitEmitterMIS);
                        nextHitEmitter = false;
                    }
                }
                //const BSDF *surfaceBSDF = its.getBSDF(ray);

                /* Adding denoise features */
                if ((bsdfType & BSDF::EDiffuse || bsdfType & BSDF::EGlossy) && numDiffuseGlossyInteractions == 0) {
                    denoiseSample.albedo = throughput * surfaceBSDF->getAlbedo(rRec.its);
                    denoiseSample.normal = rRec.its.toWorld(Vector3(0, 0, 1));
                    numDiffuseGlossyInteractions++;
                }

                guidedBSDF->prepare(&*m_guidingField, its, -ray.d, surfaceBSDF, rRec.sampler);
                Spectrum rrThroughput = throughput;
                Spectrum outgoingRadiance(1.0f);
#ifdef OPENPGL_EF_RADIANCE_CACHES
                if(m_surfaceAdjointType == Guiding::ESALo)
                    outgoingRadiance = guidedBSDF->outgoingRadiance(-ray.d); 
                else if(m_surfaceAdjointType == Guiding::ESAIrradiance){
                    Vector3 normal = its.toWorld(Vector3(0.0,0.0,1.0));
                    if(dot(normal,its.toWorld(its.wi))< 0){
                        normal = -normal;
                    }
                    outgoingRadiance = guidedBSDF->getAlbedo(its) * guidedBSDF->irradiance(normal);         
                } else
                    outgoingRadiance = guidedBSDF->outgoingRadiance(-ray.d);
#endif
#ifdef GUIDING_RR
                if(m_guidedRR && m_iteration >= 16 && !(bsdfType & BSDF::EDelta))
                {
                    q = GuidedRussianRouletteAndSplittingProbabilities::guidedRussianRouletteProbability(rrThroughput, outgoingRadiance, pixelEstimate);
                    guideRR = true;
                } else if(bsdfType & BSDF::EDelta){
                    q = 0.95f;
                    guideRR = true;
                }
#endif

                /* Possibly include emitted radiance if requested */
                /* Note: The following is usally only true a light source is hit directly
                   by a primary ray. At the end of this loop rRec.type is set to ERadianceNoEmission*/
                if (its.isEmitter() && (rRec.type & RadianceQueryRecord::EEmittedRadiance)
                    && (!m_hideEmitters || scattered))
                {
                    const Spectrum emittedRadiance = its.Le(-ray.d);
                    Li += throughput * emittedRadiance;

                    if (m_training)
                    {
                        pgl_vec3f pglEmittedRadiance = openpgl::cpp::Vector3(emittedRadiance[0], emittedRadiance[1], emittedRadiance[2]);
                        openpgl::cpp::AddDirectContribution(currentPathSegmentData, pglEmittedRadiance);
                    }
                }
                /* Include radiance from a subsurface integrator if requested */
                if (its.hasSubsurface() && (rRec.type & RadianceQueryRecord::ESubsurfaceRadiance))
                {
                    const Spectrum subsurfaceScatteredRadiance = its.LoSub(rRec.scene, rRec.sampler, -ray.d, rRec.depth);
                    Li += throughput * subsurfaceScatteredRadiance;

                    if (m_training)
                    {
                        pgl_vec3f pglSubsurfaceScatteredRadiance = openpgl::cpp::Vector3(subsurfaceScatteredRadiance[0], subsurfaceScatteredRadiance[1], subsurfaceScatteredRadiance[2]);
                        openpgl::cpp::AddScatteredContribution(currentPathSegmentData, pglSubsurfaceScatteredRadiance);
                    }
                }
                if(!isNullBSDF){
                    if (rRec.depth >= m_maxDepth && m_maxDepth > 0)
                        break;

                    /* Prevent light leaks due to the use of shading normals */
                    if ((rRec.depth >= m_maxDepth && m_maxDepth > 0)
                        || (m_strictNormals && dot(ray.d, its.geoFrame.n)
                            * Frame::cosTheta(its.wi) >= 0)) {

                        /* Only continue if:
                        1. The current path length is below the specifed maximum
                        2. If 'strictNormals'=true, when the geometric and shading
                            normals classify the incident direction to the same side */
                        break;
                    }
                }

                /* ==================================================================== */
                /*                          Luminaire sampling                          */
                /* ==================================================================== */

                DirectSamplingRecord dRec(its);

                /* Estimate the direct illumination if this is requested */
                if (m_useNee &&(rRec.type & RadianceQueryRecord::EDirectSurfaceRadiance) &&
                    (guidedBSDF->getType() & BSDF::ESmooth)) {

                    int interactions = m_maxDepth - rRec.depth - 1;

                    Spectrum value = scene->sampleAttenuatedEmitterDirect(
                            dRec, its, rRec.medium, interactions,
                            rRec.nextSample2D(), rRec.sampler);

                    if (!value.isZero()) {
                        const Emitter *emitter = static_cast<const Emitter *>(dRec.object);

                        /* Allocate a record for querying the BSDF */
                        BSDFSamplingRecord bRec(its, its.toLocal(dRec.d));

                        /* Evaluate BSDF * cos(theta) */
                        const Spectrum bsdfVal = guidedBSDF->eval(bRec);

                        /* Prevent light leaks due to the use of shading normals */
                        if (!bsdfVal.isZero() && (!m_strictNormals ||
                            dot(its.geoFrame.n, dRec.d) * Frame::cosTheta(bRec.wo) > 0)) {

                            /* Calculate prob. of having generated that direction
                               using BSDF sampling */
                            Float bsdfPdf = (emitter->isOnSurface() && dRec.measure == ESolidAngle)
                                    ? guidedBSDF->pdf(bRec) : (Float) 0.0f;

                            /* Weight using the power heuristic */
#ifdef GUIDING_RR
                            const Float weight = useFWD ? miWeight(dRec.pdf, q*bsdfPdf) : 1.0f;
#else
                            const Float weight = useFWD ? miWeight(dRec.pdf, bsdfPdf) : 1.0f;
#endif
                            const Spectrum contr = value * bsdfVal * weight;
                            Li += throughput * contr;

                            if (m_training)
                            {
                                pgl_vec3f pglContr = openpgl::cpp::Vector3(contr[0],contr[1],contr[2]);
                                openpgl::cpp::AddScatteredContribution(currentPathSegmentData, pglContr);
                            }
                        }
                    }
                }

                /* ==================================================================== */
                /*                            BSDF sampling                             */
                /* ==================================================================== */
                //std::cout << "PONG: BSDF Sampling \t depth = " << rRec.depth<< std::endl;
                /* Sample BSDF * cos(theta) */
                Float surfaceScatterPdf;
                Float surfaceScatterMISPdf;
                Spectrum bsdfWeight;
                Float rrCorrect =1.0f;
                BSDFSamplingRecord bRec(its, rRec.sampler, ERadiance);
                if (m_guidingType == GuidedBSDFType::EGUIDING_TYPE_RIS)
                {
                    bsdfWeight = guidedBSDF->sampleRIS(bRec, surfaceScatterPdf, surfaceScatterMISPdf, rrCorrect, rRec.nextSample2D());
                } else {
                    bsdfWeight = guidedBSDF->sample(bRec, surfaceScatterPdf, rrCorrect, rRec.nextSample2D());
                    surfaceScatterMISPdf = surfaceScatterPdf;
                }
                if(m_rrCorrection)
                    rrCorrectionFactor *= rrCorrect;
                if (bsdfWeight.isZero())
                    break;

                //std::cout << "PONG: BSDF Sampling: Sampled \t depth = " << rRec.depth<< std::endl;
                scattered |= bRec.sampledType != BSDF::ENull;

                /* Prevent light leaks due to the use of shading normals */
                const Vector wo = its.toWorld(bRec.wo);
                Float woDotGeoN = dot(its.geoFrame.n, wo);

                if (m_training && !isNullBSDF)
                {
                    // setting up the current path segment using the BSDF sample data
                    pgl_vec3f pglWo = openpgl::cpp::Vector3(wo[0], wo[1], wo[2]);
                    pgl_vec3f pglBSDFWeight = openpgl::cpp::Vector3(bsdfWeight[0], bsdfWeight[1], bsdfWeight[2]);
                    openpgl::cpp::SetDirectionIn(currentPathSegmentData, pglWo);
                    openpgl::cpp::SetPDFDirectionIn(currentPathSegmentData, surfaceScatterPdf);
                    openpgl::cpp::SetScatteringWeight(currentPathSegmentData, pglBSDFWeight);
                    openpgl::cpp::SetIsDelta(currentPathSegmentData, bRec.sampledType & BSDF::EDelta);
                    openpgl::cpp::SetEta(currentPathSegmentData, bRec.eta);
                    float roughness = surfaceBSDF->getRoughness(its, bRec.sampledComponent);
                    roughness = roughness == std::numeric_limits<Float>::infinity() ? 1.0f : roughness;
                    openpgl::cpp::SetRoughness(currentPathSegmentData, roughness);
                }

                //std::cout << "PONG: BSDF Sampling: 2 \t depth = " << rRec.depth<< std::endl;
                if (m_strictNormals && woDotGeoN * Frame::cosTheta(bRec.wo) <= 0)
                    break;

                /* Trace a ray in this direction */
                ray = Ray(its.p, wo, ray.time);

                /* Keep track of the throughput, medium, and relative
                   refractive index along the path */
                throughput *= bsdfWeight;
                //std::cout << "throughputs.pus_back: Bsdf weight" << throughput.toString() << std::endl; 
                throughputs.push_back(throughput);
                eta *= bRec.eta;
                if (its.isMediumTransition())
                    rRec.medium = its.getTargetMedium(ray.d);

                /* Handle index-matched medium transitions specially */
                if (bRec.sampledType == BSDF::ENull) {
                    if (!(rRec.type & RadianceQueryRecord::EIndirectSurfaceRadiance))
                        break;
                    rRec.type = scattered ? RadianceQueryRecord::ERadianceNoEmission
                        : RadianceQueryRecord::ERadiance;
                    scene->rayIntersect(ray, its);
                    //std::cout << "PONG: BSDF Sampling: NULL \t depth = " << rRec.depth<< std::endl;
#ifdef MTS_IGNORE_NULLBSDF_INTERSECTIONS
                    //rRec.depth++;
#endif
                    continue;
                }else {
                    ++rRec.depth;
                }

                // querrying emission from the next surface intersection
                Spectrum value(0.0f);
                // final position on the emitter
                Point lightPos;
                // transmittance along the way to the emitter
                Spectrum transmittance(1.0f);
                bool passedMedium = false;
                nextHitEmitter = false;
                nextHitEmitterEmission = Spectrum(0.f);
                nextHitEmitterScatteredEmission = Spectrum(0.f);
                nextHitEmitterMIS = 0.f;
                //std::cout << "PONG: NULL \t depth = " << rRec.depth<< std::endl;
                if(!isNullBSDF)
                    rayIntersectAndLookForEmitter(scene, rRec.sampler, rRec.medium, passedMedium,
                        m_maxDepth - rRec.depth - 1, ray, its, dRec, value, lightPos, transmittance);
                else
                    rayIntersectAndLookForEmitter(scene, rRec.sampler, rRec.medium, passedMedium,
                        m_maxDepth - rRec.depth, ray, its, dRec, value, lightPos, transmittance);
                /* If a luminaire was hit, estimate the local illumination and
                   weight using the power heuristic */
                if (!value.isZero() && (rRec.type & RadianceQueryRecord::EDirectSurfaceRadiance)) {
                    nextHitEmitter = true;
                    //std::cout << "nextHitEmitter = true 2" << "\t value = "<< value.toString()<< std::endl;
                    const Float emitterPdf = (m_useNee && !(bRec.sampledType & BSDF::EDelta)) ?
                        scene->pdfEmitterDirect(dRec) : 0;
#ifdef GUIDING_RR
                    const Float weight = (m_useNee) ? miWeight(q*surfaceScatterMISPdf, emitterPdf): 1.0f;
#else
                    const Float weight = (m_useNee) ? miWeight(surfaceScatterMISPdf, emitterPdf): 1.0f;
#endif
                    Li += throughput * value * weight;
                    
                    if (m_training)
                    {
                        nextHitEmitterScatteredEmission = bsdfWeight * value * weight;
                        nextHitEmitterEmission = value / transmittance;
                        nextHitEmitterTransmittance = transmittance;
                        nextHitEmitterMIS = weight;
                    }
                }

                /* ==================================================================== */
                /*                         Indirect illumination                        */
                /* ==================================================================== */

                /* Stop if indirect illumination was not requested */
                if (!its.isValid() || !(rRec.type & RadianceQueryRecord::EIndirectSurfaceRadiance)) {
                    if (nextHitEmitter) { // Add Envmap if we hit an emitter
                        nextHitEmitter = false;
                        Point3 envPos = ray.o + (m_envmapDistance * ray.d);
                        pgl_point3f pglEnvPos = openpgl::cpp::Vector3(envPos[0], envPos[1], envPos[2]);
                        pgl_vec3f pglNormal = openpgl::cpp::Vector3(0.0f, 0.0f, 1.0f);
                        pgl_vec3f pglDirectionOut = openpgl::cpp::Vector3(-ray.d[0],-ray.d[1],-ray.d[2]);

                        pgl_vec3f pglTransmittance = openpgl::cpp::Vector3(nextHitEmitterTransmittance[0], nextHitEmitterTransmittance[1], nextHitEmitterTransmittance[2]);
                        if(currentPathSegmentData)
                            openpgl::cpp::SetTransmittanceWeight(currentPathSegmentData, pglTransmittance);
                        nextPathSegmentData = pathSegmentDataStorage->NextSegment();
                        trackedTransmittanceWeight = Spectrum(1.0f);
                        openpgl::cpp::SetPosition(nextPathSegmentData, pglEnvPos);
                        openpgl::cpp::SetNormal(nextPathSegmentData, pglNormal);
                        openpgl::cpp::SetDirectionOut(nextPathSegmentData, pglDirectionOut);
                        openpgl::cpp::SetDirectContribution(nextPathSegmentData, openpgl::cpp::Vector3(nextHitEmitterEmission[0],nextHitEmitterEmission[1],nextHitEmitterEmission[2]));
                        openpgl::cpp::SetMiWeight(nextPathSegmentData, nextHitEmitterMIS);
                        openpgl::cpp::SetRussianRouletteProbability(nextPathSegmentData,1.0f);
                    }
                    break;
                }

                rRec.type = RadianceQueryRecord::ERadianceNoEmission;
            }
#if 0
            if (m_rrDepth >= 0 && rRec.depth++ > m_rrDepth) {
#else
            if (m_rrDepth >= 0 && rRec.depth > m_rrDepth) {
#endif
                /* Russian roulette: try to keep path weights equal to one,
                   while accounting for the solid angle compression at refractive
                   index boundaries. Stop with at least some probability to avoid
                   getting stuck (e.g. due to total internal reflection) */
#ifdef GUIDING_RR
                if(!guideRR)
                {
                    q = GuidedRussianRouletteAndSplittingProbabilities::standardRussianRouletteProbability(throughput*rrCorrectionFactor, eta);
                }
#else
                Float q = std::min(throughput.max()*rrCorrectionFactor * eta * eta, (Float) 0.95f);                
#endif
                if (m_training)
                {
                    openpgl::cpp::SetRussianRouletteProbability(currentPathSegmentData, q);
                }
                if (rRec.nextSample1D() >= q){
                    if (nextHitEmitter) { // Add Envmap if we hit an emitter
                        //std::cout << "PING: depth = " << rRec.depth<< std::endl;
                        //std::cout << "nextHitEmitter 1"<< std::endl;
                        nextHitEmitter = false;
                        Point3 envPos = ray.o + (m_envmapDistance * ray.d);
                        pgl_point3f pglEnvPos = openpgl::cpp::Vector3(envPos[0], envPos[1], envPos[2]);
                        pgl_vec3f pglNormal = openpgl::cpp::Vector3(0.0f, 0.0f, 1.0f);
                        pgl_vec3f pglDirectionOut = openpgl::cpp::Vector3(-ray.d[0],-ray.d[1],-ray.d[2]);
                        pgl_vec3f pglTransmittance = openpgl::cpp::Vector3(nextHitEmitterTransmittance[0], nextHitEmitterTransmittance[1], nextHitEmitterTransmittance[2]);
                        if(currentPathSegmentData)
                            openpgl::cpp::SetTransmittanceWeight(currentPathSegmentData, pglTransmittance);

                        nextPathSegmentData = pathSegmentDataStorage->NextSegment();
                        trackedTransmittanceWeight = Spectrum(1.0f);
                        openpgl::cpp::SetPosition(nextPathSegmentData, pglEnvPos);
                        openpgl::cpp::SetNormal(nextPathSegmentData, pglNormal);
                        openpgl::cpp::SetDirectionOut(nextPathSegmentData, pglDirectionOut);
                        openpgl::cpp::SetDirectContribution(nextPathSegmentData, openpgl::cpp::Vector3(nextHitEmitterEmission[0],nextHitEmitterEmission[1],nextHitEmitterEmission[2]));
                        openpgl::cpp::SetMiWeight(nextPathSegmentData, nextHitEmitterMIS);
                        openpgl::cpp::SetRussianRouletteProbability(nextPathSegmentData,1.0f);
                        //openpgl::cpp::SetTransmittanceWeight(nextPathSegmentData, pglTransmittance);
                        //pathSegmentDataStorage->Print();
                    }
                    break;
                }
                throughput /= q;
                throughputs.push_back(throughput);
            }
            scattered = true;
        }
        if(maxEventsReached) {
            if (nextHitEmitter) { // Add Envmap if we hit an emitter
                //std::cout << "PING: depth = " << rRec.depth<< std::endl;
                //std::cout << "nextHitEmitter 1"<< std::endl;
                nextHitEmitter = false;
                Point3 envPos = ray.o + (m_envmapDistance * ray.d);
                pgl_point3f pglEnvPos = openpgl::cpp::Vector3(envPos[0], envPos[1], envPos[2]);
                pgl_vec3f pglNormal = openpgl::cpp::Vector3(0.0f, 0.0f, 1.0f);
                pgl_vec3f pglDirectionOut = openpgl::cpp::Vector3(-ray.d[0],-ray.d[1],-ray.d[2]);
                pgl_vec3f pglTransmittance = openpgl::cpp::Vector3(nextHitEmitterTransmittance[0], nextHitEmitterTransmittance[1], nextHitEmitterTransmittance[2]);
                if(currentPathSegmentData)
                    openpgl::cpp::SetTransmittanceWeight(currentPathSegmentData, pglTransmittance);

                nextPathSegmentData = pathSegmentDataStorage->NextSegment();
                trackedTransmittanceWeight = Spectrum(1.0f);
                openpgl::cpp::SetPosition(nextPathSegmentData, pglEnvPos);
                openpgl::cpp::SetNormal(nextPathSegmentData, pglNormal);
                openpgl::cpp::SetDirectionOut(nextPathSegmentData, pglDirectionOut);
                openpgl::cpp::SetDirectContribution(nextPathSegmentData, openpgl::cpp::Vector3(nextHitEmitterEmission[0],nextHitEmitterEmission[1],nextHitEmitterEmission[2]));
                openpgl::cpp::SetMiWeight(nextPathSegmentData, nextHitEmitterMIS);
                openpgl::cpp::SetRussianRouletteProbability(nextPathSegmentData,1.0f);
                //openpgl::cpp::SetTransmittanceWeight(nextPathSegmentData, pglTransmittance);
                //pathSegmentDataStorage->Print();
            }
        }


        avgPathLength.incrementBase();
        avgPathLength += rRec.depth;

        avgPerIterationPathLength.incrementBase();
        avgPerIterationPathLength += rRec.depth;

        if (m_training)
        {
            pgl_vec3f pEst = pathSegmentDataStorage->CalculatePixelEstimate(false);
            pEst.x *= cameraTransmittance[0];
            pEst.y *= cameraTransmittance[1];
            pEst.z *= cameraTransmittance[2];
            pgl_vec3f cT = openpgl::cpp::Vector3(cameraTransmittance[0], cameraTransmittance[1], cameraTransmittance[2]);
            float eps = 1e-4f;
            /*
            if(rRec.depth > 1 && ((Li[0] > 1e-6f && std::abs(pEst.x - Li[0])/Li[0] > eps) || (Li[1] > 1e-6f && std::abs(pEst.y - Li[1])/Li[1] > eps) || (Li[2] > 1e-6f && std::abs(pEst.z - Li[2])/Li[2] > eps ))) {
                //std::cout << "maxEventsReached = " << maxEventsReached << std::endl;
                
                std::cout << "pEst: " << pEst.x << "\t" << pEst.y << "\t" << pEst.z << "\tLi: " << Li[0] << "\t" << Li[1] << "\t" << Li[2] << std::endl;
                std::cout << "cameraTransmittance = " << cameraTransmittance.toString() << std::endl;
                pEst = pathSegmentDataStorage->CalculatePixelEstimate2(cT, true);
                for (int i =0; i< throughputs.size();i++){
                    std::cout << "TR["<< i << "] = " << throughputs[i].toString() << std::endl;
                }
                pathSegmentDataStorage->Print();
            }
            */
           pathSegmentDataStorage->PropagateSamples(&m_sampleStorage, m_guideDirectLight, m_accountForDirectLightMiWeight);
        }
        else
        {
            pathSegmentDataStorage->Clear();
        }
#if defined(GUIDING_RR) || defined(SCATTER_GUIDING)
        denoiseSample.color = Li;
        m_pixelEstimateDenoiser.add(rRec.pixelId, denoiseSample);
#endif
        return Li;
    }

    /**
     * This function is called by the recursive ray tracing above after
     * having sampled a direction from a BSDF/phase function. Due to the
     * way in which this integrator deals with index-matched boundaries,
     * it is necessarily a bit complicated (though the improved performance
     * easily pays for the extra effort).
     *
     * This function
     *
     * 1. Intersects 'ray' against the scene geometry and returns the
     *    *first* intersection via the '_its' argument.
     *
     * 2. It checks whether the intersected shape was an emitter, or if
     *    the ray intersects nothing and there is an environment emitter.
     *    In this case, it returns the attenuated emittance, as well as
     *    a DirectSamplingRecord that can be used to query the hypothetical
     *    sampling density at the emitter.
     *
     * 3. If current shape is an index-matched medium transition, the
     *    integrator keeps on looking on whether a light source eventually
     *    follows after a potential chain of index-matched medium transitions,
     *    while respecting the specified 'maxDepth' limits. It then returns
     *    the attenuated emittance of this light source, while accounting for
     *    all attenuation that occurs on the wya.
     */
    void rayIntersectAndLookForEmitter(const Scene *scene, Sampler *sampler,
            const Medium *medium, bool &passedMedium, int maxInteractions, Ray ray, Intersection &_its,
            DirectSamplingRecord &dRec, Spectrum &value, Point &lightPos, Spectrum &transmittance) const {
        Intersection its2, *its = &_its;
        transmittance = Spectrum(1.0f);
        bool surface = false;
        int interactions = 0;

        while (true) {
            surface = scene->rayIntersect(ray, *its);

            if (medium){
                //transmittance *= medium->evalTransmittance(Ray(ray, 0, its->t), sampler);
                transmittance *= m_distanceSampler->evalTransmittance(medium, Ray(ray, 0, its->t), sampler);
            }
            if (surface && (interactions == maxInteractions ||
                !(its->getBSDF()->getType() & BSDF::ENull) ||
                its->isEmitter())) {
                /* Encountered an occluder / light source */
                break;
            }

            if (!surface)
                break;

            if (transmittance.isZero())
                return;

            if (its->isMediumTransition()){
                medium = its->getTargetMedium(ray.d);
                passedMedium = true;
            }

            Vector wo = its->shFrame.toLocal(ray.d);
            BSDFSamplingRecord bRec(*its, -wo, wo, ERadiance);
            bRec.typeMask = BSDF::ENull;
            transmittance *= its->getBSDF()->eval(bRec, EDiscrete);

            ray.o = ray(its->t);
            ray.mint = Epsilon;
            its = &its2;
#ifdef MTS_IGNORE_NULLBSDF_INTERSECTIONS
            if (++interactions > 100) { /// Just a precaution..
                Log(EWarn, "rayIntersectAndLookForEmitter(): round-off error issues?");
                return;
            }
#endif
        }

        if (surface) {
            /* Intersected something - check if it was a luminaire */
            if (its->isEmitter()) {
                dRec.setQuery(ray, *its);
                value = transmittance * its->Le(-ray.d);
                lightPos = its->p;
                //std::cout << its->toString() <<std::endl;
            }
        } else {
            /* Intersected nothing -- perhaps there is an environment map? */
            const Emitter *env = scene->getEnvironmentEmitter();

            if (env && env->fillDirectSamplingRecord(dRec, ray)){
                value = transmittance * env->evalEnvironment(RayDifferential(ray));
                lightPos = _its.p + m_envmapDistance * ray.d;
            }
        }
    }

    inline Float miWeight(Float pdfA, Float pdfB) const {
        pdfA *= pdfA; pdfB *= pdfB;
        return pdfA / (pdfA + pdfB);
    }

    void serialize(Stream *stream, InstanceManager *manager) const {
        MonteCarloIntegrator::serialize(stream, manager);
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << "ProgressiveVolumetricPathTracer[" << endl
            << "  maxDepth = " << m_maxDepth << "," << endl
            << "  rrDepth = " << m_rrDepth << "," << endl
            << "  strictNormals = " << m_strictNormals << endl
            << "]";
        return oss.str();
    }

private:

    mutable std::vector<openpgl::cpp::PathSegmentStorage*> m_perThreadPathSegmentDataStorage;
    mutable std::vector<GuidedBSDFType*> m_perThreadGuidedBSDF;
    mutable std::vector<GuidedPhaseFunctionType*> m_perThreadGuidedPhaseFunction;
    mutable openpgl::cpp::SampleStorage m_sampleStorage;
    mutable std::atomic<size_t> m_maxThreadId;
    mutable PrimitiveThreadLocal<bool> m_threadLocalInitialized;
    mutable PrimitiveThreadLocal<size_t> m_threadLocalId;

    size_t m_minSamplesToStartFitting{128};

    AABB m_sceneSpace;
    Float m_envmapDistance;

    size_t m_overallTrainingSamples;
    size_t m_trainingSamplesPerIteration;

    std::unique_ptr<openpgl::cpp::Device> m_guidingDevice;
    Properties m_guidingFieldProps;
    std::unique_ptr<openpgl::cpp::Field> m_guidingField;
    openpgl::cpp::FieldConfig m_guidingFieldConfig;

    bool m_accountForDirectLightMiWeight{false};
    bool m_deterministic;

#if defined(GUIDING_RR)
    bool m_guidedRR {true};
#endif

#if defined(SCATTER_GUIDING)
    bool m_guidedScatter {false};
#endif

#if defined(GUIDING_RR) || defined(SCATTER_GUIDING)
    bool m_pixelEstimateReady {false};
    mutable Denoiser m_pixelEstimateDenoiser;

    bool m_calulatePixelEstimate {false};
    int m_pixelEstimateWave {0};
    bool m_savePixelEstimate {false};
    bool m_loadPixelEstimate {false};
    std::string m_pixelEstimateFile {""};
    ref<Timer> m_denoiseTimer;
#endif
    bool m_rrCorrection {true};
    int m_iteration {0};

    Guiding::SurfaceAdjointType m_surfaceAdjointType {Guiding::ESALo};
    Guiding::VolumeAdjointType m_volumeAdjointType {Guiding::EVALo};

    bool m_useCosineProduct;
    //bool m_useBSDFProduct;

    bool m_useSurfaceGuiding;
    bool m_useVolumeGuiding;

    float m_bsdfProbability;
    float m_phaseFunctionProbability;

    bool m_storeSamples;

    bool m_training;
    volatile bool m_canceled;

    bool m_useNee;
    bool m_guideDirectLight;

    GuidedBSDFType::GuidingTypes m_guidingType {GuidedBSDFType::EGUIDING_TYPE_PRODUCT};

    bool m_saveGuidingCaches {false};
    bool m_loadGuidingCaches {false};
    std::string m_guidingCachesFile {""};

    ref<Timer> m_fieldUpdateTimer;
    std::vector<float> m_fieldUpdateTimings;
    std::vector<int> m_sampleCounts;


    Guiding::DistanceSamplerTypes m_distanceSamplerType {Guiding::EStandardDS};
    Guiding::DistanceSampler *m_distanceSampler;

    MTS_DECLARE_CLASS()
};

MTS_IMPLEMENT_CLASS_S(GuidedProgressiveVolumetricPathTracer, false, ProgressiveMonteCarloIntegrator)
MTS_EXPORT_PLUGIN(GuidedProgressiveVolumetricPathTracer, "Guided volumetric path tracer (PGL)");
MTS_NAMESPACE_END