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

#include <mitsuba/render/scene.h>
#include <mitsuba/render/progressiveintegrator.h>

// including headers from the guiding libary
#include <openpgl/cpp/OpenPGL.h>

#include <boost/filesystem.hpp>

#include "GuidingHelper.h"            // type definitions and Mitsuba Property parser
#include "GuidedBSDF.h"                            // implementation of a guided (MIS-bases) BSDF wrapper
#include "GuidedPhaseFunction.h"    // implementation of a guided (MIS-bases) phase functiom wrapper
#include "GuidedRussianRouletteAndSplitting.h"

#include <atomic>

//#define SUPPORT_DENOISING
//#define GUIDING_RR

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


/*! \plugin{path}{Path tracer}
 * \order{2}
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
 *        inconsistencies involving shading normals? See the description below
 *        for details.\default{no, i.e. \code{false}}
 *     }
 *     \parameter{hideEmitters}{\Boolean}{Hide directly visible emitters?
 *        See page~\pageref{sec:hideemitters} for details.
 *        \default{no, i.e. \code{false}}
 *     }
 * }
 *
 * This integrator implements a basic path tracer and is a \emph{good default choice}
 * when there is no strong reason to prefer another method.
 *
 * To use the path tracer appropriately, it is instructive to know roughly how
 * it works: its main operation is to trace many light paths using \emph{random walks}
 * starting from the sensor. A single random walk is shown below, which entails
 * casting a ray associated with a pixel in the output image and searching for
 * the first visible intersection. A new direction is then chosen at the intersection,
 * and the ray-casting step repeats over and over again (until one of several
 * stopping criteria applies).
 * \begin{center}
 * \includegraphics[width=.7\textwidth]{images/integrator_path_figure.pdf}
 * \end{center}
 * At every intersection, the path tracer tries to create a connection to
 * the light source in an attempt to find a \emph{complete} path along which
 * light can flow from the emitter to the sensor. This of course only works
 * when there is no occluding object between the intersection and the emitter.
 *
 * This directly translates into a category of scenes where
 * a path tracer can be expected to produce reasonable results: this is the case
 * when the emitters are easily ``accessible'' by the contents of the scene. For instance,
 * an interior scene that is lit by an area light will be considerably harder
 * to render when this area light is inside a glass enclosure (which
 * effectively counts as an occluder).
 *
 * Like the \pluginref{direct} plugin, the path tracer internally relies on multiple importance
 * sampling to combine BSDF and emitter samples. The main difference in comparison
 * to the former plugin is that it considers light paths of arbitrary length to compute
 * both direct and indirect illumination.
 *
 * For good results, combine the path tracer with one of the
 * low-discrepancy sample generators (i.e. \pluginref{ldsampler},
 * \pluginref{halton}, or \pluginref{sobol}).
 *
 * \paragraph{Strict normals:}\label{sec:strictnormals}
 * Triangle meshes often rely on interpolated shading normals
 * to suppress the inherently faceted appearance of the underlying geometry. These
 * ``fake'' normals are not without problems, however. They can lead to paradoxical
 * situations where a light ray impinges on an object from a direction that is classified as ``outside''
 * according to the shading normal, and ``inside'' according to the true geometric normal.
 *
 * The \code{strictNormals}
 * parameter specifies the intended behavior when such cases arise. The default (\code{false}, i.e. ``carry on'')
 * gives precedence to information given by the shading normal and considers such light paths to be valid.
 * This can theoretically cause light ``leaks'' through boundaries, but it is not much of a problem in practice.
 *
 * When set to \code{true}, the path tracer detects inconsistencies and ignores these paths. When objects
 * are poorly tesselated, this latter option may cause them to lose a significant amount of the incident
 * radiation (or, in other words, they will look dark).
 *
 * The bidirectional integrators in Mitsuba (\pluginref{bdpt}, \pluginref{pssmlt}, \pluginref{mlt} ...)
 * implicitly have \code{strictNormals} set to \code{true}. Hence, another use of this parameter
 * is to match renderings created by these methods.
 *
 * \remarks{
 *    \item This integrator does not handle participating media
 *    \item This integrator has poor convergence properties when rendering
 *    caustics and similar effects. In this case, \pluginref{bdpt} or
 *    one of the photon mappers may be preferable.
 * }
 */

class GuidedProgressivePathTracer : public ProgressiveMonteCarloIntegrator {
public:
    GuidedProgressivePathTracer(const Properties &props)
        : ProgressiveMonteCarloIntegrator(props) {

        ///////////////////////////////////////////////
        /// Guiding: setting up the parameters
        /// Note: usually only a small subset is needed
        ///////////////////////////////////////////////

        m_useNee                        = props.getBoolean("useNee", true);

        //guiding settings
        m_useSurfaceGuiding             = props.getBoolean("useSurfaceGuiding", true);
        m_guideDirectLight              = props.getBoolean("guideDirectLight", true);
        m_useCosineProduct              = props.getBoolean("useCosineProduct", true);
        //m_useBSDFProduct                = props.getBoolean("useBSDFProduct", false);
        m_bsdfProbability               = props.getFloat("bsdfProbability", 0.5f);
#ifdef GUIDING_RR
        m_guidedRR                      = props.getBoolean("guidedRR", false);
#endif
        m_rrCorrection                  = props.getBoolean("useRRCorrection", true);

        m_guidingType                   = (GuidedBSDFType::GuidingTypes) props.getInteger("guidingType", (int)GuidedBSDFType::EGUIDING_TYPE_PRODUCT);

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

        m_training = false;
        m_canceled = false;

		m_saveGuidingCaches = props.getBoolean("saveGuidingCaches", false);
		m_loadGuidingCaches = props.getBoolean("loadGuidingCaches", false);
		m_guidingCachesFile = props.getString("guidingCachesFile", "guidingCaches.field");   

        if(m_loadGuidingCaches)
		{
			fs::path guidingCachesFile = Thread::getThread()->getFileResolver()->resolve(m_guidingCachesFile);

            if (!fs::exists(guidingCachesFile))
                Log(EError, "Guiding caches file \"%s\" could not be found!", guidingCachesFile.string().c_str());

			m_guidingCachesFile = guidingCachesFile.string();
		}	

        m_accountForDirectLightMiWeight = m_useNee;

#ifdef GUIDING_RR
        if(m_guidedRR)
        {
            m_rrDepth = 1;
        }
#endif
    }

    /// Unserialize from a binary data stream
    GuidedProgressivePathTracer(Stream *stream, InstanceManager *manager)
        : ProgressiveMonteCarloIntegrator(stream, manager) { }


    bool preprocess(const Scene *scene, RenderQueue *queue, const RenderJob *job, int sceneResID, int sensorResID, int samplerResID) {
        bool success = ProgressiveMonteCarloIntegrator::preprocess(scene, queue, job, sceneResID, sensorResID, samplerResID);
        Log(EInfo, this->toString().c_str());
        if(success) {
            m_iteration = 0;
            m_canceled = false;
            m_training = true;
            //if (m_useBSDFProduct)
            //    BSDFOracle::initBSDFRepresentations(scene);

            m_sceneSpace = scene->getKDTree()->getAABB();
            m_envmapDistance = scene->getAABB().getBSphere().radius*1024.0f;

            //set up per thread storage of sample data
            ref<Scheduler> sched = Scheduler::getInstance();
            const size_t nCores = sched->getCoreCount();
            m_maxThreadId.store(0);

            // init the storage for the per path segment data
            // -- one instance per thread
            m_perThreadPathSegmentDataStorage.resize(nCores);
            for(size_t i=0; i<nCores; i++)
            {
#ifdef OPENPGL_EF_RADIANCE_CACHES
                m_perThreadPathSegmentDataStorage[i] = new openpgl::cpp::PathSegmentStorage(true);
#else
                m_perThreadPathSegmentDataStorage[i] = new openpgl::cpp::PathSegmentStorage();
#endif
                m_perThreadPathSegmentDataStorage[i]->Reserve( 2 * (m_maxDepth+2));
            }

            // set up the storage for the training samples
            m_sampleStorage = new openpgl::cpp::SampleStorage();
            m_sampleStorage->Reserve(sched->getCoreCount()*1024*1024, sched->getCoreCount()*1024*1024);
            m_sampleStorage->Clear();
            // Parse the settings for the guiding field and create an empty field
            m_guidingDevice = std::unique_ptr<openpgl::cpp::Device>(new openpgl::cpp::Device(PGL_DEVICE_TYPE_CPU_4));
            Guiding::parseFieldProperties(m_guidingFieldProps, m_guidingFieldConfig);
            SLog(EInfo, "LoadField: %d \t exists: %d", (int)m_loadGuidingCaches, (int)fs::exists(m_guidingCachesFile));
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
            }
            m_perThreadGuidedBSDF.resize(nCores);
            for(int i=0; i<nCores; i++)
            {
                m_perThreadGuidedBSDF[i] = new GuidedBSDFType(&*m_guidingField, m_useSurfaceGuiding, m_bsdfProbability, m_useCosineProduct, false);
            }
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

        SLog(EInfo, "Avg. path length: %f (%d/%d)",
            (float)avgPathLength.getValue()/(float)avgPathLength.getBase(),
            avgPathLength.getValue(),
            avgPathLength.getBase());
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
        ref<Scene> scene = static_cast<Scene *>(sched->getResource(sceneResID));
        ref<Sensor> sensor = static_cast<Sensor *>(sched->getResource(sensorResID));
        ref<Film> film = sensor->getFilm();

        auto sceneName = scene->getSourceFile().stem().string();

        ///////////////////////////////////////////////
        /// Guiding: Fitting/Updating
        ///////////////////////////////////////////////

        if (m_training){
            ref<Timer> trainingTimer                = new Timer();
#ifdef OPENPGL_EF_RADIANCE_CACHES
            SLog(EInfo, "training: %d surface samples: %d \t invalid surface samples: %d \t volume samples: %d \t invalid volume samples: %d \t minSamplesToStartFitting: %d.",
                (int)m_training,
                (int)m_sampleStorage->GetSizeSurface(),
                (int)m_sampleStorage->GetSizeInvalidSurface(),
                (int)m_sampleStorage->GetSizeVolume(),
                (int)m_sampleStorage->GetSizeInvalidVolume(),
                m_minSamplesToStartFitting);
#else
            SLog(EInfo, "training: %d surface samples: %d \t volume samples: %d \t minSamplesToStartFitting: %d.",
                (int)m_training,
                (int)m_sampleStorage->GetSizeSurface(),
                (int)m_sampleStorage->GetSizeVolume(),
                m_minSamplesToStartFitting);
#endif


#if defined(MTS_OPENMP)
            ref<Scheduler> scheduler = Scheduler::getInstance();
            size_t nCores = scheduler->getCoreCount();
            mitsuba::Thread::initializeOpenMP(nCores);
#endif

            // fit/update guiding cache
            const size_t numValidSamples = m_sampleStorage->GetSizeSurface() + m_sampleStorage->GetSizeVolume();
            if (EXPECT_TAKEN(numValidSamples >= m_minSamplesToStartFitting))
            {
                pgl_box3f bounds;
                pglBox3f(bounds, m_sceneSpace.min.x, m_sceneSpace.min.y, m_sceneSpace.min.z, m_sceneSpace.max.x, m_sceneSpace.max.y, m_sceneSpace.max.z);

                m_guidingField->SetSceneBounds(bounds);
                m_guidingField->Update(*m_sampleStorage);

                if(m_guidingField->GetIteration()*m_trainingSamplesPerIteration>= m_overallTrainingSamples)
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
                m_sampleStorage->Clear();
            }
            Log( EInfo, "Training[%d]: took %fs", m_guidingField->GetIteration(), trainingTimer->getMilliseconds() * 1e-3f );
            m_iteration++;
        }
    }

    Spectrum Li(const RayDifferential &r, RadianceQueryRecord &rRec) const {

	    //Spectrum pixelEstimate = rRec.pixelEstimate;
        //return pixelEstimate;
        // guiding specific thread local instances
	    static thread_local openpgl::cpp::PathSegmentStorage* pathSegmentDataStorage;
        static thread_local GuidedBSDFType* guidedBSDF;
        // initialize thread local instances on first use
        if (EXPECT_NOT_TAKEN(!m_threadLocalInitialized.get()))
        {
            const size_t threadId = m_maxThreadId.fetch_add(1, std::memory_order_relaxed);
            pathSegmentDataStorage = m_perThreadPathSegmentDataStorage.at(threadId);
            guidedBSDF = m_perThreadGuidedBSDF.at(threadId);
            m_threadLocalInitialized.get() = true;
        }

        // storage for the per-path segment data
        openpgl::cpp::PathSegment* nextPathSegmentData;
        openpgl::cpp::PathSegment* currentPathSegmentData;

        /* Some aliases and local variables */
        const Scene *scene = rRec.scene;
        Intersection &its = rRec.its;
        RayDifferential ray(r);
        Spectrum Li(0.0f);
        bool scattered = false;

        /* Perform the first ray intersection (or ignore if the
           intersection has already been provided). */
        rRec.rayIntersect(ray);
        ray.mint = Epsilon;

        Spectrum throughput(1.0f);
        Float eta = 1.0f;
#ifdef SUPPORT_DENOISING
        int numDiffuseGlossyInteractions = 0;
#endif
        pgl_vec3f pglZero = openpgl::cpp::Vector3(0.0f, 0.0f, 0.0f);
        pgl_vec3f pglOne = openpgl::cpp::Vector3(1.0f, 1.0f, 1.0f);

        float rrCorrectionFactor = 1.0f;
#ifdef GUIDING_RR
        Float q = 1.0f;
        bool guideRR = false;
#endif
        while (rRec.depth <= m_maxDepth || m_maxDepth < 0) {
#ifdef GUIDING_RR
            q = 1.0f;
            guideRR = false;
#endif
            if (!its.isValid()) {
                /* If no intersection could be found, potentially return
                   radiance from a environment luminaire if it exists */
                if ((rRec.type & RadianceQueryRecord::EEmittedRadiance)
                    && (!m_hideEmitters || scattered)) {
                    Spectrum value = throughput * scene->evalEnvironment(ray);
                    Li += value;
#ifdef SUPPORT_DENOISING
                    if (numDiffuseGlossyInteractions == 0)
                        rRec.primaryAlbedo = value;
#endif
                }
                break;
            }else{
                
                // if the intersection is valid and we are at the first intersection
                if(!scattered && m_training)
                {
                    const Vector3 normal = its.toWorld(Vector3(0.0,0.0,1.0));
                    const Vector3 wi = its.toWorld(its.wi);
                    pgl_point3f pglP = openpgl::cpp::Point3(its.p[0], its.p[1], its.p[2]);
                    pgl_vec3f pglNormal = openpgl::cpp::Vector3(normal[0], normal[1], normal[2]);
                    pgl_vec3f pglWi = openpgl::cpp::Vector3(wi[0], wi[1], wi[2]);
                    nextPathSegmentData = pathSegmentDataStorage->NextSegment();
                    openpgl::cpp::SetPosition(nextPathSegmentData, pglP);
                    openpgl::cpp::SetNormal(nextPathSegmentData, pglNormal);
                    openpgl::cpp::SetDirectionOut(nextPathSegmentData, pglWi);
                    openpgl::cpp::SetVolumeScatter(nextPathSegmentData, false);
                    openpgl::cpp::SetScatteredContribution(nextPathSegmentData, pglZero);
                    openpgl::cpp::SetDirectContribution(nextPathSegmentData, pglZero);
                    openpgl::cpp::SetTransmittanceWeight(nextPathSegmentData, pglOne);
                    openpgl::cpp::SetEta(nextPathSegmentData, 1.0);
                }
            }

            if (m_training)
            {
                currentPathSegmentData = nextPathSegmentData;
                pgl_vec3f pglTransmittanceWeight = openpgl::cpp::Vector3(1.0f, 1.0f, 1.0f);
                openpgl::cpp::SetTransmittanceWeight(currentPathSegmentData, pglTransmittanceWeight);
            }


            const BSDF *surfaceBSDF = its.getBSDF(ray);

#ifdef SUPPORT_DENOISING
            /* Adding denoise features */
            const int bsdfType = surfaceBSDF->getType();
            if ((bsdfType & BSDF::EDiffuse || bsdfType & BSDF::EGlossy) && numDiffuseGlossyInteractions == 0) {
                rRec.primaryAlbedo = throughput * surfaceBSDF->getAlbedo(rRec.its);
                rRec.primaryNormal = rRec.its.toWorld(Vector3(0, 0, 1));
                numDiffuseGlossyInteractions++;
            }
#endif

            guidedBSDF->prepare(&*m_guidingField, its, -ray.d, surfaceBSDF, rRec.sampler);
            Spectrum rrThroughput = throughput;
#ifdef OPENPGL_EF_RADIANCE_CACHES
            Spectrum outgoingRadiance = guidedBSDF->outgoingRadiance(-ray.d);
#else
            Spectrum outgoingRadiance(1.0f);
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

            /* Include radiance from a subsurface scattering model if requested */
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

            /* ==================================================================== */
            /*                     Direct illumination sampling                     */
            /* ==================================================================== */

            /* Estimate the direct illumination if this is requested */
            DirectSamplingRecord dRec(its);

            if (m_useNee && (rRec.type & RadianceQueryRecord::EDirectSurfaceRadiance &&
                (guidedBSDF->getType() & BSDF::ESmooth))) {
                Spectrum value = scene->sampleEmitterDirect(dRec, rRec.nextSample2D());
                if (!value.isZero()) {
                    const Emitter *emitter = static_cast<const Emitter *>(dRec.object);

                    /* Allocate a record for querying the BSDF */
                    BSDFSamplingRecord bRec(its, its.toLocal(dRec.d), ERadiance);

                    /* Evaluate BSDF * cos(theta) */
                    const Spectrum bsdfVal = guidedBSDF->eval(bRec);

                    /* Prevent light leaks due to the use of shading normals */
                    if (!bsdfVal.isZero() && (!m_strictNormals
                            || dot(its.geoFrame.n, dRec.d) * Frame::cosTheta(bRec.wo) > 0)) {

                        /* Calculate prob. of having generated that direction
                           using BSDF sampling */
                        Float bsdfPdf = (emitter->isOnSurface() && dRec.measure == ESolidAngle)
                            ? guidedBSDF->pdf(bRec) : (Float) 0.0f;

                        /* Weight using the power heuristic */
#ifdef GUIDING_RR
                        Float weight = miWeight(dRec.pdf, q*bsdfPdf);
#else
                        Float weight = miWeight(dRec.pdf, bsdfPdf);
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
            if (bsdfWeight.isZero()) {
                break;
            }

            scattered |= bRec.sampledType != BSDF::ENull;

            /* Prevent light leaks due to the use of shading normals */
            const Vector wo = its.toWorld(bRec.wo);
            Float woDotGeoN = dot(its.geoFrame.n, wo);

            if (m_training)
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

            ++rRec.depth;

            if (m_strictNormals && woDotGeoN * Frame::cosTheta(bRec.wo) <= 0)
                break;

            bool hitEmitter = false;
            Spectrum value;

            /* Trace a ray in this direction */
            ray = Ray(its.p, wo, ray.time);
            if (scene->rayIntersect(ray, its)) {
                /* Intersected something - check if it was a luminaire */
                if (its.isEmitter()) {
                    value = its.Le(-ray.d);
                    dRec.setQuery(ray, its);
                    hitEmitter = true;
                }
                Vector3 normal = its.toWorld(Vector3(0.0,0.0,1.0));
                Vector3 worldWi = its.toWorld(its.wi);
                pgl_point3f pglP = openpgl::cpp::Point3(its.p[0], its.p[1], its.p[2]);
                pgl_vec3f pglWi = openpgl::cpp::Vector3(worldWi[0], worldWi[1], worldWi[2]);
                pgl_vec3f pglNormal = openpgl::cpp::Vector3(normal[0], normal[1], normal[2]);

                nextPathSegmentData = pathSegmentDataStorage->NextSegment();
                openpgl::cpp::SetPosition(nextPathSegmentData, pglP);
                openpgl::cpp::SetNormal(nextPathSegmentData, pglNormal);
                openpgl::cpp::SetDirectionOut(nextPathSegmentData, pglWi);
            } else {
                /* Intersected nothing -- perhaps there is an environment map? */
                const Emitter *env = scene->getEnvironmentEmitter();

                if (env) {
                    if (m_hideEmitters && !scattered)
                        break;

                    value = env->evalEnvironment(ray);
                    if (!env->fillDirectSamplingRecord(dRec, ray))
                        break;
                    hitEmitter = true;
                    Point3 envPos = ray.o + (m_envmapDistance * ray.d);
                    pgl_point3f pglEnvPos = openpgl::cpp::Vector3(envPos[0], envPos[1], envPos[2]);
                    pgl_vec3f pglNormal = openpgl::cpp::Vector3(0.0f, 0.0f, 1.0f);
                    pgl_vec3f pglDirectionOut = openpgl::cpp::Vector3(-ray.d[0],-ray.d[1],-ray.d[2]);

                    nextPathSegmentData = pathSegmentDataStorage->NextSegment();
                    openpgl::cpp::SetPosition(nextPathSegmentData, pglEnvPos);
                    openpgl::cpp::SetNormal(nextPathSegmentData, pglNormal);
                    openpgl::cpp::SetDirectionOut(nextPathSegmentData, pglDirectionOut);
                } else {
                    break;
                }
            }

            /* Keep track of the throughput and relative
               refractive index along the path */
            //Spectrum outgoingRadiance = guidedBSDF->outgoingRadiance(-ray.d);
            throughput *= bsdfWeight;
            eta *= bRec.eta;

            /* If a luminaire was hit, estimate the local illumination and
               weight using the power heuristic */
            if (hitEmitter &&
                (rRec.type & RadianceQueryRecord::EDirectSurfaceRadiance)) {
                /* Compute the prob. of generating that direction using the
                   implemented direct illumination sampling technique */
                const Float lumPdf = (m_useNee && !(bRec.sampledType & BSDF::EDelta)) ?
                    scene->pdfEmitterDirect(dRec) : 0;
#ifdef GUIDING_RR
                Float weight = (m_useNee) ? miWeight(q*surfaceScatterMISPdf, lumPdf) : 1.0f;
#else
                Float weight = (m_useNee) ? miWeight(surfaceScatterMISPdf, lumPdf) : 1.0f;
#endif
                Li += throughput * value * weight;

                openpgl::cpp::SetDirectContribution(nextPathSegmentData, openpgl::cpp::Vector3(value[0],value[1],value[2]));
                openpgl::cpp::SetMiWeight(nextPathSegmentData, weight);
            }

            /* ==================================================================== */
            /*                         Indirect illumination                        */
            /* ==================================================================== */

            /* Set the recursive query type. Stop if no surface was hit by the
               BSDF sample or if indirect illumination was not requested */
            if (!its.isValid() || !(rRec.type & RadianceQueryRecord::EIndirectSurfaceRadiance))
                break;
            rRec.type = RadianceQueryRecord::ERadianceNoEmission;

            if (m_rrDepth >= 0 && rRec.depth > m_rrDepth) {
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

                //if (!std::isfinite(q) || std::isnan(q))
                //    SLog(EInfo, "q = %f \t rrCorrectionFactor = %f \t rrCorrect = %f", q, rrCorrectionFactor, rrCorrect);
                if (rRec.nextSample1D() >= q)
                    break;
                throughput /= q;
                //if (!throughput.isValid())
                //    SLog(EInfo, "q = %f \t rrCorrectionFactor = %f \t rrCorrect = %f", q, rrCorrectionFactor, rrCorrect);
            }
        }

        /* Store statistics */
        avgPathLength.incrementBase();
        avgPathLength += rRec.depth;

        avgPerIterationPathLength.incrementBase();
        avgPerIterationPathLength += rRec.depth;

        if (m_training)
        {
            /*
            size_t nSamples;
            pathSegmentDataStorage->PrepareSamples(m_accountForDirectLightMiWeight, m_guideDirectLight);
            const openpgl::cpp::SampleData* samples = pathSegmentDataStorage->GetSamples(nSamples);
            m_sampleStorage->AddSamples(samples, nSamples);
            pathSegmentDataStorage->Clear();
            */
            /* */
            pgl_vec3f pEst = pathSegmentDataStorage->CalculatePixelEstimate(true);
            if(rRec.depth > 1 && ((Li[0] > 0.f && std::abs(pEst.x - Li[0])/Li[0] > 1e-6f) || (Li[1] > 0.f && std::abs(pEst.y - Li[1])/Li[1] > 1e-6f) || (Li[2] > 0.f && std::abs(pEst.z - Li[2])/Li[2] > 1e-6f ))) {
                std::cout << "pEst: " << pEst.x << "\t" << pEst.y << "\t" << pEst.z << "\tLi: " << Li[0] << "\t" << Li[1] << "\t" << Li[2] << std::endl;
            }
        /*
           if(rRec.pixelId == (312*1024 + 437)) {
                pathSegmentDataStorage->Print();
                pgl_vec3f pEst = pathSegmentDataStorage->CalculatePixelEstimate(true);
                //std::cout << "pEst: " << pEst.x << "\t" << pEst.y << "\t" << pEst.z << std::endl;
                //std::cout << "Li: " << Li[0] << "\t" << Li[1] << "\t" << Li[2] << std::endl;  
           }
           */
           pathSegmentDataStorage->PropagateSamples(m_sampleStorage, m_guideDirectLight, m_accountForDirectLightMiWeight);
        }
        else
        {
            pathSegmentDataStorage->Clear();
        }

        return Li;
    }

    inline Float miWeight(Float pdfA, Float pdfB) const {
        pdfA *= pdfA;
        pdfB *= pdfB;
        return pdfA / (pdfA + pdfB);
    }

    void serialize(Stream *stream, InstanceManager *manager) const {
        MonteCarloIntegrator::serialize(stream, manager);
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << "GuidedProgressivePathTracer[" << endl
            << "  maxDepth = " << m_maxDepth << "," << endl
            << "  rrDepth = " << m_rrDepth << "," << endl
            << "  strictNormals = " << m_strictNormals << endl
            << "  tech = " << m_guidingFieldProps.getString("directionalDistribution") << endl
            << "  est = " << m_guidingFieldProps.getString("leafEstimator") << endl
            << "]";
        return oss.str();
    }

private:

    mutable std::vector<openpgl::cpp::PathSegmentStorage*> m_perThreadPathSegmentDataStorage;
    mutable std::vector<GuidedBSDFType*> m_perThreadGuidedBSDF;
    mutable openpgl::cpp::SampleStorage* m_sampleStorage;
    mutable std::atomic<size_t> m_maxThreadId;
    mutable PrimitiveThreadLocal<bool> m_threadLocalInitialized;

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

#ifdef GUIDING_RR
    bool m_guidedRR {true};
#endif
    bool m_rrCorrection {true};
    int m_iteration {0};

    float m_bsdfProbability;
    bool m_useCosineProduct;
    //bool m_useBSDFProduct;

    bool m_useSurfaceGuiding;

    bool m_training;
    volatile bool m_canceled;

    bool m_useNee;
    bool m_guideDirectLight;

    GuidedBSDFType::GuidingTypes m_guidingType {GuidedBSDFType::EGUIDING_TYPE_PRODUCT};

    bool m_saveGuidingCaches {false};
    bool m_loadGuidingCaches {false};
    std::string m_guidingCachesFile {""};
    MTS_DECLARE_CLASS()
};

MTS_IMPLEMENT_CLASS_S(GuidedProgressivePathTracer, false, MonteCarloIntegrator)
MTS_EXPORT_PLUGIN(GuidedProgressivePathTracer, "Guided MI path tracer (PGL)");
MTS_NAMESPACE_END
