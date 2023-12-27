#include <mitsuba/core/bitmap.h>
#include <mitsuba/core/fresolver.h>
#include <mitsuba/core/fstream.h>

#include <mitsuba/render/progressiveintegrator.h>
#include <mitsuba/render/renderproc.h>

#include <boost/filesystem.hpp>

MTS_NAMESPACE_BEGIN

	void DenoiseBuffer::init(const Vector2i filmSize) {
		m_filmSize = filmSize;
		size_t numPixels = filmSize[0] * filmSize[1];
		m_filterColor = std::unique_ptr<Vector3[]>(new Vector3[numPixels]);
		m_filterAlbedo = std::unique_ptr<Vector3[]>(new Vector3[numPixels]);
		m_filterAlbedoOutput = std::unique_ptr<Vector3[]>(new Vector3[numPixels]);
		m_filterNormal = std::unique_ptr<Vector3[]>(new Vector3[numPixels]);
		m_filterNormalOutput = std::unique_ptr<Vector3[]>(new Vector3[numPixels]);
		m_filterOutput = std::unique_ptr<Vector3[]>(new Vector3[numPixels]);

		m_oidnDevice = oidn::newDevice();
		m_oidnDevice.commit();

		m_oidnAlbedoFilter = m_oidnDevice.newFilter("RT");
		m_oidnAlbedoFilter.setImage("albedo", &m_filterAlbedo[0][0], oidn::Format::Float3, filmSize[0], filmSize[1]);
		m_oidnAlbedoFilter.setImage("output", &m_filterAlbedoOutput[0][0], oidn::Format::Float3, filmSize[0], filmSize[1]);
		m_oidnAlbedoFilter.commit();
		
		m_oidnNormalFilter = m_oidnDevice.newFilter("RT");
		m_oidnNormalFilter.setImage("normal", &m_filterNormal[0][0], oidn::Format::Float3, filmSize[0], filmSize[1]);
		m_oidnNormalFilter.setImage("output", &m_filterNormalOutput[0][0], oidn::Format::Float3, filmSize[0], filmSize[1]);
		m_oidnNormalFilter.commit();

		m_oidnFilter = m_oidnDevice.newFilter("RT");
		m_oidnFilter.setImage("color", &m_filterColor[0][0], oidn::Format::Float3, filmSize[0], filmSize[1]);
		m_oidnFilter.setImage("albedo", &m_filterAlbedoOutput[0][0], oidn::Format::Float3, filmSize[0], filmSize[1]);
		m_oidnFilter.setImage("normal", &m_filterNormalOutput[0][0], oidn::Format::Float3, filmSize[0], filmSize[1]);
		m_oidnFilter.setImage("output", &m_filterOutput[0][0], oidn::Format::Float3, filmSize[0], filmSize[1]);
		m_oidnFilter.set("hdr", true);
		m_oidnFilter.commit();
	}

	void DenoiseBuffer::denoise(){
		m_oidnAlbedoFilter.execute();
		m_oidnNormalFilter.execute();

		m_oidnFilter.execute();
	}

	void DenoiseBuffer::storeBuffers(std::string filename){
		ref<Bitmap> bitmap_output = new Bitmap(Bitmap::ERGB, Bitmap::EFloat32, m_filmSize, 3, (uint8_t*) &m_filterOutput[0]);
		bitmap_output->setChannelNames({"output.R", "output.G", "output.B"});
		ref<Bitmap> bitmap_color  = new Bitmap(Bitmap::ERGB, Bitmap::EFloat32, m_filmSize, 3, (uint8_t*) &m_filterColor[0]);
		bitmap_color->setChannelNames({"color.R", "color.G", "color.B"});
		ref<Bitmap> bitmap_albedo = new Bitmap(Bitmap::ERGB, Bitmap::EFloat32, m_filmSize, 3, (uint8_t*) &m_filterAlbedo[0]);
		bitmap_albedo->setChannelNames({"albedo.R", "albedo.G", "albedo.B"});
		ref<Bitmap> bitmap_albedo_output = new Bitmap(Bitmap::ERGB, Bitmap::EFloat32, m_filmSize, 3, (uint8_t*) &m_filterAlbedoOutput[0]);
		bitmap_albedo_output->setChannelNames({"albedo_output.R", "albedo_output.G", "albedo_output.B"});
		ref<Bitmap> bitmap_normal = new Bitmap(Bitmap::ERGB, Bitmap::EFloat32, m_filmSize, 3, (uint8_t*) &m_filterNormal[0]);
		bitmap_normal->setChannelNames({"normal.R", "normal.G", "normal.B"});
		ref<Bitmap> bitmap_normal_output = new Bitmap(Bitmap::ERGB, Bitmap::EFloat32, m_filmSize, 3, (uint8_t*) &m_filterNormalOutput[0]);
		bitmap_normal_output->setChannelNames({"normal_output.R", "normal_output.G", "normal_output.B"});
		ref<Bitmap> bitmap = Bitmap::join(Bitmap::EMultiChannel, {bitmap_color, bitmap_albedo, bitmap_albedo_output, bitmap_normal, bitmap_normal_output, bitmap_output});
		bitmap->write(Bitmap::EOpenEXR, filename);
	}

    void ProgressiveMonteCarloIntegrator::serialize(Stream *stream, InstanceManager *manager) const{
         MonteCarloIntegrator::serialize(stream, manager);
     }

     //MTS_DECLARE_CLASS()



	bool ProgressiveMonteCarloIntegrator::preprocess(const Scene *scene, RenderQueue *queue, const RenderJob *job,
			int sceneResID, int sensorResID, int samplerResID) {

		Log(EInfo, "ProgressiveMonteCarloIntegrator::preprocess!");
		SLog(EInfo, "ProgressiveMonteCarloIntegrator: \n%s\n", this->toString().c_str());

		ref<Scheduler> sched                = Scheduler::getInstance();
		ref<Sensor> sensor                  = static_cast<Sensor *>(sched->getResource(sensorResID));
		ref<Film> film                      = sensor->getFilm();
		ref<Timer> miscTimer                = new Timer();

		m_progressionTimer 					= new Timer();
		m_denoiseTimer 						= new Timer();

		Sampler *samplerRendering   = static_cast<Sampler *>(sched->getResource(samplerResID, 0));
		m_spp = samplerRendering->getSampleCount();
		if ( m_maxRenderTime > 0.f &&
			 samplerRendering->toString().find( "IndependentSampler" ) == std::string::npos &&
			 samplerRendering->toString().find( "DeterministicSampler" ) == std::string::npos ) {
			Log( EError, "Rendering limited by time supports only an independent or deterministic sampler!");
		}

		/// Allocate pixels
		Log( EInfo, "Init new Sampler ..." );
		miscTimer->reset();
		m_filmSize = film->getSize();
		Log( EInfo, "Film Size: %d \t %d", m_filmSize[0], m_filmSize[1] );
		auto numPixels = m_filmSize[0]*m_filmSize[1];
		m_samplers.resize(numPixels);
		#pragma omp parallel for
		for (int index = 0; index < numPixels; index++){
			Point2i pix;
			pix.y = index/m_filmSize[0];
			pix.x = index % m_filmSize[0];
			m_samplers[index] = samplerRendering->clone();
			m_samplers[index]->generate(pix);
		}
		Log( EInfo, "This took %fs", miscTimer->getMilliseconds() * 1e-3f );

		m_progressionCounter = 0;
		m_denoiseProgressionCounter = 0;
		m_cancel = false;

		m_denoiseBuffer.init(m_filmSize);

		return true;
	}

    void ProgressiveMonteCarloIntegrator::postprocess(const Scene *scene, RenderQueue *queue,
        const RenderJob *job, int sceneResID, int sensorResID,
        int samplerResID){
	}

	int ProgressiveMonteCarloIntegrator::renderSamples(ref<Scheduler> sched, Scene *scene,
			RenderQueue *queue, const RenderJob *job,
			int sceneResID, int sensorResID, int samplerResID, int integratorResID){

		int sppCount = 0;

		const int processBatchSize = 1;

		int numPasses = m_spp/m_samplesPerProgression;
		int numBatches = std::ceil((Float)numPasses/(Float)processBatchSize);

		//const int processBatchSize = 128;

		for (int i= 0; i< numBatches; i++){
			m_renderProcesses.clear();
			int tmpPasses = std::min(processBatchSize,numPasses-(i*processBatchSize));
			preprogression(queue, job, sceneResID, sensorResID, samplerResID);
			for (int j = 0; j < tmpPasses; j++) {
                /**/
				ref<BlockedRenderProcess> proc = new BlockedRenderProcess(job,
							queue, scene->getBlockSize());
				proc->bindResource("integrator", integratorResID);
				proc->bindResource("scene", sceneResID);
				proc->bindResource("sensor", sensorResID);
				proc->bindResource("sampler", samplerResID);
				scene->bindUsedResources(proc);
				bindUsedResources(proc);

				m_renderProcesses.push_back(proc);
                /**/
			}

			for (int j = 0; j < tmpPasses; ++j) {
				sched->schedule(m_renderProcesses[j]);
			}


			for (int j = 0; j < tmpPasses; ++j) {
				sched->wait(m_renderProcesses[j]);
				sppCount += m_samplesPerProgression;
			}
			postprogression(queue, job, sceneResID, sensorResID, samplerResID);
			if (m_cancel)
			{
				break;
			}
        }

		return sppCount;
	}


	int ProgressiveMonteCarloIntegrator::renderTime(ref<Scheduler> sched, Scene *scene,
			RenderQueue *queue, const RenderJob *job,
			int sceneResID, int sensorResID, int samplerResID, int integratorResID){

		int sppCount = 0;

		const int processBatchSize = 128;

		bool timeIsUp = false;
		ref<Timer> renderingTimer = new Timer();
		renderingTimer->reset();
		while(!timeIsUp){
			m_renderProcesses.clear();

			for (int j = 0; j < processBatchSize; j++) {

				ref<BlockedRenderProcess> proc = new BlockedRenderProcess(job,
							queue, scene->getBlockSize());
				proc->bindResource("integrator", integratorResID);
				proc->bindResource("scene", sceneResID);
				proc->bindResource("sensor", sensorResID);
				proc->bindResource("sampler", samplerResID);
				scene->bindUsedResources(proc);
				bindUsedResources(proc);

				m_renderProcesses.push_back(proc);
			}

			for (size_t j = 0; j < processBatchSize; ++j) {
				sched->schedule(m_renderProcesses[j]);
			}


			for (size_t j = 0; j < processBatchSize; ++j) {
				sched->wait(m_renderProcesses[j]);
				sppCount += m_samplesPerProgression;
				if(renderingTimer->getSeconds() >= m_maxRenderTime){
					sched->pause();
                    //sched->stop();
					timeIsUp = true;
					break;
				}
			}

			if(timeIsUp){
				this->cancel();
				sched->start();
			}
        }

		return sppCount;
	}

	bool ProgressiveMonteCarloIntegrator::render(Scene *scene,
			RenderQueue *queue, const RenderJob *job,
			int sceneResID, int sensorResID, int samplerResID) {
 //       MonteCarloIntegrator::render(scene, queue, job, sceneResID, sensorResID, samplerResID);
        

/**/
		ref<Timer> renderingTimer = new Timer();

		ref<Scheduler> sched = Scheduler::getInstance();
		ref<Sensor> sensor = static_cast<Sensor *>(sched->getResource(sensorResID));
		ref<Film> film = sensor->getFilm();

		size_t nCores = sched->getCoreCount();
		const Sampler *sampler = static_cast<const Sampler *>(sched->getResource(samplerResID, 0));
		size_t sampleCount = sampler->getSampleCount();


		const size_t iterations     = (m_maxRenderTime > 0.f) ? 1e5 : sampler->getSampleCount();

		if ( m_maxRenderTime > 0.f &&
			 sampler->toString().find( "IndependentSampler" ) == std::string::npos &&
			 sampler->toString().find( "DeterministicSampler" ) == std::string::npos ) {
			Log( EError, "Rendering limited by time supports only an independent or deterministic sampler!");
		}



		Log(EInfo, "Starting render job (%ix%i, " SIZE_T_FMT " %s, " SIZE_T_FMT
			" %s, %s %f " SSE_STR ") ..", film->getCropSize().x, film->getCropSize().y,
			sampleCount, sampleCount == 1 ? "sample" : "samples", nCores,
			nCores == 1 ? "core" : "cores", "time: ", m_maxRenderTime);

		Thread::initializeOpenMP(nCores);


		//This is a sampling-based integrator - parallelize 
		int integratorResID = sched->registerResource(this);

		renderingTimer->reset();
		if(m_maxRenderTime > 0){
			m_spp = renderTime(sched, scene, queue,job,sceneResID,sensorResID,samplerResID, integratorResID);
		}else{
			m_spp = renderSamples(sched, scene, queue,job,sceneResID,sensorResID,samplerResID, integratorResID);
		}
		 m_renderTime = renderingTimer->getMilliseconds() * 1e-3f ;

		sched->unregisterResource(integratorResID);

        Log(EInfo, "Rendered samples: %d", m_spp);

		return true;

	}


	void ProgressiveMonteCarloIntegrator::renderBlock(const Scene *scene,
			const Sensor *sensor, Sampler *sampler, ImageBlock *block,
			const bool &stop, const std::vector< TPoint2<uint8_t> > &points) const {


		ref<Sampler> rSampler = sampler;

		Float diffScaleFactor = 1.0f /
			std::sqrt((Float) m_samplesPerProgression);
		bool needsApertureSample = sensor->needsApertureSample();
		bool needsTimeSample = sensor->needsTimeSample();

		//RadianceQueryRecord rRec(scene, sampler);
		Point2 apertureSample(0.5f);
		Float timeSample = 0.5f;
		RayDifferential sensorRay;

		block->clear();

		uint32_t queryType = RadianceQueryRecord::ESensorRay;

		if (!sensor->getFilm()->hasAlpha()) // Don't compute an alpha channel if we don't have to
			queryType &= ~RadianceQueryRecord::EOpacity;

		for (size_t i = 0; i<points.size(); ++i) {
			Point2i offset = Point2i(points[i]) + Vector2i(block->getOffset());

			int pixIdx = offset[1]*m_filmSize[0] + offset[0];
			ref<Sampler> rSampler = m_samplers[pixIdx];

			RadianceQueryRecord rRec(scene, rSampler);

			for (int j = 0; j< m_samplesPerProgression; j++) {

				if (stop)
					break;

				rRec.newQuery(queryType, sensor->getMedium());
				rRec.pixelId = pixIdx;
				rRec.primaryAlbedo.fromLinearRGB(0.f, 0.f, 0.f);
				rRec.primaryNormal = Vector3(0.f, 0.f, 0.f);
				Point2 samplePos(Point2(offset) + Vector2(rRec.nextSample2D()));

				if (needsApertureSample)
					apertureSample = rRec.nextSample2D();
				if (needsTimeSample)
					timeSample = rRec.nextSample1D();

				Spectrum spec = sensor->sampleRayDifferential(
					sensorRay, samplePos, apertureSample, timeSample);

				sensorRay.scaleDifferential(diffScaleFactor);

				Vector3 color(0.f);
				Vector3 albedo(1.f);
				Vector3 normal(0.f, 0.f, -1.f);

				spec *= this->Li(sensorRay, rRec);
				float maxSpec = spec.max();
				if (maxSpec > m_maxComponentValue) {
					spec *= m_maxComponentValue / maxSpec;
				}
				spec.toLinearRGB(color[0], color[1], color[2]);
				rRec.primaryAlbedo.toLinearRGB(albedo[0], albedo[1], albedo[2]);
				normal = rRec.primaryNormal;

				if (m_denoiseProgressionCounter > 0 && m_denoiseProgressive) {
					Vector3 output = m_denoiseBuffer.m_filterOutput[pixIdx];
					Spectrum spec;
					spec.fromLinearRGB(output.x, output.y, output.z);
					block->put(samplePos, spec, rRec.alpha);
				} else {
					block->put(samplePos, spec, rRec.alpha);
				}
				rSampler->advance();

				float alpha = 1.f / (m_samplesPerProgression * (m_denoiseProgressionCounter - 1) + j + 1);
				m_denoiseBuffer.m_filterColor[pixIdx] = (1.f - alpha) * m_denoiseBuffer.m_filterColor[pixIdx] + alpha * color;
				m_denoiseBuffer.m_filterAlbedo[pixIdx] = (1.f - alpha) * m_denoiseBuffer.m_filterAlbedo[pixIdx] + alpha * albedo;
				m_denoiseBuffer.m_filterNormal[pixIdx] = (1.f - alpha) * m_denoiseBuffer.m_filterNormal[pixIdx] + alpha * normal;
			}
		}
	}

    	std::string ProgressiveMonteCarloIntegrator::toString() const {
		std::ostringstream oss;
		oss << "Progressive[" << endl

			<< "  maxRenderTime = " << m_maxRenderTime << "," << endl
            << "  spp = " << m_spp << "," << endl
			//<< "  subIntegrator = " << indent(m_subIntegrator->toString()) << endl
			<< "]";
		return oss.str();
	}

     /// Create a integrator
     ProgressiveMonteCarloIntegrator::ProgressiveMonteCarloIntegrator(const Properties &props):MonteCarloIntegrator(props){ 
		m_samplesPerProgression = props.getInteger("samplesPerProgression", 1);
        m_maxRenderTime = props.getInteger("maxRenderTime", 0);
		m_maxComponentValue = props.getFloat("maxComponentValue", std::numeric_limits<float>::infinity());
     }
     /// Unserialize an integrator
     ProgressiveMonteCarloIntegrator::ProgressiveMonteCarloIntegrator(Stream *stream, InstanceManager *manager):MonteCarloIntegrator(stream,manager){

     }
    /// Virtual destructor
     ProgressiveMonteCarloIntegrator::~ProgressiveMonteCarloIntegrator() { }

	void ProgressiveMonteCarloIntegrator::preprogression(RenderQueue *queue, const RenderJob *job,
			int sceneResID, int sensorResID, int samplerResID) {
		m_progressionTimer->reset();
		m_progressionCounter++;
		m_denoiseProgressionCounter++;
	}

	void ProgressiveMonteCarloIntegrator::postprogression(RenderQueue *queue, const RenderJob *job,
			int sceneResID, int sensorResID, int samplerResID) {
		Log( EInfo, "Progression[%d]: %d spp took %fs", m_progressionCounter, m_samplesPerProgression, m_progressionTimer->getMilliseconds() * 1e-3f );

		int progression = m_progressionCounter - 1;
	
		if (m_denoiseProgressive) {
			m_denoiseTimer->reset();
			m_denoiseBuffer.denoise();
			Log( EInfo, "Filter[%d]: took %fs", m_progressionCounter, m_denoiseTimer->getMilliseconds() * 1e-3f );
/*
			if (m_filterDump) {
				Log( EInfo, "Filter[%d]: dumping buffers");
				m_denoiseBuffer.storeBuffers("filter_" + std::to_string(m_progressionCounter) + ".exr");
			}
*/
		}
	}

    void ProgressiveMonteCarloIntegrator::cancel() {
		m_cancel = true;
		Log(EInfo, "ProgressiveIntegrator::cancel()");
        const auto& scheduler = Scheduler::getInstance();
        for (size_t i = 0; i < m_renderProcesses.size(); ++i) {
            scheduler->cancel(m_renderProcesses[i]);
        }
	}

MTS_IMPLEMENT_CLASS(ProgressiveMonteCarloIntegrator, true, MonteCarloIntegrator)

MTS_NAMESPACE_END
