#include <mitsuba/core/bitmap.h>
#include <mitsuba/core/fresolver.h>
#include <mitsuba/core/fstream.h>
#include <mitsuba/render/denoiser.h>

MTS_NAMESPACE_BEGIN

	void Denoiser::init(const Vector2i filmSize, const bool filterFeatures, const bool filterSufVolume) {
		m_filterFeatures = filterFeatures;
		m_filterSurfVol = filterSufVolume;

		m_oidnDevice = oidn::newDevice();
		m_oidnDevice.commit();

		// Check for errors
		const char* errorMessage;
		if (m_oidnDevice.getError(errorMessage) != oidn::Error::None)
			std::cout << "Error0: " << errorMessage << std::endl;
		
		m_filmSize = filmSize;
		size_t numPixels = filmSize[0] * filmSize[1];
		m_filterColor = std::unique_ptr<Vector3[]>(new Vector3[numPixels]);
		m_filterAlbedo = std::unique_ptr<Vector3[]>(new Vector3[numPixels]);
		m_filterNormal = std::unique_ptr<Vector3[]>(new Vector3[numPixels]);
		m_filterOutput = std::unique_ptr<Vector3[]>(new Vector3[numPixels]);

#if defined(DENOISE_SURF_VOL)
		m_filterSurfColor = std::unique_ptr<Vector3[]>(new Vector3[numPixels]);
		m_filterSurfAlbedo = std::unique_ptr<Vector3[]>(new Vector3[numPixels]);
		m_filterSurfNormal = std::unique_ptr<Vector3[]>(new Vector3[numPixels]);
		m_filterSurfOutput = std::unique_ptr<Vector3[]>(new Vector3[numPixels]);

		m_filterVolColor = std::unique_ptr<Vector3[]>(new Vector3[numPixels]);
		m_filterVolAlbedo = std::unique_ptr<Vector3[]>(new Vector3[numPixels]);
		m_filterVolNormal = std::unique_ptr<Vector3[]>(new Vector3[numPixels]);
		m_filterVolOutput = std::unique_ptr<Vector3[]>(new Vector3[numPixels]);
#endif

		m_bufferColor = m_oidnDevice.newBuffer(numPixels * 3 * sizeof(float));
		m_bufferAlbedo = m_oidnDevice.newBuffer(numPixels * 3 * sizeof(float));		
		m_bufferNormal = m_oidnDevice.newBuffer(numPixels * 3 * sizeof(float));
		m_bufferOutput = m_oidnDevice.newBuffer(numPixels * 3 * sizeof(float));

#if defined(DENOISE_SURF_VOL)
		m_bufferSurfColor = m_oidnDevice.newBuffer(numPixels * 3 * sizeof(float));
		m_bufferSurfAlbedo = m_oidnDevice.newBuffer(numPixels * 3 * sizeof(float));		
		m_bufferSurfNormal = m_oidnDevice.newBuffer(numPixels * 3 * sizeof(float));
		m_bufferSurfOutput = m_oidnDevice.newBuffer(numPixels * 3 * sizeof(float));

		m_bufferVolColor = m_oidnDevice.newBuffer(numPixels * 3 * sizeof(float));
		m_bufferVolAlbedo = m_oidnDevice.newBuffer(numPixels * 3 * sizeof(float));		
		m_bufferVolNormal = m_oidnDevice.newBuffer(numPixels * 3 * sizeof(float));
		m_bufferVolOutput = m_oidnDevice.newBuffer(numPixels * 3 * sizeof(float));
#endif

		m_sampleCounts = std::unique_ptr<int[]>(new int[numPixels]);
		for (int i = 0; i < numPixels; i++) {
			m_sampleCounts[i] = 0;
		}

		if (m_filterFeatures){
			m_bufferNormalOutput = m_oidnDevice.newBuffer(numPixels * 3 * sizeof(float));
			m_bufferAlbedoOutput = m_oidnDevice.newBuffer(numPixels * 3 * sizeof(float));

			m_oidnAlbedoFilter = m_oidnDevice.newFilter("RT");
			m_oidnAlbedoFilter.setImage("albedo", m_bufferAlbedo, oidn::Format::Float3, filmSize[0], filmSize[1]);
			m_oidnAlbedoFilter.setImage("output", m_bufferAlbedoOutput, oidn::Format::Float3, filmSize[0], filmSize[1]);
			m_oidnAlbedoFilter.commit();
			
			m_oidnNormalFilter = m_oidnDevice.newFilter("RT");
			m_oidnNormalFilter.setImage("normal", m_bufferNormal, oidn::Format::Float3, filmSize[0], filmSize[1]);
			m_oidnNormalFilter.setImage("output", m_bufferNormalOutput, oidn::Format::Float3, filmSize[0], filmSize[1]);
			m_oidnNormalFilter.commit();

			m_oidnFilter = m_oidnDevice.newFilter("RT");
			m_oidnFilter.setImage("color", m_bufferColor, oidn::Format::Float3, filmSize[0], filmSize[1]);
			m_oidnFilter.setImage("albedo", m_bufferAlbedoOutput, oidn::Format::Float3, filmSize[0], filmSize[1]);
			m_oidnFilter.setImage("normal", m_bufferNormalOutput, oidn::Format::Float3, filmSize[0], filmSize[1]);
			m_oidnFilter.setImage("output", m_bufferOutput, oidn::Format::Float3, filmSize[0], filmSize[1]);
			m_oidnFilter.set("cleanAux", true); // auxiliary images will be prefiltered
			m_oidnFilter.set("hdr", true);
			m_oidnFilter.commit();

#if defined(DENOISE_SURF_VOL)
			if(m_filterSurfVol) {
				m_bufferSurfNormalOutput = m_oidnDevice.newBuffer(numPixels * 3 * sizeof(float));
				m_bufferSurfAlbedoOutput = m_oidnDevice.newBuffer(numPixels * 3 * sizeof(float));

				m_oidnSurfAlbedoFilter = m_oidnDevice.newFilter("RT");
				m_oidnSurfAlbedoFilter.setImage("albedo", m_bufferSurfAlbedo, oidn::Format::Float3, filmSize[0], filmSize[1]);
				m_oidnSurfAlbedoFilter.setImage("output", m_bufferSurfAlbedoOutput, oidn::Format::Float3, filmSize[0], filmSize[1]);
				m_oidnSurfAlbedoFilter.commit();
				
				m_oidnSurfNormalFilter = m_oidnDevice.newFilter("RT");
				m_oidnSurfNormalFilter.setImage("normal", m_bufferSurfNormal, oidn::Format::Float3, filmSize[0], filmSize[1]);
				m_oidnSurfNormalFilter.setImage("output", m_bufferSurfNormalOutput, oidn::Format::Float3, filmSize[0], filmSize[1]);
				m_oidnSurfNormalFilter.commit();

				m_oidnSurfFilter = m_oidnDevice.newFilter("RT");
				m_oidnSurfFilter.setImage("color", m_bufferSurfColor, oidn::Format::Float3, filmSize[0], filmSize[1]);
				m_oidnSurfFilter.setImage("albedo", m_bufferSurfAlbedoOutput, oidn::Format::Float3, filmSize[0], filmSize[1]);
				m_oidnSurfFilter.setImage("normal", m_bufferSurfNormalOutput, oidn::Format::Float3, filmSize[0], filmSize[1]);
				m_oidnSurfFilter.setImage("output", m_bufferSurfOutput, oidn::Format::Float3, filmSize[0], filmSize[1]);
				m_oidnSurfFilter.set("cleanAux", true); // auxiliary images will be prefiltered
				m_oidnSurfFilter.set("hdr", true);
				m_oidnSurfFilter.commit();

				m_bufferVolNormalOutput = m_oidnDevice.newBuffer(numPixels * 3 * sizeof(float));
				m_bufferVolAlbedoOutput = m_oidnDevice.newBuffer(numPixels * 3 * sizeof(float));

				m_oidnVolAlbedoFilter = m_oidnDevice.newFilter("RT");
				m_oidnVolAlbedoFilter.setImage("albedo", m_bufferVolAlbedo, oidn::Format::Float3, filmSize[0], filmSize[1]);
				m_oidnVolAlbedoFilter.setImage("output", m_bufferVolAlbedoOutput, oidn::Format::Float3, filmSize[0], filmSize[1]);
				m_oidnVolAlbedoFilter.commit();
				
				m_oidnVolNormalFilter = m_oidnDevice.newFilter("RT");
				m_oidnVolNormalFilter.setImage("normal", m_bufferNormal, oidn::Format::Float3, filmSize[0], filmSize[1]);
				m_oidnVolNormalFilter.setImage("output", m_bufferNormalOutput, oidn::Format::Float3, filmSize[0], filmSize[1]);
				m_oidnVolNormalFilter.commit();

				m_oidnVolFilter = m_oidnDevice.newFilter("RT");
				m_oidnVolFilter.setImage("color", m_bufferVolColor, oidn::Format::Float3, filmSize[0], filmSize[1]);
				m_oidnVolFilter.setImage("albedo", m_bufferVolAlbedoOutput, oidn::Format::Float3, filmSize[0], filmSize[1]);
				m_oidnVolFilter.setImage("normal", m_bufferVolNormalOutput, oidn::Format::Float3, filmSize[0], filmSize[1]);
				m_oidnVolFilter.setImage("output", m_bufferVolOutput, oidn::Format::Float3, filmSize[0], filmSize[1]);
				m_oidnVolFilter.set("cleanAux", true); // auxiliary images will be prefiltered
				m_oidnVolFilter.set("hdr", true);
				m_oidnVolFilter.commit();
			}
#endif
		} else {
			m_oidnFilter = m_oidnDevice.newFilter("RT");
			m_oidnFilter.setImage("color", m_bufferColor, oidn::Format::Float3, filmSize[0], filmSize[1]);
			m_oidnFilter.setImage("albedo", m_bufferAlbedo, oidn::Format::Float3, filmSize[0], filmSize[1]);
			m_oidnFilter.setImage("normal", m_bufferNormal, oidn::Format::Float3, filmSize[0], filmSize[1]);
			m_oidnFilter.setImage("output", m_bufferOutput, oidn::Format::Float3, filmSize[0], filmSize[1]);

			m_oidnFilter.set("hdr", true);
			m_oidnFilter.commit();
#if defined(DENOISE_SURF_VOL)
			m_oidnSurfFilter = m_oidnDevice.newFilter("RT");
			m_oidnSurfFilter.setImage("color", m_bufferSurfColor, oidn::Format::Float3, filmSize[0], filmSize[1]);
			m_oidnSurfFilter.setImage("albedo", m_bufferSurfAlbedo, oidn::Format::Float3, filmSize[0], filmSize[1]);
			m_oidnSurfFilter.setImage("normal", m_bufferSurfNormal, oidn::Format::Float3, filmSize[0], filmSize[1]);
			m_oidnSurfFilter.setImage("output", m_bufferSurfOutput, oidn::Format::Float3, filmSize[0], filmSize[1]);

			m_oidnSurfFilter.set("hdr", true);
			m_oidnSurfFilter.commit();

			m_oidnVolFilter = m_oidnDevice.newFilter("RT");
			m_oidnVolFilter.setImage("color", m_bufferVolColor, oidn::Format::Float3, filmSize[0], filmSize[1]);
			m_oidnVolFilter.setImage("albedo", m_bufferVolAlbedo, oidn::Format::Float3, filmSize[0], filmSize[1]);
			m_oidnVolFilter.setImage("normal", m_bufferVolNormal, oidn::Format::Float3, filmSize[0], filmSize[1]);
			m_oidnVolFilter.setImage("output", m_bufferVolOutput, oidn::Format::Float3, filmSize[0], filmSize[1]);

			m_oidnVolFilter.set("hdr", true);
			m_oidnVolFilter.commit();
#endif
		}
		// Check for errors
		if (m_oidnDevice.getError(errorMessage) != oidn::Error::None)
			std::cout << "Error1: " << errorMessage << std::endl;
	}

	void Denoiser::denoise(){
		// Copy data to OIDN buffers
		m_bufferColor.write(0, m_filmSize[0]*m_filmSize[1]*3*sizeof(float), &m_filterColor[0]);
		m_bufferNormal.write(0, m_filmSize[0]*m_filmSize[1]*3*sizeof(float), &m_filterNormal[0]);
		m_bufferAlbedo.write(0, m_filmSize[0]*m_filmSize[1]*3*sizeof(float), &m_filterAlbedo[0]);
#if defined(DENOISE_SURF_VOL)
		if(m_filterSurfVol) {
			m_bufferSurfColor.write(0, m_filmSize[0]*m_filmSize[1]*3*sizeof(float), &m_filterSurfColor[0]);
			m_bufferSurfNormal.write(0, m_filmSize[0]*m_filmSize[1]*3*sizeof(float), &m_filterSurfNormal[0]);
			m_bufferSurfAlbedo.write(0, m_filmSize[0]*m_filmSize[1]*3*sizeof(float), &m_filterSurfAlbedo[0]);

			m_bufferVolColor.write(0, m_filmSize[0]*m_filmSize[1]*3*sizeof(float), &m_filterVolColor[0]);
			m_bufferVolNormal.write(0, m_filmSize[0]*m_filmSize[1]*3*sizeof(float), &m_filterVolNormal[0]);
			m_bufferVolAlbedo.write(0, m_filmSize[0]*m_filmSize[1]*3*sizeof(float), &m_filterVolAlbedo[0]);
		}
#endif
		if (m_filterFeatures){
			m_oidnAlbedoFilter.execute();
			m_oidnNormalFilter.execute();
#if defined(DENOISE_SURF_VOL)
			if(m_filterSurfVol) {
				m_oidnSurfAlbedoFilter.execute();
				m_oidnSurfNormalFilter.execute();
				m_oidnVolAlbedoFilter.execute();
				m_oidnVolNormalFilter.execute();
			}
#endif
		}
		m_oidnFilter.execute();
#if defined(DENOISE_SURF_VOL)
		if(m_filterSurfVol) {
			m_oidnSurfFilter.execute();
			m_oidnVolFilter.execute();
		}
#endif
		// Copy denoied image from OIDN buffer
		m_bufferOutput.read(0, m_filmSize[0]*m_filmSize[1]*3*sizeof(float), &m_filterOutput[0]);
#if defined(DENOISE_SURF_VOL)
		if(m_filterSurfVol) {
			m_bufferSurfOutput.read(0, m_filmSize[0]*m_filmSize[1]*3*sizeof(float), &m_filterSurfOutput[0]);
			m_bufferVolOutput.read(0, m_filmSize[0]*m_filmSize[1]*3*sizeof(float), &m_filterVolOutput[0]);
		}
#endif
		// Check for errors
		const char* errorMessage;
		if (m_oidnDevice.getError(errorMessage) != oidn::Error::None)
			std::cout << "Error: " << errorMessage << std::endl;
	}

	void Denoiser::storeBuffers(std::string filename){
		ref<Bitmap> bitmap_output = new Bitmap(Bitmap::ERGB, Bitmap::EFloat32, m_filmSize, 3, (uint8_t*) m_bufferOutput.getData());
		bitmap_output->setChannelNames({"output.R", "output.G", "output.B"});
		ref<Bitmap> bitmap_color  = new Bitmap(Bitmap::ERGB, Bitmap::EFloat32, m_filmSize, 3, (uint8_t*) m_bufferColor.getData());
		bitmap_color->setChannelNames({"color.R", "color.G", "color.B"});
		ref<Bitmap> bitmap_albedo = new Bitmap(Bitmap::ERGB, Bitmap::EFloat32, m_filmSize, 3, (uint8_t*) m_bufferAlbedo.getData());
		bitmap_albedo->setChannelNames({"albedo.R", "albedo.G", "albedo.B"});
		ref<Bitmap> bitmap_normal = new Bitmap(Bitmap::ERGB, Bitmap::EFloat32, m_filmSize, 3, (uint8_t*) m_bufferNormal.getData());
		bitmap_normal->setChannelNames({"normal.R", "normal.G", "normal.B"});

		if (m_filterFeatures){
			ref<Bitmap> bitmap_albedo_output = new Bitmap(Bitmap::ERGB, Bitmap::EFloat32, m_filmSize, 3, (uint8_t*) m_bufferAlbedoOutput.getData());
			bitmap_albedo_output->setChannelNames({"albedo_output.R", "albedo_output.G", "albedo_output.B"});
			ref<Bitmap> bitmap_normal_output = new Bitmap(Bitmap::ERGB, Bitmap::EFloat32, m_filmSize, 3, (uint8_t*) m_bufferNormalOutput.getData());
			bitmap_normal_output->setChannelNames({"normal_output.R", "normal_output.G", "normal_output.B"});
			ref<Bitmap> bitmap = Bitmap::join(Bitmap::EMultiChannel, {bitmap_color, bitmap_albedo, bitmap_albedo_output, bitmap_normal, bitmap_normal_output, bitmap_output});
			bitmap->write(Bitmap::EOpenEXR, filename);
		} else {
			ref<Bitmap> bitmap = Bitmap::join(Bitmap::EMultiChannel, {bitmap_color, bitmap_albedo, bitmap_normal, bitmap_output});
			bitmap->write(Bitmap::EOpenEXR, filename);			
		}
	}

	void Denoiser::loadBuffers(std::string filename){
		ref<Bitmap> bitmap = new Bitmap(filename, "color");
		Vector2i size = bitmap->getSize();
		this->init(size);

		m_bufferColor.write(0, size[0]*size[1]*3*sizeof(float), bitmap->getData());
		std::memcpy(&m_filterColor[0], bitmap->getData(), m_filmSize[0]*m_filmSize[1]*sizeof(Vector3));
		bitmap = new Bitmap(filename, "output");
		m_bufferOutput.write(0, size[0]*size[1]*3*sizeof(float), bitmap->getData());
		std::memcpy(&m_filterOutput[0], bitmap->getData(), m_filmSize[0]*m_filmSize[1]*sizeof(Vector3));
		bitmap = new Bitmap(filename, "normal");
		m_bufferNormal.write(0, size[0]*size[1]*3*sizeof(float), bitmap->getData());
		std::memcpy(&m_filterNormal[0], bitmap->getData(), m_filmSize[0]*m_filmSize[1]*sizeof(Vector3));
		bitmap = new Bitmap(filename, "normal_output");
		m_bufferNormalOutput.write(0, size[0]*size[1]*3*sizeof(float), bitmap->getData());
		bitmap = new Bitmap(filename, "albedo");
		m_bufferAlbedo.write(0, size[0]*size[1]*3*sizeof(float), bitmap->getData());
		std::memcpy(&m_filterAlbedo[0], bitmap->getData(), m_filmSize[0]*m_filmSize[1]*sizeof(Vector3));
		bitmap = new Bitmap(filename, "albedo_output");
		m_bufferAlbedoOutput.write(0, size[0]*size[1]*3*sizeof(float), bitmap->getData());
	}

	void Denoiser::add(int pixIdx, const Denoiser::Sample& sample){
		m_sampleCounts[pixIdx] +=  1;
		float alpha = 1.f / m_sampleCounts[pixIdx];
		m_filterColor[pixIdx] = (1.f - alpha) * m_filterColor[pixIdx] + alpha * Vector3(sample.color[0], sample.color[1], sample.color[2]);
		m_filterAlbedo[pixIdx] = (1.f - alpha) * m_filterAlbedo[pixIdx] + alpha  * Vector3(sample.albedo[0], sample.albedo[1], sample.albedo[2]);
		m_filterNormal[pixIdx] = (1.f - alpha) * m_filterNormal[pixIdx] + alpha * sample.normal;
#if defined(DENOISE_SURF_VOL)
		if (m_filterFeatures){
			if(sample.isSurface) {
				m_filterSurfColor[pixIdx] = (1.f - alpha) * m_filterColor[pixIdx] + alpha * Vector3(sample.color[0], sample.color[1], sample.color[2]);
				m_filterSurfAlbedo[pixIdx] = (1.f - alpha) * m_filterAlbedo[pixIdx] + alpha  * Vector3(sample.albedo[0], sample.albedo[1], sample.albedo[2]);
				m_filterSurfNormal[pixIdx] = (1.f - alpha) * m_filterNormal[pixIdx] + alpha * sample.normal;
			} else {
				m_filterVolColor[pixIdx] = (1.f - alpha) * m_filterColor[pixIdx] + alpha * Vector3(sample.color[0], sample.color[1], sample.color[2]);
				m_filterVolAlbedo[pixIdx] = (1.f - alpha) * m_filterAlbedo[pixIdx] + alpha  * Vector3(sample.albedo[0], sample.albedo[1], sample.albedo[2]);
				m_filterVolNormal[pixIdx] = (1.f - alpha) * m_filterNormal[pixIdx] + alpha * sample.normal;
			}
		}
#endif
	}

	Spectrum Denoiser::get(int pixIdx) const {
		Spectrum spec;
		Vector3 output = m_filterOutput[pixIdx];
		spec.fromLinearRGB(output.x, output.y, output.z);
		return spec;
	}

#if defined(DENOISE_SURF_VOL)
	Spectrum Denoiser::getSurf(int pixIdx) const {
		Spectrum spec;
		Vector3 output = m_filterSurfOutput[pixIdx];
		spec.fromLinearRGB(output.x, output.y, output.z);
		return spec;
	}

	Spectrum Denoiser::getVol(int pixIdx) const {
		Spectrum spec;
		Vector3 output = m_filterVolOutput[pixIdx];
		spec.fromLinearRGB(output.x, output.y, output.z);
		return spec;
	}
#endif


MTS_NAMESPACE_END