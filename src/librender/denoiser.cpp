#include <mitsuba/core/bitmap.h>
#include <mitsuba/core/fresolver.h>
#include <mitsuba/core/fstream.h>
#include <mitsuba/render/denoiser.h>

MTS_NAMESPACE_BEGIN

	void Denoiser::init(const Vector2i filmSize, const bool filterFeatures) {
		m_filterFeatures = filterFeatures;
		
		m_oidnDevice = oidn::newDevice(oidn::DeviceType::CPU);
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

		m_bufferColor = m_oidnDevice.newBuffer(numPixels * 3 * sizeof(float));
		m_bufferAlbedo = m_oidnDevice.newBuffer(numPixels * 3 * sizeof(float));		
		m_bufferNormal = m_oidnDevice.newBuffer(numPixels * 3 * sizeof(float));
		m_bufferOutput = m_oidnDevice.newBuffer(numPixels * 3 * sizeof(float));

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
			m_oidnFilter.setImage("normal", m_bufferNormal, oidn::Format::Float3, filmSize[0], filmSize[1]);
			m_oidnFilter.setImage("output", m_bufferNormalOutput, oidn::Format::Float3, filmSize[0], filmSize[1]);
			m_oidnFilter.set("cleanAux", true); // auxiliary images will be prefiltered
			m_oidnFilter.set("hdr", true);
			m_oidnFilter.commit();
		} else {
			m_oidnFilter = m_oidnDevice.newFilter("RT");
			m_oidnFilter.setImage("color", m_bufferColor, oidn::Format::Float3, filmSize[0], filmSize[1]);
			m_oidnFilter.setImage("albedo", m_bufferAlbedo, oidn::Format::Float3, filmSize[0], filmSize[1]);
			m_oidnFilter.setImage("normal", m_bufferNormal, oidn::Format::Float3, filmSize[0], filmSize[1]);
			m_oidnFilter.setImage("output", m_bufferOutput, oidn::Format::Float3, filmSize[0], filmSize[1]);

			m_oidnFilter.set("hdr", true);
			m_oidnFilter.commit();
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
		if (m_filterFeatures){
			m_oidnAlbedoFilter.execute();
			m_oidnNormalFilter.execute();
		}
		m_oidnFilter.execute();

		// Copy denoied image from OIDN buffer
		m_bufferOutput.read(0, m_filmSize[0]*m_filmSize[1]*3*sizeof(float), &m_filterOutput[0]);

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
		m_filterAlbedo[pixIdx] = (1.f - alpha) * m_filterAlbedo[pixIdx] + alpha  * Vector3(sample.albedo[0], sample.albedo[1], sample.albedo[2]);;
		m_filterNormal[pixIdx] = (1.f - alpha) * m_filterNormal[pixIdx] + alpha * sample.normal;
	}

	Spectrum Denoiser::get(int pixIdx) const {
		Spectrum spec;
		Vector3 output = m_filterOutput[pixIdx];
		spec.fromLinearRGB(output.x, output.y, output.z);
		return spec;
	}


MTS_NAMESPACE_END