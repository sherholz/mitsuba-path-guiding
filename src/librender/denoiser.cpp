#include <mitsuba/core/bitmap.h>
#include <mitsuba/core/fresolver.h>
#include <mitsuba/core/fstream.h>
#include <mitsuba/render/denoiser.h>

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

		m_sampleCounts = std::unique_ptr<int[]>(new int[numPixels]);
		for (int i = 0; i < numPixels; i++) {
			m_sampleCounts[i] = 0;
		}

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

	void DenoiseBuffer::loadBuffers(std::string filename){
		ref<Bitmap> bitmap = new Bitmap(filename, "color");
		Vector2i size = bitmap->getSize();
		this->init(size);

		std::memcpy(&m_filterColor[0], bitmap->getData(), size[0]*size[1]*sizeof(Vector3));
		bitmap = new Bitmap(filename, "output");
		std::memcpy(&m_filterOutput[0], bitmap->getData(), size[0]*size[1]*sizeof(Vector3));
		bitmap = new Bitmap(filename, "normal");
		std::memcpy(&m_filterNormal[0], bitmap->getData(), size[0]*size[1]*sizeof(Vector3));
		bitmap = new Bitmap(filename, "normal_output");
		std::memcpy(&m_filterNormalOutput[0], bitmap->getData(), size[0]*size[1]*sizeof(Vector3));
		bitmap = new Bitmap(filename, "albedo");
		std::memcpy(&m_filterAlbedo[0], bitmap->getData(), size[0]*size[1]*sizeof(Vector3));
		bitmap = new Bitmap(filename, "albedo_output");
		std::memcpy(&m_filterAlbedoOutput[0], bitmap->getData(),size[0]*size[1]*sizeof(Vector3));
	}

	void DenoiseBuffer::add(int pixIdx, const DenoiseBuffer::Sample& sample){
		m_sampleCounts[pixIdx] +=  1;
		float alpha = 1.f / m_sampleCounts[pixIdx];
		m_filterColor[pixIdx] = (1.f - alpha) * m_filterColor[pixIdx] + alpha * Vector3(sample.color[0], sample.color[1], sample.color[2]);
		m_filterAlbedo[pixIdx] = (1.f - alpha) * m_filterAlbedo[pixIdx] + alpha  * Vector3(sample.albedo[0], sample.albedo[1], sample.albedo[2]);;
		m_filterNormal[pixIdx] = (1.f - alpha) * m_filterNormal[pixIdx] + alpha * sample.normal;
	}

	Spectrum DenoiseBuffer::get(int pixIdx) const {
		Vector3 output = m_filterOutput[pixIdx];
		Spectrum spec;
		spec.fromLinearRGB(output.x, output.y, output.z);
		return spec;
	}


MTS_NAMESPACE_END