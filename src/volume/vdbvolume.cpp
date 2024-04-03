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

#include <openvdb/openvdb.h>
#include <openvdb/tools/Interpolation.h>
#include <openvdb/tools/RayIntersector.h>
#include <openvdb/math/DDA.h>

#include <mitsuba/render/volume.h>
#include <mitsuba/core/fresolver.h>
#include <mitsuba/core/properties.h>

#include <memory>

MTS_NAMESPACE_BEGIN

/*!\plugin{gridvolume}{Openvdb-based volume data source}
 * \parameters{
 *     \parameter{filename}{\String}{
 *       Specifies the filename of the volume data file to be loaded
 *     }

 *     \parameter{toWorld}{\Transform}{
 *         Optional linear transformation that should be applied to the data
 *     }
 *     \parameter{min, max}{\Point}{
 *         Optional parameter that can be used to re-scale the data so that
 *         it lies in the bounding box between \code{min} and \code{max}.
 *     }
 * }
 *
 * This class implements access to memory-mapped volume data stored on a
 * 3D grid using a simple binary exchange format.
 * The format uses a little endian encoding and is specified as
 * follows:\vspace{3mm}
 *
 * \begin{center}
 * \begin{tabular}{>{\bfseries}p{2cm}p{11cm}}
 * \toprule
 * Position & Content\\
 * \midrule
 * Bytes 1-3&   ASCII Bytes '\code{V}', '\code{O}', and '\code{L}' \\
 * Byte  4&     File format version number (currently 3)\\
 * Bytes 5-8&   Encoding identifier (32-bit integer). The following
 * choices are available:
 * \begin{enumerate}[1.]
 * \item Dense \code{float32}-based representation
 * \item Dense \code{float16}-based representation (\emph{currently not supported by this implementation})
 * \item Dense \code{uint8}-based representation (The range 0..255 will be mapped to 0..1)
 * \item Dense quantized directions. The directions are stored in spherical
 * coordinates with a total storage cost of 16 bit per entry.
 * \end{enumerate}\\
 * Bytes 9-12 &  Number of cells along the X axis (32 bit integer)\\
 * Bytes 13-16 &  Number of cells along the Y axis (32 bit integer)\\
 * Bytes 17-20 &  Number of cells along the Z axis (32 bit integer)\\
 * Bytes 21-24 &  Number of channels (32 bit integer, supported values: 1 or 3)\\
 * Bytes 25-48 &  Axis-aligned bounding box of the data stored in single
 *                precision (order: xmin, ymin, zmin, xmax, ymax, zmax)\\
 * Bytes 49-*  &  Binary data of the volume stored in the specified encoding.
 *                The data are ordered so that the following C-style indexing
 *                operation makes sense after the file has been mapped into memory:\newline
 *                   \ \ \code{data[((zpos*yres + ypos)*xres + xpos)*channels + chan]}\newline
 *                where \code{(xpos, ypos, zpos, chan)} denotes the lookup location.\\
 *
 * \bottomrule
 * \end{tabular}
 * \end{center}
 *
 * Note that Mitsuba expects that entries in direction volumes are either
 * zero or valid unit vectors.
 *
 * When using this data source to represent floating point density volumes,
 * please ensure that the values are all normalized to lie in the
 * range $[0, 1]$---otherwise, the Woodcock-Tracking integration method in
 * \pluginref{heterogeneous} will produce incorrect results.
 */

class VDBDataSource : public VolumeDataSource {
//typedef openvdb::tools::VolumeRayIntersector<openvdb::FloatGrid> RayIntersector_t;
//typedef openvdb::tools::VolumeRayIntersector<openvdb::FloatGrid>::RayType RayType_t;

public:
/*
	class VDBRayIntersector: public RayIntersector{
	public:

		VDBRayIntersector(RayIntersector_t *rayIntersector, RayType_t &indexRay, Transform gridToWorld): m_sampler(rayIntersector->grid()){
			m_gridToWorld = gridToWorld;
			m_indexRay = indexRay;
			m_rayIntersectorVDB = rayIntersector;

		}

		virtual ~VDBRayIntersector(){
			delete m_rayIntersectorVDB;
		}


		virtual bool marchIdx(Float &t0, Float &t1){

			// the min max distance in index space of the OpenVDB grid
			openvdb::Real vdbT0;
			openvdb::Real vdbT1;

			bool hit = m_rayIntersectorVDB->march(vdbT0,vdbT1);

			// check if the hit does not just scratch a corner of a cell
			// therefore (vdbT1 - vdbT0) would be zero (or really close to zero)
			// TODO: choose best Epsilon (maybe OpenVDB Delta)
			if(hit && Float(vdbT1 - vdbT0)< Epsilon ){
				//SLog(EInfo, "(vdbT1 - vdbT0)< Epsilon");
				hit = m_rayIntersectorVDB->march(vdbT0,vdbT1);
			}

			t0 = Float(vdbT0);
			t1 = Float(vdbT1);
			// initialize the DDA stepper used to
			// step through each grid cell between t0 and t1
			m_dda.init(m_indexRay, vdbT0, vdbT1);
			return hit;

		}

		virtual bool march(Float &t0, Float &t1){
			Float idxT0;
			Float idxT1;

			// does one marching step in the OpenVDB volume
			// this step skips non-active tiles
			bool result = marchIdx(idxT0, idxT1);
			if (result){

				// gets the index positions in the vdb data/grid space
				openvdb::Vec3d idx0 = m_rayIntersectorVDB->getIndexPos(idxT0);
				openvdb::Vec3d idx1 = m_rayIntersectorVDB->getIndexPos(idxT1);

				openvdb::Vec3d idx = m_rayIntersectorVDB->getIndexPos(0.0f);

				// transforms the vdb data/grid positions into world space positions
				Point3 p = m_gridToWorld.transformAffine(Point3(idx[0],idx[1],idx[2]));
				Point3 p0 = m_gridToWorld.transformAffine(Point3(idx0[0],idx0[1],idx0[2]));
				Point3 p1 = m_gridToWorld.transformAffine(Point3(idx1[0],idx1[1],idx1[2]));

				// calculates the distances in world space
				t0 = (p0-p).length();
				t1 = (p1-p).length();

				IMPORTANCE_ASSERT(t0>=0.0f)
				IMPORTANCE_ASSERT(t1>0.0f);
				IMPORTANCE_ASSERT(t1>t0);


				// TODO: can't this be done once in the consturctor??

				// calculates the traveled distance in index and world space
				m_distIdx = idxT1-idxT0;
				m_dist = t1-t0;

				// the scale to transform from index to world space
				// needed by the DDA to calculate the actual step size in world space
				m_scale = m_dist/m_distIdx;

				IMPORTANCE_ASSERT(m_dist >0.f);
				IMPORTANCE_ASSERT(m_distIdx > 0.0f);
				IMPORTANCE_ASSERT(std::isfinite(m_scale));
			}
			return result;
		}


		virtual bool step(Float &value, Float &stepSize){

			// get the voxel position at the current step
			openvdb::Coord voxel = m_dda.voxel();

			//calculate the current distance and the following distance (after the next step)
			Float time = m_dda.time();
			Float next = m_dda.next();
			// calculate the step size in world space
			stepSize = std::max(Float(0.0),(next-time)*m_scale);
			// get the data from the grid at the current position
			value = Float(m_sampler.isSample(voxel));

			// do a DDA step to the next voxel
			bool result = m_dda.step();
/*
			if (result){
				IMPORTANCE_ASSERT(value >= 0.0f);
				IMPORTANCE_ASSERT(stepSize > 0.0f);
			}
*/
/*
			return result;
		}


	private:

		/// pointer to the OpenVDB RayIntersector
		RayIntersector_t *m_rayIntersectorVDB;
		/// the ray to cast in the index space of the VDB grid
		RayType_t m_indexRay;
		/// the DDA model to walk along the VDB grid between on march step
		openvdb::math::DDA<RayType_t> m_dda;
		/// The transformation to convert a point from the index/data space of the VDB grid to world space.
		Transform m_gridToWorld;

		/// the sampler to access the data of from the grid
		openvdb::tools::GridSampler<openvdb::FloatGrid, openvdb::tools::PointSampler> m_sampler;

		/// The distance traveled during the last march step in world space
		Float m_dist;
		/// The distance traveled during the last march step in index/data space
		Float m_distIdx;
		/// The scale factor to transform from index/data to world space
		Float m_scale;
	};
*/
public:

	VDBDataSource(const Properties &props)
		: VolumeDataSource(props) {
		m_volumeToWorld = props.getTransform("toWorld", Transform());

		if (props.hasProperty("min") && props.hasProperty("max")) {
			/* Optionally allow to use an AABB other than
			   the one specified by the grid file */
			m_dataAABB.min = props.getPoint("min");
			m_dataAABB.max = props.getPoint("max");
		}

		openvdb::initialize();

		m_filename = props.getString("filename");
		fs::path filename = m_filename;
		//resolves the full path of the file and checks if it even exists
		FileResolver* pathsolver = Thread::getThread()->getFileResolver();
		fs::path fsfilename = pathsolver->resolve(filename);
		loadFromFile(fsfilename);
	}

	VDBDataSource(Stream *stream, InstanceManager *manager)
			: VolumeDataSource(stream, manager) {

		m_volumeToWorld = Transform(stream);
		m_dataAABB = AABB(stream);

		fs::path filename = stream->readString();
		loadFromFile(filename);

		configure();
	}

	virtual ~VDBDataSource() {
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		VolumeDataSource::serialize(stream, manager);

		m_volumeToWorld.serialize(stream);
		m_dataAABB.serialize(stream);

		stream->writeString(m_filename);
	}

	void configure() {
		Vector extents(m_dataAABB.getExtents());
		m_worldToVolume = m_volumeToWorld.inverse();

		m_worldToGrid = Transform::translate(Vector3(Float(m_idxBBoxMin[0]),Float(m_idxBBoxMin[1]),Float(m_idxBBoxMin[2]))-Vector(0.5))*Transform::scale(Vector(
				(m_res[0]) / extents[0],
				(m_res[1]) / extents[1],
				(m_res[2]) / extents[2])
			) * Transform::translate(-Vector(m_dataAABB.min)) * m_worldToVolume;
		m_gridToWorld = m_worldToGrid.inverse();

		m_stepSize = std::numeric_limits<Float>::infinity();
		for (int i=0; i<3; ++i)
			m_stepSize = 0.5f * std::min(m_stepSize, extents[i] / (Float) (m_res[i]-1));
		m_aabb.reset();
		for (int i=0; i<8; ++i)
			m_aabb.expandBy(m_volumeToWorld(m_dataAABB.getCorner(i)));

		SLog(EInfo, "extents: %s", extents.toString().c_str());
		SLog(EInfo, "volumeToWorld: %s", m_volumeToWorld.toString().c_str());
		SLog(EInfo, "worldToGrid: %s", m_worldToGrid.toString().c_str());
		
		SLog(EInfo, "%s", toString().c_str());
		}

	void loadFromFile(const fs::path &filename) {

		// load vdb file
		std::string sfilename = filename.string();
		openvdb::io::File file(sfilename.c_str());
		file.open();

		std::ostringstream oss;

		// iterate over all grids in the file
		openvdb::GridPtrVecPtr grids = file.getGrids();
		for (openvdb::io::File::NameIterator nameIter = file.beginName(); nameIter != file.endName(); ++nameIter){
			// search for the grid containing the volume density
			if (!nameIter.gridName().compare("density")){
				oss << "VDB: found density" << endl;
				m_baseGrid = file.readGrid(nameIter.gridName());
				openvdb::Coord gridDim = m_baseGrid->evalActiveVoxelDim();
				m_res = Vector3i(gridDim.x(), gridDim.y(), gridDim.z());
				oss << "VDB: gridDim: " << gridDim.x() << "\t" << gridDim.y() << "\t" << gridDim.z() << endl;
				oss << "VDB: m_res: " << m_res[0] << "\t" << m_res[1] << "\t" << m_res[2] << endl;
				m_floatGrid = openvdb::gridPtrCast<openvdb::FloatGrid>(m_baseGrid);
				m_floatGrid->evalMinMax(m_floatMin, m_floatMax);
				m_channels = 1;

				// just plot out all meta data
				for (openvdb::MetaMap::MetaIterator iter = m_baseGrid->beginMeta();
					iter != m_baseGrid->endMeta(); ++iter)
				{
					const std::string& name = iter->first;
					openvdb::Metadata::Ptr value = iter->second;
					std::string valueAsString = value->str();
					oss << name << " = " << valueAsString << std::endl;
				}

				openvdb::CoordBBox idxBBox;
				m_floatGrid->tree().evalLeafBoundingBox(idxBBox);
				openvdb::Coord idxBBoxMin = idxBBox.min();
				openvdb::Coord idxBBoxMax = idxBBox.max();
				oss << "VDB: leafBBMin: " << idxBBoxMin[0] << "\t" << idxBBoxMin[1] << "\t" << idxBBoxMin[2] << endl;
				oss << "VDB: leafBBMax: " << idxBBoxMax[0] << "\t" << idxBBoxMax[1] << "\t" << idxBBoxMax[2] << endl;

				m_floatGrid->tree().evalActiveVoxelBoundingBox(idxBBox);
				idxBBoxMin = idxBBox.min();
				idxBBoxMax = idxBBox.max();
				oss << "VDB: activeBBMin: " << idxBBoxMin[0] << "\t" << idxBBoxMin[1] << "\t" << idxBBoxMin[2] << endl;
				oss << "VDB: activeBBMax: " << idxBBoxMax[0] << "\t" << idxBBoxMax[1] << "\t" << idxBBoxMax[2] << endl;
				oss << "VDB: floatMin: " << m_floatMin << "\tfloatMax" << m_floatMax << endl;

				m_activeBBMin = Vector3i(idxBBoxMin[0], idxBBoxMin[1], idxBBoxMin[2]);
				m_activeBBMax = Vector3i(idxBBoxMax[0], idxBBoxMax[1], idxBBoxMax[2]);

				m_idxBBoxMin = Vector3i(m_activeBBMin[0],m_activeBBMin[1],m_activeBBMin[2]);
				m_idxBBoxMax = Vector3i(m_activeBBMax[0],m_activeBBMax[1],m_activeBBMax[2]);

				openvdb::math::Transform gridTransform = m_floatGrid->transform();
				gridTransform.print(oss);

				openvdb::Vec3d vecMin = gridTransform.indexToWorld(idxBBoxMin);
				openvdb::Vec3d vecMax = gridTransform.indexToWorld(idxBBoxMax);

				openvdb::CoordBBox wsBBox;

				oss << "VDB: vecMin: " << vecMin[0] << "\t" << vecMin[1] << "\t" << vecMin[2] << endl;
				oss << "VDB: vecMax: " << vecMax[0] << "\t" << vecMax[1] << "\t" << vecMax[2] << endl;



				m_rayIntersector = new openvdb::tools::VolumeRayIntersector<openvdb::FloatGrid>(*m_floatGrid);

			}
			// print out all vdb information
			SLog(EInfo, "%s", oss.str().c_str());
		}
		file.close();
	}
/*
	RayIntersector *getRayIntersector(const Ray &ray) const{


		openvdb::tools::VolumeRayIntersector<openvdb::FloatGrid> *vbdRayIntersector = new openvdb::tools::VolumeRayIntersector<openvdb::FloatGrid>(*m_rayIntersector);

		Transform worldToGrid = Transform::translate(Vector(0.5)) * m_worldToGrid;

		Point3 idxO = worldToGrid.transformAffine(ray.o);
		Vector3 idxD;
		Float idxMinT = 0.0;
		Float idxMaxT = std::numeric_limits<Float>::infinity();

		IMPORTANCE_ASSERT(std::isfinite(ray.mint));
		IMPORTANCE_ASSERT(std::isfinite(ray.maxt));
		IMPORTANCE_ASSERT(ray.mint >=0.0f);

		if(ray.mint > 0.0){
			Point3 idxP0 = worldToGrid.transformAffine(ray(ray.mint));
			idxMinT = std::max(0.0f, (idxP0 -idxO).length());
		}

		//SLog(EInfo, "o: %s \t idxO: %s \t idxMinT: %f", ray.o.toString().c_str(), idxO.toString().c_str(), idxMinT);

		IMPORTANCE_ASSERT(!std::isinf(ray.maxt));
		if(std::isinf(ray.maxt)){
			idxD = normalize(worldToGrid(ray.d));
			SLog(EInfo, "ray.maxt == inf");

		}else{
			Point3 idxP0 = worldToGrid.transformAffine(ray(ray.maxt));
			idxD = (idxP0 -idxO);
			idxMaxT = idxD.length();
			IMPORTANCE_ASSERT(idxMaxT > 1e-5f);
			idxD /= idxMaxT;
			IMPORTANCE_ASSERT(std::isfinite(idxD[0]) && std::isfinite(idxD[1]) && std::isfinite(idxD[2]));
		}


		IMPORTANCE_ASSERT(idxMinT >= 0.0f);
		IMPORTANCE_ASSERT(std::isfinite(idxMaxT) );
		//Ray gridRay = m_worldToGrid.transformAffine(ray);
		Ray gridRay = Ray(idxO, idxD, idxMinT, idxMaxT, ray.time);
//		SLog(EInfo, "ray: %s", ray.toString().c_str());
//		SLog(EInfo, "gridRay: %s", gridRay.toString().c_str());
		openvdb::Vec3R origin(gridRay.o[0],gridRay.o[1],gridRay.o[2]);
		openvdb::Vec3R dir(gridRay.d[0],gridRay.d[1],gridRay.d[2]);
		openvdb::tools::VolumeRayIntersector<openvdb::FloatGrid>::RayType vdbRay(origin, dir);

		if (idxMaxT<=0.0){
			SLog(EInfo, "idxMaxT<=0.0: ray %s \t idxO: %f \t %f \t %f \t idxD: %f \t %f \t %f", ray.toString().c_str(), idxO[0], idxO[1], idxO[2], idxD[0], idxD[1], idxD[2]);
		}

		if(!std::isinf(ray.maxt)&& idxMaxT > 0.0){
			vdbRay.setMaxTime(idxMaxT);
		}


		bool intersects = vbdRayIntersector->setIndexRay(vdbRay);
		IMPORTANCE_ASSERT(intersects);
		if(!intersects){
			SLog(EInfo, "!!!VDB: indexRay does not intersect the BBox!!!");
			SLog(EInfo, "!!!VDB: ray: %s!!!", ray.toString().c_str());
			SLog(EInfo, "!!!VDB: vdbRay: o: %f \t %f \t %f \t d: %f \t %f \t %f \t minT: %f \t maxT: %f !!!", Float(origin[0]), Float(origin[1]), Float(origin[2]), Float(dir[0]), Float(dir[1]), Float(dir[2]), idxMinT, idxMaxT);
			std::ostringstream oss;
			//			m_dda.print(oss)

		}
		VDBRayIntersector *rayIntersector = new VDBRayIntersector(vbdRayIntersector, vdbRay, worldToGrid.inverse());

		//RayIntersector rayIntersector;
		//rayIntersector.ptr = (void*)vbdRayIntersector;
		//SLog(EInfo, "ptr: %p", rayIntersector.ptr);
		return rayIntersector;
	}
*/


	Float lookupFloat(const Point &_p) const {
		const Point p = m_worldToGrid.transformAffine(_p);

		//SLog(EInfo, "lookupFloat: idxP: %s", p.toString().c_str());
		/*
		const int x1 = math::floorToInt(p.x),
			  y1 = math::floorToInt(p.y),
			  z1 = math::floorToInt(p.z),
			  x2 = x1+1, y2 = y1+1, z2 = z1+1;

		if (x1 < m_activeBBMin.x || y1 < m_activeBBMin.y || z1 < m_activeBBMin.z || x2 >= m_res.x+m_activeBBMin.x ||
		    y2 >= m_res.y+m_activeBBMin.y || z2 >= m_res.z+m_activeBBMin.z)
			return 0;
		*/
		// Choose fractional coordinates in index space.
		const openvdb::Vec3R ijk(p.x,p.y,p.z);

		// Compute the value via trilinear (first-order) interpolation.
		//Float v1 = openvdb::tools::BoxSampler::sample(m_floatGrid->tree(), ijk);
		//Float v1 = openvdb::tools::PointSampler::sample(m_floatGrid->tree(), ijk);

		openvdb::tools::GridSampler<openvdb::FloatGrid, openvdb::tools::PointSampler> sampler(*m_floatGrid);
		Float v1 = sampler.isSample(ijk);
		// Instantiate the GridSampler template on the grid type and on a box sampler
		// for thread-safe but uncached trilinear interpolation.
		//openvdb::tools::GridSampler<openvdb::FloatGrid, openvdb::tools::BoxSampler> sampler(*m_floatGrid);
		//Float v1 = sampler.isSample(ijk);
		//Float v1 = sampler.wsSample(ijk);
		return v1;
	}

	Spectrum lookupSpectrum(const Point &_p) const {
		const Point p = m_worldToGrid.transformAffine(_p);
		const int x1 = math::floorToInt(p.x),
			  y1 = math::floorToInt(p.y),
			  z1 = math::floorToInt(p.z),
			  x2 = x1+1, y2 = y1+1, z2 = z1+1;

		if (x1 < 0 || y1 < 0 || z1 < 0 || x2 >= m_res.x ||
		    y2 >= m_res.y || z2 >= m_res.z)
			return Spectrum(0.0f);

		const Float fx = p.x - x1, fy = p.y - y1, fz = p.z - z1,
				_fx = 1.0f - fx, _fy = 1.0f - fy, _fz = 1.0f - fz;


		return Spectrum(0.0f);

	}

	Vector lookupVector(const Point &_p) const {
		const Point p = m_worldToGrid.transformAffine(_p);
		const int x1 = math::floorToInt(p.x),
			  y1 = math::floorToInt(p.y),
			  z1 = math::floorToInt(p.z),
			  x2 = x1+1, y2 = y1+1, z2 = z1+1;

		if (x1 < 0 || y1 < 0 || z1 < 0 || x2 >= m_res.x ||
		    y2 >= m_res.y || z2 >= m_res.z)
			return Vector(0.0f);

		const Float fx = p.x - x1, fy = p.y - y1, fz = p.z - z1;
		//Vector value;

		return Vector(0.0f);
	}

	bool supportsFloatLookups() const { return m_channels == 1; }
	bool supportsSpectrumLookups() const { return m_channels == 3; }
	bool supportsVectorLookups() const { return m_channels == 3; }
	Float getStepSize() const { return m_stepSize; }

	Float getMaximumFloatValue() const {
		return m_floatMax;
	}

	std::string toString() const {
		std::ostringstream oss;
		oss << "VDBVolume[" << endl
			<< "  res = " << m_res.toString() << "," << endl
			<< "  channels = " << m_channels << "," << endl
			<< "  dataAABB = " << m_dataAABB.toString() << endl
			<< "  aabb = " << m_aabb.toString() << endl
			<< "  min = " << m_floatMin << endl
			<< "  max = " << m_floatMax << endl
			<< "]";
		return oss.str();
	}

	MTS_DECLARE_CLASS()

protected:
	std::string  m_filename;
	uint8_t *m_data;
//	bool m_sendData;
//	EVolumeType m_volumeType;
	Vector3i m_res;
	int m_channels;
	Transform m_worldToGrid;
	Transform m_gridToWorld;
	Transform m_worldToVolume;
	Transform m_volumeToWorld;
	Float m_stepSize;
	AABB m_dataAABB;

	Float m_floatMin;
	Float m_floatMax;
	Vector3i m_activeBBMin;
	Vector3i m_activeBBMax;


	openvdb::GridBase::Ptr m_baseGrid;

	openvdb::FloatGrid::Ptr m_floatGrid;
	Vector3i m_idxBBoxMin;
	Vector3i m_idxBBoxMax;

	openvdb::tools::VolumeRayIntersector<openvdb::FloatGrid> *m_rayIntersector;
};

MTS_IMPLEMENT_CLASS_S(VDBDataSource, false, VolumeDataSource);
MTS_EXPORT_PLUGIN(VDBDataSource, "VDB data source");
MTS_NAMESPACE_END
