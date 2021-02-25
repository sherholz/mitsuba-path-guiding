#include <mitsuba/core/plugin.h>
#include <mitsuba/core/bitmap.h>
#include <mitsuba/core/fstream.h>
#include <mitsuba/core/fresolver.h>
#include <mitsuba/render/util.h>
#include <mitsuba/render/shape.h>
#include <boost/algorithm/string.hpp>

#include <iostream>
#include <fstream>

MTS_NAMESPACE_BEGIN

class Scene2Obj : public Utility {
public:

    void appendShapeToOBJFile(std::ofstream &objfile, Shape *shape, const int &shapeIdx, uint32_t &vertexCount, uint32_t &normalCount, uint32_t &stCount) const
    {
        ref<TriMesh> trimesh = shape->createTriMesh();
        std::string name = shape->getID();

        int nVertex = trimesh->getVertexCount();
        int nTriangles = trimesh->getTriangleCount();
        int nNormals = 0;

        std::cout  << "o " << name << std::endl;
        objfile << "o " << name <<std::endl;


        bool hasTexcoords = trimesh->hasVertexTexcoords();
        bool hasNormals = trimesh->hasVertexNormals();
        if (hasNormals)
        {
            nNormals = nVertex;
        }

        Point *vertices = trimesh->getVertexPositions();
        for(int i =0; i<nVertex ;i++)
        {
            Point v = vertices[i];
            objfile << "v " << v[0] << "\t" << v[1] << "\t" << v[2] << std::endl;
        }
        if(hasNormals)
        {
            Vector3 *normals = trimesh->getVertexNormals();
            for(int i =0; i<nNormals ;i++)
            {
                Vector n = normals[i];
                objfile << "vn " << n[0] << "\t" << n[1] << "\t" << n[2] << std::endl;
            }
        }

        if (hasTexcoords)
        {
            Point2 *texcoords = trimesh->getVertexTexcoords();
            for(int i =0; i<nVertex ;i++)
            {
                Point2 st = texcoords[i];
                objfile << "vt " << st[0] << "\t" << st[1] << std::endl;
            }
        }

        Triangle *triangles = trimesh->getTriangles();
        for(int i =0; i<nTriangles ;i++)
        {
            Triangle t=triangles[i];
            if(hasNormals)
            {
                if(hasTexcoords)
                {
                    objfile << "f " << vertexCount+t.idx[0]+1 << "/" << stCount+t.idx[0]+1 << "/" << normalCount+t.idx[0]+1 << " ";
                    objfile << vertexCount+t.idx[1]+1 << "/" << stCount+t.idx[1]+1 << "/" << normalCount+t.idx[1]+1 << " ";
                    objfile << vertexCount+t.idx[2]+1 << "/" << stCount+t.idx[2]+1 << "/" << normalCount+t.idx[2]+1 << std::endl;
                }
                else
                {
                    objfile << "f " << vertexCount+t.idx[0]+1 << "//" << normalCount+t.idx[0]+1 << " ";
                    objfile << vertexCount+t.idx[1]+1 << "//" << normalCount+t.idx[1]+1 << " ";
                    objfile << vertexCount+t.idx[2]+1 << "//" << normalCount+t.idx[2]+1 << std::endl;
                }
            }
            else
            {
                if(hasTexcoords)
                {
                    objfile << "f " << vertexCount+t.idx[0]+1 << "/" << stCount+t.idx[0]+1 << " ";
                    objfile << vertexCount+t.idx[1]+1 << "/" << stCount+t.idx[1]+1 << " ";
                    objfile << vertexCount+t.idx[2]+1 << "/" << stCount+t.idx[2]+1 << std::endl;
                }
                else
                {
                    objfile << "f " << vertexCount+t.idx[0]+1 <<  " ";
                    objfile << vertexCount+t.idx[1]+1 << " ";
                    objfile << vertexCount+t.idx[2]+1 << std::endl;
                }
            }
        }
        vertexCount += nVertex;
        stCount+= hasTexcoords ? nVertex : 0;
        normalCount+=nNormals;
    }

    int run(int argc, char **argv) {
        std::cout << argc << std::endl;
        if (argc != 3) {
            cout << "Exports the geometry of a Mitsuba scene into an OBJ file where each shape of the scene will be a separate group/object." << endl;
            cout << "Syntax: mtsutil scene2obj <scene cbox.xml> <obj cbox.obj>" << endl;
            return -1;
        }

        std::cout << "SceneFile: " << argv[1] << std::endl;
        std::cout << "OBJFile: " << argv[2] << std::endl;


        ref<FileResolver> fileResolver = Thread::getThread()->getFileResolver();

        ref<Scene> scene;

        std::string lowercase = boost::to_lower_copy(std::string(argv[1]));
        if (boost::ends_with(lowercase, ".xml")) {
            fs::path
                filename = fileResolver->resolve(argv[1]),
                filePath = fs::absolute(filename).parent_path(),
                baseName = filename.stem();
            ref<FileResolver> frClone = fileResolver->clone();
            frClone->prependPath(filePath);
            Thread::getThread()->setFileResolver(frClone);
            scene = loadScene(argv[optind]);

            std::ofstream objfile;
            objfile.open(argv[2]);
            ref_vector<Shape> shapes = scene->getShapes();
            uint32_t vertexCount = 0;
            uint32_t normalCount = 0;
            uint32_t stCount = 0;
            for (size_t i =0; i< shapes.size(); i++)
            {
                ref<Shape> shape = shapes[i];
                appendShapeToOBJFile(objfile, shape.get(), i, vertexCount, normalCount,stCount);
            }

        }
        return 0;
    }

    MTS_DECLARE_UTILITY()
};


MTS_EXPORT_UTILITY(Scene2Obj, "Extracts all shapes of a scene to an objs using groups for the individual scene objects.")
MTS_NAMESPACE_END