#include <mitsuba/core/plugin.h>
#include <mitsuba/core/bitmap.h>
#include <mitsuba/core/fstream.h>
#include <mitsuba/render/util.h>

#include <iostream>
#include <fstream>

MTS_NAMESPACE_BEGIN

class Serialized2Obj : public Utility {
public:
    int run(int argc, char **argv) {
        std::cout << argc << std::endl;
        if (argc != 4) {
            cout << "Exports one shape of a Mitsuba serialized shape into an OBJ file. " << endl;
            cout << "Syntax: mtsutil serialized2obj <serialized shape.serialized> <index 1> <obj object1.obj>" << endl;
            return -1;
        }

        std::cout << "SerializedFile: " << argv[1] << std::endl;
        std::cout << "shapeIndex: " << argv[2] << std::endl;
        std::cout << "OBJFile: " << argv[3] << std::endl;

        char *end_ptr = NULL;
        int shapeIndex = (int) strtol(argv[2], &end_ptr, 10);
        if (*end_ptr != '\0')
            SLog(EError, "Could not parse integer value");

        std::ofstream objfile;
        objfile.open(argv[3]);
        Properties propsSerialized("serialized");
        propsSerialized.setString("filename", argv[1]);
        propsSerialized.setInteger("shapeIndex", shapeIndex);

        ref<Shape> shape = static_cast<Shape *>(PluginManager::getInstance()->createObject(MTS_CLASS(Shape), propsSerialized));
        std::cout << "Shape: " << shape->toString() << "\tisCompound: " << shape->isCompound()<< std::endl;
        ref<TriMesh> trimesh = shape->createTriMesh();
        std::string name = trimesh->getName();

        int nVertex = trimesh->getVertexCount();
        int nTriangles = trimesh->getTriangleCount();
        int nNormals = 0;

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
                    objfile << "f " << t.idx[0]+1 << "/" << t.idx[0]+1 << "/" << t.idx[0]+1 << " ";
                    objfile << t.idx[1]+1 << "/" << t.idx[1]+1 << "/" << t.idx[1]+1 << " ";
                    objfile << t.idx[2]+1 << "/" << t.idx[2]+1 << "/" << t.idx[2]+1 << std::endl;
                }
                else
                {
                    objfile << "f " << t.idx[0]+1 << "//" << t.idx[0]+1 << " ";
                    objfile << t.idx[1]+1 << "//" << t.idx[1]+1 << " ";
                    objfile << t.idx[2]+1 << "//" << t.idx[2]+1 << std::endl;
                }
            }
            else
            {
                if(hasTexcoords)
                {
                    objfile << "f " << t.idx[0]+1 << "/" << t.idx[0]+1 << " ";
                    objfile << t.idx[1]+1 << "/" << t.idx[1]+1 << " ";
                    objfile << t.idx[2]+1 << "/" << t.idx[2]+1 << std::endl;
                }
                else
                {
                    objfile << "f " << t.idx[0]+1 <<  " ";
                    objfile << t.idx[1]+1 << " ";
                    objfile << t.idx[2]+1 << std::endl;
                }
            }
        }

        objfile.close();
        return 0;
    }



    MTS_DECLARE_UTILITY()
};


MTS_EXPORT_UTILITY(Serialized2Obj, "Converts a serialized object into an OBJ object.")
MTS_NAMESPACE_END