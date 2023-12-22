#include <mitsuba/mitsuba.h>
#include <openpgl/openpgl.h>

namespace Guiding
{

enum SurfaceAdjointType{
    ESALi = 0,
    ESALo,
    ESAIrradiance
};

enum VolumeAdjointType{
    EVALi = 0,
    EVALo,
    EVAInscattered,
    EVAFluence
};

SurfaceAdjointType getSurfaceAdjointType(const std::string sa){
    SurfaceAdjointType sat = ESALo;
    if (sa == "irradiance")
        sat = ESAIrradiance;
    else if (sa == "li")
        sat = ESALi;
    else if (sa == "lo")
        sat = ESALo;
    return sat;
}

VolumeAdjointType getVolumeAdjointType(const std::string va){
    VolumeAdjointType vat = EVALo;
    if (va == "inscattered")
        vat = EVAInscattered;
    else if (va == "fluence")
        vat = EVAFluence;
    else if (va == "li")
        vat = EVALi;
    else if (va == "lo")
        vat = EVALo;
    return vat;
}

///////////////////////////////////////////////////////////////
/////  Field Configuration Parser /////////////////////////////
//////////////////////////////////////////////////////////////


void parseFieldProperties(mitsuba::Properties &props, openpgl::cpp::FieldConfig &fieldConfig)
{
    auto directionalDistribution = props.getString("directionalDistribution", "PAVMM");


    PGL_DIRECTIONAL_DISTRIBUTION_TYPE distributionType;
    if (directionalDistribution == std::string("PAVMM"))
        distributionType = PGL_DIRECTIONAL_DISTRIBUTION_PARALLAX_AWARE_VMM;
    else if (directionalDistribution == std::string("VMM"))
        distributionType = PGL_DIRECTIONAL_DISTRIBUTION_VMM;
    else if (directionalDistribution == std::string("DQT"))
        distributionType = PGL_DIRECTIONAL_DISTRIBUTION_QUADTREE;
    else
        abort();

    bool deterministic = props.getBoolean("deterministic", true);
    int samplePerLeafNode= props.getInteger("maxSamplesPerLeafNode", 32000);

    fieldConfig.Init(PGL_SPATIAL_STRUCTURE_KDTREE, distributionType, deterministic, samplePerLeafNode);
}

}