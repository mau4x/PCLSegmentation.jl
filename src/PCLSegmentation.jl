"""
The **pcl_segmentation** library contains algorithms for segmenting a point
cloud into distinct clusters.

http://docs.pointclouds.org/trunk/group__segmentation.html

## Exports

$(EXPORTS)
"""
module PCLSegmentation

export AbstractSegmentation, segment, SACSegmentation, RegionGrowingRGB,
    setOptimizeCoefficients, setModelType, setMethodType, setMaxIterations,
    setDistanceThreshold,
    setSearchMethod, setDistanceThreshold, setPointColorThreshold,
    setRegionColorThreshold, setMinClusterSize, setMaxClusterSize,
    setSmoothnessThreshold, setCurvatureThreshold,
    setSearchMethod, extract, getColoredCloud

using DocStringExtensions
using LibPCL
using PCLCommon
using PCLSearch
using PCLSampleConsensus
using PCLKDTree
using PCLOctree
using PCLKeyPoints
using Cxx
using CxxStd

const libpcl_segmentation = LibPCL.find_library_e("libpcl_segmentation")
try
    Libdl.dlopen(libpcl_segmentation, Libdl.RTLD_GLOBAL)
catch e
    warn("You might need to set DYLD_LIBRARY_PATH to load dependencies proeprty.")
    rethrow(e)
end

cxx"""
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing_rgb.h>
"""

abstract AbstractSegmentation <: PCLBase

import PCLCommon: setIndices

setIndices(s::AbstractSegmentation, indices::BoostSharedPtr) =
    icxx"$(s.handle)->setIndices($indices);"
function segment(s::AbstractSegmentation, inliers::PointIndices,
        coefficients::ModelCoefficients)
    icxx"$(s.handle)->segment(*$(inliers.handle), *$(coefficients.handle));"
end

for (name, supername) in [
    (:SACSegmentation, AbstractSegmentation),
    (:RegionGrowingRGB, AbstractSegmentation),
    ]
    cxxname = "pcl::$name"
    valname = Symbol(name, "Val")
    @eval begin
        @defpcltype $name{T} <: $supername $cxxname
        @defptrconstructor $name{T}() $cxxname
        @defconstructor $valname{T}() $cxxname
    end
end

for f in [
    :setOptimizeCoefficients,
    :setModelType,
    :setMethodType,
    :setMaxIterations,
    :setDistanceThreshold,
    ]
    body = Expr(:macrocall, Symbol("@icxx_str"), "\$(s.handle)->$f(\$arg);")
    @eval $f(s::SACSegmentation, arg) = $body
end

for f in [
    :setSearchMethod,
    :setDistanceThreshold,
    :setPointColorThreshold,
    :setRegionColorThreshold,
    :setMinClusterSize,
    :setMaxClusterSize,
    :setSmoothnessThreshold,
    :setCurvatureThreshold,
    ]
    body = Expr(:macrocall, Symbol("@icxx_str"), "\$(s.handle)->$f(\$arg);")
    @eval $f(s::RegionGrowingRGB, arg) = $body
end

setSearchMethod(s::RegionGrowingRGB, tree::PCLKDTree.KdTree) =
    setSearchMethod(s, tree.handle)

extract(s::RegionGrowingRGB, clusters::CxxStd.StdVector) =
    icxx"$(s.handle)->extract($clusters);"

getColoredCloud(s::RegionGrowingRGB) =
    PointCloud(icxx"$(s.handle)->getColoredCloud();")

end # module
