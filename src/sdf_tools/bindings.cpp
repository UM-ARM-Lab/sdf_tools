#include <cstdint>

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "arc_utilities/zlib_helpers.hpp"
#include "sdf_tools/collision_map.hpp"

namespace py = pybind11;
using namespace Eigen;
using namespace sdf_tools;

PYBIND11_MODULE(pysdf_tools, m)
{
  Py_Initialize();

  // Bindings
  py::class_<COLLISION_CELL>(m, "COLLISION_CELL")
      .def(py::init<float>())
      .def(py::init<float, uint32_t>())
      .def_readwrite("occupancy", &COLLISION_CELL::occupancy)
      .def_readwrite("component", &COLLISION_CELL::component);

  Isometry3d::ConstTranslationPart (Isometry3d::*const_translation)() const = &Isometry3d::translation;
  py::class_<Isometry3d>(m, "Isometry3d")
      .def(py::init<Matrix4d>())
      .def("translation", const_translation);

  using VoxelGridVecd = VoxelGrid::VoxelGrid<std::vector<double>>;
  py::class_<VoxelGridVecd>(m, "VoxelGrid")
      .def("GetRawData", &VoxelGridVecd::GetImmutableRawData, "Please done mutate this")
      .def("GetNumXCells", &VoxelGridVecd::GetNumXCells)
      .def("GetNumYCells", &VoxelGridVecd::GetNumYCells)
      .def("GetNumZCells", &VoxelGridVecd::GetNumZCells);

  std::vector<double> (SignedDistanceField::*get_gradient_1)(int64_t, int64_t, int64_t, bool) const =
      &SignedDistanceField::GetGradient;
  py::class_<SignedDistanceField>(m, "SignedDistanceField")
      .def(py::init<>())
      .def("GetRawData", &SignedDistanceField::GetImmutableRawData, "Please done mutate this")
      .def("GetFullGradient", &SignedDistanceField::GetFullGradient)
      .def("GetGradient", get_gradient_1, "get the gradient based on index", py::arg("x_index"), py::arg("y_index"),
           py::arg("z_index"), py::arg("enable_edge_gradients") = false)
      .def("GetMessageRepresentation", &SignedDistanceField::GetMessageRepresentation)
      .def("LoadFromMessageRepresentation", &SignedDistanceField::LoadFromMessageRepresentation)
      .def("SaveToFile", &SignedDistanceField::SaveToFile)
      .def("LoadFromFile", &SignedDistanceField::LoadFromFile)
      .def("SerializeSelf", &SignedDistanceField::SerializeSelf)
      .def("DeserializeSelf", &SignedDistanceField::DeserializeSelf, "deserialize", py::arg("buffer"),
           py::arg("current"), py::arg("value_deserializer"))
      .def("GetOriginTransform", &SignedDistanceField::GetOriginTransform)
      .def("GetNumXCells", &SignedDistanceField::GetNumXCells)
      .def("GetNumYCells", &SignedDistanceField::GetNumYCells)
      .def("GetNumZCells", &SignedDistanceField::GetNumZCells);

  bool (CollisionMapGrid::*set_value_1)(int64_t, int64_t, int64_t, const COLLISION_CELL&) = &CollisionMapGrid::SetValue;
  py::class_<CollisionMapGrid>(m, "CollisionMapGrid")
      .def(py::init<Isometry3d, std::string, double, double, double, double, COLLISION_CELL>())
      .def("SetValue", set_value_1)
      .def("GetNumXCells", &CollisionMapGrid::GetNumXCells)
      .def("GetNumYCells", &CollisionMapGrid::GetNumYCells)
      .def("GetNumZCells", &CollisionMapGrid::GetNumZCells)
      .def("ExtractSignedDistanceField", &CollisionMapGrid::ExtractSignedDistanceField);

  m.def("DecompressBytes", ZlibHelpers::DecompressBytes);
  m.def("DeserializeFixedSizePODFloat", arc_utilities::DeserializeFixedSizePOD<float>);
}
