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

  Isometry3d::ConstTranslationPart(Isometry3d::*
  const_translation)() const = &Isometry3d::translation;
  py::class_<Isometry3d>(m, "Isometry3d")
      .def(py::init<Matrix4d>())
      .def("translation", const_translation);

  std::vector<double>(SignedDistanceField::*
  get_gradient_1)(int64_t, int64_t, int64_t, bool) const =
  &SignedDistanceField::GetGradient;
  std::pair<float const &, bool>(SignedDistanceField::*
  sdf_get_value_by_coordinates)(double, double, double) const = &SignedDistanceField::GetImmutable;

  std::pair<float const &, bool>(SignedDistanceField::*
  sdf_get_value_by_index)(int64_t, int64_t, int64_t) const = &SignedDistanceField::GetImmutable;
  py::class_<SignedDistanceField>(m, "SignedDistanceField")
      .def(py::init<>())
      .def("GetRawData", &SignedDistanceField::GetImmutableRawData, "Please don't mutate this")
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
      .def("GetValueByCoordinates", sdf_get_value_by_coordinates, "Please don't mutate this", py::arg("x"),
           py::arg("y"), py::arg("z"))
      .def("GetValueByIndex", sdf_get_value_by_index, "Please don't mutate this", py::arg("x_index"),
           py::arg("y_index"), py::arg("z_index"))
      .def("GetNumXCells", &SignedDistanceField::GetNumXCells)
      .def("GetNumYCells", &SignedDistanceField::GetNumYCells)
      .def("GetNumZCells", &SignedDistanceField::GetNumZCells);

  bool
  (CollisionMapGrid::*set_value_1)(int64_t, int64_t, int64_t, const COLLISION_CELL &) = &CollisionMapGrid::SetValue;

  std::pair<COLLISION_CELL const &, bool>(CollisionMapGrid::*
  get_value_by_coordinates)(double, double, double) const = &CollisionMapGrid::GetImmutable;

  std::pair<COLLISION_CELL const &, bool>(CollisionMapGrid::*
  get_value_by_index)(int64_t, int64_t, int64_t) const = &CollisionMapGrid::GetImmutable;

  py::class_<CollisionMapGrid>(m, "CollisionMapGrid")
      .def(py::init<Isometry3d, std::string, double, double, double, double, COLLISION_CELL>())
      .def("SetValue", set_value_1)
      .def("GetRawData", &CollisionMapGrid::GetImmutableRawData, "Please don't mutate this")
      .def("GetValueByCoordinates", get_value_by_coordinates, "Please don't mutate this", py::arg("x"), py::arg("y"),
           py::arg("z"))
      .def("GetValueByIndex", get_value_by_index, "Please don't mutate this", py::arg("x_index"), py::arg("y_index"),
           py::arg("z_index"))
      .def("GetNumXCells", &CollisionMapGrid::GetNumXCells)
      .def("GetNumYCells", &CollisionMapGrid::GetNumYCells)
      .def("GetNumZCells", &CollisionMapGrid::GetNumZCells)
      .def("ExtractSignedDistanceField", &CollisionMapGrid::ExtractSignedDistanceField);

  m.def("DecompressBytes", ZlibHelpers::DecompressBytes);
  m.def("DeserializeFixedSizePODFloat", arc_utilities::DeserializeFixedSizePOD<float>);
  m.def("DeserializeFixedSizePODd", arc_utilities::DeserializeFixedSizePOD<std::vector<double>>);

  using VoxelGridVecd = VoxelGrid::VoxelGrid<std::vector<double>>;
  std::pair<std::vector<double> const &, bool>(VoxelGridVecd::*
  grad_get_value_by_coordinates)(double, double, double) const = &VoxelGridVecd::GetImmutable;

  std::pair<std::vector<double> const &, bool>(VoxelGridVecd::*
  grad_get_value_by_index)(int64_t, int64_t, int64_t) const = &VoxelGridVecd::GetImmutable;

  py::class_<VoxelGridVecd>(m, "VoxelGrid")
      .def(py::init<>())
      .def("GetRawData", &VoxelGridVecd::GetImmutableRawData, "Please don't mutate this")
      .def("GetNumXCells", &VoxelGridVecd::GetNumXCells)
      .def("GetNumYCells", &VoxelGridVecd::GetNumYCells)
      .def("GetNumZCells", &VoxelGridVecd::GetNumZCells)
      .def("GetValueByCoordinates", grad_get_value_by_coordinates, "Please don't mutate this", py::arg("x"),
           py::arg("y"),
           py::arg("z"))
      .def("GetValueByIndex", grad_get_value_by_index, "Please don't mutate this", py::arg("x_index"),
           py::arg("y_index"),
           py::arg("z_index"))
      .def("SerializeSelf", &VoxelGridVecd::SerializeSelf)
      .def("DeserializeSelf", &VoxelGridVecd::DeserializeSelf, "deserialize", py::arg("buffer"),
           py::arg("current"), py::arg("value_deserializer"));

}
