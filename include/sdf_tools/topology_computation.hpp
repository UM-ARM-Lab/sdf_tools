#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <vector>
#include <list>
#include <unordered_map>
#include <functional>
#include <arc_utilities/voxel_grid.hpp>

#ifndef TOPOLOGY_COMPUTATION_HPP
#define TOPOLOGY_COMPUTATION_HPP

#define ENABLE_UNORDERED_MAP_SIZE_HINTS

namespace topology_computation
{
using VoxelGrid::GRID_INDEX;

template<typename T, typename BackingStore=std::vector<T>>
int64_t MarkConnectedComponent(
    const VoxelGrid::VoxelGrid<T, BackingStore>& source_grid,
    const std::function<bool(const GRID_INDEX&,
                             const GRID_INDEX&)>& are_connected_fn,
    const std::function<uint32_t(const GRID_INDEX&)>& get_component_fn,
    const std::function<void(const GRID_INDEX&,
                             const uint32_t)>& mark_component_fn,
    const GRID_INDEX& start_index,
    const uint32_t connected_component)
{
  // Make the working queue
  std::list<GRID_INDEX> working_queue;
  // Make a hash table to store queued indices (so we don't repeat work)
  // Let's provide an hint at the size of hashmap we'll need, since this will
  // reduce the need to resize & rehash. We're going to assume that connected
  // components, in general, will take ~1/16 of the grid in size
  // which means, with 2 cells/hash bucket, we'll initialize to grid size/32
#ifdef ENABLE_UNORDERED_MAP_SIZE_HINTS
  const size_t queued_hashtable_size_hint
      = source_grid.GetImmutableRawData().size() / 32;
  std::unordered_map<GRID_INDEX, int8_t> queued_hashtable(
        queued_hashtable_size_hint);
#else
  std::unordered_map<GRID_INDEX, int8_t> queued_hashtable;
#endif
  // Enqueue the starting index
  working_queue.push_back(start_index);
  queued_hashtable[start_index] = 1;
  // Work
  int64_t marked_cells = 0;
  while (working_queue.size() > 0)
  {
    // Get an item off the queue to work with
    const GRID_INDEX current_index = working_queue.front();
    working_queue.pop_front();
    // Mark the connected component
    mark_component_fn(current_index, connected_component);
    // Go through the possible neighbors and enqueue as needed
    // Since there are only six cases
    // (voxels must share a face to be considered connected),
    // we handle each explicitly
    const std::vector<GRID_INDEX> neighbor_indices
        = {GRID_INDEX(current_index.x - 1, current_index.y, current_index.z),
           GRID_INDEX(current_index.x + 1, current_index.y, current_index.z),
           GRID_INDEX(current_index.x, current_index.y - 1, current_index.z),
           GRID_INDEX(current_index.x, current_index.y + 1, current_index.z),
           GRID_INDEX(current_index.x, current_index.y, current_index.z - 1),
           GRID_INDEX(current_index.x, current_index.y, current_index.z + 1)};
    for (size_t idx = 0; idx < neighbor_indices.size(); idx++)
    {
      const GRID_INDEX& neighbor_index = neighbor_indices[idx];
      if (get_component_fn(neighbor_index) == 0)
      {
        if (are_connected_fn(current_index, neighbor_index))
        {
          if (queued_hashtable[neighbor_index] <= 0)
          {
            queued_hashtable[neighbor_index] = 1;
            working_queue.push_back(neighbor_index);
          }
        }
      }
    }
  }
  return marked_cells;
}

template<typename T, typename BackingStore=std::vector<T>>
uint32_t ComputeConnectedComponents(
    const VoxelGrid::VoxelGrid<T, BackingStore>& source_grid,
    const std::function<bool(const GRID_INDEX&,
                             const GRID_INDEX&)>& are_connected_fn,
    const std::function<int64_t(const GRID_INDEX&)>& get_component_fn,
    const std::function<void(const GRID_INDEX&,
                             const uint32_t)>& mark_component_fn)
{
  // Reset components first
  for (int64_t x_index = 0; x_index < source_grid.GetNumXCells(); x_index++)
  {
    for (int64_t y_index = 0; y_index < source_grid.GetNumYCells(); y_index++)
    {
      for (int64_t z_index = 0; z_index < source_grid.GetNumZCells(); z_index++)
      {
        const GRID_INDEX index(x_index, y_index, z_index);
        mark_component_fn(index, 0u);
      }
    }
  }
  // Mark the components
  int64_t total_cells = source_grid.GetNumXCells()
                        * source_grid.GetNumYCells()
                        * source_grid.GetNumZCells();
  int64_t marked_cells = 0;
  uint32_t connected_components = 0;
  // Sweep through the grid
  for (int64_t x_index = 0; x_index < source_grid.GetNumXCells(); x_index++)
  {
    for (int64_t y_index = 0; y_index < source_grid.GetNumYCells(); y_index++)
    {
      for (int64_t z_index = 0; z_index < source_grid.GetNumZCells(); z_index++)
      {
        const GRID_INDEX index(x_index, y_index, z_index);
        // Check if the cell has already been marked, if so, ignore
        if (get_component_fn(index) == 0)
        {
          // Start marking a new connected component
          connected_components++;
          const int64_t cells_marked
              = MarkConnectedComponent(source_grid,
                                       are_connected_fn,
                                       get_component_fn,
                                       mark_component_fn,
                                       index, connected_components);
          marked_cells += cells_marked;
          // Short-circuit if we've marked everything
          if (marked_cells == total_cells)
          {
            return connected_components;
          }
        }
      }
    }
  }
  return connected_components;
}

inline int32_t ComputeConnectivityOfSurfaceVertices(
    const std::unordered_map<GRID_INDEX, uint8_t>&
      surface_vertex_connectivity)
{
  int32_t connected_components = 0;
  int64_t processed_vertices = 0;
  // Compute a hint for initial vertex components hashmap size
  // real # of surface vertices
  // surface vertices
  size_t vertex_components_size_hint = surface_vertex_connectivity.size();
  std::unordered_map<GRID_INDEX, int32_t>
      vertex_components(vertex_components_size_hint);
  // Iterate through the vertices
  std::unordered_map<GRID_INDEX, uint8_t>::const_iterator
      surface_vertices_itr;
  for (surface_vertices_itr = surface_vertex_connectivity.begin();
       surface_vertices_itr != surface_vertex_connectivity.end();
       ++surface_vertices_itr)
  {
    const GRID_INDEX key = surface_vertices_itr->first;
    //const uint8_t& connectivity = surface_vertices_itr->second.second;
    // First, check if the vertex has already been marked
    if (vertex_components[key] > 0)
    {
      continue;
    }
    else
    {
      // If not, we start marking a new connected component
      connected_components++;
      // Make the working queue
      std::list<GRID_INDEX> working_queue;
      // Make a hash table to store queued indices (so we don't repeat work)
      // Compute a hint for initial queued hashtable hashmap size
      // If we assume that most object surfaces are, in fact, intact,
      // then the first (and only) queued_hashtable will need to store an entry
      // for every vertex on the surface.
      size_t queued_hashtable_size_hint = surface_vertex_connectivity.size();
      std::unordered_map<GRID_INDEX, int8_t>
          queued_hashtable(queued_hashtable_size_hint);
      // Add the current point
      working_queue.push_back(key);
      queued_hashtable[key] = 1;
      // Keep track of the number of vertices we've processed
      int64_t component_processed_vertices = 0;
      // Loop from the queue
      while (working_queue.size() > 0)
      {
        // Get the top of thw working queue
        const GRID_INDEX current_vertex = working_queue.front();
        working_queue.pop_front();
        component_processed_vertices++;
        vertex_components[current_vertex] = connected_components;
        // Check the six possibly-connected vertices and
        // add them to the queue if they are connected
        // Get the connectivity of our index
        uint8_t connectivity = surface_vertex_connectivity.at(current_vertex);
        // Go through the neighbors
        if ((connectivity & 0b00000001) > 0)
        {
          // Try to add the vertex
          const GRID_INDEX connected_vertex(current_vertex.x,
                                            current_vertex.y,
                                            current_vertex.z - 1);
          // We only add if we haven't already processed it
          if (queued_hashtable[connected_vertex] <= 0)
          {
            queued_hashtable[connected_vertex] = 1;
            working_queue.push_back(connected_vertex);
          }
        }
        if ((connectivity & 0b00000010) > 0)
        {
          // Try to add the vertex
          const GRID_INDEX connected_vertex(current_vertex.x,
                                            current_vertex.y,
                                            current_vertex.z + 1);
          // We only add if we haven't already processed it
          if (queued_hashtable[connected_vertex] <= 0)
          {
            queued_hashtable[connected_vertex] = 1;
            working_queue.push_back(connected_vertex);
          }
        }
        if ((connectivity & 0b00000100) > 0)
        {
          // Try to add the vertex
          const GRID_INDEX connected_vertex(current_vertex.x,
                                            current_vertex.y - 1,
                                            current_vertex.z);
          // We only add if we haven't already processed it
          if (queued_hashtable[connected_vertex] <= 0)
          {
            queued_hashtable[connected_vertex] = 1;
            working_queue.push_back(connected_vertex);
          }
        }
        if ((connectivity & 0b00001000) > 0)
        {
          // Try to add the vertex
          const GRID_INDEX connected_vertex(current_vertex.x,
                                            current_vertex.y + 1,
                                            current_vertex.z);
          // We only add if we haven't already processed it
          if (queued_hashtable[connected_vertex] <= 0)
          {
            queued_hashtable[connected_vertex] = 1;
            working_queue.push_back(connected_vertex);
          }
        }
        if ((connectivity & 0b00010000) > 0)
        {
          // Try to add the vertex
          const GRID_INDEX connected_vertex(current_vertex.x - 1,
                                            current_vertex.y,
                                            current_vertex.z);
          // We only add if we haven't already processed it
          if (queued_hashtable[connected_vertex] <= 0)
          {
            queued_hashtable[connected_vertex] = 1;
            working_queue.push_back(connected_vertex);
          }
        }
        if ((connectivity & 0b00100000) > 0)
        {
          // Try to add the vertex
          const GRID_INDEX connected_vertex(current_vertex.x + 1,
                                            current_vertex.y,
                                            current_vertex.z);
          // We only add if we haven't already processed it
          if (queued_hashtable[connected_vertex] <= 0)
          {
            queued_hashtable[connected_vertex] = 1;
            working_queue.push_back(connected_vertex);
          }
        }
      }
      processed_vertices += component_processed_vertices;
      if (processed_vertices == (int64_t)surface_vertex_connectivity.size())
      {
        break;
      }
    }
  }
  return connected_components;
}

template<typename T, typename BackingStore=std::vector<T>>
std::map<uint32_t, std::unordered_map<GRID_INDEX, uint8_t>>
ExtractComponentSurfaces(
    const VoxelGrid::VoxelGrid<T, BackingStore>& source_grid,
    const std::function<int64_t(const GRID_INDEX&)>& get_component_fn,
    const std::function<bool(const GRID_INDEX&)>& is_surface_index_fn)
{
  std::map<uint32_t, std::unordered_map<GRID_INDEX, uint8_t>>
      component_surfaces;
  // Loop through the grid and extract surface cells for each component
  for (int64_t x_index = 0; x_index < source_grid.GetNumXCells(); x_index++)
  {
    for (int64_t y_index = 0; y_index < source_grid.GetNumYCells(); y_index++)
    {
      for (int64_t z_index = 0; z_index < source_grid.GetNumZCells(); z_index++)
      {
        const GRID_INDEX current_index(x_index, y_index, z_index);
        if (is_surface_index_fn(current_index))
        {
          component_surfaces[get_component_fn(current_index)][current_index]
              = 1;
        }
      }
    }
  }
  return component_surfaces;
}

inline std::pair<int32_t, int32_t> ComputeHolesInSurface(
    const uint32_t component,
    const std::unordered_map<GRID_INDEX, uint8_t>& surface,
    const std::function<int64_t(const GRID_INDEX&)>& get_component_fn,
    const bool verbose)
{
  // We have a list of all voxels with an exposed surface face
  // We loop through this list of voxels, and convert each voxel
  // into 8 vertices (the corners), which we individually check:
  //
  // First - we check to see if the vertex has already been
  // evaluated
  //
  // Second - we check if the vertex is actually on the surface
  // (make sure at least one of the three adjacent vertices is
  // exposed)
  //
  // Third - insert into hashtable of surface vertices
  //
  // Once we have completed this process, we loop back through
  // the hashtable of surface vertices and compute the number
  // of distance-1 neighboring surface vertices (we do this by
  // checking each of the six potential neighbor vertices) and
  // keep a running count of all vertices with 3, 5, and 6
  // neighbors.
  //
  // Once we have evaluated all the neighbors of all surface
  // vertices, we count the number of holes in the grid using
  // the formula from Chen and Rong, "Linear Time Recognition
  // Algorithms for Topological Invariants in 3D":
  //
  // #holes = 1 + (M5 + 2 * M6 - M3) / 8
  //
  // where M5 is the number of vertices with 5 neighbors,
  // M6 is the number of vertices with 6 neighbors, and
  // M3 is the number of vertices with 3 neighbors
  //
  // Storage for surface vertices
  // Compute a hint for initial surface vertex hashmap size
  // expected # of surface vertices
  // surface cells * 8
#ifdef ENABLE_UNORDERED_MAP_SIZE_HINTS
  const size_t surface_vertices_size_hint = surface.size() * 8;
  std::unordered_map<GRID_INDEX, uint8_t> surface_vertices(
        surface_vertices_size_hint);
#else
  std::unordered_map<GRID_INDEX, uint8_t> surface_vertices;
#endif
  // Loop through all the surface voxels and extract surface vertices
  for (auto surface_itr = surface.begin(); surface_itr != surface.end(); ++surface_itr)
  {
    const GRID_INDEX& current_index = surface_itr->first;
    // First, grab all six neighbors from the grid
    const int64_t xyzm1_component
        = get_component_fn(GRID_INDEX(current_index.x,
                                      current_index.y,
                                      current_index.z - 1));
    const int64_t xyzp1_component
        = get_component_fn(GRID_INDEX(current_index.x,
                                      current_index.y,
                                      current_index.z - 1));
    const int64_t xym1z_component
        = get_component_fn(GRID_INDEX(current_index.x,
                                      current_index.y - 1,
                                      current_index.z));
    const int64_t xyp1z_component
        = get_component_fn(GRID_INDEX(current_index.x,
                                      current_index.y + 1,
                                      current_index.z));
    const int64_t xm1yz_component
        = get_component_fn(GRID_INDEX(current_index.x - 1,
                                      current_index.y,
                                      current_index.z));
    const int64_t xp1yz_component
        = get_component_fn(GRID_INDEX(current_index.x + 1,
                                      current_index.y,
                                      current_index.z));
    // Generate all 8 vertices for the current voxel, check if an adjacent
    // vertex is on the surface, and insert it if so
    // First, check the (-,-,-) vertex
    if (component != xyzm1_component
        || component != xym1z_component
        || component != xm1yz_component)
    {
      const GRID_INDEX vertex1(current_index.x,
                               current_index.y,
                               current_index.z);
      surface_vertices[vertex1] = 1;
    }
    // Second, check the (-,-,+) vertex
    if (component != xyzp1_component
        || component != xym1z_component
        || component != xm1yz_component)
    {
      const GRID_INDEX vertex2(current_index.x,
                               current_index.y,
                               current_index.z + 1);
      surface_vertices[vertex2] = 1;
    }
    // Third, check the (-,+,-) vertex
    if (component != xyzm1_component
        || component != xyp1z_component
        || component != xm1yz_component)
    {
      const GRID_INDEX vertex3(current_index.x,
                               current_index.y + 1,
                               current_index.z);
      surface_vertices[vertex3] = 1;
    }
    // Fourth, check the (-,+,+) vertex
    if (component != xyzp1_component
        || component != xyp1z_component
        || component != xm1yz_component)
    {
      const GRID_INDEX vertex4(current_index.x,
                               current_index.y + 1,
                               current_index.z + 1);
      surface_vertices[vertex4] = 1;
    }
    // Fifth, check the (+,-,-) vertex
    if (component != xyzm1_component
        || component != xym1z_component
        || component != xp1yz_component)
    {
      const GRID_INDEX vertex5(current_index.x + 1,
                               current_index.y,
                               current_index.z);
      surface_vertices[vertex5] = 1;
    }
    // Sixth, check the (+,-,+) vertex
    if (component != xyzp1_component
        || component != xym1z_component
        || component != xp1yz_component)
    {
      const GRID_INDEX vertex6(current_index.x + 1,
                               current_index.y,
                               current_index.z + 1);
      surface_vertices[vertex6] = 1;
    }
    // Seventh, check the (+,+,-) vertex
    if (component != xyzm1_component
        || component != xyp1z_component
        || component != xp1yz_component)
    {
      const GRID_INDEX vertex7(current_index.x + 1,
                               current_index.y + 1,
                               current_index.z);
      surface_vertices[vertex7] = 1;
    }
    // Eighth, check the (+,+,+) vertex
    if (component != xyzp1_component
        || component != xyp1z_component
        || component != xp1yz_component)
    {
      const GRID_INDEX vertex8(current_index.x + 1,
                               current_index.y + 1,
                               current_index.z + 1);
      surface_vertices[vertex8] = 1;
    }
  }
  if (verbose)
  {
    std::cerr << "Surface with " << surface.size() << " voxels has "
              << surface_vertices.size() << " surface vertices" << std::endl;
  }
  // Iterate through the surface vertices and count the neighbors of each vertex
  int32_t M3 = 0;
  int32_t M5 = 0;
  int32_t M6 = 0;
  // Store the connectivity of each vertex
  // Compute a hint for initial vertex connectivity hashmap size
  // real # of surface vertices
  // surface vertices
  const size_t vertex_connectivity_size_hint = surface_vertices.size();
  std::unordered_map<GRID_INDEX, uint8_t> vertex_connectivity(
        vertex_connectivity_size_hint);
  std::unordered_map<GRID_INDEX, uint8_t>::iterator surface_vertices_itr;
  for (surface_vertices_itr = surface_vertices.begin();
       surface_vertices_itr != surface_vertices.end();
       ++surface_vertices_itr)
  {
    const GRID_INDEX key = surface_vertices_itr->first;
    // Insert into the connectivity map
    vertex_connectivity[key] = 0b00000000;
    // Check the six edges from the current vertex and count the number of
    // exposed edges (an edge is exposed if the at least one of the four
    // surrounding voxels is not part of the current component)
    int32_t edge_count = 0;
    // First, get the 8 voxels that surround the current vertex
    const int64_t xm1ym1zm1_component
        = get_component_fn(GRID_INDEX(key.x - 1, key.y - 1, key.z - 1));
    const int64_t xm1ym1zp1_component
        = get_component_fn(GRID_INDEX(key.x - 1, key.y - 1, key.z + 0));
    const int64_t xm1yp1zm1_component
        = get_component_fn(GRID_INDEX(key.x - 1, key.y + 0, key.z - 1));
    const int64_t xm1yp1zp1_component
        = get_component_fn(GRID_INDEX(key.x - 1, key.y + 0, key.z + 0));
    const int64_t xp1ym1zm1_component
        = get_component_fn(GRID_INDEX(key.x + 0, key.y - 1, key.z - 1));
    const int64_t xp1ym1zp1_component
        = get_component_fn(GRID_INDEX(key.x + 0, key.y - 1, key.z + 0));
    const int64_t xp1yp1zm1_component
        = get_component_fn(GRID_INDEX(key.x + 0, key.y + 0, key.z - 1));
    const int64_t xp1yp1zp1_component
        = get_component_fn(GRID_INDEX(key.x + 0, key.y + 0, key.z + 0));
    // Check the "z- down" edge
    if (component != xm1ym1zm1_component || component != xm1yp1zm1_component
        || component != xp1ym1zm1_component || component != xp1yp1zm1_component)
    {
      if (!(component != xm1ym1zm1_component
            && component != xm1yp1zm1_component
            && component != xp1ym1zm1_component
            && component != xp1yp1zm1_component))
      {
        edge_count++;
        vertex_connectivity[key] |= 0b00000001;
      }
    }
    // Check the "z+ up" edge
    if (component != xm1ym1zp1_component || component != xm1yp1zp1_component
        || component != xp1ym1zp1_component || component != xp1yp1zp1_component)
    {
      if (!(component != xm1ym1zp1_component
            && component != xm1yp1zp1_component
            && component != xp1ym1zp1_component
            && component != xp1yp1zp1_component))
      {
        edge_count++;
        vertex_connectivity[key] |= 0b00000010;
      }
    }
    // Check the "y- right" edge
    if (component != xm1ym1zm1_component || component != xm1ym1zp1_component
        || component != xp1ym1zm1_component || component != xp1ym1zp1_component)
    {
      if (!(component != xm1ym1zm1_component
            && component != xm1ym1zp1_component
            && component != xp1ym1zm1_component
            && component != xp1ym1zp1_component))
      {
        edge_count++;
        vertex_connectivity[key] |= 0b00000100;
      }
    }
    // Check the "y+ left" edge
    if (component != xm1yp1zm1_component || component != xm1yp1zp1_component
        || component != xp1yp1zm1_component || component != xp1yp1zp1_component)
    {
      if (!(component != xm1yp1zm1_component
            && component != xm1yp1zp1_component
            && component != xp1yp1zm1_component
            && component != xp1yp1zp1_component))
      {
        edge_count++;
        vertex_connectivity[key] |= 0b00001000;
      }
    }
    // Check the "x- back" edge
    if (component != xm1ym1zm1_component || component != xm1ym1zp1_component
        || component != xm1yp1zm1_component || component != xm1yp1zp1_component)
    {
      if (!(component != xm1ym1zm1_component
            && component != xm1ym1zp1_component
            && component != xm1yp1zm1_component
            && component != xm1yp1zp1_component))
      {
        edge_count++;
        vertex_connectivity[key] |= 0b00010000;
      }
    }
    // Check the "x+ front" edge
    if (component != xp1ym1zm1_component || component != xp1ym1zp1_component
        || component != xp1yp1zm1_component || component != xp1yp1zp1_component)
    {
      if (!(component != xp1ym1zm1_component
            && component != xp1ym1zp1_component
            && component != xp1yp1zm1_component
            && component != xp1yp1zp1_component))
      {
        edge_count++;
        vertex_connectivity[key] |= 0b00100000;
      }
    }
    // Increment M counts
    if (edge_count == 3)
    {
      M3++;
    }
    else if (edge_count == 5)
    {
      M5++;
    }
    else if (edge_count == 6)
    {
      M6++;
    }
  }
  // Check to see if the set of vertices is connected.
  // If not, our object contains void(s)
  const int32_t number_of_surfaces
      = ComputeConnectivityOfSurfaceVertices(vertex_connectivity);
  const int32_t number_of_voids = number_of_surfaces - 1;
  // Compute the number of holes in the surface
  const int32_t raw_number_of_holes = 1 + ((M5 + (2 * M6) - M3) / 8);
  const int32_t number_of_holes = raw_number_of_holes + number_of_voids;
  if (verbose)
  {
    std::cout << "Processing surface with M3 = " << M3 << " M5 = " << M5
              << " M6 = " << M6 << " holes = " << number_of_holes
              << " surfaces = " << number_of_surfaces
              << " voids = " << number_of_voids << std::endl;
  }
  return std::pair<int32_t, int32_t>(number_of_holes, number_of_voids);
}

template<typename T, typename BackingStore=std::vector<T>>
std::map<uint32_t, std::pair<int32_t, int32_t>>
ComputeComponentTopology(
    const VoxelGrid::VoxelGrid<T, BackingStore>& source_grid,
    const std::function<int64_t(const GRID_INDEX&)>& get_component_fn,
    const std::function<bool(const GRID_INDEX&)>& is_surface_index_fn,
    const bool verbose)
{
  // Extract the surfaces of each connected component
  const std::map<uint32_t, std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>>
      component_surfaces = ExtractComponentSurfaces(source_grid,
                                                    get_component_fn,
                                                    is_surface_index_fn);
  // Compute the number of holes in each surface
  std::map<uint32_t, std::pair<int32_t, int32_t>> component_holes_and_voids;
  for (auto component_surfaces_itr = component_surfaces.begin();
       component_surfaces_itr != component_surfaces.end();
       ++component_surfaces_itr)
  {
    const uint32_t component_number = component_surfaces_itr->first;
    const std::unordered_map<VoxelGrid::GRID_INDEX, uint8_t>& component_surface
        = component_surfaces_itr->second;
    const std::pair<int32_t, int32_t> number_of_holes_and_voids
        = ComputeHolesInSurface(component_number,
                                component_surface,
                                get_component_fn,
                                verbose);
    component_holes_and_voids[component_number] = number_of_holes_and_voids;
  }
  return component_holes_and_voids;
}

// Extracts the active indices from a surface map as a vector, which is useful
// in contexts where a 1-dimensional index into the surface is needed
inline std::vector<GRID_INDEX> ExtractStaticSurface(
    const std::unordered_map<GRID_INDEX, uint8_t>& raw_surface)
{
  std::vector<GRID_INDEX> static_surface;
  // This may be larger than the actual surface we'll extract
  static_surface.reserve(raw_surface.size());
  for (auto itr = raw_surface.begin(); itr != raw_surface.end(); ++itr)
  {
    const GRID_INDEX& index = itr->first;
    const uint8_t value = itr->second;
    if (value == 1)
    {
      static_surface.push_back(index);
    }
  }
  // Try to reclaim the unnecessary vector capacity
  static_surface.shrink_to_fit();
  return static_surface;
}

inline std::unordered_map<GRID_INDEX, uint8_t> ConvertToDynamicSurface(
    const std::vector<GRID_INDEX>& static_surface)
{
  std::unordered_map<GRID_INDEX, uint8_t> dynamic_surface(
        static_surface.size());
  for (size_t idx = 0; idx < static_surface.size(); idx++)
  {
    const GRID_INDEX& grid_index = static_surface[idx];
    dynamic_surface[grid_index] = 1u;
  }
  return dynamic_surface;
}

inline std::unordered_map<GRID_INDEX, size_t> BuildSurfaceIndexMap(
    const std::vector<GRID_INDEX>& static_surface)
{
  std::unordered_map<GRID_INDEX, size_t> dynamic_surface(static_surface.size());
  for (size_t idx = 0; idx < static_surface.size(); idx++)
  {
    const GRID_INDEX& current_index = static_surface[idx];
    dynamic_surface[current_index] = idx;
  }
  return dynamic_surface;
}
}

#endif // TOPOLOGY_COMPUTATION_HPP
