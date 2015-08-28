#include "arc_utilities/voxel_grid.hpp"
#include "arc_utilities/pretty_print.hpp"
#include "arc_utilities/dynamic_spatial_hashed_voxel_grid.hpp"
#include "sdf_tools/dynamic_spatial_hashed_collision_map.hpp"
#include "sdf_tools/sdf.hpp"

void test_voxel_grid_indices()
{
    VoxelGrid::VoxelGrid<int> test_grid(1.0, 20.0, 20.0, 20.0, 0);
    // Load with special values
    int check_val = 1;
    std::vector<int> check_vals;
    for (int64_t x_index = 0; x_index < test_grid.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < test_grid.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < test_grid.GetNumZCells(); z_index++)
            {
                test_grid.SetWithValue(x_index, y_index, z_index, check_val);
                check_vals.push_back(check_val);
                check_val++;
            }
        }
    }
    // Check the values
    int check_index = 0;
    bool pass = true;
    for (int64_t x_index = 0; x_index < test_grid.GetNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < test_grid.GetNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < test_grid.GetNumZCells(); z_index++)
            {
                int ref_val = test_grid.GetImmutable(x_index, y_index, z_index).first;
                //std::cout << "Value in grid: " << ref_val << " Value should be: " << check_vals[check_index] << std::endl;
                if (ref_val == check_vals[check_index])
                {
                    //std::cout << "Check pass" << std::endl;
                }
                else
                {
                    std::cout << "Check fail" << std::endl;
                    pass = false;
                }
                check_index++;
            }
        }
    }
    if (pass)
    {
        std::cout << "VG-I - All checks pass" << std::endl;
    }
    else
    {
        std::cout << "*** VG-I - Checks failed ***" << std::endl;
    }
}

void test_voxel_grid_locations()
{
    VoxelGrid::VoxelGrid<int> test_grid(1.0, 20.0, 20.0, 20.0, 0);
    // Load with special values
    int check_val = 1;
    std::vector<int> check_vals;
    for (double x_pos = -9.5; x_pos <= 9.5; x_pos += 1.0)
    {
        for (double y_pos = -9.5; y_pos <= 9.5; y_pos += 1.0)
        {
            for (double z_pos = -9.5; z_pos <= 9.5; z_pos += 1.0)
            {
                test_grid.SetWithValue(x_pos, y_pos, z_pos, check_val);
                check_vals.push_back(check_val);
                check_val++;
            }
        }
    }
    // Check the values
    int check_index = 0;
    bool pass = true;
    for (double x_pos = -9.5; x_pos <= 9.5; x_pos += 1.0)
    {
        for (double y_pos = -9.5; y_pos <= 9.5; y_pos += 1.0)
        {
            for (double z_pos = -9.5; z_pos <= 9.5; z_pos += 1.0)
            {
                int ref_val = test_grid.GetImmutable(x_pos, y_pos, z_pos).first;
                //std::cout << "Value in grid: " << ref_val << " Value should be: " << check_vals[check_index] << std::endl;
                if (ref_val == check_vals[check_index])
                {
                    //std::cout << "Value check pass" << std::endl;
                }
                else
                {
                    std::cout << "Value check fail" << std::endl;
                    pass = false;
                }
                check_index++;
                std::vector<double> query_point = {x_pos, y_pos, z_pos};
                //std::cout << "Query point - " << PrettyPrint::PrettyPrint(query_point) << std::endl;
                std::vector<int64_t> query_index = test_grid.LocationToGridIndex(x_pos, y_pos, z_pos);
                //std::cout << "Query index - " << PrettyPrint::PrettyPrint(query_index) << std::endl;
                std::vector<double> query_location = test_grid.GridIndexToLocation(query_index[0], query_index[1], query_index[2]);
                //std::cout << "Query location - " << PrettyPrint::PrettyPrint(query_location) << std::endl;
                std::vector<int64_t> found_query_index = test_grid.LocationToGridIndex(query_location[0], query_location[1], query_location[2]);
                //std::cout << "Found query index - " << PrettyPrint::PrettyPrint(found_query_index) << std::endl;
                if (query_point[0] == query_location[0] && query_point[1] == query_location[1] && query_point[2] == query_location[2])
                {
                    //std::cout << "Position check pass" << std::endl;
                }
                else
                {
                    std::cout << "Position check fail" << std::endl;
                    pass = false;
                }
                if (query_index[0] == found_query_index[0] && query_index[1] == found_query_index[1] && query_index[2] == found_query_index[2])
                {
                    //std::cout << "Position index check pass" << std::endl;
                }
                else
                {
                    std::cout << "Position index check fail" << std::endl;
                    pass = false;
                }
            }
        }
    }
    if (pass)
    {
        std::cout << "VG-L - All checks pass" << std::endl;
    }
    else
    {
        std::cout << "*** VG-L - Checks failed ***" << std::endl;
    }
}

void test_dsh_voxel_grid_locations()
{
    VoxelGrid::DynamicSpatialHashedVoxelGrid<int> test_grid(1.0, 4, 4, 4, 0);
    // Load with special values
    int check_val = 1;
    std::vector<int> check_vals;
    for (double x_pos = -9.5; x_pos <= 9.5; x_pos += 1.0)
    {
        for (double y_pos = -9.5; y_pos <= 9.5; y_pos += 1.0)
        {
            for (double z_pos = -9.5; z_pos <= 9.5; z_pos += 1.0)
            {
                test_grid.SetCellWithValue(x_pos, y_pos, z_pos, check_val);
                check_vals.push_back(check_val);
                check_val++;
            }
        }
    }
    // Check the values
    int check_index = 0;
    bool pass = true;
    for (double x_pos = -9.5; x_pos <= 9.5; x_pos += 1.0)
    {
        for (double y_pos = -9.5; y_pos <= 9.5; y_pos += 1.0)
        {
            for (double z_pos = -9.5; z_pos <= 9.5; z_pos += 1.0)
            {
                int ref_val = test_grid.GetImmutable(x_pos, y_pos, z_pos).first;
                //std::cout << "Value in grid: " << ref_val << " Value should be: " << check_vals[check_index] << std::endl;
                if (ref_val == check_vals[check_index])
                {
                    //std::cout << "Value check pass" << std::endl;
                }
                else
                {
                    std::cout << "Value check fail" << std::endl;
                    pass = false;
                }
                check_index++;
            }
        }
    }
    if (pass)
    {
        std::cout << "DSHVG - All checks pass" << std::endl;
    }
    else
    {
        std::cout << "*** DSHVG - Checks failed ***" << std::endl;
    }
}


void test_float_binary_conversion(float test_val)
{
    std::cout << "Initial value " << test_val << std::endl;
    std::vector<u_int8_t> binary_value = FloatToBinary(test_val);
    float final_val = FloatFromBinary(binary_value);
    std::cout << "Final value " << final_val << std::endl;
}

int main()
{
    test_voxel_grid_indices();
    test_voxel_grid_locations();
    test_dsh_voxel_grid_locations();
    test_float_binary_conversion(5280.0);
    return 0;
}
