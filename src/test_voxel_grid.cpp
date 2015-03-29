#include "sdf_tools/voxel_grid.hpp"
#include "sdf_tools/sdf.hpp"

void test_voxel_grid()
{
    VOXEL_GRID::VoxelGrid<int> test_grid(0.5, 10.0, 10.0, 10.0, 0);
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
                std::cout << "Value in grid: " << ref_val << " Value should be: " << check_vals[check_index] << std::endl;
                if (ref_val == check_vals[check_index])
                {
                    std::cout << "Check pass" << std::endl;
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
        std::cout << "All checks pass" << std::endl;
    }
    else
    {
        std::cout << "*** Checks failed ***" << std::endl;
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
    test_voxel_grid();
    test_float_binary_conversion(5280.0);
    return 0;
}
