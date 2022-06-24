//
// Created by zkc on 01/05/22.
//
#include "CURL_Reconstruction_mex.h"
#include "mex.hpp"
#include "mexAdapter.hpp"

class MexFunction : public matlab::mex::Function {
public:
    void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        matlab::data::TypedArray <matlab::data::Struct> lidarConfig = std::move(inputs[0]);
        matlab::data::TypedArray <matlab::data::Struct> reconsConfig = std::move(inputs[1]);
        matlab::data::CellArray coefficients = std::move(inputs[2]);
        matlab::data::TypedArray<double> polar = std::move(inputs[3]);
        matlab::data::TypedArray<double> mask = std::move(inputs[4]);
        processing(outputs, lidarConfig, reconsConfig, coefficients, polar, mask);
    }

    void processing(matlab::mex::ArgumentList &outputs, matlab::data::TypedArray <matlab::data::Struct> &lidarConfig, matlab::data::TypedArray <matlab::data::Struct> &reconsConfig,
                    matlab::data::CellArray coefficients, matlab::data::TypedArray<double> &polar, matlab::data::TypedArray<double> &mask) {
        CURL_Reconstruction CURL;
        // load parameters
        // lidarConfig
        matlab::data::TypedArrayRef<double> num_lasers = lidarConfig[0]["num_lasers"];
        matlab::data::TypedArrayRef<double> full_num_lasers = lidarConfig[0]["full_num_lasers"];
        matlab::data::TypedArrayRef<double> img_length = lidarConfig[0]["img_length"];
        matlab::data::TypedArrayRef<double> row_times = lidarConfig[0]["row_times"];
        matlab::data::TypedArrayRef<double> col_times = lidarConfig[0]["col_times"];
        matlab::data::TypedArrayRef<double> full_fov_up = lidarConfig[0]["full_fov_up"];
        matlab::data::TypedArrayRef<double> fov_down = lidarConfig[0]["fov_down"];
        matlab::data::TypedArrayRef<double> length_percentage = lidarConfig[0]["length_percentage"];
        matlab::data::TypedArrayRef<double> min_length = lidarConfig[0]["min_length"];
        matlab::data::TypedArrayRef<double> full_fov = lidarConfig[0]["full_fov"];
        matlab::data::TypedArrayRef<double> u_res = lidarConfig[0]["u_res"];
        matlab::data::TypedArrayRef<double> fov = lidarConfig[0]["fov"];
        matlab::data::TypedArrayRef<double> fov_up = lidarConfig[0]["fov_up"];
        matlab::data::TypedArrayRef<double> is_density_enhance = lidarConfig[0]["is_density_enhance"];
        matlab::data::TypedArrayRef<double> max_length = lidarConfig[0]["max_length"];
        CURL.lidarConfig.num_lasers = num_lasers[0];
        CURL.lidarConfig.full_num_lasers = full_num_lasers[0];
        CURL.lidarConfig.img_length = img_length[0];
        CURL.lidarConfig.row_times = row_times[0];
        CURL.lidarConfig.col_times = col_times[0];
        CURL.lidarConfig.full_fov_up = full_fov_up[0];
        CURL.lidarConfig.fov_down = fov_down[0];
        CURL.lidarConfig.length_percentage = length_percentage[0];
        CURL.lidarConfig.min_length = min_length[0];
        CURL.lidarConfig.full_fov = full_fov[0];
        CURL.lidarConfig.u_res = u_res[0];
        CURL.lidarConfig.fov = fov[0];
        CURL.lidarConfig.fov_up = fov_up[0];
        CURL.lidarConfig.is_density_enhance = is_density_enhance[0];
        CURL.lidarConfig.max_length = max_length[0];
        // reconsConfig
        matlab::data::TypedArrayRef<double> recons_row_times = reconsConfig[0]["row_times"];
        matlab::data::TypedArrayRef<double> recons_col_times = reconsConfig[0]["col_times"];
        matlab::data::TypedArrayRef<double> recons_is_density_enhance = reconsConfig[0]["is_density_enhance"];
        CURL.reconsConfig.row_times = recons_row_times[0];
        CURL.reconsConfig.col_times = recons_col_times[0];
        CURL.reconsConfig.is_density_enhance = recons_is_density_enhance[0];
        // load coefficients
        auto dim = coefficients.getDimensions();
        CURL.coefficients.resize(dim[0] * dim[1]);
        CURL.patch_num_row = dim[0];
        CURL.patch_num_col = dim[1];
        for (int i = 0; i < dim[0]; ++i) {
            for (int j = 0; j < dim[1]; ++j) {
                matlab::data::TypedArrayRef<double> coefficients_cell_mat_array = coefficients[i][j];
                auto dim_vec = coefficients_cell_mat_array.getDimensions();
                if (dim_vec[0] > 0) {
                    CURL.coefficients[i * dim[1] + j].resize(dim_vec[0]);
                    for (int k = 0; k < dim_vec[0]; ++k) {
                        CURL.coefficients[i * dim[1] + j](k) = coefficients_cell_mat_array[k][0];
                    }
                }
            }
        }
        // load polar table
        dim = polar.getDimensions();
        CURL.table_init_SPHARM_grid.resize(dim[0], dim[1]);
        for (int i = 0; i < dim[0]; i++) {
            for (int j = 0; j < dim[1]; j++) {
                CURL.table_init_SPHARM_grid(i, j) = polar[i][j];
            }
        }
        // load mask
        dim = mask.getDimensions();
        CURL.mask_recons_update_grid.resize(dim[0], dim[1]);
        for (int i = 0; i < dim[0]; i++) {
            for (int j = 0; j < dim[1]; j++) {
                CURL.mask_recons_update_grid(i, j) = mask[i][j];
            }
        }
        // start the calculation
        Timer timer;
        timer.start();
        CURL.processing();
        double extraction_time = timer.elapsedSeconds();
        // export the time
        matlab::data::ArrayFactory factory;
        matlab::data::Array extraction_time_mat = factory.createArray<double>({1, 1});
        extraction_time_mat[0] = extraction_time;
        outputs[0] = std::move(extraction_time_mat);
        // export reconsPointCloud
        unsigned long int reconsPointCloud_row_size = CURL.reconsPointCloud.rows();
        unsigned long int reconsPointCloud_col_size = CURL.reconsPointCloud.cols();
        matlab::data::Array reconsPointCloud = factory.createArray<double>({reconsPointCloud_row_size, reconsPointCloud_col_size});
        for (int i = 0; i < reconsPointCloud_row_size; ++i) {
            for (int j = 0; j < reconsPointCloud_col_size; ++j) {
                reconsPointCloud[i][j] = CURL.reconsPointCloud(i, j);
            }
        }
        outputs[1] = std::move(reconsPointCloud);
        // export reconsPointCloudFiltered
        unsigned long int reconsPointCloudFiltered_row_size = CURL.reconsPointCloudFiltered.rows();
        unsigned long int reconsPointCloudFiltered_col_size = CURL.reconsPointCloudFiltered.cols();
        matlab::data::Array reconsPointCloudFiltered = factory.createArray<double>({reconsPointCloudFiltered_row_size, reconsPointCloudFiltered_col_size});
        for (int i = 0; i < reconsPointCloudFiltered_row_size; ++i) {
            for (int j = 0; j < reconsPointCloudFiltered_col_size; ++j) {
                reconsPointCloudFiltered[i][j] = CURL.reconsPointCloudFiltered(i, j);
            }
        }
        outputs[2] = std::move(reconsPointCloudFiltered);
    }
};

