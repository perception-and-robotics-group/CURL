#include "CURL_Extraction_mex.h"
#include "mex.hpp"
#include "mexAdapter.hpp"

class MexFunction : public matlab::mex::Function {
public:
    void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        matlab::data::TypedArray <matlab::data::Struct> lidarConfig = std::move(inputs[0]);
        matlab::data::TypedArray <matlab::data::Struct> patchConfig = std::move(inputs[1]);
        matlab::data::TypedArray <matlab::data::Struct> thresholdConfig = std::move(inputs[2]);
        matlab::data::TypedArray<double> points = std::move(inputs[3]);
        matlab::data::TypedArray<double> polar = std::move(inputs[4]);
        matlab::data::TypedArray<double> mask = std::move(inputs[5]);
        processing(outputs, lidarConfig, patchConfig, thresholdConfig, points, polar, mask);
    }

    void processing(matlab::mex::ArgumentList &outputs, matlab::data::TypedArray <matlab::data::Struct> &lidarConfig, matlab::data::TypedArray <matlab::data::Struct> &patchConfig,
                    matlab::data::TypedArray <matlab::data::Struct> &thresholdConfig, matlab::data::TypedArray<double> &points,
                    matlab::data::TypedArray<double> &polar, matlab::data::TypedArray<double> &mask) {
        CURL_Extraction CURL;
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
        // patchConfig
        matlab::data::TypedArrayRef<double> degree_initial = patchConfig[0]["degree_initial"];
        matlab::data::TypedArrayRef<double> degree_iter_step = patchConfig[0]["degree_iter_step"];
        matlab::data::TypedArrayRef<double> degree_max = patchConfig[0]["degree_max"];
        matlab::data::TypedArrayRef<double> patch_row_rso = patchConfig[0]["patch_row_rso"];
        matlab::data::TypedArrayRef<double> patch_col_rso = patchConfig[0]["patch_col_rso"];
        matlab::data::TypedArrayRef<double> row_overlap_step = patchConfig[0]["row_overlap_step"];
        matlab::data::TypedArrayRef<double> col_overlap_step = patchConfig[0]["col_overlap_step"];
        matlab::data::TypedArrayRef<double> k = patchConfig[0]["k"];
        matlab::data::TypedArrayRef<bool> is_1_1_recons_only = patchConfig[0]["is_1_1_recons_only"];
        CURL.patchConfig.degree_initial = degree_initial[0];
        CURL.patchConfig.degree_iter_step = degree_iter_step[0];
        CURL.patchConfig.degree_max = degree_max[0];
        CURL.patchConfig.patch_row_rso = patch_row_rso[0];
        CURL.patchConfig.patch_col_rso = patch_col_rso[0];
        CURL.patchConfig.row_overlap_step = row_overlap_step[0];
        CURL.patchConfig.col_overlap_step = col_overlap_step[0];
        CURL.patchConfig.k = k[0];
        CURL.patchConfig.is_1_1_recons_only = is_1_1_recons_only[0];
    
        //thresholdConfig
        matlab::data::TypedArrayRef<double> avg_filter_thres = thresholdConfig[0]["avg_filter_thres"];
        matlab::data::TypedArrayRef<double> rmv_filter_thres = thresholdConfig[0]["rmv_filter_thres"];
        matlab::data::TypedArrayRef<double> avg_error_thres_patch = thresholdConfig[0]["avg_error_thres_patch"];
        matlab::data::TypedArrayRef<double> remove = thresholdConfig[0]["remove"];
        matlab::data::TypedArrayRef<double> u = thresholdConfig[0]["u"];
        matlab::data::TypedArrayRef<double> v = thresholdConfig[0]["v"];
        matlab::data::TypedArrayRef<double> d = thresholdConfig[0]["d"];        

        CURL.thresholdConfig.avg_filter_thres = avg_filter_thres[0];
        CURL.thresholdConfig.rmv_filter_thres = rmv_filter_thres[0];
        CURL.thresholdConfig.avg_error_thres_patch = avg_error_thres_patch[0];
        CURL.thresholdConfig.remove = remove[0];
        CURL.thresholdConfig.u = u[0];
        CURL.thresholdConfig.v = v[0];
        CURL.thresholdConfig.d = d[0];

        // load matrices
        auto dim = points.getDimensions();
        CURL.points_from_mesh_all_point_cloud.resize(dim[0], dim[1]);
        for (int i = 0; i < dim[0]; i++) {
            for (int j = 0; j < dim[1]; j++) {
                CURL.points_from_mesh_all_point_cloud(i, j) = points[i][j];
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
        // export the coefficients
        long unsigned int i_sequence_size = CURL.i_sequence.size();
        long unsigned int j_sequence_size = CURL.j_sequence.size();
        matlab::data::CellArray coefficients_cell_mat = factory.createArray<matlab::data::Array>({i_sequence_size, j_sequence_size});
        long unsigned int coefficients_size;
        for (int i = 0; i < i_sequence_size; ++i) {
            for (int j = 0; j < j_sequence_size; ++j) {
                if (CURL.coefficients[i * j_sequence_size + j].size() > 0) {
                    coefficients_size = CURL.coefficients[i * j_sequence_size + j].size();
                    coefficients_cell_mat[i][j] = factory.createArray<double>({coefficients_size, 1});
                    matlab::data::TypedArrayRef<double> coefficients_cell_mat_array = coefficients_cell_mat[i][j];
                    for (int k = 0; k < coefficients_size; ++k) {
                        coefficients_cell_mat_array[k][0] = CURL.coefficients[i * j_sequence_size + j](k);
                    }
                }
            }
        }
        outputs[1] = std::move(coefficients_cell_mat);
        // export the mask
        long unsigned int i_mask_size = CURL.mask_recons_update_grid.rows();
        long unsigned int j_mask_size = CURL.mask_recons_update_grid.cols();
        matlab::data::Array mask_mat = factory.createArray<double>({i_mask_size, j_mask_size});
        for (int i = 0; i < i_mask_size; ++i) {
            for (int j = 0; j < j_mask_size; ++j) {
                mask_mat[i][j] = double(CURL.mask_recons_update_grid(i, j));
            }
        }
        outputs[2] = std::move(mask_mat);
    }
};

