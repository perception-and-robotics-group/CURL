//
// Created by zkc on 30/04/22.
//

#ifndef EXAMPLE_CURL_EXTRACTION_MAT_H
#define EXAMPLE_CURL_EXTRACTION_MAT_H
//
// Created by zkc on 22/04/22.
//
#include "ReadMatrix.h"
#include "Eigen/Core"
#include <cmath>
#include <algorithm>
#include "CURL_tools.h"
#include "Timer.h"
#include <fstream>

class CURL_Extraction {
public:
    int patch_row = 0;
    int patch_col = 0;
    int SH_table_up_limit = 2;
    LidarConfig lidarConfig;
    PatchConfig patchConfig;
    ThresholdConfig thresholdConfig;
    Eigen::MatrixXi mask_recons_update_grid;
    Eigen::VectorXi mask_recons_update_col;
    Matrix3dCol points_from_mesh_all_point_cloud;
    Eigen::MatrixXd table_init_SPHARM_grid;
    std::vector<Eigen::VectorXd> coefficients;
    std::vector<Eigen::MatrixXd> SH_table;
    Eigen::MatrixXd r_grid, azi_grid, elev_grid, index_grid;
    LidarConfig lidarConfig_new;
    Eigen::VectorXi i_sequence;
    Eigen::VectorXi j_sequence;


    template<typename T>
    friend T load_Matrix(const std::string &read_dir);

    friend void getSH(int N, Eigen::MatrixXd &dirs, Eigen::MatrixXd &Y_N);

    friend void leastSquaresSHT(int N, Eigen::Ref<Eigen::VectorXd> F, Eigen::MatrixXd dirs, Eigen::VectorXd &F_N);

    inline double w(double e, double k) {
        return 1 / (1 + pow(e / k, 2));
    }

    CURL_Extraction() = default;

    void load_matrices(const std::string &folder) {
        points_from_mesh_all_point_cloud = load_Matrix<Matrix3dCol>(folder + "/points_from_mesh_all_point_cloud.txt");
        table_init_SPHARM_grid = load_Matrix<Eigen::MatrixXd>(folder + "/table_init_SPHARM_grid.txt");
        mask_recons_update_grid = load_Matrix<Eigen::MatrixXd>(folder + "/mask_recons_update_grid.txt").cast<int>();
        mask_recons_update_grid.transposeInPlace();
        mask_recons_update_col = Eigen::Map<Eigen::VectorXi>(mask_recons_update_grid.data(), mask_recons_update_grid.size());
        mask_recons_update_grid.transposeInPlace();
    }

    void griding() {
//        Eigen::MatrixXd sph_points_from_mesh_all_point_cloud(points_from_mesh_all_point_cloud.rows(),
//                                                             points_from_mesh_all_point_cloud.cols());
//        sph_points_from_mesh_all_point_cloud = cart2sph(points_from_mesh_all_point_cloud);
        Matrix3dCol sph_points_from_mesh_all_point_cloud = Matrix3dCol::Zero(points_from_mesh_all_point_cloud.rows(),
                                                                             3);
        cart2sph(points_from_mesh_all_point_cloud, sph_points_from_mesh_all_point_cloud);
        r_grid = sph_points_from_mesh_all_point_cloud.col(2).reshaped<Eigen::RowMajor>(
                lidarConfig.num_lasers * lidarConfig.row_times, lidarConfig.img_length * lidarConfig.col_times);
        azi_grid = table_init_SPHARM_grid.col(0).reshaped<Eigen::RowMajor>(
                lidarConfig.num_lasers * lidarConfig.row_times, lidarConfig.img_length * lidarConfig.col_times);
        elev_grid = table_init_SPHARM_grid.col(1).reshaped<Eigen::RowMajor>(
                lidarConfig.num_lasers * lidarConfig.row_times, lidarConfig.img_length * lidarConfig.col_times);
        index_grid.resize(points_from_mesh_all_point_cloud.rows(), 1);
        index_grid = Eigen::VectorXd::LinSpaced(points_from_mesh_all_point_cloud.rows(), 0,
                                                points_from_mesh_all_point_cloud.rows() - 1);
        index_grid = Eigen::Map<Eigen::MatrixXd>(index_grid.data(),
                                                 lidarConfig.img_length * lidarConfig.col_times,
                                                 lidarConfig.num_lasers * lidarConfig.row_times);
        index_grid.transposeInPlace();
    }

    //output: idx_low idx_high idx_low_ori idx_high_ori
    void enlarge_patches_row(LidarConfig &lidarConfig_new, const int &idx, int *output) const {
        output[0] = idx - patchConfig.row_overlap_step;
        if (output[0] < 0) {
            output[0] = 0;
        }
        output[1] = idx + patchConfig.patch_row_rso - 1 + patchConfig.row_overlap_step;
        if (output[1] >= lidarConfig_new.num_lasers) {
            output[1] = lidarConfig_new.num_lasers - 1;
        }
        output[2] = idx;
        output[3] = idx + patchConfig.patch_row_rso - 1;
        if (output[3] >= lidarConfig_new.num_lasers) {
            output[3] = lidarConfig_new.num_lasers - 1;
        }
    }

    //output: idx_low,idx_high,idx_low_ori,idx_high_ori
    void enlarge_patches_col(LidarConfig &lidarConfig_new, const int &idx, int *output) const {
        output[0] = idx - patchConfig.col_overlap_step;
        if (output[0] < 0) {
            output[0] = output[0] + lidarConfig_new.img_length;
        }
        output[2] = idx;
        output[3] = idx + patchConfig.patch_col_rso - 1;
        if (output[3] >= lidarConfig_new.img_length) {
            output[3] = lidarConfig.img_length - 1;
        }
        output[1] = output[3] + patchConfig.col_overlap_step;
        if (output[1] >= lidarConfig_new.img_length) {
            output[1] = output[1] - lidarConfig_new.img_length;
        }
    }

    // output: patches
    void generate_patch_col_elements(const int &i, const int &j, int *i_idx, int *j_idx, Patches &patches) const {
        if (j_idx[0] < j_idx[1]) {
            // extended total patch
            patches.point_cloud_r = r_grid(Eigen::seq(i_idx[0], i_idx[1]), Eigen::seq(j_idx[0], j_idx[1]));
            patches.azi = azi_grid(Eigen::seq(i_idx[0], i_idx[1]), Eigen::seq(j_idx[0], j_idx[1]));
            patches.elev = elev_grid(Eigen::seq(i_idx[0], i_idx[1]), Eigen::seq(j_idx[0], j_idx[1]));
            patches.polar_idx = index_grid(Eigen::seq(i_idx[0], i_idx[1]), Eigen::seq(j_idx[0], j_idx[1]));
            // original patch
            patches.point_cloud_ori_r = r_grid(Eigen::seq(i_idx[2], i_idx[3]), Eigen::seq(j_idx[2], j_idx[3]));
            patches.azi_ori = azi_grid(Eigen::seq(i_idx[2], i_idx[3]), Eigen::seq(j_idx[2], j_idx[3]));
            patches.elev_ori = elev_grid(Eigen::seq(i_idx[2], i_idx[3]), Eigen::seq(j_idx[2], j_idx[3]));
            patches.polar_ori_idx = index_grid(Eigen::seq(i_idx[2], i_idx[3]), Eigen::seq(j_idx[2], j_idx[3]));
            // training patch
            patches.training_r = Eigen::MatrixXd::Zero(patches.point_cloud_r.rows(), patches.point_cloud_r.cols());
            patches.testing_r = Eigen::MatrixXd::Zero(patches.point_cloud_r.rows(), patches.point_cloud_r.cols());
            int n;
            if ((i - patchConfig.row_overlap_step) < 0) {
                n = std::min(std::floor(patchConfig.patch_row_rso / 2.0), std::floor(
                        (int(patches.point_cloud_r.cols()) - 2 * patchConfig.col_overlap_step + 1) / 2.0));
                for (int l = 1; l <= n; ++l) {
                    patches.testing_r(i + 2 * l, patchConfig.col_overlap_step + 2 * l - 2) = patches.point_cloud_r(
                            i + 2 * l, patchConfig.col_overlap_step + 2 * l - 2);
                    patches.testing_r(i + 2 * l, patches.point_cloud_r.cols() - patchConfig.col_overlap_step - 2 * l +
                                                 1) = patches.point_cloud_r(i + 2 * l, patches.point_cloud_r.cols() -
                                                                                       patchConfig.col_overlap_step -
                                                                                       2 * l + 1);
                }
                for (int k = 0; k < patches.testing_r.size(); ++k) {
                    if (*(patches.testing_r.data() + k) == 0) {
                        *(patches.training_r.data() + k) = *(patches.point_cloud_r.data() + k);
                    }
                }
                patches.training_ori_r = patches.training_r(Eigen::seq(i, i + patchConfig.patch_row_rso - 1),
                                                            Eigen::seq(patchConfig.col_overlap_step,
                                                                       patches.training_r.cols() -
                                                                       patchConfig.col_overlap_step - 1));

            } else if ((i_idx[1] - i_idx[0] - patchConfig.row_overlap_step + 1) < patchConfig.patch_row_rso) {
                n = std::min(std::floor((i_idx[1] - i_idx[0] - patchConfig.row_overlap_step + 1) / 2.0), std::floor(
                        (int(patches.point_cloud_r.cols()) - 2 * patchConfig.col_overlap_step + 1) / 2.0));
                for (int l = 1; l <= n; ++l) {
                    patches.testing_r(patchConfig.row_overlap_step + 2 * l - 1,
                                      patchConfig.col_overlap_step + 2 * l - 2) = patches.point_cloud_r(
                            patchConfig.row_overlap_step + 2 * l - 1, patchConfig.col_overlap_step + 2 * l - 2);
                    patches.testing_r(patchConfig.row_overlap_step + 2 * l - 1,
                                      patches.point_cloud_r.cols() - patchConfig.col_overlap_step - 2 * l +
                                      1) = patches.point_cloud_r(patchConfig.row_overlap_step + 2 * l - 1,
                                                                 patches.point_cloud_r.cols() -
                                                                 patchConfig.col_overlap_step - 2 * l + 1);
                }
                for (int k = 0; k < patches.testing_r.size(); ++k) {
                    if (*(patches.testing_r.data() + k) == 0) {
                        *(patches.training_r.data() + k) = *(patches.point_cloud_r.data() + k);
                    }
                }
                patches.training_ori_r = patches.training_r(
                        Eigen::seq(patchConfig.row_overlap_step, patches.training_r.rows() - 1),
                        Eigen::seq(patchConfig.col_overlap_step,
                                   patches.training_r.cols() - patchConfig.col_overlap_step - 1));

            } else {
                n = std::min(std::floor(patchConfig.patch_row_rso / 2.0), std::floor(
                        (int(patches.point_cloud_r.cols()) - 2 * patchConfig.col_overlap_step + 1) / 2.0));
                for (int l = 1; l <= n; ++l) {
                    patches.testing_r(patchConfig.row_overlap_step + 2 * l - 1,
                                      patchConfig.col_overlap_step + 2 * l - 2) = patches.point_cloud_r(
                            patchConfig.row_overlap_step + 2 * l - 1, patchConfig.col_overlap_step + 2 * l - 2);
                    patches.testing_r(patchConfig.row_overlap_step + 2 * l - 1,
                                      patches.point_cloud_r.cols() - patchConfig.col_overlap_step - 2 * l +
                                      1) = patches.point_cloud_r(patchConfig.row_overlap_step + 2 * l - 1,
                                                                 patches.point_cloud_r.cols() -
                                                                 patchConfig.col_overlap_step - 2 * l + 1);
                }
                for (int k = 0; k < patches.testing_r.size(); ++k) {
                    if (*(patches.testing_r.data() + k) == 0) {
                        *(patches.training_r.data() + k) = *(patches.point_cloud_r.data() + k);
                    }
                }
                patches.training_ori_r = patches.training_r(Eigen::seq(patchConfig.row_overlap_step,
                                                                       patchConfig.row_overlap_step +
                                                                       patchConfig.patch_row_rso - 1),
                                                            Eigen::seq(patchConfig.col_overlap_step,
                                                                       patches.point_cloud_r.cols() -
                                                                       patchConfig.col_overlap_step - 1));
            }
        } else if (j_idx[0] > j_idx[1]) {
            // extended total patch
            patches.point_cloud_r.resize(i_idx[1] - i_idx[0] + 1,
                                         patchConfig.patch_col_rso + patchConfig.col_overlap_step * 2);
            patches.azi.resize(i_idx[1] - i_idx[0] + 1,
                               patchConfig.patch_col_rso + patchConfig.col_overlap_step * 2);
            patches.elev.resize(i_idx[1] - i_idx[0] + 1,
                                patchConfig.patch_col_rso + patchConfig.col_overlap_step * 2);
            patches.polar_idx.resize(i_idx[1] - i_idx[0] + 1,
                                     patchConfig.patch_col_rso + patchConfig.col_overlap_step * 2);
            patches.point_cloud_r
                    << r_grid(Eigen::seq(i_idx[0], i_idx[1]),
                              Eigen::seq(j_idx[0], lidarConfig_new.img_length - 1)), r_grid(
                    Eigen::seq(i_idx[0], i_idx[1]), Eigen::seq(0, j_idx[1]));
            patches.azi << azi_grid(Eigen::seq(i_idx[0], i_idx[1]),
                                    Eigen::seq(j_idx[0], lidarConfig_new.img_length - 1)), azi_grid(
                    Eigen::seq(i_idx[0], i_idx[1]), Eigen::seq(0, j_idx[1]));
            patches.elev << elev_grid(Eigen::seq(i_idx[0], i_idx[1]),
                                      Eigen::seq(j_idx[0], lidarConfig_new.img_length - 1)), elev_grid(
                    Eigen::seq(i_idx[0], i_idx[1]), Eigen::seq(0, j_idx[1]));
            patches.polar_idx << index_grid(Eigen::seq(i_idx[0], i_idx[1]),
                                            Eigen::seq(j_idx[0], lidarConfig_new.img_length - 1)), index_grid(
                    Eigen::seq(i_idx[0], i_idx[1]), Eigen::seq(0, j_idx[1]));
            // original patch
            patches.point_cloud_ori_r.resize(i_idx[3] - i_idx[2] + 1, j_idx[3] - j_idx[2] + 1);
            patches.azi_ori.resize(i_idx[3] - i_idx[2] + 1, j_idx[3] - j_idx[2] + 1);
            patches.elev_ori.resize(i_idx[3] - i_idx[2] + 1, j_idx[3] - j_idx[2] + 1);
            patches.polar_ori_idx.resize(i_idx[3] - i_idx[2] + 1, j_idx[3] - j_idx[2] + 1);
            patches.point_cloud_ori_r << r_grid(Eigen::seq(i_idx[2], i_idx[3]), Eigen::seq(j_idx[2], j_idx[3]));
            patches.azi_ori << azi_grid(Eigen::seq(i_idx[2], i_idx[3]), Eigen::seq(j_idx[2], j_idx[3]));
            patches.elev_ori << elev_grid(Eigen::seq(i_idx[2], i_idx[3]), Eigen::seq(j_idx[2], j_idx[3]));
            patches.polar_ori_idx << index_grid(Eigen::seq(i_idx[2], i_idx[3]), Eigen::seq(j_idx[2], j_idx[3]));
            // training patch
            patches.training_r = Eigen::MatrixXd::Zero(patches.point_cloud_r.rows(), patches.point_cloud_r.cols());
            patches.testing_r = Eigen::MatrixXd::Zero(patches.point_cloud_r.rows(), patches.point_cloud_r.cols());
            int n;
            if ((i - patchConfig.row_overlap_step) < 0) {
                n = std::min(std::floor(patchConfig.patch_row_rso / 2.0),
                             std::floor(
                                     (int(patches.point_cloud_r.cols()) - 2 * patchConfig.col_overlap_step + 1) / 2.0));
                for (int l = 1; l <= n; ++l) {
                    patches.testing_r(i + 2 * l, patchConfig.col_overlap_step + 2 * l - 2) = patches.point_cloud_r(
                            i + 2 * l, patchConfig.col_overlap_step + 2 * l - 2);


                    patches.testing_r(i + 2 * l, patches.point_cloud_r.cols() - patchConfig.col_overlap_step - 2 * l +
                                                 1) = patches.point_cloud_r(i + 2 * l,
                                                                            patches.point_cloud_r.cols() -
                                                                            patchConfig.col_overlap_step - 2 * l + 1);
                }
                for (int k = 0; k < patches.testing_r.size(); ++k) {
                    if (*(patches.testing_r.data() + k) == 0) {
                        *(patches.training_r.data() + k) = *(patches.point_cloud_r.data() + k);
                    }
                }
                patches.training_ori_r = patches.training_r(Eigen::seq(i, i + patchConfig.patch_row_rso - 1),
                                                            Eigen::seq(patchConfig.col_overlap_step,
                                                                       patches.training_r.cols() -
                                                                       patchConfig.col_overlap_step - 1));
            } else if ((i_idx[1] - i_idx[0] - patchConfig.row_overlap_step + 1) < patchConfig.patch_row_rso) {
                n = std::min(std::floor((i_idx[1] - i_idx[0] - patchConfig.row_overlap_step + 1) / 2.0),
                             std::floor(
                                     (int(patches.point_cloud_r.cols()) - 2 * patchConfig.col_overlap_step + 1) / 2.0));
                for (int l = 1; l <= n; ++l) {
                    patches.testing_r(patchConfig.row_overlap_step + 2 * l - 1,
                                      patchConfig.col_overlap_step + 2 * l - 2) = patches.point_cloud_r(
                            patchConfig.row_overlap_step + 2 * l - 1, patchConfig.col_overlap_step + 2 * l - 2);
                    patches.testing_r(patchConfig.row_overlap_step + 2 * l - 1,
                                      patches.point_cloud_r.cols() - patchConfig.col_overlap_step - 2 * l +
                                      1) = patches.point_cloud_r(patchConfig.row_overlap_step + 2 * l - 1,
                                                                 patches.point_cloud_r.cols() -
                                                                 patchConfig.col_overlap_step - 2 * l + 1);
                }
                for (int k = 0; k < patches.testing_r.size(); ++k) {
                    if (*(patches.testing_r.data() + k) == 0) {
                        *(patches.training_r.data() + k) = *(patches.point_cloud_r.data() + k);
                    }
                }
                patches.training_ori_r = patches.training_r(
                        Eigen::seq(patchConfig.row_overlap_step, patches.training_r.rows() - 1),
                        Eigen::seq(patchConfig.col_overlap_step,
                                   patches.training_r.cols() - patchConfig.col_overlap_step));

            } else {
                n = std::min(std::floor(patchConfig.patch_row_rso / 2.0), std::floor(
                        (int(patches.point_cloud_r.cols()) - 2 * patchConfig.col_overlap_step + 1) / 2.0));
                for (int l = 1; l <= n; ++l) {
                    patches.testing_r(patchConfig.row_overlap_step + 2 * l - 1,
                                      patchConfig.col_overlap_step + 2 * l - 2) = patches.point_cloud_r(
                            patchConfig.row_overlap_step + 2 * l - 1, patchConfig.col_overlap_step + 2 * l - 2);
                    patches.testing_r(patchConfig.row_overlap_step + 2 * l - 1,
                                      patches.point_cloud_r.cols() - patchConfig.col_overlap_step - 2 * l +
                                      1) = patches.point_cloud_r(patchConfig.row_overlap_step + 2 * l - 1,
                                                                 patches.point_cloud_r.cols() -
                                                                 patchConfig.col_overlap_step - 2 * l + 1);
                }
                for (int k = 0; k < patches.testing_r.size(); ++k) {
                    if (*(patches.testing_r.data() + k) == 0) {
                        *(patches.training_r.data() + k) = *(patches.point_cloud_r.data() + k);
                    }
                }
                patches.training_ori_r = patches.training_r(Eigen::seq(patchConfig.row_overlap_step,
                                                                       patchConfig.row_overlap_step +
                                                                       patchConfig.patch_row_rso - 1),
                                                            Eigen::seq(patchConfig.col_overlap_step,
                                                                       patches.training_r.cols() -
                                                                       patchConfig.col_overlap_step - 1));
            }
        } else {
            std::cout << "Patch mush larger than 1" << std::endl;
            return;
        }
        patches.point_cloud_r.transposeInPlace();
        patches.point_cloud_ori_r.transposeInPlace();
        patches.training_r.transposeInPlace();
        patches.training_ori_r.transposeInPlace();
        patches.testing_r.transposeInPlace();
        patches.azi.transposeInPlace();
        patches.elev.transposeInPlace();
        patches.polar_idx.transposeInPlace();
        patches.azi_ori.transposeInPlace();
        patches.elev_ori.transposeInPlace();
        patches.polar_ori_idx.transposeInPlace();

        patches.point_cloud_r_col = Eigen::Map<Eigen::VectorXd>(patches.point_cloud_r.data(),
                                                                patches.point_cloud_r.size());
        patches.point_cloud_ori_r_col = Eigen::Map<Eigen::VectorXd>(patches.point_cloud_ori_r.data(),
                                                                    patches.point_cloud_ori_r.size());
        patches.training_r_col = Eigen::Map<Eigen::VectorXd>(patches.training_r.data(), patches.training_r.size());
        patches.training_ori_r_col = Eigen::Map<Eigen::VectorXd>(patches.training_ori_r.data(),
                                                                 patches.training_ori_r.size());
        patches.testing_r_col = Eigen::Map<Eigen::VectorXd>(patches.testing_r.data(), patches.testing_r.size());
        patches.polar.resize(patches.azi.size(), 2);
        patches.polar_ori.resize(patches.azi_ori.size(), 2);
        patches.polar
                << Eigen::Map<Eigen::MatrixXd>(patches.azi.data(), patches.azi.size(), 1), Eigen::Map<Eigen::MatrixXd>(
                patches.elev.data(), patches.elev.size(), 1);
        patches.polar_ori << Eigen::Map<Eigen::MatrixXd>(patches.azi_ori.data(), patches.azi_ori.size(),
                                                         1), Eigen::Map<Eigen::MatrixXd>(patches.elev_ori.data(),
                                                                                         patches.elev_ori.size(), 1);
        patches.polar_idx_col = Eigen::Map<Eigen::VectorXd>(patches.polar_idx.data(), patches.polar_idx.size());
        patches.polar_ori_idx_col = Eigen::Map<Eigen::VectorXd>(patches.polar_ori_idx.data(),
                                                                patches.polar_ori_idx.size());
        patches.point_cloud_r.transposeInPlace();
        patches.point_cloud_ori_r.transposeInPlace();
        patches.training_r.transposeInPlace();
        patches.training_ori_r.transposeInPlace();
        patches.testing_r.transposeInPlace();
        patches.azi.transposeInPlace();
        patches.elev.transposeInPlace();
        patches.polar_idx.transposeInPlace();
        patches.azi_ori.transposeInPlace();
        patches.elev_ori.transposeInPlace();
        patches.polar_ori_idx.transposeInPlace();
    }

    void extract_coefficients(const double &thres_training, const double &thres_filtering) {
//        omp_lock_t writelock;
//        omp_init_lock(&writelock);
#pragma omp parallel for num_threads(omp_get_num_procs()) default(none) shared( i_sequence, j_sequence, thres_training, thres_filtering, coefficients, Eigen::all)
        for (int i = 0; i < i_sequence.size(); ++i) {
            for (int j = 0; j < j_sequence.size(); ++j) {
//                std::cout << "i: " << i << "j: " << j << std::endl;
                // parameters has to be initilized in the for loop when use parallel for
                int i_idx[4]; // idx_low idx_high idx_low_ori idx_high_ori
                int j_idx[4];
                int degree_iterative = 0;
                int table_id;
                double training_error = std::numeric_limits<double>::max();
                double testing_error = std::numeric_limits<double>::max();
                double total_error = std::numeric_limits<double>::max();
                double last_total_error = std::numeric_limits<double>::max();
                Eigen::VectorXd final_error_vec;
                Eigen::VectorXi mask_mid;
                Patches patches;
                Eigen::VectorXd training_recons;
                Eigen::VectorXd testing_recons;
                Eigen::VectorXd final_recons;
                Eigen::MatrixXd SH_tmp_matrix;
                Eigen::MatrixXd last_SH_tmp_matrix;
                Eigen::MatrixXd ori_idx_mid;
                std::vector<int> indices;
                enlarge_patches_row(lidarConfig_new, i_sequence(i), i_idx);
                enlarge_patches_col(lidarConfig_new, j_sequence(j), j_idx);
                generate_patch_col_elements(i_sequence(i), j_sequence(j), i_idx, j_idx, patches);
                if (patches.training_ori_r_col.sum() > 0) {
                    degree_iterative = patchConfig.degree_initial;
                    while (true) {
                        table_id = (degree_iterative - patchConfig.degree_initial) / patchConfig.degree_iter_step;
                        // find indices that are useful
                        indices.clear();
                        for (int k = 0; k < patches.training_r_col.size(); ++k) {
                            if (patches.training_r_col(k) > 0) {
                                indices.push_back(k);
                            }
                        }
                        /**************************************************************/
                        if ((table_id + 1) > SH_table_up_limit) {
                            SH_tmp_matrix = getSH(degree_iterative, patches.polar);
                            coefficients[i * j_sequence.size() + j] = SH_tmp_matrix(indices, Eigen::all).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(patches.training_r_col(indices));
                            training_recons = SH_tmp_matrix(indices, Eigen::all) * coefficients[i * j_sequence.size() + j];
                            /**************************************************************/
                        } else {
//                             check whether the table already have the value
                            for (int k = 0; k < patches.polar_idx_col.size(); ++k) {
                                if (SH_table[table_id](int(patches.polar_idx_col(k)), Eigen::all).sum() == 0) {
                                    SH_table[table_id](patches.polar_idx_col,
                                                       Eigen::all) = getSH(
                                            degree_iterative, patches.polar);
                                    break;
                                }
                            }
                            coefficients[i * j_sequence.size() + j] = SH_table[table_id](patches.polar_idx_col(indices), Eigen::all).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(
                                    patches.training_r_col(indices));
                            training_recons = SH_table[table_id](patches.polar_idx_col(indices), Eigen::all) * coefficients[i * j_sequence.size() + j];
                        }
                        training_error = (patches.training_r_col(indices) - training_recons).array().abs().mean();
                        if (training_error <= thres_training) {
                            // find indices that are useful
                            indices.clear();
                            for (int k = 0; k < patches.point_cloud_r_col.size(); ++k) {
                                if (patches.point_cloud_r_col(k) > 0) {
                                    indices.push_back(k);
                                }
                            }
                            /**************************************************************/
                            if ((table_id + 1) > SH_table_up_limit) {
                                coefficients[i * j_sequence.size() + j] = SH_tmp_matrix(indices, Eigen::all).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(
                                        patches.point_cloud_r_col(indices));
                                // find index of polar_ori in polar
                                patches.ori_idx = Eigen::VectorXd::LinSpaced(patches.point_cloud_r.size(), 0, patches.point_cloud_r.size() - 1);
                                patches.ori_idx = Eigen::Map<Eigen::MatrixXd>(patches.ori_idx.data(), patches.point_cloud_r.cols(), patches.point_cloud_r.rows());
                                patches.ori_idx.transposeInPlace();
                                patches.ori_idx = patches.ori_idx(Eigen::seq(int(i_idx[2] - i_idx[0]), int(patches.point_cloud_r.rows() - (i_idx[1] - i_idx[3]) - 1)),
                                                                  Eigen::seq(int(patchConfig.col_overlap_step), int(patches.point_cloud_r.cols() - patchConfig.col_overlap_step - 1)));
                                patches.ori_idx.transposeInPlace();
                                patches.ori_idx_col = Eigen::Map<Eigen::VectorXd>(patches.ori_idx.data(), patches.ori_idx.size());

                                final_recons = SH_tmp_matrix(patches.ori_idx_col, Eigen::all) * coefficients[i * j_sequence.size() + j];
                                /**************************************************************/
                            } else {
                                // check whether the table already have the value
                                coefficients[i * j_sequence.size() + j] = SH_table[table_id](patches.polar_idx_col(indices), Eigen::all).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(
                                        patches.point_cloud_r_col(indices));
                                // check whether the table already have the value
                                final_recons = SH_table[table_id](patches.polar_ori_idx_col, Eigen::all) * coefficients[i * j_sequence.size() + j];
                            }
                            final_error_vec = (final_recons - patches.point_cloud_ori_r_col).array().abs();
                            mask_mid = Eigen::VectorXi::Zero(final_error_vec.size());
                            for (int k = 0; k < final_error_vec.size(); ++k) {
                                if ((final_error_vec(k) < thres_filtering) ||
                                    (patches.point_cloud_ori_r_col(k) == 0 && final_recons(k) > lidarConfig.min_length && final_recons(k) < lidarConfig.max_length)) {
                                    mask_mid[k] = 1;
                                }
                            }
                            mask_recons_update_col(patches.polar_ori_idx_col) = mask_mid.array() * mask_recons_update_col(patches.polar_ori_idx_col).array();
                            break;
                        }
                        if (patches.testing_r_col.sum() > 0) {
                            // find indices that are useful
                            indices.clear();
                            for (int k = 0; k < patches.testing_r_col.size(); ++k) {
                                if (patches.testing_r_col(k) > 0) {
                                    indices.push_back(k);
                                }
                            }
                            /**************************************************************/
                            if ((table_id + 1) > SH_table_up_limit) {
                                testing_recons = SH_tmp_matrix(indices, Eigen::all) * coefficients[i * j_sequence.size() + j];
                                /**************************************************************/
                            } else {
                                // check whether the table already have the value
                                testing_recons = SH_table[table_id](patches.polar_idx_col(indices), Eigen::all) * coefficients[i * j_sequence.size() + j];
                            }
                            testing_error = (patches.testing_r_col(indices) - testing_recons).array().abs().mean();
                        } else { testing_error = 0; }
                        total_error = w(degree_iterative, patchConfig.k) * training_error + (1 - w(degree_iterative, patchConfig.k)) * testing_error;
                        if (total_error >= last_total_error || degree_iterative > patchConfig.degree_max) {
                            degree_iterative -= patchConfig.degree_iter_step;
                            table_id = (degree_iterative - patchConfig.degree_initial) / patchConfig.degree_iter_step;
                            // find indices that are useful
                            indices.clear();
                            for (int k = 0; k < patches.point_cloud_r_col.size(); ++k) {
                                if (patches.point_cloud_r_col(k) > 0) {
                                    indices.push_back(k);
                                }
                            }
                            /**************************************************************/
                            if ((table_id + 1) > SH_table_up_limit) {
                                coefficients[i * j_sequence.size() + j] = last_SH_tmp_matrix(indices, Eigen::all).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(
                                        patches.point_cloud_r_col(indices));
                                // find index of polar_ori in polar
                                patches.ori_idx = Eigen::VectorXd::LinSpaced(patches.point_cloud_r.size(), 0, patches.point_cloud_r.size() - 1);
                                patches.ori_idx = Eigen::Map<Eigen::MatrixXd>(patches.ori_idx.data(), patches.point_cloud_r.cols(), patches.point_cloud_r.rows());
                                patches.ori_idx.transposeInPlace();
                                patches.ori_idx = patches.ori_idx(Eigen::seq(int(i_idx[2] - i_idx[0]), int(patches.point_cloud_r.rows() - (i_idx[1] - i_idx[3]) - 1)),
                                                                  Eigen::seq(int(patchConfig.col_overlap_step), int(patches.point_cloud_r.cols() - patchConfig.col_overlap_step - 1)));
                                patches.ori_idx.transposeInPlace();
                                patches.ori_idx_col = Eigen::Map<Eigen::VectorXd>(patches.ori_idx.data(), patches.ori_idx.size());

                                final_recons = last_SH_tmp_matrix(patches.ori_idx_col, Eigen::all) * coefficients[i * j_sequence.size() + j];
                                /**************************************************************/
                            } else {
                                // check whether the table already have the value
                                coefficients[i * j_sequence.size() + j] = SH_table[table_id](patches.polar_idx_col(indices), Eigen::all).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(
                                        patches.point_cloud_r_col(indices));
                                // check whether the table already have the value
                                final_recons = SH_table[table_id](patches.polar_ori_idx_col, Eigen::all) * coefficients[i * j_sequence.size() + j];
                            }
                            final_error_vec = (final_recons - patches.point_cloud_ori_r_col).array().abs();
                            mask_mid = Eigen::VectorXi::Zero(final_error_vec.size());
                            for (int k = 0; k < final_error_vec.size(); ++k) {
                                if ((final_error_vec(k) < thres_filtering) ||
                                    (patches.point_cloud_ori_r_col(k) == 0 && final_recons(k) > lidarConfig.min_length && final_recons(k) < lidarConfig.max_length)) {
                                    mask_mid[k] = 1;
                                }
                            }
                            mask_recons_update_col(patches.polar_ori_idx_col) = mask_mid.array() * mask_recons_update_col(patches.polar_ori_idx_col).array();
                            break;
                        }
                        last_total_error = total_error;
                        degree_iterative += patchConfig.degree_iter_step;
                        if ((table_id + 1) > SH_table_up_limit) {
                            last_SH_tmp_matrix = SH_tmp_matrix;
                        }
                    }
                } else if (patches.point_cloud_ori_r_col.sum() > 0) {
                    degree_iterative = patchConfig.degree_initial;
                    while (true) {
                        table_id = (degree_iterative - patchConfig.degree_initial) / patchConfig.degree_iter_step;
                        // find indices that are useful
                        indices.clear();
                        for (int k = 0; k < patches.point_cloud_r_col.size(); ++k) {
                            if (patches.point_cloud_r_col(k) > 0) {
                                indices.push_back(k);
                            }
                        }
                        /**************************************************************/
                        if ((table_id + 1) > SH_table_up_limit) {
                            SH_tmp_matrix = getSH(degree_iterative, patches.polar);
                            coefficients[i * j_sequence.size() + j] = SH_tmp_matrix(indices, Eigen::all).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(patches.point_cloud_r_col(indices));
                            training_recons = SH_tmp_matrix(indices, Eigen::all) * coefficients[i * j_sequence.size() + j];
                            /**************************************************************/
                        } else {
//                             check whether the table already have the value
                            for (int k = 0; k < patches.polar_idx_col.size(); ++k) {
                                if (SH_table[table_id](int(patches.polar_idx_col(k)), Eigen::all).sum() == 0) {
                                    SH_table[table_id](patches.polar_idx_col,
                                                       Eigen::all) = getSH(
                                            degree_iterative, patches.polar);
                                    break;
                                }
                            }
                            coefficients[i * j_sequence.size() + j] = SH_table[table_id](patches.polar_idx_col(indices), Eigen::all).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(
                                    patches.point_cloud_r_col(indices));
                            training_recons = SH_table[table_id](patches.polar_idx_col(indices), Eigen::all) * coefficients[i * j_sequence.size() + j];
                        }
                        training_error = (patches.point_cloud_r_col(indices) - training_recons).array().abs().mean();
                        if (training_error <= thres_training) {
                            /**************************************************************/
                            if ((table_id + 1) > SH_table_up_limit) {
                                // find index of polar_ori in polar
                                patches.ori_idx = Eigen::VectorXd::LinSpaced(patches.point_cloud_r.size(), 0, patches.point_cloud_r.size() - 1);
                                patches.ori_idx = Eigen::Map<Eigen::MatrixXd>(patches.ori_idx.data(), patches.point_cloud_r.cols(), patches.point_cloud_r.rows());
                                patches.ori_idx.transposeInPlace();
                                patches.ori_idx = patches.ori_idx(Eigen::seq(int(i_idx[2] - i_idx[0]), int(patches.point_cloud_r.rows() - (i_idx[1] - i_idx[3]) - 1)),
                                                                  Eigen::seq(int(patchConfig.col_overlap_step), int(patches.point_cloud_r.cols() - patchConfig.col_overlap_step - 1)));
                                patches.ori_idx.transposeInPlace();
                                patches.ori_idx_col = Eigen::Map<Eigen::VectorXd>(patches.ori_idx.data(), patches.ori_idx.size());

                                final_recons = SH_tmp_matrix(patches.ori_idx_col, Eigen::all) * coefficients[i * j_sequence.size() + j];
                                /**************************************************************/
                            } else {
                                // check whether the table already have the value
                                final_recons = SH_table[table_id](patches.polar_ori_idx_col, Eigen::all) * coefficients[i * j_sequence.size() + j];
                            }
                            final_error_vec = (final_recons - patches.point_cloud_ori_r_col).array().abs();
                            mask_mid = Eigen::VectorXi::Zero(final_error_vec.size());
                            for (int k = 0; k < final_error_vec.size(); ++k) {
                                if ((final_error_vec(k) < thres_filtering) ||
                                    (patches.point_cloud_ori_r_col(k) == 0 && final_recons(k) > lidarConfig.min_length && final_recons(k) < lidarConfig.max_length)) {
                                    mask_mid[k] = 1;
                                }
                            }
                            mask_recons_update_col(patches.polar_ori_idx_col) = mask_mid.array() * mask_recons_update_col(patches.polar_ori_idx_col).array();
                            break;
                        }
                        total_error = training_error;
                        if (total_error >= last_total_error || degree_iterative > patchConfig.degree_max) {
                            degree_iterative -= patchConfig.degree_iter_step;
                            table_id = (degree_iterative - patchConfig.degree_initial) / patchConfig.degree_iter_step;
                            // find indices that are useful
                            indices.clear();
                            for (int k = 0; k < patches.point_cloud_r_col.size(); ++k) {
                                if (patches.point_cloud_r_col(k) > 0) {
                                    indices.push_back(k);
                                }
                            }
                            /**************************************************************/
                            if ((table_id + 1) > SH_table_up_limit) {
                                coefficients[i * j_sequence.size() + j] = last_SH_tmp_matrix(indices, Eigen::all).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(
                                        patches.point_cloud_r_col(indices));
                                // find index of polar_ori in polar
                                patches.ori_idx = Eigen::VectorXd::LinSpaced(patches.point_cloud_r.size(), 0, patches.point_cloud_r.size() - 1);
                                patches.ori_idx = Eigen::Map<Eigen::MatrixXd>(patches.ori_idx.data(), patches.point_cloud_r.cols(), patches.point_cloud_r.rows());
                                patches.ori_idx.transposeInPlace();
                                patches.ori_idx = patches.ori_idx(Eigen::seq(int(i_idx[2] - i_idx[0]), int(patches.point_cloud_r.rows() - (i_idx[1] - i_idx[3]) - 1)),
                                                                  Eigen::seq(int(patchConfig.col_overlap_step), int(patches.point_cloud_r.cols() - patchConfig.col_overlap_step - 1)));
                                patches.ori_idx.transposeInPlace();
                                patches.ori_idx_col = Eigen::Map<Eigen::VectorXd>(patches.ori_idx.data(), patches.ori_idx.size());

                                final_recons = last_SH_tmp_matrix(patches.ori_idx_col, Eigen::all) * coefficients[i * j_sequence.size() + j];
                                /**************************************************************/
                            } else {
                                // check whether the table already have the value
                                coefficients[i * j_sequence.size() + j] = SH_table[table_id](patches.polar_idx_col(indices), Eigen::all).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(
                                        patches.point_cloud_r_col(indices));
                                // check whether the table already have the value
                                final_recons = SH_table[table_id](patches.polar_ori_idx_col, Eigen::all) * coefficients[i * j_sequence.size() + j];
                            }
                            final_error_vec = (final_recons - patches.point_cloud_ori_r_col).array().abs();
                            mask_mid = Eigen::VectorXi::Zero(final_error_vec.size());
                            for (int k = 0; k < final_error_vec.size(); ++k) {
                                if ((final_error_vec(k) < thres_filtering) ||
                                    (patches.point_cloud_ori_r_col(k) == 0 && final_recons(k) > lidarConfig.min_length && final_recons(k) < lidarConfig.max_length)) {
                                    mask_mid[k] = 1;
                                }
                            }
                            mask_recons_update_col(patches.polar_ori_idx_col) = mask_mid.array() * mask_recons_update_col(patches.polar_ori_idx_col).array();
                            break;
                        }
                        last_total_error = total_error;
                        degree_iterative += patchConfig.degree_iter_step;
                        if ((table_id + 1) > SH_table_up_limit) {
                            last_SH_tmp_matrix = SH_tmp_matrix;
                        }
                    }
                }
            }
        }
//        omp_destroy_lock(&writelock);
    }

    void extract_coefficients_1_1(const double &thres_training, const double &thres_filtering) {
//        omp_lock_t writelock;
//        omp_init_lock(&writelock);
#pragma omp parallel for num_threads(omp_get_num_procs()) default(none) shared( i_sequence, j_sequence, thres_training, thres_filtering, coefficients, Eigen::all)
        for (int i = 0; i < i_sequence.size(); ++i) {
            for (int j = 0; j < j_sequence.size(); ++j) {
//                std::cout << "i: " << i << "j: " << j << std::endl;
                // parameters has to be initilized in the for loop when use parallel for
                int i_idx[4]; // idx_low idx_high idx_low_ori idx_high_ori
                int j_idx[4];
                int degree_iterative = 0;
                int table_id;
                double training_error = std::numeric_limits<double>::max();
                double testing_error = std::numeric_limits<double>::max();
                double total_error = std::numeric_limits<double>::max();
                double last_total_error = std::numeric_limits<double>::max();
                Eigen::VectorXd final_error_vec;
                Eigen::VectorXi mask_mid;
                Patches patches;
                Eigen::VectorXd training_recons;
                Eigen::VectorXd testing_recons;
                Eigen::VectorXd final_recons;
                Eigen::MatrixXd SH_tmp_matrix;
                Eigen::MatrixXd last_SH_tmp_matrix;
                Eigen::MatrixXd ori_idx_mid;
                std::vector<int> indices;
                enlarge_patches_row(lidarConfig_new, i_sequence(i), i_idx);
                enlarge_patches_col(lidarConfig_new, j_sequence(j), j_idx);
                generate_patch_col_elements(i_sequence(i), j_sequence(j), i_idx, j_idx, patches);
                if (patches.point_cloud_ori_r_col.sum() > 0) {
                    degree_iterative = patchConfig.degree_initial;
                    while (true) {
                        table_id = (degree_iterative - patchConfig.degree_initial) / patchConfig.degree_iter_step;
                        // find indices that are useful
                        indices.clear();
                        for (int k = 0; k < patches.point_cloud_r_col.size(); ++k) {
                            if (patches.point_cloud_r_col(k) > 0) {
                                indices.push_back(k);
                            }
                        }
                        /**************************************************************/
                        if ((table_id + 1) > SH_table_up_limit) {
                            SH_tmp_matrix = getSH(degree_iterative, patches.polar);
                            coefficients[i * j_sequence.size() + j] = SH_tmp_matrix(indices, Eigen::all).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(patches.point_cloud_r_col(indices));
                            training_recons = SH_tmp_matrix(indices, Eigen::all) * coefficients[i * j_sequence.size() + j];
                            /**************************************************************/
                        } else {
//                             check whether the table already have the value
                            for (int k = 0; k < patches.polar_idx_col.size(); ++k) {
                                if (SH_table[table_id](int(patches.polar_idx_col(k)), Eigen::all).sum() == 0) {
                                    SH_table[table_id](patches.polar_idx_col,
                                                       Eigen::all) = getSH(
                                            degree_iterative, patches.polar);
                                    break;
                                }
                            }
                            coefficients[i * j_sequence.size() + j] = SH_table[table_id](patches.polar_idx_col(indices), Eigen::all).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(
                                    patches.point_cloud_r_col(indices));
                            training_recons = SH_table[table_id](patches.polar_idx_col(indices), Eigen::all) * coefficients[i * j_sequence.size() + j];
                        }
                        training_error = (patches.point_cloud_r_col(indices) - training_recons).array().abs().mean();
                        if (training_error <= thres_training) {
                            /**************************************************************/
                            if ((table_id + 1) > SH_table_up_limit) {
                                // find index of polar_ori in polar
                                patches.ori_idx = Eigen::VectorXd::LinSpaced(patches.point_cloud_r.size(), 0, patches.point_cloud_r.size() - 1);
                                patches.ori_idx = Eigen::Map<Eigen::MatrixXd>(patches.ori_idx.data(), patches.point_cloud_r.cols(), patches.point_cloud_r.rows());
                                patches.ori_idx.transposeInPlace();
                                patches.ori_idx = patches.ori_idx(Eigen::seq(int(i_idx[2] - i_idx[0]), int(patches.point_cloud_r.rows() - (i_idx[1] - i_idx[3]) - 1)),
                                                                  Eigen::seq(int(patchConfig.col_overlap_step), int(patches.point_cloud_r.cols() - patchConfig.col_overlap_step - 1)));
                                patches.ori_idx.transposeInPlace();
                                patches.ori_idx_col = Eigen::Map<Eigen::VectorXd>(patches.ori_idx.data(), patches.ori_idx.size());

                                final_recons = SH_tmp_matrix(patches.ori_idx_col, Eigen::all) * coefficients[i * j_sequence.size() + j];
                                /**************************************************************/
                            } else {
                                // check whether the table already have the value
                                final_recons = SH_table[table_id](patches.polar_ori_idx_col, Eigen::all) * coefficients[i * j_sequence.size() + j];
                            }
                            final_error_vec = (final_recons - patches.point_cloud_ori_r_col).array().abs();
                            mask_mid = Eigen::VectorXi::Zero(final_error_vec.size());
                            for (int k = 0; k < final_error_vec.size(); ++k) {
                                if ((final_error_vec(k) < thres_filtering) ||
                                    (patches.point_cloud_ori_r_col(k) == 0 && final_recons(k) > lidarConfig.min_length && final_recons(k) < lidarConfig.max_length)) {
                                    mask_mid[k] = 1;
                                }
                            }
                            mask_recons_update_col(patches.polar_ori_idx_col) = mask_mid.array() * mask_recons_update_col(patches.polar_ori_idx_col).array();
                            break;
                        }
                        if (degree_iterative > patchConfig.degree_max) {
                            degree_iterative -= patchConfig.degree_iter_step;
                            table_id = (degree_iterative - patchConfig.degree_initial) / patchConfig.degree_iter_step;
                            // find indices that are useful
                            indices.clear();
                            for (int k = 0; k < patches.point_cloud_r_col.size(); ++k) {
                                if (patches.point_cloud_r_col(k) > 0) {
                                    indices.push_back(k);
                                }
                            }
                            /**************************************************************/
                            if ((table_id + 1) > SH_table_up_limit) {
                                coefficients[i * j_sequence.size() + j] = last_SH_tmp_matrix(indices, Eigen::all).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(
                                        patches.point_cloud_r_col(indices));
                                // find index of polar_ori in polar
                                patches.ori_idx = Eigen::VectorXd::LinSpaced(patches.point_cloud_r.size(), 0, patches.point_cloud_r.size() - 1);
                                patches.ori_idx = Eigen::Map<Eigen::MatrixXd>(patches.ori_idx.data(), patches.point_cloud_r.cols(), patches.point_cloud_r.rows());
                                patches.ori_idx.transposeInPlace();
                                patches.ori_idx = patches.ori_idx(Eigen::seq(int(i_idx[2] - i_idx[0]), int(patches.point_cloud_r.rows() - (i_idx[1] - i_idx[3]) - 1)),
                                                                  Eigen::seq(int(patchConfig.col_overlap_step), int(patches.point_cloud_r.cols() - patchConfig.col_overlap_step - 1)));
                                patches.ori_idx.transposeInPlace();
                                patches.ori_idx_col = Eigen::Map<Eigen::VectorXd>(patches.ori_idx.data(), patches.ori_idx.size());

                                final_recons = last_SH_tmp_matrix(patches.ori_idx_col, Eigen::all) * coefficients[i * j_sequence.size() + j];
                                /**************************************************************/
                            } else {
                                // check whether the table already have the value
                                coefficients[i * j_sequence.size() + j] = SH_table[table_id](patches.polar_idx_col(indices), Eigen::all).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(
                                        patches.point_cloud_r_col(indices));
                                // check whether the table already have the value
                                final_recons = SH_table[table_id](patches.polar_ori_idx_col, Eigen::all) * coefficients[i * j_sequence.size() + j];
                            }
                            final_error_vec = (final_recons - patches.point_cloud_ori_r_col).array().abs();
                            mask_mid = Eigen::VectorXi::Zero(final_error_vec.size());
                            for (int k = 0; k < final_error_vec.size(); ++k) {
                                if ((final_error_vec(k) < thres_filtering) ||
                                    (patches.point_cloud_ori_r_col(k) == 0 && final_recons(k) > lidarConfig.min_length && final_recons(k) < lidarConfig.max_length)) {
                                    mask_mid[k] = 1;
                                }
                            }
                            mask_recons_update_col(patches.polar_ori_idx_col) = mask_mid.array() * mask_recons_update_col(patches.polar_ori_idx_col).array();
                            break;
                        }
                        degree_iterative += patchConfig.degree_iter_step;
                        if ((table_id + 1) > SH_table_up_limit) {
                            last_SH_tmp_matrix = SH_tmp_matrix;
                        }
                    }
                }
            }
        }
//        omp_destroy_lock(&writelock);
    }

    void processing() {
        if (patchConfig.is_1_1_recons_only) {
            patchConfig.row_overlap_step = 0;
            patchConfig.col_overlap_step = 0;
        }
        griding();
        lidarConfig_new.num_lasers = lidarConfig.num_lasers * lidarConfig.row_times;
        lidarConfig_new.img_length = lidarConfig.img_length * lidarConfig.col_times;
        patch_row = std::ceil(lidarConfig_new.num_lasers / patchConfig.patch_row_rso);
        patch_col = std::ceil(lidarConfig_new.img_length / patchConfig.patch_col_rso);
        coefficients.resize(patch_row * patch_col); //i*patch_col+j
        SH_table.resize(SH_table_up_limit);
        mask_recons_update_grid.transposeInPlace();
        mask_recons_update_col = Eigen::Map<Eigen::VectorXi>(mask_recons_update_grid.data(), mask_recons_update_grid.size());
        mask_recons_update_grid.transposeInPlace();
        for (int i = 0; i < SH_table_up_limit; i++) {
            SH_table[i] = Eigen::MatrixXd::Zero(lidarConfig_new.num_lasers * lidarConfig_new.img_length,
                                                pow(patchConfig.degree_initial + i * patchConfig.degree_iter_step +
                                                    1,
                                                    2));
        }
        i_sequence.resize(std::ceil(lidarConfig_new.num_lasers / patchConfig.patch_row_rso));
        for (int i = 0; i < i_sequence.size(); ++i) {
            i_sequence(i) = i * patchConfig.patch_row_rso;
        }
        j_sequence.resize(std::ceil(lidarConfig_new.img_length / patchConfig.patch_col_rso));
        for (int i; i < j_sequence.size(); ++i) {
            j_sequence(i) = i * patchConfig.patch_col_rso;
        }
        if (patchConfig.is_1_1_recons_only) {
            extract_coefficients_1_1(thresholdConfig.avg_filter_thres, thresholdConfig.rmv_filter_thres);
        } else {
            extract_coefficients(thresholdConfig.avg_filter_thres, thresholdConfig.rmv_filter_thres);
        }
        mask_recons_update_grid = Eigen::Map<Eigen::MatrixXi>(mask_recons_update_col.data(),
                                                              lidarConfig_new.img_length,
                                                              lidarConfig_new.num_lasers);
        mask_recons_update_grid.transposeInPlace();
        r_grid = r_grid.array() * mask_recons_update_grid.array().cast<double>();
        coefficients.clear();
        coefficients.resize(patch_row * patch_col); //i*patch_col+j
        if (patchConfig.is_1_1_recons_only) {
            extract_coefficients_1_1(thresholdConfig.avg_error_thres_patch, thresholdConfig.remove);
        } else {
            extract_coefficients(thresholdConfig.avg_error_thres_patch, thresholdConfig.remove);
        }

        mask_recons_update_grid = Eigen::Map<Eigen::MatrixXi>(mask_recons_update_col.data(),
                                                              lidarConfig_new.img_length,
                                                              lidarConfig_new.num_lasers);
        mask_recons_update_grid.transposeInPlace();
//        save_files("/home/zkc/project/libigl-example-project/tmp_data/CURL_Reconstruction_data");
    }

    void save_files(const std::string &dir) {
        std::ofstream file;
        typedef std::numeric_limits<double> dbl;
        file.open(dir + "/coefficients.txt");
        file.precision(17);
        file << i_sequence.size() << " " << j_sequence.size() << '\n';
        for (int i = 0; i < coefficients.size(); ++i) {
            for (int j = 0; j < coefficients[i].size(); ++j) {
                file << coefficients[i](j) << ' ';
            }
            file << '\n';
        }
        file.close();
    }
};

#endif //EXAMPLE_CURL_EXTRACTION_MAT_H
