//
// Created by zkc on 01/05/22.
//

#ifndef EXAMPLE_CURL_RECONSTRUCTION_MEX_H
#define EXAMPLE_CURL_RECONSTRUCTION_MEX_H

#include "ReadMatrix.h"
#include "Eigen/Core"
#include <cmath>
#include <algorithm>
#include "CURL_tools.h"
#include "Timer.h"
#include <fstream>

class CURL_Reconstruction {
public:
    // inputs
    LidarConfig lidarConfig;
    ReconsConfig reconsConfig;
    Eigen::MatrixXd table_init_SPHARM_grid;
    Eigen::MatrixXi mask_recons_update_grid;
    int patch_num_row = 0;
    int patch_num_col = 0;
    std::vector<Eigen::VectorXd> coefficients;
    // outputs
    Matrix3dCol reconsPointCloud;
    Matrix3dCol reconsPointCloudFiltered;
    // mid products
    Eigen::VectorXi mask_recons_update_col;
    int patch_row_rso = 0;
    int patch_col_rso = 0;
    Eigen::VectorXi i_sequence;
    Eigen::VectorXi j_sequence;
    Eigen::MatrixXd index_grid;
    Eigen::VectorXd r_col;

    void griding() {
        index_grid.resize(table_init_SPHARM_grid.rows(), 1);
        index_grid = Eigen::VectorXd::LinSpaced(table_init_SPHARM_grid.rows(), 0, table_init_SPHARM_grid.rows() - 1);
        index_grid = Eigen::Map<Eigen::MatrixXd>(index_grid.data(), lidarConfig.img_length * reconsConfig.col_times, lidarConfig.num_lasers * reconsConfig.row_times);
        index_grid.transposeInPlace();
    }

    void recons_patches_row(int &idx, int *i_idx) {
        i_idx[0] = idx;
        i_idx[1] = idx + patch_row_rso - 1;
        if (i_idx[1] > (lidarConfig.num_lasers * reconsConfig.row_times - 1)) {
            i_idx[1] = lidarConfig.num_lasers * reconsConfig.row_times - 1;
        }
    }

    void recons_patches_col(int &idx, int *j_idx) {
        j_idx[0] = idx;
        j_idx[1] = idx + patch_col_rso - 1;
        if (j_idx[1] > (lidarConfig.img_length * reconsConfig.col_times - 1)) {
            j_idx[1] = lidarConfig.img_length * reconsConfig.col_times - 1;
        }
    }

    void generate_patch_col_elements(int *i_idx, int *j_idx, Patches &patches) const {
        patches.polar_ori_idx = index_grid(Eigen::seq(i_idx[0], i_idx[1]), Eigen::seq(j_idx[0], j_idx[1]));
        patches.polar_ori_idx.resize(i_idx[1] - i_idx[0] + 1, j_idx[1] - j_idx[0] + 1);
        patches.polar_ori_idx << index_grid(Eigen::seq(i_idx[0], i_idx[1]), Eigen::seq(j_idx[0], j_idx[1]));
        patches.polar_ori_idx.transposeInPlace();
        patches.polar_ori_idx_col = Eigen::Map<Eigen::VectorXd>(patches.polar_ori_idx.data(), patches.polar_ori_idx.size());
        patches.polar_ori_idx.transposeInPlace();
    }

    void reconstruct_points() {
#pragma omp parallel for num_threads(omp_get_num_procs()) default(none) shared( i_sequence, j_sequence, coefficients, table_init_SPHARM_grid, r_col, Eigen::all)
        for (int i = 0; i < i_sequence.size(); ++i) {
            for (int j = 0; j < j_sequence.size(); ++j) {
                int i_idx[2]; // idx_low_ori idx_high_ori
                int j_idx[2];
                int degree;
                std::vector<int> indices;
                Patches patches;
                recons_patches_row(i_sequence(i), i_idx);
                recons_patches_col(j_sequence(j), j_idx);
                generate_patch_col_elements(i_idx, j_idx, patches);
                degree = sqrt(coefficients[i * j_sequence.size() + j].size()) - 1;
                indices.clear();
                for (int k = 0; k < patches.polar_ori_idx_col.size(); ++k) {
                    if (mask_recons_update_col(int(patches.polar_ori_idx_col(k))) > 0) {
                        indices.push_back(k);
                    }
                }
                r_col(patches.polar_ori_idx_col(indices)) = getSH(degree, table_init_SPHARM_grid(patches.polar_ori_idx_col(indices), Eigen::all)) * coefficients[i * j_sequence.size() + j];
            }
        }
    }

    void processing() {
        griding();
        patch_row_rso = std::ceil(lidarConfig.num_lasers * reconsConfig.row_times / patch_num_row);
        patch_col_rso = std::ceil(lidarConfig.img_length * reconsConfig.col_times / patch_num_col);
//        i_sequence = Eigen::VectorXi::LinSpaced(patch_num_row, 0, lidarConfig.num_lasers * reconsConfig.row_times - 1);
//        j_sequence = Eigen::VectorXi::LinSpaced(patch_num_col, 0, lidarConfig.img_length * reconsConfig.col_times - 1);
        i_sequence.resize(patch_num_row);
        j_sequence.resize(patch_num_col);
        for (int i = 0; i < patch_num_row; ++i) {
            i_sequence(i) = i * patch_row_rso;
        }
        for (int i = 0; i < patch_num_col; ++i) {
            j_sequence(i) = i * patch_col_rso;
        }
        mask_recons_update_grid.transposeInPlace();
        mask_recons_update_col = Eigen::Map<Eigen::VectorXi>(mask_recons_update_grid.data(), mask_recons_update_grid.size());
        mask_recons_update_grid.transposeInPlace();
        r_col = Eigen::VectorXd::Zero(table_init_SPHARM_grid.rows());
        reconstruct_points();
        std::vector<int> indices;
        std::vector<int> indices_filter;
        for (int i = 0; i < r_col.size(); ++i) {
            if (r_col(i) > 0) {
                indices.push_back(i);
            }
            if (r_col(i) > lidarConfig.min_length && r_col(i) < lidarConfig.max_length) {
                indices_filter.push_back(i);
            }
        }
        table_init_SPHARM_grid.col(0) = table_init_SPHARM_grid.col(0).array() - M_PI;
        table_init_SPHARM_grid.col(1) = M_PI_2 - table_init_SPHARM_grid.col(1).array();
        reconsPointCloud.resize(indices.size(), 3);
        sph2cart(r_col(indices), table_init_SPHARM_grid(indices, Eigen::all), reconsPointCloud);
        reconsPointCloudFiltered.resize(indices_filter.size(), 3);
        sph2cart(r_col(indices_filter), table_init_SPHARM_grid(indices_filter, Eigen::all), reconsPointCloudFiltered);
    }
};


#endif //EXAMPLE_CURL_RECONSTRUCTION_MEX_H
