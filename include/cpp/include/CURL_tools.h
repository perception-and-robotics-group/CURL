//
// Created by zkc on 22/04/22.
//

#ifndef EXAMPLE_TOOLS_H
#define EXAMPLE_TOOLS_H

#include "Eigen/Core"
#include "memory"

typedef Eigen::Matrix<double, Eigen::Dynamic, 3> Matrix3dCol;
typedef Eigen::Matrix<double, Eigen::Dynamic, 2> Matrix2dCol;

struct LidarConfig {
    int num_lasers = 0;
    int full_num_lasers = 0;
    int img_length = 0;
    int row_times = 0;
    int col_times = 0;
    double full_fov_up = 0;
    double fov_down = 0;
    double length_percentage = 0;
    double min_length = 0;
    double full_fov = 0;
    double u_res = 0;
    double fov = 0;
    double fov_up = 0;
    int is_density_enhance = 0;
    double max_length = 0;
};

struct PatchConfig {
    int degree_initial = 0;
    int degree_iter_step = 0;
    int degree_max = 0;
    int patch_row_rso = 0;
    int patch_col_rso = 0;
    int row_overlap_step = 0;
    int col_overlap_step = 0;
    int k = 0;
    bool is_1_1_recons_only = false;
};

struct ThresholdConfig {
    double avg_filter_thres = 0;
    double rmv_filter_thres = 0;
    double avg_error_thres_patch = 0;
    double remove = 0;
    double u = 0;
    double v = 0;
    double d = 0;
};

struct ReconsConfig {
    int row_times = 0;
    int col_times = 0;
    int is_density_enhance = 0;
};

struct Patches {
    Eigen::MatrixXd point_cloud_r;
    Eigen::MatrixXd point_cloud_ori_r;
    Eigen::MatrixXd training_r;
    Eigen::MatrixXd training_ori_r;
    Eigen::MatrixXd testing_r;
    Eigen::MatrixXd azi;
    Eigen::MatrixXd azi_ori;
    Eigen::MatrixXd elev;
    Eigen::MatrixXd elev_ori;
    Eigen::VectorXd point_cloud_r_col;
    Eigen::VectorXd point_cloud_ori_r_col;
    Eigen::VectorXd training_r_col;
    Eigen::VectorXd training_ori_r_col;
    Eigen::VectorXd testing_r_col;
    Eigen::MatrixXd polar;
    Eigen::MatrixXd polar_idx;
    Eigen::VectorXd polar_idx_col;
    Eigen::MatrixXd polar_ori;
    Eigen::MatrixXd polar_ori_idx;
    Eigen::VectorXd polar_ori_idx_col;
    Eigen::MatrixXd ori_idx;
    Eigen::VectorXd ori_idx_col;
};


//void atan2(Eigen::Ref<Eigen::VectorXd> y, Eigen::Ref<Eigen::VectorXd> x, Eigen::Ref<Eigen::VectorXd> result) {
//
////#pragma omp parallel for num_threads(omp_get_num_procs()) default(none) shared(result, x, y)
//    for (int i = 0; i < y.rows(); i++) {
//        result(i) = std::atan2(y(i), x(i));
//    }
//}

Eigen::VectorXd atan2(Eigen::Ref<Eigen::VectorXd> y, Eigen::Ref<Eigen::VectorXd> x) {
    Eigen::VectorXd result = Eigen::VectorXd::Zero(y.rows());
//#pragma omp parallel for num_threads(omp_get_num_procs()) default(none) shared(result, x, y)
    for (int i = 0; i < y.rows(); i++) {
        if (y(i) != 0 || x(i) > 0) {
            result(i) = 2 * atan(y(i) / (sqrt(x(i) * x(i) + y(i) * y(i)) + x(i)));
        } else if (x(i) < 0 && y(i) == 0) {
            result(i) = M_PIl;
        }
    }
    return result;
}


void atan2(Eigen::Ref<Eigen::VectorXd> y, Eigen::Ref<Eigen::VectorXd> x, Eigen::Ref<Eigen::VectorXd> result) {
//#pragma omp parallel for num_threads(omp_get_num_procs()) default(none) shared(result, x, y)
    for (int i = 0; i < y.rows(); i++) {
        if (y(i) != 0 || x(i) > 0) {
            result(i) = 2 * atan(y(i) / (sqrt(x(i) * x(i) + y(i) * y(i)) + x(i)));
        } else if (x(i) < 0 && y(i) == 0) {
            result(i) = M_PIl;
        }
    }
}

// output: azi elev r
Matrix3dCol cart2sph(Eigen::Ref<Matrix3dCol> xyz) {
    Matrix3dCol sph(xyz.rows(), 3);
    Eigen::VectorXd xyz_col_x = Eigen::Map<Eigen::VectorXd>(xyz.data(), xyz.rows());
    Eigen::VectorXd xyz_col_y = Eigen::Map<Eigen::VectorXd>(xyz.data() + xyz.rows(), xyz.rows());
    Eigen::VectorXd xyz_col_z = Eigen::Map<Eigen::VectorXd>(xyz.data() + xyz.rows() * 2, xyz.rows());
    Eigen::VectorXd hypotxy = (xyz.col(0).array().pow(2) + xyz.col(1).array().pow(2)).array().sqrt();
    sph.col(2) = (hypotxy.array().pow(2) + xyz.col(2).array().pow(2)).array().sqrt();
    sph.col(1) = atan2(xyz_col_z, hypotxy);
    sph.col(0) = atan2(xyz_col_y, xyz_col_x);
    return sph;
}

// Use Eigen::Ref can use parts as the input without const
// output: azi elev r
void cart2sph(Eigen::Ref<Matrix3dCol> xyz, Eigen::Ref<Matrix3dCol> sph) {
    Eigen::VectorXd xyz_col_x = Eigen::Map<Eigen::VectorXd>(xyz.data(), xyz.rows());
    Eigen::VectorXd xyz_col_y = Eigen::Map<Eigen::VectorXd>(xyz.data() + xyz.rows(), xyz.rows());
    Eigen::VectorXd xyz_col_z = Eigen::Map<Eigen::VectorXd>(xyz.data() + xyz.rows() * 2, xyz.rows());
    Eigen::VectorXd hypotxy = (xyz.col(0).array().pow(2) + xyz.col(1).array().pow(2)).array().sqrt();
    sph.col(2) = (hypotxy.array().pow(2) + xyz.col(2).array().pow(2)).array().sqrt();
    atan2(xyz_col_z, hypotxy, sph.col(1));
    atan2(xyz_col_y, xyz_col_x, sph.col(0));
}

void sph2cart(Eigen::VectorXd r_col, Eigen::MatrixXd sph, Eigen::Ref<Matrix3dCol> xyz) {
    xyz.col(2) = r_col.array() * sin(sph.col(1).array()).array();
    Eigen::VectorXd rcoselev = r_col.array() * cos(sph.col(1).array()).array();
    xyz.col(0) = rcoselev.array() * cos(sph.col(0).array()).array();
    xyz.col(1) = rcoselev.array() * sin(sph.col(0).array()).array();
}

void assocLegendre(int l, Eigen::VectorXd X, Eigen::MatrixXd &P) {
    P.resize(l + 1, X.size());
//#pragma omp parallel for num_threads(omp_get_num_procs()) default(none) shared(l, X, P)
    for (int i = 0; i < X.size(); i++) {
        for (int m = 0; m <= l; m++) {
            P(m, i) = pow(-1, m) * std::assoc_legendre(l, m, X(i));
        }
    }
}

Eigen::VectorXd factorial(Eigen::VectorXi n) {
    Eigen::VectorXd f(n.size());
    double f_mid;
    for (int i = 0; i < n.size(); i++) {
        f_mid = 1;
        for (int j = 1; j <= n(i); j++) {
            f_mid = f_mid * j;
        }
        f(i) = f_mid;
    }
    return f;
}

Eigen::MatrixXd getSH(int N, Eigen::MatrixXd dirs) {
    Eigen::MatrixXd Y_N;
    // initialization
    Eigen::VectorXi m;
    Eigen::MatrixXd Lnm_real;
    Eigen::MatrixXd condon;
    Y_N.resize(int(pow((N + 1), 2)), dirs.rows());
    int idx_Y = 0;
    Eigen::VectorXd norm_real;
    Eigen::MatrixXd Nnm_real;
    Eigen::MatrixXd Nnm_real_mid;
    Eigen::MatrixXd CosSin;
    Eigen::MatrixXd Ynm;
    // start calculation
    for (int n = 0; n <= N; n++) {
        m = Eigen::VectorXi::LinSpaced(n + 1, 0, n);
        assocLegendre(n, cos((Eigen::Map<Eigen::VectorXd>(dirs.data() + dirs.rows(), dirs.rows())).array()), Lnm_real);
        if (n != 0) {
            Eigen::MatrixXi condon_mid(m.size() + m.size() - 1, 1);
            condon_mid << m(Eigen::seq(m.size() - 1, 2 - 1, -1)), m;
            condon = Eigen::MatrixXd::Ones(m.size() + m.size() - 1, 1).array() * (-1);
            condon = condon.array().pow(condon_mid.array().cast<double>()).matrix() *
                     Eigen::MatrixXd::Ones(1, dirs.rows());
            Eigen::MatrixXd Lnm_real_mid(Lnm_real.rows() + Lnm_real.rows() - 1, Lnm_real.cols());
            Lnm_real_mid << Lnm_real(Eigen::seq(Lnm_real.rows() - 1, 2 - 1, -1), Eigen::all), Lnm_real;
            Lnm_real = condon.array() * Lnm_real_mid.array();
        }
        // normalisations
        norm_real = ((2 * n + 1) * factorial(n - m.array()).array() /
                     (4 * M_PIl * factorial(n + m.array())).array()).array().sqrt();
        // convert to matrix, for direct matrix multiplication with the rest
        Nnm_real = norm_real * Eigen::MatrixXd::Ones(1, dirs.rows());
        if (n != 0) {
            Nnm_real_mid.resize(Nnm_real.rows() + Nnm_real.rows() - 1, Nnm_real.cols());
            Nnm_real_mid << Nnm_real(Eigen::seq(Nnm_real.rows() - 1, 2 - 1, -1), Eigen::all), Nnm_real;
            Nnm_real = Nnm_real_mid;
        }
        CosSin = Eigen::MatrixXd::Zero(2 * n + 1, dirs.rows());
        // zero degree
        CosSin(n, Eigen::all) = Eigen::MatrixXd::Ones(1, dirs.rows());
        // positive and negative degrees
        if (n != 0) {
            CosSin(m(Eigen::seq(2 - 1, m.size() - 1)).array() + n, Eigen::all) =
                    sqrt(2) * cos((m(Eigen::seq(2 - 1, m.size() - 1)).cast<double>() *
                                   (dirs(Eigen::all, 0).transpose())).array());
            CosSin(-m(Eigen::seq(m.size() - 1, 2 - 1, -1)).array() + n, Eigen::all) =
                    sqrt(2) * sin((m(Eigen::seq(m.size() - 1, 2 - 1, -1)).cast<double>() *
                                   (dirs(Eigen::all, 0).transpose())).array());
        }
        Ynm = Nnm_real.array() * Lnm_real.array() * CosSin.array();
        Y_N(Eigen::seq(idx_Y, idx_Y + (2 * n)), Eigen::all) = Ynm;
        idx_Y = idx_Y + 2 * n + 1;
    }
    Y_N.transposeInPlace();
    return Y_N;
}

Eigen::VectorXd leastSquaresSHT(int N, Eigen::VectorXd F, Eigen::MatrixXd dirs) {
    Eigen::VectorXd F_N;
    Eigen::MatrixXd Y_N;
    Y_N = getSH(N, dirs);
    F_N = Y_N.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(F);
    return F_N;
}


template<typename T>
T load_Matrix(const std::string &read_dir) {
    T matrix;
    std::ifstream infile;
    std::string STRING, ITEM;
    std::vector<std::vector<double>> vecMatrix;
    std::vector<double> vec;
    infile.open(read_dir);
    while (!infile.eof()) // To get you all the lines.
    {
        std::getline(infile, STRING); // Saves the line in STRING.
        if (STRING.empty()) {
            break;
        }
        std::stringstream ss(STRING);
        vec.clear();
        while (ss >> ITEM) {
            vec.push_back(std::stod(ITEM));
        }
        vecMatrix.push_back(vec);
    }
    infile.close();

    matrix.resize(int(vecMatrix.size()), int(vecMatrix[0].size()));
    for (int i = 0; i < int(vecMatrix.size()); i++) {
        for (int j = 0; j < int(vecMatrix[0].size()); j++) {
            matrix(i, j) = vecMatrix[i][j];
        }
    }
    return matrix;
}

void load_coefficients(const std::string &read_dir, std::vector<Eigen::VectorXd> &coefficients, int &row, int &col) {
    std::vector<std::vector<double>> vecMatrix;
//    std::vector<double> vec;

    std::ifstream infile;
    std::string STRING, ITEM;
    infile.open(read_dir);
    std::getline(infile, STRING); // Saves the line in STRING.
    std::stringstream ss(STRING);
    ss >> ITEM;
    row = std::stod(ITEM);
    ss >> ITEM;
    col = std::stod(ITEM);
    vecMatrix.resize(row * col);
    for (int i = 0; i < row * col; ++i) {
        std::getline(infile, STRING); // Saves the line in STRING.
        if (!STRING.empty()) {
            std::stringstream ss(STRING);
            while (ss >> ITEM) {
                vecMatrix[i].push_back(std::stod(ITEM));
            }
        }
    }
    infile.close();
    coefficients.resize(row * col);
    for (int i = 0; i < row * col; ++i) {
        if (!vecMatrix[i].empty()) {
            coefficients[i].resize(vecMatrix[i].size());
            for (int j = 0; j < vecMatrix[i].size(); ++j) {
                coefficients[i](j) = vecMatrix[i][j];
            }
        }
    }
}


#endif //EXAMPLE_TOOLS_H
