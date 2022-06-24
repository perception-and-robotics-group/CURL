#include "igl/AABB.h"
#include "Timer.h"
#include "mex.hpp"
#include "mexAdapter.hpp"

class MexFunction : public matlab::mex::Function {
public:
    void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        checkArguments(outputs, inputs);
        matlab::data::TypedArray<double> vertices = std::move(inputs[0]);
        matlab::data::TypedArray<double> faces = std::move(inputs[1]);
        matlab::data::TypedArray<double> rays = std::move(inputs[2]);
        matlab::data::TypedArray<double> recons_table_check = std::move(inputs[3]);
        ray_tracing(outputs, vertices, faces, rays, recons_table_check);
    }

    void ray_tracing(matlab::mex::ArgumentList &outputs,
                      matlab::data::TypedArray<double> &vertices,
                      matlab::data::TypedArray<double> &faces,
                      matlab::data::TypedArray<double> &rays,
                      matlab::data::TypedArray<double> &recons_table_check) {
        // vertices
        auto dim = vertices.getDimensions();
        Eigen::MatrixXd V(dim[0], dim[1]);
        for (int i = 0; i < dim[0]; i++) {
            for (int j = 0; j < dim[1]; j++) {
                V(i, j) = vertices[i][j];
            }
        }
        // faces
        dim = faces.getDimensions();
        Eigen::MatrixXi F(dim[0], dim[1]);
        for (int i = 0; i < dim[0]; i++) {
            for (int j = 0; j < dim[1]; j++) {
                F(i, j) = int(faces[i][j]);
            }
        }
        F = F.array() - 1;
        // rays
        dim = rays.getDimensions();
        Eigen::ArrayXXd R(dim[0], dim[1]);
        for (int i = 0; i < dim[0]; i++) {
            for (int j = 0; j < dim[1]; j++) {
                R(i, j) = rays[i][j];
            }
        }
        Timer timer;
        timer.start();
        // calculate triangle area for each face
        Eigen::MatrixXd vec_1(F.rows(), 3);
        Eigen::MatrixXd vec_2(F.rows(), 3);
        Eigen::VectorXd area(F.rows());
        vec_1 = V(F(Eigen::all, 1), Eigen::all) - V(F(Eigen::all, 0), Eigen::all);
        vec_2 = V(F(Eigen::all, 2), Eigen::all) - V(F(Eigen::all, 0), Eigen::all);
        for (int i = 0; i < vec_1.rows(); i++) {
            area(i) = vec_1(i, {0, 1, 2}).cross(vec_2(i, {0, 1, 2})).norm();
        }
        Eigen::VectorXd area_sort = area;
        std::sort(area_sort.data(), area_sort.data() + area_sort.size());
        double area_thres = area_sort(int(area_sort.size() * 0.9) - 1);
        // preprocessing the ray
        Eigen::ArrayXXd directions(R.rows(), 3);
        directions << cos(R.col(1)) * cos(R.col(0)),
                cos(R.col(1)) * sin(R.col(0)), sin(R.col(1));
        // AABB tree
        igl::AABB<Eigen::MatrixXd, 3> tree;
        std::vector<igl::Hit> hits(directions.rows());
        tree.init(V, F);
        // ray tracing
        Eigen::MatrixXd V_up = Eigen::MatrixXd::Zero(R.rows(), 3);
        Eigen::VectorXi F_id = Eigen::VectorXi::Zero(R.rows()) * NULL;
        Eigen::Matrix<double, 1, 3> origin{0, 0, 0};
#pragma omp parallel for num_threads(omp_get_num_procs()) default(none) shared(directions, tree, V, F, origin, hits, F_id, V_up, area, area_thres, recons_table_check)
        for (int i = 0; i < directions.rows(); i++) {
            if (recons_table_check[i] != 1) {
                tree.intersect_ray(V, F, origin, directions.row(i), hits[i]);
                if (area(hits[i].id) <= area_thres) {
                    F_id(i) = hits[i].id+1;
                    V_up.row(i) =
                            V.row(F(hits[i].id, 0)) * (1 - hits[i].u - hits[i].v) +
                            V.row(F(hits[i].id, 1)) * hits[i].u +
                            V.row(F(hits[i].id, 2)) * hits[i].v;
                }
            }
        }
        double extraction_time = timer.elapsedSeconds();
        // copy result to matlab
        matlab::data::ArrayFactory factory;
        matlab::data::Array ray_tracing_time_mat = factory.createArray<double>({1, 1});
        ray_tracing_time_mat[0] = extraction_time;
        outputs[0] = std::move(ray_tracing_time_mat);
        unsigned long int x = V_up.rows();
        unsigned long int y = V_up.cols();
        matlab::data::Array points = factory.createArray<double>({x, y});
        matlab::data::Array id = factory.createArray<int>({x, 1});
        for (int i = 0; i < V_up.rows(); i++) {
            for (int j = 0; j < V_up.cols(); j++) {
                points[i][j] = V_up(i, j);
            }
            id[i] = F_id(i);
        }
        outputs[1] = std::move(points);
        outputs[2] = std::move(id);
    }

    void checkArguments(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();
        matlab::data::ArrayFactory factory;
        // arguments 1: vertices
        if (inputs.size() != 4) {
            matlabPtr->feval(u"error",
                             0, std::vector<matlab::data::Array>({factory.createScalar("Four inputs required")}));
        }
        auto dim = inputs[0].getDimensions();
        if (dim[1] != 3) {
            matlabPtr->feval(u"error",
                             0, std::vector<matlab::data::Array>(
                            {factory.createScalar("Input vertices must have 3 columns")}));
        }

        if (inputs[0].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[0].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error",
                             0, std::vector<matlab::data::Array>(
                            {factory.createScalar("Input vertices must be a noncomplex scalar double")}));
        }

        // arguments 2: faces
        dim = inputs[1].getDimensions();
        if (dim[1] != 3) {
            matlabPtr->feval(u"error",
                             0, std::vector<matlab::data::Array>(
                            {factory.createScalar("Input faces must have 3 columns")}));
        }

        if (inputs[1].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[1].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error",
                             0, std::vector<matlab::data::Array>(
                            {factory.createScalar("Input faces must be a noncomplex scalar double")}));
        }
        // arguments 3: rays
        dim = inputs[2].getDimensions();
        if (dim[1] != 2) {
            matlabPtr->feval(u"error",
                             0, std::vector<matlab::data::Array>(
                            {factory.createScalar("Input rays must have 2 columns")}));
        }

        if (inputs[2].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[2].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error",
                             0, std::vector<matlab::data::Array>(
                            {factory.createScalar("Input rays must be a noncomplex scalar double")}));
        }
        // arguments 3: recons_table_check
        dim = inputs[3].getDimensions();
        if (dim[1] != 1) {
            matlabPtr->feval(u"error",
                             0, std::vector<matlab::data::Array>(
                            {factory.createScalar("Input rays must have 1 columns")}));
        }

        if (inputs[3].getType() != matlab::data::ArrayType::DOUBLE ||
            inputs[3].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
            matlabPtr->feval(u"error",
                             0, std::vector<matlab::data::Array>(
                            {factory.createScalar("Input rays must be a noncomplex scalar double")}));
        }
    }
};