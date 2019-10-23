//
// Created by ivan on 2019-10-22.
//

#include <igl/opengl/glfw/Viewer.h>
#include <igl/cotmatrix.h>
#include <igl/gaussian_curvature.h>
#include <igl/massmatrix.h>
#include<Eigen/SparseCholesky>


Eigen::MatrixXd V, U;
Eigen::MatrixXi F;
Eigen::SparseMatrix<double> L, Minv; // Laplassian

const double dt = 3e-5;
const double lambda = 2;

bool on_key_down(igl::opengl::glfw::Viewer& viewer, char key, int modifier) {
    Eigen::SparseMatrix<double> A,M;
    Eigen::VectorXd Areas, C;
    Eigen::MatrixXd b;
    Eigen::SparseLU<Eigen::SparseMatrix<double> > solver;
    std::cout<<"zbs"<<std::endl;
    igl::massmatrix(U,F,igl::MASSMATRIX_TYPE_BARYCENTRIC, M);
    b = M*U;
    A = M - dt * lambda * L;
    solver.compute(A);
    Areas = M.diagonal();
    igl::jet(Areas, true, C);
    switch (key){
        case '1': {
//            A = M - dt * lambda * L;
//            assert(M.cols() == U.rows());
//            solver.compute(A);
            if (solver.info() != Eigen::Success) {
                // decomposition failed
                std::cout << "Decomposition failed" << std::endl;
                return false;
            }
            U = solver.solve(M * U).eval();
            std::cout << "Success" << std::endl;
            viewer.data().set_mesh(U, F); // copies the mesh into viewer
            viewer.data().set_colors(C);
            break;
        }
        case '2': {
            U = V;
            break;
        }
        default: {
            std::cout << key << std::endl;
        }
    }
    return false;
}

int main(int argc, char *argv[]) {
    igl::readOFF("../data/cow.off", V,F);
    U = V;
    std::cout<< "Vertices: " << std::endl << V.size() << std::endl;
    std::cout<< "Faces: " << std::endl << F.size() << std::endl;

    igl::cotmatrix(V,F,L); // compute Laplassian
    Eigen::SparseMatrix<double> M;
    igl::massmatrix(U,F,igl::MASSMATRIX_TYPE_VORONOI, M); // compute massmatrix
    Eigen::MatrixXd b;
    b = M*U;
    std::cout << M.nonZeros() << std::endl;
    Eigen::VectorXd Areas;
    Areas = M.diagonal();
    Eigen::MatrixXd C;
    igl::parula(Areas, true, C);
//    Plot the mesh
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(U,F); // copies the mesh into viewer
    viewer.data().set_colors(C);
    viewer.callback_key_down = &on_key_down;
    viewer.launch();
}
