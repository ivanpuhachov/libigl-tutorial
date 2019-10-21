#include <igl/opengl/glfw/Viewer.h>
#include <igl/gaussian_curvature.h>
#include <igl/massmatrix.h>
#include <igl/invert_diag.h>
#include <igl/jet.h>

Eigen:: MatrixXd V;
Eigen::MatrixXi F;

int main(int argc, char *argv[]) {
    igl::readOFF("../data/bumpy.off", V,F);
    std::cout<< "Vertices: " << std::endl << V.size() << std::endl;
    std::cout<< "Faces: " << std::endl << F.size() << std::endl;

    Eigen::VectorXd K;

    igl::gaussian_curvature(V,F,K);
    Eigen::SparseMatrix<double>M, Minv;

    igl::massmatrix(V,F,igl::MASSMATRIX_TYPE_DEFAULT, M);
    igl::invert_diag(M, Minv); // invert the diagonal elements of a matrix

    std::cout<<K.size() << std::endl;
    std::cout<<M.size() << std::endl;

    K = (Minv*K).eval(); // Divide by area to get integral average

    Eigen::MatrixXd C;
    igl::jet(K,true,C); // parameter true stands for normalization

//    Plot the mesh
    igl::opengl::glfw::Viewer viewer;
//    viewer.data().show_lines = false;

    viewer.data().set_mesh(V,F); // copies the mesh into viewer
//    viewer.data().set_face_based(true); // Change the visualization mode, invalidating the cache if necessary
    viewer.data().set_colors(C);
    viewer.launch();
}
