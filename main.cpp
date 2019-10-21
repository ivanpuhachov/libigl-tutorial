#include <igl/opengl/glfw/Viewer.h>
#include <igl/cotmatrix.h>
#include <igl/gaussian_curvature.h>
#include <igl/massmatrix.h>
#include <igl/invert_diag.h>
#include <igl/jet.h>
#include <igl/principal_curvature.h>

Eigen:: MatrixXd V;
Eigen::MatrixXi F;

int main(int argc, char *argv[]) {
    igl::readOFF("../data/bumpy.off", V,F);
    std::cout<< "Vertices: " << std::endl << V.size() << std::endl;
    std::cout<< "Faces: " << std::endl << F.size() << std::endl;

//    Extract mean curvature from Laplace-Beltrami operator (cotmatrix)
    Eigen::MatrixXd HN;
    Eigen::SparseMatrix<double> L,M,Minv;
    igl::cotmatrix(V,F,L);
    igl::massmatrix(V,F,igl::MASSMATRIX_TYPE_VORONOI,M);
    igl::invert_diag(M,Minv);
    HN = -Minv * (L*V);
    Eigen::VectorXd HH = HN.rowwise().norm();

//    Compute curvature directions via quadratic fitting
    Eigen::MatrixXd PD1, PD2;
    Eigen::VectorXd PV1, PV2;
    igl::principal_curvature(V,F,PD1, PD2, PV1, PV2);

//    mean curvature from main curvatures
    Eigen::MatrixXd H = 0.5*(PV1+PV2);


//    Plot the mesh
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V,F); // copies the mesh into viewer

    Eigen::MatrixXd C;
    igl::parula(HH,true,C);
    viewer.data().set_colors(C);

    const double avg = igl::avg_edge_length(V,F);

    const Eigen::RowVector3d red(0.8,0.1,0.1), blue(0.1,0.1,0.8);
    viewer.data().add_edges(V+PD2*avg, V-PD2*avg, red);
    viewer.data().add_edges(V+PD1*avg, V-PD2*avg, blue);

    viewer.launch();
}
