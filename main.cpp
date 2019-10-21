#include <igl/opengl/glfw/Viewer.h>

int main(int argc, char *argv[]) {
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::VectorXd Z;
    Eigen::MatrixXd C;

    igl::readOFF("../data/screwdriver.off", V,F);
    Z = V.col(2);
    igl::jet(Z,true,C);
    std::cout<< "Vertices: " << std::endl << V << std::endl;
    std::cout<< "Faces: " << std::endl << F << std::endl;


//    Plot the mesh
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V,F); // copies the mesh into viewer
    viewer.data().set_face_based(true); // Change the visualization mode, invalidating the cache if necessary
    viewer.data().set_colors(C);
    viewer.launch();
}
