#include <igl/opengl/glfw/Viewer.h>

int main(int argc, char *argv[]) {
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    igl::readOFF("../data/cube.off", V,F);
    std::cout<< "Vertices: " << std::endl << V << std::endl;
    std::cout<< "Faces: " << std::endl << F << std::endl;

//    Plot the mesh
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V,F);
    viewer.data().set_face_based(true);
    viewer.launch();
}
