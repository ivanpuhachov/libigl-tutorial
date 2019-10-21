#include <igl/opengl/glfw/Viewer.h>
//#include <igl/opengl/glfw/imgui/ImGuiMenu.h>

int main(int argc, char *argv[]) {
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::VectorXd Z;
    Eigen::MatrixXd C;

    igl::readOFF("../data/cow.off", V,F);
    Z = V.col(2);
    igl::jet(Z,true,C);
    std::cout<< "Vertices: " << std::endl << V.size() << std::endl;
    std::cout<< "Faces: " << std::endl << F.size() << std::endl;

    Eigen::Vector3d m = V.colwise().minCoeff();
    Eigen::Vector3d M = V.colwise().maxCoeff();

   Eigen::MatrixXd V_box(8,3);
   V_box <<
   m(0), m(1), m(2),
   M(0), m(1), m(2),
   M(0), M(1), m(2),
   m(0), M(1), m(2),
   m(0), m(1), M(2),
   M(0), m(1), M(2),
   M(0), M(1), M(2),
   m(0), M(1), M(2);

    Eigen::MatrixXi E_box(12,2);
    E_box <<
    0, 1,
    1, 2,
    2, 3,
    3, 0,
    4, 5,
    5, 6,
    6, 7,
    7, 4,
    0, 4,
    1, 5,
    2, 6,
    7 ,3;

//    Plot the mesh
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V,F); // copies the mesh into viewer
    viewer.data().set_face_based(true); // Change the visualization mode, invalidating the cache if necessary
    viewer.data().set_colors(C);

    viewer.data().add_points(V_box, Eigen::RowVector3d(0,1,1));
    for (int i=0; i<E_box.rows(); i++)
    {
        viewer.data().add_edges(
                V_box.row(E_box(i,0)),
                V_box.row(E_box(i,1)),
                Eigen::RowVector3d(0,0,1)
                );
    }

    viewer.launch();
}
