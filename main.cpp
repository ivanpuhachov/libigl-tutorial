#include <igl/opengl/glfw/Viewer.h>
//#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/per_vertex_normals.h>
#include <igl/per_face_normals.h>
#include <igl/per_corner_normals.h>

Eigen:: MatrixXd V;
Eigen::MatrixXi F;

Eigen::MatrixXd N_vertices;
Eigen::MatrixXd N_faces;
Eigen::MatrixXd N_corners;

bool on_key_down(igl::opengl::glfw::Viewer& viewer, char key, int modifier)
{
    switch(key) {
        case '1':
            viewer.data().set_normals(N_faces);
            return true;
        case '2':
            viewer.data().set_normals(N_vertices);
            return true;
        case '3':
            viewer.data().set_normals(N_corners);
            return true;
        default:
            break;
    }
    return false;
    }


int main(int argc, char *argv[]) {
    igl::readOFF("../data/fandisk.off", V,F);
    std::cout<< "Vertices: " << std::endl << V.size() << std::endl;
    std::cout<< "Faces: " << std::endl << F.size() << std::endl;

    igl::per_vertex_normals(V,F,N_vertices);
    igl::per_face_normals(V,F,N_faces);
    igl::per_corner_normals(V,F,20,N_corners); //corner_threshold - threshold in degrees on sharp angles

//    Plot the mesh
    igl::opengl::glfw::Viewer viewer;
    viewer.callback_key_down = &on_key_down;
    viewer.data().show_lines = false;

    viewer.data().set_mesh(V,F); // copies the mesh into viewer
    viewer.data().set_normals(N_faces);
    viewer.data().set_face_based(true); // Change the visualization mode, invalidating the cache if necessary
    std::cout<<
             "Press '1' for per-face normals."<<std::endl<<
             "Press '2' for per-vertex normals."<<std::endl<<
             "Press '3' for per-corner normals."<<std::endl;
    viewer.launch();
}
