// Because of Mosek complications, we don't use static library if Mosek is used.
#ifdef LIBIGL_WITH_MOSEK
#ifdef IGL_STATIC_LIBRARY
#undef IGL_STATIC_LIBRARY
#endif
#endif

#include <igl/boundary_conditions.h>
#include <igl/colon.h>
#include <igl/column_to_quats.h>
#include <igl/directed_edge_parents.h>
#include <igl/forward_kinematics.h>
#include <igl/jet.h>
#include <igl/lbs_matrix.h>
#include <igl/deform_skeleton.h>
#include <igl/normalize_row_sums.h>
#include <igl/readDMAT.h>
#include <igl/readMESH.h>
#include <igl/readTGF.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/bbw.h>
//#include <igl/embree/bone_heat.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include <algorithm>
#include <iostream>

typedef
std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond> >
        RotationList;

const Eigen::RowVector3d sea_green(70./255.,252./255.,167./255.);
int selected = 0; // selected area to draw weights heatmap
Eigen::MatrixXd V,W,U,C,M;
Eigen::MatrixXi T,F,BE;
Eigen::VectorXi P;
RotationList pose;
double anim_t = 1.0;
double anim_t_dir = -0.03;

/* GLOBAL VARIABLES
// tetrahedral mesh
V - double matrix of vertex positions  #V by 3
T - #T list of tet indices into vertex positions
F - #F list of face indices into vertex positions

// weights
W - W  #V by #W list of *unnormalized* weights
M - V by (dim+1)*#T (composed of weights and rest positions)

// plotting
U - transformed double matrix of vertex positions  #V by 3

// graph
C - # vertices by 3 list of skeleton vertex positions
BE - # edges by 2 list of skeleton edge indices
P - list of parent indices into BE (-1) means root
*/

void set_color(igl::opengl::glfw::Viewer &viewer)
// configures colors of mesh according to weights (W) of selected point (selected)
{
    Eigen::MatrixXd C;
    igl::jet(W.col(selected).eval(),true,C); // jet returns #C by 3 list of rgb colors
    viewer.data().set_colors(C);
}

bool key_down(igl::opengl::glfw::Viewer &viewer, unsigned char key, int mods)
// how to react on special keyboard commands
{
    switch(key)
    {
        case ' ':
            // viewer.core().is_animating = !viewer.core().is_animating;
            break;
        case '.':
            selected++;
            selected = std::min(std::max(selected,0),(int)W.cols()-1);
            set_color(viewer);
            break;
        case ',':
            selected--;
            selected = std::min(std::max(selected,0),(int)W.cols()-1);
            set_color(viewer);
            break;
    }
    return true;
}

int main(int argc, char *argv[])
{
    using namespace Eigen;
    using namespace std;
    igl::readMESH("../cpp_input/cactus01.mesh",V,T,F); // load tetrahedral mesh
    U=V;
    igl::readTGF("../cpp_input/cactus01.tgf",C,BE); // reading trivial graph format
    cout<<C.size();
    cout << BE.size();
    // List of boundary indices (aka fixed value indices into VV)
    VectorXi b;
    // List of boundary conditions of each weight function
    MatrixXd bc;
    VectorXi tmp1;
    MatrixXi tmp2;
    bool bbb = igl::boundary_conditions(V,T,C,VectorXi(),BE,tmp2,b,bc); // Compute boundary conditions for automatic weights computation
    if (!bbb){
        cout<< "BBBBBBBBB" << '\n';
    }
    std::cout << b.size() << '\n';
    // b -    list of boundary indices (indices into V of vertices which have
    //     known, fixed values)
    // bc -   #b by #weights list of known/fixed values for boundary vertices
    //     (notice the #b != #weights in general because #b will include all the
    //     intermediary samples along each bone, etc.. The ordering of the
    //     weights corresponds to [P;BE]

    // compute BBW weights matrix
    igl::BBWData bbw_data;
    // only a few iterations for sake of demo
    bbw_data.active_set_params.max_iter = 8;
    bbw_data.verbosity = 2; // 0 - quiet, 1 - loud, 2 - louder
    if(!igl::bbw(V,T,b,bc,bbw_data,W))
    {
        return EXIT_FAILURE;
    }

    //MatrixXd Vsurf = V.topLeftCorner(F.maxCoeff()+1,V.cols());
    //MatrixXd Wsurf;
    //if(!igl::bone_heat(Vsurf,F,C,VectorXi(),BE,MatrixXi(),Wsurf))
    //{
    //  return false;
    //}
    //W.setConstant(V.rows(),Wsurf.cols(),1);
    //W.topLeftCorner(Wsurf.rows(),Wsurf.cols()) = Wsurf = Wsurf = Wsurf = Wsurf;

    // Normalize weights to sum to one
    igl::normalize_row_sums(W,W);
    // precompute linear blend skinning matrix
    igl::lbs_matrix(V,W,M);

    // Plot the mesh with pseudocolors
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(U, F); // function set_mesh copies the mesh into the viewer
    set_color(viewer);
    viewer.data().set_edges(C,BE,sea_green);
    viewer.data().show_lines = false;
    viewer.data().show_overlay_depth = false;
    viewer.data().line_width = 1;
    viewer.callback_key_down = &key_down;
    viewer.core().is_animating = false;
    viewer.core().animation_max_fps = 30.;
    cout<<
        "Press '.' to show next weight function."<<endl<<
        "Press ',' to show previous weight function."<<endl<<
        "Press [space] to toggle animation."<<endl;
    viewer.launch(); // creates a window, an OpenGL context and it starts the draw loop
    return EXIT_SUCCESS;
}
