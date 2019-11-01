// Because of Mosek complications, we don't use static library if Mosek is used.
#ifdef LIBIGL_WITH_MOSEK
#ifdef IGL_STATIC_LIBRARY
#undef IGL_STATIC_LIBRARY
#endif
#endif
#include <igl/slice.h>
#include <igl/boundary_conditions.h>
#include <igl/directed_edge_parents.h>
#include <igl/jet.h>
#include <igl/lbs_matrix.h>
#include <igl/normalize_row_sums.h>
#include <igl/readMESH.h>
#include <igl/readTGF.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/bbw.h>
#include <igl/harmonic.h>
#include <mutex>
#include <iostream>
#include <igl/active_set.h>
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
int selected = 0;
Eigen::MatrixXd V,W,U,C,M,u_curve;
Eigen::MatrixXi T,F,BE,f_curve;
Eigen::VectorXi P;
RotationList pose;
double anim_t = 1.0;
double anim_t_dir = -0.03;

int layer =0;

int magic_vertices_start = 806;
int magic_vertices_step = 434;
int magic_faces_start = 2044;
int magic_faces_step = 868;

void set_color(igl::opengl::glfw::Viewer &viewer)
{
    Eigen::MatrixXd CC;
    u_curve = U.block(magic_vertices_start+layer*magic_vertices_step,0,2*magic_vertices_step,3);
    igl::jet(W.block(magic_vertices_start+layer*magic_vertices_step, selected,2*magic_vertices_step,1).eval(),true,CC);
    viewer.data().set_mesh(u_curve, f_curve);
    viewer.data().set_colors(CC);
    Eigen::MatrixXd pp;
    pp = U.block(5972+63*layer, 0, 30, 3);
    igl::jet(W.block(5972+63*layer, selected,30,1).eval(),true,CC);
    viewer.data().set_points(pp,CC);
}

bool key_down(igl::opengl::glfw::Viewer &viewer, unsigned char key, int mods)
{
    switch(key)
    {
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
        case '2':
            layer++;
            if (layer>=10) {
                layer = 0;
            }
            set_color(viewer);
            break;
        case '1':
            layer--;
            if (layer<=0) {
                layer = 9;
            }
            set_color(viewer);
            break;
        default:
            std::cout<<key<<std::endl;
            std::cout<<layer<<std::endl;
            std::cout<<selected<<std::endl;

    }
    return true;
}

bool my_bbw(
        const Eigen::MatrixXd & V,
        const Eigen::MatrixXi & Ele,
        const Eigen::VectorXi & b,
        const Eigen::MatrixXd & bc,
        igl::BBWData & data,
        Eigen::MatrixXd & W
)
{
    using namespace std;
    using namespace Eigen;
    assert(!data.partition_unity && "partition_unity not implemented yet");
    // number of domain vertices
    int n = V.rows();
    // number of handles
    int m = bc.cols();
    // Build biharmonic operator
    Eigen::SparseMatrix<double> Q;
    igl::harmonic(V,Ele,2,Q);
    W.derived().resize(n,m);
    // No linear terms
    VectorXd c = VectorXd::Zero(n);
    // No linear constraints
    SparseMatrix<double> A(0, n),Aeq(0, n),Aieq(0, n);
    VectorXd Beq(0,1),Bieq(0,1);
    // Upper and lower box constraints (Constant bounds)
    VectorXd ux = VectorXd::Ones(n);
    VectorXd lx = VectorXd::Zero(n);
    igl::active_set_params eff_params = data.active_set_params;
    if(data.verbosity >= 1)
    {
        cout<<"BBW: max_iter: "<<data.active_set_params.max_iter<<endl;
        cout<<"BBW: eff_max_iter: "<<eff_params.max_iter<<endl;
    }
    if(data.verbosity >= 1)
    {
        cout<<"BBW: Computing initial weights for "<<m<<" handle"<<
            (m!=1?"s":"")<<"."<<endl;
    }
    igl::min_quad_with_fixed_data<double> mqwf;
    min_quad_with_fixed_precompute(Q,b,Aeq,true,mqwf);
    min_quad_with_fixed_solve(mqwf,c,bc,Beq,W);
    // decrement
    eff_params.max_iter--;
    bool error = false;
    // Loop over handles
    std::mutex critical;
    const auto & optimize_weight = [&](const int i)
    {
        // Quicker exit for paralle_for
        if(error)
        {
            return;
        }
        if(data.verbosity >= 1)
        {
            std::lock_guard<std::mutex> lock(critical);
            cout<<"BBW: Computing weight for handle "<<i+1<<" out of "<<m<<
                "."<<endl;
        }
        VectorXd bci = bc.col(i);
        VectorXd Wi;
        // use initial guess
        Wi = W.col(i);
        igl::SolverStatus ret = active_set(
                Q,c,b,bci,Aeq,Beq,Aieq,Bieq,lx,ux,eff_params,Wi);
        switch(ret)
        {
            case igl::SOLVER_STATUS_CONVERGED:
                break;
            case igl::SOLVER_STATUS_MAX_ITER:
                cerr<<"active_set: max iter without convergence."<<endl;
                break;
            case igl::SOLVER_STATUS_ERROR:
            default:
                cerr<<"active_set error."<<endl;
                error = true;
        }
        W.col(i) = Wi;
    };
#ifdef WIN32
    for (int i = 0; i < m; ++i)
    optimize_weight(i);
#else
    igl::parallel_for(m,optimize_weight,2);
#endif
    if(error)
    {
        return false;
    }

#ifndef NDEBUG
    const double min_rowsum = W.rowwise().sum().array().abs().minCoeff();
    if(min_rowsum < 0.1)
    {
        cerr<<"bbw.cpp: Warning, minimum row sum is very low. Consider more "
              "active set iterations or enforcing partition of unity."<<endl;
    }
#endif

    return true;
}

int main(int argc, char *argv[])
{
    using namespace Eigen;
    using namespace std;
    igl::readMESH("../cpp_input/mesh.mesh",V,T,F);
    U=V;
    igl::readTGF("../cpp_input/mesh.tgf",C,BE);
    // retrieve parents for forward kinematics
    igl::directed_edge_parents(BE,P);

    // List of boundary indices (aka fixed value indices into VV)
    VectorXi b;
    // List of boundary conditions of each weight function
    MatrixXd bc;
    igl::boundary_conditions(V,T,C,VectorXi(),BE,MatrixXi(),b,bc);

    // compute BBW weights matrix
    igl::BBWData bbw_data;
    // only a few iterations for sake of demo
    bbw_data.active_set_params.max_iter = 8;
    bbw_data.verbosity = 2;
    if(!my_bbw(V,T,b,bc,bbw_data,W))
    {
        return EXIT_FAILURE;
    }
    // Normalize weights to sum to one
    igl::normalize_row_sums(W,W);
    // precompute linear blend skinning matrix
    igl::lbs_matrix(V,W,M);
    // make magic with visualization
    u_curve = U.block(magic_vertices_start+layer*magic_vertices_step,0,2*magic_vertices_step,3);
//    std::cout<<u_curve <<std::endl;
//    f_curve = F.block(magic_faces_start, 0, magic_faces_step,3);
//    Eigen::MatrixXi mat = Eigen::MatrixXi::Constant(magic_faces_step,3,magic_vertices_start-1);
//    f_curve = f_curve - mat;
    f_curve = Eigen::MatrixXi::Zero(magic_faces_step,3);
    for (int i=0;i<magic_vertices_step;i++){
        f_curve(i,0) = i % magic_vertices_step;
        f_curve(i, 1) = (i+1) % magic_vertices_step + magic_vertices_step;
        f_curve(i, 2) = (i+1) % magic_vertices_step;
    }
    for (int i=magic_vertices_step;i<2*magic_vertices_step;i++){
        f_curve(i,0) = i % magic_vertices_step;
        f_curve(i, 2) = (i+1) % magic_vertices_step + magic_vertices_step;
        f_curve(i, 1) = i;
    }

//    std::cout<<f_curve <<std::endl;
//    u_curve = U.block(0,0,magic_vertices_start,3);
//    f_curve = F.block(0,0,magic_faces_start,3);
//    u_curve=U;
//    f_curve=F;
    // Plot the mesh with pseudocolors
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(u_curve, f_curve);

//    Eigen::MatrixXd C;
//    igl::jet(W.block(magic_vertices_start+layer*magic_vertices_step,selected,2*magic_vertices_step,1).eval(),true,C);
//    viewer.data().set_colors(C);

    set_color(viewer);

    Eigen::MatrixXd pp;
    pp = U.block(5972, 0, 30, 3);
    Eigen::MatrixXd CC;
    igl::jet(W.block(5972, selected,30,1).eval(),true,CC);
    viewer.data().set_points(pp,CC);

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
    viewer.launch();
    return EXIT_SUCCESS;
}