// Because of Mosek complications, we don't use static library if Mosek is used.
#ifdef LIBIGL_WITH_MOSEK
#ifdef IGL_STATIC_LIBRARY
#undef IGL_STATIC_LIBRARY
#endif
#endif


#include <igl/jet.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/bbw.h>
#include <igl/harmonic.h>
#include <mutex>
#include <iostream>
#include <igl/active_set.h>
#include <igl/triangle/triangulate.h>
#include <igl/min_quad_with_fixed.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>

typedef
std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond> >
        RotationList;

const Eigen::RowVector3d sea_green(70./255.,252./255.,167./255.);
int selected = 0;
Eigen::MatrixXd V,W,U,C,M;
Eigen::MatrixXi T,F,BE;
Eigen::VectorXi P;
RotationList pose;
double anim_t = 1.0;
double anim_t_dir = -0.03;

void set_color(igl::opengl::glfw::Viewer &viewer)
{
    Eigen::MatrixXd C;
    igl::jet(W.col(selected).eval(),true,C);
    viewer.data().set_colors(C);
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
    SparseMatrix<double> A(0,n),Aeq(0,n),Aieq(0,n);
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

    MatrixXd V;
    MatrixXi E;

    MatrixXd V2;
    MatrixXi F2;

    V.resize(5,2);
    E.resize(4,2);
    V << -1,-1, -1,1, 1,1, 1,-1, 0,0;
    E << 0,1, 1,2, 2,3, 3,0;
    igl::triangle::triangulate(V,E,MatrixXd(),"a0.0005q",V2,F2);

    int n = V2.rows();

    SparseMatrix<double> Q,Aeq;
    igl::harmonic(V2,F2,2,Q);

    VectorXd B, bc(5,1), Beq(n,1), Z;
    VectorXi b(5,1);
    b << 0,1,2,3,4;
    bc << 0,0,0,0,1.0;
    B = VectorXd::Zero(n,1);

    igl::min_quad_with_fixed_data<double> mqwf;
    igl::min_quad_with_fixed_precompute(Q,b,Aeq,true,mqwf);
    igl::min_quad_with_fixed_solve(mqwf,B,bc,Beq,Z);

    std::cout<<n<<std::endl;
    std::cout<<Z.size()<<std::endl;


    MatrixXd V3d(n,3);
    V3d.block(0,0,n,2) = V2;
    V3d.block(0,2,n,1) = Z.col(0);

    // Plot the generated mesh
    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);
    viewer.data().set_mesh(V3d,F2);

    MatrixXd CC;
    igl::jet(Z.col(0),true,CC);
    viewer.data().set_colors(CC);
    viewer.data().show_lines = false;
    viewer.launch();
}

