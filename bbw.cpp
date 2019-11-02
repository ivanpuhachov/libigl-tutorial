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
#include <igl/parula.h>
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


#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include <algorithm>
#include <iostream>

#include <igl/in_element.h>
#include <igl/barycentric_coordinates.h>

const Eigen::RowVector3d sea_green(70./255.,252./255.,167./255.);
int selected = 8;
Eigen::MatrixXd V,W,U,C,M;
Eigen::MatrixXi T,F,BE;
Eigen::VectorXi P;

int layer =0;

Eigen::MatrixXd pointsToPlot;
igl::AABB<Eigen::MatrixXd,3> tree;
Eigen::MatrixXi TetpointsToPlot; // indices of vertices associated to tetrahedra where point is located
Eigen::MatrixXd barCoordsPoints;
Eigen::VectorXd WpointsToPlot;

void weightPoints(igl::opengl::glfw::Viewer &viewer) {
    WpointsToPlot = Eigen::VectorXd::Zero(pointsToPlot.rows());

    Eigen::MatrixXd Wabcd(pointsToPlot.rows(),4);
    Eigen::VectorXd Wa, Wb, Wc, Wd;

    igl::slice(W,TetpointsToPlot.col(0),Eigen::VectorXi::Ones(1)*selected,Wa);
    igl::slice(W,TetpointsToPlot.col(1),Eigen::VectorXi::Ones(1)*selected,Wb);
    igl::slice(W,TetpointsToPlot.col(2),Eigen::VectorXi::Ones(1)*selected,Wc);
    igl::slice(W,TetpointsToPlot.col(3),Eigen::VectorXi::Ones(1)*selected,Wd);

    Wabcd.col(0) = Wa;
    Wabcd.col(1) = Wb;
    Wabcd.col(2) = Wc;
    Wabcd.col(3) = Wd;

    //    MatrixXd barCoordsPoints(pointsToPlot.rows(),4);
    std::cout<<Wabcd.size()<<std::endl;

    Eigen::Array2Xd res;
//    res = Wabcd.array() * barCoordsPoints.array();
    for (int i=0; i<Wabcd.rows(); i++)
    {
        for (int j=0; j<Wabcd.cols(); j++)
        {
            WpointsToPlot(i) += Wabcd(i,j)*barCoordsPoints(i,j);
        }
    }

    Eigen::MatrixXd CC;
    igl::parula(WpointsToPlot.eval(),false,CC);

    viewer.data().set_points(pointsToPlot,CC);
}

void updateEdges(igl::opengl::glfw::Viewer &viewer)
{
    viewer.data().set_edges(C,BE.block(selected,0,1,2),sea_green);
    weightPoints(viewer);
}

void updatePoints(igl::opengl::glfw::Viewer &viewer)
{
    pointsToPlot.col(2) = Eigen::VectorXd::Ones(pointsToPlot.rows()) * (1+2*layer);

    Eigen::VectorXi IndexpointsToPlot; // indices of tetrahedra where points are located
    igl::in_element(V,T,pointsToPlot,tree,IndexpointsToPlot); // fill indices

    igl::slice(T,IndexpointsToPlot,1,TetpointsToPlot);

    Eigen::MatrixXd tetA,tetB,tetC,tetD, Wabcd(pointsToPlot.rows(),4);
    Eigen::VectorXd Wa, Wb, Wc, Wd;
    igl::slice(V,TetpointsToPlot.col(0),1,tetA);
    igl::slice(V,TetpointsToPlot.col(1),1,tetB);
    igl::slice(V,TetpointsToPlot.col(2),1,tetC);
    igl::slice(V,TetpointsToPlot.col(3),1,tetD);

    igl::barycentric_coordinates(pointsToPlot,tetA,tetB,tetC,tetD,barCoordsPoints);

    weightPoints(viewer);
}

bool key_down(igl::opengl::glfw::Viewer &viewer, unsigned char key, int mods)
{
    switch(key)
    {
        case '.':
            selected++;
            selected %= W.cols();
            updateEdges(viewer);
            break;
        case ',':
            selected--;
            if (selected<0) selected = W.cols()-1;
            updateEdges(viewer);
            break;
        case 'N':
            layer++;
            layer %= 10;
            updatePoints(viewer);
            break;
        case 'M':
            layer--;
            if (layer<0) layer =9;
            updatePoints(viewer);
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

    pointsToPlot = U.block(5952, 0, 60, 3);

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

    tree.init(V,T); // initialize tree to search through tetrahedra

    Eigen::MatrixXd u_curve;
    Eigen::MatrixXi f_curve;

    int magic_vertices_start = 806;
    int magic_vertices_step = 434;
    int magic_faces_step = 868;

    u_curve = U.block(magic_vertices_start+layer*magic_vertices_step,0,2*magic_vertices_step,3);
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


    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(u_curve, f_curve);

    updatePoints(viewer);

    viewer.data().set_edges(C,BE.block(selected,0,1,2),sea_green);
    viewer.data().show_lines = false;
    viewer.data().show_overlay_depth = false;
    viewer.data().line_width = 10;
//    viewer.core().background_color.setOnes();
    viewer.callback_key_down = &key_down;
    cout<<
        "Press '.' to show next weight function."<<endl<<
        "Press ',' to show previous weight function."<<endl<<
        "Press [space] to toggle animation."<<endl;
    viewer.launch();
    return EXIT_SUCCESS;
}