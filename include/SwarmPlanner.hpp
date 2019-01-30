//
// Created by jungwon on 19. 1. 21.
//

#pragma once

#include <fstream>
#include <iostream>

#include <timer.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <octomap/OcTree.h>

#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

class SwarmPlanner {
public:
    SwarmPlanner(const std::vector<octomap::point3d>& _start,
                 const std::vector<octomap::point3d>& _goal,
                 const std::vector<double>& _quad_size,
                 int _plan_Npoly, int _plan_n, double _plan_downwash,
                 const std::vector<std::vector<std::pair<octomap::point3d,octomap::point3d>>>& _obstacle_boxes,
                 const std::vector<std::vector<std::vector<std::pair<int, double>>>>& _relative_boxes,
                 const std::vector<std::vector<double>>& _ts_each,
                 const std::vector<double>& _ts_total)
             : start(_start),
               goal(_goal),
               quad_size(_quad_size),
               plan_downwash(_plan_downwash),
               obstacle_boxes(_obstacle_boxes),
               relative_boxes(_relative_boxes),
               ts_each(_ts_each),
               ts_total(_ts_total)
    {
        N = _plan_Npoly;
        n = _plan_n;
        M = ts_total.size()-1;
        qn = obstacle_boxes.size();
        outdim = 3;
    }

    void update(){
        IloEnv   env;
        try {
            Timer timer;
            timer.reset();
            updateConstraints();
            timer.stop();
            std::cout << "Building ConstMatrix success!" << std::endl;
            std::cout << "Building ConstMatrix runtime: " << timer.elapsedSeconds() << std::endl;


            for(int k = 0; k < 3; k++) {
                IloModel model(env);
                IloNumVarArray var(env);
                IloRangeArray con(env);

                populatebyrow(model, var, con, k);

                timer.reset();
                IloCplex cplex(model);

//                cplex.exportModel("/home/jungwon/catkin_ws/src/SwarmPlanning/debug/qpex1.lp");

                // Optimize the problem and obtain solution.
                if (!cplex.solve()) {
                    env.error() << "Failed to optimize QP" << endl;
                    throw (-1);
                }

                IloNumArray vals(env);
//                env.out() << "QP Solution status = " << cplex.getStatus() << endl;
                env.out() << "QP Solution value  = " << cplex.getObjValue() << endl;
//            cplex.getValues(vals, var);
//            env.out() << "Values        = " << vals << endl;
//            cplex.getSlacks(vals, con);
//            env.out() << "Slacks        = " << vals << endl;
//            cplex.getDuals(vals, con);
//            env.out() << "Duals         = " << vals << endl;
//            cplex.getReducedCosts(vals, var);
//            env.out() << "Reduced Costs = " << vals << endl;
//
//                cplex.exportModel("qpex1.lp");

                timer.stop();
                std::cout << "solve: " << timer.elapsedSeconds() << std::endl;
            }

        }
        catch (IloException& e) {
            cerr << "Concert exception caught: " << e << endl;
        }
        catch (...) {
            cerr << "Unknown exception caught" << endl;
        }
        env.end();
    }

private:
    // std::shared_ptr<Eigen::MatrixXd> Q_obj, Aeq_obj, Alq_obj, deq_obj, dlq_obj;
    Eigen::MatrixXd Q, Aeq, Alq, deq, dlq;
    std::vector<octomap::point3d> start;
    std::vector<octomap::point3d> goal;
    std::vector<double> quad_size;
    std::vector<std::vector<std::pair<octomap::point3d,octomap::point3d>>> obstacle_boxes;
    std::vector<std::vector<std::vector<std::pair<int, double>>>> relative_boxes;
    std::vector<std::vector<double>> ts_each;
    std::vector<double> ts_total;
    int N, n, M, qn, outdim;
    double plan_downwash;

    void populatebyrow (IloModel model, IloNumVarArray x, IloRangeArray c, int k) {
        Timer timer;
        timer.reset();
        IloEnv env = model.getEnv();

        for(int i = 0; i < Q.rows(); i++) {
            x.add(IloNumVar(env, -IloInfinity, IloInfinity));
        }
        timer.stop();
        std::cout << "1: " << timer.elapsedSeconds() << std::endl;

        timer.reset();
        IloNumExpr cost(env);
        for(int i = 0; i < Q.rows(); i++){
            for(int j = 0; j < Q.cols(); j++){
                if(Q(i, j) != 0) {
                    cost += Q(i, j) * x[i] * x[j];
                }
            }
        }
        model.add(IloMinimize(env, cost));
        timer.stop();
        std::cout << "2: " << timer.elapsedSeconds() << std::endl;

        timer.reset();
        for(int i = 0; i < Aeq.rows(); i++){
            int count = 0;
            IloNumExpr expr(env);
            for(int j = 0; j < Aeq.cols(); j++){
                if(Aeq(i, j) != 0) {
                    expr += Aeq(i, j) * x[j];
                }
            }
            c.add(expr == deq(i,k));
            expr.end();
        }
        timer.stop();
        std::cout << "3: " << timer.elapsedSeconds() << std::endl;

        timer.reset();
        int cccc = 1;
        for(int i = 0; i < Alq.rows(); i++){
            IloNumExpr expr(env);
            for(int j = 0; j < Alq.cols(); j++){
                if(Alq(i, j) != 0) {
                    expr += Alq(i, j) * x[j];
                }
            }
            c.add(expr <= dlq(i,k));
            expr.end();
        }
        timer.stop();
        std::cout << "4: " << timer.elapsedSeconds() << std::endl;

        timer.reset();
        model.add(c);
        timer.stop();
        std::cout << "5: " << timer.elapsedSeconds() << std::endl;
    }  // END populatebyrow

    void updateConstraints(){
        build_Q();
        build_Aeq();
        build_Alq();
        build_deq();
        build_dlq();
    }

    // Build cost matrix Q
    void build_Q() {
//        Q_obj.reset(new Eigen::MatrixXd(qn * M*(N+1), qn * M*(N+1)));
//        Q_obj->setZero();
        Q = Eigen::MatrixXd::Zero(qn * M*(N+1), qn * M*(N+1));
        Eigen::MatrixXd Q_p = Eigen::MatrixXd::Zero(M*(N+1), M*(N+1));
        Eigen::MatrixXd Q_pp = Eigen::MatrixXd::Zero(N+1, N+1);
        if (n == 3 && N == 5) {
            Q_pp <<   720, -1800,  1200,     0,     0,  -120,
                    -1800,  4800, -3600,     0,   600,     0,
                     1200, -3600,  3600, -1200,     0,     0,
                        0,     0, -1200,  3600, -3600,  1200,
                        0,   600,     0, -3600,  4800, -1800,
                     -120,     0,     0,  1200, -1800,   720;
        } else {
            std::cerr << "TODO: debug when Npoly is not 5" << std::endl;
        }

        for (int m = 0; m < M; m++) {
            Q_p.block((N+1)*m, (N+1)*m, N+1, N+1) = Q_pp * pow(ts_total[m+1]-ts_total[m], -2*n+1);
        }
        for (int qi = 0; qi < qn; qi++) {
//            Q_obj->block(qi*M*(N+1), qi*M*(N+1), M*(N+1), M*(N+1)) = Q_p;
            Q.block(qi*M*(N+1), qi*M*(N+1), M*(N+1), M*(N+1)) = Q_p;
        }
//        std::ofstream file("/home/jungwon/catkin_ws/src/SwarmPlanning/debug/Q.txt");
//        if (file.is_open())
//        {
//            file << Q << '\n';
//        }
//        file.close();
    }

    //                                         //
    // Equality Constraints Mapping Matrix Aeq //
    //                                         //
    void build_Aeq() {
//        Aeq_obj.reset(new Eigen::MatrixXd(qn * (2*n + (M-1)*n), qn * (N+1)*M));
//        Aeq_obj->setZero();
        Aeq = Eigen::MatrixXd::Zero(qn * (2*n + (M-1)*n), qn * (N+1)*M);
        Eigen::MatrixXd A_waypoints = Eigen::MatrixXd::Zero(2*n, (N+1)*M);
        Eigen::MatrixXd A_cont = Eigen::MatrixXd::Zero((M-1)*n, (N+1)*M);
        Eigen::MatrixXd A_0 = Eigen::MatrixXd::Zero(N+1, N+1);
        Eigen::MatrixXd A_T = Eigen::MatrixXd::Zero(N+1, N+1);

        // Build A_0, A_T
        if (n == 3 && N == 5) {
            A_0 <<  1,  0,  0,  0,  0,  0,
                   -1,  1,  0,  0,  0,  0,
                    1, -2,  1,  0,  0,  0,
                   -1,  3, -3,  1,  0,  0,
                    1, -4,  6, -4,  1,  0,
                   -1,  5,-10, 10, -5,  1;
            A_T <<  0,  0,  0,  0,  0,  1,
                    0,  0,  0,  0, -1,  1,
                    0,  0,  0,  1, -2,  1,
                    0,  0, -1,  3, -3,  1,
                    0,  1, -4,  6, -4,  1,
                   -1,  5,-10, 10, -5,  1;
        } else {
            std::cerr << "TODO: debug when Npoly is not 5" << std::endl;
        }

        // Build A_waypoints
        int nn = 1;
        for (int i = 0; i < n; i++) {
            A_waypoints.block(i, 0, 1, N+1) = pow(ts_total[1] - ts_total[0], -i) * nn * A_0.row(i);
            A_waypoints.block(n+i, (N+1)*(M-1), 1, N+1) =
                    pow(ts_total[ts_total.size()-1]-ts_total[ts_total.size()-2], -i) * nn * A_T.row(i);
            nn = nn * (N-i);
        }

        // Build A_cont
        for (int m = 1; m < M; m++) {
            nn = 1;
            for (int j = 0; j < n; j++) {
                A_cont.block(n*(m-1)+j, (N+1)*(m-1), 1, N+1) = pow(ts_total[m]-ts_total[m-1], -j) * nn * A_T.row(j);
                A_cont.block(n*(m-1)+j, (N+1)*m, 1, N+1) = -pow(ts_total[m+1] - ts_total[m], -j) * nn * A_0.row(j);
                nn = nn * (N-j);
            }
        }

        // Build Aeq
        int Aeq_p_rows = A_waypoints.rows() + A_cont.rows();
        int Aeq_p_cols = A_waypoints.cols();
        for (int qi = 0; qi < qn; qi++) {
//            Aeq_obj->block(qi * Aeq_p_rows, qi * Aeq_p_cols, Aeq_p_rows, Aeq_p_cols) << A_waypoints,
//                                                                                        A_cont;
            Aeq.block(qi * Aeq_p_rows, qi * Aeq_p_cols, Aeq_p_rows, Aeq_p_cols) << A_waypoints,
                                                                                   A_cont;
        }
//        std::ofstream file("/home/jungwon/catkin_ws/src/SwarmPlanning/debug/Aeq.txt");
//        if (file.is_open())
//        {
//            file << Aeq << '\n';
//        }
//        file.close();
    }

    //                                           //
    // Equality Constraints Condition Vector deq //
    //                                           //
    void build_deq() {
//        deq_obj.reset(new Eigen::MatrixXd(qn * (2*n + (M-1)*n), outdim));
//        deq_obj->setZero();
        deq = Eigen::MatrixXd::Zero(qn * (2*n + (M-1)*n), outdim);
        for (int qi = 0; qi < qn; qi++) {
            Eigen::MatrixXd d_waypoints = Eigen::MatrixXd::Zero(2 * n, outdim);
            Eigen::MatrixXd d_cont = Eigen::MatrixXd::Zero((M - 1) * n, outdim);
            for (int k = 0; k < outdim; k++) {
                switch (k) {
                    case 0:
                        d_waypoints(0, k) = start[qi].x();
                        d_waypoints(n, k) = goal[qi].x();
                        break;
                    case 1:
                        d_waypoints(0, k) = start[qi].y();
                        d_waypoints(n, k) = goal[qi].y();
                        break;
                    case 2:
                        d_waypoints(0, k) = start[qi].z();
                        d_waypoints(n, k) = goal[qi].z();
                        break;
                    default:
                        std::cerr << "outdim is over 3" << std::endl;
                }

            }
            // Build deq
            int deq_p_rows = d_waypoints.rows() + d_cont.rows();
            int deq_p_cols = outdim;
//            deq_obj->block(qi * deq_p_rows, 0, deq_p_rows, deq_p_cols) << d_waypoints,
//                                                                          d_cont;
            deq.block(qi * deq_p_rows, 0, deq_p_rows, deq_p_cols) << d_waypoints,
                                                                     d_cont;
        }
//        std::ofstream file("/home/jungwon/catkin_ws/src/SwarmPlanning/debug/deq.txt");
//        if (file.is_open())
//        {
//            file << deq << '\n';
//        }
//        file.close();
    }

    //                                           //
    // Inequality Constraints Mapping Matrix Alq //
    //                                           //
    void build_Alq() {
//        Alq_obj.reset(new Eigen::MatrixXd(qn*2*(N+1)*M + qn*(qn-1)*(N+1)*M, qn*(N+1)*M));
//        Alq_obj->setZero();
        Alq = Eigen::MatrixXd::Zero(qn*2*(N+1)*M + qn*(qn-1)*(N+1)*M, qn*(N+1)*M);
        Eigen::MatrixXd Alq_box_p = Eigen::MatrixXd::Zero(2*(N+1)*M, (N+1)*M);
        Eigen::MatrixXd Alq_box = Eigen::MatrixXd::Zero(qn*2*(N+1)*M, qn*(N+1)*M);
        Eigen::MatrixXd Alq_rel = Eigen::MatrixXd::Zero(qn*(qn-1)*(N+1)*M, qn*(N+1)*M);

        // Build Alq_box
        Alq_box_p.block(0, 0, (N+1)*M, (N+1)*M) = Eigen::MatrixXd::Identity((N+1)*M, (N+1)*M);
        Alq_box_p.block((N+1)*M, 0, (N+1)*M, (N+1)*M) = -Eigen::MatrixXd::Identity((N+1)*M, (N+1)*M);
        for (int qi = 0; qi < qn; qi++) {
            Alq_box.block(qi*Alq_box_p.rows(), qi*Alq_box_p.cols(), Alq_box_p.rows(), Alq_box_p.cols()) = Alq_box_p;
        }

        // Build Alq_rel
        int iter = 0;
        for (int qi = 0; qi < qn; qi++) {
            for (int qj = qi + 1; qj < qn; qj++) {
                Alq_rel.block((N+1)*M*2*iter, (N+1)*M*qi, (N+1)*M, (N+1)*M) = -Eigen::MatrixXd::Identity((N+1)*M, (N+1)*M);
                Alq_rel.block((N+1)*M*2*iter, (N+1)*M*qj, (N+1)*M, (N+1)*M) = Eigen::MatrixXd::Identity((N+1)*M, (N+1)*M);

                Alq_rel.block((N+1)*M*(2*iter+1), (N+1)*M*qi, (N+1)*M, (N+1)*M) = Eigen::MatrixXd::Identity((N+1)*M, (N+1)*M);
                Alq_rel.block((N+1)*M*(2*iter+1), (N+1)*M*qj, (N+1)*M, (N+1)*M) = -Eigen::MatrixXd::Identity((N+1)*M, (N+1)*M);

                iter++;
            }
        }

        //Build Alq
//        Alq_obj->block(0,0,Alq_box.rows(), Alq_box.cols()) = Alq_box;
//        Alq_obj->block(Alq_box.rows(), 0, Alq_rel.rows(), Alq_rel.cols()) = Alq_rel;
        Alq << Alq_box,
               Alq_rel;
//        std::ofstream file("/home/jungwon/catkin_ws/src/SwarmPlanning/debug/Alq.txt");
//        if (file.is_open())
//        {
//            file << Alq << '\n';
//        }
//        file.close();
    }

    //                                             //
    // Inequality Constraints Condition Vector dlq //
    //                                             //
    void build_dlq(){
//        dlq_obj.reset(new Eigen::MatrixXd(qn*2*(N+1)*M + qn*(qn-1)*(N+1)*M, outdim));
//        dlq_obj->setZero();
        dlq = Eigen::MatrixXd::Zero(qn*2*(N+1)*M + qn*(qn-1)*(N+1)*M, outdim);
        Eigen::MatrixXd dlq_rel = Eigen::MatrixXd::Zero(qn*(qn-1)*(N+1)*M, outdim);
        Eigen::MatrixXd dlq_box = Eigen::MatrixXd::Zero(qn*2*(N+1)*M, outdim);

        // Build dlq_box
        for(int qi=0; qi<qn; qi++){
            Eigen::MatrixXd d_upper = Eigen::MatrixXd::Zero((N+1)*M, outdim);
            Eigen::MatrixXd d_lower = Eigen::MatrixXd::Zero((N+1)*M, outdim);

            int bi = 0;
            for(int m = 0; m < M; m++){
                // find box number
                while(ts_each[qi][bi+1] < ts_total[m+1]){
                    bi++;
                }

                d_upper.block((N+1)*m, 0, N+1, 1) = Eigen::MatrixXd::Constant(N+1, 1, obstacle_boxes[qi][bi].second.x());
                d_lower.block((N+1)*m, 0, N+1, 1) = Eigen::MatrixXd::Constant(N+1, 1, -obstacle_boxes[qi][bi].first.x());

                d_upper.block((N+1)*m, 1, N+1, 1) = Eigen::MatrixXd::Constant(N+1, 1, obstacle_boxes[qi][bi].second.y());
                d_lower.block((N+1)*m, 1, N+1, 1) = Eigen::MatrixXd::Constant(N+1, 1, -obstacle_boxes[qi][bi].first.y());

                d_upper.block((N+1)*m, 2, N+1, 1) = Eigen::MatrixXd::Constant(N+1, 1, obstacle_boxes[qi][bi].second.z());
                d_lower.block((N+1)*m, 2, N+1, 1) = Eigen::MatrixXd::Constant(N+1, 1, -obstacle_boxes[qi][bi].first.z());
            }

            int dlq_box_rows = d_upper.rows()+d_lower.rows();
            dlq_box.block(qi*dlq_box_rows, 0, dlq_box_rows, outdim) << d_upper,
                                                                       d_lower;
        }

        // Build dlq_rel
        int iter = 0;
        for(int qi=0; qi<qn; qi++){
            for(int qj=qi+1; qj<qn; qj++){
                Eigen::MatrixXd d_upper = Eigen::MatrixXd::Constant((N+1)*M, outdim, 10000000);
                Eigen::MatrixXd d_lower = Eigen::MatrixXd::Constant((N+1)*M, outdim, 10000000);
                double margin = quad_size[qi] + quad_size[qj];

                for(int m = 0; m < M; m++){
                    // Find box number
                    int ri = relative_boxes[qi][qj].size()-1;
                    while(ri >= 0 && relative_boxes[qi][qj][ri].second < ts_total[m+1]){
                        ri--;
                    }

                    int sector = relative_boxes[qi][qj][ri].first;
                    switch(sector){
                        case -3:
                            d_upper.block((N+1)*m, abs(sector)-1, N+1, 1) = Eigen::MatrixXd::Constant(N+1, 1, -margin*plan_downwash);
                            break;
                        case -2:
                        case -1:
                            d_upper.block((N+1)*m, abs(sector)-1, N+1, 1) = Eigen::MatrixXd::Constant(N+1, 1, -margin);
                            break;
                        case 1:
                        case 2:
                            d_lower.block((N+1)*m, abs(sector)-1, N+1, 1) = Eigen::MatrixXd::Constant(N+1, 1, -margin);
                            break;
                        case 3:
                            d_lower.block((N+1)*m, abs(sector)-1, N+1, 1) = Eigen::MatrixXd::Constant(N+1, 1, -margin*plan_downwash);
                            break;
                        default:
                            std::cerr << "invalid relative obstacle" << std::endl;
                    }
                }
                int dlq_rel_rows = d_upper.rows()+d_lower.rows();
                dlq_rel.block(iter*dlq_rel_rows, 0, dlq_rel_rows, outdim) << d_upper,
                                                                             d_lower;
                iter++;
            }
        }

        // Build dlq
//        dlq_obj->block(0, 0, dlq_box.rows(), dlq_box.cols()) = dlq_box;
//        dlq_obj->block(dlq_box.rows(), 0, dlq_rel.rows(), dlq_rel.cols()) = dlq_rel;
        dlq << dlq_box,
               dlq_rel;
//        std::ofstream file("/home/jungwon/catkin_ws/src/SwarmPlanning/debug/dlq.txt");
//        if (file.is_open())
//        {
//            file << dlq << '\n';
//        }
//        file.close();
    }
};