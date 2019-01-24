////
//// Created by jungwon on 19. 1. 24.
////
//
//#pragma once
//
//#include <fstream>
//#include <iostream>
//
//#include <timer.hpp>
//
//#include <ros/ros.h>
//#include <ros/console.h>
//
//#define IL_STD
//#include <ilcplex/ilocplex.h>
//ILOSTLBEGIN
//
//class SwarmPlanner {
//public:
//
//    IloInt nbElements, nbRaw, nbScrap, nbIngot;
//    IloNum alloy;
//    IloNumArray nm, nr, ns, ni, p, P;
//    IloNumArray2 PRaw, PScrap, PIngot;
//
//    SwarmPlanner(){
//        IloEnv env;
//        try {
//            IloInt j;
//
//            define_data(env);
//
//            IloModel model(env);
//
//            IloNumVarArray m(env, nbElements, 0.0, IloInfinity);
//            IloNumVarArray r(env, nbRaw,   0.0, IloInfinity);
//            IloNumVarArray s(env, nbScrap, 0.0, IloInfinity);
//            IloNumVarArray i(env, nbIngot, 0.0, 100000);
//            IloNumVarArray e(env, nbElements);
//
//            // Objective Function: Minimize Cost
//            model.add(IloMinimize(env, IloScalProd(nm, m) + IloScalProd(nr, r) +
//                                       IloScalProd(ns, s) + IloScalProd(ni, i)  ));
//
//            // Min and max quantity of each element in alloy
//            for (j = 0; j < nbElements; j++) {
//                e[j] = IloNumVar(env, p[j] * alloy, P[j] * alloy);
//            }
//
//            // Constraint: produce requested quantity of alloy
//            model.add(IloSum(e) == alloy);
//
//            // Constraints: Satisfy element quantity requirements for alloy
//            for (j = 0; j < nbElements; j++) {
//                model.add(e[j] == m[j] + IloScalProd(PRaw[j], r)
//                                  + IloScalProd(PScrap[j], s)
//                                  + IloScalProd(PIngot[j], i));
//            }
//
//            // Optimize
//            IloCplex cplex(model);
//            cplex.setOut(env.getNullStream());
//            cplex.setWarning(env.getNullStream());
//            cplex.solve();
//
//            if (cplex.getStatus() == IloAlgorithm::Infeasible)
//                env.out() << "No Solution" << endl;
//
//            env.out() << "Solution status: " << cplex.getStatus() << endl;
//
//            // Print results
//            env.out() << "Cost:" << cplex.getObjValue() << endl;
//            env.out() << "Pure metal:" << endl;
//            for(j = 0; j < nbElements; j++)
//                env.out() << j << ") " << cplex.getValue(m[j]) << endl;
//            env.out() << "Raw material:" << endl;
//            for(j = 0; j < nbRaw; j++)
//                env.out() << j << ") " << cplex.getValue(r[j]) << endl;
//            env.out() << "Scrap:" << endl;
//            for(j = 0; j < nbScrap; j++)
//                env.out() << j << ") " << cplex.getValue(s[j]) << endl;
//            env.out() << "Ingots : " << endl;
//            for(j = 0; j < nbIngot; j++)
//                env.out() << j << ") " << cplex.getValue(i[j]) << endl;
//            env.out() << "Elements:" << endl;
//            for(j = 0; j < nbElements; j++)
//                env.out() << j << ") " << cplex.getValue(e[j]) << endl;
//        }
//        catch (IloException& ex) {
//            cerr << "Error: " << ex << endl;
//        }
//        catch (...) {
//            cerr << "Error" << endl;
//        }
//        env.end();}
//
//    void define_data(IloEnv env) {
//        nbElements = 3;
//        nbRaw      = 2;
//        nbScrap    = 2;
//        nbIngot    = 1;
//        alloy      = 71;
//        nm = IloNumArray(env, nbElements, 22.0, 10.0, 13.0);
//        nr = IloNumArray(env, nbRaw, 6.0, 5.0);
//        ns = IloNumArray(env, nbScrap, 7.0, 8.0);
//        ni = IloNumArray(env, nbIngot, 9.0);
//        p = IloNumArray(env, nbElements, 0.05, 0.30, 0.60);
//        P = IloNumArray(env, nbElements, 0.10, 0.40, 0.80);
//        PRaw   = IloNumArray2(env, nbElements);
//        PScrap = IloNumArray2(env, nbElements);
//        PIngot = IloNumArray2(env, nbElements);
//        PRaw[0] = IloNumArray(env, nbRaw, 0.20, 0.01);
//        PRaw[1] = IloNumArray(env, nbRaw, 0.05, 0.00);
//        PRaw[2] = IloNumArray(env, nbRaw, 0.05, 0.30);
//
//        PScrap[0] = IloNumArray(env, nbScrap, 0.00, 0.01);
//        PScrap[1] = IloNumArray(env, nbScrap, 0.60, 0.00);
//        PScrap[2] = IloNumArray(env, nbScrap, 0.40, 0.70);
//
//        PIngot[0] = IloNumArray(env, nbIngot, 0.10);
//        PIngot[1] = IloNumArray(env, nbIngot, 0.45);
//        PIngot[2] = IloNumArray(env, nbIngot, 0.45);
//    }
//private:
//
//};