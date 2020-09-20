//
// Created by gabri on 09/12/2019.
//
#include "NodeClass.h"
#include <math.h>
#include <arduino.h>

using namespace std;

NodeClass::NodeClass() {

}

NodeClass::NodeClass(int _ID,float _d[3],float _d_av[3],float _y[3], float _k[3], float _c[3], float _o, float _L) {
    ID = _ID;
     d = _d;
     d_av = _d_av;
     y = _y;
     k = _k;
     c= _c;
     o = _o;
     L= _L;
}


float NodeClass :: norm(float _k[3]) {
    k = _k;
    n = 0.0;
    for (int i = 0; i < 3; i++) {
        n = pow(k[i],2) + n;
    }
    return n;
}

float NodeClass :: M(){
    //_n = norm(k);
    m = n - pow(k[ID-1],2);
    return m;

}

float NodeClass :: evaluate_cost(NodeClass node, float _d[3], float rho){
    float cost;
    float _n,aux,aux1;
    _n = 0.0;
    aux = 0.0;
    aux1 = 0.0;
    for (int i =0;i<3;i++) {
        aux = node.c[i] * _d[i] + aux;
        aux1 = node.y[i] * (_d[i] - node.d_av[i]) + aux1;
        _n = pow(d[i] - node.d_av[i], 2) + _n;
    }
        cost = aux + aux1 + (rho/2)*_n;
    return cost;

}

int NodeClass :: check_feasibility(NodeClass node, float _d[3]) {
    int check, i;
    float tol, aux;
    tol = 0.001;
    aux = 0.0;
    if (d[node.ID-1] < -tol) { ///!!!!!!!
        check = 0;
        return check;
    }
    if (d[node.ID-1] > 100 + tol) { ////!!!!!!!!!!!
        check = 0;
        return check;
    }
    for (i = 0; i < 3; i++) {
        aux = _d[i] * node.k[i] + aux;
    }
    if (aux < (node.L - node.o - tol)) {
        check = 0;
        return check;
    }
    check = 1;
    //cout << "\n \n aux: " << aux;
    //cout << "\n \n outra parte: "<< node.L - node.o - tol;
    return check;
}

float * NodeClass :: consensus_iterate(NodeClass node, float rho) {

    static float D[3];
    float Cost;
    static float result[4];
    float d_best[3] = {-1, -1, -1};
    float cost_best = 1000000; //large number
    float cost_unconstrained, cost_boundary_0, cost_boundary_100, cost_linear_0, cost_linear_100, cost_boundary_linear;
    int sol_unconstrained = 1;
    int sol_boundary_linear = 1;
    int sol_boundary_0 = 1;
    int sol_boundary_100 = 1;
    int sol_linear_0 = 1;
    int sol_linear_100 = 1;
    float d_u[3];
    float z[3];
    float d_b1[3], d_bl[3], d_l0[3], d_l1[3], d_b0[3];
    float aux = 0.0;
    int i,j;

    for (i = 0; i < 3; i++) {
        z[i] = rho * node.d_av[i] - node.y[i] - node.c[i];
        d_u[i] = (1 / rho) * z[i];
    }
    
    sol_unconstrained = node.check_feasibility(node, d_u);
    if (sol_unconstrained == 1) {
        cost_unconstrained = node.evaluate_cost(node, d_u, rho);
        if (cost_unconstrained < cost_best) {    
            for (i = 0; i < 3; i++) {
                d_best[i] = d_u[i];
                cost_best = cost_unconstrained;

            }
        }
    }else{

          for(j=0;j<3;j++){
              aux = z[j] * node.k[j] +aux;
          }

          // Compute minimum constrained  to linear boundary

          for(i=0;i<3;i++){
              d_bl[i] = (1 / rho) * z[i] - (node.k[i] / node.n) * (node.o - node.L + (1 / rho)*aux);
          }
  
          //compute minimum constrained to 0 boundary
          for(i=0;i<3;i++){
              d_b0[i] = (1 / rho) * z[i];
          }
          d_b0[node.ID-1] = 0;
  
          //compute minimum constrained to 100 boundary
          for(i=0;i<3;i++){
              d_b1[i] = (1 / rho) * z[i];
          }
          d_b1[node.ID-1] = 100.0;
  
          //compute minimum constrained to linear and 0 boundary
          for(i=0;i<3;i++){
              d_l0[i] = (1 / rho) * z[i] - (1 / node.m) * node.k[i] * (node.o - node.L) +
                    (1 / rho / node.m) * node.k[i] * (node.k[node.ID-1] * z[node.ID-1] - aux);
          }
          d_l0[node.ID-1] = 0.0;
  
          //compute minimum constrained to linear and 100 boundary
          for(i=0;i<3;i++){
              d_l1[i] = (1 / rho) * z[i] - (1 / node.m) * node.k[i] * (node.o - node.L + 100 * node.k[node.ID-1]) +
                    (1 / rho / node.m) * node.k[i] * (node.k[node.ID-1] * z[node.ID-1] -aux);
          }
          
          d_l1[node.ID-1] = 100.0;
          aux = 0.0;
  
      
      
      sol_boundary_linear = node.check_feasibility(node, d_bl);
      sol_boundary_0 = node.check_feasibility(node, d_b0);
      sol_boundary_100 = node.check_feasibility(node, d_b1);
      sol_linear_0 = node.check_feasibility(node, d_l0);
      sol_linear_100 = node.check_feasibility(node, d_l1);
      
      if (sol_boundary_linear == 1) {
          cost_boundary_linear = node.evaluate_cost(node, d_bl, rho);
          if (cost_boundary_linear < cost_best) {
              for(i=0;i<3;i++) {
                  d_best[i] = d_bl[i];
                  cost_best = cost_boundary_linear;
                  
              }
          }
      }
      if (sol_boundary_0 == 1) {
          cost_boundary_0 = node.evaluate_cost(node, d_b0, rho);
          if (cost_boundary_0 < cost_best) {
              for(i=0;i<3;i++) {
                  d_best[i] = d_b0[i];
                  cost_best = cost_boundary_0;
              }
          }
      }
      
      if (sol_boundary_100 == 1) {
          cost_boundary_100 = node.evaluate_cost(node, d_b1, rho);
          if (cost_boundary_100 < cost_best) {
              for(i=0;i<3;i++) {
                  d_best[i] = d_b1[i];
                  cost_best = cost_boundary_100;
              }
          }
      }
      
      if (sol_linear_0 == 1) {
          cost_linear_0 = node.evaluate_cost(node, d_l0, rho);
          if (cost_linear_0 < cost_best) {
              for(i=0;i<3;i++) {
                  d_best[i] = d_l0[i];
                  cost_best = cost_linear_0;
              }
          }
      }
      if (sol_linear_100 == 1) {
          cost_linear_100 = node.evaluate_cost(node, d_l1, rho);
          if (cost_linear_100 < cost_best) {
              for(i=0;i<3;i++) {
                  d_best[i] = d_l1[i];
                  cost_best = cost_linear_100;
              }
          }
      }
    }


    for(i=0;i<3;i++) {
        D[i] =d_best[i];
        result[i] = D[i];
    }
    Cost = cost_best;
    result[3] = Cost;


    return result;
}

