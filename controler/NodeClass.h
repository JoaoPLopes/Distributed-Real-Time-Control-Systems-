#ifndef NodeClass_H
#define NodeClass_H
// The consensus function returns a vector with 4 elements
// The 3 first are related to D, and the last one is the cost.


class NodeClass{
public:
   int ID;      //index
   float *d;    // coupling
   float *d_av; // coupling averages
   float *y;    //
   float *k;    // Gains
   float *c;    // cost _c;
   float o;     // external iluminance
   float L;    // Target iluminance
   float n;    // norm (doesn't need initialization)
   float m;     // m (doesn't need initialization)
 // Methods
//public:
    NodeClass();
    NodeClass(int ID, float _d[3],float _d_av[3],float _y[3],  float _k[3], float _c[3], float _o, float _L); // Constructor
    float norm(float _k[3]); // return n
    float M();              // return m
    float evaluate_cost(NodeClass node, float _d[3], float rho);
    int check_feasibility(NodeClass node, float _d[3]);
    float * consensus_iterate(NodeClass node, float rho); //Returns a vector with the result. The 3 first positions are D and the Last is the Cost
   // float  cost_consensus_iterate(NodeClass node, float rho); // Return the Cost calculated



};

#endif
