#include <COptimizer_callback.h>

callback_class::callback_class(std::vector<GRBVar>* R, std::vector<GRBVar>* D) {
    R = R_new;
    D = D_new;
}

void callback_class::callback() {
      try {
        if (where == GRB_CB_MIPSOL) {
          // Found an integer feasible solution - does it visit every node?
          double **x = new double*[n];
          int *tour = new int[n];
          
        }
      } catch (GRBException e) {
        cout << "Error number: " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
      } catch (...) {
        cout << "Error during callback" << endl;
      }
}