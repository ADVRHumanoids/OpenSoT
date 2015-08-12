/* File : KlamptController.i */
%module ExampleKlamptController
%include std_string.i
%include std_map.i

%{
/* Note : always include headers following the inheritance order */

// Klampt Controller
#include "include/KlamptController.h"
#include "example_klampt_controller.h"
%}

/* Note : always include headers following the inheritance order */
// Klampt Controller
%include "include/KlamptController.h"
%include "example_klampt_controller.h"

namespace std {
  %template(JntMap) map<string,double>;
};

%feature("autodoc","1");
