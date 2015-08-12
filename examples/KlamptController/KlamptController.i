/* File : KlamptController.i */
%module ExampleKlamptController
%include "std_string.i"
%include "std_map.i"

%{
/* Note : always include headers following the inheritance order */

// Klampt Controller
#include "example_klampt_controller.h"
%}

/* Note : always include headers following the inheritance order */
// Klampt Controller

%include "example_klampt_controller.h"
