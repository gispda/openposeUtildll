/* File : zedservice.i */
%module(threads="1") zedservice

%include "std_string.i"

%{
#include "zedservice.h"
%}

/* Let's just grab the original header file here */
%include "zedservice.h"
