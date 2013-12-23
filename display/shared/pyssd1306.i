%feature("autodoc", "1");

%module(docstring="SSD1306 Interface") pyssd1306
%{
   #include "pyssd1306.h"
%}

%include "stdint.i"
%include "pyssd1306.h"
