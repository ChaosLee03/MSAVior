# intCV


intCV is a tool that used for detecting variable correlation, which based on DG to build System dependence graph. The workflow of "intCV" is as follows: 
The first step is to scan the program's bytecode to obtain attributes of each pair of variables, and then determine whether these variable pairs are related using a pre-trained classifier.

DG is a library containing various bits for program analysis. However, the main motivation of this library is program slicing. The library contains implementation of a pointer analysis, data dependence analysis, control dependence analysis, and an analysis of relations between values in LLVM bitcode. All of the analyses target LLVM bitcode, but most of them are written in a generic way, so they are not dependent on LLVM in particular.


