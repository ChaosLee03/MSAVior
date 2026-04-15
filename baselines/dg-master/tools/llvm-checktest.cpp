////
//// Created by WZXPC on 2023/8/31.
////
//#include <fstream>
//#include <iostream>
//#include <set>
//#include <sstream>
//#include <string>
//#include <regex>
//
//#include <cassert>
//#include <cstdio>
//#include "json.h"
//
//
//#include <llvm/IR/LLVMContext.h>
//#include <llvm/IR/Module.h>
//#include <llvm/IR/Operator.h>
//#include <llvm/Support/raw_ostream.h>
//#if LLVM_VERSION_MAJOR >= 4
//#include <llvm/Bitcode/BitcodeReader.h>
//#include <llvm/Bitcode/BitcodeWriter.h>
//#else
//#include <llvm/Bitcode/ReaderWriter.h>
//#endif
//#include "dg/PointerAnalysis/PointerAnalysisFS.h"
//
//#include "dg/tools/llvm-slicer-opts.h"
//#include "dg/tools/llvm-slicer-utils.h"
//
//#include "dg/util/TimeMeasure.h"
//llvm::cl::opt<bool> enable_debug(
//        "dbg", llvm::cl::desc("Enable debugging messages (default=false)."),
//        llvm::cl::init(false), llvm::cl::cat(SlicingOpts));
//
//int main(int argc, char **argv) {
//    setupStackTraceOnError(argc, argv);
//    SlicerOptions options = parseSlicerOptions(argc, argv);
//
//    if (enable_debug) {
//        DBG_ENABLE();
//    }
//
//    llvm::LLVMContext context;
//    std::unique_ptr<llvm::Module> M =
//            parseModule("llvm-sdg-dump", context, options);
//    if (!M)
//        return 1;
//
//    if (!M->getFunction(options.dgOptions.entryFunction)) {
//        llvm::errs() << "The entry function not found: "
//                     << options.dgOptions.entryFunction << "\n";
//        return 1;
//    }
//
//    return 0;
//}
