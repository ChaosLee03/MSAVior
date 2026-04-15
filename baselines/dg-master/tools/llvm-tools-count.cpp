//
// Created by WZXPC on 2023/7/26.
//
#include <cassert>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <vector>
#include <string>

#include "dg/tools/llvm-slicer-opts.h"
#include "dg/tools/llvm-slicer-utils.h"
#include "dg/tools/llvm-slicer.h"

#if LLVM_VERSION_MAJOR >= 4
#include <llvm/Bitcode/BitcodeReader.h>
#include <llvm/Bitcode/BitcodeWriter.h>
#else
#include <llvm/Bitcode/ReaderWriter.h>
#endif

#include <llvm/IR/InstIterator.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Operator.h>

#include <llvm/Support/raw_ostream.h>

#include "dg/llvm/SystemDependenceGraph/SDG2Dot.h"
#include "dg/util/debug.h"
#include "json.h"
using namespace std;
llvm::cl::opt<bool> enable_debug(
        "dbg", llvm::cl::desc("Enable debugging messages (default=false)."),
        llvm::cl::init(false), llvm::cl::cat(SlicingOpts));
void recordVariables(llvm::Function &F, std::set<llvm::Value*> &variables) {
    for (llvm::BasicBlock &BB : F) {
        for (llvm::Instruction &I : BB) {
            if (isa<llvm::AllocaInst>(&I)) {
                variables.insert(&I);
            }
            for (unsigned int i = 0; i < I.getNumOperands(); i++) {
                llvm::Value *operand = I.getOperand(i);

                if (llvm::isa<llvm::Value>(operand) && !variables.count(operand)) {
                    variables.insert(operand);
                }
            }
        }
    }
}
int main(int argc, char *argv[]) {
    setupStackTraceOnError(argc, argv);
    SlicerOptions options = parseSlicerOptions(argc, argv);

    if (enable_debug) {
        DBG_ENABLE();
    }

    llvm::LLVMContext context;
    std::unique_ptr<llvm::Module> M =
            parseModule("llvm-sdg-dump", context, options);
    if (!M)
        return 1;

    if (!M->getFunction(options.dgOptions.entryFunction)) {
        llvm::errs() << "The entry function not found: "
                     << options.dgOptions.entryFunction << "\n";
        return 1;
    }
    int globalvars = 0;
    for (auto &g : M->globals()) {
        globalvars++;
        for (auto &U : g.uses()) {
            llvm::User *user = U.getUser();
            if (auto *I = llvm::dyn_cast<llvm::Instruction>(user)) {

            }
        }
    }
    int Fcount = 0, BBcount = 0, ISRcount = 0;
    int noglobalvars = 0;
    set<llvm::Value*> variables;
    set<llvm::Value*> ISRvariables;
    set<llvm::Value*> Functionvariables;
    for (auto &F : *M) {
        auto SubProgram = F.getSubprogram();
        if (!SubProgram)
            continue ;
        Fcount++;
        for (auto &BB : F) {
            BBcount++;
            for (auto &I : BB) {
                for (unsigned int i = 0; i < I.getNumOperands(); i++) {
                    if (!isa<llvm::AllocaInst>(I))
                        continue;
                    noglobalvars++;
                    auto *alloca = dyn_cast<llvm::AllocaInst>(&I);
                    auto v = alloca->getOperand(0);
                    llvm::outs() << *alloca << "\n";
                    llvm::outs() << *v << "\n";
                    llvm::Value *op = I.getOperand(i);
                    llvm::outs() << *op << "\n";
                    if (isa<llvm::Constant>(op))
                        continue;
                    if (llvm::isa<llvm::Value>(op) && !variables.count(op)) {
                        variables.insert(op);
                        llvm::outs() << *op << "\n";
                    }
                }
            }
        }
    }
    llvm::outs() << "Function count: " << Fcount << "\n";
    llvm::outs() << "Basic block count: " << BBcount << "\n";
    llvm::outs() << "Variable count: " << noglobalvars + globalvars - 1 << "\n";// - 1是减去自动生成的%retval
    llvm::outs() << "Global variable count: " << globalvars << "\n";
    set<llvm::Value*> interruptVariables;
    set<llvm::Value*> normalFunctionVariables;
    for (auto &F : *M) {
        auto SubProgram = F.getSubprogram();
        if (!SubProgram)
            continue ;
        if (F.getName().endswith("ISR")) {
            recordVariables(F, interruptVariables);
            ISRcount++;
        }
        else
            recordVariables(F, normalFunctionVariables);
    }
    set<llvm::Value*> sharedVariables;
    for (auto *var : interruptVariables) {
        if (normalFunctionVariables.count(var)) {
            sharedVariables.insert(var);
            llvm::outs() << *var << "   =========\n";
        }
    }
    llvm::outs() << "ISR count: " << ISRcount << "\n";
    llvm::outs() << "Shared variable count: " << sharedVariables.size() << "\n";
}