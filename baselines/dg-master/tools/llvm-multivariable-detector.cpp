//
// Created by WZX on 2023/7/5.
//
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <string>

#include <cassert>
#include <cstdio>
#include "json.h"

#include <llvm/IR/Instructions.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Operator.h>
#include <llvm/Support/raw_ostream.h>

#if LLVM_VERSION_MAJOR >= 4
#include <llvm/Bitcode/BitcodeReader.h>
#include <llvm/Bitcode/BitcodeWriter.h>
#else
#include <llvm/Bitcode/ReaderWriter.h>
#endif

#include "dg/PointerAnalysis/PointerAnalysisFI.h"
#include "dg/PointerAnalysis/PointerAnalysisFS.h"
#include "dg/PointerAnalysis/PointerAnalysisFSInv.h"
#include "dg/llvm/DataDependence/DataDependence.h"
#include "dg/llvm/LLVMDG2Dot.h"
#include "dg/llvm/LLVMDependenceGraph.h"
#include "dg/llvm/LLVMDependenceGraphBuilder.h"
#include "dg/llvm/LLVMSlicer.h"
#include "dg/llvm/PointerAnalysis/PointerAnalysis.h"

#include "dg/tools/llvm-slicer-opts.h"
#include "dg/tools/llvm-slicer-utils.h"

#include "dg/util/TimeMeasure.h"



using namespace dg;
using namespace dg::debug;
using namespace std;
using namespace llvm;
using llvm::errs;

const int LINE = 10; // 最长的原子区间长度



// 一个关联变量组
class CorrelatedVars {
    int len; // 这个组里有len个变量
    set<string> vars; // 这个组里的变量
  public:
    explicit CorrelatedVars(const set<string> &vars) : vars(vars) {
        len = vars.size();
    }
};

// 存放所有关联变量
// 用于检查中断里访问的变量是否是具有关联的变量
class AllCorrelatedVars {
    set<string> vars; // 所有关联变量
    int len; // 所有关联变量的个数
  public:
    explicit AllCorrelatedVars(const set<string> &vars) : vars(vars) {
        len = vars.size();
    }
    AllCorrelatedVars() {
        len = 0;
    }
    bool find(string varName) {
        return vars.find(varName) != vars.end();
    }
    bool add(const set<string> &vars) {
        int originlen = this->vars.size();
        this->vars.insert(vars.begin(), vars.end());
        len = this->vars.size();
        return len >= originlen;
    }
};

// 对变量的访问
class varVisit {
    string varName; // 变量名
    int visitMode; // 访问模式 0:读 1:写
    long long int line{}; // 行号
    set<string> *correlatedVars{}; // 指向该变量所属的关联变量组
    public:
    varVisit(const string &varName, int visitMode, long long int line,
             set<string> *correlatedVars)
            : varName(varName), visitMode(visitMode), line(line),
              correlatedVars(correlatedVars) {}
    varVisit(const string &varName, int visitMode, long long int line)
            : varName(varName), visitMode(visitMode), line(line) {}
    varVisit(string varName, int visitMode) : varName(varName), visitMode(visitMode) {}
    string toString() {
        stringstream ss;
        ss << varName << " " << visitMode << " " << line << "\n";
        return ss.str();
    }
};

// 存放某中断函数里的所有关联变量以及访问模式
class InterruptVars {
    string funcName; // 中断函数名
    Function *func; // 中断函数
    vector<varVisit> varVisits; // 中断函数里的所有关联变量以及访问模式
  public:
    InterruptVars(const string &funcName, Function *func,
                  const vector<varVisit> &varVisits)
            : funcName(funcName), func(func), varVisits(varVisits) {}
    InterruptVars() {}
    explicit InterruptVars(Function *func) : func(func) {funcName = func->getName().str();}
    void addVarVisit(const varVisit &varVisit) {
        varVisits.push_back(varVisit);
    }
    int getVarVisitNum() {
        return varVisits.size();
    }
};
class AtomicArea {
    Instruction *Astart; // 原子区间的起始指令
    Instruction *Aend; // 原子区间的结束指令
    set<string> vars;
    int startline, endline; // 原子区的行号范围
  public:
    AtomicArea(Instruction *start, Instruction *anEnd, const set<string> &vars)
            : Astart(start), Aend(anEnd), vars(vars) {
        if (Astart->getDebugLoc()) {
            startline = Astart->getDebugLoc()->getLine();
        }
        if (Aend->getDebugLoc()) {
            endline = Aend->getDebugLoc()->getLine();
        }
    }
    AtomicArea(Instruction *start, Instruction *anEnd)
            : Astart(start), Aend(anEnd) {}
    Instruction *getStart() const {
        return Astart;
    }
    AtomicArea() {}
    Instruction *getEnd() const {
        return Aend;
    }
    void setStart(Instruction *start) {
        AtomicArea::Astart = start;
    }
    void setEnd(Instruction *end) {
        AtomicArea::Aend = end;
    }
    void addVar(const string &var) {
        vars.insert(var);
    }
    void setVars(const set<string> &vars) { AtomicArea::vars = vars; }
    set<string> getVars() {
        return vars;
    }
    string toString() {
        stringstream ss;
        ss << "start: " << Astart->getDebugLoc()->getLine() << " end: " << Aend->getDebugLoc()->getLine() << "\n";
        ss << "start: " << Astart->getOpcodeName() << " " << Astart->getOperand(0)->getName().str() << "\n";
        ss << "end: " << Aend->getOpcodeName() << " " << Aend->getOperand(0)->getName().str() << "\n";
        ss << "vars: ";
        for (auto &var : vars) {
            ss << var << " ";
        }
        ss << "\n";
        return ss.str();
    }
    bool operator< (const AtomicArea &other) const {
        return startline < other.startline;
    }
};


vector<AtomicArea*> AtomicAreas;// 存放所有原子区间



bool checkIfCorrelated(Value* v, AllCorrelatedVars &acv, int depth = 0) {
    if (depth >= 5) { // 递归深度太大，不再递归
        return false;
    }
    //获取变量名
    string name = v->getName().str();
    if (acv.find(name)) { // 找到了，确实是关联变量
        return true;
    }
    if (auto *inst = dyn_cast<Instruction>(v)) {
        for (auto i = 0; i < inst->getNumOperands(); i++) {
            if (checkIfCorrelated(inst->getOperand(i), acv, depth + 1)) { // 递归的去看是不是关联变量，因为SSA
                return true;
            }
        }
    }
    return false;
}
llvm::cl::opt<bool> enable_debug(
        "dbg", llvm::cl::desc("Enable debugging messages (default=false)."),
        llvm::cl::init(false), llvm::cl::cat(SlicingOpts));

int main(int argc, char *argv[]) {
    //读入json文件
    Json::Value reader;
    ifstream ifs("/Users/wzxpc/Downloads/myproject/myoutput/output_data.json");
    if (!ifs.is_open()) {
        std::cerr << "open json file failed" << endl;
        return 0;
    }
    ifs >> reader;
    //解析json文件
    // 假设所有关联变量存储在中allVarSet中
    vector<set<string>> allVarSet;
    for (const auto & item : reader) {
        string varpair = item[0].asString();
        int index = varpair.find(",");
        string var1 = varpair.substr(0, index);
        string var2 = varpair.substr(index + 1);
        set<string> vars;
        int index1 = var1.find(":::") + 3;
        int index2 = var2.find(":::") + 3;
        string var1Name = var1.substr(index1);
        string var2Name = var2.substr(index2);
        cout << var1Name << " " << var2Name << "\n";
        vars.insert(var1Name);
        vars.insert(var2Name);
        allVarSet.push_back(vars);
    }
    unordered_map<string, int> varToSet;
    vector<set<string>> allVarSetNoPair;
    for (auto &var_set : allVarSet) {
        bool found = false;
        int idx = -1;
        for (auto &var : var_set) {
            if (varToSet.count(var)) {
                found = true;
                idx = varToSet[var];
                break;
            }
        }
        if (found) {
            for (auto &var : var_set) {
                allVarSetNoPair[idx].insert(var);
            }
        }
        else {
            allVarSetNoPair.push_back(var_set);
            idx = allVarSetNoPair.size() - 1;
        }
        for (auto &var : var_set) {
            varToSet[var] = idx;
        }
    }
//    cout << "***************************\n";
//    for (auto &s : allVarSetNoPair) {
//        for (auto &var : s) {
//            cout << var << " ";
//        }
//        cout << "\n";
//    }
//    cout << "***************************\n";
    // 检查一个变量属于哪个变量组
    // 假设已经填好了
    unordered_map<string, set<string>> variableToGroup;

//    unordered_map<set<string>, string> groupToInterrupt;
    for (auto &s : allVarSetNoPair) {
        for (auto &var : s) {
            variableToGroup[var] = s;
        }
    }
    AllCorrelatedVars allCorrelatedVars; // 存放所有关联变量，用于判断中断里面有没有访问到关联变量
    for (auto &var : allVarSetNoPair) {
        if (!allCorrelatedVars.add(var)) {
            std::cerr << "add all correlated vars failed" << endl;
            return 1;
        }
    }
    // 放一个存储所有中断函数的容器
    vector<InterruptVars*> interruptVars;

    // 放一个放所有中断里出现过的关联变量的容器
    set<string> CorrelatedVarsInInterrupt;

    setupStackTraceOnError(argc, argv);
    SlicerOptions options = parseSlicerOptions(argc, argv);

    if (enable_debug) {
        DBG_ENABLE();
    }

    llvm::LLVMContext context;
    std::unique_ptr<llvm::Module> M =
            parseModule("llvm-dg-dump", context, options);
    if (!M)
        return 1;
    // 遍历每个函数
    for (auto &F : *M) {
        // 获取函数名
        string funcName = F.getName().str();
        // 判断是不是中断函数
        if (funcName.find("interrupt") != string::npos) {
            // 如果是中断函数,检查中断里访问了哪些关联变量
            // 获取这个函数的所有关联变量
            InterruptVars *interruptVar = new InterruptVars(&F);
            // 遍历这个函数
            cout << "正在遍历" << funcName << "\n";
            for (auto &BB : F) {
                // 遍历这个基本块
                for (auto &I : BB) {
                    // 获取这条指令访问的变量
                    for (int i = 0; i < I.getNumOperands(); i++) {
                        auto *op = I.getOperand(i);
                        string varName = op->getName().str();
                        // 判断这个变量是不是关联变量
                        if (allCorrelatedVars.find(varName)) {
                            // 如果是关联变量,获取对这个变量的访问
                            // 判断读写类型
                            int visitMode = 0; // 0:读 1:写
                            if (I.getOpcode() == Instruction::Load) {
                                // 如果是读,把这个变量加入到varSet中
                                visitMode = 0;
                            } else if (I.getOpcode() == Instruction::Store) {
                                // 如果是写,把这个变量加入到varSet中
                                visitMode = 1;
                            } else if (I.getOpcode() == Instruction::Call) {
                                // 如果是读,把这个变量加入到varSet中
                                visitMode = 0;
                            }
                            // 获取行号
                            long long int line = I.getDebugLoc().getLine();
                            // 添加它属于哪个变量组
                            set<string> *correlatedVars = &variableToGroup[varName];
                            // 添加一个访问
                            varVisit visit(varName, visitMode, line, correlatedVars);
                            interruptVar->addVarVisit(visit);
                            CorrelatedVarsInInterrupt.insert(varName);
                            outs() << visit.toString();
                        }
                        // 如果不是就不管
                    }
                }
            }
            // 把这个中断函数加入到中断函数容器中
            if (interruptVar->getVarVisitNum() > 0) {
                interruptVars.push_back(interruptVar);
            } else {
                delete interruptVar;
            }
        }
    }
    interruptVars; // 这里面应该存放了所有的中断函数，存放了这个函数访问的所有关联变量和它所属的变量组
    // 应该把所有的中断函数里访问的关联变量组都收集起来，然后在主程序里找有没有对这些变量的访问,这主要是为了在主程序里划定原子区
    set<int> lines; // 这些存放的行号代表已经生成原子区了
    for (auto &F : *M) {
        string funcName = F.getName().str();
        if (funcName.find("interrupt") == string::npos) {
            // 不是中断函数就检查
            for (auto &BB : F) {
                for (auto iter = BB.begin(); iter != BB.end(); iter++) {
                    auto &I = *iter;
                    // 检查是否访问了中断里出现的关联变量
                    int line = 0;
                    if (I.getDebugLoc()) {
                        line = I.getDebugLoc().getLine();
                    }
                    else
                        continue ;
                    if (lines.find(line) != lines.end())
                        continue ;
                    for (int i = 0; i < I.getNumOperands(); i++) {
                        auto *op = I.getOperand(i);
                        string varName = op->getName().str();
                        if (CorrelatedVarsInInterrupt.find(varName) != CorrelatedVarsInInterrupt.end()) {
                            lines.insert(line);
                            // 如果访问了中断里出现的关联变量，检查是哪个变量组的，再生成原子区
                            // 判断读写类型
                            int visitMode = 0; // 0:读 1:写
                            if (I.getOpcode() == Instruction::Store)
                                visitMode = 1;
                            set<string>varsgroup = variableToGroup[varName];
                            // 生成原子区
                            AtomicArea *ar = new AtomicArea();
                            ar->setStart(&I);
                            ar->setVars(varsgroup);
                            ar->setEnd(&I);
                            // 获取这个基本块的最后一个节点
                            Instruction *end = BB.getTerminator();

                            if (auto *br = dyn_cast<BranchInst>(&I)) {
                                if (br->isConditional()) {
                                    // 如果是一个条件分支跳转
                                    // 判断一下这个条件分支跳转的条件是不是关联变量
                                    auto *cond = br->getCondition();
                                    if (checkIfCorrelated(cond, allCorrelatedVars)) {
                                        /*
                                         * 如果是关联变量，那么就要把这个if的结构找出来都加入到原子区里(如果里面访问到了这一组的关联变量)
                                         */
                                        BasicBlock::iterator iter2 = iter;
                                        int count = 0;
                                        while (count < LINE) {
                                            for (int i = 0; i < iter2->getNumOperands(); i++) {
                                                auto *op1 = iter2->getOperand(i);
                                                string varName1 = op1->getName().str();
                                                if (varsgroup.find(varName1) != varsgroup.end()) {
                                                    ar->setEnd(&*iter2);
                                                }
                                            }
                                            iter2++;
                                            count++;
                                        }
                                    }
                                }
                            } else {
                                BasicBlock::iterator iter2 = iter;
                                int count = 0;
                                while (iter2 != BB.end() && count < LINE) {
                                    //                                outs() << *iter2 << "\n";
                                    for (int i = 0; i < iter2->getNumOperands(); i++) {
                                        auto *op1 = iter2->getOperand(i);
                                        string varName1 = op1->getName().str();
                                        if (varsgroup.find(varName1) != varsgroup.end()) {
                                            ar->setEnd(&*iter2);
                                        }
                                    }
                                    if (&*iter2 == end)
                                        break;
                                    iter2++;
                                    count++;
                                }
                            }

                            AtomicAreas.push_back(ar);
                        }
                    }
                }
            }
        }
    }
    AtomicAreas.erase(
            std::remove_if(AtomicAreas.begin(), AtomicAreas.end(),
                           [](const AtomicArea* area) {
                                return std::any_of(AtomicAreas.begin(), AtomicAreas.end(),
                                                  [area](const AtomicArea* other) {
                                                        return other!=area &&
                                                        other->getStart() <= area->getStart() &&
                                                        other->getEnd() >= area->getEnd();
                                                  });
                           }), AtomicAreas.end()
            );
    std::sort(AtomicAreas.begin(), AtomicAreas.end(),[](const AtomicArea* area1, const AtomicArea* area2) {
        return area1->getStart() < area2->getStart();
    });


    for (auto it : AtomicAreas) {
        outs() << it->toString();
        outs() << "--------------------\n";
    }
}