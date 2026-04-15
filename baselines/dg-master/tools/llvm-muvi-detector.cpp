//
// Created by WZXPC on 2023/7/25.
//
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <regex>

#include <cassert>
#include <cstdio>
#include "json.h"

#include <llvm/IR/Operator.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
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
#include <chrono>

llvm::cl::opt<bool> enable_debug(
        "dbg", llvm::cl::desc("Enable debugging messages (default=false)."),
        llvm::cl::init(false), llvm::cl::cat(SlicingOpts));


using namespace dg;
using namespace dg::debug;
using namespace std;
using namespace llvm;
using llvm::errs;
enum RWtype {
    read,
    write
};
class Record{
    string name;
    int linelocation;
    RWtype type;
    int id;
  public:
    Record() {}
    Record(string name) : name(name) { linelocation = 9999;}
    Record(string name, int linelocation, RWtype type)
            : name(std::move(name)), linelocation(linelocation), type(type) {}

    string getName() { return name; }
    int getLineLocation() { return linelocation; }
    RWtype getType() { return type; }
    int getId() { return id; }
    void setid(int id) { this->id = id; }
    int getLine() { return linelocation;}
    struct cmp {
        bool operator()(Record *a, Record *b) const {
            return a->getName() != b->getName();
        }
    };
    string toString() {
        string str;
        str += name;
        str += " ";
        str += to_string(linelocation);
        str += " ";
        str += to_string(type);
        return str;
    }
};
class FuncVariable {
    string funcName;
    set<Record*> variables;
    int startline, endline;
  public:
    FuncVariable() {}
    FuncVariable(string funcName) : funcName(funcName) {}
    void addVariable(Record *variable) { variables.insert(variable); }
    void setStartLine(int startline) { this->startline = startline; }
    void setEndLine(int endline) { this->endline = endline; }
    int getStartLine() { return startline; }
    int getEndLine() { return endline; }
    string getFuncName() { return funcName; }
    set<Record*> getVariables() { return variables; }
    int getLength() { return variables.size();}
    struct cmp {
        bool operator()(FuncVariable *a, FuncVariable *b) const {
            return a->getFuncName() != b->getFuncName();
        }
    };
};
class SequenceData {
    string sequence;
    int sup;
    double conf;

  public:
    SequenceData(const string &sequence, int sup, double conf)
            : sequence(sequence), sup(sup), conf(conf) {}
    SequenceData() {}
    explicit SequenceData(const string &sequence) : sequence(sequence) {}
    const string &getSequence() const { return sequence; }
    void setSequence(const string &sequence) {
        SequenceData::sequence = sequence;
    }
    int getSup() const { return sup; }
    void setSup(int sup) { SequenceData::sup = sup; }
    double getConf() const { return conf; }
    void setConf(double conf) { SequenceData::conf = conf; }
    void autoSup() {
        auto pos1 = sequence.find("#SUP:");
        auto pos2 = sequence.find("#CONF:");
        string supstr = sequence.substr(pos1 + 6, pos2 - pos1 - 6);
        sup = stoi(supstr);
        string confstr = sequence.substr(pos2 + 7);
//        cout << supstr << " " << confstr << "\n";
        conf = stod(confstr);
    }
};
class CloseVariable{
    set<string> variables;

};
bool isprime(int a) {
        if (a == 1)
                return false;
        for (int i = 2; i <= sqrt(a); i++) {
                if (a % i == 0)
                return false;
        }
        return true;
}
bool isStructArray(Type* type) {
    if (ArrayType* arrayType = dyn_cast<ArrayType>(type)) {
        Type* elementType = arrayType->getElementType();
        return isStructArray(elementType);
    } else if (llvm::StructType* structType = dyn_cast<StructType>(type)) {
        // 全局变量是结构体类型
        return true;
    }
    return false;
}

vector<Record*> AllRecord;
map<string, int> name2id;
map<int, string> id2name;
vector<set<Record*>> closeRecord;
const int linedistance = 10;
vector<set<string>>nameinfunc;
vector<set<int>>idinfunc;
vector<set<string>>nameinall;
vector<set<int>>idinall;
vector<FuncVariable*> allfunc;
int main(int argc, char *argv[]) {
    auto start_time = std::chrono::steady_clock::now();
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

    for (auto &F : *M) {
        FuncVariable *func = new FuncVariable(F.getName().str()); // 创建一个收集函数里所有全局变量的东西
        auto subprogram = F.getSubprogram();
        if (!subprogram)
            continue;
        auto startline = subprogram->getLine();
        int endline = 0;
        for (auto &BB : F) {
            for (auto &I : BB) {
                if (I.getDebugLoc()) {
                    int line = I.getDebugLoc().getLine();
                    if (line > endline) {
                        endline = line;
                    }
                    RWtype type = read;
                    if (isa<StoreInst>(I)) {
                        type = write;
                    }
                    string name = "";
                    if (GetElementPtrInst *gepinst = dyn_cast<GetElementPtrInst>(&I)) {
                        //数组下标是变量
                        if (auto *g0 = gepinst->getOperand(0)) {
                            if (isa<GlobalVariable>(g0)) {
                                // 不考虑数组下标是变量的情况
                            }
                        }
                    }
                    else {
                        int nums = I.getNumOperands();
                        for (int i = 0; i < nums; i++) {
                            auto *op = I.getOperand(i);
                            // 直接就是一个普通的全局变量
                            if (isa<GlobalVariable>(op)) {
                                GlobalVariable *GV = dyn_cast<GlobalVariable>(op);
                                name = GV->getName().str();
                                if (name == "llvm.global_ctors" ||
                                    name == "llvm.global_dtors" ||
                                    name == "llvm.dgb.declares") {
                                    continue;
                                }
                                Record *record = new Record(name, line, type);
                                AllRecord.push_back(record);
                                if (name2id.count(name) == 0) {
                                    name2id[name] = name2id.size();
                                    id2name[name2id.size() - 1] = name;
                                    record->setid(name2id.size() - 1);
                                }
                                else {
                                    record->setid(name2id[name]);
                                }
                                func->addVariable(record);
                            }
                            else if (isa<GEPOperator>(op)) {
                                GEPOperator *GEPOp = dyn_cast<GEPOperator>(op);
                                int num = GEPOp->getNumOperands();
                                int level = num - 2; //剩下的是偏移的层数
                                auto *basevalue = GEPOp->getOperand(0);
                                if (!isa<GlobalVariable>(basevalue)) {
                                    continue;
                                }
                                auto *basevariable = dyn_cast<GlobalVariable>(basevalue);
                                if (auto *PT = dyn_cast<PointerType>(basevalue->getType())) {
                                    string fullname = "";
                                    fullname += basevalue->getName().str();
                                    string base = fullname;
                                    if (auto *AT = dyn_cast<ArrayType>(PT->getElementType())) {
                                        //是数组
                                        Type *ElementTypeOfAT = AT->getElementType();
                                        string plusname;


                                        if (auto *Idx = GEPOp->getOperand(2)) {
                                            //获取第一个维度的下标
                                            if (auto *CIdx = dyn_cast<ConstantInt>(Idx)) {
                                                unsigned idx = CIdx->getZExtValue();
                                                fullname += "[" + to_string(idx) + "]";
                                            }
                                            else {
                                                //下标不是一个ConstantInt

                                            }
                                        }
                                        //判断操作数的类型，是不是高维数组
                                        Value *ope = GEPOp->getOperand(0);
                                        //outs() << *ope << "\n";
                                        int dimension = 1;//记录数组的维度
                                        if (auto *pointertype = dyn_cast<PointerType>(ope->getType())) {
                                            if (auto *arraytype = dyn_cast<ArrayType>(pointertype->getElementType())) {
                                                while (auto *nexttype = dyn_cast<ArrayType>(arraytype->getElementType())) {
                                                    dimension++;
                                                    arraytype = nexttype;
                                                }
                                            }
                                        }
                                        int tempdimension = dimension;
                                        for (unsigned int i = 3; i < GEPOp->getNumOperands() && dimension > 1; i++, dimension--) {
                                            //获取其他维度下标
                                            if (auto *Idx = GEPOp->getOperand(i)) {
                                                if (auto *CIdx = dyn_cast<ConstantInt>(Idx)) {
                                                    unsigned idx = CIdx->getZExtValue();
                                                    fullname += "[" + to_string(idx) + "]";
                                                }
                                            }
                                        }
                                        if (isStructArray(ElementTypeOfAT)) {
                                            //是结构体数组
                                            level = level - tempdimension + 1;
                                            auto mn = dyn_cast<MDNode>(basevariable->getMetadata("dbg")->getOperand(0));
                                            if (auto *mn2 = dyn_cast<MDNode>(mn->getOperand(3))) {
                                                if (auto *mn3 = dyn_cast<MDNode>(mn2->getOperand(3))) {
                                                    if (auto *mn3dot5 = dyn_cast<MDNode>(mn3->getOperand(4))) {
                                                        function<void(int, MDNode*)> f = [&] (int depth, MDNode* mn0) { //处理结构体嵌套
                                                            if (depth == level)
                                                                return;
                                                            unsigned idx = cast<ConstantInt>(GEPOp->getOperand(tempdimension + 1 + depth))->getZExtValue();
                                                            if (auto *mn4 = dyn_cast<MDNode>(mn0->getOperand(idx))) {
                                                                if (MDString *mds = dyn_cast<MDString>(mn4->getOperand(2))) {
                                                                    plusname = plusname + "." + mds->getString().str();
                                                                }
                                                                if (depth + 1 == level)
                                                                    return;
                                                                if (auto *mn5 = dyn_cast<MDNode>(mn4->getOperand(3))) {
                                                                    if (auto *mn6 = dyn_cast<MDNode>(mn5->getOperand(4))) {
                                                                        f(depth + 1, mn6);
                                                                    }
                                                                }
                                                            }
                                                        };
                                                        f(1, mn3dot5);
                                                    }

                                                }
                                            }
                                        }
                                        fullname += plusname;
                                        if (fullname.find('[') == string::npos)
                                            continue;
                                        Record *record = new Record(fullname, line, type);
                                        AllRecord.push_back(record);
                                        if (name2id.count(fullname) == 0) {
                                            name2id[fullname] = name2id.size();
                                            id2name[name2id.size() - 1] = fullname;
                                            record->setid(name2id.size() - 1);
                                        }
                                        else {
                                            record->setid(name2id[fullname]);
                                        }
                                        func->addVariable(record);
                                    }
                                    else if (auto *ST = dyn_cast<StructType>(PT->getElementType())) {
//                                        outs() << "结构体\n";
                                        //是结构体
                                        string structname = std::move(fullname);
                                        //成员的名称
                                        string fieldname;

                                        auto mn = dyn_cast<MDNode>(basevariable->getMetadata("dbg")->getOperand(0));
                                        if (auto *mn2 = dyn_cast<MDNode>(mn->getOperand(3))) {
                                            if (auto *mn3 = dyn_cast<MDNode>(mn2->getOperand(4))) {
                                                function<void(int, MDNode*)> f = [&] (int depth, MDNode* mn0) { //处理结构体嵌套
                                                    if (depth == level)
                                                        return;
                                                    unsigned idx = cast<ConstantInt>(GEPOp->getOperand(2 + depth))->getZExtValue();
                                                    if (auto *mn4 = dyn_cast<MDNode>(mn0->getOperand(idx))) {
                                                        if (MDString *mds = dyn_cast<MDString>(mn4->getOperand(2))) {
                                                            fieldname = fieldname + "." + mds->getString().str();
                                                        }
                                                        if (depth + 1 == level)
                                                            return;
                                                        if (auto *mn5 = dyn_cast<MDNode>(mn4->getOperand(3))) {
                                                            if (auto *mn6 = dyn_cast<MDNode>(mn5->getOperand(4))) {
                                                                f(depth + 1, mn6);
                                                            }
                                                        }
                                                    }
                                                };
                                                f(0, mn3);
                                            }
                                        }
                                        fullname = structname + fieldname;
                                        outs() << fullname << "\n";
                                        if (fullname.find('[') == string::npos)
                                            continue;
                                        Record *record = new Record(fullname, line, type);
                                        AllRecord.push_back(record);
                                        if (name2id.count(fullname) == 0) {
                                            name2id[fullname] = name2id.size();
                                            id2name[name2id.size() - 1] = fullname;
                                            record->setid(name2id.size() - 1);
                                        }
                                        else {
                                            record->setid(name2id[fullname]);
                                        }
                                        func->addVariable(record);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        func->setStartLine(startline);
        func->setEndLine(endline);
        allfunc.push_back(func);
    }
//    outs() << "383\n";
//    for (auto &r : AllRecord) {
//        outs() << r->getName() << " " << r->getLine() << " " << r->getType() << "\n";
//    }
    outs() << AllRecord.size() << "\n";
    std::sort(AllRecord.begin(), AllRecord.end(), [&] (Record *a, Record *b) -> bool {
        if (!a)
            return false;
        if (!b)
            return true;
        outs() << a->getName() << " " << b->getName() << "\n";
        outs() << a->getLine() << " " << b->getLine() << "\n";
        return a->getLine() >= b->getLine();
    });
    for (auto &f : allfunc) {
        for (int line = f->getStartLine(); line <= f->getEndLine(); line++) {
            set<Record*> temp;
            for (auto r : f->getVariables()) {
                int theline = r->getLine();
                if (theline >= line + linedistance || theline > f->getEndLine())
                    break ;
                if (theline < line)
                    continue;
                temp.insert(r);

            }
            if (temp.size()) {
                closeRecord.push_back(temp);
                for (auto &r: temp) {
                    outs() << r->getName() << " " << r->getLine() << "\n";
                }
                outs() << "--------------------\n";
            }
        }
    }
    // Resolve paths relative to the input bitcode file's directory
    string basedir = ".";
    string inputfile = options.inputFile;
    auto lastslash = inputfile.find_last_of("/\\");
    if (lastslash != string::npos) {
        basedir = inputfile.substr(0, lastslash);
    }

    string outputpath = basedir + "/muvi_output.txt";
    ofstream output(outputpath);
    for (auto &rset : closeRecord) {
        set<int> idset;
        for (auto &r : rset) {
            outs() << r->getName() << " " << r->getId() << "\n";
            idset.insert(r->getId());
        }
        for (auto &id : idset) {
            output << id << " ";
        }
        output << "\n";
        outs() << "--------------------\n";
    }
    output.close();

    // Locate spmf jar: check several candidate paths
    string spmfjar;
    {
        vector<string> candidates = {
            basedir + "/spmf-1.7.jar",
            "spmf-1.7.jar",
            "../tools/spmf-1.7.jar",
            "tools/spmf-1.7.jar",
        };
        for (auto &c : candidates) {
            ifstream testjar(c);
            if (testjar.good()) { spmfjar = c; break; }
        }
        if (spmfjar.empty()) {
            errs() << "Error: cannot find spmf-1.7.jar\n";
            return 1;
        }
    }

    string minningresultpath = basedir + "/muvi_mining_result.txt";
    string javacmd = "java -jar \"" + spmfjar + "\" run \"Closed_association_rules(using_fpclose)\"";
    javacmd += " \"" + outputpath + "\"";
    javacmd += " \"" + minningresultpath + "\"";
    string support = "0";
    string conf = "0";
    javacmd += " \"" + support + "\" \"" + conf + "\"";
    outs() << "Running: " << javacmd << "\n";
    system(javacmd.c_str());
    ifstream minningresult(minningresultpath);
    ofstream output2(basedir + "/muvi_result.txt");
    if (!output2.is_open()) {
        outs() << "open failed\n";
        return 0;
    }
    string line;
    vector<SequenceData*> Seqs;
    int num = 0;
    ofstream outid2name(basedir + "/muvi_id2name.txt");
    for (auto &p : id2name) {
        outid2name << p.first << " " << p.second << "\n";
    }
    outid2name.close();
    while (getline(minningresult, line)) {
        auto pos = line.find('#');
        SequenceData *sequenceData = new SequenceData(line);
        sequenceData->autoSup();
        Seqs.push_back(sequenceData);
        string left = line.substr(0, pos);
        string right = line.substr(pos);
        regex pattern("[0-9]+|==>");
        smatch smatch1;
        string newleft = "";
        while (regex_search(left, smatch1, pattern)) {
            string temp = smatch1[0];
            if (temp[0] != '=') {
                int id = stoi(temp);
                newleft += id2name[id] + " ";
            }
            else {
                newleft += "==> ";
            }
            left = smatch1.suffix();
        }
        line = newleft + right;
        sequenceData->setSequence(line);
        cout << line << "\n";

    }
    std::sort(Seqs.begin(), Seqs.end(), [](SequenceData *a, SequenceData *b) {
        if (a->getConf() != b->getConf())
            return a->getConf() > b->getConf();
        return a->getSup() > b->getSup();
    });
    for (auto &Seq : Seqs) {
        if (isprime(num)) {
            num++;
            continue ;
        }

        output2 << Seq->getSequence() << "\n";

    }
    output2.close();
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    std::cout << "程序运行时间长度：" << duration << " 毫秒" << std::endl;
    return 0;
}
// afs
// afsgg
//
//
// fgd