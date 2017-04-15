#include "llvm/Pass.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/IR/Verifier.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/CallingConv.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/IR/IRPrintingPasses.h"
#include "llvm/IR/InlineAsm.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Transforms/IPO/PassManagerBuilder.h"
using namespace llvm;

namespace {
  struct FunctionNamePass : public ModulePass {
    static char ID;
    FunctionNamePass() : ModulePass(ID) {}

    bool sgx_debug = false;
    const std::string ecall_prefix = "sgx_";

    virtual bool runOnModule(Module &M) {
      std::vector<std::string> ocall_list;
      std::vector<std::string> ecall_list;

      std::error_code ec;
      raw_fd_ostream log("tsgx_eocalls_log.txt", ec, sys::fs::F_None);

      errs() << "-----FunctionName Pass starts-----\n";
      for (auto&F : M) {
        // Skip if function is forward declared.
        if (F.empty()) {
          continue;
        }

        if (sgx_debug) errs() << "function: " << F.getName() << " - " << F.size() << "\n";

        std::string fun_name = F.getName().str();
        if (fun_name.compare(0, ecall_prefix.size(), ecall_prefix) == 0) {
          ecall_list.push_back(fun_name.substr(4, fun_name.size() - 4));
        } else {
          ocall_list.push_back(fun_name);
        }
      }

      for (auto&F : M) {
        if (sgx_debug) errs() << "I saw function: " << F.getName() << "\n";
      }

      log << "ecall\n";
      for (int i = 0; i < ecall_list.size(); i++) {
        log << ecall_list[i] << "\n";
      }

      log << "ocall\n";
      for (int i = 0; i < ocall_list.size(); i++) {
        log << ocall_list[i] << "\n";
      }
      errs() << "-----FunctionName Pass ends-----\n";

      log.close();

      return false;
    }
  };
}

char FunctionNamePass::ID = 0;

static RegisterPass<FunctionNamePass> X("function-name", "Function Name extraction");

