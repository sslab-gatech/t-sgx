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
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/MemoryBuffer.h"
#include "llvm/Support/LineIterator.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Transforms/IPO/PassManagerBuilder.h"
using namespace llvm;

namespace {
  struct FunctionWrapperPass : public ModulePass {
    static char ID;
    FunctionWrapperPass() : ModulePass(ID) {}

    unsigned getAlignmentByType(Type *Ty) {
      if (Ty->isIntegerTy()) {
        return cast<IntegerType>(Ty)->getBitWidth() / 8;
      }
      if (Ty->isFloatTy()) {
        return 4;
      }
      if (Ty->isDoubleTy()) {
        return 8;
      }

      return 1;
    }

    bool tsgx_debug = false;

    static bool parse_file(std::vector<std::string> *ecall_list, std::vector<std::string> *ocall_list)
    {
      const std::string ecall_string = "ecall";
      const std::string ocall_string = "ocall";

      auto file = MemoryBuffer::getFile("tsgx_eocalls_log.txt");
      if (std::error_code EC = file.getError()) {
          errs() << "Could not open tsgx_eocalls_log.txt\n";
          return false;
      }

      errs() << "Successfully open tsgx_eocalls_log.txt\n";
      line_iterator L(*file.get());
      if ((*L).str().compare(ecall_string)) {
          errs() << "wrong format of tsgx_eocalls_log.txt\n";
          return false;
      }

      ++L;
      while (!L.is_at_end() && (*L).str().compare(ocall_string)) {
          (*ecall_list).push_back((*L).str());
          ++L;
      }

     if (L.is_at_end() || (*L).str().compare(ocall_string)) {
         errs() << "wrong format of tsgx_eocalls_log.txt\n";
         return false;
     }

      ++L;
      while (!L.is_at_end()) {
          (*ocall_list).push_back((*L).str());
          ++L;
      }

      return true;
    }

    virtual bool runOnModule(Module &M) {
      LLVMContext &ctx = M.getContext();

      std::error_code ec;
      raw_fd_ostream log("tsgx_functions_log.txt", ec, sys::fs::F_Append);

      std::vector<std::string> ecall_list;
      std::vector<std::string> ocall_list;

      if (!parse_file(&ecall_list, &ocall_list)) {
          errs() << "Parsing tsgx_eocalls_log.txt failed.\n";
      }

#if 0 // For debugging.
      errs() << "ecall list:\n";
      for (std::vector<std::string>::iterator ei = ecall_list.begin();
                                              ei != ecall_list.end(); ei++) {
          errs() << *ei << "\n";
      }
      errs() << "ocall list:\n";
      for (std::vector<std::string>::iterator oi = ocall_list.begin();
                                              oi != ocall_list.end(); oi++) {
          errs() << *oi << "\n";
      }
#endif

      // Type Definitions

      PointerType* type_ptr_int8  = PointerType::get(Type::getInt8Ty(ctx), 0);

      // Create Function wrappers

      // Find out the functions to be modified.
      std::vector<Function*> fun_list;
      for (auto&F : M) {
        if (tsgx_debug) errs() << "Function: " << F.getName() << " ... ";

        // Skip if the function is forward declared or an ocall.
        // Hard-coded here for skipping tsgx_init function.
        if (F.empty() ||
            F.getName().str() == "tsgx_init" ||
            std::find(ocall_list.begin(), ocall_list.end(), F.getName().str()) != ocall_list.end()) {
          if (tsgx_debug) errs() << "skip\n";
          continue;
        }

        if (std::find(ecall_list.begin(), ecall_list.end(), F.getName().str()) == ecall_list.end()) {
          MDNode *node = MDNode::get(ctx, MDString::get(ctx, "internal"));
          F.setMetadata("sgxtsx.fun.info", node);
          log << F.getName().str() << "\n";
          if (tsgx_debug) errs() << "target function\n";
          continue;
        }

        if (tsgx_debug) errs() << "match in ecall list\n";
        MDNode *node = MDNode::get(ctx, MDString::get(ctx, "internal"));
        F.setMetadata("sgxtsx.fun.info", node);
        log << "_" << F.getName().str() << "\n";
        fun_list.push_back(&F);
      }

      // Modify the original function name by adding prefix "_",
      // and create an new function with the same name of the original function.
      std::vector<Function*> n_fun_list;
      for (std::vector<Function*>::iterator it = fun_list.begin();
           it != fun_list.end(); ++it) {
        Function *o_fun = *it;

        FunctionType *fun_type = o_fun->getFunctionType();
        StringRef n_fun_name(o_fun->getName());
        std::string n_fun_name_str(o_fun->getName().str());
        o_fun->setName("_" + o_fun->getName());
        Function *n_fun = Function::Create(fun_type, o_fun->getLinkage(), StringRef(n_fun_name_str), &M);
        MDNode *node = MDNode::get(ctx, MDString::get(ctx, "external"));
        n_fun->setMetadata("sgxtsx.fun.info", node);

        // Copy attributes
        AttributeSet n_attrs = n_fun->getAttributes();
        n_fun->copyAttributesFrom(o_fun);
        n_fun->setAttributes(n_attrs);

        // Copy the arguments's name and attributes
        AttributeSet o_attrs = o_fun->getAttributes();
        Function::arg_iterator n_arg = n_fun->arg_begin();
        for (Function::const_arg_iterator o_it = o_fun->arg_begin(), o_end = o_fun->arg_end();
             o_it != o_end; ++o_it) {
            AttributeSet attrs = o_attrs.getParamAttributes(o_it->getArgNo() + 1);
            if (attrs.getNumSlots() > 0)
              n_arg->addAttr(attrs);
            n_arg->setName(o_it->getName());
            n_arg++;
        }

        n_fun->setAttributes(
          n_fun->getAttributes()
            .addAttributes(n_fun->getContext(), AttributeSet::ReturnIndex,
                           o_attrs.getRetAttributes())
            .addAttributes(n_fun->getContext(), AttributeSet::FunctionIndex,
                           o_attrs.getFnAttributes()));

        n_fun_list.push_back(n_fun);
      }

      for (auto&F : M) {
        if (tsgx_debug) errs() << "I saw function: " << F.getName() << "\n";
      }

      // Arugument forwarding
      errs() << "-----Function Wrapper Pass start-----\n";
      int fun_index = 0;
      for (std::vector<Function*>::iterator it = n_fun_list.begin();
           it != n_fun_list.end(); ++it) {
        Function *n_fun = *it;
        BasicBlock* label_entry = BasicBlock::Create(ctx, "entry", n_fun, 0);

        // Get argument list
        std::vector<AllocaInst*> allocinst_list;
        for (Function::arg_iterator n_args = n_fun->arg_begin();
             n_args != n_fun->arg_end(); ++n_args) {
          StringRef ptr_name(n_args->getName().str() + ".addr");

          AllocaInst* ptr_addr = new AllocaInst(n_args->getType(), ptr_name.str(), label_entry);
          unsigned align = getAlignmentByType(n_args->getType());
          ptr_addr->setAlignment(align);
          allocinst_list.push_back(ptr_addr);
        }

        //AllocaInst *ptr_args = new AllocaInst(struct_type_list[fun_index], "args", label_entry);
        // TODO: how to set alignment?
        //ptr_args->setAlignment(4);

        std::vector<AllocaInst*>::iterator alloc_it;
        alloc_it = allocinst_list.begin();
        for (Function::arg_iterator args = n_fun->arg_begin();
             args != n_fun->arg_end(); ++args) {
          AllocaInst *allocinst = *alloc_it;
          StoreInst* store = new StoreInst(args, allocinst, false, label_entry);
          unsigned align = getAlignmentByType(args->getType());
          store->setAlignment(align);
          alloc_it++;
        }

        std::vector<Value*> call_params;
        alloc_it = allocinst_list.begin();
        for (Function::arg_iterator args = n_fun->arg_begin();
             args != n_fun->arg_end(); ++args) {
          // Load
          AllocaInst *allocinst = *alloc_it;
          LoadInst* load = new LoadInst(allocinst, "", false, label_entry);
          unsigned align = getAlignmentByType(args->getType());
          load->setAlignment(align);

          call_params.push_back(load);
          alloc_it++;
        }

        Function* callee = M.getFunction(StringRef("_" + n_fun->getName().str()));
        CallInst* call = CallInst::Create(callee, call_params, "", label_entry);
        call->setCallingConv(CallingConv::C);
        call->setTailCall(false);
        AttributeSet call_PAL;
        call->setAttributes(call_PAL);

        Type *ret_type = n_fun->getReturnType();
        if (!ret_type->isVoidTy()) {
          // Return based on the return type
          ReturnInst::Create(ctx, call, label_entry);
        } else {
          // Return void
          ReturnInst::Create(ctx, label_entry);
        }

        fun_index++;
      }
      errs() << "-----Function Wrapper done-----\n";

      log.close();

      return false;
    }
  };
}

char FunctionWrapperPass::ID = 0;

static RegisterPass<FunctionWrapperPass> X("function-wrapper", "TSX transformation");

