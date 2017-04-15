#ifndef LLVM_LIB_TARGET_X86_X86CACHEANALYSIS_H
#define LLVM_LIB_TARGET_X86_X86CACHEANALYSIS_H

#include <vector>
#include <map>
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/CodeGen/AsmPrinter.h"

namespace llvm {

  class X86CacheAnalysis {
  public:
    X86CacheAnalysis();
    void doInitialize(MachineFunction &MF, AsmPrinter &AP);
    void doAnalysis();
    void set_count(int num);
    void print_count();

    bool isRAXSrc(unsigned bb_num);
    bool isRAXDst(unsigned bb_num);
    bool isRAXSrcAfterCall(unsigned bb_num, int index);
    bool isRAXDstAfterCall(unsigned bb_num, int index);

    int getBBCacheWayUsage(int bb_num);
    int getJointBBCacheWayUsage(int bb_num1, int bb_num2);
    int getFunCacheWayUsage();

  private:
    const AsmPrinter *AP;
    const MachineFunction *MF;
    const MachineLoopInfo *LI;

    // tarjan algorithm
    int num;
    std::vector<const MachineBasicBlock*> stack;
    std::vector<const MachineBasicBlock*> scc_stack;
    std::vector<const MachineBasicBlock*> sort_stack;
    std::map<const MachineBasicBlock*, int> order;
    std::map<const MachineBasicBlock*, int> link;
    void tarjan_init();
    void tarjan(const MachineBasicBlock *MB);

    // RAX access pattern analysis
    typedef struct {
      bool is_src;
      bool is_dst;
    } access_pattern;

    typedef struct {
      access_pattern bb_pattern;
      std::map<int, access_pattern> call_pattern;
    } rax_access_pattern;

    std::map<unsigned, rax_access_pattern> rax_analysis_result;

    void doRAXPatternAnalysis(const MachineBasicBlock *MB);

    // Cache access analysis

    // Instruction Classification
    typedef enum {
      MEM_IMM,
      MEM_MEM,
      MEM_REG
    } Mem_Type;

    typedef enum {
      Op_MOV,
      Op_LEA,
      Op_MUL,
      Op_IMUL,
      Op_DIV,
      Op_IDIV,
      Op_NEG,
      Op_NOT,
      Op_INC,
      Op_DEC,
      Op_AND,
      Op_OR,
      Op_XOR,
      Op_ADD,
      Op_SUB,
      Op_ADC,
      Op_SBB,
      Op_CMP,
      Op_TEST,
      Op_POP,
      Op_PUSH
    } Instr_Operation;

    typedef enum {
      Ty_I,
      Ty_R,
      Ty_M,
      Ty_RR,
      Ty_RI,
      Ty_RM,
      Ty_MR,
      Ty_MI,
      Ty_RR_REV,
      Ty_RRI,
      Ty_RMI,
    } Instr_Operator_Type;

    void simulateInstruction(const MachineInstr *MI, Instr_Operation OP, Instr_Operator_Type Ty);
    void processInstruction(const MachineInstr *MI);
    void processBaicBlock(const MachineBasicBlock *MB);

    // Value-Set Analysis (VSA)
    typedef enum {
      RGN_GLOBAL,
      RGN_LOCAL,
      RGN_HEAP,
      RGN_NONE
    } VSA_Memory_Region;

    typedef struct {
      unsigned int stride;
      int lower_bound;
      int upper_bound;
      bool has_value;
    } VSA_Strided_Interval;

    typedef struct vsa_alocs{
      VSA_Memory_Region rgn;
      VSA_Strided_Interval values;
      VSA_Strided_Interval addresses;
      int id;
      int src_id;
      struct vsa_alocs *src;
    } VSA_A_Locs;

    typedef struct {
      VSA_A_Locs *alocs;
      int64_t value;
      bool isALocs;
    } ValueOrALocs;

    std::vector<const GlobalValue *> global_value_list;
    std::map<unsigned, VSA_A_Locs> register_alocs;
    std::map<std::string, VSA_A_Locs> global_alocs;
    std::map<int, VSA_A_Locs> local_alocs;

    int alocs_id;

    void VSA_init();
    void init_alocs(VSA_A_Locs *alocs, VSA_Memory_Region rgn);
    void set_alocs(VSA_A_Locs **alocs, const MachineInstr *MI, Mem_Type Ty, int index);

    void mov_alocs(ValueOrALocs *src, VSA_A_Locs *dst);
    void lea_alocs(ValueOrALocs *src, VSA_A_Locs *dst);
    void add_alocs(ValueOrALocs *src, VSA_A_Locs *dst);
    void cmp_alocs(ValueOrALocs *src, VSA_A_Locs *dst);

    std::map<int, std::map<int, std::vector<bool>>> cache_model;
    void classifyMemoryWrite(const MachineInstr *MI, int index);
    void cache_write(int bb_num, int id, int offset);

    // Debug
    typedef enum {
      DBG_DST,
      DBG_SRC
    } Debug_Target_Type;

    void debug_all_operators(const MachineInstr*);
    void debug_reg_alisas(unsigned);
    void debug_operators(const MachineInstr *, Debug_Target_Type, Mem_Type, int);
    void debug_alocs(VSA_A_Locs *alocs, const void *index);
    void debug_cache_access(int bb_num);
    void debug_fun_cache_access();

    int count;
  };
}

#endif
