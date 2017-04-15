#include "X86InstrInfo.h"
#include "X86CacheAnalysis.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include <iostream>
#include <algorithm>
using namespace llvm;

#define DEBUG_TYPE "x86cacheanalysis"

bool tsgx_debug = false;

// Utilities
static unsigned get_reg_alisas(unsigned reg) {
  unsigned alias_reg;

  switch (reg) {
  case X86::AL:
  case X86::AX:
  case X86::EAX:
  case X86::RAX:
    alias_reg = X86::RAX;
    break;
  case X86::BL:
  case X86::BX:
  case X86::EBX:
  case X86::RBX:
    alias_reg = X86::RBX;
    break;
  case X86::CL:
  case X86::CX:
  case X86::ECX:
  case X86::RCX:
    alias_reg = X86::RCX;
    break;
  case X86::DL:
  case X86::DX:
  case X86::EDX:
  case X86::RDX:
    alias_reg = X86::RDX;
    break;
  case X86::SIL:
  case X86::SI:
  case X86::ESI:
  case X86::RSI:
    alias_reg = X86::RSI;
    break;
  case X86::DIL:
  case X86::DI:
  case X86::EDI:
  case X86::RDI:
    alias_reg = X86::RDI;
    break;
  case X86::R8B:
  case X86::R8W:
  case X86::R8D:
  case X86::R8:
    alias_reg = X86::R8;
    break;
  case X86::R9B:
  case X86::R9W:
  case X86::R9D:
  case X86::R9:
    alias_reg = X86::R9;
    break;
  case X86::R10B:
  case X86::R10W:
  case X86::R10D:
  case X86::R10:
    alias_reg = X86::R10;
    break;
  case X86::R11B:
  case X86::R11W:
  case X86::R11D:
  case X86::R11:
    alias_reg = X86::R11;
    break;
  case X86::R12B:
  case X86::R12W:
  case X86::R12D:
  case X86::R12:
    alias_reg = X86::R12;
    break;
  case X86::R13B:
  case X86::R13W:
  case X86::R13D:
  case X86::R13:
    alias_reg = X86::R13;
    break;
  case X86::RBP:
  case X86::EBP:
    alias_reg = X86::RBP;
    break;
  case X86::RSP:
  case X86::ESP:
    alias_reg = X86::RSP;
    break;
  default:
    alias_reg = reg;
  }

  return alias_reg;
}

X86CacheAnalysis::X86CacheAnalysis() {
  AP = nullptr;
  MF = nullptr;
  LI = nullptr;
  count = 0;
}

void X86CacheAnalysis::doInitialize(MachineFunction &MF, AsmPrinter &AP) {
  this->MF = &MF;
  this->AP = &AP;
  this->LI = AP.LI;

  if (!rax_analysis_result.empty())
    rax_analysis_result.clear();
}

// tarjan algorithm
void X86CacheAnalysis::tarjan_init() {
  num = 0;

  if (!stack.empty())
    stack.clear();

  if (!scc_stack.empty())
    scc_stack.clear();

  if (!sort_stack.empty())
    sort_stack.clear();

  if (!order.empty())
    order.clear();

  if (!link.empty())
    link.clear();

}

void X86CacheAnalysis::tarjan(const MachineBasicBlock *MB) {
#if 0 // Enable if needed for debugging.
  int bb_num = MB->getNumber();
  std::cout << "Visit block #" << bb_num << "\n";
#endif
  num++;
  order[MB] = num;
  link[MB] = order[MB];
  stack.push_back(MB);
  for (MachineBasicBlock::const_succ_iterator S = MB->succ_begin(), SE = MB->succ_end();
       S != SE; S++) {
    MachineBasicBlock *nextMB = *S;
    if (order.find(nextMB) == order.end()) {
      tarjan(nextMB);
      link[MB] = std::min(link[MB], link[nextMB]);
      continue;
    }

    if (std::find(stack.begin(), stack.end(), nextMB) != stack.end()) {
      link[MB] = std::min(link[MB], order[nextMB]);
    }
  }

  if (link[MB] == order[MB]) {
    do {
      sort_stack.push_back(stack.back());
      stack.pop_back();
    } while (sort_stack.back() != MB);
  }
}

// VSA
void X86CacheAnalysis::VSA_init() {
  if (!global_value_list.empty())
    global_value_list.clear();

  if (!register_alocs.empty())
    register_alocs.clear();

  if (!global_alocs.empty())
    global_alocs.clear();

  if (!local_alocs.empty())
    local_alocs.clear();

  alocs_id = 0;
}

void X86CacheAnalysis::init_alocs(VSA_A_Locs *alocs, VSA_Memory_Region rgn)
{
  alocs->rgn = rgn;
  alocs->id = ++alocs_id;
  alocs->src = nullptr;
  alocs->src_id = 0;
  alocs->values.upper_bound = 0;
  alocs->values.lower_bound = 0;
  alocs->values.stride = 0;
  alocs->values.has_value = false;
  alocs->addresses.upper_bound = 0;
  alocs->addresses.lower_bound = 0;
  alocs->addresses.stride = 0;
  alocs->addresses.has_value = false;
}

void X86CacheAnalysis::set_alocs(VSA_A_Locs **alocs, const MachineInstr *MI, Mem_Type Ty, int index) {
  switch (Ty) {
  case MEM_REG: {
    unsigned reg = get_reg_alisas(MI->getOperand(index).getReg());
    if (register_alocs.find(reg) != register_alocs.end()) {
      *alocs = &register_alocs[reg];
      break;
    }
    *alocs = &register_alocs[reg];
    init_alocs(*alocs, RGN_NONE);
    break;
  }
  case MEM_MEM: {
    if (MI->getOperand(index + 3).isImm()) {
      unsigned base = get_reg_alisas(MI->getOperand(index).getReg());
      int disp = MI->getOperand(index + 3).getImm();
      if (base == X86::RBP) {
        if (local_alocs.find(disp) != local_alocs.end()) {
          *alocs = &local_alocs[disp];
          break;
        }
        *alocs = &local_alocs[disp];
        init_alocs(*alocs, RGN_LOCAL);
      } else {
        if (register_alocs.find(base) != register_alocs.find(base)) {
          // TODO: Deal with indirect address by registers other than RBP
        }
      }
    } else if (MI->getOperand(index + 3).isGlobal()) {
      std::string gv_name;
      SmallString<128> name;
      const GlobalValue *GV = MI->getOperand(index + 3).getGlobal();
      AP->getNameWithPrefix(name, GV);
      gv_name = StringRef(name).str();
      if (global_alocs.find(gv_name) != global_alocs.end()) {
        *alocs = &global_alocs[gv_name];
        break;
      }
      *alocs = &global_alocs[gv_name];
      init_alocs(*alocs, RGN_GLOBAL);
    }
    break;
  }
  case MEM_IMM: break;
  default: break;
  }
}

void X86CacheAnalysis::mov_alocs(ValueOrALocs *src, VSA_A_Locs *dst)
{
  if (dst == nullptr ||
      src == nullptr ||
      (src->isALocs && src->alocs == nullptr)) {
    return;
  }

  if (!src->isALocs) {
    dst->src_id = 0;
    dst->src = nullptr;
    dst->values.upper_bound = src->value;
    dst->values.lower_bound = src->value;
    dst->values.stride = 0;
    dst->values.has_value = true;
    dst->addresses.upper_bound = 0;
    dst->addresses.lower_bound = 0;
    dst->addresses.stride = 0;
    dst->addresses.has_value = false;
    return;
  }

  VSA_A_Locs *ptr;
  ptr = src->alocs;
  while (ptr->src != nullptr) {
    if (ptr->id == ptr->src->id) {
      break;
    }
    ptr = ptr->src;
  }
  dst->src = ptr;
  dst->src_id = ptr->id;
  dst->values.upper_bound = src->alocs->values.upper_bound;
  dst->values.lower_bound = src->alocs->values.lower_bound;
  dst->values.stride = src->alocs->values.stride;
  dst->values.has_value = src->alocs->values.has_value;
  dst->addresses.upper_bound = src->alocs->addresses.upper_bound;
  dst->addresses.lower_bound = src->alocs->addresses.lower_bound;
  dst->addresses.stride = src->alocs->addresses.stride;
  dst->addresses.has_value = src->alocs->addresses.has_value;
}

void X86CacheAnalysis::lea_alocs(ValueOrALocs *src, VSA_A_Locs *dst)
{
  if (dst == nullptr ||
      src == nullptr ||
      (src->isALocs && src->alocs == nullptr)) {
    return;
  }

  if (!src->isALocs)
    return;

  VSA_A_Locs *ptr;
  ptr = src->alocs;
  while (ptr->src != nullptr) {
    if (ptr->id == ptr->src->id) {
      break;
    }
    ptr = ptr->src;
  }
  dst->src = ptr;
  dst->src_id = ptr->id;
  dst->values.upper_bound = 0;
  dst->values.lower_bound = 0;
  dst->values.stride = 0;
  dst->values.has_value = false;
  dst->addresses.upper_bound = src->alocs->addresses.upper_bound;
  dst->addresses.lower_bound = src->alocs->addresses.lower_bound;
  dst->addresses.stride = src->alocs->addresses.stride;
  dst->addresses.has_value = true;
}

unsigned int gcd(unsigned int n1, unsigned int n2) {
  unsigned int tmp;
  while (n2 != 0) {
    tmp = n1;
    n1 = n2;
    n2 = tmp % n2;
  }
  return n1;
}

void X86CacheAnalysis::add_alocs(ValueOrALocs *src, VSA_A_Locs *dst)
{
  if (dst == nullptr ||
      src == nullptr ||
      (src->isALocs && src->alocs == nullptr)) {
    return;
  }

  if (!src->isALocs) {
    if (dst->values.has_value) {
      dst->values.upper_bound += src->value;
      dst->values.lower_bound += src->value;
      dst->values.stride = 0;
    }
    if (dst->addresses.has_value) {
      dst->addresses.upper_bound += src->value;
      dst->addresses.lower_bound += src->value;
      dst->addresses.stride = 0;
    }
    return;
  }

  if (src->alocs->values.has_value && dst->values.has_value) {
    dst->values.upper_bound += src->alocs->values.upper_bound;
    dst->values.lower_bound += src->alocs->values.lower_bound;
    dst->values.stride = gcd(dst->values.stride, src->alocs->values.stride);
  }

  if (src->alocs->values.has_value && dst->addresses.has_value) {
    dst->addresses.upper_bound += src->alocs->values.upper_bound;
    dst->addresses.lower_bound += src->alocs->values.lower_bound;
    dst->addresses.stride = gcd(dst->values.stride, src->alocs->addresses.stride);
  }
}

void X86CacheAnalysis::cmp_alocs(ValueOrALocs *src, VSA_A_Locs *dst)
{
  if (dst == nullptr ||
      src == nullptr ||
      (src->isALocs && src->alocs == nullptr)) {
    return;
  }

  if (!src->isALocs) {
    if (dst->values.has_value) {
      if (dst->values.upper_bound < src->value) {
        dst->values.upper_bound = src->value;
      }
      if (dst->values.lower_bound > src->value) {
        dst->values.lower_bound = src->value;
      }
    }
    if (dst->addresses.has_value) {
      if (dst->addresses.upper_bound < src->value) {
        dst->addresses.upper_bound = src->value;
      }
      if (dst->addresses.lower_bound > src->value) {
        dst->addresses.lower_bound += src->value;
      }
    }
    return;
  }

  if (src->alocs->values.has_value && dst->values.has_value) {
    if (dst->values.upper_bound < src->alocs->values.upper_bound) {
      dst->values.upper_bound = src->alocs->values.upper_bound;
    }
    if (dst->values.lower_bound > src->alocs->values.lower_bound) {
      dst->values.lower_bound = src->alocs->values.lower_bound;
    }
  }

  if (src->alocs->values.has_value && dst->addresses.has_value) {
    if (dst->addresses.upper_bound < src->alocs->values.upper_bound) {
      dst->addresses.upper_bound = src->alocs->values.upper_bound;
    }
    if (dst->addresses.lower_bound > src->alocs->values.lower_bound) {
      dst->addresses.lower_bound = src->alocs->values.lower_bound;
    }
  }
}

void X86CacheAnalysis::cache_write(int bb_num, int id, int offset) {
  int way = 0;;
  int set;

  if (offset < 0) {
    while (offset < 0) {
      offset += 0x1000;
      way--;
    }

    // Workaround
    way += 8;
    set = offset / 64;
  } else {
    way = offset / 4096;
    set = (offset % 4096) / 64;
  }

  if (cache_model[bb_num].find(id) == cache_model[bb_num].end()) {
    cache_model[bb_num][id].reserve(512);
    for (int i = 0; i < 512; i++) {
      cache_model[bb_num][id].push_back(false);
    }
  }
  cache_model[bb_num][id][way * 64 + set] = true;
}

void X86CacheAnalysis::classifyMemoryWrite(const MachineInstr *MI, int index) {
  if (!MI->getOperand(index).isReg() ||
      !MI->getOperand(index + 1).isImm() ||
      !MI->getOperand(index + 2).isReg()) {
    return;
  }

  int bb_num = MI->getParent()->getNumber();
  unsigned base = get_reg_alisas(MI->getOperand(index).getReg());
  int scale = MI->getOperand(index + 1).getImm();
  unsigned idx = get_reg_alisas(MI->getOperand(index + 2).getReg());

  if (MI->getOperand(index + 3).isImm()) {
    int disp = MI->getOperand(index + 3).getImm();
    if (base == X86::RBP) {
      int id = register_alocs[base].id;
      if (register_alocs.find(idx) == register_alocs.end()) {
        cache_write(bb_num, id, disp);
      } else {
        int lower_bound = scale * register_alocs[idx].values.lower_bound + disp;
        int upper_bound = scale * register_alocs[idx].values.upper_bound + disp;
        for (int i = lower_bound; i < upper_bound; i += 64) {
          cache_write(bb_num, id, i);
        }
        if (tsgx_debug) std::cout << "[Memory access] Stack [" <<  lower_bound << ", " << upper_bound << "]\n";
      }
    } else {
      if (register_alocs.find(base) != register_alocs.end()) {
        VSA_A_Locs *src;
        src = register_alocs[base].src;
        if (src != nullptr) {
          if (src->rgn == RGN_GLOBAL) {
            cache_write(bb_num, src->id, disp);
            if (tsgx_debug) std::cout << "[Memory Access] Global(" << src->id << ") " << disp << "\n";
          } else {
             if (register_alocs.find(idx) == register_alocs.end()) {
               if (tsgx_debug) std::cout << "[Memory access] Register(" << src->id << ") " << disp << "\n";
               cache_write(bb_num, src->id, disp);
             } else {
               int lower_bound = scale * register_alocs[idx].values.lower_bound + disp;
               int upper_bound = scale * register_alocs[idx].values.upper_bound + disp;
               if ((upper_bound - lower_bound) > 32768) {
                 for (int i = 0; i < 32768; i += 64) {
                   cache_write(bb_num, src->id, i);
                 }
               } else {
                 for (int i = lower_bound; i < upper_bound; i += 64) {
                   cache_write(bb_num, src->id, i);
                 }
               }
               if (tsgx_debug) std::cout << "[Memory access] Register(" << src->id << ") [" <<  lower_bound << ", " << upper_bound << "]\n";
             }
          }
        } else {
          if (tsgx_debug) std::cout << "[Memory Access] Unkown " << disp << "\n";
        }
      } else {
        if (tsgx_debug) std::cout << "[Memory Access] Unkown...\n";
      }
    }
  } else if (MI->getOperand(index + 3).isGlobal()) {

  }
}

int X86CacheAnalysis::getBBCacheWayUsage(int bb_num) {
  unsigned way_usage_upperbound = 0;
  for (std::map<int, std::vector<bool>>::iterator it = cache_model[bb_num].begin();
                                                  it != cache_model[bb_num].end();
                                                  it++) {
    std::vector<bool> cachelines = it->second;
    unsigned max_way_usage = 0;
    for (int i = 0; i < 64; i++) {
      unsigned way_usage_count = 0;
      for (int j = 0; j < 8; j++) {
        if (cachelines[j * 64 + i]) {
          way_usage_count++;
        }
      }
      if (max_way_usage < way_usage_count)
        max_way_usage = way_usage_count;
    }
    way_usage_upperbound += max_way_usage;
  }

  return way_usage_upperbound;
}

int X86CacheAnalysis::getJointBBCacheWayUsage(int bb_num1, int bb_num2) {
  // Aggregate cache usage
  std::map<int, std::vector<bool>> fun_cache;

  for (std::map<int, std::vector<bool>>::iterator bc = cache_model[bb_num1].begin();
                                                  bc != cache_model[bb_num1].end();
                                                  bc++) {
    int id = bc->first;
    std::vector<bool> cachelines = bc->second;
    int i;

    if (fun_cache.find(id) == fun_cache.end()) {
      fun_cache[id].reserve(512);
      for (i = 0; i < 512; i++) {
        fun_cache[id].push_back(false);
      }
    }
    for (i = 0; i < 512; i++) {
      if (cachelines[i])
        fun_cache[id][i] = true;
    }
  }

  for (std::map<int, std::vector<bool>>::iterator bc = cache_model[bb_num2].begin();
                                                  bc != cache_model[bb_num2].end();
                                                  bc++) {
    int id = bc->first;
    std::vector<bool> cachelines = bc->second;
    int i;

    if (fun_cache.find(id) == fun_cache.end()) {
      fun_cache[id].reserve(512);
      for (i = 0; i < 512; i++) {
        fun_cache[id].push_back(false);
      }
    }
    for (i = 0; i < 512; i++) {
      if (cachelines[i])
        fun_cache[id][i] = true;
    }
  }

  unsigned way_usage_upperbound = 0;
  for (std::map<int, std::vector<bool>>::iterator fc = fun_cache.begin();
                                                  fc != fun_cache.end();
                                                  fc++) {
    std::vector<bool> cachelines = fc->second;
    unsigned max_way_usage = 0;
    for (int i = 0; i < 64; i++) {
      unsigned way_usage_count = 0;
      for (int j = 0; j < 8; j++) {
        if (cachelines[j * 64 + i]) {
          way_usage_count++;
        }
      }
      if (max_way_usage < way_usage_count)
        max_way_usage = way_usage_count;
    }
    way_usage_upperbound += max_way_usage;
  }

  return way_usage_upperbound;
}

int X86CacheAnalysis::getFunCacheWayUsage() {
  // Aggregate cache usage
  std::map<int, std::vector<bool>> fun_cache;

  for (std::map<int, std::map<int, std::vector<bool>>>::iterator  it = cache_model.begin();
                                                                  it != cache_model.end();
                                                                  it++) {
    std::map<int, std::vector<bool>> bb_cache = it->second;
    for (std::map<int, std::vector<bool>>::iterator bc = bb_cache.begin();
                                                    bc != bb_cache.end();
                                                    bc++) {
      int id = bc->first;
      std::vector<bool> cachelines = bc->second;
      int i;

      if (fun_cache.find(id) == fun_cache.end()) {
        fun_cache[id].reserve(512);
        for (i = 0; i < 512; i++) {
          fun_cache[id].push_back(false);
        }
      }
      for (i = 0; i < 512; i++) {
        if (cachelines[i])
          fun_cache[id][i] = true;
      }
    }
  }

  unsigned way_usage_upperbound = 0;
  for (std::map<int, std::vector<bool>>::iterator fc = fun_cache.begin();
                                                  fc != fun_cache.end();
                                                  fc++) {
    std::vector<bool> cachelines = fc->second;
    unsigned max_way_usage = 0;
    for (int i = 0; i < 64; i++) {
      unsigned way_usage_count = 0;
      for (int j = 0; j < 8; j++) {
        if (cachelines[j * 64 + i]) {
          way_usage_count++;
        }
      }
      if (max_way_usage < way_usage_count)
        max_way_usage = way_usage_count;
    }
    way_usage_upperbound += max_way_usage;
  }

  return way_usage_upperbound;
}

void X86CacheAnalysis::simulateInstruction(const MachineInstr *MI,
                                           Instr_Operation OP,
                                           Instr_Operator_Type Ty) {
  ValueOrALocs src;
  VSA_A_Locs *dst = nullptr;

  src.isALocs = true;
  src.alocs = nullptr;
  src.value = 0;

  if (tsgx_debug) std::cout << "[simulateInstruction] OP Type: ";

  switch (Ty) {
    case Ty_I:
      break;
    case Ty_R:
      break;
    case Ty_M:
      break;
    case Ty_RR:
      if (tsgx_debug) std::cout << "RR\n";
      if (!MI->getOperand(0).isReg() || !MI->getOperand(1).isReg()) {
        break;
      }
      if (tsgx_debug) {
        debug_operators(MI, DBG_DST, MEM_REG, 0);
        debug_operators(MI, DBG_SRC, MEM_REG, 1);
      }
      set_alocs(&dst, MI, MEM_REG, 0);
      set_alocs(&src.alocs, MI, MEM_REG, 1);
      src.isALocs = true;
      break;
    case Ty_RI:
      if (tsgx_debug) std::cout << "RI\n";
      if (!MI->getOperand(0).isReg() || !MI->getOperand(1).isImm()) {
        break;
      }
      if (tsgx_debug) {
        debug_operators(MI, DBG_DST, MEM_REG, 0);
        debug_operators(MI, DBG_SRC, MEM_IMM, 1);
      }
      set_alocs(&dst, MI, MEM_REG, 0);
      src.value = MI->getOperand(1).getImm();
      src.isALocs = false;
      break;
    case Ty_RM:
      if (tsgx_debug) std::cout << "RM\n";
      if (MI->getNumOperands() < 5 ||
          !MI->getOperand(0).isReg() ||
          !MI->getOperand(1).isReg() ||
          !MI->getOperand(2).isImm() ||
          !MI->getOperand(3).isReg()) {
        break;
      }
      if (tsgx_debug) {
        debug_operators(MI, DBG_DST, MEM_REG, 0);
        debug_operators(MI, DBG_SRC, MEM_MEM, 1);
      }
      set_alocs(&dst, MI, MEM_REG, 0);
      set_alocs(&src.alocs, MI, MEM_MEM, 1);
      src.isALocs = true;
      break;
    case Ty_MI:
      if (tsgx_debug) std::cout << "MI\n";
      if (MI->getNumOperands() < 6 ||
          !MI->getOperand(0).isReg() ||
          !MI->getOperand(1).isImm() ||
          !MI->getOperand(2).isReg() ||
          !MI->getOperand(5).isImm()) {
        break;
      }
      if (tsgx_debug) {
        debug_operators(MI, DBG_DST, MEM_MEM, 0);
        debug_operators(MI, DBG_SRC, MEM_IMM, 5);
      }
      set_alocs(&dst, MI, MEM_MEM, 0);
      src.value = MI->getOperand(5).getImm();
      src.isALocs = false;

      if (OP != Op_CMP && OP != Op_TEST) {
        classifyMemoryWrite(MI, 0);
      }
      break;
    case Ty_MR:
      if (tsgx_debug) std::cout << "MR\n";
      if (MI->getNumOperands() < 6 ||
          !MI->getOperand(0).isReg() ||
          !MI->getOperand(1).isImm() ||
          !MI->getOperand(2).isReg() ||
          !MI->getOperand(5).isReg()) {
        break;
      }
      if (tsgx_debug) {
        debug_operators(MI, DBG_DST, MEM_MEM, 0);
        debug_operators(MI, DBG_SRC, MEM_REG, 5);
      }
      set_alocs(&dst, MI, MEM_MEM, 0);
      set_alocs(&src.alocs, MI, MEM_REG, 5);
      src.isALocs = true;

      if (OP != Op_CMP && OP != Op_TEST) {
        classifyMemoryWrite(MI, 0);
      }
      break;
    case Ty_RR_REV:
      if (tsgx_debug) std::cout << "RR_REV\n";
      if (!MI->getOperand(0).isReg() || !MI->getOperand(1).isReg()) {
        break;
      }
      if (tsgx_debug) {
        debug_operators(MI, DBG_DST, MEM_REG, 0);
        debug_operators(MI, DBG_SRC, MEM_REG, 1);
      }
      set_alocs(&dst, MI, MEM_REG, 0);
      set_alocs(&src.alocs, MI, MEM_REG, 1);
      src.isALocs = true;
      break;
    case Ty_RRI:
      break;
    case Ty_RMI:
      break;
    default:
      if (tsgx_debug) std::cout << "Unsupported:" << Ty << " ";
      break;
  }

  if (dst == nullptr)
    return;

  if (tsgx_debug) std::cout << "Operation: ";
  switch (OP) {
  case Op_MOV:
    if (tsgx_debug) std::cout << "MOV";
    mov_alocs(&src, dst);
    break;
  case Op_LEA:
    if (tsgx_debug) std::cout << "LEA";
    lea_alocs(&src, dst);
    break;
  case Op_AND:
    if (tsgx_debug) std::cout << "AND";
    break;
  case Op_OR:
    if (tsgx_debug) std::cout << "OR";
    break;
  case Op_XOR:
    if (tsgx_debug) std::cout << "XOR";
    break;
  case Op_ADD:
    if (tsgx_debug) std::cout << "ADD";
    break;
  case Op_SUB:
    if (tsgx_debug) std::cout << "SUB";
    break;
  case Op_ADC:
    if (tsgx_debug) std::cout << "ADC";
    break;
  case Op_SBB:
    if (tsgx_debug) std::cout << "SBB";
    break;
  case Op_CMP:
    if (tsgx_debug) std::cout << "CMP";
    cmp_alocs(&src, dst);
    break;
  default:
    if (tsgx_debug) std::cout << "Unsupported:" << OP;
    break;
  }
  if (tsgx_debug) std::cout << "\n";
}

void X86CacheAnalysis::processInstruction(const MachineInstr *MI) {
  unsigned op_code = MI->getOpcode();
  const MachineBasicBlock *MB = MI->getParent();
  MachineBasicBlock::const_iterator MBBI(MI);
  int inst_num = std::distance(MB->begin(), MBBI);

  if (tsgx_debug) {
    std::cout << "[processInstruction] " << inst_num << ": ";
    debug_all_operators(MI);
  }

  switch (op_code) {
  // Move Instructions.
  case X86::MOV8rr:
  case X86::MOV16rr:
  case X86::MOV32rr:
  case X86::MOV64rr:
    simulateInstruction(MI, Op_MOV, Ty_RR);
    break;
  case X86::MOV8ri:
  case X86::MOV16ri:
  case X86::MOV32ri:
  case X86::MOV64ri32:
    simulateInstruction(MI, Op_MOV, Ty_RI);
    break;
  case X86::MOV8mi:
  case X86::MOV16mi:
  case X86::MOV32mi:
  case X86::MOV64mi32:
    simulateInstruction(MI, Op_MOV, Ty_MI);
    break;
  case X86::MOV8ao32:
  case X86::MOV16ao32:
  case X86::MOV32ao32:
  case X86::MOV64ao32:
  case X86::MOV8ao16:
  case X86::MOV16ao16:
  case X86::MOV32ao16:
    simulateInstruction(MI, Op_MOV, Ty_RM);
    break;
  case X86::MOV8o32a:
  case X86::MOV16o32a:
  case X86::MOV32o32a:
  case X86::MOV64o32a:
  case X86::MOV8o16a:
  case X86::MOV16o16a:
  case X86::MOV32o16a:
    simulateInstruction(MI, Op_MOV, Ty_MR);
    break;
  case X86::MOV8ao64:
  case X86::MOV16ao64:
  case X86::MOV32ao64:
  case X86::MOV64ao64:
    simulateInstruction(MI, Op_MOV, Ty_RM);
    break;
  case X86::MOV8o64a:
  case X86::MOV16o64a:
  case X86::MOV32o64a:
  case X86::MOV64o64a:
    simulateInstruction(MI, Op_MOV, Ty_MR);
    break;
  case X86::MOV8rr_REV:
  case X86::MOV16rr_REV:
  case X86::MOV32rr_REV:
  case X86::MOV64rr_REV:
    simulateInstruction(MI, Op_MOV, Ty_RR_REV);
    break;
  case X86::MOV8rm:
  case X86::MOV16rm:
  case X86::MOV32rm:
  case X86::MOV64rm:
    simulateInstruction(MI, Op_MOV, Ty_RM);
    break;
  case X86::MOV8mr:
  case X86::MOV16mr:
  case X86::MOV32mr:
  case X86::MOV64mr:
    simulateInstruction(MI, Op_MOV, Ty_MR);
    break;
  case X86::MOVSX16rr8:
  case X86::MOVSX32rr16:
  case X86::MOVSX32rr8:
  case X86::MOVSX64rr16:
  case X86::MOVSX64rr32:
  case X86::MOVSX64rr8:
    simulateInstruction(MI, Op_MOV, Ty_RR);
    break;
  case X86::MOVSX16rm8:
  case X86::MOVSX32rm16:
  case X86::MOVSX32rm8:
  case X86::MOVSX64rm16:
  case X86::MOVSX64rm32:
  case X86::MOVSX64rm8:
    simulateInstruction(MI, Op_MOV, Ty_RM);
    break;
  case X86::MOVSDrm:
  case X86::MOVSSrm:
    simulateInstruction(MI, Op_MOV, Ty_RM);
    break;
  case X86::MOVSDmr:
  case X86::MOVSSmr:
    simulateInstruction(MI, Op_MOV, Ty_MR);
    break;
  // Arithmetic Instructions.
  case X86::LEA16r:
  case X86::LEA32r:
  case X86::LEA64_32r:
  case X86::LEA64r:
    simulateInstruction(MI, Op_LEA, Ty_RM);
    break;
  case X86::MUL8r:
  case X86::MUL16r:
  case X86::MUL32r:
  case X86::MUL64r:
    break;
  case X86::MUL8m:
  case X86::MUL16m:
  case X86::MUL32m:
  case X86::MUL64m:
    break;
  case X86::IMUL8m:
  case X86::IMUL16m:
  case X86::IMUL32m:
  case X86::IMUL64m:
    break;
  case X86::IMUL16rri:
  case X86::IMUL16rri8:
  case X86::IMUL32rri:
  case X86::IMUL32rri8:
  case X86::IMUL64rri32:
  case X86::IMUL64rri8:
    break;
  case X86::IMUL16rmi:
  case X86::IMUL16rmi8:
  case X86::IMUL32rmi:
  case X86::IMUL32rmi8:
  case X86::IMUL64rmi32:
  case X86::IMUL64rmi8:
    break;
  case X86::DIV8r:
  case X86::DIV16r:
  case X86::DIV32r:
  case X86::DIV64r:
    break;
  case X86::DIV8m:
  case X86::DIV16m:
  case X86::DIV32m:
  case X86::DIV64m:
    break;
  case X86::IDIV8r:
  case X86::IDIV16r:
  case X86::IDIV32r:
  case X86::IDIV64r:
    break;
  case X86::IDIV8m:
  case X86::IDIV16m:
  case X86::IDIV32m:
  case X86::IDIV64m:
    break;
  // Two address Instructions.
  case X86::NEG8r:
  case X86::NEG16r:
  case X86::NEG32r:
  case X86::NEG64r:
    break;
  case X86::NEG8m:
  case X86::NEG16m:
  case X86::NEG32m:
  case X86::NEG64m:
    break;
  case X86::NOT8r:
  case X86::NOT16r:
  case X86::NOT32r:
  case X86::NOT64r:
    break;
  case X86::NOT8m:
  case X86::NOT16m:
  case X86::NOT32m:
  case X86::NOT64m:
    break;
  case X86::INC8r:
  case X86::INC16r:
  case X86::INC32r:
  case X86::INC64r:
    break;
  case X86::INC8m:
  case X86::INC16m:
  case X86::INC32m:
  case X86::INC64m:
    break;
  case X86::DEC8r:
  case X86::DEC16r:
  case X86::DEC32r:
  case X86::DEC64r:
    break;
  case X86::DEC8m:
  case X86::DEC16m:
  case X86::DEC32m:
  case X86::DEC64m:
    break;
  // Three address Instructions.
  case X86::AND8rr:
  case X86::AND16rr:
  case X86::AND32rr:
  case X86::AND64rr:
    break;
  case X86::AND8rr_REV:
  case X86::AND16rr_REV:
  case X86::AND32rr_REV:
  case X86::AND64rr_REV:
    break;
  case X86::AND8rm:
  case X86::AND16rm:
  case X86::AND32rm:
  case X86::AND64rm:
    break;
  case X86::AND8ri:
  case X86::AND16ri8:
  case X86::AND32ri8:
  case X86::AND64ri8:
  case X86::AND16ri:
  case X86::AND32ri:
  case X86::AND64ri32:
    break;
  case X86::AND8mr:
  case X86::AND16mr:
  case X86::AND32mr:
  case X86::AND64mr:
    simulateInstruction(MI, Op_AND, Ty_MR);
    break;
  case X86::AND16mi8:
  case X86::AND32mi8:
  case X86::AND64mi8:
  case X86::AND8mi:
  case X86::AND16mi:
  case X86::AND32mi:
  case X86::AND64mi32:
    simulateInstruction(MI, Op_AND, Ty_MI);
    break;
  case X86::AND8i8:
  case X86::AND16i16:
  case X86::AND32i32:
  case X86::AND64i32:
    break;
  case X86::OR8rr:
  case X86::OR16rr:
  case X86::OR32rr:
  case X86::OR64rr:
    break;
  case X86::OR8rr_REV:
  case X86::OR16rr_REV:
  case X86::OR32rr_REV:
  case X86::OR64rr_REV:
    break;
  case X86::OR8rm:
  case X86::OR16rm:
  case X86::OR32rm:
  case X86::OR64rm:
    break;
  case X86::OR8ri:
  case X86::OR16ri8:
  case X86::OR32ri8:
  case X86::OR64ri8:
  case X86::OR16ri:
  case X86::OR32ri:
  case X86::OR64ri32:
    break;
  case X86::OR8mr:
  case X86::OR16mr:
  case X86::OR32mr:
  case X86::OR64mr:
    simulateInstruction(MI, Op_OR, Ty_MR);
    break;
  case X86::OR16mi8:
  case X86::OR32mi8:
  case X86::OR64mi8:
  case X86::OR8mi:
  case X86::OR16mi:
  case X86::OR32mi:
  case X86::OR64mi32:
    simulateInstruction(MI, Op_OR, Ty_MI);
    break;
  case X86::OR8i8:
  case X86::OR16i16:
  case X86::OR32i32:
  case X86::OR64i32:
    break;
  case X86::XOR8rr:
  case X86::XOR16rr:
  case X86::XOR32rr:
  case X86::XOR64rr:
    break;
  case X86::XOR8rr_REV:
  case X86::XOR16rr_REV:
  case X86::XOR32rr_REV:
  case X86::XOR64rr_REV:
    break;
  case X86::XOR8rm:
  case X86::XOR16rm:
  case X86::XOR32rm:
  case X86::XOR64rm:
    break;
  case X86::XOR8ri:
  case X86::XOR16ri8:
  case X86::XOR32ri8:
  case X86::XOR64ri8:
  case X86::XOR16ri:
  case X86::XOR32ri:
  case X86::XOR64ri32:
    break;
  case X86::XOR8mr:
  case X86::XOR16mr:
  case X86::XOR32mr:
  case X86::XOR64mr:
    simulateInstruction(MI, Op_XOR, Ty_MR);
    break;
  case X86::XOR16mi8:
  case X86::XOR32mi8:
  case X86::XOR64mi8:
  case X86::XOR8mi:
  case X86::XOR16mi:
  case X86::XOR32mi:
  case X86::XOR64mi32:
    simulateInstruction(MI, Op_XOR, Ty_MI);
    break;
  case X86::XOR8i8:
  case X86::XOR16i16:
  case X86::XOR32i32:
  case X86::XOR64i32:
    break;
  case X86::ADD8rr:
  case X86::ADD16rr:
  case X86::ADD32rr:
  case X86::ADD64rr:
    break;
  case X86::ADD8rr_REV:
  case X86::ADD16rr_REV:
  case X86::ADD32rr_REV:
  case X86::ADD64rr_REV:
    break;
  case X86::ADD8rm:
  case X86::ADD16rm:
  case X86::ADD32rm:
  case X86::ADD64rm:
    break;
  case X86::ADD8ri:
  case X86::ADD16ri8:
  case X86::ADD32ri8:
  case X86::ADD64ri8:
  case X86::ADD16ri:
  case X86::ADD32ri:
  case X86::ADD64ri32:
    break;
  case X86::ADD8mr:
  case X86::ADD16mr:
  case X86::ADD32mr:
  case X86::ADD64mr:
    simulateInstruction(MI, Op_ADD, Ty_MR);
    break;
  case X86::ADD16mi8:
  case X86::ADD32mi8:
  case X86::ADD64mi8:
  case X86::ADD8mi:
  case X86::ADD16mi:
  case X86::ADD32mi:
  case X86::ADD64mi32:
    simulateInstruction(MI, Op_ADD, Ty_MI);
    break;
  case X86::ADD8i8:
  case X86::ADD16i16:
  case X86::ADD32i32:
  case X86::ADD64i32:
    break;
  case X86::SUB8rr:
  case X86::SUB16rr:
  case X86::SUB32rr:
  case X86::SUB64rr:
    break;
  case X86::SUB8rr_REV:
  case X86::SUB16rr_REV:
  case X86::SUB32rr_REV:
  case X86::SUB64rr_REV:
    break;
  case X86::SUB8rm:
  case X86::SUB16rm:
  case X86::SUB32rm:
  case X86::SUB64rm:
    break;
  case X86::SUB8ri:
  case X86::SUB16ri8:
  case X86::SUB32ri8:
  case X86::SUB64ri8:
  case X86::SUB16ri:
  case X86::SUB32ri:
  case X86::SUB64ri32:
    break;
  case X86::SUB8mr:
  case X86::SUB16mr:
  case X86::SUB32mr:
  case X86::SUB64mr:
    simulateInstruction(MI, Op_SUB, Ty_MR);
    break;
  case X86::SUB16mi8:
  case X86::SUB32mi8:
  case X86::SUB64mi8:
  case X86::SUB8mi:
  case X86::SUB16mi:
  case X86::SUB32mi:
  case X86::SUB64mi32:
    simulateInstruction(MI, Op_SUB, Ty_MI);
    break;
  case X86::SUB8i8:
  case X86::SUB16i16:
  case X86::SUB32i32:
  case X86::SUB64i32:
    break;
  case X86::ADC8rr:
  case X86::ADC16rr:
  case X86::ADC32rr:
  case X86::ADC64rr:
    break;
  case X86::ADC8rr_REV:
  case X86::ADC16rr_REV:
  case X86::ADC32rr_REV:
  case X86::ADC64rr_REV:
    break;
  case X86::ADC8rm:
  case X86::ADC16rm:
  case X86::ADC32rm:
  case X86::ADC64rm:
    break;
  case X86::ADC8ri:
  case X86::ADC16ri8:
  case X86::ADC32ri8:
  case X86::ADC64ri8:
  case X86::ADC16ri:
  case X86::ADC32ri:
  case X86::ADC64ri32:
    break;
  case X86::ADC8mr:
  case X86::ADC16mr:
  case X86::ADC32mr:
  case X86::ADC64mr:
    simulateInstruction(MI, Op_ADC, Ty_MR);
    break;
  case X86::ADC16mi8:
  case X86::ADC32mi8:
  case X86::ADC64mi8:
  case X86::ADC8mi:
  case X86::ADC16mi:
  case X86::ADC32mi:
  case X86::ADC64mi32:
    simulateInstruction(MI, Op_ADC, Ty_MI);
    break;
  case X86::ADC8i8:
  case X86::ADC16i16:
  case X86::ADC32i32:
  case X86::ADC64i32:
    break;
  case X86::SBB8rr:
  case X86::SBB16rr:
  case X86::SBB32rr:
  case X86::SBB64rr:
    break;
  case X86::SBB8rr_REV:
  case X86::SBB16rr_REV:
  case X86::SBB32rr_REV:
  case X86::SBB64rr_REV:
    break;
  case X86::SBB8rm:
  case X86::SBB16rm:
  case X86::SBB32rm:
  case X86::SBB64rm:
    break;
  case X86::SBB8ri:
  case X86::SBB16ri8:
  case X86::SBB32ri8:
  case X86::SBB64ri8:
  case X86::SBB16ri:
  case X86::SBB32ri:
  case X86::SBB64ri32:
    break;
  case X86::SBB8mr:
  case X86::SBB16mr:
  case X86::SBB32mr:
  case X86::SBB64mr:
    simulateInstruction(MI, Op_SBB, Ty_MR);
    break;
  case X86::SBB16mi8:
  case X86::SBB32mi8:
  case X86::SBB64mi8:
  case X86::SBB8mi:
  case X86::SBB16mi:
  case X86::SBB32mi:
  case X86::SBB64mi32:
    simulateInstruction(MI, Op_SBB, Ty_MI);
    break;
  case X86::SBB8i8:
  case X86::SBB16i16:
  case X86::SBB32i32:
  case X86::SBB64i32:
    break;
  case X86::CMP8rr:
  case X86::CMP16rr:
  case X86::CMP32rr:
  case X86::CMP64rr:
    simulateInstruction(MI, Op_CMP, Ty_RR);
    break;
  case X86::CMP8rr_REV:
  case X86::CMP16rr_REV:
  case X86::CMP32rr_REV:
  case X86::CMP64rr_REV:
    simulateInstruction(MI, Op_CMP, Ty_RR);
    break;
  case X86::CMP8rm:
  case X86::CMP16rm:
  case X86::CMP32rm:
  case X86::CMP64rm:
    simulateInstruction(MI, Op_CMP, Ty_RM);
    break;
  case X86::CMP8ri:
  case X86::CMP16ri8:
  case X86::CMP32ri8:
  case X86::CMP64ri8:
  case X86::CMP16ri:
  case X86::CMP32ri:
  case X86::CMP64ri32:
    simulateInstruction(MI, Op_CMP, Ty_RI);
    break;
  case X86::CMP8mr:
  case X86::CMP16mr:
  case X86::CMP32mr:
  case X86::CMP64mr:
    simulateInstruction(MI, Op_CMP, Ty_MR);
    break;
  case X86::CMP16mi8:
  case X86::CMP32mi8:
  case X86::CMP64mi8:
  case X86::CMP8mi:
  case X86::CMP16mi:
  case X86::CMP32mi:
  case X86::CMP64mi32:
    simulateInstruction(MI, Op_CMP, Ty_MI);
    break;
  case X86::CMP8i8:
  case X86::CMP16i16:
  case X86::CMP32i32:
  case X86::CMP64i32:
    simulateInstruction(MI, Op_CMP, Ty_RI);
    break;
  // TEST instructions.
  case X86::TEST8rr:
  case X86::TEST16rr:
  case X86::TEST32rr:
  case X86::TEST64rr:
    break;
  case X86::TEST8rm:
  case X86::TEST16rm:
  case X86::TEST32rm:
  case X86::TEST64rm:
    break;
  case X86::TEST8ri:
  case X86::TEST16ri:
  case X86::TEST32ri:
  case X86::TEST64ri32:
    break;
  case X86::TEST8mi:
  case X86::TEST16mi:
  case X86::TEST32mi:
  case X86::TEST64mi32:
    break;
  case X86::TEST8i8:
  case X86::TEST16i16:
  case X86::TEST32i32:
  case X86::TEST64i32:
    break;
  // POP/PUSH Instructions.
  case X86::POP16r:
  case X86::POP32r:
  case X86::POP16rmr:
  case X86::POP16rmm:
  case X86::POP32rmr:
  case X86::POP32rmm:
    break;
  case X86::PUSH16r:
  case X86::PUSH32r:
  case X86::PUSH16rmr:
  case X86::PUSH16rmm:
  case X86::PUSH32rmr:
  case X86::PUSH32rmm:
  case X86::PUSH16i8:
  case X86::PUSHi16:
  case X86::PUSH32i8:
  case X86::PUSHi32:
    break;
  case X86::POP64r:
  case X86::POP64rmr:
  case X86::POP64rmm:
    break;
  case X86::PUSH64r:
  case X86::PUSH64rmr:
  case X86::PUSH64rmm:
  case X86::PUSH64i8:
  case X86::PUSH64i32:
    break;
  // TODO: ANDN, MULX, ADCX, ADOX, etc,.
  default:
    if (tsgx_debug) std::cout << "unsupported instruction: " << op_code << "\n";
    break;
  }
}

void X86CacheAnalysis::processBaicBlock(const MachineBasicBlock *MB) {
  if (tsgx_debug) std::cout << "[processBaicBlock] block num: " << MB->getNumber() << "\n";

  for (MachineBasicBlock::const_iterator I = MB->begin(), IE = MB->end(); I != IE; ++I) {
    processInstruction(I);
  }
}

// RAX access pattern analysis aims to identifies
// whether the accesses of RAX cross successive
// basic blocks. Because of TSX overwrites RAX on aborts,
// we need to keep the RAX value when needed.
void X86CacheAnalysis::doRAXPatternAnalysis(const MachineBasicBlock *MB) {
  unsigned bb_num = MB->getNumber();
  int index;
  int min_src_index;
  int min_dst_index;
  bool hasCall = false;
  MachineBasicBlock::const_iterator I;

  index = 0;
  min_src_index = -1;
  min_dst_index = -1;
  for (I = MB->begin(); I != MB->end(); I++) {

    if (I->isCall()) {
      hasCall = true;
      continue;
    }

    if (I->readsRegister(X86::RAX, nullptr) ||
        I->readsRegister(X86::EAX, nullptr) ||
        I->readsRegister(X86::AX, nullptr) ||
        I->readsRegister(X86::AL, nullptr) ||
        I->readsRegister(X86::AH, nullptr)) {
#if 0 // Enable if needed for debugging.
      std::cout << index << "(read): ";
      debug_all_operators(I);
#endif
      if (min_src_index == -1)
        min_src_index = index;
    }

    if (I->modifiesRegister(X86::RAX, nullptr) ||
        I->modifiesRegister(X86::EAX, nullptr) ||
        I->modifiesRegister(X86::AX, nullptr) ||
        I->modifiesRegister(X86::AL, nullptr) ||
        I->modifiesRegister(X86::AH, nullptr)) {
#if 0 // Enable if needed for debugging.
      std::cout << index << "(write): ";
      debug_all_operators(I);
#endif
      if (min_dst_index == -1)
        min_dst_index = index;
    }
    index++;
  }

  rax_analysis_result[bb_num].bb_pattern.is_dst = false;
  rax_analysis_result[bb_num].bb_pattern.is_src = false;

  // Case 1: RAX is used as destination
  if (min_dst_index != -1) {
    rax_analysis_result[bb_num].bb_pattern.is_dst = true;
  }

  // Case 2: RAX is used as source prior to used as destination
  //         or used as the source only.
  if ((min_src_index != -1 && min_dst_index != -1 && (min_src_index <= min_dst_index)) ||
      (min_src_index != -1 && min_dst_index == -1)) {
    rax_analysis_result[bb_num].bb_pattern.is_src = true;
  }

  if (!hasCall) {
    return;
  }

  int call_index;

  call_index = -1;
  I = MB->begin();
  while (I != MB->end()) {
    if (I->isCall()) {
      if (call_index != -1) {
        rax_analysis_result[bb_num].call_pattern[call_index].is_dst = false;
        rax_analysis_result[bb_num].call_pattern[call_index].is_src = false;

        if (min_dst_index != -1) {
          rax_analysis_result[bb_num].call_pattern[call_index].is_dst = true;
        }

        if ((min_src_index != -1 && min_dst_index != -1 && (min_src_index <= min_dst_index)) ||
            (min_src_index != -1 && min_dst_index == -1)) {
          rax_analysis_result[bb_num].call_pattern[call_index].is_src = true;
        }
      }

      call_index = std::distance(MB->begin(), I);
      index = 0;
      min_src_index = -1;
      min_dst_index = -1;
      I++;
      continue;
    }

    if (call_index == -1) {
      index++;
      I++;
      continue;
    }

    if (I->readsRegister(X86::RAX, nullptr) ||
        I->readsRegister(X86::EAX, nullptr) ||
        I->readsRegister(X86::AX, nullptr) ||
        I->readsRegister(X86::AL, nullptr) ||
        I->readsRegister(X86::AH, nullptr)) {
#if 0 // Enable if needed for debugging.
      std::cout << index << "(read): ";
      debug_all_operators(I);
#endif
      if (min_src_index == -1)
        min_src_index = index;
    }

    if (I->modifiesRegister(X86::RAX, nullptr) ||
        I->modifiesRegister(X86::EAX, nullptr) ||
        I->modifiesRegister(X86::AX, nullptr) ||
        I->modifiesRegister(X86::AL, nullptr) ||
        I->modifiesRegister(X86::AH, nullptr)) {
#if 0 // Enable if needed for debugging.
      std::cout << index << "(write): ";
      debug_all_operators(I);
#endif
      if (min_dst_index == -1)
        min_dst_index = index;
    }
    index++;
    I++;
  }

  rax_analysis_result[bb_num].call_pattern[call_index].is_dst = false;
  rax_analysis_result[bb_num].call_pattern[call_index].is_src = false;

  if (min_dst_index != -1) {
    rax_analysis_result[bb_num].call_pattern[call_index].is_dst = true;
  }

  if ((min_src_index != -1 && min_dst_index != -1 && (min_src_index <= min_dst_index)) ||
      (min_src_index != -1 && min_dst_index == -1)) {
    rax_analysis_result[bb_num].call_pattern[call_index].is_src = true;
  }
}

bool X86CacheAnalysis::isRAXSrc(unsigned bb_num) {
  if (rax_analysis_result.find(bb_num) == rax_analysis_result.end()) {
    return false;
  }
  return rax_analysis_result[bb_num].bb_pattern.is_src;
}

bool X86CacheAnalysis::isRAXDst(unsigned bb_num) {
  if (rax_analysis_result.find(bb_num) == rax_analysis_result.end()) {
    return false;
  }
  return rax_analysis_result[bb_num].bb_pattern.is_dst;
}

bool X86CacheAnalysis::isRAXSrcAfterCall(unsigned bb_num, int index) {
  if (rax_analysis_result.find(bb_num) == rax_analysis_result.end()) {
    return false;
  }
  if (rax_analysis_result[bb_num].call_pattern.find(index) == rax_analysis_result[bb_num].call_pattern.end()) {
    return false;
  }
  return rax_analysis_result[bb_num].call_pattern[index].is_src;
}

bool X86CacheAnalysis::isRAXDstAfterCall(unsigned bb_num, int index) {
  if (rax_analysis_result.find(bb_num) == rax_analysis_result.end()) {
    return false;
  }
  if (rax_analysis_result[bb_num].call_pattern.find(index) == rax_analysis_result[bb_num].call_pattern.end()) {
    return false;
  }
  return rax_analysis_result[bb_num].call_pattern[index].is_dst;
}

void X86CacheAnalysis::doAnalysis() {
  // tarjan initialization.
  tarjan_init();

  // Perform tarjan algorithm.
  tarjan(&(MF->front()));

  // VSA initialization.
  VSA_init();

  // Traverse CFG in topological order.
  int track_link = link[sort_stack.back()];
  for (std::vector<const MachineBasicBlock*>::reverse_iterator it = sort_stack.rbegin();
       it < sort_stack.rend(); it++) {
    const MachineBasicBlock *MB = *it;

    int current_link = link[MB];
    if (track_link != current_link) {
      track_link = current_link;
    }

    // Process basic block for cache analysis.
    processBaicBlock(MB);

    if (tsgx_debug) debug_cache_access(MB->getNumber());

    // Do rax pattern analysis.
    doRAXPatternAnalysis(MB);
  }

#if 0  // Enable if needed for debugging.
  for (std::map<unsigned, VSA_A_Locs>::iterator it = register_alocs.begin();
                                                it != register_alocs.end(); ++it) {
    debug_alocs(&it->second, &it->first);
  }

  for (std::map<int, VSA_A_Locs>::iterator it = local_alocs.begin();
                                           it != local_alocs.end(); ++it) {
    debug_alocs(&it->second, &it->first);
  }

  for (std::map<std::string, VSA_A_Locs>::iterator it = global_alocs.begin();
                                           it != global_alocs.end(); ++it) {
    debug_alocs(&it->second, &it->first);
  }
  debug_fun_cache_access();
#endif

#if 0  // Enable if needed for debugging.
track_link = link[sort_stack.back()];
  int scc_count = 0;
  std::cout << "SCC #" << scc_count << ": ";

  for (std::vector<const MachineBasicBlock*>::reverse_iterator it = sort_stack.rbegin();
       it < sort_stack.rend(); it++) {
    const MachineBasicBlock *MB = *it;

    int current_link = link[MB];
    if (track_link != current_link) {
      track_link = current_link;
      scc_count++;
      std::cout << "\n";
      std::cout << "SCC #" << scc_count << ": ";
    }
    std::cout << MB->getNumber() << " ";
  }
  std::cout << "\n";
#endif

}

// Debugging functions.

void X86CacheAnalysis::debug_reg_alisas(unsigned reg) {
  switch (reg) {
  case X86::AL:
  case X86::AX:
  case X86::EAX:
  case X86::RAX:
    std::cout << "RAX";
    break;
  case X86::BL:
  case X86::BX:
  case X86::EBX:
  case X86::RBX:
    std::cout << "RBX";
    break;
  case X86::CL:
  case X86::CX:
  case X86::ECX:
  case X86::RCX:
    std::cout << "RCX";
    break;
  case X86::DL:
  case X86::DX:
  case X86::EDX:
  case X86::RDX:
    std::cout << "RDX";
    break;
  case X86::SIL:
  case X86::SI:
  case X86::ESI:
  case X86::RSI:
    std::cout << "RSI";
    break;
  case X86::DIL:
  case X86::DI:
  case X86::EDI:
  case X86::RDI:
    std::cout << "RDI";
    break;
  case X86::R8B:
  case X86::R8W:
  case X86::R8D:
  case X86::R8:
    std::cout << "R8";
    break;
  case X86::R9B:
  case X86::R9W:
  case X86::R9D:
  case X86::R9:
    std::cout << "R9";
    break;
  case X86::R10B:
  case X86::R10W:
  case X86::R10D:
  case X86::R10:
    std::cout << "R10";
    break;
  case X86::R11B:
  case X86::R11W:
  case X86::R11D:
  case X86::R11:
    std::cout << "R11";
    break;
  case X86::R12B:
  case X86::R12W:
  case X86::R12D:
  case X86::R12:
    std::cout << "R13";
    break;
  case X86::R13B:
  case X86::R13W:
  case X86::R13D:
  case X86::R13:
    std::cout << "R13";
    break;
  case X86::RBP:
  case X86::EBP:
    std::cout << "RBP";
    break;
  case X86::RSP:
  case X86::ESP:
    std::cout << "RSP";
    break;
  default:
    std::cout << "UNKOWN";
  }
}

void X86CacheAnalysis::debug_all_operators(const MachineInstr *MI) {
  for (const MachineOperand &MO : MI->operands()) {
    switch (MO.getType()) {
    case MachineOperand::MO_Register:
      //std::cout << "REG[" << MO.getReg() << "] ";
      std::cout << "REG[";
      debug_reg_alisas(MO.getReg());
      std::cout << "] ";
      break;
    case MachineOperand::MO_Immediate:
      std::cout << "IMM[" << MO.getImm() << "] ";
      break;
    case MachineOperand::MO_GlobalAddress:
      std::cout << "GV ";
      break;
    default:
      std::cout << "Other type: " << MO.getType() << " ";
    }
  }
  std::cout << "\n";
}

void X86CacheAnalysis::debug_operators(const MachineInstr *MI, Debug_Target_Type target, Mem_Type Ty, int index) {
  switch (target) {
  case DBG_SRC: std::cout << "SRC: "; break;
  case DBG_DST: std::cout << "DST: "; break;
  default: break;
  }

  switch (Ty) {
  case MEM_REG:
    std::cout << "REG[";
    debug_reg_alisas(MI->getOperand(index).getReg());
    std::cout << "]\n";
    break;
  case MEM_IMM:
    std::cout << "IMM[" << MI->getOperand(index).getImm() << "]\n";
    break;
  case MEM_MEM:
    if (MI->getOperand(index + 3).isImm()) {
      std::cout << "MEM REG[";
      debug_reg_alisas(MI->getOperand(index).getReg());
      std::cout << "] + " << MI->getOperand(index + 1).getImm() << " * REG[";
      debug_reg_alisas(MI->getOperand(index + 2).getReg());
      std::cout << "] + " << MI->getOperand(index + 3).getImm() << "\n";
    }
    else if (MI->getOperand(index + 3).isGlobal()) {
      std::cout << "MEM REG[";
      debug_reg_alisas(MI->getOperand(index).getReg());
      std::cout << "] + " << MI->getOperand(index + 1).getImm() << " * REG[";
      debug_reg_alisas(MI->getOperand(index + 2).getReg());
      std::cout << "] + GlobalValue\n";
    }
    break;
  default:
    break;
  }
}

void X86CacheAnalysis::debug_alocs(VSA_A_Locs *alocs, const void *index) {
  std::cout << "A-Locs(";
  switch (alocs->rgn) {
  case RGN_NONE: {
    std::cout << "Register, ";
    debug_reg_alisas(*(int *)index);
    std::cout << ") ";
    std::cout << alocs->values.stride << "[" << alocs->values.lower_bound
              << ", " << alocs->values.upper_bound << "], "
              << alocs->addresses.stride << "[" << alocs->addresses.lower_bound
              << ", " << alocs->addresses.upper_bound << "]\n";
    break;
  }
  case RGN_LOCAL: {
    std::cout << "Stack," << *(int *)index;
    std::cout << ") ";
    std::cout << alocs->values.stride << "[" << alocs->values.lower_bound
              << ", " << alocs->values.upper_bound << "], "
              << alocs->addresses.stride << "[" << alocs->addresses.lower_bound
              << ", " << alocs->addresses.upper_bound << "]\n";
    break;
  }
  case RGN_HEAP:
    std::cout << "Heap";
    break;
  case RGN_GLOBAL: {
    std::cout << "Global: " << *(std::string *)index;
    std::cout << ") ";
    std::cout << alocs->values.stride << "[" << alocs->values.lower_bound
              << ", " << alocs->values.upper_bound << "], "
              << alocs->addresses.stride << "[" << alocs->addresses.lower_bound
              << ", " << alocs->addresses.upper_bound << "]\n";
    break;
  }
  default:
    break;
  }
}

void X86CacheAnalysis::debug_cache_access(int bb_num) {
  unsigned way_usage_upperbound = 0;
  for (std::map<int, std::vector<bool>>::iterator it = cache_model[bb_num].begin();
                                                  it != cache_model[bb_num].end();
                                                  it++) {
    std::vector<bool> cachelines = it->second;
    std::cout << "id: " << it->first << "\n";
    unsigned max_way_usage = 0;
    for (int i = 0; i < 64; i++) {
      unsigned way_usage_count = 0;
      for (int j = 0; j < 8; j++) {
        if (cachelines[j * 64 + i]) {
          std::cout << "access set " << i << " way " << j << "\n";
          way_usage_count++;
        }
      }
      if (max_way_usage < way_usage_count)
        max_way_usage = way_usage_count;
    }
    way_usage_upperbound += max_way_usage;
  }
  std::cout << "BB " << bb_num << " Cache usage : up to " << way_usage_upperbound << " ways contain write-set\n";
}

void X86CacheAnalysis::debug_fun_cache_access() {
  // Aggregate cache usage
  std::map<int, std::vector<bool>> fun_cache;

  for (std::map<int, std::map<int, std::vector<bool>>>::iterator  it = cache_model.begin();
                                                                  it != cache_model.end();
                                                                  it++) {
    std::map<int, std::vector<bool>> bb_cache = it->second;
    for (std::map<int, std::vector<bool>>::iterator bc = bb_cache.begin();
                                                    bc != bb_cache.end();
                                                    bc++) {
      int id = bc->first;
      std::vector<bool> cachelines = bc->second;
      int i;

      if (fun_cache.find(id) == fun_cache.end()) {
        fun_cache[id].reserve(512);
        for (i = 0; i < 512; i++) {
          fun_cache[id].push_back(false);
        }
      }
      for (i = 0; i < 512; i++) {
        if (cachelines[i])
          fun_cache[id][i] = true;
      }
    }
  }

  unsigned way_usage_upperbound = 0;
  for (std::map<int, std::vector<bool>>::iterator fc = fun_cache.begin();
                                                  fc != fun_cache.end();
                                                  fc++) {
    std::vector<bool> cachelines = fc->second;
    std::cout << "id: " << fc->first << "\n";
    unsigned max_way_usage = 0;
    for (int i = 0; i < 64; i++) {
      unsigned way_usage_count = 0;
      for (int j = 0; j < 8; j++) {
        if (cachelines[j * 64 + i]) {
          std::cout << "access set " << i << " way " << j << "\n";
          way_usage_count++;
        }
      }
      if (max_way_usage < way_usage_count)
        max_way_usage = way_usage_count;
    }
    way_usage_upperbound += max_way_usage;
  }

  std::cout << "Function Cache usage : up to " << way_usage_upperbound << " ways contain write-set\n";
}

