#ifndef __CPU_PRED_PERCEPTRON_PRED_HH__
#define __CPU_PRED_PERCEPTRON_PRED_HH__

#include <vector>

#include "base/types.hh"
#include "params/PerceptronBP.hh"
#include "cpu/pred/bpred_unit.hh"

namespace gem5
{

namespace branch_prediction
{

class PerceptronBP : public BPredUnit
{
    public:
        PerceptronBP(const PerceptronBPParams &params);
        
        bool lookup(ThreadID tid, Addr pc, void * &bp_history) override;
        void updateHistories(ThreadID tid, Addr pc, bool uncond, bool taken,
                             Addr target, const StaticInstPtr &inst,
                             void * &bp_history) override;
        void squash(ThreadID tid, void * &bp_history) override;
        void update(ThreadID tid, Addr pc, bool taken,
                    void * &bp_history, bool squashed,
                    const StaticInstPtr & inst, Addr target) override;
        
    private:

        struct BPHistory
        {
            unsigned globalHistoryReg;
            unsigned branchAddr;
            int local;
            std::vector<Addr> path;
            unsigned new_pc;
            bool branchAddrReplaced;
            int last_y;
        };

        std::vector<unsigned> globalHistoryReg;
        unsigned globalHistoryBits;
        unsigned globalHistoryMask;

        unsigned localMask;
        unsigned pcMask;
        unsigned localHistoryBits;

        unsigned history_length;

        unsigned branchAddrBits; // number of lower order PC bits to use

        std::vector<int> branchHistoryTable;
        std::vector<std::vector<Addr>> pathTable;
        std::vector<std::vector<int>> historyWeightTable;
        std::vector<std::vector<int>> addrWeightTable;
};

} // namespace branch_prediction
} // namespace gem5

#endif // __CPU_PRED_PERCEPTRON_HH__