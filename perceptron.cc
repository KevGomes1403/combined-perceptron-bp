#include "cpu/pred/perceptron.hh"

#include "base/intmath.hh"

namespace gem5
{

namespace branch_prediction
{

PerceptronBP::PerceptronBP(const PerceptronBPParams &params)
    : BPredUnit(params),
    globalHistoryBits(params.globalHistoryBits),
    localHistoryBits(params.localHistoryBits),
    history_length(globalHistoryBits + localHistoryBits),
    branchAddrBits(params.branchAddrBits)
    {
        auto makeMask = [](unsigned bits)->Addr {
            if (bits == 0) return Addr(0);
            if (bits >= sizeof(Addr)*8) return ~Addr(0);
            return (Addr(1) << bits) - 1;
        };
        
        globalHistoryMask = makeMask(globalHistoryBits);
        localMask     = makeMask(localHistoryBits);
        pcMask        = makeMask(branchAddrBits);

        // Initialize global history table
        globalHistoryReg.resize(params.numThreads, 0);

        // Initialize local BHT
        branchHistoryTable.resize(size_t(1) << branchAddrBits, 0);

        // Initialize path table
        pathTable.resize(params.numThreads, std::vector<Addr>(std::max({branchAddrBits, history_length}), Addr(0)));

        // Initialize History Weight Table
        historyWeightTable.resize(Addr(1) << branchAddrBits, std::vector<int>(history_length+1, 0));

        // Initilize Addr Weight Table
        addrWeightTable.resize(Addr(1) << branchAddrBits, std::vector<int>(branchAddrBits+1, 0));

    }

bool PerceptronBP::lookup(ThreadID tid, Addr pc, void * &bp_history) {

    Addr global, local, history_reg;
    bool bit_j;

    const Addr new_pc = pc & pcMask;
    global = globalHistoryReg[tid] & globalHistoryMask;
    local  = branchHistoryTable[new_pc] & localMask;

    int y_hist = historyWeightTable[new_pc][0];

    // Concatenate: global in MSBs, branch in LSBs
    history_reg = (global << localHistoryBits) | local;

    for (int j = 1; j <= history_length; ++j) {
        unsigned k = pathTable[tid][j-1];

        bit_j = (history_reg >> (j-1)) & 1;

        if(bit_j) y_hist += historyWeightTable[k][j];
        else y_hist -= historyWeightTable[k][j];
    }

    int y_addr = addrWeightTable[new_pc][0];

    for (int j = 1; j <= branchAddrBits; ++j) {
        unsigned k = pathTable[tid][j-1];

        bit_j = (new_pc >> (j-1)) & 1;

        if (bit_j) y_addr += addrWeightTable[k][j];
        else y_addr -= addrWeightTable[k][j];
    }

    int y = y_addr + y_hist;

    // Create BPHistory and pass it back to be recorded.
    BPHistory *history = new BPHistory;
    history->globalHistoryReg = globalHistoryReg[tid];
    history->local = local;
    history->path  = pathTable[tid]; 
    history->branchAddrReplaced = false;
    history->new_pc = new_pc;
    history->last_y = y;
    bp_history = (void *) history;

    return y >= 0;
}

void PerceptronBP::update(ThreadID tid, Addr pc, bool taken,
                          void * &bp_history, bool squashed,
                          const StaticInstPtr & inst, Addr target) 
{    
    assert(bp_history);
    BPHistory *history = static_cast<BPHistory *>(bp_history);
    
    const Addr new_pc = pc & pcMask;

    if (squashed || (history->last_y <= 64 && history->last_y >= -64)) {
        if (taken) ++historyWeightTable[new_pc][0];
        else --historyWeightTable[new_pc][0];

        Addr global, local, history_reg;
        bool bit_j;

        global = history->globalHistoryReg & globalHistoryMask; 
        local = history->local;

        // Concatenate: global in MSBs, branch in LSBs
        history_reg = (global << localHistoryBits) | local;

        for (int j = 1; j <= history_length; ++j) {
            unsigned k = history->path[j-1];

            // Access bit i (0 = LSB)
            bit_j = (history_reg >> (j-1)) & 1;

            if (taken == bit_j) ++historyWeightTable[k][j];
            else --historyWeightTable[k][j];
        }

        if (taken) ++addrWeightTable[new_pc][0];
        else --addrWeightTable[new_pc][0];

        for (int j = 1; j <= branchAddrBits; ++j) {
            unsigned k = history->path[j-1];
            if ( taken == (((new_pc >> (j-1)) & 1) != 0) ) ++addrWeightTable[k][j];
            else --addrWeightTable[k][j];
        }

    }

    if (!squashed) {
        delete history;
        bp_history = nullptr;
    }

    return;

}

void PerceptronBP::updateHistories(ThreadID tid, Addr pc, bool uncond, bool taken,
                                   Addr target, const StaticInstPtr &inst,
                                   void * &bp_history) 
{
    const Addr new_pc = pc & pcMask;

    if (uncond) {
        BPHistory *history = new BPHistory;
        history->globalHistoryReg = globalHistoryReg[tid];
        history->branchAddrReplaced = false;
        history->new_pc = new_pc;
        history->local = branchHistoryTable[new_pc] & localMask;
        history->path = pathTable[tid];
        history->last_y = 0;
        bp_history = (void *)history;
    }

    BPHistory *history = static_cast<BPHistory*>(bp_history);

    // Update global history
    globalHistoryReg[tid] = ((globalHistoryReg[tid] << 1) | (taken ? 1 : 0)) & globalHistoryMask;
    
    // Update BHT
    branchHistoryTable[new_pc] = ((branchHistoryTable[new_pc] << 1) | (taken ? 1 : 0)) & localMask;

    // Update path table
    const size_t PT_LEN = std::max<size_t>(branchAddrBits, history_length);
    auto& pt = pathTable[tid];
    if (pt.size() == PT_LEN) {
        history->branchAddr = pt.back();
        history->branchAddrReplaced = true;
        pt.pop_back();
    }
    pt.insert(pt.begin(), new_pc);
}

void PerceptronBP::squash(ThreadID tid, void * &bp_history)
{
    BPHistory *history = static_cast<BPHistory*>(bp_history);
    globalHistoryReg[tid] = history->globalHistoryReg; // Restore global history

    branchHistoryTable[history->new_pc] = history->local;

    if (!pathTable[tid].empty()) pathTable[tid].erase(pathTable[tid].begin());

    if (history->branchAddrReplaced) {
        pathTable[tid].push_back(history->branchAddr);
    }

    delete history;
    bp_history = nullptr;
}

} // namespace branch_prediction
} // namespace gem5
