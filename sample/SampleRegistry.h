//
// Created by xumiz on 2026/1/1.
//

#ifndef TAIYI_SAMPLEREGISTRY_H
#define TAIYI_SAMPLEREGISTRY_H

#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>
#include <stdexcept>
#include "ISample.h"


enum SampleId : int {
    EMPTY_SCENE,
    HANGING_CLOTH,
    FALLING_BUNNY,
    RUBBER_BALL,
    FALLING_CLOTH,
    RIGID_BOX,
};

struct SampleInfo {
    SampleId id;
    const char* display_name;
};

class SampleRegistry {
public:
    using Factory = std::function<SamplePtr()>;

    void Register(const SampleId id, const char* display_name, Factory f) {
        infos_.push_back({id, display_name});
        factories_[id] = std::move(f);
    }

    SamplePtr Create(const SampleId id) const {
        const auto it = factories_.find(id);
        if (it == factories_.end()) throw std::runtime_error("Sample factory not found");
        return (it->second)();
    }

    const std::vector<SampleInfo>& Infos() const { return infos_; }

private:
    std::vector<SampleInfo> infos_;
    std::unordered_map<SampleId, Factory> factories_;
};


void RegisterAllSamples(SampleRegistry& reg);


#endif //TAIYI_SAMPLEREGISTRY_H