#include <algorithm>
#include <filesystem>
#include <string>

#include "gtest/gtest.h"
#include "orc/Orc.h"
#include "orc/util/import_mujoco.h"

namespace {
using namespace orc;
namespace fs = std::filesystem;

std::vector<fs::path> GetModelFiles(const fs::path& directory) {
    std::vector<fs::path> mjbFiles;

    if (!fs::exists(directory) || !fs::is_directory(directory)) {
        return mjbFiles;
    }

    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.is_regular_file()) {
            std::string ext = entry.path().extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);  // normalize
            if (ext == ".mjb") {
                mjbFiles.push_back(entry.path());
            }
        }
    }

    return mjbFiles;
}

/**
 * @brief Test if all binary models in models folder are loaded correctly, i.e.,
 * they're not compiled using a different version of mujoco.
 *
 */
TEST(ModelLoadingTest, CheckIfLoading) {
    const fs::path modelDir = "../models";
    auto mjbFiles = GetModelFiles(modelDir);

    ASSERT_FALSE(mjbFiles.empty()) << "No .mjb files found in 'models/' directory.";

    for (const auto& file : mjbFiles) {
        mjModel* m = mj_loadModel(file.c_str(), NULL);
        ASSERT_FALSE(m == NULL);
        mj_deleteModel(m);
    }
}
}  // namespace
