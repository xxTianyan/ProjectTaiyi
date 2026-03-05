#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "tiny_vbd/builder/Builder.h"
#include "tiny_vbd/solver/Solver.h"

namespace tinyvbd {

void exportObj(const std::filesystem::path& path, const RenderMesh& mesh) {
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("failed to open obj output");
    }

    for (const Vec3& p : mesh.render_positions) {
        out << "v " << p.x() << ' ' << p.y() << ' ' << p.z() << '\n';
    }

    for (std::size_t i = 0; i + 2 < mesh.render_indices.size(); i += 3) {
        out << "f " << mesh.render_indices[i] + 1 << ' '
            << mesh.render_indices[i + 1] + 1 << ' '
            << mesh.render_indices[i + 2] + 1 << '\n';
    }
}

} // namespace tinyvbd

int main() {
    using namespace tinyvbd;

    Builder builder;

    ClothPatchDesc cloth;
    cloth.rows = 30;
    cloth.cols = 30;
    cloth.spacing = 0.05f;
    cloth.origin = Vec3(-0.75f, 1.5f, -0.75f);
    cloth.pin_top_row = true;
    builder.addClothPatch(cloth);

    PlaneColliderDesc plane;
    plane.normal = Vec3(0.0f, 1.0f, 0.0f);
    plane.offset = 0.0f;
    builder.addPlaneCollider(plane);

    const Model model = builder.build();
    tinyvbd::State state = Solver::createState(model);

    constexpr float dt = 1.0f / 60.0f;
    constexpr int substeps = 4;
    constexpr int iterations = 12;

    std::filesystem::create_directories("output");

    for (int frame = 0; frame < 180; ++frame) {
        Solver::step(model, state, dt, substeps, iterations, Vec3(0.0f, -9.8f, 0.0f));

        if (frame % 30 == 0) {
            std::ostringstream name;
            name << "output/frame_" << std::setfill('0') << std::setw(4) << frame << ".obj";
            exportObj(name.str(), state.render_mesh);
        }
    }

    std::cout << "Sim done: vertices=" << state.render_mesh.render_positions.size()
              << ", triangles=" << state.render_mesh.render_indices.size() / 3 << '\n';

    return 0;
}
