#include "visualization/qt_gui/mainwindow.h"
#include <QApplication>
#include "dependencies/memory_analysis.h"
#include <libkahypar.h>
#include "dependencies/2d_grid/text_map_loader.h"
#include "dependencies/massive_scene_loader/ScenarioLoader2D.h"
#include "test/test_data.h"

auto map_test_config = freeNav::MAPFTestConfig_Berlin_1_256;

auto is_char_occupied1 = [](const char& value) -> bool {
    if (value == '.') { return false; }
    return true;
};

freeNav::TextMapLoader loader(map_test_config.at("map_path"), is_char_occupied1);

// instance loader

auto is_occupied = [](const freeNav::Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };

auto set_occupied = [](const freeNav::Pointi<2> & pt) { loader.setOccupied(pt); };

freeNav::IS_OCCUPIED_FUNC<2> is_occupied_func = is_occupied;

freeNav::SET_OCCUPIED_FUNC<2> set_occupied_func = set_occupied;

struct timezone tz;
struct timeval tv_pre;
struct timeval tv_after;

using namespace std;


int main(int argc, char *argv[])
{
    // start conflict based search
    std::cout << "start GUI" << std::endl;
    const auto& dim = loader.getDimensionInfo();
    freeNav::Instances<2> ists;
    ScenarioLoader2D sl(map_test_config.at("scene_path").c_str());
    int max_count_of_case = atoi((map_test_config.at("agent_num")).c_str());
    int count_of_experiments = sl.GetNumExperiments();


    // load experiment data
    for (int i = 0; i < std::min(max_count_of_case, count_of_experiments); i++) {
        const auto &experiment = sl.GetNthExperiment(i);
        freeNav::Pointi<2> pt1({experiment.GetStartX(), experiment.GetStartY()});
        freeNav::Pointi<2> pt2({experiment.GetGoalX(), experiment.GetGoalY()});
        freeNav::Instance<2> ist = {pt1, pt2};
        //std::cout << " start, target = " << pt1 << ", " << pt2 << std::endl;
        ists.push_back(ist);
    }

    // start GUI for visualization
    Q_INIT_RESOURCE(images);
    QApplication app(argc, argv);
    app.setAttribute(Qt::AA_DontCreateNativeWidgetSiblings);

    MainWindow window(is_occupied, loader.getDimensionInfo(), ists, {});
    window.show();

    return app.exec();
}






























