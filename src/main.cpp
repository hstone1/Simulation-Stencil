#include <QApplication>
#include "mainwindow.h"
#include <QCommandLineParser>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include "clsettings.h"

using namespace std;

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    MainWindow w;
    srand (static_cast <unsigned> (time(0)));

    QCommandLineParser parse;
    parse.addHelpOption();
    parse.addPositionalArgument("filename", "the filename for a mesh object to simulate.");
    parse.addPositionalArgument("simulation_speed", "relative speed at which to run the simulation '0.1' or ");

    QCommandLineOption render("render", "render the meshes to output file for the creation of an animation", "output_dir");
    parse.addOption(render);
    parse.process(app);

    QStringList pargs = parse.positionalArguments();

    if (pargs.length() < 1) {
        cerr << "ERROR: Requires argument for simulation object filename" << endl;
        exit(1);
    }

    ConfigStore::get().setMeshFilename(pargs[0]);
    if (pargs.length() > 1)
        ConfigStore::get().setSpeedFactor(pargs[1].toFloat());
    else
        ConfigStore::get().setSpeedFactor(1.0f);

    if (parse.isSet(render))
        ConfigStore::get().setRenderingSetting(parse.value(render));
    else
        ConfigStore::get().setNotRendering();

    w.show();
    //w.setWindowState(w.windowState() | Qt::WindowFullScreen); // Comment out this line to have a windowed 800x600 game on startup.

    return app.exec();
}

